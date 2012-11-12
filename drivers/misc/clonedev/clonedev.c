/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Device for display cloning on external output.
 *
 * Author: Per-Daniel Olsson <per-daniel.olsson@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <linux/clonedev.h>
#include <linux/compdev.h>
#include <linux/compdev_util.h>
#include <linux/hwmem.h>
#include <linux/mm.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

#include <video/mcde.h>
#include <video/b2r2_blt.h>

#define MAX_SCENE_IMAGES 2

static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_list_lock);

static int dev_counter;

struct clonedev {
	struct mutex lock;
	struct miscdevice mdev;
	struct device *dev;
	char name[10];
	struct list_head list;
	bool open;
	struct compdev *src_compdev;
	struct compdev *dst_compdev;
	bool overlay_case;
	struct compdev_size src_size;
	struct compdev_size dst_size;
	struct compdev_rect crop_rect;
	struct compdev_scene_info s_info;
	struct compdev_img scene_images[MAX_SCENE_IMAGES];
	u8 scene_img_count;
	enum clonedev_mode mode;
	struct buffer_cache_context cache_ctx;
	int blt_handle;
	u8 crop_ratio;
};

static int clonedev_blt(struct clonedev *cd,
		struct compdev_img *src_img,
		struct compdev_img *dst_img,
		bool blend, bool sync)
{
	struct b2r2_blt_req req;
	int req_id;

	dev_dbg(cd->dev, "%s\n", __func__);

	memset(&req, 0, sizeof(req));
	req.size = sizeof(req);

	if (src_img->buf.type == COMPDEV_PTR_PHYSICAL) {
		req.src_img.buf.type = B2R2_BLT_PTR_PHYSICAL;
		req.src_img.buf.fd = src_img->buf.fd;
	} else {
		struct hwmem_alloc *alloc;

		req.src_img.buf.type = B2R2_BLT_PTR_HWMEM_BUF_NAME_OFFSET;
		req.src_img.buf.hwmem_buf_name = src_img->buf.hwmem_buf_name;

		alloc = hwmem_resolve_by_name(src_img->buf.hwmem_buf_name);
		if (IS_ERR_OR_NULL(alloc)) {
			dev_err(cd->dev, "HWMEM resolve src failed\n");
		} else {
			hwmem_set_access(alloc,
				HWMEM_ACCESS_READ | HWMEM_ACCESS_IMPORT,
				task_tgid_nr(current));
			hwmem_release(alloc);
		}
	}
	req.src_img.pitch = src_img->pitch;
	req.src_img.buf.offset = src_img->buf.offset;
	req.src_img.buf.len = src_img->buf.len;
	req.src_img.fmt = compdev_to_blt_format(src_img->fmt);
	req.src_img.width = src_img->width;
	req.src_img.height = src_img->height;

	req.src_rect.x = src_img->src_rect.x;
	req.src_rect.y = src_img->src_rect.y;
	req.src_rect.width = src_img->src_rect.width;
	req.src_rect.height = src_img->src_rect.height;

	if (dst_img->buf.type == COMPDEV_PTR_PHYSICAL) {
		req.dst_img.buf.type = B2R2_BLT_PTR_PHYSICAL;
		req.dst_img.buf.fd = dst_img->buf.fd;
	} else {
		req.dst_img.buf.type = B2R2_BLT_PTR_HWMEM_BUF_NAME_OFFSET;
		req.dst_img.buf.hwmem_buf_name = dst_img->buf.hwmem_buf_name;
	}
	req.dst_img.pitch = dst_img->pitch;
	req.dst_img.buf.offset = dst_img->buf.offset;
	req.dst_img.buf.len = dst_img->buf.len;
	req.dst_img.fmt = compdev_to_blt_format(dst_img->fmt);
	req.dst_img.width = dst_img->width;
	req.dst_img.height = dst_img->height;

	req.transform = compdev_to_blt_transform(src_img->transform);
	req.dst_rect.x = src_img->dst_rect.x;
	req.dst_rect.y = src_img->dst_rect.y;
	req.dst_rect.width = src_img->dst_rect.width;
	req.dst_rect.height = src_img->dst_rect.height;

	req.global_alpha = 0xff;
	req.flags = B2R2_BLT_FLAG_DITHER | B2R2_BLT_FLAG_ASYNCH;

	if (blend)
		req.flags |= B2R2_BLT_FLAG_PER_PIXEL_ALPHA_BLEND;

	req_id = b2r2_blt_request(cd->blt_handle, &req);
	if (req_id < 0)
		dev_err(cd->dev, "%s: Err b2r2_blt_request, handle %d, id %d",
			__func__, cd->blt_handle, req_id);
	else if (sync && b2r2_blt_synch(cd->blt_handle, req_id) < 0)
		dev_err(cd->dev, "%s: Could not perform b2r2_blt_synch",
			__func__);

	return req_id;
}

static void clonedev_best_fit(struct compdev_rect *crop_rect,
		struct compdev_rect *dst_rect,
		enum   compdev_transform  transform)
{
	/* aspect ratio in 26.6 fixed point with remainder */
	uint32_t aq;
	uint32_t ar;
	uint32_t nw;
	uint32_t dw;
	uint32_t nh;
	uint32_t dh;
	uint32_t dst_w;
	uint32_t dst_h;

	if (transform == COMPDEV_TRANSFORM_ROT_90_CCW ||
			transform == COMPDEV_TRANSFORM_ROT_270_CCW) {
		nw = dst_rect->width << 6;
		dw = dst_rect->height;
		nh = dst_rect->height << 6;
		dh = dst_rect->width;
	} else {
		nw = dst_rect->height << 6;
		dw = dst_rect->width;
		nh = dst_rect->width << 6;
		dh = dst_rect->height;
	}
	aq = nw / dw;
	ar = nw % dw;

	/* Base destination rect on crop_rect width */
	dst_w = crop_rect->width;
	dst_h = (aq * crop_rect->width +
			((ar * crop_rect->width) / dw)) >> 6;
	dst_rect->x = 0;
	dst_rect->y = 0;

	/*
	 * Clamp to crop_rect height if the destination rectangle
	 * is too high, and base the destination rectangle width
	 * on the crop_rect height instead.
	 */
	if (dst_h > crop_rect->height) {
		aq = nh / dh;
		ar = nh % dh;
		dst_h = crop_rect->height;
		dst_w = (aq * crop_rect->height +
				((ar * crop_rect->height) / dh)) >> 6;
	}

	dst_rect->width = dst_w;
	dst_rect->height = dst_h;

	/* Center the image if necessary */
	if (dst_w < crop_rect->width)
		dst_rect->x += (crop_rect->width - dst_w) >> 1;

	if (dst_h < crop_rect->height)
		dst_rect->y += (crop_rect->height - dst_h) >> 1;
}

static void clonedev_rescale_destrect(struct compdev_rect *boundary,
		struct compdev_size *src_size,
		struct compdev_rect *dst_rect,
		enum compdev_transform transform)
{
	uint32_t q, r, src_width;
	uint32_t x, y, height, width;

	if (transform == COMPDEV_TRANSFORM_ROT_0) {
		x = dst_rect->x;
		y = dst_rect->y;
		width = dst_rect->width;
		height = dst_rect->height;
		src_width = src_size->width;
	} else if (transform == COMPDEV_TRANSFORM_ROT_90_CW) {
		x = src_size->height - dst_rect->y - dst_rect->height;
		y = dst_rect->x;
		width = dst_rect->height;
		height = dst_rect->width;
		src_width = src_size->height;
	} else if (transform == COMPDEV_TRANSFORM_ROT_90_CCW) {
		x = dst_rect->y;
		y = src_size->width - dst_rect->x - dst_rect->width;
		width = dst_rect->height;
		height = dst_rect->width;
		src_width = src_size->height;
	}

	q = (boundary->width << 6) / src_width;
	r = (boundary->width << 6) % src_width;

	dst_rect->x      = (((boundary->x << 6) + ((q * x + r * x / src_width) +
				(0x1 << 5))) >> 6) & ~0x1;
	dst_rect->y      = ((q * y + r * y / src_width) >> 6) + boundary->y;
	dst_rect->width  = (((q * width + r * width / src_width) +
				(0x1 << 5)) >> 6) & ~0x1;
	dst_rect->height = (q * height + r * height / src_width) >> 6;
}

static int clonedev_set_mode_locked(struct clonedev *cd,
		enum clonedev_mode mode)
{
	cd->mode = mode;
	cd->scene_img_count = 0;
	cd->s_info.img_count = 1;

	return 0;
}

static void clonedev_recalculate_cropping(struct clonedev *cd)
{
	u32 ratio;
	u32 cropped_width;
	u32 cropped_height;

	/* Use 16.16 fix point */
	ratio = ((u32)cd->crop_ratio << 16) / 100;
	cropped_width = (u32)cd->dst_size.width * ratio + (0x1 << 15);
	cropped_height = (u32)cd->dst_size.height * ratio + (0x1 << 15);

	cd->crop_rect.width = (cropped_width >> 16) & ~0x00000001;
	cd->crop_rect.height = (cropped_height >> 16) & ~0x00000001;
	cd->crop_rect.x = (cd->dst_size.width - cd->crop_rect.width) >> 1;
	cd->crop_rect.y = (cd->dst_size.height - cd->crop_rect.height) >> 1;
}

static int clonedev_set_crop_ratio_locked(struct clonedev *cd, u8 crop_ratio)
{
	int ret = 0;

	if (crop_ratio > 100 || crop_ratio < 1) {
		dev_dbg(cd->dev, "%s: Illegal crop ratio (%d)\n",
			__func__, crop_ratio);
		ret = -EINVAL;
	} else {
		cd->crop_ratio = crop_ratio;
		clonedev_recalculate_cropping(cd);
	}

	return ret;
}

static void set_transform_and_dest_rect(struct clonedev *cd,
		struct compdev_img *img)
{
	struct compdev_rect temp_rect = {0};
	temp_rect.width = cd->src_size.width;
	temp_rect.height = cd->src_size.height;

	/* First adjust src rect to crop_rect */
	clonedev_best_fit(&cd->crop_rect,
			&temp_rect,
			img->transform);

	/* Now use temp_rect as the boundary */
	clonedev_rescale_destrect(&temp_rect,
			&cd->src_size,
			&img->dst_rect,
			img->transform);

	if (cd->overlay_case)
		img->transform = cd->s_info.ovly_transform;
	else
		img->transform = cd->s_info.fb_transform;

	/* Invert the rotation to adapt to TV */
	if (img->transform == COMPDEV_TRANSFORM_ROT_90_CCW)
		img->transform = COMPDEV_TRANSFORM_ROT_270_CCW;
	else if (img->transform == COMPDEV_TRANSFORM_ROT_270_CCW)
		img->transform = COMPDEV_TRANSFORM_ROT_90_CCW;
}

static void get_bounding_rect(struct compdev_rect *rect1,
		struct compdev_rect *rect2, struct compdev_rect *bounds)
{
	bounds->x = min(rect1->x, rect2->x);
	bounds->y = min(rect1->y, rect2->y);
	bounds->width = max(rect1->x + rect1->width, rect2->x + rect2->width)
			- bounds->x;
	bounds->height = max(rect1->y + rect1->height, rect2->y + rect2->height)
			- bounds->y;
}

static enum compdev_fmt clonedev_compatible_fmt(enum compdev_fmt fmt)
{
	switch (fmt) {
	case COMPDEV_FMT_RGB565:
	case COMPDEV_FMT_RGB888:
	case COMPDEV_FMT_RGBA8888:
	case COMPDEV_FMT_RGBX8888:
		return fmt;
	case COMPDEV_FMT_YUV422:
	case COMPDEV_FMT_YCBCR42XMBN:
	case COMPDEV_FMT_YUV420_SP:
	case COMPDEV_FMT_YVU420_SP:
	case COMPDEV_FMT_YUV420_P:
	case COMPDEV_FMT_YVU420_P:
	case COMPDEV_FMT_YV12:
		return COMPDEV_FMT_RGB888;
	default:
		return COMPDEV_FMT_RGBA8888;
	}
}

static u32 get_b2r2_color_black(enum b2r2_blt_fmt fmt)
{
	switch (fmt) {
	case B2R2_BLT_FMT_CB_Y_CR_Y:
		return 0x80108010;
	case B2R2_BLT_FMT_32_BIT_ARGB8888:
		return 0xFF000000;
	case B2R2_BLT_FMT_24_BIT_RGB888:
	case B2R2_BLT_FMT_16_BIT_RGB565:
	default:
		return 0;
	}
}


static inline void clear_rect(struct clonedev *cd,
		struct b2r2_blt_req *bltreq, int x, int y,
		int width, int height)
{
	int req_id;

	bltreq->dst_rect.x = x;
	bltreq->dst_rect.y = y;
	bltreq->dst_rect.width = width;
	bltreq->dst_rect.height = height;
	if (width != 0 && height != 0) {
		req_id = b2r2_blt_request(cd->blt_handle, bltreq);
		if (req_id < 0)
			dev_err(cd->dev, "%s: Err b2r2_blt_request, id %d",
					__func__, req_id);
	}
}

static void clonedev_clear_background(struct clonedev *cd,
			struct compdev_img *dst_img,
			struct compdev_rect *img_rect)
{
	/* Clear the dst_img outside of the rect specified by img_rect */
	struct b2r2_blt_req *bltreq;

	bltreq = kzalloc(sizeof(*bltreq), GFP_KERNEL);
	if (bltreq == NULL)
		return;

	bltreq->size = sizeof(struct b2r2_blt_req);
	bltreq->flags = B2R2_BLT_FLAG_ASYNCH | B2R2_BLT_FLAG_SOURCE_FILL_RAW;
	bltreq->transform = B2R2_BLT_TRANSFORM_NONE;
	bltreq->dst_img.buf.type = B2R2_BLT_PTR_HWMEM_BUF_NAME_OFFSET;
	bltreq->dst_img.buf.hwmem_buf_name = dst_img->buf.hwmem_buf_name;
	bltreq->dst_img.width = dst_img->width;
	bltreq->dst_img.height = dst_img->height;
	bltreq->dst_img.fmt = compdev_to_blt_format(dst_img->fmt);
	bltreq->dst_img.pitch = dst_img->pitch;
	bltreq->src_color = get_b2r2_color_black(bltreq->dst_img.fmt);

	clear_rect(cd, bltreq,
			0, 0, img_rect->x, dst_img->height);
	clear_rect(cd, bltreq,	img_rect->x, 0,
			img_rect->width, img_rect->y);
	clear_rect(cd, bltreq,	img_rect->x + img_rect->width,
			0, dst_img->width - img_rect->width - img_rect->x,
			dst_img->height);
	clear_rect(cd, bltreq,	img_rect->x,
			img_rect->y + img_rect->height,	img_rect->width,
			dst_img->height - img_rect->y - img_rect->height);

	kfree(bltreq);
}


static void clonedev_compose_locked(struct clonedev *cd)
{
	struct compdev_img *img0;
	struct compdev_img *img1 = NULL;
	bool protected = false;
	struct compdev_img_internal *dst_img;
	int b2r2_req_id;
	enum compdev_fmt dst_fmt;

	/* Now there should be two images */
	if (cd->scene_img_count == 0) {
		dev_err(cd->dev, "%s: There should be two images at this "
				"point\n", __func__);
		return;
	}

	img0 = &cd->scene_images[0];
	if (cd->scene_img_count >= 2)
		img1 = &cd->scene_images[1];

	/* Adjust to output size */
	set_transform_and_dest_rect(cd, img0);
	if (img1)
		set_transform_and_dest_rect(cd, img1);

	/* Organize the images according to z-order, img1 on top of img0 */
	if (img1 != NULL && img0->z_position < img1->z_position) {
		struct compdev_img *temp;
		temp = img0;
		img0 = img1;
		img1 = temp;
	}

	if (img0->flags & COMPDEV_PROTECTED_FLAG ||
			(img1 != NULL && img1->flags & COMPDEV_PROTECTED_FLAG))
		protected = true;

	/*
	 * NOTE: Should be hard coded to RGB888 but somehow
	 * the mcde driver can not handle the offset into
	 * the destination for 24-bit source which happends
	 * in the boot app. So 24-bit will have to do for the
	 * video case.
	 */
	dst_fmt = clonedev_compatible_fmt(img0->fmt);

	dst_img = compdev_buffer_cache_get_image(&cd->cache_ctx, dst_fmt,
			cd->crop_rect.width, cd->crop_rect.height, protected);

	if (dst_img != NULL) {
		if (cd->blt_handle < 0) {
			dev_dbg(cd->dev, "%s: B2R2 opened\n", __func__);
			cd->blt_handle = b2r2_blt_open();
			if (cd->blt_handle < 0)
				dev_err(cd->dev, "%s(%d): Failed to "
					"open b2r2 device\n",
					__func__, __LINE__);
		}

		/* Set destination image parameters */
		if (img1 != NULL)
			get_bounding_rect(&img0->dst_rect,
				&img1->dst_rect,
				&dst_img->img.src_rect);
		else
			dst_img->img.src_rect = img0->dst_rect;
		dst_img->img.dst_rect = dst_img->img.src_rect;
		dst_img->img.z_position = 1;
		dst_img->img.flags |= img0->flags;
		dst_img->img.transform = COMPDEV_TRANSFORM_ROT_0;


		/* Clear destination buf outside of img0 */
		clonedev_clear_background(cd, &dst_img->img,
			&img0->dst_rect);

		/* Handle the blit jobs */
		if (img1 == NULL) {
			b2r2_req_id = clonedev_blt(cd, img0, &dst_img->img,
						false, false);
		} else {
			int ret;
			b2r2_req_id = clonedev_blt(cd, img0, &dst_img->img,
						false, false);
			ret = clonedev_blt(cd, img1, &dst_img->img,
						true, false);
			if (ret >= 0)
				b2r2_req_id = ret;
		}

		dst_img->img.dst_rect = cd->crop_rect;
		dst_img->img.src_rect.x = 0;
		dst_img->img.src_rect.y = 0;
		dst_img->img.src_rect.width = cd->crop_rect.width;
		dst_img->img.src_rect.height = cd->crop_rect.height;

		compdev_post_single_buffer_asynch(cd->dst_compdev,
				&dst_img->img, cd->blt_handle, b2r2_req_id);

		compdev_free_img(&cd->cache_ctx, dst_img);
	} else {
		dev_err(cd->dev, "%s: Could not allocate hwmem "
				"temporary buffer\n", __func__);
	}
}

static void clonedev_post_buffer_callback(void *data,
		struct compdev_img *cb_img)
{
	struct clonedev *cd = (struct clonedev *)data;

	mutex_lock(&cd->lock);

	switch (cd->mode) {
	case CLONEDEV_CLONE_VIDEO_OR_UI:
		if (!cd->overlay_case || (cd->overlay_case &&
				(cb_img->flags & COMPDEV_OVERLAY_FLAG))) {
			cd->scene_images[cd->scene_img_count] = *cb_img;
			cd->scene_img_count++;
			clonedev_compose_locked(cd);
			cd->scene_img_count = 0;
			cd->s_info.img_count = 1;
			cd->s_info.reuse_fb_img = 0;
		}
		break;
	case CLONEDEV_CLONE_VIDEO_AND_UI:
		cd->scene_images[cd->scene_img_count] = *cb_img;
		cd->scene_img_count++;
		if (cd->scene_img_count >= cd->s_info.img_count) {
			clonedev_compose_locked(cd);
			cd->scene_img_count = 0;
			cd->s_info.img_count = 1;
			cd->s_info.reuse_fb_img = 0;
		}
		break;
	case CLONEDEV_CLONE_VIDEO:
	case CLONEDEV_CLONE_UI:
	case CLONEDEV_CLONE_NONE:
	default:
		break;
	}

	mutex_unlock(&cd->lock);
}

static void clonedev_post_scene_info_callback(void *data,
		struct compdev_scene_info *s_info)
{
	struct clonedev *cd = (struct clonedev *)data;

	mutex_lock(&cd->lock);
	switch (cd->mode) {
	case CLONEDEV_CLONE_VIDEO_OR_UI:
		if (s_info->reuse_fb_img) {
			cd->overlay_case = true;
		} else {
			if (s_info->img_count > 1)
				cd->overlay_case = true;
			else
				cd->overlay_case = false;
		}

		cd->s_info = *s_info;
		cd->s_info.img_count = 1;
		compdev_post_scene_info(cd->dst_compdev, &cd->s_info);
		break;
	case CLONEDEV_CLONE_VIDEO_AND_UI:
	{
		struct compdev_scene_info scene_info = *s_info;
		if (s_info->reuse_fb_img) {
			scene_info.img_count = 2;
			cd->overlay_case = true;
		} else {
			if (s_info->img_count > 1)
				cd->overlay_case = true;
			else
				cd->overlay_case = false;
		}

		cd->scene_img_count = 0;
		cd->s_info = scene_info;
		scene_info.img_count = 1;
		compdev_post_scene_info(cd->dst_compdev, &scene_info);
	}
		break;
	case CLONEDEV_CLONE_VIDEO:
		/* TODO: Implement */
		break;
	case CLONEDEV_CLONE_UI:
		/* TODO: Implement */
		break;
	case CLONEDEV_CLONE_NONE:
		/* Do nothing */
		break;
	default:
		break;
	}
	mutex_unlock(&cd->lock);
}

static void clonedev_dest_size_changed_callback(void *data,
		struct compdev_size *size)
{
	struct clonedev *cd = (void *)data;

	mutex_lock(&cd->lock);
	cd->dst_size = *size;
	clonedev_recalculate_cropping(cd);
	mutex_unlock(&cd->lock);
}

static int clonedev_open(struct inode *inode, struct file *file)
{
	struct clonedev *cd = NULL;

	mutex_lock(&dev_list_lock);
	list_for_each_entry(cd, &dev_list, list)
		if (cd->mdev.minor == iminor(inode))
			break;

	if (&cd->list == &dev_list) {
		mutex_unlock(&dev_list_lock);
		return -ENODEV;
	}

	if (cd->open) {
		mutex_unlock(&dev_list_lock);
		return -EBUSY;
	}

	cd->open = true;

	mutex_unlock(&dev_list_lock);

	file->private_data = cd;

	return 0;
}

static int clonedev_release(struct inode *inode, struct file *file)
{
	struct clonedev *cd = NULL;

	mutex_lock(&dev_list_lock);
	list_for_each_entry(cd, &dev_list, list)
		if (cd->mdev.minor == iminor(inode))
			break;

	if (&cd->list == &dev_list) {
		mutex_unlock(&dev_list_lock);
		return -ENODEV;
	}

	cd->open = false;

	mutex_unlock(&dev_list_lock);

	return 0;
}

static long clonedev_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret;
	enum clonedev_mode mode;
	u8 crop_ratio;
	struct clonedev *cd = (struct clonedev *)file->private_data;

	mutex_lock(&cd->lock);

	switch (cmd) {
	case CLONEDEV_SET_MODE_IOC:
		mode = (enum clonedev_mode)arg;
		ret = clonedev_set_mode_locked(cd, mode);
		break;
	case CLONEDEV_SET_CROP_RATIO_IOC:
		crop_ratio = (u8)arg;
		ret = clonedev_set_crop_ratio_locked(cd, crop_ratio);
		break;
	default:
		ret = -ENOSYS;
	}

	mutex_unlock(&cd->lock);

	return ret;
}

static const struct file_operations clonedev_fops = {
	.open = clonedev_open,
	.release = clonedev_release,
	.unlocked_ioctl = clonedev_ioctl,
};

static int init_clonedev(struct clonedev *cd)
{
#ifdef CONFIG_COMPDEV_JANITOR
	char wq_name[20];
#endif
	mutex_init(&cd->lock);
	INIT_LIST_HEAD(&cd->list);
	cd->blt_handle = -1;
	memset(&cd->cache_ctx, 0, sizeof(cd->cache_ctx));

	cd->mdev.minor = MISC_DYNAMIC_MINOR;
	cd->mdev.name = cd->name;
	cd->mdev.fops = &clonedev_fops;

#ifdef CONFIG_COMPDEV_JANITOR
	snprintf(wq_name, sizeof(wq_name), "%s_janitor", cd->name);

	mutex_init(&cd->cache_ctx.janitor_lock);
	cd->cache_ctx.janitor_thread = create_workqueue(wq_name);
	if (!cd->cache_ctx.janitor_thread) {
		mutex_destroy(&cd->cache_ctx.janitor_lock);
		return -ENOMEM;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&cd->cache_ctx.free_buffers_work,
		compdev_free_cache_context_buffers);
#endif

	return 0;
}

int clonedev_create(void)
{
	int ret = 0;
	struct clonedev *cd;

	cd = kzalloc(sizeof(struct clonedev), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	mutex_lock(&dev_list_lock);

	snprintf(cd->name, sizeof(cd->name), "%s%d", CLONEDEV_DEFAULT_DEVICE_PREFIX,
			dev_counter++);
	if (init_clonedev(cd) < 0) {
		kfree(cd);
		dev_counter--;
		mutex_unlock(&dev_list_lock);
		return -ENOMEM;
	}

	mutex_lock(&cd->lock);

	ret = compdev_get(0, &cd->src_compdev);
	if (ret < 0)
		goto fail_register_misc;

	ret = compdev_get(1, &cd->dst_compdev);
	if (ret < 0)
		goto fail_register_misc;

	ret = compdev_get_size(cd->src_compdev, &cd->src_size);
	if (ret < 0)
		goto fail_register_misc;

	ret = compdev_get_size(cd->dst_compdev, &cd->dst_size);
	if (ret < 0)
		goto fail_register_misc;

	cd->crop_rect.x = 0;
	cd->crop_rect.y = 0;
	cd->crop_rect.width = cd->dst_size.width;
	cd->crop_rect.height = cd->dst_size.height;
	cd->crop_ratio = 100;

	ret = compdev_register_listener_callbacks(cd->src_compdev, (void *)cd,
			&clonedev_post_buffer_callback,
			&clonedev_post_scene_info_callback,
			NULL);
	if (ret < 0)
		goto fail_register_misc;

	ret = compdev_register_listener_callbacks(cd->dst_compdev, (void *)cd,
			NULL,
			NULL,
			&clonedev_dest_size_changed_callback);
	if (ret < 0)
		goto fail_register_misc;

	/* Default setting */
	cd->mode = CLONEDEV_CLONE_NONE;

	ret = misc_register(&cd->mdev);
	if (ret)
		goto fail_register_misc;

	list_add_tail(&cd->list, &dev_list);

	cd->dev = cd->mdev.this_device;
	cd->cache_ctx.dev = cd->dev;

	mutex_unlock(&cd->lock);
	mutex_unlock(&dev_list_lock);

	return ret;

fail_register_misc:
	if (cd->src_compdev != NULL)
		compdev_put(cd->src_compdev);
	if (cd->dst_compdev != NULL)
		compdev_put(cd->dst_compdev);
#ifdef CONFIG_COMPDEV_JANITOR
	mutex_destroy(&cd->cache_ctx.janitor_lock);
	destroy_workqueue(cd->cache_ctx.janitor_thread);
#endif
	mutex_unlock(&cd->lock);
	kfree(cd);
	dev_counter--;
	mutex_unlock(&dev_list_lock);

	return ret;
}

void clonedev_destroy(void)
{
	struct clonedev *cd;
	struct clonedev *tmp;
	int i;

	mutex_lock(&dev_list_lock);
	list_for_each_entry_safe(cd, tmp, &dev_list, list) {
		compdev_deregister_callbacks(cd->src_compdev);
		compdev_put(cd->src_compdev);
		compdev_put(cd->dst_compdev);
		list_del(&cd->list);
		misc_deregister(&cd->mdev);

#ifdef CONFIG_COMPDEV_JANITOR
		cancel_delayed_work_sync(&cd->cache_ctx.free_buffers_work);
		flush_workqueue(cd->cache_ctx.janitor_thread);
#endif

		for (i = 0; i < BUFFER_CACHE_DEPTH; i++) {
			if (cd->cache_ctx.img[i] != NULL) {
				kref_put(&cd->cache_ctx.img[i]->ref_count,
					compdev_image_release);
				cd->cache_ctx.img[i] = NULL;
			}
		}

#ifdef CONFIG_COMPDEV_JANITOR
		mutex_destroy(&cd->cache_ctx.janitor_lock);
		destroy_workqueue(cd->cache_ctx.janitor_thread);
#endif

		if (cd->blt_handle >= 0)
			b2r2_blt_close(cd->blt_handle);

		kfree(cd);
		break;
	}
	dev_counter--;
	mutex_unlock(&dev_list_lock);
}

static void clonedev_destroy_all(void)
{
	struct clonedev *cd;
	struct clonedev *tmp;

	mutex_lock(&dev_list_lock);
	list_for_each_entry_safe(cd, tmp, &dev_list, list) {
		list_del(&cd->list);
		misc_deregister(&cd->mdev);
		kfree(cd);
	}
	mutex_unlock(&dev_list_lock);

	mutex_destroy(&dev_list_lock);
}

static int __init clonedev_init(void)
{
	pr_info("%s\n", __func__);

	mutex_init(&dev_list_lock);

	return 0;
}
module_init(clonedev_init);

static void __exit clonedev_exit(void)
{
	clonedev_destroy_all();
	pr_info("%s\n", __func__);
}
module_exit(clonedev_exit);

MODULE_AUTHOR("Per-Daniel Olsson <per-daniel.olsson@stericsson.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Device for display cloning on external output");

