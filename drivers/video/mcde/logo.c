/* drivers/video/msm/logo.c
 *
 * Show Logo in RLE 565 format
 *
 * Copyright (C) 2008 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>

#include <linux/irq.h>
#include <asm/system.h>

#define fb_width(fb)	((fb)->var.xres)
#define fb_linewidth(fb) ((fb)->fix.line_length / (fb_depth(fb) == 2 ? 2 : 4))
#define fb_height(fb)	((fb)->var.yres)
#define fb_depth(fb)	((fb)->var.bits_per_pixel >> 3)
#define fb_size(fb)	(fb_width(fb) * fb_height(fb) * fb_depth(fb))
#define INIT_IMAGE_FILE "/logo.rle"

static void memset16(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	count >>= 1;
	while (count--)
		*ptr++ = val;
}

static void memset32(void *_ptr, unsigned int val, unsigned count)
{
	unsigned int *ptr = _ptr;
	count >>= 2;
	while (count--)
		*ptr++ = val;
}

/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
static int load_565rle_image(char *filename)
{
	struct fb_info *info;
	int fd, err = 0;
	unsigned max, width, stride, line_pos = 0;
	unsigned short *data, *ptr;
	unsigned char *bits;
	signed count;

	info = registered_fb[0];
	if (!info) {
		printk(KERN_ERR "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_ERR "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = sys_lseek(fd, (off_t)0, 2);
	if (count <= 0) {
		err = -EIO;
		printk(KERN_ERR "%s: sys_lseek failed %s\n",
			__func__, filename);
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_ERR "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if (sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		printk(KERN_ERR "%s: sys_read failed %s\n",
			__func__, filename);
		goto err_logo_free_data;
	}
	width = fb_width(info);
	stride = fb_linewidth(info);
	max = width * fb_height(info);
	ptr = data;
	bits = (unsigned char *)(info->screen_base);
	while (count > 3) {
		int n = ptr[0];

		if (n > max)
			break;
		max -= n;
		while (n > 0) {
			unsigned int j =
				(line_pos+n > width ? width-line_pos : n);

			if (fb_depth(info) == 2) {
				memset16(bits, ptr[1], j << 1);
			} else {
				/* Should probably add check for framebuffer
				 * format here*/
				unsigned int widepixel = ptr[1];
				widepixel = (widepixel & 0xf800) << (19-11) |
					(widepixel & 0x07e0) << (10-5) |
					(widepixel & 0x001f) << (3-0);
				memset32(bits, widepixel, j << 2);
			}
			bits += j * fb_depth(info);
			line_pos += j;
			n -= j;
			if (line_pos == width) {
				bits += (stride-width) * fb_depth(info);
				line_pos = 0;
			}
		}
		ptr += 2;
		count -= 4;
	}

err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);

	return err;
}

static void draw_logo(void)
{
	struct fb_info *fb_info;
	struct fb_var_screeninfo var;
	int err = 0;

	fb_info = registered_fb[0];
	if (fb_info && fb_info->fbops->fb_open) {
		err = fb_info->fbops->fb_open(fb_info, 0);
		if (err)
			printk(KERN_ERR "%s: fb_open failed.\n", __func__);
		fb_info->fbops->fb_blank(FB_BLANK_UNBLANK, fb_info);
		fb_info->var.activate = FB_ACTIVATE_FORCE|FB_ACTIVATE_NOW;
		memcpy(&var, &fb_info->var, sizeof(struct fb_var_screeninfo));
		var.xoffset = 1;
		var.yoffset = 0;
		err = fb_info->fbops->fb_pan_display(&var, fb_info);
		if (err)
			printk(KERN_ERR "%s: fb_pan_display failed 1.\n",
				__func__);
		var.xoffset = 0;
		err = fb_info->fbops->fb_pan_display(&var, fb_info);
		if (err)
			printk(KERN_ERR "%s: fb_pan_display failed 2.\n",
				__func__);
	}
	printk(KERN_INFO "Drawing splash screen\n");
}

int mcde_logo_init(void)
{
	if (!load_565rle_image(INIT_IMAGE_FILE))
		draw_logo();

	return 0;
}
