/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson MCDE base driver
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stat.h>
#include <linux/time.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <asm/page.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include "mcde_debugfs.h"

#define MAX_NUM_OVERLAYS 2
#define MAX_NUM_CHANNELS 4
#define DEFAULT_DMESG_FPS_LOG_INTERVAL 100

struct fps_info {
	u32 enable_dmesg;
	u32 interval_ms;
	struct timespec timestamp_last;
	u32 frame_counter_last;
	u32 frame_counter;
	u32 fpks;
};

struct overlay_info {
	u8 id;
	struct dentry *dentry;
	struct fps_info fps;
};

struct channel_info {
	u8 id;
	struct dentry *dentry;
	struct mcde_chnl_state *chnl;
	struct fps_info fps;
	struct overlay_info overlays[MAX_NUM_OVERLAYS];
};

static struct mcde_info {
	struct device *dev;
	struct dentry *dentry;
	struct channel_info channels[MAX_NUM_CHANNELS];
} mcde;

static int mcde_ovly_print(struct seq_file *s, void *p)
{
	struct mcde_ovly_state *ovly = s->private;

	dev_info(mcde.dev, "%s: --- START ---\n", __func__);
	mcde_hw_ovly_print(ovly);
	dev_info(mcde.dev, "%s: --- END ---\n", __func__);
	return 0;
}

static int mcde_dump_ovly_open(struct inode *inode, struct file *file)
{
	return single_open(file, mcde_ovly_print, inode->i_private);
}

static const struct file_operations mcde_dump_ovly_fops = {
	.open = mcde_dump_ovly_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int mcde_chnl_print(struct seq_file *s, void *p)
{
	struct mcde_chnl_state *chnl = s->private;

	dev_info(mcde.dev, "%s: --- START ---\n", __func__);
	mcde_hw_chnl_print(chnl);
	dev_info(mcde.dev, "%s: --- END ---\n", __func__);
	return 0;
}

static int mcde_dump_chnl_open(struct inode *inode, struct file *file)
{
	return single_open(file, mcde_chnl_print, inode->i_private);
}

static const struct file_operations mcde_dump_chnl_fops = {
	.open = mcde_dump_chnl_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

/* Requires: lhs > rhs */
static inline u32 timespec_ms_diff(struct timespec lhs, struct timespec rhs)
{
	struct timespec tmp_ts = timespec_sub(lhs, rhs);
	u64 tmp_ns = (u64)timespec_to_ns(&tmp_ts);
	do_div(tmp_ns, NSEC_PER_MSEC);
	return (u32)tmp_ns;
}

/* Returns "frames per 1000 secs", divide by 1000 to get fps with 3 decimals */
static u32 update_fps(struct fps_info *fps)
{
	struct timespec now;
	u32 fpks = 0, ms_since_last, num_frames;

	getrawmonotonic(&now);
	fps->frame_counter++;

	ms_since_last = timespec_ms_diff(now, fps->timestamp_last);
	num_frames = fps->frame_counter - fps->frame_counter_last;
	if (num_frames > 1 && ms_since_last >= fps->interval_ms) {
		fpks = (num_frames * 1000000) / ms_since_last;
		fps->timestamp_last = now;
		fps->frame_counter_last = fps->frame_counter;
		fps->fpks = fpks;
	}

	return fpks;
}

static void update_chnl_fps(struct channel_info *ci)
{
	u32 fpks = update_fps(&ci->fps);
	if (fpks && ci->fps.enable_dmesg)
		dev_info(mcde.dev, "FPS: chnl=%d fps=%d.%.3d\n", ci->id,
						fpks / 1000, fpks % 1000);
}

static void update_ovly_fps(struct channel_info *ci, struct overlay_info *oi)
{
	u32 fpks = update_fps(&oi->fps);
	if (fpks && oi->fps.enable_dmesg)
		dev_info(mcde.dev, "FPS: ovly=%d.%d fps=%d.%.3d\n", ci->id,
					oi->id, fpks / 1000, fpks % 1000);
}

int mcde_debugfs_create(struct device *dev)
{
	if (mcde.dev)
		return -EBUSY;

	mcde.dentry = debugfs_create_dir("mcde", NULL);
	if (!mcde.dentry)
		return -ENOMEM;
	mcde.dev = dev;

	return 0;
}

static struct channel_info *find_chnl(u8 chnl_id)
{
	if (chnl_id > MAX_NUM_CHANNELS)
		return NULL;
	return &mcde.channels[chnl_id];
}

static struct overlay_info *find_ovly(struct channel_info *ci, u8 ovly_id)
{
	if (!ci || ovly_id >= MAX_NUM_OVERLAYS)
		return NULL;
	return &ci->overlays[ovly_id];
}

static void create_fps_files(struct dentry *dentry, struct fps_info *fps)
{
	debugfs_create_u32("frame_counter", S_IRUGO, dentry,
							&fps->frame_counter);
	debugfs_create_u32("frames_per_ksecs", S_IRUGO, dentry, &fps->fpks);
	debugfs_create_u32("interval_ms", S_IRUGO|S_IWUGO, dentry,
							&fps->interval_ms);
	debugfs_create_u32("dmesg", S_IRUGO|S_IWUGO, dentry,
							&fps->enable_dmesg);
}

int mcde_debugfs_channel_create(u8 chnl_id, struct mcde_chnl_state *chnl)
{
	struct channel_info *ci = find_chnl(chnl_id);
	char name[10];

	if (!chnl || !ci)
		return -EINVAL;
	if (ci->chnl)
		return -EBUSY;

	snprintf(name, sizeof(name), "chnl%d", chnl_id);
	ci->dentry = debugfs_create_dir(name, mcde.dentry);
	if (!ci->dentry)
		return -ENOMEM;

	create_fps_files(ci->dentry, &ci->fps);
	debugfs_create_file("dump_chnl", S_IRUGO, ci->dentry,
						chnl, &mcde_dump_chnl_fops);
	ci->fps.interval_ms = DEFAULT_DMESG_FPS_LOG_INTERVAL;
	ci->id = chnl_id;
	ci->chnl = chnl;

	return 0;
}

int mcde_debugfs_overlay_create(u8 chnl_id, u8 ovly_id,
						struct mcde_ovly_state *ovly)
{
	struct channel_info *ci = find_chnl(chnl_id);
	struct overlay_info *oi = find_ovly(ci, ovly_id);
	char name[10];

	if (!oi || !ci || ovly_id >= MAX_NUM_OVERLAYS)
		return -EINVAL;
	if (oi->dentry)
		return -EBUSY;

	snprintf(name, sizeof(name), "ovly%d", ovly_id);
	oi->dentry = debugfs_create_dir(name, ci->dentry);
	if (!oi->dentry)
		return -ENOMEM;

	create_fps_files(oi->dentry, &oi->fps);
	debugfs_create_file("dump_ovly", S_IRUGO, oi->dentry,
						ovly, &mcde_dump_ovly_fops);
	oi->fps.interval_ms = DEFAULT_DMESG_FPS_LOG_INTERVAL;
	oi->id = ovly_id;

	return 0;
}

void mcde_debugfs_channel_update(u8 chnl_id)
{
	struct channel_info *ci = find_chnl(chnl_id);

	if (!ci || !ci->chnl)
		return;

	update_chnl_fps(ci);
}

void mcde_debugfs_overlay_update(u8 chnl_id, u8 ovly_id)
{
	struct channel_info *ci = find_chnl(chnl_id);
	struct overlay_info *oi = find_ovly(ci, ovly_id);

	if (!oi || !oi->dentry)
		return;

	update_ovly_fps(ci, oi);
}

