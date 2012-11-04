/* drivers/video/mcde/display-panel_dsi_debugfs.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Sony Ericsson DSI display driver debug fs
 *
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/fb.h>

#include <video/mcde_dss.h>
#include <video/mcde_display.h>
#include <video/mcde_display-panel_dsi.h>

static char *res_buf;
static int buf_sz;

#define TMP_BUF_SZ 128
#define NBR_MATRIX_PARAMETERS 12

enum dbg_cmd_type {
	DCS,
	GEN,
	READ_CMD,
};

enum dbg_matrix_type {
	RGB,
	YUV,
};

static void update_res_buf(char *string)
{
	res_buf = krealloc(res_buf, buf_sz + strlen(string) + 1, GFP_KERNEL);
	if (!res_buf) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		return;
	}

	memcpy(res_buf + buf_sz, string, strlen(string) + 1);
	buf_sz += strlen(string); /* Exclude the NULL termination */
}

static void reset_res_buf(void)
{
	kfree(res_buf);
	res_buf = NULL;
	buf_sz = 0;
}

static void print_cmds2file(const struct panel_reg *preg, struct seq_file *s)
{
	int n, i;

	if (!preg) {
		seq_printf(s, "---------\n");
		goto exit;
	}
	for (n = 0; ; ++n)
		switch (preg[n].type) {
		case CMD_END:
			if ((preg[n].addr == 0) &&
					(preg[n].value[0] == 0)) {
				seq_printf(s, "---------\n");
				goto exit;
			} else {
				seq_printf(s, "CMD_END: invalid, addr:0x%.2x "
					"val:0x%.2x\n", preg[n].addr,
					preg[n].value[0]);
			}
			break;
		case CMD_WAIT_MS:
			if (preg[n].addr == 0)
				seq_printf(s, "CMD_WAIT_MS: %d ms\n",
						preg[n].value[0]);
			else
				seq_printf(s, "CMD_WAIT_MS: invalid addr:0x%.2x"
					" val:0x%.2x\n", preg[n].addr,
					preg[n].value[0]);
			break;
		case CMD_GEN:
			seq_printf(s, "CMD_GEN: addr: 0x%.2x val:",
								preg[n].addr);
			for (i = 0; i < preg[n].len; i++)
				seq_printf(s, " 0x%.2x", preg[n].value[i]);
			seq_printf(s, "\n");
			break;
		case CMD_DCS:
			seq_printf(s, "CMD_DCS: addr: 0x%.2x val:",
								preg[n].addr);
			for (i = 0; i < preg[n].len; i++)
				seq_printf(s, " 0x%.2x", preg[n].value[i]);
			seq_printf(s, "\n");
			break;
		case CMD_RESET:
			seq_printf(s, "CMD_RESET: lvl: %d\n",
						preg[n].value[0]);
			break;
		case CMD_PLATFORM:
			seq_printf(s, "CMD_PLATFORM: val: %d\n",
						preg[n].value[0]);
			break;
		default:
			seq_printf(s,
				"UNKNOWN CMD: 0x%.2x addr: 0x%.2x val:0x%.2x\n",
			preg[n].type, preg[n].addr, preg[n].value[0]);
		}
exit:
	return;
}

static int info_show(struct seq_file *s, void *unused)
{
	struct mcde_display_device *ddev = s->private;
	struct panel_record *rd;
	struct device *dev;

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		return 0;
	}
	dev = &ddev->dev;
	rd = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (rd->panel->name) {
		seq_printf(s, "Panel: %s\n", rd->panel->name);
		seq_printf(s, "xres = %d, yres = %d\n", rd->panel->x_res,
							rd->panel->y_res);
		seq_printf(s, "width = %d mm, height = %d mm\n",
					rd->panel->width, rd->panel->height);
		seq_printf(s, "disable_ulpm = %d\n", rd->panel->disable_ulpm);
		seq_printf(s, "Use deep standby as power off = %d\n",
					ddev->deep_standby_as_power_off);
		seq_printf(s, "---------\n");
		seq_printf(s, "off_to_standby commands:\n");
		print_cmds2file(rd->panel->pinfo->off_to_standby, s);
		seq_printf(s, "deep_standby_to_standby commands:\n");
		print_cmds2file(rd->panel->pinfo->deep_standby_to_standby, s);
		seq_printf(s, "standby_to_off commands:\n");
		print_cmds2file(rd->panel->pinfo->standby_to_off, s);
		seq_printf(s, "standby_to_deep_standby commands:\n");
		print_cmds2file(rd->panel->pinfo->standby_to_deep_standby, s);
		seq_printf(s, "on_to_standby commands:\n");
		print_cmds2file(rd->panel->pinfo->on_to_standby, s);
		seq_printf(s, "standby_to_on commands:\n");
		print_cmds2file(rd->panel->pinfo->standby_to_on, s);
		seq_printf(s, "standby_to_intermediate commands:\n");
		print_cmds2file(rd->panel->pinfo->standby_to_intermediate, s);
		seq_printf(s, "intermediate_to_on commands:\n");
		print_cmds2file(rd->panel->pinfo->intermediate_to_on, s);
	} else {
		seq_printf(s, "No panel name\n");
	}
	return 0;
}

static void print_params(enum dbg_cmd_type cmd, u8 reg, int len, u8 *data)
{
	int i = 0;
	char tmp[TMP_BUF_SZ];

	if (cmd == DCS)
		update_res_buf("DCS WRITE\n");
	else if (cmd == GEN)
		update_res_buf("GENERIC WRITE\n");
	else if (cmd == READ_CMD)
		update_res_buf("READ\n");

	if (len > 0) {
		snprintf(tmp, sizeof(tmp), "reg=0x%.2X\n", reg);
		update_res_buf(tmp);
		snprintf(tmp, sizeof(tmp), "len=%d\n", len);
		update_res_buf(tmp);
		for (i = 0; i < len; i++) {
			snprintf(tmp, sizeof(tmp), "data[%d]=0x%.2X\n", i,
								data[i]);
			update_res_buf(tmp);
		}
	} else {
		update_res_buf("Something went wrong, length is zero.\n");
		snprintf(tmp, sizeof(tmp),
				"reg=0x%.2X, len=%d, data[0]=0x%.2X\n",
				reg, len, data[0]);
		update_res_buf(tmp);
	}
}

static ssize_t reg_read(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mcde_display_device *ddev = s->private;
	struct device *dev;
	u8 reg;
	int len;
	u32 tmp_data;
	u8 data[MCDE_MAX_DSI_DIRECT_CMD_WRITE];
	char *buf;
	int ret;
	struct mcde_chnl_state *chnl;
	char tmp[TMP_BUF_SZ];

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		ret = -ENODEV;
		goto exit;
	}
	chnl = ddev->chnl_state;
	dev = &ddev->dev;

	dev_dbg(dev, "%s\n", __func__);

	reset_res_buf();

	buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!buf) {
		dev_err(dev, "%s: Failed to allocate buffer\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	dev_dbg(dev, "%s: buf = %p, ubuf = %p, count = %d, "
				"sizeof(char) * count = %d, line = %d\n",
		__func__, buf, ubuf, count, sizeof(char) * count, __LINE__);

	if (copy_from_user(buf, ubuf, count)) {
		ret = -EFAULT;
		goto fail_free_mem;
	}

	if (sscanf(buf, "%4hhx %1u", &reg, &len) != 2) {
		update_res_buf("Read - parameter error\n");
		ret = -EINVAL;
		goto fail_free_mem;
	}

	dev_dbg(dev, "reg=%hx, len=%d\n", reg, len);

	/* length restriction in mcde read function */
	if (len > MCDE_MAX_DCS_READ)
		len = MCDE_MAX_DCS_READ;

	memset(data, 0, sizeof(data));

	ret = mcde_dsi_dcs_read(chnl, reg, &tmp_data, &len);
	if (!ret) {
		memcpy(data, &tmp_data, len);
		print_params(READ_CMD, reg, len, data);
	} else {
		snprintf(tmp, sizeof(tmp), "Read failed, ret = %d!\n", ret);
		update_res_buf(tmp);
	}

fail_free_mem:
	kfree(buf);
exit:
	return count;
}

static ssize_t reg_write(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mcde_display_device *ddev = s->private;
	struct device *dev;
	struct mcde_chnl_state *chnl;
	char *buf;
	const char *p;
	enum dbg_cmd_type cmd;
	u8 data[MCDE_MAX_DSI_DIRECT_CMD_WRITE + 1]; /* Make room for cmd  */
	int i = 0;
	int ret;
	char tmp[TMP_BUF_SZ];

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		ret = -ENODEV;
		goto exit;
	}
	chnl = ddev->chnl_state;
	dev = &ddev->dev;

	dev_dbg(dev, "%s\n", __func__);

	reset_res_buf();

	buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!buf) {
		dev_err(dev, "%s: Failed to allocate buffer\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	dev_dbg(dev, "%s: buf = %p, ubuf = %p, count = %d, "
				"sizeof(char) * count = %d, line = %d\n",
		__func__, buf, ubuf, count, sizeof(char) * count, __LINE__);

	if (copy_from_user(buf, ubuf, count)) {
		ret = -EFAULT;
		goto fail_free_mem;
	}

	p = buf;

	if (!strncmp(buf, "dcs", 3)) {
		dev_dbg(dev, "%s: dcs\n", __func__);
		cmd = DCS;
	} else if (!strncmp(buf, "gen", 3)) {
		dev_dbg(dev, "%s: gen\n", __func__);
		cmd = GEN;
	} else {
		update_res_buf("Write - unknown type\n");
		ret = -EFAULT;
		goto fail_free_mem;
	}

	p = p+4;

	/* Get first param, Register */
	if (sscanf(p, "%4hhx", &data[0]) != 1) {
		update_res_buf("Write - parameter error\n");
		ret = -EINVAL;
		goto fail_free_mem;
	}
	i++;

	while (isxdigit(*p) || (*p == 'x'))
		p++;

	/* Get data */
	while (true) {
		if (isspace(*p)) {
			p++;
		} else {
			if (sscanf(p, "%4hhx", &data[i]) == 1) {
				while (isxdigit(*p) || (*p == 'x'))
					p++;
			}
			i++;
		}
		if (iscntrl(*p))
			break;

		if (i > MCDE_MAX_DSI_DIRECT_CMD_WRITE) {
			update_res_buf("Write - Too many parameters\n");
			ret = -EINVAL;
			goto fail_free_mem;
		}
	}

	if (cmd == DCS) {
		if (i == 1)
			ret = mcde_dsi_dcs_write(chnl, data[0], NULL, 0);
		else
			ret = mcde_dsi_dcs_write(chnl, data[0], &data[1],
									i - 1);
	} else {
		ret = mcde_dsi_generic_write(chnl, data, i);
	}

	if (!ret) {
		print_params(cmd, data[0], i - 1, &data[1]);
	} else {
		snprintf(tmp, sizeof(tmp), "Write failed, ret = %d!\n", ret);
		update_res_buf(tmp);
	}

fail_free_mem:
	kfree(buf);
exit:
	return count;
}

static int panels_show(struct seq_file *s, void *unused)
{
	struct mcde_display_device *ddev = s->private;
	struct panel_device *pdev = container_of(ddev, struct panel_device,
									base);
	struct device *dev;
	struct panel_platform_data *pdata;
	int n = 0;

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		return 0;
	}
	dev = &ddev->dev;

	pdata = pdev->base.dev.platform_data;

	dev_dbg(dev, "%s\n", __func__);

	seq_printf(s, "Supported display panels:\n");
	while (pdata->panels[n] != NULL) {
		seq_printf(s, "MCDE panel[%d]=%s\n", n, pdata->panels[n]->name);
		n++;
	}
	return 0;
}

static int result_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s", res_buf);
	if (!res_buf)
		seq_printf(s, "\n");
	return 0;
}

static int reset(struct seq_file *s, void *unused)
{
	struct mcde_display_device *ddev = s->private;
	struct device *dev;
	int ret;

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		goto exit;
	}
	dev = &ddev->dev;

	if (ddev->platform_reset) {
		ret = ddev->platform_reset(ddev, 1);
		if (ret)
			goto exit;
		usleep_range(11000, 20000);
		ret = ddev->platform_reset(ddev, 0);
		if (ret)
			goto exit;
		usleep_range(21, 1000);
		ret = ddev->platform_reset(ddev, 1);
		if (ret)
			goto exit;
		msleep(21);
		dev_info(dev, "%s: Display reset performed.\n", __func__);
	}
exit:
	return 0;
}

static int restart(struct seq_file *s, void *unused)
{
	struct mcde_display_device *ddev = s->private;
	struct panel_device *pdev = container_of(ddev, struct panel_device,
									base);
	struct panel_record *rd;
	struct device *dev;
	int ret;

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		goto exit;
	}
	dev = &ddev->dev;
	rd = dev_get_drvdata(dev);

	if (rd->panel->pinfo->standby_to_intermediate) {
		ret = panel_execute_cmd_extern(pdev,
				rd->panel->pinfo->standby_to_intermediate);
		if (ret)
			goto exit;
		if (rd->panel->pinfo->intermediate_to_on) {
			ret = panel_execute_cmd_extern(pdev,
					rd->panel->pinfo->intermediate_to_on);
			if (ret)
				goto exit;
		}
	} else if (rd->panel->pinfo->standby_to_on) {
		ret = panel_execute_cmd_extern(pdev,
				rd->panel->pinfo->standby_to_on);
		if (ret)
			goto exit;
	}
	dev_info(dev, "%s: Display restart performed.\n", __func__);
exit:
	return 0;
}

static ssize_t write_matrix(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mcde_display_device *ddev = s->private;
	struct device *dev;
	struct mcde_chnl_state *chnl;
	char *buf;
	const char *p;
	int i = 0;
	struct mcde_oled_transform matrix;
	u16 coeff;
	enum dbg_matrix_type matrix_type;
	int num_buffers;

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		goto exit;
	}
	dev = &ddev->dev;

	dev_dbg(dev, "%s\n", __func__);

	reset_res_buf();

	buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!buf) {
		dev_err(dev, "%s: Failed to allocate buffer\n", __func__);
		goto exit;
	}

	dev_dbg(dev, "%s: buf = %p, ubuf = %p, count = %d, "
				"sizeof(char) * count = %d, line = %d\n",
		__func__, buf, ubuf, count, sizeof(char) * count, __LINE__);

	if (copy_from_user(buf, ubuf, count))
		goto fail_free_mem;

	p = buf;

	if (!strncmp(buf, "rgb", 3)) {
		dev_dbg(dev, "%s: rgb\n", __func__);
		matrix_type = RGB;
	} else if (!strncmp(buf, "yuv", 3)) {
		dev_dbg(dev, "%s: yuv\n", __func__);
		matrix_type = YUV;
	} else {
		update_res_buf("Get matrix failed - unknown matrix\n");
		goto fail_free_mem;
	}

	p = p+4;

	/* Get data */
	while (true) {
		if (isspace(*p)) {
			p++;
		} else {
			if (sscanf(p, "%4hx", &coeff) == 1) {
				while (isxdigit(*p) || (*p == 'x'))
					p++;
			}
			if (i < 9)
				matrix.matrix[i / 3][i % 3] = coeff;
			else
				matrix.offset[i - 9] = coeff;
			i++;
		}
		if (iscntrl(*p))
			break;

		if (i > NBR_MATRIX_PARAMETERS)
			break;
	}

	if (i != NBR_MATRIX_PARAMETERS) {
		update_res_buf("Write matrix - Wrong nbr of parameters\n");
		goto fail_free_mem;
	}

	chnl = ddev->chnl_state;
	if (matrix_type == RGB)
		set_rgb_extra_matrix(&matrix);
	else
		set_yuv_extra_matrix(&matrix);

	update_res_buf("Write matrix - Success\n");
	mcde_chnl_apply(chnl);
	num_buffers = ddev->fbi->var.yres_virtual / ddev->fbi->var.yres;
	mcde_chnl_update(chnl, num_buffers == 3);

fail_free_mem:
	kfree(buf);
exit:
	return count;
}

static ssize_t read_matrix(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mcde_display_device *ddev = s->private;
	struct device *dev;
	char *buf;
	char tmp[TMP_BUF_SZ];
	struct mcde_oled_transform *matrix;

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		goto exit;
	}
	dev = &ddev->dev;

	dev_dbg(dev, "%s\n", __func__);

	reset_res_buf();

	buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!buf) {
		dev_err(dev, "%s: Failed to allocate buffer\n", __func__);
		goto exit;
	}

	dev_dbg(dev, "%s: buf = %p, ubuf = %p, count = %d, "
				"sizeof(char) * count = %d, line = %d\n",
		__func__, buf, ubuf, count, sizeof(char) * count, __LINE__);

	if (copy_from_user(buf, ubuf, count))
		goto fail_free_mem;

	if (!strncmp(buf, "rgb", 3)) {
		dev_dbg(dev, "%s: rgb\n", __func__);
		matrix = get_rgb_extra_matrix();
	} else if (!strncmp(buf, "yuv", 3)) {
		dev_dbg(dev, "%s: yuv\n", __func__);
		matrix = get_yuv_extra_matrix();
	} else {
		update_res_buf("Get matrix failed - unknown matrix\n");
		goto fail_free_mem;
	}

	snprintf(tmp, sizeof(tmp), "matrix =\n0x%hx 0x%hx 0x%hx\n0x%hx 0x%hx "
		"0x%hx\n0x%hx 0x%hx 0x%hx\nOffset = 0x%hx 0x%hx 0x%hx\n",
		matrix->matrix[0][0], matrix->matrix[0][1],
		matrix->matrix[0][2], matrix->matrix[1][0],
		matrix->matrix[1][1], matrix->matrix[1][2],
		matrix->matrix[2][0], matrix->matrix[2][1],
		matrix->matrix[2][2], matrix->offset[0],
		matrix->offset[1], matrix->offset[2]);
	update_res_buf(tmp);

fail_free_mem:
	kfree(buf);
exit:
	return count;
}

static int info_open(struct inode *inode, struct file *file)
{
	return single_open(file, info_show, inode->i_private);
}

static const struct file_operations info_fops = {
	.owner		= THIS_MODULE,
	.open		= info_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int read_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations read_fops = {
	.owner			= THIS_MODULE,
	.open			= read_open,
	.write			= reg_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int write_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations write_fops = {
	.owner			= THIS_MODULE,
	.open			= write_open,
	.write			= reg_write,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int panels_open(struct inode *inode, struct file *file)
{
	return single_open(file, panels_show, inode->i_private);
}

static const struct file_operations panels_fops = {
	.owner		= THIS_MODULE,
	.open		= panels_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int result_open(struct inode *inode, struct file *file)
{
	return single_open(file, result_show, inode->i_private);
}

static const struct file_operations result_fops = {
	.owner		= THIS_MODULE,
	.open		= result_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int reset_open(struct inode *inode, struct file *file)
{
	return single_open(file, reset, inode->i_private);
}

static const struct file_operations reset_fops = {
	.owner		= THIS_MODULE,
	.open		= reset_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int restart_open(struct inode *inode, struct file *file)
{
	return single_open(file, restart, inode->i_private);
}

static const struct file_operations restart_fops = {
	.owner		= THIS_MODULE,
	.open		= restart_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int write_matrix_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations write_matrix_fops = {
	.owner			= THIS_MODULE,
	.open			= write_matrix_open,
	.write			= write_matrix,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int read_matrix_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations read_matrix_fops = {
	.owner			= THIS_MODULE,
	.open			= read_matrix_open,
	.write			= read_matrix,
	.llseek			= seq_lseek,
	.release		= single_release,
};

void __devinit panel_create_debugfs(struct mcde_display_device *ddev)
{
	struct device *dev;
	struct panel_record *rd;

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		return;
	}
	dev = &ddev->dev;
	rd = dev_get_drvdata(dev);

	dev_dbg(dev, "%s: create folder %s\n", __func__,
						kobject_name(&dev->kobj));
	rd->dir = debugfs_create_dir(kobject_name(&dev->kobj), 0);
	if (!rd->dir) {
		dev_err(dev, "%s: dbgfs create dir failed\n", __func__);
	} else {
		if (!debugfs_create_file("info", S_IRUGO, rd->dir, ddev,
								&info_fops)) {
			dev_err(dev, "%s: failed to create dbgfs info file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("read", S_IWUSR, rd->dir, ddev,
								&read_fops)) {
			dev_err(dev, "%s: failed to create dbgfs read file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("write", S_IWUSR, rd->dir, ddev,
								&write_fops)) {
			dev_err(dev, "%s: failed to create dbgfs write file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("panels", S_IRUGO, rd->dir, ddev,
							&panels_fops)) {
			dev_err(dev, "%s: failed to create dbgfs panels file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("result", S_IRUGO, rd->dir, ddev,
								&result_fops)) {
			dev_err(dev, "%s: failed to create dbgfs result file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("reset", S_IRUGO, rd->dir, ddev,
								&reset_fops)) {
			dev_err(dev, "%s: failed to create dbgfs reset file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("restart", S_IRUGO, rd->dir, ddev,
							&restart_fops)) {
			dev_err(dev,
				"%s: failed to create dbgfs restart file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("read_matrix", S_IRUGO, rd->dir, ddev,
							&read_matrix_fops)) {
			dev_err(dev,
				"%s: failed to create dbgfs read_matrix file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("write_matrix", S_IRUGO, rd->dir, ddev,
							&write_matrix_fops)) {
			dev_err(dev, "%s: failed to create dbgfs write_matrix "
							"file\n", __func__);
			return;
		}
	}
}

void __devexit panel_remove_debugfs(struct mcde_display_device *ddev)
{
	struct device *dev;
	struct panel_record *rd;

	if (!ddev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		return;
	}
	dev = &ddev->dev;
	rd = dev_get_drvdata(dev);
	debugfs_remove_recursive(rd->dir);
}
