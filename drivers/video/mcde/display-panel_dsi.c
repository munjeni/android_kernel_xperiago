/* drivers/video/mcde/display-panel_dsi.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Sony Ericsson DSI display driver
 *
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#endif

#include <video/mcde_dss.h>
#include <video/mcde_display.h>
#include <video/mcde_display-panel_dsi.h>

#define DDB_READ_NBR_RETRIES 5
#define PLUG_N_PLAY_RETRIES 2

static int platform_startup_reset(struct mcde_display_device *ddev)
{
	int ret = -ENODEV;

	if (ddev->platform_reset) {
		ret = ddev->platform_reset(ddev, 1);
		if (ret)
			goto exit;
		usleep_range(11000, 20000); /* Worst spec says > 10 ms */
		ret = ddev->platform_reset(ddev, 0);
		if (ret)
			goto exit;
		usleep_range(21, 1000); /* Worst spec says > 20 us */
		ret = ddev->platform_reset(ddev, 1);
		if (ret)
			goto exit;
		msleep(21); /* Worst spec says > 32 ms from first reset high */
	}
exit:
	return ret;
}

static int panel_handle_cmd_reset(struct panel_device *dev,
						const struct panel_reg preg)
{
	struct mcde_display_device *ddev = &dev->base;
	int ret = -EINVAL;

	if (preg.addr == 0) {
		dev_vdbg(&ddev->dev, "%s: CMD_RESET lvl=%d\n", __func__,
							preg.value[0]);
		if (ddev->platform_reset)
			ret = ddev->platform_reset(ddev, preg.value[0]);
	} else {
		dev_err(&ddev->dev, "%s: CMD_RESET, but addr is not 0 "
					"(addr = 0x%x)\n", __func__, preg.addr);
	}
	return ret;
}

static int panel_handle_cmd_platform(struct panel_device *dev,
						const struct panel_reg preg)
{
	struct mcde_display_device *ddev = &dev->base;
	int ret = -EINVAL;

	if (preg.addr == 0) {
		dev_vdbg(&ddev->dev, "%s: CMD_PLATFORM val=%d\n", __func__,
							preg.value[0]);
		if (preg.value[0] == 0) {
			if (ddev->platform_disable)
				ret = ddev->platform_disable(ddev);
		} else if (preg.value[0] == 1) {
			if (ddev->platform_enable)
				ret = ddev->platform_enable(ddev);
		} else {
			dev_err(&ddev->dev, "%s: CMD_PLATFORM, but value is "
						"invalid (value = %d)\n",
						__func__, preg.value[0]);
		}
	} else {
		dev_err(&ddev->dev, "%s: CMD_PLATFORM, but addr is not 0 "
					"(addr = 0x%x)\n", __func__, preg.addr);
	}
	return ret;
}

static int panel_handle_cmd_gen(struct panel_device *dev,
						const struct panel_reg preg)
{
	struct mcde_display_device *ddev = &dev->base;
	int ret = -EINVAL;
	int i;
	int len = preg.len + 1; /* addr included in para */
	u8 para[MCDE_MAX_DSI_DIRECT_CMD_WRITE];

	if (len > (MCDE_MAX_DSI_DIRECT_CMD_WRITE - 1))
		len = MCDE_MAX_DSI_DIRECT_CMD_WRITE - 1;

	dev_vdbg(&ddev->dev, "%s: CMD_GEN: addr:0x%.2X, len:%.2d\n",
				__func__, preg.addr, len);
	para[0] = preg.addr;
	for (i = 0; i < len; i++) {
		para[i+1] = preg.value[i];
		dev_vdbg(&ddev->dev, "data[%d]=%.2X\n", i, preg.value[i]);
	}

	ret = mcde_dsi_generic_write(ddev->chnl_state, para, len + 1);

	return ret;
}

static int panel_handle_cmd_dcs(struct panel_device *dev,
						const struct panel_reg preg)
{
	struct mcde_display_device *ddev = &dev->base;
	int ret = -EINVAL;
	int i;
	int len = preg.len;

	if (len > MCDE_MAX_DSI_DIRECT_CMD_WRITE)
		len = MCDE_MAX_DSI_DIRECT_CMD_WRITE;

	dev_vdbg(&ddev->dev, "%s: CMD_DCS: addr:0x%.2X, len:%.2d ",
				__func__, preg.addr, len);
	for (i = 0; i < len; i++)
		dev_vdbg(&ddev->dev, "data[%d]=%.2X\n",
						i, preg.value[i]);

	ret = mcde_dsi_dcs_write(ddev->chnl_state, preg.addr,
							(u8 *)preg.value, len);

	return ret;
}

static int panel_execute_cmd(struct panel_device *dev,
						const struct panel_reg *preg)
{
	struct mcde_display_device *ddev = &dev->base;
	struct panel_record *rd;
	int n;
	int ret = 0;

	if (!preg) {
		dev_err(&ddev->dev, "%s: no register\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	rd = dev_get_drvdata(&ddev->dev);
	if (!rd) {
		dev_err(&ddev->dev, "%s: no record\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	for (n = 0; ; ++n)
		switch (preg[n].type) {
		case CMD_END:
			if (preg[n].addr == 0)
				if (preg[n].value[0] == 0)
					goto exit;
			break;
		case CMD_WAIT_MS:
			if (preg[n].addr == 0) {
				dev_dbg(&ddev->dev, "%s: CMD_WAIT_MS = %d ms\n",
					__func__, preg[n].value[0]);
				/* No need to use usleep_range() here since
				   all delays are specified as "more than" */
				msleep(preg[n].value[0]);
			} else {
				dev_err(&ddev->dev, "%s: CMD_WAIT_MS, but addr"
					" is not 0 (addr = 0x%x)\n", __func__,
					preg[n].addr);
			}
			break;
		case CMD_GEN:
			ret = panel_handle_cmd_gen(dev, preg[n]);
			if (ret)
				goto exit;
			break;
		case CMD_DCS:
			ret = panel_handle_cmd_dcs(dev, preg[n]);
			if (ret)
				goto exit;
			break;
		case CMD_RESET:
			ret = panel_handle_cmd_reset(dev, preg[n]);
			if (ret)
				goto exit;
			break;
		case CMD_PLATFORM:
			ret = panel_handle_cmd_platform(dev, preg[n]);
			if (ret)
				goto exit;
			break;
		default:
			dev_err(&ddev->dev, "%s: Unknown command type!\n",
								__func__);
		}
exit:
	return ret;
}

#ifdef CONFIG_DEBUG_FS
int panel_execute_cmd_extern(struct panel_device *dev,
						const struct panel_reg *preg)
{
	return panel_execute_cmd(dev, preg);
}
#endif

static int panel_set_power_mode_internal(struct mcde_display_device *ddev,
					struct panel_record *rd,
					enum mcde_display_power_mode power_mode)
{
	struct panel_device *dev = container_of(ddev, struct panel_device,
									base);
	int ret = 0;

	if ((!rd->panel) || (!rd->panel->pinfo)) {
		dev_err(&ddev->dev, "%s: no panel\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if ((ddev->power_mode == MCDE_DISPLAY_PM_OFF) &&
					(power_mode != MCDE_DISPLAY_PM_OFF)) {
		if (ddev->deep_standby_as_power_off) {
			dev_dbg(&ddev->dev, "%s: DEEP STANDBY->STANDBY\n",
								__func__);
			ret = panel_execute_cmd(dev,
				rd->panel->pinfo->deep_standby_to_standby);
			if (ret)
				goto exit;
		} else {
			dev_dbg(&ddev->dev, "%s: OFF->STANDBY\n", __func__);
			ret = panel_execute_cmd(dev,
					rd->panel->pinfo->off_to_standby);
			if (ret)
				goto exit;
		}

		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
		/* force register settings */
		if (ddev->port->type == MCDE_PORTTYPE_DPI)
			ddev->update_flags = UPDATE_FLAG_VIDEO_MODE |
						UPDATE_FLAG_PIXEL_FORMAT;
	}

	if (ddev->port->type == MCDE_PORTTYPE_DSI) {
		if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
				power_mode >= MCDE_DISPLAY_PM_INTERMEDIATE) {
			if (rd->panel->pinfo->standby_to_intermediate) {
				dev_dbg(&ddev->dev,
						"%s: STANDBY->INTERMEDIATE\n",
						__func__);
				ret = panel_execute_cmd(dev,
						rd->panel->pinfo->
						standby_to_intermediate);
				if (ret)
					goto exit;
				ddev->power_mode = MCDE_DISPLAY_PM_INTERMEDIATE;
			}
		}
		if (ddev->power_mode == MCDE_DISPLAY_PM_INTERMEDIATE &&
					power_mode == MCDE_DISPLAY_PM_ON) {
			dev_dbg(&ddev->dev, "%s: INTERMEDIATE->ON\n", __func__);

			ret = panel_execute_cmd(dev,
					rd->panel->pinfo->intermediate_to_on);
			dev_info(&ddev->dev, "%s: Display on\n", __func__);
			ddev->power_mode = MCDE_DISPLAY_PM_ON;
			goto set_power_and_exit;
		} else if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
					power_mode == MCDE_DISPLAY_PM_ON) {
			dev_dbg(&ddev->dev, "%s: STANDBY->ON\n", __func__);

			ret = panel_execute_cmd(dev,
					rd->panel->pinfo->standby_to_on);
			dev_info(&ddev->dev, "%s: Display on\n", __func__);
			ddev->power_mode = MCDE_DISPLAY_PM_ON;
			goto set_power_and_exit;
		} else if (ddev->power_mode == MCDE_DISPLAY_PM_ON &&
					power_mode <= MCDE_DISPLAY_PM_STANDBY) {
			dev_dbg(&ddev->dev, "%s: ON->STANDBY\n", __func__);
			ret = panel_execute_cmd(dev,
					rd->panel->pinfo->on_to_standby);
			if (ret)
				goto exit;
			dev_info(&ddev->dev, "%s: Display off\n", __func__);
			ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
		}
	} else if (ddev->port->type == MCDE_PORTTYPE_DPI) {
		ddev->power_mode = power_mode;
	} else if (ddev->power_mode != power_mode) {
		ret = -EINVAL;
		goto exit;
	}

	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
					power_mode == MCDE_DISPLAY_PM_OFF) {
		if (ddev->deep_standby_as_power_off) {
			dev_dbg(&ddev->dev, "%s: STANDBY->DEEP STANDBY\n",
								__func__);

			ret = panel_execute_cmd(dev,
				rd->panel->pinfo->standby_to_deep_standby);
			if (ret)
				goto exit;
		} else {
			dev_dbg(&ddev->dev, "%s: STANDBY->OFF\n", __func__);

			ret = panel_execute_cmd(dev,
					rd->panel->pinfo->standby_to_off);
			if (ret)
				goto exit;

		}
		ddev->power_mode = MCDE_DISPLAY_PM_OFF;
	}

set_power_and_exit:
	if (ddev->chnl_state)
		mcde_chnl_set_power_mode(ddev->chnl_state, ddev->power_mode);
exit:
	return ret;
}

static int panel_set_power_mode(struct mcde_display_device *ddev,
					enum mcde_display_power_mode power_mode)
{
	struct panel_device *dev = container_of(ddev, struct panel_device,
									base);
	struct panel_record *rd;
	int ret;

	if (!dev || !&ddev->dev) {
		pr_err("%s: no device\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	rd = dev_get_drvdata(&ddev->dev);
	if (!rd) {
		dev_err(&ddev->dev, "%s: no record\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	dev_dbg(&ddev->dev, "%s: power_mode = %d\n", __func__, power_mode);

	mutex_lock(&rd->lock);
	ret = panel_set_power_mode_internal(ddev, rd, power_mode);
	mutex_unlock(&rd->lock);
exit:
	return ret;
}

static int panel_try_video_mode(struct mcde_display_device *ddev,
				struct mcde_video_mode *video_mode)
{
	struct mcde_video_mode *vmode = &ddev->video_mode;

	/* Report part of video mode parameters to pass CTS test */
	video_mode->pixclock = vmode->pixclock;
	video_mode->vfp = vmode->vfp;

	return 0;
}


static int __devinit panel_id_reg_check(struct panel_device *dev,
					struct mcde_chnl_state *chnl,
					const struct panel *panel)
{
	int ret = 0;
	int reg = 0;
	u32 data = 0;
	int len = 1;

	dev_dbg(&dev->base.dev, "%s\n", __func__);

	for (reg = 0; ; ++reg) {
		if (panel->id_regs[reg].type == CMD_END) {
			if (reg == 0)
				return -ENODEV;
			break;
		} else {
			ret = mcde_dsi_dcs_read(chnl, panel->id_regs[reg].addr,
								&data, &len);
			dev_vdbg(&dev->base.dev, "%s: ret = %d, R0x%hhX = "
						"0x%.8x\n", __func__, ret,
						panel->id_regs[reg].addr, data);

			if (ret < 0) {
				dev_err(&dev->base.dev, "%s: mcde read error, "
						"ret = %d\n", __func__, ret);
				return -EIO;
			}

			if ((u8)data != panel->id_regs[reg].value[0])
				return -ENODEV;
		}
	}
	return 0;
}

static bool __devinit ddb_find_exit_code(struct panel_device *dev, u32 d)
{
	bool ret = false;
	int i = 0;
	u8 tmp;

	for (i = 0; i <= 3 ; i++) {
		tmp = (u8)(d >> (i * 8));
		if (tmp == DDB_EXIT) {
			dev_vdbg(&dev->base.dev, "%s: DDB_EXIT found!\n",
								__func__);
			ret = true;
			break;
		}
	}

	return ret;
}

static int __devinit read_device_descriptor_block(struct panel_device *dev,
					struct mcde_chnl_state *chnl,
					u32 *ddb,
					int *len)
{
	int ret = 0;
	u32 data[DISPLAY_DDB_LENGTH];
	int rd_len;
	u32 curr_data = 0;
	bool exit_code_found = false;
	int i;
	int try;
	bool read_ok = false;

	dev_dbg(&dev->base.dev, "%s\n", __func__);

	memset(data, 0, sizeof(data));
	*len = 0;

	/* Start by reading DDBs once */
	for (try = 0; try < DDB_READ_NBR_RETRIES ; try++) {
		/* Continue even if we cannot send this command */
		(void)mcde_dsi_set_max_pkt_size(chnl);

		rd_len = MCDE_MAX_DCS_READ;
		ret = mcde_dsi_dcs_read(chnl, DCS_CMD_READ_DDB_START,
					&data[0], &rd_len);
		dev_vdbg(&dev->base.dev, "%s: 0xA1 = 0x%.8x, rd_len=%d\n",
						__func__, data[0], rd_len);
		if (ret < 0) {
			dev_warn(&dev->base.dev, "%s: mcde read error DDBs, "
					"ret = %d, rd_len=%d, try again!\n",
					__func__, ret, rd_len);
			continue;
		}
		if (rd_len == 0) {
			dev_warn(&dev->base.dev, "%s Read returned zero byte,"
						" try again!\n", __func__);
			continue;
		}
		if (rd_len != MCDE_MAX_DCS_READ) {
			dev_info(&dev->base.dev, "%s Read returned %d byte,"
			" wanted %d!\n", __func__, rd_len, MCDE_MAX_DCS_READ);
			read_ok = true;
			break;
		} else {
			read_ok = true;
			break;
		}
	}

	if (!read_ok && (try >= DDB_READ_NBR_RETRIES - 1)) {
		dev_info(&dev->base.dev, "%s Max nbr tries reached!\n",
					__func__);
		ret = -EIO;
		goto fail_exit;
	}

	exit_code_found = ddb_find_exit_code(dev, data[0]);

	/* Continue by issuing DDBc until stop condition is met */
	for (i = 1; (i < DISPLAY_DDB_LENGTH) && !exit_code_found; i++) {
		ret = mcde_dsi_dcs_read(chnl, DCS_CMD_READ_DDB_CONTINUE,
							&curr_data, &rd_len);
		dev_vdbg(&dev->base.dev, "%s: data = 0x%.8x, rd_len=%d, i=%d\n",
						__func__, curr_data, rd_len, i);

		if ((rd_len < 1) || (ret < 0)) {
			dev_err(&dev->base.dev, "%s: mcde read error DDBc, "
				"ret = %d, rd_len=%d\n", __func__, ret, rd_len);
			ret = -EIO;
			goto fail_exit;
		}
		if (rd_len == 1)
			data[i / 4] |= curr_data << (8 * (i % 4));
		else
			data[i] = curr_data;
		exit_code_found = ddb_find_exit_code(dev, curr_data);
	}

	if (rd_len == 1)
		*len = (i + 3) / 4;
	else
		*len = i;
	memcpy(ddb, data, (*len) * sizeof(data[0]));

	if (*len == DISPLAY_DDB_LENGTH) {
		ret = -EIO;
		goto fail_exit;
	}
	return 0;
fail_exit:
	return ret;
}

static int __devinit ddb_compare(struct panel_device *dev,
					const struct panel *panel,
					u32 *rdddb,
					int len)
{
	u8 i = 0;
	u32 val = 0;
	u8 match_cnt = 0;
	u32 mask;
	u32 mddb;

	dev_dbg(&dev->base.dev, "%s\n", __func__);

	if (!panel->ddb || !panel->ddb_mask)
		goto fail_exit;

	/* compare panel data with actual read data */
	do {
		val = panel->ddb[i];
		mask = panel->ddb_mask[i];
		mddb = rdddb[i] & mask;

		dev_vdbg(&dev->base.dev, "%s: i:%d, val:0x%.8x, rddb: 0x%.8x,"
			"mddb:0x%.8x\n", __func__, i, val, rdddb[i], mddb);

		if ((val == DDB_END) && (i == 0))
			goto fail_exit;

		if ((val == DDB_END) && (match_cnt > 0))
			goto found_exit;

		if (val == mddb)
			match_cnt++;
		else
			goto fail_exit;

		/* For some panels we never find the exit byte, in those cases
		   we only match on the first word */
		if ((len == DISPLAY_DDB_LENGTH) && (i == 0) && (val == mddb))
			goto found_exit;
		i++;

		if (ddb_find_exit_code(dev, val))
			break;
	} while (i < len);

	dev_vdbg(&dev->base.dev, "%s: match_cnt=%d, i=%d\n",
							__func__, match_cnt, i);
	if (match_cnt == i)
		goto found_exit;
fail_exit:
	dev_dbg(&dev->base.dev, "%s: No match found\n", __func__);
	return -EINVAL;
found_exit:
	dev_dbg(&dev->base.dev, "%s: Match found\n", __func__);
	return 0;
}

static int __devinit display_plug_n_play(struct panel_device *dev,
					struct mcde_chnl_state *chnl,
					struct panel_platform_data *pdata,
					struct panel_record *rd)
{
	int ret = 0;
	u32 rdddb[DISPLAY_DDB_LENGTH];
	int len;
	int n = 0;

	dev_dbg(&dev->base.dev, "%s\n", __func__);

	/* First try with DDB */
	ret = read_device_descriptor_block(dev, chnl, rdddb, &len);

	if (len == DISPLAY_DDB_LENGTH) {
		/* Even if no exit was found, we might have correct data
		   for some panels */
		ret = 0;
	}

	if (ret)
		goto read_fail;

	while (pdata->panels[n] != NULL) {
		dev_dbg(&dev->base.dev, "%s: panel[%d]\n", __func__, n);
		ret = ddb_compare(dev, pdata->panels[n], rdddb, len);
		if (!ret)
			goto found_exit;
		n++;
	}

	/* Secondly try old method, with 0xDA, 0xDB, 0xDC regs */
	n = 0;
	while (pdata->panels[n] != NULL) {
		ret = panel_id_reg_check(dev, chnl, pdata->panels[n]);
		if (ret == 0)
			goto found_exit;
		if (ret == -EIO)
			goto read_fail;
		n++;
	}

read_fail:
	dev_err(&dev->base.dev, "%s: Plug n Play failed!\n", __func__);
	for (n = 0; n < len; n++)
		dev_err(&dev->base.dev, "%s: rdddb[%d] = 0x%.8x\n", __func__, n,
								rdddb[n]);
	return ret;

found_exit:
	dev_dbg(&dev->base.dev, "%s: Plug n Play success!\n", __func__);
	rd->panel = pdata->panels[n];
	rd->ddb_id = rdddb[0];
	return 0;
}

static ssize_t ddb_id_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct panel_record *rd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%.8x", rd->ddb_id);
}

static struct device_attribute attributes[] = {
	__ATTR(ddb_id, 0444, ddb_id_show, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;
error:
	for (; i >= 0 ; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: Unable to create interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static int __devinit panel_probe(struct mcde_display_device *ddev)
{
	struct mcde_chnl_state *chnl = NULL;
	struct panel_device *dev = container_of(ddev, struct panel_device,
									base);
	struct panel_record *rd;
	struct panel_platform_data *pdata;
	int ret = 0;
	int i;

	dev_info(&dev->base.dev, "%s\n", __func__);

	rd = kzalloc(sizeof(struct panel_record), GFP_KERNEL);
	if (rd == NULL) {
		dev_err(&dev->base.dev, "%s: Out of memory\n", __func__);
		ret = -ENOMEM;
		goto out;
	}

	mutex_init(&rd->lock);
	dev_set_drvdata(&dev->base.dev, rd);

	/* exit sleep mode and enter normal power on */
	if (ddev->platform_enable) {
		ret = ddev->platform_enable(ddev);
		if (ret)
			goto free_and_exit;

	} else {
		dev_err(&dev->base.dev, "%s: No enable function\n", __func__);
		ret = -EINVAL;
		goto free_and_exit;
	}

	pdata = dev->base.dev.platform_data;

	/* Find correct panel */
	for (i = 0; i < PLUG_N_PLAY_RETRIES; i++) {
		ret = platform_startup_reset(ddev);
		if (ret)
			goto free_and_exit;

		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;

		/* Acquire MCDE resources */
		chnl = mcde_chnl_get(dev->base.chnl_id, dev->base.fifo,
								dev->base.port);
		if (IS_ERR(chnl)) {
			ret = PTR_ERR(chnl);
			dev_warn(&dev->base.dev, "Failed to acquire MCDE "
								"channel\n");
			goto free_and_exit;
		}

		ret = display_plug_n_play(dev, chnl, pdata, rd);
		if (!ret)
			break;
		dev_warn(&dev->base.dev, "%s: pnp failed. Retry.\n", __func__);
		mcde_chnl_put(chnl);
		chnl = NULL;
	}
	if (ret)
		goto free_and_exit;

	dev_info(&dev->base.dev, "%s: Found display: %s\n", __func__,
							rd->panel->name);

	dev->base.native_x_res = rd->panel->x_res;
	dev->base.native_y_res = rd->panel->y_res;

	dev->base.physical_width = rd->panel->width;
	dev->base.physical_height = rd->panel->height;

	dev->base.set_power_mode = panel_set_power_mode;
	dev->base.try_video_mode = panel_try_video_mode;

	ddev->port->dcs_addr_mode.normal = rd->panel->addr_mode_normal;
	ddev->port->dcs_addr_mode.hor_flip = rd->panel->addr_mode_hor_flip;

	if (rd->panel->custom_interface_init)
		rd->panel->custom_interface_init(ddev);

#ifdef CONFIG_DEBUG_FS
	panel_create_debugfs(ddev);
#endif

	/* continue even if create sysfs fails */
	create_sysfs_interfaces(&dev->base.dev);

	/* close  MCDE channel */
	mcde_chnl_put(chnl);
	chnl = NULL;
	dev_info(&dev->base.dev, "%s: probe success\n", __func__);
	return 0;

free_and_exit:
	if (chnl) {
		mcde_chnl_put(chnl);
		chnl = NULL;
	}
	dev_set_drvdata(&dev->base.dev, NULL);
	kfree(rd);
out:
	return ret;
}

static int __devexit panel_remove(struct mcde_display_device *ddev)
{
	struct panel_device *dev = container_of(ddev, struct panel_device,
									base);
	struct panel_record *rd;

	dev_dbg(&dev->base.dev, "%s\n", __func__);

	remove_sysfs_interfaces(&dev->base.dev);
#ifdef CONFIG_DEBUG_FS
	panel_remove_debugfs(ddev);
#endif
	dev->base.set_power_mode(&dev->base, MCDE_DISPLAY_PM_OFF);
	if (ddev->deep_standby_as_power_off) {
		if (ddev->platform_disable)
			(void)ddev->platform_disable(ddev);
	}
	rd = dev_get_drvdata(&dev->base.dev);

	if (rd->panel->custom_interface_remove)
		rd->panel->custom_interface_remove(ddev);

	dev_set_drvdata(&dev->base.dev, NULL);
	kfree(rd);
	return 0;
}

static struct mcde_display_driver panel_driver = {
	.probe	= panel_probe,
	.remove = panel_remove,
	.driver = {
		.name	= MCDE_DISPLAY_PANEL_NAME,
	},
};

static int __init panel_init(void)
{
	pr_info("%s: Sony Ericsson MIPI/DSI display driver, "
		"%s. (Built %s @ %s)\n", __func__, panel_driver.driver.name,
							__DATE__, __TIME__);
	return mcde_display_driver_register(&panel_driver);
}
module_init(panel_init);

static void __exit panel_exit(void)
{
	pr_info("%s\n", __func__);
	mcde_display_driver_unregister(&panel_driver);
}
module_exit(panel_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MCDE Panel DSI display driver");
