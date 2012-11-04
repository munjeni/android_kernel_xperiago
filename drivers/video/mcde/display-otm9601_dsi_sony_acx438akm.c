/* drivers/video/mcde/display-otm9601_dsi_sony_acx438akm.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Sony Ericsson Orise 9601 driver for Sony panel
 *
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/fb.h>
#include <video/mcde.h>

#include "video/mcde_display-panel_dsi.h"

static const struct panel_reg off_to_standby_cmds[] = {
	/* Spec: Power On */
	{CMD_PLATFORM, 0, 1,  {1} },
	{CMD_WAIT_MS,  0, 1,  {1} }, /* Spec says > 10 us */
	{CMD_RESET,    0, 1,  {1} },
	{CMD_WAIT_MS,  0, 1,  {6} }, /* Spec says > 5 ms */
	{CMD_RESET,    0, 1,  {0} },
	{CMD_WAIT_MS,  0, 1,  {1} }, /* Spec says > 20 us */
	{CMD_RESET,    0, 1,  {1} },
	{CMD_WAIT_MS,  0, 1, {11} }, /* Spec says > 10 ms */
	{CMD_WAIT_MS,  0, 1, {15} }, /* Spec says > 32 ms from reset high*/
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg deep_standby_to_standby_cmds[] = {
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_off_cmds[] = {
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_deep_standby_cmds[] = {
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg on_to_standby_cmds[] = {
	/* Spec: D) Set Display off */
	{CMD_DCS, DCS_CMD_SET_DISPLAY_OFF, 1, {0} },
	/* Spec: E) Sleep Set */
	{CMD_DCS, DCS_CMD_ENTER_SLEEP_MODE, 1, {0} },
	{CMD_WAIT_MS, 0, 1, {71} }, /* Spec says > 70 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_intermediate_cmds[] = {
	/* Spec: LED PWM on */
	{CMD_DCS, 0x51, 1, {0xFF} },
	{CMD_DCS, 0x53, 1, {0x2C} },
	/* Spec: SET_TEAR_ON */
	{CMD_DCS, 0x35, 1, {0x00} },
	/* Set TE Scan Line */
	{CMD_DCS, 0x44, 2, {0x01, 0x87} },

	/* Aladdin mode settings */
	{CMD_DCS, 0x55, 1, {0x07} },
	{CMD_DCS, 0x50, 1, {0x88} },
	{CMD_DCS, 0x57, 1, {0x6D} },
	{CMD_DCS, 0x58, 1, {0xFF} },
	{CMD_DCS, 0x59, 1, {0x01} },
	{CMD_DCS, 0x5A, 1, {0x02} },
	{CMD_DCS, 0x5B, 1, {0x00} },
	{CMD_DCS, 0x5C, 1, {0x06} },
	{CMD_DCS, 0x5D, 1, {0x51} },
	{CMD_DCS, 0x60, 1, {0x08} },
	{CMD_DCS, 0x61, 1, {0x25} },
	{CMD_DCS, 0x62, 1, {0x40} },
	{CMD_DCS, 0x63, 1, {0x05} },
	{CMD_DCS, 0x64, 1, {0x40} },
	{CMD_DCS, 0x65, 1, {0x64} },
	{CMD_DCS, 0x66, 1, {0x59} },
	{CMD_DCS, 0x67, 1, {0x02} },
	{CMD_DCS, 0x69, 1, {0x80} },
	{CMD_DCS, 0x6A, 1, {0x80} },
	{CMD_DCS, 0x6B, 1, {0x80} },

	/* Spec: B) Exit Sleep */
	{CMD_DCS, DCS_CMD_EXIT_SLEEP_MODE, 1, {0} },
	/* Spec says wait > 10 ms (for RAM to be ready), another 83 ms */
	/* (5 frames at 60 Hz) is needed to wait for the TE signal to be */
	/* enabled. */
	{CMD_WAIT_MS, 0, 1, {94} },
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg intermediate_to_on_cmds[] = {
	/* Spec: C) Display On */
	{CMD_DCS, DCS_CMD_SET_DISPLAY_ON, 1, {0} },
	{CMD_END, 0, 1, {0} }
};

static const u32 ddb_rev23841A01[] = {
	0x011A8423,
	0x000000FF,
	DDB_END
};

static const u32 ddb_mask_rev23841A01[] = {
	0xFFFFFFFF,
	0x000000FF,
	DDB_END
};

static const u32 ddb_2384_no_rev[] = {
	0x00008423,
	0x000000FF,
	DDB_END
};

static const u32 ddb_mask_2384_no_rev[] = {
	0x0000FFFF,
	0x000000FF,
	DDB_END
};

static const struct panel_reg id_regs[] = {
	{CMD_END, 0, 1, {0} }
};

static struct panel_controller otm9601_controller_panel = {
	.off_to_standby			= off_to_standby_cmds,
	.deep_standby_to_standby	= deep_standby_to_standby_cmds,
	.standby_to_off			= standby_to_off_cmds,
	.standby_to_deep_standby	= standby_to_deep_standby_cmds,
	.on_to_standby			= on_to_standby_cmds,
	.standby_to_intermediate	= standby_to_intermediate_cmds,
	.intermediate_to_on		= intermediate_to_on_cmds,
};

static struct mcde_oled_transform outdoor_green_shift_fix = {
	.matrix = {
		{0x1051, 0x223D, 0x023D},
		{0x21EB, 0x1051, 0x01EB},
		{0x21EB, 0x01EB, 0x1028},
	},
	.offset = {0x2051, 0x2051, 0x2028},
};

static struct mcde_oled_transform yuv240_2_rgb_with_green_shift_fix = {
	/* Note that in MCDE YUV 422 pixels come as VYU pixels */
	.matrix = {
		{0x1BE2, 0x12FE, 0x0559},
		{0x3052, 0x12FE, 0x2289},
		{0x0000, 0x12A0, 0x1FFF},
	},
	.offset = {0x3221, 0x07E2, 0x3150},
};

static ssize_t panel_store_set_mode(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret;
	struct mcde_display_device *ddev =
			container_of(dev, struct mcde_display_device, dev);
	struct panel_device *pdev = container_of(ddev, struct panel_device,
									base);
	struct panel_record *rd = dev_get_drvdata(&pdev->base.dev);
	struct mcde_chnl_state *chnl = ddev->chnl_state;
	unsigned long val;
	u8 mode;
	bool outdoor_matrix_enable;
	int num_buffers;

	dev_dbg(dev, "%s\n", __func__);

	if (strict_strtoul(buf, 16, &val)) {
		dev_err(dev, "%s param error\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if (val >= 4) {
		/* Outdoor mode should be set to 0x05 for */
		/* AP2 units and later (MSB of ddb_id >= 0x05) */
		if ((rd->ddb_id >> 24) >= 0x05)
			mode = 0x05;
		else
			mode = 0x04;
		outdoor_matrix_enable = true;
	} else {
		/* All indoor modes will be set to use free control mode */
		mode = 0x07;
		outdoor_matrix_enable = false;
	}
	mcde_dsi_dcs_write(chnl, 0x55, &mode, 1);
	mcde_extra_oled_conversion(outdoor_matrix_enable);
	num_buffers = ddev->fbi->var.yres_virtual / ddev->fbi->var.yres;
	mcde_chnl_apply(chnl);
	mcde_chnl_update(chnl, num_buffers == 3);
	ret = strnlen(buf, count);
exit:
	return ret;
}

static struct device_attribute panel_attributes[] = {
	__ATTR(mode, 0644, NULL, panel_store_set_mode),
};

static int panel_sysfs_register(struct device *dev)
{
	int i;

	dev_dbg(dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(panel_attributes); i++)
		if (device_create_file(dev, panel_attributes + i))
			goto error;

	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, panel_attributes + i);

	dev_err(dev, "%s: Unable to create interface\n", __func__);

	return -ENODEV;
}

static void panel_sysfs_remove(struct device *dev)
{
	int i;

	dev_dbg(dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(panel_attributes); i++)
		device_remove_file(dev, panel_attributes + i);
}

static int custom_interface_init(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct panel_device *dev = container_of(ddev, struct panel_device,
									base);
	struct panel_record *rd = dev_get_drvdata(&dev->base.dev);

	dev_dbg(&dev->base.dev, "%s\n", __func__);

	panel_sysfs_register(&dev->base.dev);
	if (rd->panel->disable_ulpm)
		mcde_disable_ulpm_support(true);

	set_rgb_extra_matrix(&outdoor_green_shift_fix);
	set_yuv_extra_matrix(&yuv240_2_rgb_with_green_shift_fix);
	return ret;
}

static int custom_interface_remove(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct panel_device *dev = container_of(ddev, struct panel_device,
									base);

	dev_dbg(&dev->base.dev, "%s\n", __func__);

	panel_sysfs_remove(&ddev->dev);

	return ret;
}

const struct panel otm9601_dsi_sony_acx438akm_rev23841A01 = {
	.name = "Sony ACX438AKM MIPI/DSI 23841A01 (SP1)",
	.ddb = ddb_rev23841A01,
	.ddb_mask = ddb_mask_rev23841A01,
	.id_regs = id_regs,
	.pinfo = &otm9601_controller_panel,
	.x_res = 540,
	.y_res = 960,
	.width = 50,
	.height = 89,
	.ddr_qos_value = 100,
	.ape_qos_value = 100,
	.custom_interface_init = custom_interface_init,
	.custom_interface_remove = custom_interface_remove,
	.disable_ulpm = true,
};

const struct panel otm9601_dsi_sony_acx438akm_2384_no_rev = {
	.name = "Sony ACX438AKM MIPI/DSI 2384 no rev",
	.ddb = ddb_2384_no_rev,
	.ddb_mask = ddb_mask_2384_no_rev,
	.id_regs = id_regs,
	.pinfo = &otm9601_controller_panel,
	.x_res = 540,
	.y_res = 960,
	.width = 50,
	.height = 89,
	.ddr_qos_value = 100,
	.ape_qos_value = 100,
	.custom_interface_init = custom_interface_init,
	.custom_interface_remove = custom_interface_remove,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MCDE DSI Sony ACX438AKM display driver");
