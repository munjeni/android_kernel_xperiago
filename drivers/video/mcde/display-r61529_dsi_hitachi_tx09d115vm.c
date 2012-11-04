/* drivers/video/mcde/display-r61529_dsi_hitachi_tx09d115vm.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Sony Mobile Renesas 61529 driver for Hitachi panel
 *
 * Author: Yantao Pan <yantao2.pan@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <video/mcde_display-panel_dsi.h>

static const struct panel_reg off_to_standby_cmds[] = {
	/* Spec: Power On */
	{CMD_PLATFORM, 0, 1,  {1} },
	{CMD_RESET,    0, 1,  {0} },
	{CMD_WAIT_MS,  0, 1,  {2} }, /* Spec says > 1 ms */
	{CMD_RESET,    0, 1,  {1} },
	{CMD_WAIT_MS,  0, 1,  {6} }, /* Spec says > 5 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg deep_standby_to_standby_cmds[] = {
	/* Sequence not in spec */
	{CMD_RESET,   0, 1, {0} },
	{CMD_WAIT_MS, 0, 1, {2} }, /* Spec says > 1 ms */
	{CMD_RESET,   0, 1, {1} },
	{CMD_WAIT_MS, 0, 1, {6} }, /* Spec says > 5 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_off_cmds[] = {
	/* Sequence not in spec */
	{CMD_RESET,    0, 1, {0} },
	{CMD_WAIT_MS,  0, 1, {11} },
	{CMD_PLATFORM, 0, 1, {0} },
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_deep_standby_cmds[] = {
	{CMD_GEN, 0xB0, 1, {0x04} },
	{CMD_GEN, 0xB1, 1, {0x01} },
	{CMD_WAIT_MS, 0, 1, {2} }, /* Spec says > 1 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg on_to_standby_cmds[] = {
	{CMD_DCS, DCS_CMD_SET_DISPLAY_OFF, 1, {0} },
	{CMD_WAIT_MS, 0, 1, {15} }, /* Spec says < 16 ms */
	{CMD_DCS, DCS_CMD_ENTER_SLEEP_MODE, 1, {0} },
	{CMD_WAIT_MS, 0, 1, {85} }, /* Spec says > 84 ms */

	{CMD_DCS, 0x34, 1, {0x00} }, /* TE off */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_intermediate_cmds[] = {
	{CMD_DCS, 0x35, 1, {0x00} }, /* TE on */

	/* Spec: Exit Sleep */
	{CMD_DCS, DCS_CMD_EXIT_SLEEP_MODE, 1,  {0} },
	{CMD_WAIT_MS, 0, 1, {101} }, /* Spec says > 100 ms */

	/*Off DBC function*/
	{CMD_GEN, 0xB0, 1, {0x04} }, /* Access Protect */
	{CMD_GEN, 0xB8, 1, {0x00} }, /* DBC off */
	{CMD_GEN, 0xB9, 4, {0x00, 0xFF, 0x01, 0x18} }, /* LCD_PWM DBC off */

	/*Update Image*/
	{CMD_DCS, 0x2A, 4, {0x00, 0x00, 0x01, 0x3F} },
	{CMD_DCS, 0x2B, 4, {0x00, 0x00, 0x01, 0xDF} },

	/* ...send pixel data ...*/

	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg intermediate_to_on_cmds[] = {
	/* Spec: Display On */
	{CMD_DCS, DCS_CMD_SET_DISPLAY_ON, 1,  {0} },
	{CMD_WAIT_MS, 0, 1, {15} }, /* Spec says < 16 ms */
	{CMD_END, 0, 1, {0} }
};

static const u32 ddb_5210_no_rev[] = {
	0x00001052,
	DDB_END
};

static const u32 ddb_mask_5210_no_rev[] = {
	0x0000FFFF,
	DDB_END
};

static const struct panel_reg id_regs[] = {
	{CMD_END, 0, 1, {0} }
};

static struct panel_controller r61529_controller_panel = {
	.off_to_standby			= off_to_standby_cmds,
	.deep_standby_to_standby	= deep_standby_to_standby_cmds,
	.standby_to_off			= standby_to_off_cmds,
	.standby_to_deep_standby	= standby_to_deep_standby_cmds,
	.on_to_standby			= on_to_standby_cmds,
	.standby_to_intermediate	= standby_to_intermediate_cmds,
	.intermediate_to_on		= intermediate_to_on_cmds,
};

static int custom_interface_init(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct panel_device *dev = container_of(ddev, struct panel_device,
									base);
	struct mcde_port *port = ddev->port;
	struct mcde_video_mode *vmode = &ddev->video_mode;

	dev_dbg(&dev->base.dev, "%s\n", __func__);

	port->phy.dsi.num_data_lanes = 1;

	/* pixclock = (bitclock*num_lanes)/(8*bpp) */
	/* Bitclock = 420160000 MHz (DSI_HS_FREQ_HZ)*/
	vmode->pixclock = 74963;

	/* Just for the CTS test case about frame frequency  */
	/* Pixclock [Hz] = fps * (xres + left_margin + right_margin) */
	/*                     * (yres + lower_margin + upper_margin)*/
	/* Just transfer lower margin to upper layer, */
	/* Assume left_margin, right_margin, upper_margin is 0*/
	/* fps=65Hz */
	vmode->vfp = 161;

	return ret;
}

const struct panel r61529_dsi_hitachi_5210_no_rev = {
	.name = "Hitachi DSI R61529 5210 no rev",
	.ddb = ddb_5210_no_rev,
	.ddb_mask = ddb_mask_5210_no_rev,
	.id_regs = id_regs,
	.pinfo = &r61529_controller_panel,
	.x_res = 320,
	.y_res = 480,
	.width = 45,
	.height = 67,
	.custom_interface_init = custom_interface_init,
	.custom_interface_remove = NULL,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MCDE DSI Hitachi TX09D115VM display driver");
