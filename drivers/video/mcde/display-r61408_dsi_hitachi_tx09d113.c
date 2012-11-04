/* drivers/video/mcde/display-r61408_dsi_hitachi_tx09d113.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Sony Ericsson Renesas 61408 driver for Hitachi panel
 *
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <video/mcde_display-panel_dsi.h>

static const struct panel_reg off_to_standby_cmds[] = {
	/* Spec: Power On */
	{CMD_PLATFORM, 0, 1, {1} },
	{CMD_RESET,    0, 1, {0} },
	{CMD_WAIT_MS,  0, 1, {2} }, /* Spec says > 1 ms */
	{CMD_RESET,    0, 1, {1} },
	{CMD_WAIT_MS,  0, 1, {6} }, /* Spec says > 5 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg deep_standby_to_standby_cmds[] = {
	/* Sequence not in spec */
	{CMD_RESET,   0, 1,  {0} },
	{CMD_WAIT_MS, 0, 1,  {2} }, /* Spec says > 1 ms */
	{CMD_RESET,   0, 1, {1} },
	{CMD_WAIT_MS, 0, 1, {6} }, /* Spec says > 5 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_off_cmds[] = {
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_deep_standby_cmds[] = {
	{CMD_GEN, 0xB0, 3, {0x04, 0x00, 0x00} },
	{CMD_GEN, 0xB1, 3, {0x01, 0x00, 0x00} },
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg on_to_standby_cmds[] = {
	/* LEDPWM */
	/* LEDPWM - Manufacturer command protect */
	{CMD_GEN, 0xB0, 3, {0x04, 0x00, 0x00} },
	/* LEDPWM - BLC function OFF */
	{CMD_GEN, 0xB8, 3, {0x00, 0x00, 0x00} },
	/* LEDPWM - Manufacturer command protect end */
	{CMD_GEN, 0xB0, 3, {0x03, 0x00, 0x00} },
	{CMD_DCS, DCS_CMD_SET_DISPLAY_OFF, 1, {0} },
	{CMD_DCS, DCS_CMD_ENTER_SLEEP_MODE, 1, {0} },
	{CMD_WAIT_MS, 0, 1, {73} }, /* Spec says > 72 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_intermediate_cmds[] = {
	/* Spec: Exit Sleep */
	{CMD_DCS, DCS_CMD_EXIT_SLEEP_MODE, 1,  {0} },
	{CMD_WAIT_MS, 0, 1, {127} }, /* Spec says > 126 ms */
	/* LEDPWM */
	/* LEDPWM - Manufacturer command protect */
	{CMD_GEN, 0xB0, 3, {0x04, 0x00, 0x00} },
	/* LEDPWM - BLC20 function ON */
	{CMD_GEN, 0xB8, 11, {0x01, 0x09, 0x09, 0xFF, 0xFF, 0xD8,
				0xD8, 0x02, 0x18, 0x90, 0x90} },
	{CMD_DCS, 0xB9, 3, {0x00, 0xFF, 0x02} },
	/* LEDPWM - Manufacturer command protect end */
	{CMD_GEN, 0xB0, 3, {0x03, 0x00, 0x00} },
	/* TE */
	{CMD_DCS, 0x35, 1, {0x00} }, /* TE on */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg intermediate_to_on_cmds[] = {
	/* Spec: Display On */
	{CMD_DCS, DCS_CMD_SET_DISPLAY_ON, 1,  {0} },
	{CMD_END, 0, 1, {0} }
};

static const u32 ddb_12614433_no_rev[] = {
	0x33446112,
	0x0000FF00,
	DDB_END
};

static const u32 ddb_mask_12614433_no_rev[] = {
	0xFFFFFFFF,
	0x0000FF00,
	DDB_END
};

static const struct panel_reg id_regs[] = {
	{CMD_END, 0, 1, {0} }
};

static struct panel_controller r61408_controller_panel = {
	.off_to_standby			= off_to_standby_cmds,
	.deep_standby_to_standby	= deep_standby_to_standby_cmds,
	.standby_to_off			= standby_to_off_cmds,
	.standby_to_deep_standby	= standby_to_deep_standby_cmds,
	.on_to_standby			= on_to_standby_cmds,
	.standby_to_intermediate	= standby_to_intermediate_cmds,
	.intermediate_to_on		= intermediate_to_on_cmds,
};

const struct panel r61408_dsi_hitachi_12614433_no_rev = {
	.name = "Hitachi tx09d113 MIPI/DSI 12614433 no rev",
	.ddb = ddb_12614433_no_rev,
	.ddb_mask = ddb_mask_12614433_no_rev,
	.id_regs = id_regs,
	.pinfo = &r61408_controller_panel,
	.x_res = 480,
	.y_res = 854,
	.width = 43,
	.height = 77,
	.custom_interface_init = NULL,
	.custom_interface_remove = NULL,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MCDE DSI Hitachi tx09d113 display driver");
