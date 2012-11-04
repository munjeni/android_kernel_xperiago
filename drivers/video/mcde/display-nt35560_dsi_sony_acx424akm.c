/* drivers/video/mcde/display-nt35560_dsi_sony_acx424akm.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Sony Ericsson Novatek 35560 driver for Sony panel
 *
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
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
	{CMD_RESET,    0, 1,  {1} },
	{CMD_WAIT_MS,  0, 1, {11} }, /* Spec says > 10 ms */
	{CMD_RESET,    0, 1,  {0} },
	{CMD_WAIT_MS,  0, 1,  {1} }, /* Spec says > 20 us */
	{CMD_RESET,    0, 1,  {1} },
	{CMD_WAIT_MS,  0, 1, {11} }, /* Spec says > 10 ms */
	{CMD_END,      0, 1,  {0} }
};

static const struct panel_reg deep_standby_to_standby_cmds[] = {
	/* Spec: G) Exit Deep Standby */
	{CMD_RESET,   0, 1, {0} },
	{CMD_WAIT_MS, 0, 1, {4} }, /* Spec says > 3 ms */
	{CMD_RESET,   0, 1, {1} },
	{CMD_END,     0, 1, {0} }
};

static const struct panel_reg standby_to_off_cmds[] = {
	/* Spec: Power off */
	{CMD_RESET,    0, 1,  {0} },
	{CMD_WAIT_MS,  0, 1, {11} }, /* Spec says > 10 ms */
	{CMD_PLATFORM, 0, 1,  {0} },
	{CMD_END,      0, 1,  {0} }
};

static const struct panel_reg standby_to_deep_standby_cmds[] = {
	/* Spec: F) Deep Standby Set */
	{CMD_DCS, 0x4F, 1, {0x01} },
	{CMD_END,    0, 1, {0} }
};

static const struct panel_reg on_to_standby_cmds[] = {
	/* Spec: E) Sleep Set */
	{CMD_DCS, DCS_CMD_ENTER_SLEEP_MODE, 0, {0} },
	/* Spec: LED PWM on -> off */
	{CMD_DCS, 0x55, 1, {0x00} },
	{CMD_DCS, 0x53, 1, {0x00} },
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_intermediate_cmds_018103[] = {
	/* Spec: A) Initialization */
	{CMD_WAIT_MS, 0, 1, {11} }, /* Spec says > 10 ms */
	/* Spec: LED PWM off -> on */
	{CMD_DCS, 0xF3, 1, {0xAA} },
	{CMD_GEN, 0x00, 3, {0x01, 0x00, 0x00} },
	{CMD_GEN, 0x7D, 3, {0x01, 0x00, 0x00} },
	{CMD_GEN, 0x22, 3, {0x04, 0x00, 0x00} },
	{CMD_GEN, 0x7F, 3, {0xAA, 0x00, 0x00} },
	{CMD_DCS, 0x51, 1, {0xFF} },
	{CMD_DCS, 0x5E, 1, {0x99} },
	{CMD_DCS, 0x53, 1, {0x2C} },
	{CMD_DCS, 0x55, 1, {0x03} }, /* video mode */
	/* Set tear on */
	{CMD_DCS, 0x35, 1, {0x00} },
	/* Spec: B) Exit Sleep */
	{CMD_DCS, DCS_CMD_EXIT_SLEEP_MODE, 0, {0} },
	{CMD_WAIT_MS, 0, 1, {11} }, /* Spec says > 10 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg standby_to_intermediate_cmds[] = {
	/* Spec: A) Initialization */
	{CMD_WAIT_MS, 0, 1, {11} }, /* Spec says > 10 ms */
	/* Spec: LED PWM off -> on */
	{CMD_DCS, 0xF3, 1, {0xAA} },
	{CMD_GEN, 0x00, 3, {0x01, 0x00, 0x00} },
	{CMD_GEN, 0x7D, 3, {0x01, 0x00, 0x00} },
	{CMD_GEN, 0x22, 3, {0x04, 0x00, 0x00} },
	{CMD_GEN, 0x7F, 3, {0xAA, 0x00, 0x00} },
	{CMD_DCS, 0x51, 1, {0xFF} },
	{CMD_DCS, 0x5E, 1, {0x99} },
	{CMD_DCS, 0x53, 1, {0x2C} },
	{CMD_DCS, 0x55, 1, {0x01} },
	/* Set tear on */
	{CMD_DCS, 0x35, 1, {0x00} },
#ifdef CONFIG_MCDE_DISPLAY_ROTATE_NT35560_DSI_SONY_ACX424AKM
	/* Rotate screen content */
	{CMD_DCS, 0xF3, 1, {0xAA} },
	{CMD_GEN, 0x00, 3, {0x00, 0x00, 0x00} },
	{CMD_GEN, 0xC9, 3, {0x01, 0x00, 0x00} },
	{CMD_GEN, 0xA2, 3, {0x03, 0x00, 0x00} },
	{CMD_GEN, 0xFF, 3, {0xAA, 0x00, 0x00} },
#else
	/* unrotate screen content */
	{CMD_DCS, 0xF3, 1, {0xAA} },
	{CMD_GEN, 0x00, 3, {0x00, 0x00, 0x00} },
	{CMD_GEN, 0xC9, 3, {0x01, 0x00, 0x00} },
	{CMD_GEN, 0xA2, 3, {0x00, 0x00, 0x00} },
	{CMD_GEN, 0xFF, 3, {0xAA, 0x00, 0x00} },
#endif
	/* Spec: B) Exit Sleep */
	{CMD_DCS, DCS_CMD_EXIT_SLEEP_MODE, 0, {0} },
	{CMD_WAIT_MS, 0, 1, {11} }, /* Spec says > 10 ms */
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg intermediate_to_on_cmds[] = {
	/* Spec: C) Display On */
	{CMD_DCS, DCS_CMD_SET_DISPLAY_ON, 0, {0} },
	{CMD_END, 0, 1, {0} }
};

static const u32 ddb_018103[] = {
	0x01242010,
	0x0000FF00,
	DDB_END
};

static const u32 ddb_mask_018103[] = {
	0xFFFFFFFF,
	0x0000FF00,
	DDB_END
};

static const u32 ddb_12529607_no_rev[] = {
	0x07965212,
	0x00FF0000,
	DDB_END
};

static const u32 ddb_mask_12529607_no_rev[] = {
	0xFFFFFFFF,
	0x00FF0000,
	DDB_END
};

static const struct panel_reg id_regs_018103[] = {
	{CMD_DCS, 0xDA, 1, {0x01} },
	{CMD_DCS, 0xDB, 1, {0x81} },
	{CMD_DCS, 0xDC, 1, {0x03} },
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg id_regs_01811A[] = {
	{CMD_DCS, 0xDA, 1, {0x01} },
	{CMD_DCS, 0xDB, 1, {0x81} },
	{CMD_DCS, 0xDC, 1, {0x1A} },
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg id_regs_01811B[] = {
	{CMD_DCS, 0xDA, 1, {0x01} },
	{CMD_DCS, 0xDB, 1, {0x81} },
	{CMD_DCS, 0xDC, 1, {0x1B} },
	{CMD_END, 0, 1, {0} }
};

static const struct panel_reg id_regs_dummy[] = {
	{CMD_END, 0, 1, {0} }
};

static struct panel_controller nt35560_controller_panel = {
	.off_to_standby			= off_to_standby_cmds,
	.deep_standby_to_standby	= deep_standby_to_standby_cmds,
	.standby_to_off			= standby_to_off_cmds,
	.standby_to_deep_standby	= standby_to_deep_standby_cmds,
	.on_to_standby			= on_to_standby_cmds,
	.standby_to_intermediate	= standby_to_intermediate_cmds,
	.intermediate_to_on		= intermediate_to_on_cmds,
};

static struct panel_controller nt35560_controller_panel_018103 = {
	.off_to_standby			= off_to_standby_cmds,
	.deep_standby_to_standby	= deep_standby_to_standby_cmds,
	.standby_to_off			= standby_to_off_cmds,
	.standby_to_deep_standby	= standby_to_deep_standby_cmds,
	.on_to_standby			= on_to_standby_cmds,
	.standby_to_intermediate	= standby_to_intermediate_cmds_018103,
	.intermediate_to_on		= intermediate_to_on_cmds,
};

const struct panel nt35560_dsi_sony_acx424akm_rev018103 = {
	.name = "Sony DSI ACX424AKM rev018103",
	.ddb = ddb_018103,
	.ddb_mask = ddb_mask_018103,
	.id_regs = id_regs_018103,
	.pinfo = &nt35560_controller_panel_018103,
	.x_res = 480,
	.y_res = 854,
	.width = 46,
	.height = 82,
	.custom_interface_init = NULL,
	.custom_interface_remove = NULL,
};

const struct panel nt35560_dsi_sony_acx424akm_rev01811A = {
	.name = "Sony DSI ACX424AKM rev01811A",
	.ddb = NULL,
	.ddb_mask = NULL,
	.id_regs = id_regs_01811A,
	.pinfo = &nt35560_controller_panel,
	.x_res = 480,
	.y_res = 854,
	.width = 46,
	.height = 82,
	.custom_interface_init = NULL,
	.custom_interface_remove = NULL,
};

const struct panel nt35560_dsi_sony_acx424akm_rev01811B = {
	.name = "Sony DSI ACX424AKM rev01811B",
	.ddb = NULL,
	.ddb_mask = NULL,
	.id_regs = id_regs_01811B,
	.pinfo = &nt35560_controller_panel,
	.x_res = 480,
	.y_res = 854,
	.width = 46,
	.height = 82,
	.custom_interface_init = NULL,
	.custom_interface_remove = NULL,
};

const struct panel nt35560_dsi_sony_acx424akm_12529607_no_rev = {
	.name = "Sony ACX424AKM MIPI/DSI 12529607 no rev",
	.ddb = ddb_12529607_no_rev,
	.ddb_mask = ddb_mask_12529607_no_rev,
	.id_regs = id_regs_dummy,
	.pinfo = &nt35560_controller_panel,
	.x_res = 480,
	.y_res = 854,
	.width = 46,
	.height = 82,
	.custom_interface_init = NULL,
	.custom_interface_remove = NULL,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MCDE nt35560 Sony ACX424AKM DSI display driver");
