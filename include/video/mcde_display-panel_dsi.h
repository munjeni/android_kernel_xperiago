/* include/video/mcde_display-panel_dsi.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Sony Ericsson DSI header file
 *
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _MCDE_DISPLAY_PANEL_DSI_H_
#define _MCDE_DISPLAY_PANEL_DSI_H_

#include <linux/regulator/consumer.h>

#include "mcde_display.h"
#include <linux/workqueue.h>

#define DISPLAY_DDB_LENGTH	14

#define MCDE_DISPLAY_PANEL_NAME "mcde_display_panel"

struct panel_record {
	const struct panel *panel;
	struct mutex lock;
	u32 ddb_id;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dir;
#endif
};

enum cmd_type {
	CMD_END,
	CMD_WAIT_MS,
	CMD_GEN,
	CMD_DCS,
	CMD_RESET,
	CMD_PLATFORM
};

struct panel_reg {
	enum cmd_type	type;
	u8		addr;
	int		len;
	u8		value[MCDE_MAX_DSI_DIRECT_CMD_WRITE];
};

#define DDB_EXIT (0xFF)
#define DDB_END (0xFFFFFFFF)

struct panel_controller {
	const struct panel_reg *off_to_standby;
	const struct panel_reg *deep_standby_to_standby;
	const struct panel_reg *standby_to_off;
	const struct panel_reg *standby_to_deep_standby;
	const struct panel_reg *on_to_standby;
	const struct panel_reg *standby_to_on;
	const struct panel_reg *standby_to_intermediate;
	const struct panel_reg *intermediate_to_on;
};

struct panel {
	u32				panel_id;
	const char			*name;
	const u32			*ddb;
	const u32			*ddb_mask;
	const struct panel_reg		*id_regs;
	struct panel_controller		*pinfo;
	const u32			x_res;
	const u32			y_res;
	const u32			width;	/* in mm */
	const u32			height;	/* in mm */
	const s32			ddr_qos_value;
	const s32			ape_qos_value;
	const bool			disable_ulpm;
	const u8			addr_mode_normal;
	const u8			addr_mode_hor_flip;
	int (*custom_interface_init)(struct mcde_display_device *ddev);
	int (*custom_interface_remove)(struct mcde_display_device *ddev);
};

struct panel_platform_data {
	/* Platform info */
	int reset_gpio;
	bool reset_high;
	const char *regulator_id;
	const char *io_regulator_id;
	bool skip_init;
	bool ddr_is_requested;
	bool ape_is_requested;

	/* Driver data */
	int max_supply_voltage;
	int min_supply_voltage;

	const struct panel **panels;
};

struct panel_device {
	struct mcde_display_device base;
	struct regulator *regulator;
	struct regulator *io_regulator;
	struct delayed_work ddr_ape_timeout_work;
};

#ifdef CONFIG_MCDE_DISPLAY_NT35560_DSI_SONY_ACX424AKM
extern const struct panel nt35560_dsi_sony_acx424akm_rev018103;
extern const struct panel nt35560_dsi_sony_acx424akm_rev01811A;
extern const struct panel nt35560_dsi_sony_acx424akm_rev01811B;
extern const struct panel nt35560_dsi_sony_acx424akm_12529607_no_rev;
#endif
#ifdef CONFIG_MCDE_DISPLAY_NT35560_DSI_HITACHI_DX09D09VM
extern const struct panel nt35560_dsi_hitachi_dx09d09vm_rev125541211A;
extern const struct panel nt35560_dsi_hitachi_dx09d09vm_rev01821C;
extern const struct panel nt35560_dsi_hitachi_dx09d09vm_12529606_no_rev;
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61408_DSI_SEIKO_RB443
extern const struct panel r61408_dsi_seiko_9387_no_rev;
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61529_DSI_SEIKO_RJ248
extern const struct panel r61529_dsi_seiko_rev35721A;
extern const struct panel r61529_dsi_seiko_alternative_rev35721A;
extern const struct panel r61529_dsi_seiko_rev35721B;
extern const struct panel r61529_dsi_seiko_rev35721C;
extern const struct panel r61529_dsi_seiko_rev35721D;
extern const struct panel r61529_dsi_seiko_rev357201;
#endif

#ifdef CONFIG_MCDE_DISPLAY_R61529_DSI_HITACHI_TX09D115VM
extern const struct panel r61529_dsi_hitachi_5210_no_rev;
#endif

#ifdef CONFIG_MCDE_DISPLAY_OTM9601_DSI_SONY_ACX438AKM
extern const struct panel otm9601_dsi_sony_acx438akm_rev23841A01;
extern const struct panel otm9601_dsi_sony_acx438akm_2384_no_rev;
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61408_DSI_HITACHI_TX09D113
extern const struct panel r61408_dsi_hitachi_12614433_no_rev;
#endif

#ifdef CONFIG_MCDE_DISPLAY_R61408_DSI_SONY_L5F31178
extern const struct panel r61408_dsi_sony_9415_no_rev;
#endif

#ifdef CONFIG_MCDE_DISPLAY_R61408_DSI_SAMSUNG_LMS347TF01
extern const struct panel r61408_dsi_samsung_0425_no_rev;
#endif

#ifdef CONFIG_DEBUG_FS
extern void panel_create_debugfs(struct mcde_display_device *ddev);
extern void panel_remove_debugfs(struct mcde_display_device *ddev);
int panel_execute_cmd_extern(struct panel_device *dev,
						const struct panel_reg *preg);
#endif

#endif
