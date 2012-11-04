/*
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
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

static const struct panel *panels[] = {
#ifdef CONFIG_MCDE_DISPLAY_NT35560_DSI_SONY_ACX424AKM
	&nt35560_dsi_sony_acx424akm_12529607_no_rev,
	&nt35560_dsi_sony_acx424akm_rev018103,
	&nt35560_dsi_sony_acx424akm_rev01811A,
	&nt35560_dsi_sony_acx424akm_rev01811B,
#endif
#ifdef CONFIG_MCDE_DISPLAY_NT35560_DSI_HITACHI_DX09D09VM
	&nt35560_dsi_hitachi_dx09d09vm_12529606_no_rev,
	&nt35560_dsi_hitachi_dx09d09vm_rev01821C,
	&nt35560_dsi_hitachi_dx09d09vm_rev125541211A,
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61408_DSI_SEIKO_RB443
	&r61408_dsi_seiko_9387_no_rev,
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61529_DSI_SEIKO_RJ248
	&r61529_dsi_seiko_rev35721A,
	&r61529_dsi_seiko_alternative_rev35721A,
	&r61529_dsi_seiko_rev35721B,
	&r61529_dsi_seiko_rev35721C,
	&r61529_dsi_seiko_rev35721D,
	&r61529_dsi_seiko_rev357201,
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61529_DSI_HITACHI_TX09D115VM
	&r61529_dsi_hitachi_5210_no_rev,
#endif
#ifdef CONFIG_MCDE_DISPLAY_OTM9601_DSI_SONY_ACX438AKM
	&otm9601_dsi_sony_acx438akm_rev23841A01,
	&otm9601_dsi_sony_acx438akm_2384_no_rev,
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61408_DSI_HITACHI_TX09D113
	&r61408_dsi_hitachi_12614433_no_rev,
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61408_DSI_SONY_L5F31178
	&r61408_dsi_sony_9415_no_rev,
#endif
#ifdef CONFIG_MCDE_DISPLAY_R61408_DSI_SAMSUNG_LMS347TF01
	&r61408_dsi_samsung_0425_no_rev,
#endif
	NULL,
};

struct panel_platform_data panel_display0_pdata = {
	.reset_gpio = 21,
	.skip_init = false,
	.ddr_is_requested = false,
#ifdef CONFIG_REGULATOR
	.regulator_id = "v-display",
	.min_supply_voltage = 2800000, /* 2.8V */
	.max_supply_voltage = 2800000, /* 2.8V */
	.io_regulator_id = "vio-display",
#endif
	.panels = panels,
};
