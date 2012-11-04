/*
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
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
#ifdef CONFIG_MCDE_DISPLAY_OTM9601_DSI_SONY_ACX438AKM
	&otm9601_dsi_sony_acx438akm_rev23841A01,
	&otm9601_dsi_sony_acx438akm_2384_no_rev,
#endif
	NULL,
};

struct panel_platform_data panel_display0_pdata = {
	.reset_gpio = 21,
	.skip_init = false,
	.ddr_is_requested = false,
#ifdef CONFIG_REGULATOR
	.regulator_id = "v-display-3v1",
	.min_supply_voltage = 0, /* To make sure we don't change the voltage */
	.max_supply_voltage = 0, /* To make sure we don't change the voltage */
	.io_regulator_id = "vio-display",
#endif
	.panels = panels,
};
