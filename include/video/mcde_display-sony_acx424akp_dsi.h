/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson MCDE Sony acx424akp DCS display driver
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
#ifndef __MCDE_DISPLAY_SONY_ACX424AKP__H__
#define __MCDE_DISPLAY_SONY_ACX424AKP__H__

enum display_panel_type {
	DISPLAY_NONE			= 0,
	DISPLAY_SONY_ACX424AKP          = 0x1b81,
	DISPLAY_SONY_ACX424AKP_ID2      = 0x1a81,
	DISPLAY_SONY_ACX424AKP_ID3      = 0x0080,
};

struct  mcde_display_sony_acx424akp_platform_data {
	/* Platform info */
	int reset_gpio;
	enum display_panel_type disp_panel; /* display panel types */
};

#endif /* __MCDE_DISPLAY_SONY_ACX424AKP__H__ */

