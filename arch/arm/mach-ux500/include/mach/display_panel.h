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
#ifndef _DISPLAY_PANEL_H_
#define _DISPLAY_PANEL_H_

#include <video/mcde_display-panel_dsi.h>

extern struct panel_platform_data panel_display0_pdata;

#ifdef CONFIG_DISPLAY_PANEL_DSI_SECONDARY
extern struct panel_platform_data panel_display1_pdata;
#endif /* CONFIG_DISPLAY_PANEL_DSI_SECONDARY */

#endif
