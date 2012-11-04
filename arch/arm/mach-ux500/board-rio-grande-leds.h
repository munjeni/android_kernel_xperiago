/* /kernel/arch/arm/mach-ux500/board-rio-grande-leds.h
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _BOARD_RIO_GRANDE_LEDS_H
#define _BOARD_RIO_GRANDE_LEDS_H

#ifdef CONFIG_LEDS_AS3677
extern struct as3677_platform_data as3677_pdata;
#endif

#if defined(CONFIG_LEDS_AS3676) || defined(CONFIG_LEDS_AS3676_VENDOR)
extern struct as3676_platform_data as3676_platform_data;
#endif

#endif

