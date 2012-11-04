/* kernel/arch/arm/mach-ux500/leds-lotus.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifdef CONFIG_LEDS_AS3677

#include <linux/leds.h>
#include <linux/leds-as3677.h>

struct as3677_platform_data as3677_pdata = {
	.step_up_vmax = VMAX_25,
	.dls_analog = true,
	.leds[0] = {
		.name = "lcd-backlight",
		.on_charge_pump = 0,
		.max_current_uA = 20000,
		.startup_current_uA = 20000,
	},
	.leds[2] = {
		.name = "button-backlight",
		.on_charge_pump = 1,
		.max_current_uA = 5000,
	},
	.leds[3] = {
		.name = "red",
		.on_charge_pump = 1,
		.max_current_uA = 5000,
	},
	.leds[4] = {
		.name = "green",
		.on_charge_pump = 1,
		.max_current_uA = 5000,
	},
	.leds[5] = {
		.name = "blue",
		.on_charge_pump = 1,
		.max_current_uA = 5000,
	}
};
#endif

