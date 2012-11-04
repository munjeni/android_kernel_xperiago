/* kernel/arch/arm/mach-ux500/charger-kumquat.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Sun Yi <yi3.sun@sonyericsson.com>
 * Author: Sergii Kriachko <Sergii.Kriachko@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/mfd/abx500/ab8500-bm.h>

struct device_data device_data = {
	.charge_full_design = 1290, /* C */
	.normal_cur_lvl = 903, /* 0.7C */
	.termination_curr = 64, /* C/20 */
	.lowbat_threshold = 3350,
	.lowbat_hysteresis = 50,
	.b_chem = TYPE1_BAT_CURVE_LAUREL,
};
