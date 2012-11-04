/*
 * Copyright (C) 2010 ST-Ericsson SA
 *
 * License terms:GNU General Public License (GPL) version 2
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>

#include <linux/ste_timed_vibra.h>
#include <sound/ux500_ab8500_ext.h>

/* For details check ste_timed_vibra docbook */
static struct ste_timed_vibra_platform_data rotary_vibra_plat_data = {
	.is_linear_vibra = false,
	.reverse_polarity = true,
	.boost_level    = 100,
	.boost_time     = 40,
	.on_level       = 65,
	.off_level      = 100,
	.off_time       = 25,
	.timed_vibra_control = ux500_ab8500_audio_pwm_vibra,
};

/* Timed output vibrator device */
static struct platform_device ux500_vibra_device = {
	.name = "ste_timed_output_vibra",
};

void __init mop500_vibra_init(void)
{
	int ret;

	ux500_vibra_device.dev.platform_data = &rotary_vibra_plat_data;

	ret = platform_device_register(&ux500_vibra_device);
	if (ret < 0)
		pr_err("vibra dev register failed");
}
