/* kernel/arch/arm/mach-ux500/include/mach/simple_remote_ux500_pf.h
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Authors: Joachim Holst <joachim.holst@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef LINUX_SIMPLE_REMOTE_UX500_PLATFORM
#define LINUX_SIMPLE_REMOTE_UX500_PLATFORM

#include <linux/mfd/ab8500.h>

#define SIMPLE_REMOTE_PF_NAME "simple_remote_pf"

struct simple_remote_platform_data {
	unsigned int headset_detect_enable_pin;
	unsigned int headset_detect_read_pin;
	int headset_mode_switch_pin;  /* Set to -1 if not supported by HW */

	int invert_plug_det; /* Default is 0 = plug inserted */

	int (*initialize)(struct simple_remote_platform_data *);
	void (*deinitialize)(struct simple_remote_platform_data *);
};

void simple_remote_pf_button_handler(uint32_t key, uint32_t event);

extern struct simple_remote_platform_data simple_remote_pf_data;
#endif /* LINUX_SIMPLE_REMOTE_UX500_PLATFORM */
