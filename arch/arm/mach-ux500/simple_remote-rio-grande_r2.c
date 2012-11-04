/*
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/gpio.h>
#include <mach/simple_remote_ux500_pf.h>

#define PLUG_DET_READ_PIN 96

static int simple_remote_pf_initialize_gpio(
	struct simple_remote_platform_data *data)
{
	int err = 0;

	if (!data) {
		pr_err("*** %s - Error: Invalid inparameter "
		       "(null pointer). Aborting!\n", __func__);
		return -EINVAL;
	}

	err = gpio_request(data->headset_detect_read_pin,
			   "Simple_remote_plug_detect_read");
	if (err) {
		pr_err("%s - Error %d - Request hs-detect_read pin",
		       __func__, err);
		goto out;
	}

	err = gpio_direction_input(data->headset_detect_read_pin);
	if (err) {
		pr_err("%s - Error %d - Set hs-detect pin as input\n",
		       __func__, err);
		goto out_hs_det_read;
	}
	return err;

out_hs_det_read:
	gpio_free(data->headset_detect_read_pin);
out:
	return err;
}

static void simple_remote_pf_deinitialize_gpio(
	struct simple_remote_platform_data *data)
{
	gpio_free(data->headset_detect_read_pin);
}

struct simple_remote_platform_data simple_remote_pf_data = {
	.headset_detect_enable_pin = -1,
	.headset_detect_read_pin = PLUG_DET_READ_PIN,
	.headset_mode_switch_pin = -1,
	.initialize = &simple_remote_pf_initialize_gpio,
	.deinitialize = &simple_remote_pf_deinitialize_gpio,

#ifdef CONFIG_SIMPLE_REMOTE_INVERT_PLUG_DETECTION_STATE
	.invert_plug_det = 1,
#else
	.invert_plug_det = 0,
#endif
};
