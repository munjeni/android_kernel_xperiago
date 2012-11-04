/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson MCDE generic DCS display driver
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
#ifndef __MCDE_DISPLAY_GENERIC__H__
#define __MCDE_DISPLAY_GENERIC__H__

#include <linux/regulator/consumer.h>

#include "mcde_display.h"

struct mcde_display_generic_platform_data {
	/* Platform info */
	int reset_gpio;
	bool reset_high;
	const char *regulator_id;
	const char *io_regulator_id;
	int reset_delay; /* ms */
	int sleep_out_delay; /* ms */
	u32 ddb_id;

	/* Driver data */
	bool generic_platform_enable;
	struct regulator *regulator;
	int max_supply_voltage;
	int min_supply_voltage;
};

#endif /* __MCDE_DISPLAY_GENERIC__H__ */

