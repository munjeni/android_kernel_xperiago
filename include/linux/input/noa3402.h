/* kernel/include/linux/input/noa3402.h
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 *         Louis Benoit <louis.benoit@onsemi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __NOA3402_H__
#define __NOA3402_H__

#define NOA3402_NAME "noa3402"

#ifdef __KERNEL__

#include <linux/regulator/consumer.h>
#include <linux/earlysuspend.h>

#define PWM_SENSITIVITY_STD	0
#define PWM_SENSITIVITY_HALF	(1 << 5)
#define PWM_SENSITIVITY_QUARTER	(1 << 6)
#define PWM_SENSITIVITY_EIGHTH	((1 << 5) | (1 << 6))

#define PWM_RES_7_BIT	0
#define PWM_RES_8_BIT	(1 << 3)
#define PWM_RES_9_BIT	(1 << 4)
#define PWM_RES_10_BIT	((1 << 3) | (1 << 4))

#define PWM_TYPE_LINEAR	0
#define PWM_TYPE_LOG	(1 << 2)

#define LED_CURRENT_MA_TO_REG(ma) ((ma - 5) / 5)	/* milliampere */

#define PS_INTEGRATION_150_US	0			/* 75us for Rev C */
#define PS_INTEGRATION_300_US	(1 << 0)		/* 150us for Rev C */
#define PS_INTEGRATION_600_US	(1 << 1)		/* 300us for Rev C */
#define PS_INTEGRATION_1200_US	(1 << 0 | 1 << 1)	/* 600us for Rev C */

#define PS_INTERVAL_MS_TO_REG(ms) ((ms - 5) / 5)

#define ALS_INTEGRATION_6_25_MS	0
#define ALS_INTEGRATION_12_5_MS	(1 << 0)
#define ALS_INTEGRATION_25_MS	(1 << 1)
#define ALS_INTEGRATION_50_MS	(1 << 0 | 1 << 1)
#define ALS_INTEGRATION_100_MS	(1 << 2)
#define ALS_INTEGRATION_200_MS	(1 << 0 | 1 << 2)
#define ALS_INTEGRATION_400_MS	(1 << 1 | 1 << 2)
#define ALS_INTEGRATION_800_MS	(1 << 0 | 1 << 1 | 1 << 2)

#define ALS_INTERVAL_MS_TO_REG(ms) (ms / 50)

enum noa_regs {
	PART_ID,
	RESET,
	INT_CONFIG,
	PWM_CONFIG,
	PS_LED_CURRENT,
	PS_TH_UP_MSB,
	PS_TH_UP_LSB,
	PS_TH_LO_MSB,
	PS_TH_LO_LSB,
	PS_FILTER_CONFIG,
	PS_CONFIG,
	PS_INTERVAL,
	PS_CONTROL,
	ALS_TH_UP_MSB,
	ALS_TH_UP_LSB,
	ALS_TH_LO_MSB,
	ALS_TH_LO_LSB,
	ALS_FILTER_CONFIG,
	ALS_CONFIG,
	ALS_INTERVAL,
	ALS_CONTROL,
	INTERRUPT,
	PS_DATA_MSB,
	PS_DATA_LSB,
	ALS_DATA_MSB,
	ALS_DATA_LSB,
	NBR_REGS
};

enum active_status {
	ALS_POWER = (1 << 0),
	PS_POWER = (1 << 1),
};

struct noa3402_chip {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct dentry			*dir;
	struct mutex			mutex;
	u8				regs[NBR_REGS];
	int				irq;
	enum active_status		active;
	struct regulator		*noa_regulator;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend		early_suspend;
#endif
};

struct noa3402_platform_data {
	u16 gpio;
	const char *regulator_id;
	u8 pwm_sensitivity;
	u8 pwm_res;
	u8 pwm_type;
	u8 ps_led_current;
	u8 ps_filter_nbr_correct;
	u8 ps_filter_nbr_measurements;
	u8 ps_integration_time;
	u8 ps_interval;
	u8 als_integration_time;
	u8 als_interval;
	unsigned int is_irq_wakeup;
	char *phys_dev_path;
	int (*pwr_enable)(struct noa3402_chip *chip, bool enable);
	int (*hw_setup)(struct noa3402_chip *chip, bool enable);
};
#endif

#endif
