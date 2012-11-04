/* /kernel/arch/arm/mach-ux500/input-lotus.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include "board-rio-grande-keypad.h"
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <plat/pincfg.h>
#include "pins.h"
#include <linux/cyttsp.h>

#define CYTTSP_IRQ_GPIO 88

#ifdef CONFIG_KEYBOARD_NOMADIK_SKE

static const unsigned int keymap[] = {
	KEY(1, 6, KEY_VOLUMEDOWN),

	KEY(2, 6, KEY_VOLUMEUP),
};

static struct matrix_keymap_data keymap_data = {
	.keymap		= keymap,
	.keymap_size	= ARRAY_SIZE(keymap),
};

/*
 * Nomadik SKE keypad
 */
#define ROW_PIN_I0	164
#define ROW_PIN_I1	163
#define ROW_PIN_I2	162
#define ROW_PIN_I3	161
#define ROW_PIN_I4	156
#define ROW_PIN_I5	155
#define ROW_PIN_I6	154
#define ROW_PIN_I7	153

#define COL_PIN_O0	168
#define COL_PIN_O1	167
#define COL_PIN_O2	166
#define COL_PIN_O3	165
#define COL_PIN_O4	160
#define COL_PIN_O5	159
#define COL_PIN_O6	158
#define COL_PIN_O7	157

#define SKE_KPD_MAX_ROWS	8
#define SKE_KPD_MAX_COLS	8

static int ske_kp_rows[] = {
	ROW_PIN_I0, ROW_PIN_I1, ROW_PIN_I2, ROW_PIN_I3,
	ROW_PIN_I4, ROW_PIN_I5, ROW_PIN_I6, ROW_PIN_I7
};

static int ske_kp_cols[] = {
	COL_PIN_O0, COL_PIN_O1, COL_PIN_O2, COL_PIN_O3,
	COL_PIN_O4, COL_PIN_O5, COL_PIN_O6, COL_PIN_O7,
};

static bool ske_config;
/*
 * ske_set_gpio_row: request and set gpio rows
 */
static int ske_set_gpio_row(int gpio)
{
	int ret;
	if (!ske_config) {
		ret = gpio_request(gpio, "ske-kp");
		if (ret < 0) {
			pr_err("ske_set_gpio_row: gpio request failed\n");
			return ret;
		}

	}

	ret = gpio_direction_output(gpio, 1);
	if (ret < 0) {
		pr_err("ske_set_gpio_row: gpio direction failed\n");
		gpio_free(gpio);
	}

	return ret;
}

/*
 * ske_kp_init - enable the gpio configuration
 */
static int ske_kp_init(void)
{
	struct ux500_pins *pins;
	int ret, i;

	pins = ux500_pins_get("ske");
	if (pins)
		ux500_pins_enable(pins);

	for (i = 0; i < SKE_KPD_MAX_ROWS; i++) {
		ret = ske_set_gpio_row(ske_kp_rows[i]);
		if (ret < 0) {
			pr_err("ske_kp_init: failed init\n");
			return ret;
		}
	}
	if (!ske_config)
		ske_config = true;

	return 0;
}

static int ske_kp_exit(void)
{
	struct ux500_pins *pins;

	pins = ux500_pins_get("ske");
	if (pins)
		ux500_pins_disable(pins);

	return 0;
}

static struct ske_keypad_platform_data pdp_ske_keypad_data = {
	.init		= ske_kp_init,
	.exit	   = ske_kp_exit,
	.gpio_input_pins = ske_kp_rows,
	.gpio_output_pins = ske_kp_cols,
	.keymap_data	= &keymap_data,
	.no_autorepeat	= true,
	.krow		= SKE_KPD_MAX_ROWS,	/* 8x8 matrix */
	.kcol		= SKE_KPD_MAX_COLS,
	.debounce_ms	= 20,		   /* in timeout period */
	.switch_delay	= 200,		  /* in jiffies */
};

struct ske_keypad_platform_data *get_ske_keypad_data(void)
{
	return &pdp_ske_keypad_data;
}

#endif /* CONFIG_KEYBOARD_NOMADIK_SKE */

struct cyttsp_platform_data cyttsp_data = {
	.maxx = 320,
	.maxy = 480,
	.maxz = 100,
	.flags = 0,			/* REVERSE_X_FLAG, REVERSE_Y_FLAG, or
					   REVERSE_X_FLAG | REVERSE_Y_FLAG */
	.gen = CY_GEN3,
	.use_st = 0,
	.use_mt = 1,
	.mt_sync = input_mt_sync,
	.use_trk_id = 1,
	.use_hndshk = 0,
	.use_sleep = 1,
	.use_gestures = 0,
	.use_load_file = 0,
	.use_force_fw_update = 0,
	.use_charger_mode = 1,
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 |
			CY_GEST_GRP3 | CY_GEST_GRP4 | CY_ACT_DIST,
	.act_intrvl = 16,	/* 16 ms corresponds to 62,5 Hz */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.name = CY_SPI_NAME,
	.irq_gpio = CYTTSP_IRQ_GPIO,
};


