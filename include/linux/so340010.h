/*
 * so340010.h
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 */
#ifndef _SO340010_H_
#define _SO340010_H_

#define SO340010_DEV_NAME "so340010"

enum so340010_gpio_mode {
	SO340010_OUTPUT_LO,
	SO340010_OUTPUT_HI,
	SO340010_INPUT,
	SO340010_INPUT_PULLUP,
};

enum so340010_btn_mode {
	SO340010_INRESTRICTED = (0 << 4),
	SO340010_STRONGEST    = (2 << 4),
	SO340010_FIRT_ONLY    = (3 << 4),
};

enum so340010_btn_out_mode {
	SO340010_ACTIVE_HI      = (0x00 << 13),
	SO340010_ACTIVE_LO      = (0x01 << 13),
	SO340010_TOGGLE_TOUCH   = (0x02 << 13),
	SO340010_TOGGLE_LIFT    = (0x03 << 13),
	SO340010_RADIO_BTN_HIGH = (0x04 << 13),
	SO340010_RADIO_BTN_LOW  = (0x05 << 13),
};

enum so340010_btn_map2dir {
	SO340010_MAP2DIRECTION,
	SO340010_MAP2DATA,
};

enum so340010_led_effect {
	SO340010_RAMP_UP_DN       = 0x00,
	SO340010_RAMP_UP          = 0x02,
	SO340010_RAMP_DN          = 0x04,
	SO340010_CLIPPED_TRIANGLE = 0x06,
	SO340010_SQUARE_WAVE      = 0x07,
};

enum so340010_common {
	SO340010_PIN_NUM = 4,
	SO340010_BTN_NUM = 4,
	SO340010_NO_POLLING = -1,
};

enum so340010_suspend_mode {
	SO340010_SUSPEND_NEVER,
	SO340010_SUSPEND_EARLY,
	SO340010_SUSPEND_SYS,
};

enum so340010_pin_mode  {
	SO340010_GPIO,
	SO340010_LED,
};

enum so34_platform_power {
	SO34_PWR_ENABLED = 1 << 0,
};

struct so340010_btn_cfg {
	u8 enable;
	/*
	* sensitivity range is 0 - 255
	*/
	u8 sensitivity;
	int code;
	bool map2gpio;
	enum so340010_suspend_mode suspend;
};

struct so340010_led_cfg {
	/*
	* max LED level range is 0 - 31
	*/
	u8 max_level;
	enum so340010_led_effect effect;
	const char *name;
};

struct so340010_gpio_cfg {
	enum so340010_gpio_mode mode;
	void (*gpio_changed_cb)(int gpio_value);
};

struct so340010_pin_cfg {
	enum so340010_pin_mode mode;
	union {
		struct so340010_gpio_cfg   gpio;
		struct so340010_led_cfg    led;
	} u;
};

struct so340010_config {
	struct so340010_pin_cfg    pin[SO340010_PIN_NUM];
	struct so340010_btn_cfg    btn[SO340010_BTN_NUM];
	u8 period_a_x12_5_ms;
	u8 period_b_x12_5_ms;
	enum so340010_btn_out_mode btn_out_mode;
	enum so340010_btn_mode     btn_mode;
	enum so340010_btn_map2dir  btn_map2dir;
	enum so340010_suspend_mode gpi_suspend;
	/*
	* enables IRQ on GPIO input
	*/
	bool gpi_enable;
	/*
	* set polling to SO340010_NO_POLLING if interrupt is used
	*/
	int polling_ms;
	const char *input_name;
	bool enable_wake;
	int (*platform_setup)(struct device *dev);
	void (*platform_teardown)(struct device *dev);
	int (*platform_pwr)(struct device *dev,
		enum so34_platform_power request,
		enum so34_platform_power *result);
};
#endif
