/* drivers/input/misc/so340010.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/so340010.h>

#define NAME_LIMIT 128

#define ODD_LEDS_EFFECT_MASK   0x0e00
#define ODD_LEDS_EFFECT_SHIFT  13
#define EVEN_LEDS_EFFECT_MASK  0x00e0
#define EVEN_LEDS_EFFECT_SHIFT 5
#define ODD_LEDS_LEVEL_MASK    0x1f00
#define ODD_LEDS_LEVEL_SHIFT   8
#define EVEN_LEDS_LEVEL_MASK   0x001f
#define EVEN_LEDS_LEVEL_SHIFT  0
#define ODD_BTN_SENS_MASK      0xff00
#define ODD_BTN_SENS_SHIFT     8
#define EVEN_BTN_SENS_MASK     0x00ff
#define EVEN_BTN_SENS_SHIFT    0

#define GPIO_OUT_HIGH(n)  (0x0101 << n)
#define GPIO_OUT_LOW(n)   (0x0100 << n)
#define GPIO_IN_PULLUP(n) (0x0001 << n)
#define GPIO_IN_NOPULL(n) (0x0000 << n)



enum so34_reg {
	SO34_IF_CONFIG,
	SO34_G_CONFIG,
	SO34_BTN_ENA,
	SO34_GPIO_CTL,
	SO34_BTN_SNS_1,
	SO34_BTN_SNS_2,
	SO34_BTN_MAP,
	SO34_TIMER_CTL,
	SO34_LED_ENA,
	SO34_LED_PERIOD,
	SO34_LED_CTL_1,
	SO34_LED_CTL_2,

	SO34_GPIO_STATE,
	SO34_BTN_STATE,
	SO34_TIMER_STATE,
	SO34_PRESSURE_1,
	SO34_PRESSURE_2,

	SO34_CHIP_RESET,

	SO34_REG_NUM,
	SO34_RW_REG_NUM = (SO34_LED_CTL_2 - SO34_IF_CONFIG + 1),
	SO34_RO_REG_NUM = (SO34_PRESSURE_2 - SO34_GPIO_STATE + 1),
	SO34_RW_REG_LAST = SO34_LED_CTL_2,
};

static const u16 so34_map[] = {
	[SO34_IF_CONFIG]   = 0x0000,
	[SO34_G_CONFIG]    = 0x0001,
	[SO34_BTN_ENA]     = 0x0004,
	[SO34_GPIO_CTL]    = 0x000e,
	[SO34_BTN_SNS_1]   = 0x0010,
	[SO34_BTN_SNS_2]   = 0x0011,
	[SO34_BTN_MAP]     = 0x001e,
	[SO34_TIMER_CTL]   = 0x001f,
	[SO34_LED_ENA]     = 0x0022,
	[SO34_LED_PERIOD]  = 0x0023,
	[SO34_LED_CTL_1]   = 0x0024,
	[SO34_LED_CTL_2]   = 0x0025,

	[SO34_GPIO_STATE]  = 0x0108,
	[SO34_BTN_STATE]   = 0x0109,
	[SO34_TIMER_STATE] = 0x010b,
	[SO34_PRESSURE_1]  = 0x010c,
	[SO34_PRESSURE_2]  = 0x010d,

	[SO34_CHIP_RESET]  = 0x0300,
};

struct so34_reg_region {
	enum so34_reg first;
	u8 num;
};

enum so34_rr {
	SO34_RR_CONFIG,
	SO34_RR_BTN_ENA,
	SO34_RR_GPIO_CTL,
	SO34_RR_BTN_SNS,
	SO34_RR_BTN_MAP,
	SO34_RR_LED,
	SO34_RR_STATE,
	SO34_RR_TIMER,
};

static const struct so34_reg_region so34_reg_regions[] = {
	[SO34_RR_CONFIG]   = {SO34_IF_CONFIG,   2},
	[SO34_RR_BTN_ENA]  = {SO34_BTN_ENA,     1},
	[SO34_RR_GPIO_CTL] = {SO34_GPIO_CTL,    1},
	[SO34_RR_BTN_SNS]  = {SO34_BTN_SNS_1,   2},
	[SO34_RR_BTN_MAP]  = {SO34_BTN_MAP,     2},
	[SO34_RR_LED]      = {SO34_LED_ENA,     4},
	[SO34_RR_STATE]    = {SO34_GPIO_STATE,  2},
	[SO34_RR_TIMER]    = {SO34_TIMER_STATE, 3},
};

#define MAX_RR_SIZE 4

enum so34_bits {
	SO34_DATA_EN    = 1 << 2,
	SO34_IRQ_DI_LO  = 0,
	SO34_IRQ_DI_HI  = 1,
	SO34_IRQ_ENABLE = 3,
	SO34_SLEEP      = 1 << 7,
	SO34_GPI_IRQ    = 1 << 6,
	SO34_MAP2DIR    = 1 << 12,
	SO34_RESET      = 1,
	SO34_ALL_BTNS   = 0xf,
};

enum so34_pm_cmd {
	SO34_SUSPEND,
	SO34_RESUME,
	SO34_SUSPEND_EARLY,
	SO34_RESUME_LATE,
	SO34_SUSPEND_USER,
	SO34_RESUME_USER,
};

enum so34_pm_state {
	SO34_SYS_SUSPEND   = 1 << 0,
	SO34_EARLY_SUSPEND = 1 << 1,
};

struct so34_led_intf {
	u8 hw_led;
	struct led_classdev ldev;
};

#define LED_INTF_NUM SO340010_PIN_NUM

struct so34_btn_intf {
	u8  sens;
	int code;
};

#define BTN_INTF_NUM SO340010_BTN_NUM

enum hw_use {
	HW_LED0    = 1 << 0,
	HW_LED1    = 1 << 1,
	HW_LED2    = 1 << 2,
	HW_LED3    = 1 << 3,
	HW_GPI     = 1 << SO340010_PIN_NUM,
	HW_BUTTONS = 1 << (SO340010_PIN_NUM + 1),
	HW_I2C     = 1 << (SO340010_PIN_NUM + 2),
};

struct so34_data {
	struct i2c_client *client;
	struct input_dev *idev;
	struct work_struct gpio_cb_w;
	struct delayed_work poll_w;
	struct workqueue_struct *wq;
	struct mutex lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct so34_led_intf *led[LED_INTF_NUM];
	struct so34_btn_intf btn[BTN_INTF_NUM];

	u8 gpio_state;
	u8 toggled;

	u16 reg[SO34_REG_NUM];

	u8 btn_early_susp;
	u8 btn_sys_susp;
	u8 all_buttons;

	const struct so340010_config *pcfg;
	int led_num;
	int poll_jf;
	enum so34_pm_state pm_state;
	enum hw_use hw_in_use;
	enum so34_platform_power pltf_pwr;

	struct wake_lock ui_lock;
};

#define DEFAULT_POLL_MS 13

#define LOCK(p) do { \
	dev_vdbg(&(p)->client->dev, "%s: lock\n", __func__); \
	mutex_lock(&p->lock); \
} while (0)

#define UNLOCK(p) do { \
	dev_vdbg(&(p)->client->dev, "%s: unlock\n", __func__); \
	mutex_unlock(&p->lock); \
} while (0)

#ifdef CONFIG_HAS_WAKELOCK
#define LOCK_UI(p) do { \
	dev_vdbg(&(p)->client->dev, "%s: lock-ui\n", __func__); \
	wake_lock(&(p)->ui_lock); \
	mutex_lock(&p->lock); \
} while (0)

#define UNLOCK_UI(p) do { \
	dev_vdbg(&(p)->client->dev, "%s: unlock-ui\n", __func__); \
	mutex_unlock(&p->lock); \
	wake_unlock(&(p)->ui_lock); \
} while (0)
#else
#define LOCK_UI(p)   LOCK(p)
#define UNLOCK_UI(p) UNLOCK(p)
#endif

#define INIT_LOCK(p) mutex_init(&(p)->lock);

static void so34_dump(struct so34_data *d, const char *func)
{
#ifdef VERBOSE_DEBUG
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(d->reg); i++)
		dev_vdbg(&d->client->dev, "%s: reg: 0x%04x = 0x%04x\n",
			func, so34_map[i], d->reg[i]);
#endif
}

static int so34_write_region(struct so34_data *d, enum so34_rr rr)
{
	int rc;
	u16 b[MAX_RR_SIZE + 1];
	u8 num = so34_reg_regions[rr].num;
	u16 first = so34_reg_regions[rr].first;
	u8 size = (num + 1) * sizeof(d->reg[0]);

#ifdef __LITTLE_ENDIAN
	do {
		unsigned i;
		u16 *dest = &b[1];
		u16 *src = &d->reg[first];

		b[0] = swab16(so34_map[first]);
		for (i = 0; i < num; i++)
			*dest++ = swab16(*src++);
	} while (0);
#else
	b[0] = so34_map[first];
	memcpy(&b[1], &d->reg[first], num * sizeof(d->reg[0]));
#endif
	rc = i2c_master_send(d->client, (u8 *)b, size);
	if (rc < 0) {
		dev_err(&d->client->dev, "%s: failed (%d)\n", __func__, rc);
	} else if (rc != size) {
		dev_err(&d->client->dev, "%s: only %d of %d sent\n",
				__func__, rc, (num + 1) * sizeof(d->reg[0]));
		rc = -EIO;
	} else {
		rc = 0;
	}
	return rc;
}

static int so34_write(struct so34_data *d, enum so34_reg reg, u16 val)
{
	int rc;
	u16 b[2];

	if (val == d->reg[reg])
		return 0;
#ifdef __LITTLE_ENDIAN
	b[0] = swab16(so34_map[reg]);
	b[1] = swab16(val);
#else
	b[0] = so34_map[reg];
	b[1] = val;
#endif
	rc = i2c_master_send(d->client, (char *)b, sizeof(b));
	if (rc < 0) {
		dev_err(&d->client->dev, "%s: failed (%d)\n", __func__, rc);
	} else if (rc != sizeof(b)) {
		dev_err(&d->client->dev, "%s: only %d of %d sent\n",
				__func__, rc, sizeof(b));
		rc = -EIO;
	} else {
		d->reg[reg] = val;
		rc = 0;
	}
	return rc;
}

static int so34_read_region(struct so34_data *d, enum so34_rr rr)
{
	int rc;
	u8 num = so34_reg_regions[rr].num;
	u16 first = so34_reg_regions[rr].first;

#ifdef __LITTLE_ENDIAN
	u16 reg = swab16(so34_map[first]);
#else
	u16 reg = so34_map[first];
#endif

	rc = i2c_master_send(d->client, (char *)&reg, sizeof(reg));
	if (rc != sizeof(reg)) {
		dev_err(&d->client->dev, "%s: failed (%d)", __func__, rc);
		if (rc > 0)
			return -EIO;
	}

	rc = i2c_master_recv(d->client, (u8 *)&d->reg[first],
				num * sizeof(d->reg[0]));
	if (rc < 0) {
		dev_err(&d->client->dev, "%s: filed (%d)\n", __func__, rc);
	} else if (rc != num * sizeof(d->reg[0])) {
		dev_err(&d->client->dev, "%s: only %d of %d received\n",
			__func__, rc, num * sizeof(d->reg[0]));
		rc = -EIO;
	} else {
		rc = 0;
	}
#ifdef __LITTLE_ENDIAN
	do {
		unsigned i;
		u16 *buf = &d->reg[first];
		for (i = 0; i < num; i++, buf++)
			*buf = swab16(*buf);
	} while (0);
#endif
	return rc;
}

static struct so34_led_intf *get_led_intf(struct so34_data *d, const char *name)
{
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(d->led) && d->led[i]; i++) {
		if (!strncmp(name, d->led[i]->ldev.name, NAME_LIMIT))
			return d->led[i];
	}
	return NULL;
}

static u16 so34_set_led_effect(struct so34_data *d, u8 led_idx,
			enum so340010_led_effect effect)
{
	unsigned i = SO34_LED_CTL_1 + (led_idx >> 1);

	if (led_idx & 1) {
		d->reg[i] &= ~ODD_LEDS_EFFECT_MASK;
		d->reg[i] |= (u16)effect << ODD_LEDS_EFFECT_SHIFT;
	} else {
		d->reg[i] &= ~EVEN_LEDS_EFFECT_MASK;
		d->reg[i] |= (u16)effect << EVEN_LEDS_EFFECT_SHIFT;
	}
	return i;
}

static int so34_set_led_level(struct so34_data *d, u8 led_idx, u8 level256)
{
	u16 reg = SO34_LED_CTL_1 + (led_idx >> 1);
	u8 max_level = d->pcfg->pin[led_idx].u.led.max_level;
	u16 level = level256 >> 3;
	u16 val = d->reg[reg];
	u16 en_reg = d->reg[SO34_LED_ENA];
	u16 old_val;

	if (level > max_level)
		level = max_level;

	if (led_idx & 1) {
		old_val = val & ODD_LEDS_LEVEL_MASK;
		val &= ~ODD_LEDS_LEVEL_MASK;
		val |= level << ODD_LEDS_LEVEL_SHIFT;
	} else {
		old_val = val & EVEN_LEDS_LEVEL_MASK;
		val &= ~EVEN_LEDS_LEVEL_MASK;
		val |= level << EVEN_LEDS_LEVEL_SHIFT;
	}
	if (level && !old_val) {
		dev_dbg(&d->client->dev, "%s: enabling led %d\n",
				__func__, led_idx);
		so34_write(d, SO34_LED_ENA, en_reg | 1 << led_idx);
		d->hw_in_use |= (1 << led_idx);
	} else if (!level && old_val) {
		dev_dbg(&d->client->dev, "%s: disabling led %d\n",
				__func__, led_idx);
		so34_write(d, SO34_LED_ENA, en_reg & ~(1 << led_idx));
		d->hw_in_use &= ~(1 << led_idx);
	}
	return so34_write(d, reg, val);
}

static u16 so34_set_btn_sens(struct so34_data *d, u8 btn_idx, u8 sens)
{
	unsigned i = SO34_BTN_SNS_1 + (btn_idx >> 1);

	if (btn_idx & 1) {
		d->reg[i] &= ~ODD_BTN_SENS_MASK;
		d->reg[i] |= (u16)sens << ODD_BTN_SENS_SHIFT;
	} else {
		d->reg[i] &= ~EVEN_BTN_SENS_MASK;
		d->reg[i] |= sens << EVEN_BTN_SENS_SHIFT;
	}
	return i;
}

static inline struct so34_led_intf *ldev_to_intf(struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct so34_led_intf, ldev);
}

static void so34_set_led_brightness(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct device *dev = led_cdev->dev->parent;
	struct so34_data *d = dev_get_drvdata(dev);
	struct so34_led_intf *intf = ldev_to_intf(led_cdev);
	u8 i;

	LOCK_UI(d);
	for (i = 0; i < SO340010_PIN_NUM; i++) {
		if (!(intf->hw_led & (1 << i)))
			continue;
		if (so34_set_led_level(d, i, value))
			dev_err(dev, "led '%s' failed to set brightness"
				" (hw led %d).\n", led_cdev->name, i);
	}
	UNLOCK_UI(d);
}

static int so34_add_led_intf(struct so34_data *d, u8 li, const char *name)
{
	struct so34_led_intf *intf;
	struct device *dev = &d->client->dev;

	intf = get_led_intf(d, name);
	if (!intf) {
		if (d->led_num >= ARRAY_SIZE(d->led)) {
			dev_err(dev, "No space for new leds.\n");
			return -EINVAL;
		}
		intf = kzalloc(sizeof(*intf), GFP_KERNEL);
		if (!intf) {
			dev_err(dev, "%s: no memory.\n", __func__);
			return -ENOMEM;
		}
		d->led[d->led_num++] = intf;
		intf->hw_led |= (1 << li);
		intf->ldev.name = name;
		intf->ldev.brightness = LED_OFF;
		intf->ldev.brightness_set = so34_set_led_brightness;
		intf->ldev.blink_set = NULL;
		dev_info(dev, "New led '%s': hw leds 0x%x\n", name,
				intf->hw_led);
	} else {
		intf->hw_led |= (1 << li);
		dev_info(dev, "HW led %d added to led '%s': hw leds 0x%x\n",
				li, name, intf->hw_led);
	}
	return 0;
}

static int so34_add_btn_intf(struct so34_data *d, u8 idx, int code, u8 sens)
{
	struct so34_btn_intf *btn = &d->btn[idx];

	btn->sens = sens;
	btn->code = code;
	return 0;
}

static int so34_configure(struct so34_data *d)
{
	unsigned i;
	struct device *dev = &d->client->dev;
	const struct so340010_led_cfg *led;
	const struct so340010_btn_cfg *btn = d->pcfg->btn;
	const struct so340010_gpio_cfg *gpio;
	const struct so340010_pin_cfg *pin = d->pcfg->pin;
	int rc;

	memset(d->reg, 0, sizeof(d->reg));

	d->reg[SO34_CHIP_RESET] = SO34_RESET;
	/* default gpio state - output low */
	d->reg[SO34_GPIO_CTL] = GPIO_OUT_LOW(0) | GPIO_OUT_LOW(1) |
				GPIO_OUT_LOW(2) | GPIO_OUT_LOW(3);

	for (i = 0; i < ARRAY_SIZE(d->pcfg->pin); i++) {
		u16 m = 1 << i;

		if (pin[i].mode == SO340010_GPIO) {
			gpio = &pin[i].u.gpio;

			switch (gpio->mode) {
			case SO340010_INPUT_PULLUP:
				d->reg[SO34_GPIO_CTL] |= m;
				/* no break here */
			case SO340010_INPUT:
				d->reg[SO34_GPIO_CTL] &= ~(m << 8);
				dev_dbg(dev, "pin[%d] - gpio input\n", i);
				break;
			case SO340010_OUTPUT_HI:
				d->reg[SO34_GPIO_CTL] |= m;
				/* no break here */
			case SO340010_OUTPUT_LO:
				d->reg[SO34_GPIO_CTL] |= m << 8;
				dev_dbg(dev, "pin[%d] - gpio output\n", i);
				break;
			default:
				break;
			}
		} else {
			led = &pin[i].u.led;

			d->reg[SO34_GPIO_CTL] |= m;
			(void)so34_set_led_effect(d, i, led->effect);
			rc = so34_add_led_intf(d, i, led->name);
			if (rc)
				return -ENODEV;
			dev_dbg(dev, "LED '%s' on pin %d, ef 0x%x, max %d\n",
					led->name, i, led->effect,
					led->max_level);
		}

	}
	for (i = 0; i < ARRAY_SIZE(d->pcfg->btn); i++) {
		u8 m = 1 << i;

		if (!btn[i].enable)
			continue;
		d->reg[SO34_BTN_ENA] |= m;
		if (btn[i].map2gpio)
			d->reg[SO34_BTN_MAP] |= m;

		(void)so34_set_btn_sens(d, i, btn[i].sensitivity);

		if (btn[i].suspend == SO340010_SUSPEND_EARLY)
			d->btn_early_susp |= m;
		else if (btn[i].suspend == SO340010_SUSPEND_SYS || d->poll_jf)
			d->btn_sys_susp |= m;
		d->all_buttons |= m;

		rc = so34_add_btn_intf(d, i, btn[i].code,  btn[i].sensitivity);
		if (rc)
			return -ENODEV;
		dev_dbg(dev, "button on line %d - code %d\n", i, btn[i].code);

	}

	if (d->pcfg->btn_map2dir == SO340010_MAP2DIRECTION)
		d->reg[SO34_BTN_MAP] |= SO34_MAP2DIR;
	d->reg[SO34_BTN_MAP] |= d->pcfg->btn_out_mode;
	d->reg[SO34_G_CONFIG] |= d->pcfg->btn_mode;
	d->reg[SO34_LED_PERIOD] = d->pcfg->period_a_x12_5_ms;
	d->reg[SO34_LED_PERIOD] <<= 8;
	d->reg[SO34_LED_PERIOD] |= d->pcfg->period_b_x12_5_ms;
	d->reg[SO34_IF_CONFIG] = SO34_DATA_EN;
	if (!d->poll_jf)
		d->reg[SO34_IF_CONFIG] |= SO34_IRQ_ENABLE;
	if (d->pcfg->gpi_enable)
		d->reg[SO34_G_CONFIG] |= SO34_GPI_IRQ;
	return 0;
}

static int so34_process_data(struct so34_data *d)
{
	struct so34_btn_intf *btn = d->btn;
	struct device *dev = &d->client->dev;
	u16 *reg = d->reg;
	int rc;
	unsigned i;
	u8 val;
	u16 m = 0;

	rc = so34_read_region(d, SO34_RR_STATE);
	if (rc)
		return rc;

	if (d->pcfg->btn_map2dir == SO340010_MAP2DIRECTION)
		m = ((reg[SO34_BTN_MAP] & ~reg[SO34_BTN_STATE]) |
		~(reg[SO34_BTN_MAP] | (reg[SO34_GPIO_CTL] >> 8))) & 0x0f;

	val = d->reg[SO34_GPIO_STATE];
	d->toggled = (val ^ d->gpio_state) & m;
	d->gpio_state = val;
	if (d->toggled) {
		dev_dbg(dev, "gpios 0x%x, toggled = 0x%x\n", val, d->toggled);
		schedule_work(&d->gpio_cb_w);
	}
	for (i = 0; i < ARRAY_SIZE(d->btn); i++) {
		m = 1 << i;
		if (!(m & reg[SO34_BTN_ENA]))
			continue;
		val  = !!(reg[SO34_BTN_STATE] & (1 << i));
		input_report_key(d->idev, btn[i].code, val);
		dev_vdbg(dev, "Key %d, code %d, value %d", i, btn[i].code, val);
	}
	input_sync(d->idev);
	return 0;
}

static void so34_gpio_cb_wfunc(struct work_struct *work)
{
	struct so34_data *d = container_of(work, struct so34_data, gpio_cb_w);
	const struct so340010_pin_cfg *pin = d->pcfg->pin;
	const struct so340010_gpio_cfg *gpio;
	unsigned i;
	u8 toggled;
	u8 state;

	LOCK(d);
	state = d->gpio_state;
	toggled = d->toggled;
	UNLOCK(d);

	for (i = 0; i < ARRAY_SIZE(d->pcfg->pin); i++, gpio++) {
		if (pin[i].mode == SO340010_GPIO) {
			u8 m = 1 << i;
			gpio = &pin[i].u.gpio;

			if ((gpio->gpio_changed_cb) && (toggled & m))
				gpio->gpio_changed_cb(!!(state & m));
		}
	}
}

static int so34_flush_config(struct so34_data *d)
{
	enum so34_rr rr;
	int rc;

	so34_dump(d, __func__);
	for (rr = SO34_RR_CONFIG; rr <= SO34_RR_LED; rr++) {
		rc = so34_write_region(d, rr);
		if (rc)
			break;
	}
	return rc;
}

static void so34_sw_reset(struct so34_data *d)
{
	d->reg[SO34_CHIP_RESET] = 0;
	if(so34_write(d, SO34_CHIP_RESET, SO34_RESET))
		dev_info(&d->client->dev, "%s: failed.\n", __func__);
	msleep(250);
}

static int so34_reset(struct so34_data *d)
{
	int rc;
	struct device *dev = &d->client->dev;

	dev_info(dev, "%s\n", __func__);
	so34_sw_reset(d);

	if (d->pcfg->gpi_enable)
		d->reg[SO34_G_CONFIG] |= SO34_GPI_IRQ;
	if (!d->reg[SO34_BTN_ENA])
		d->reg[SO34_G_CONFIG] |= SO34_SLEEP;

	rc = so34_flush_config(d);
	if (rc)
		goto exit;
exit:
	dev_info(dev, "%s: complete, rc = %d\n", __func__, rc);
	return rc;
}

static void so34_poll_func(struct work_struct *work)
{
	int rc;
	struct so34_data *d = container_of(work, struct so34_data, poll_w.work);

	LOCK(d);
	if (d->pm_state & SO34_SYS_SUSPEND)
		goto exit;
	rc = so34_process_data(d);
	if (rc)
		rc = so34_reset(d);
	if (!rc)
		queue_delayed_work(d->wq, &d->poll_w, d->poll_jf);
exit:
	UNLOCK(d);
}

static irqreturn_t so34_isr_tread(int irq, void *dev_id)
{
	struct so34_data *d = dev_id;
	int rc;

	LOCK(d);
	if (d->pm_state & SO34_SYS_SUSPEND)
		goto exit;
	rc = so34_process_data(d);
	if (rc)
		rc = so34_reset(d);
exit:
	UNLOCK(d);
	return IRQ_HANDLED;
}

static bool so34_has_clients(struct so34_data *d)
{
	bool has_btns = d->reg[SO34_BTN_ENA];
	bool has_gpins = ~d->reg[SO34_GPIO_CTL] & 0xf00;

	return has_btns || has_gpins;
}

static int set_enabled_buttons(struct so34_data *d)
{
	int rc = 0;
	u16 buttons = d->all_buttons;
	struct device *dev = &d->client->dev;

	buttons |= d->reg[SO34_BTN_MAP] & 0xf;
	if (d->pm_state & SO34_EARLY_SUSPEND)
		buttons &= ~d->btn_early_susp;
	if (d->pm_state & SO34_SYS_SUSPEND)
		buttons &= ~(d->btn_sys_susp | d->btn_early_susp);
	if (buttons != d->reg[SO34_BTN_ENA]) {
		rc = so34_write(d, SO34_BTN_ENA, buttons);
		if (rc)
			dev_err(dev, "%s: failed (%d).\n", __func__, rc);
	}
	return rc;
}

static int so34_platform_power_stub(struct so34_data *d,
		enum so34_platform_power request,
		enum so34_platform_power *result)
{
	dev_dbg(&d->client->dev, "platform power not supported,"
		" 0x%02x requested\n", request);
	*result = SO34_PWR_ENABLED;
	return 0;
}

static int so34_platform_power(struct so34_data *d,
		enum so34_platform_power request)
{
	enum so34_platform_power result;
	int rc;

	if (request != d->pltf_pwr) {
		rc = d->pcfg->platform_pwr ? d->pcfg->platform_pwr(
				&d->client->dev, request, &result) :
				so34_platform_power_stub(d, request, &result);
		if (!rc)
			d->pltf_pwr = result;
		dev_dbg(&d->client->dev, "%s: set to 0x%x, 0x%x requested\n",
				__func__, d->pltf_pwr, request);
	} else {
		dev_dbg(&d->client->dev, "%s: no changes (0x%02x)\n", __func__,
			d->pltf_pwr);
		rc = 0;
	}
	return rc;
}

static void so34_req_sleep(struct so34_data *d, enum so34_pm_cmd mode)
{
	struct device *dev = &d->client->dev;
	u16 val = d->reg[SO34_G_CONFIG];
	bool gpi_disable = false;
	const char *sleep_reason;

	LOCK(d);

	if (mode == SO34_SUSPEND_EARLY) {
		gpi_disable = d->pcfg->gpi_suspend == SO340010_SUSPEND_EARLY;
		sleep_reason = "early";
		d->pm_state |= SO34_EARLY_SUSPEND;
	} else if (mode == SO34_SUSPEND) {
		gpi_disable = d->pcfg->gpi_suspend == SO340010_SUSPEND_SYS;
		sleep_reason = "system";
		d->pm_state |= SO34_SYS_SUSPEND;
	} else {
		sleep_reason = "user";
	}
	dev_dbg(dev, "%s: mode %s-suspend\n", __func__, sleep_reason);

	if (set_enabled_buttons(d))
		dev_err(dev, "%s [%d]: failed.\n", __func__, __LINE__);

	if (gpi_disable && !d->poll_jf) {
		d->hw_in_use &= ~HW_GPI;
		val &= ~SO34_GPI_IRQ;
	}

	if (!d->reg[SO34_BTN_ENA]) {
		val |= SO34_SLEEP;
		d->hw_in_use &= ~HW_BUTTONS;
	}

	if (val != d->reg[SO34_G_CONFIG]) {
		if (so34_write(d, SO34_G_CONFIG, val))
			dev_err(dev, "%s [%d]: failed.\n", __func__, __LINE__);
	}
	if ((!so34_has_clients(d) || mode == SO34_SUSPEND) && d->poll_jf)
		goto stop_polling;
	UNLOCK(d);
	return;
stop_polling:
	UNLOCK(d);
	cancel_delayed_work_sync(&d->poll_w);
	d->hw_in_use &= ~HW_I2C;
	dev_dbg(dev, "%s: polling stopped.\n", __func__);
}

static void so34_req_wake(struct so34_data *d, enum so34_pm_cmd mode)
{
	struct device *dev = &d->client->dev;
	u16 val = d->reg[SO34_G_CONFIG];
	bool gpi_enable = false;
	const char *wake_reason;

	LOCK(d);
	if (mode == SO34_RESUME_LATE) {
		gpi_enable = d->pcfg->gpi_suspend == SO340010_SUSPEND_EARLY;
		wake_reason = "late";
		d->pm_state &= ~SO34_EARLY_SUSPEND;
	} else if (mode == SO34_RESUME) {
		gpi_enable = d->pcfg->gpi_suspend == SO340010_SUSPEND_SYS;
		wake_reason = "system";
		d->pm_state &= ~SO34_SYS_SUSPEND;
	} else {
		wake_reason = "user";
	}
	dev_dbg(dev, "%s: mode %s-wake\n", __func__, wake_reason);

	if (set_enabled_buttons(d))
		dev_err(dev, "%s [%d]: failed.\n", __func__, __LINE__);

	if (gpi_enable && !d->poll_jf) {
		d->hw_in_use |= HW_GPI;
		val |= SO34_GPI_IRQ;
	}

	if (d->reg[SO34_BTN_ENA]) {
		val &= ~SO34_SLEEP;
		d->hw_in_use |= HW_BUTTONS;
	}

	if (val != d->reg[SO34_G_CONFIG]) {
		if (so34_write(d, SO34_G_CONFIG, val))
			dev_err(dev, "%s: failed.\n", __func__);
	}
	if (so34_has_clients(d) && d->poll_jf) {
		queue_delayed_work(d->wq, &d->poll_w, d->poll_jf);
		d->hw_in_use |= HW_I2C;
		dev_dbg(dev, "%s: polling started.\n", __func__);
	} else if (so34_has_clients(d)) {
		so34_read_region(d, SO34_RR_STATE);
	}
	UNLOCK(d);
}
static void so34_dummy_read(struct so34_data *d)
{
	int dummy_read_tries = 3;
	struct device *dev = &d->client->dev;

	while (dummy_read_tries--) {
		if (!so34_read_region(d, SO34_RR_GPIO_CTL))
			return;
		mdelay(10);
	}
	dev_warn(dev, "%s: dummy read start-up loop failed\n", __func__);
}

#ifdef CONFIG_SUSPEND
static int so34_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct so34_data *d = i2c_get_clientdata(client);
	enum so34_platform_power request = d->hw_in_use ? SO34_PWR_ENABLED : 0;

	dev_dbg(dev, "%s\n", __func__);
	so34_req_sleep(d, SO34_SUSPEND);
	return so34_platform_power(d, request);
}

static int so34_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct so34_data *d = i2c_get_clientdata(client);
	bool powered = d->pltf_pwr & SO34_PWR_ENABLED;
	int rc;

	dev_dbg(dev, "%s\n", __func__);
	rc = so34_platform_power(d, SO34_PWR_ENABLED);
	if (!rc) {
		if (!powered) {
			so34_dummy_read(d);
			rc = so34_reset(d);
		}
		if (!rc) {
			so34_req_wake(d, SO34_RESUME);
			rc = so34_process_data(d);
			if (rc)
				rc = so34_reset(d);
		}
	}
	return rc;
}
#else
#define so34_suspend NULL
#define so34_resume NULL
#endif

static const struct dev_pm_ops so34_pm = {
	.suspend = so34_suspend,
	.resume = so34_resume,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void so34_early_suspend(struct early_suspend *h)
{
	struct so34_data *d = container_of(h, struct so34_data, early_suspend);
	struct device *dev = &d->client->dev;
	dev_dbg(dev, "%s\n", __func__);
	so34_req_sleep(d, SO34_SUSPEND_EARLY);
}

static void so34_late_resume(struct early_suspend *h)
{
	struct so34_data *d = container_of(h, struct so34_data, early_suspend);
	struct device *dev = &d->client->dev;

	dev_dbg(dev, "%s\n", __func__);
	so34_req_wake(d, SO34_RESUME_LATE);
}
#endif

static void so34_cleanup(struct so34_data *d)
{
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(d->led); i++)
		kfree(d->led[i]);
}

static int so34_register_ldevs(struct so34_data *d)
{
	int i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(d->led) && d->led[i]; i++) {
		rc = led_classdev_register(&d->client->dev, &d->led[i]->ldev);
		if (rc)
			goto roll_back;
	}
	return rc;
roll_back:
	while (--i >= 0)
		led_classdev_unregister(&d->led[i]->ldev);
	dev_err(&d->client->dev, "%s: failed, rc = %d.\n", __func__, rc);
	return rc;
}

static void so34_unregister_ldevs(struct so34_data *d)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(d->led) && d->led[i]; i++)
		led_classdev_unregister(&d->led[i]->ldev);
}

static int __devinit so34_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct so340010_config *cfg = client->dev.platform_data;
	struct device *dev = &client->dev;
	struct so34_data *d;
	int rc;
	unsigned i;

	dev_info(dev, "%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		rc = -EIO;
		goto err_check_functionality;
	}
	if (!cfg) {
		rc = -EINVAL;
		dev_err(dev, "%s: configuration data required.\n", __func__);
		goto err_no_platform_data;
	}
	if (cfg->platform_setup) {
		rc = cfg->platform_setup(dev);
		if (rc)
			goto err_psetup;
	}
	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d) {
		rc = -ENOMEM;
		goto err_alloc_data_failed;
	}

	d->client = client;
	d->pcfg = cfg;
	i2c_set_clientdata(client, d);
	INIT_LOCK(d);
	INIT_WORK(&d->gpio_cb_w, so34_gpio_cb_wfunc);
	wake_lock_init(&d->ui_lock, WAKE_LOCK_SUSPEND, SO340010_DEV_NAME);
	if (cfg->polling_ms > 0)
		d->poll_jf =  msecs_to_jiffies(cfg->polling_ms);
	else if (client->irq <= 0)
		d->poll_jf =  msecs_to_jiffies(DEFAULT_POLL_MS);

	rc = so34_platform_power(d, SO34_PWR_ENABLED);
	if (rc)
		goto err_power_on;
	so34_dummy_read(d);

	rc = so34_configure(d);
	if (rc) {
		dev_err(dev, "%s: device configuration failed\n", __func__);
		goto err_configure;
	}
	d->reg[SO34_G_CONFIG] |= SO34_SLEEP;
	rc = so34_flush_config(d);
	if (rc) {
		dev_err(dev, "%s: configuration not applied\n", __func__);
		goto err_configure;
	}
	dev_info(dev, "%s: HW started in sleep mode\n", __func__);

	rc = so34_register_ldevs(d);
	if (rc)
		goto err_reg_ldevs;

	d->idev = input_allocate_device();
	if (!d->idev) {
		dev_err(dev, "%s: input_allocate_device failed\n", __func__);
		rc = -ENOMEM;
		goto err_allocate_device;
	}
	input_set_drvdata(d->idev, d);
	d->idev->name = cfg->input_name;
	rc = input_register_device(d->idev);
	if (rc) {
		dev_err(dev, "%s: input dev register failed\n", __func__);
		input_free_device(d->idev);
		goto err_input_register;
	}
	for (i = 0; i < ARRAY_SIZE(cfg->btn); i++) {
		if (cfg->btn[i].enable)
			input_set_capability(d->idev, EV_KEY, cfg->btn[i].code);
	}
	if (d->poll_jf) {
		d->wq = create_singlethread_workqueue(SO340010_DEV_NAME);
		if (!d->wq) {
			dev_err(dev, "%s: workqueue alloc error\n", __func__);
			goto err_alloc_wq_failed;
		}
		INIT_DELAYED_WORK(&d->poll_w, so34_poll_func);
		dev_info(dev, "%s: setting polling mode (%d jiffies)\n",
			__func__, d->poll_jf);
	} else {
		rc = request_threaded_irq(client->irq, NULL, so34_isr_tread,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				SO340010_DEV_NAME, d);
		if (rc) {
			dev_err(dev, "%s: failed to request irq.\n", __func__);
			goto err_irq_request;
		}
		if (d->reg[SO34_BTN_ENA] &
				~(d->btn_sys_susp | d->btn_early_susp)) {
			rc = irq_set_irq_wake(client->irq, 1);
			if (rc)
				dev_info(dev, "%s: irq wake not set, rc %d\n",
						__func__, rc);
		}
		dev_info(dev, "%s: setting irq mode\n", __func__);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	d->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	d->early_suspend.suspend = so34_early_suspend;
	d->early_suspend.resume = so34_late_resume;
	register_early_suspend(&d->early_suspend);
#endif
	so34_req_wake(d, SO34_RESUME_USER);
	dev_info(dev, "%s: completed.\n", __func__);
	return 0;


err_irq_request:
err_alloc_wq_failed:
	input_unregister_device(d->idev);
err_input_register:
err_allocate_device:
	so34_unregister_ldevs(d);
err_reg_ldevs:
	so34_cleanup(d);
err_configure:
	so34_platform_power(d, 0);
err_power_on:
	wake_lock_destroy(&d->ui_lock);
	mutex_destroy(&d->lock);
	kfree(d);
err_alloc_data_failed:
	if (cfg->platform_teardown)
		cfg->platform_teardown(dev);
err_psetup:
err_no_platform_data:
err_check_functionality:
	dev_err(dev, "%s: failed.\n", __func__);
	return rc;
}

static int __devexit so34_remove(struct i2c_client *client)
{
	struct so34_data *d = i2c_get_clientdata(client);

	unregister_early_suspend(&d->early_suspend);
	if (d->wq)
		destroy_workqueue(d->wq);
	if (!d->poll_jf)
		free_irq(client->irq, d);
	input_unregister_device(d->idev);
	so34_unregister_ldevs(d);
	so34_cleanup(d);
	wake_lock_destroy(&d->ui_lock);
	mutex_destroy(&d->lock);
	so34_platform_power(d, 0);
	if (d->pcfg->platform_teardown)
		d->pcfg->platform_teardown(&client->dev);
	kfree(d);
	return 0;
}

static const struct i2c_device_id so34_id[] = {
	{SO340010_DEV_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, so34_id);

static struct i2c_driver so34_driver = {
	.driver = {
		.name = SO340010_DEV_NAME,
		.owner = THIS_MODULE,
		.pm = &so34_pm,
	},
	.probe = so34_probe,
	.remove = __devexit_p(so34_remove),
	.id_table = so34_id,
};

static int __init so34_init(void)
{
	int err = i2c_add_driver(&so34_driver);
	pr_info("%s: so340010-16qnf sensing IC driver, built %s @ %s\n",
		 __func__, __DATE__, __TIME__);
	return err;
}

static void __exit so34_exit(void)
{
	i2c_del_driver(&so34_driver);
}

module_init(so34_init);
module_exit(so34_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonyericsson.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("so340010-16qnf configurable capacitive sensing IC driver");
