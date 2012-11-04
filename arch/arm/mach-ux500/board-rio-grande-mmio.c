/*
 * Copyright (C) 2011 ST-Ericsson
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Joakim Axelsson <joakim.axelsson@stericsson.com> for ST-Ericsson
 * Author: Rajat Verma <rajat.verma@stericsson.com> for ST-Ericsson.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/mmio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/vmalloc.h>
#include <plat/gpio-nomadik.h>
#include <plat/pincfg.h>
#include <mach/gpio.h>
#include <mach/devices.h>
#include <mach/db8500-regs.h>
#include "pins-db8500.h"
#include "pins.h"
#include "board-mop500.h"

#define XSHUTDOWN_PRIMARY_SENSOR	141
#define XSHUTDOWN_SECONDARY_SENSOR	142

static pin_cfg_t poweron_pins[] = {
	GPIO64_GPIO,
	GPIO67_GPIO,
	GPIO65_GPIO,
	GPIO5_GPIO
};

static pin_cfg_t i2c2_pins[] = {
	GPIO8_I2C2_SDA,
	GPIO9_I2C2_SCL
};
static pin_cfg_t ipi2c_pins[] = {
	GPIO8_IPI2C_SDA,
	GPIO9_IPI2C_SCL
};
static pin_cfg_t i2c_disable_pins[] = {
	GPIO8_GPIO,
	GPIO9_GPIO
};
static pin_cfg_t xshutdown_host[] = {
	GPIO141_GPIO,
	GPIO142_GPIO
};
static pin_cfg_t xshutdown_fw[] = {
	GPIO141_IP_GPIO2,
	GPIO142_IP_GPIO3
};
static pin_cfg_t xshutdown_disable[] = {
	GPIO141_GPIO | PIN_OUTPUT_LOW,
	GPIO142_GPIO | PIN_OUTPUT_HIGH
};

struct mmio_board_data {
	int number_of_regulators;
	struct regulator **mmio_regulators;
	/* Pin configs  */
	int xenon_charge;
	struct mmio_gpio xshutdown_pins[CAMERA_SLOT_END];
	/* Internal clocks */
	struct clk *clk_ptr_bml;
	struct clk *clk_ptr_ipi2c;
	/* External clocks */
	struct clk *clk_ptr_ext[CAMERA_SLOT_END];
	/* marks if clocks, power lines are already enabled */
	int clocks_enabled;
	int power_enabled;
};

/* Fill names of regulators required for powering up the
 * camera sensor in below array */
static char *regulator_names[] = {"vddcsi1v2"};

static int mmio_pwr_onoff(struct mmio_platform_data *pdata, int onoff)
{
	int err = 0;

	dev_dbg(pdata->dev, ">>> %s: onoff = %d\n", __func__, onoff);

	err = nmk_config_pin((PIN_NUM(64) | PIN_GPIO | (onoff ? PIN_OUTPUT_HIGH : PIN_OUTPUT_LOW)), 0);
	if (err)
		goto out;

	udelay(150);

	err = nmk_config_pin((PIN_NUM(67) | PIN_GPIO | (onoff ? PIN_OUTPUT_HIGH : PIN_OUTPUT_LOW)), 0);
	if (err)
		goto out;

	udelay(110);

	err = nmk_config_pin((PIN_NUM(65) | PIN_GPIO | (onoff ? PIN_OUTPUT_HIGH : PIN_OUTPUT_LOW)), 0);
	if (err)
		goto out;

	err = nmk_config_pin((PIN_NUM(5) | PIN_GPIO | (onoff ? PIN_OUTPUT_HIGH : PIN_OUTPUT_LOW)), 0);
	if (err)
		goto out;

	udelay(110);

	dev_dbg(pdata->dev, "<<< %s: OK\n", __func__);

	return 0;

out:
	dev_err(pdata->dev, "%s(): error=%d while setting LDO GPIOs", __func__, err);

	return err;
}



/* This function is used to translate the physical GPIO used for reset GPIO
 * to logical IPGPIO that needs to be communicated to Firmware. so that
 * firmware can control reset GPIO of a RAW Bayer sensor */
static int mmio_get_ipgpio(struct mmio_platform_data *pdata, int gpio,
			   int *ip_gpio)
{
	int err = 0;
	dev_dbg(pdata->dev, "%s() : IPGPIO requested for %d", __func__, gpio);
	switch (gpio) {
	case 67:
	case 140:
		*ip_gpio = 7;
		break;
	case 5:
	case 66:
		*ip_gpio = 6;
		break;
	case 81:
	case 65:
		*ip_gpio = 5;
		break;
	case 80:
	case 64:
		*ip_gpio = 4;
		break;
	case 10:
	case 79:
	case 142:
		*ip_gpio = 3;
		break;
	case 11:
	case 78:
	case 141:
		*ip_gpio = 2;
		break;
	case 7:
	case 150:
		*ip_gpio = 1;
		break;
	case 6:
	case 149:
		*ip_gpio = 0;
		break;
	default:
		*ip_gpio = -1;
		err = -1;
		break;
	}
	return err;
}

static int mmio_clock_init(struct mmio_platform_data *pdata)
{
	int err;
	struct mmio_board_data *extra = pdata->extra;
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	extra->clocks_enabled = 0;

	extra->clk_ptr_bml = clk_get_sys("bml", NULL);
	if (IS_ERR(extra->clk_ptr_bml)) {
		err = PTR_ERR(extra->clk_ptr_bml);
		dev_err(pdata->dev, "Error %d getting clock 'bml'\n", err);
		goto err_bml_clk;
	}
	extra->clk_ptr_ipi2c = clk_get_sys("ipi2", NULL);
	if (IS_ERR(extra->clk_ptr_ipi2c)) {
		err = PTR_ERR(extra->clk_ptr_ipi2c);
		dev_err(pdata->dev, "Error %d getting clock 'ipi2'\n", err);
		goto err_ipi2c_clk;
	}
	extra->clk_ptr_ext[PRIMARY_CAMERA] = clk_get_sys("pri-cam", NULL);
	if (IS_ERR(extra->clk_ptr_ext[PRIMARY_CAMERA])) {
		err = PTR_ERR(extra->clk_ptr_ext[PRIMARY_CAMERA]);
		dev_err(pdata->dev, "Error %d getting clock 'pri-cam'\n", err);
		goto err_pri_ext_clk;
	}
	extra->clk_ptr_ext[SECONDARY_CAMERA] = clk_get_sys("sec-cam", NULL);
	if (IS_ERR(extra->clk_ptr_ext[SECONDARY_CAMERA])) {
		err = PTR_ERR(extra->clk_ptr_ext[SECONDARY_CAMERA]);
		dev_err(pdata->dev, "Error %d getting clock 'sec-cam'\n", err);
		goto err_sec_ext_clk;
	}
	dev_dbg(pdata->dev, "Board %s() Exit\n", __func__);
	return 0;
err_sec_ext_clk:
	clk_put(extra->clk_ptr_ext[PRIMARY_CAMERA]);
err_pri_ext_clk:
	clk_put(extra->clk_ptr_ipi2c);
err_ipi2c_clk:
	clk_put(extra->clk_ptr_bml);
err_bml_clk:
	return err;
}

static void mmio_clock_exit(struct mmio_platform_data *pdata)
{
	struct mmio_board_data *extra = pdata->extra;
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);
	clk_put(extra->clk_ptr_bml);
	clk_put(extra->clk_ptr_ipi2c);
	clk_put(extra->clk_ptr_ext[PRIMARY_CAMERA]);
	clk_put(extra->clk_ptr_ext[SECONDARY_CAMERA]);
}

static int mmio_pin_cfg_init(struct mmio_platform_data *pdata)
{
	struct mmio_board_data *extra = pdata->extra;
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	extra->xshutdown_pins[PRIMARY_CAMERA].gpio = XSHUTDOWN_PRIMARY_SENSOR;
	extra->xshutdown_pins[PRIMARY_CAMERA].active_high = 0;
	extra->xshutdown_pins[PRIMARY_CAMERA].udelay = 500;

	dev_dbg(pdata->dev, "setting XSHUTDOWN_SECONDARY_SENSOR in extra->xshutdown_pins\n");
	extra->xshutdown_pins[SECONDARY_CAMERA].gpio =
					XSHUTDOWN_SECONDARY_SENSOR;
	extra->xshutdown_pins[SECONDARY_CAMERA].active_high = 0;
	extra->xshutdown_pins[SECONDARY_CAMERA].udelay = 500;

	extra->xenon_charge = 0;

	dev_dbg(pdata->dev, "Board %s() Exit\n", __func__);
	return 0;
}

static void mmio_pin_cfg_exit(struct mmio_platform_data *pdata)
{
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);
}


/* For now, both sensors on HREF have some power up sequence. If different
 * sequences are needed for primary and secondary sensors, it can be
 * implemented easily. Just use camera_slot field of mmio_platform_data
 * to determine which camera needs to be powered up */

static int mmio_power_init(struct mmio_platform_data *pdata)
{
	int err = 0, i = 0, pincount;
	struct mmio_board_data *extra = pdata->extra;

	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	extra->power_enabled = 0;

	extra->number_of_regulators = sizeof(regulator_names)/
					sizeof(regulator_names[0]);
	extra->mmio_regulators =
	    kzalloc(sizeof(struct regulator *) * extra->number_of_regulators,
		    GFP_KERNEL);
	if (!extra->mmio_regulators) {
		dev_err(pdata->dev , "Error while allocating memory for mmio"
				"regulators\n");
		err = -ENOMEM;
		goto err_no_mem_reg;
	}
	for (i = 0; i <
		extra->number_of_regulators; i++) {

		dev_dbg(pdata->dev, "%s() ... regulator_get(%s)\n", __func__, regulator_names[i]);
		extra->mmio_regulators[i] =
			regulator_get(pdata->dev, regulator_names[i]);
		if (IS_ERR(extra->mmio_regulators[i])) {
			err = PTR_ERR(extra->mmio_regulators[i]);
			dev_err(pdata->dev , "Error %d getting regulator '%s'"
				"\n", err, regulator_names[i]);
			goto err_regulator;
		}
	}

	/* request GPIOs for camera LDOs */
	pincount = (sizeof(poweron_pins) / sizeof(pin_cfg_t));
	for (i = 0; i < pincount; i++) {

		dev_dbg(pdata->dev, ".. request GPIO[%2.2lu]\n", PIN_NUM(poweron_pins[i]));
		err = gpio_request(PIN_NUM(poweron_pins[i]), "mmio power LDO");

		if (err) {
			dev_err(pdata->dev, "%s(): error=%d while requesting LDO GPIO[%2.2lu]\n",
				__func__,
				err,
				PIN_NUM(poweron_pins[i]));

			for (i--; i >= 0; i--) {
				dev_dbg(pdata->dev, ".. freeing up GPIO[%2.2lu]\n", PIN_NUM(poweron_pins[i]));
				gpio_free(PIN_NUM(poweron_pins[i]));
			}

			i = extra->number_of_regulators;
			goto err_regulator;
		}
	}

	dev_dbg(pdata->dev, "Board %s() Exit\n", __func__);
	return 0;

err_regulator:
	/* Return regulators we have already requested */
	while (i--) {
		dev_dbg(pdata->dev, "%s() ... regulator_put(%s)\n", __func__, regulator_names[i]);
		regulator_put(extra->mmio_regulators[i]);
	}
	kfree(extra->mmio_regulators);

err_no_mem_reg:
	return err;
}


static void mmio_power_exit(struct mmio_platform_data *pdata)
{
	int i = 0, pincount;
	struct mmio_board_data *extra = pdata->extra;

	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	pincount = (sizeof(poweron_pins) / sizeof(pin_cfg_t));

	/* release GPIOs for camera LDOs */
	for (i = (pincount-1); i >= 0; i--) {

		dev_dbg(pdata->dev, ".. free GPIO[%2.2lu]\n",
					PIN_NUM(poweron_pins[i]));
		gpio_free(PIN_NUM(poweron_pins[i]));
	}

	for (i = 0; i < extra->number_of_regulators; i++) {
		dev_dbg(pdata->dev, "%s() ... regulator_put(%s)\n", __func__, regulator_names[i]);
		regulator_put(extra->mmio_regulators[i]);
	}
	kfree(extra->mmio_regulators);
}

static int mmio_platform_init(struct mmio_platform_data *pdata)
{
	int err = 0;
	struct mmio_board_data *extra = NULL;

	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	/* Alloc memory for our own extra data */
	extra = kzalloc(sizeof(struct mmio_board_data), GFP_KERNEL);
	if (!extra) {
		dev_err(pdata->dev, "%s: memory alloc failed for "
		"mmio_board_data\n", __func__);
		err = -ENOMEM;
		goto err_no_mem_extra;
	}
	/* Hook the data for other callbacks to use */
	pdata->extra = extra;
	pdata->camera_slot = -1;

	err = mmio_power_init(pdata);
	if (err)
		goto err_regulator;

	err = mmio_clock_init(pdata);
	if (err)
		goto err_clock;

	err = mmio_pin_cfg_init(pdata);
	if (err)
		goto err_pin_cfg;

	/* Store logical IPGPIO for physical reset GPIOs used */
	dev_dbg(pdata->dev, "translating xshutdown_pins for primary camera\n");
	err = mmio_get_ipgpio(pdata,
		extra->xshutdown_pins[PRIMARY_CAMERA].gpio,
		&(pdata->reset_ipgpio[PRIMARY_CAMERA]));
	if (err) {
		dev_err(pdata->dev, "Error getting ipgpio for pri cam\n");
		goto err_ipgpio;
	}
	dev_dbg(pdata->dev, "translating xshutdown_pins for secondary camera\n");
	err = mmio_get_ipgpio(pdata,
		extra->xshutdown_pins[SECONDARY_CAMERA].gpio,
		&(pdata->reset_ipgpio[SECONDARY_CAMERA]));
	if (err) {
		dev_err(pdata->dev, "Error getting ipgpio for sec cam\n");
		goto err_ipgpio;
	}
	dev_dbg(pdata->dev, "Board %s() Exit\n", __func__);
	return 0;
err_ipgpio:
	mmio_pin_cfg_exit(pdata);
err_pin_cfg:
	mmio_clock_exit(pdata);
err_clock:
	mmio_power_exit(pdata);
err_regulator:
	kfree(extra);
err_no_mem_extra:
	return err;
}

static void mmio_platform_exit(struct mmio_platform_data *pdata)
{
	struct mmio_board_data *extra = pdata->extra;
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);
	mmio_power_exit(pdata);
	mmio_clock_exit(pdata);
	mmio_pin_cfg_exit(pdata);
	kfree(extra);
	pdata->extra = NULL;
}

static int mmio_power_enable(struct mmio_platform_data *pdata)
{
	int err = 0, i = 0;
	struct mmio_board_data *extra = pdata->extra;
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	/* if already enabled */
	if (extra->power_enabled) {
		dev_dbg(pdata->dev, "Board %s() Exit: camera power already"
			" enabled, do NOT enable again!\n", __func__);
		return 0;
	}

	/* Enable the regulators */
	for (i = 0; i < extra->number_of_regulators; i++) {
		dev_dbg(pdata->dev, "%s() ... regulator_enable(%s)\n", __func__, regulator_names[i]);
		err = regulator_enable(extra->mmio_regulators[i]);
		if (IS_ERR(extra->mmio_regulators[i])) {
			err = PTR_ERR(extra->mmio_regulators[i]);
			dev_err(pdata->dev , "Error %d enabling regulator '%s'"
			"\n", err, regulator_names[i]);
			goto err_regulator;
		}
	}

	err = mmio_pwr_onoff(pdata, 1);

	extra->power_enabled = 1;

	dev_dbg(pdata->dev, "Board %s() Exit: %d\n", __func__, err);
	return err;

err_regulator:
	/* Disable regulators we already enabled */
	while (i--) {
		dev_dbg(pdata->dev, "%s() ... regulator_disable(%s)\n", __func__, regulator_names[i]);
		regulator_disable(extra->mmio_regulators[i]);
	}

	dev_dbg(pdata->dev, "Board %s() Exit: %d\n", __func__, err);
	return err;
}

static void mmio_power_disable(struct mmio_platform_data *pdata)
{
	int i, err;

	struct mmio_board_data *extra = pdata->extra;
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	/* if already disabled */
	if (!extra->power_enabled) {
		dev_dbg(pdata->dev, "Board %s() Exit: camera power already"
			" disabled, do NOT disable again!\n", __func__);
		return 0;
	}

	/* Disable the regulators */
	for (i = 0; i < extra->number_of_regulators; i++) {
		dev_dbg(pdata->dev, "%s() ... regulator_disable(%s)\n", __func__, regulator_names[i]);
		regulator_disable(extra->mmio_regulators[i]);
	}

	err = mmio_pwr_onoff(pdata, 0);

	extra->power_enabled = 0;

	dev_dbg(pdata->dev, "Board %s() Exit: %d\n", __func__, err);
}

static int mmio_clock_enable(struct mmio_platform_data *pdata)
{
	int err = 0;
	struct mmio_board_data *extra = pdata->extra;
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	/* if already enabled */
	if (extra->clocks_enabled) {
		dev_dbg(pdata->dev, "Board %s() Exit: Clocks already"
			" enabled, do NOT enable again!\n", __func__);
		return 0;
	}

	/* Enable internal clocks */
	err = clk_enable(extra->clk_ptr_bml);
	if (err) {
		dev_err(pdata->dev, "Error activating bml clock %d\n", err);
		goto err_bml_clk;
	}
	err = clk_enable(extra->clk_ptr_ipi2c);
	if (err) {
		dev_err(pdata->dev, "Error activating i2c2 clock %d\n", err);
		goto err_ipi2c_clk;
	}
	/* Enable appropriate external clock */
	err = clk_enable(extra->clk_ptr_ext[pdata->camera_slot]);
	if (err) {
		dev_err(pdata->dev, "Error activating clock for sensor %d, err"
			"%d\n", pdata->camera_slot, err);
		goto err_ext_clk;
	}

	extra->clocks_enabled = 1;

	dev_dbg(pdata->dev, "Board %s() Exit\n", __func__);
	return 0;
err_ext_clk:
	clk_disable(extra->clk_ptr_ipi2c);
err_ipi2c_clk:
	clk_disable(extra->clk_ptr_bml);
err_bml_clk:
	return err;
}

static void mmio_clock_disable(struct mmio_platform_data *pdata)
{
	struct mmio_board_data *extra = pdata->extra;
	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	/* if already disabled */
	if (!extra->clocks_enabled) {
		dev_dbg(pdata->dev, "Board %s() Exit: Clocks already"
			" disabled, do NOT disable again!\n", __func__);
		return;
	}

	clk_disable(extra->clk_ptr_bml);
	clk_disable(extra->clk_ptr_ipi2c);
	clk_disable(extra->clk_ptr_ext[pdata->camera_slot]);

	extra->clocks_enabled = 0;
}

static int mmio_config_xshutdown_pins(struct mmio_platform_data *pdata,
				      enum mmio_select_xshutdown_t select,
				      int is_active_high)
{
	int err = 0;
	struct mmio_board_data *extra = pdata->extra;

	dev_dbg(pdata->dev , "Board %s() Enter. is_active_high:%d cam:%d\n",
		__func__, is_active_high, pdata->camera_slot);

	switch (select) {
	case MMIO_ENABLE_XSHUTDOWN_HOST:
		dev_dbg(pdata->dev , "Board %s() HOST\n", __func__);
		extra->xshutdown_pins[pdata->camera_slot].active_high =
			is_active_high;
		err = nmk_config_pin(xshutdown_host[pdata->camera_slot] |
			(is_active_high ? PIN_OUTPUT_LOW : PIN_OUTPUT_HIGH), 0);
		dev_dbg(pdata->dev, "ENABLE_XSHUTDOWN_HOST: GPIO[%lu] set to %s\n",
			PIN_NUM(xshutdown_host[pdata->camera_slot]),
			(is_active_high ? "PIN_OUTPUT_LOW" : "PIN_OUTPUT_HIGH"));
		break;
	case MMIO_ENABLE_XSHUTDOWN_FW:
		if (pdata->camera_slot == 0) {
			dev_dbg(pdata->dev, "Board %s() FW\n", __func__);
			err = nmk_config_pin(xshutdown_fw[pdata->camera_slot], 0);
		}
		dev_dbg(pdata->dev, "ENABLE_XSHUTDOWN_FW: GPIO[%lu] set to Alt-%lu\n",
			PIN_NUM(xshutdown_fw[pdata->camera_slot]),
			PIN_ALT(xshutdown_fw[pdata->camera_slot]));
		break;
	case MMIO_DISABLE_XSHUTDOWN:
		dev_dbg(pdata->dev, "Board %s() DISABLE\n", __func__);
		err = nmk_config_pin(xshutdown_disable[pdata->camera_slot], 0);
		dev_dbg(pdata->dev, "DISABLE_XSHUTDOWN: GPIO[%lu] set to Alt-%lu\n",
			PIN_NUM(xshutdown_disable[pdata->camera_slot]),
			PIN_ALT(xshutdown_disable[pdata->camera_slot]));
		break;
	default:
		break;
	}
	if (err)
		dev_dbg(pdata->dev, "Error configuring xshutdown, err = %d\n",
		err);
	return err;
}


static void mmio_set_xshutdown(struct mmio_platform_data *pdata)
{
	struct mmio_board_data *extra = pdata->extra;
	int value;

	dev_dbg(pdata->dev, "Board %s() Enter\n", __func__);

	if (pdata->camera_slot != 0) {
		value = (extra->xshutdown_pins[pdata->camera_slot].active_high ? 1 : 0);
		dev_dbg(pdata->dev, "setting XSHUTDOWN_(%d)_SENSOR gpio-%d to %d\n", pdata->camera_slot, extra->xshutdown_pins[pdata->camera_slot].gpio, value);
		gpio_set_value(extra->xshutdown_pins[pdata->camera_slot].gpio, value);
		dev_dbg(pdata->dev, "udelay(%d)\n", extra->xshutdown_pins[pdata->camera_slot].udelay);
		udelay(extra->xshutdown_pins[pdata->camera_slot].udelay);
	}
		dev_dbg(pdata->dev, "%s: Problem avoided\n", __func__);
}


static int mmio_config_i2c_pins(struct mmio_platform_data *pdata,
				enum mmio_select_i2c_t select)
{
	int err = 0;

	dev_dbg(pdata->dev, "Board %s() Enter, pdata->camera_slot: %d\n", __func__, pdata->camera_slot);

	switch (select) {
	case MMIO_ACTIVATE_I2C_HOST:
		dev_dbg(pdata->dev, "%s: MMIO_ACTIVATE_I2C_HOST\n", __func__);
		err = nmk_config_pins(i2c2_pins, ARRAY_SIZE(i2c2_pins));
		break;
	case MMIO_ACTIVATE_IPI2C2:
		dev_dbg(pdata->dev, "%s: MMIO_ACTIVATE_IPI2C2\n", __func__);
		err = nmk_config_pins(ipi2c_pins, ARRAY_SIZE(ipi2c_pins));
		break;
	case MMIO_DEACTIVATE_I2C:
		dev_dbg(pdata->dev, "%s: MMIO_DEACTIVATE_I2C\n", __func__);
		err = nmk_config_pins(i2c_disable_pins,
			ARRAY_SIZE(i2c_disable_pins));
		break;
	default:
		break;
	}
	return err;
}

static struct mmio_platform_data mmio_config = {

	.platform_init = mmio_platform_init,
	.platform_exit = mmio_platform_exit,
	.power_enable = mmio_power_enable,
	.power_disable = mmio_power_disable,
	.clock_enable = mmio_clock_enable,
	.clock_disable = mmio_clock_disable,
	.config_i2c_pins = mmio_config_i2c_pins,
	.config_xshutdown_pins = mmio_config_xshutdown_pins,
	.set_xshutdown = mmio_set_xshutdown,
	.sia_base = U8500_SIA_BASE,
	.cr_base = U8500_CR_BASE
};


struct platform_device ux500_mmio_device = {
	.name = MMIO_NAME,
	.id = -1,
	.dev = {
		.platform_data = &mmio_config,
	}
};
