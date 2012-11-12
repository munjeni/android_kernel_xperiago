/*
 * Copyright (C) 2008-2009 ST-Ericsson
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/amba/serial.h>
#include <linux/spi/spi.h>
#include <linux/hsi/hsi.h>
#include <linux/mfd/core.h>
#include <linux/mfd/abx500.h>
#include <linux/regulator/ab8500.h>
#include <linux/mfd/tc3589x.h>
#include <linux/mfd/abx500/ab8500-gpio.h>
#include <linux/console.h>
#include <linux/regulator/fixed.h>
#include <linux/leds-lp5521.h>
#include <linux/input.h>
#include <linux/smsc911x.h>
#include <linux/gpio_keys.h>
#include <linux/delay.h>
#include <linux/mfd/abx500/ab8500-denc.h>
#include <linux/spi/stm_msp.h>
#include <linux/leds_pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/leds.h>
#include <linux/mfd/abx500/ux500_sysctrl.h>
#include <linux/memblock.h>
#include <linux/input/abx500-accdet.h>
#include <linux/input/ab8505_micro_usb_iddet.h>
#include "pins.h"

#ifdef CONFIG_INPUT_LSM303DLH_ACCELEROMETER
#include <linux/lsm303dlh_acc.h>
#endif
#ifdef CONFIG_INPUT_LSM303DLHC_ACCELEROMETER
#include <linux/lsm303dlhc_acc.h>
#endif
#ifdef CONFIG_INPUT_LSM303DLHC_ACCELEROMETER_LT
#include <linux/lsm303dlhc_acc_lt.h>
#endif
#ifdef CONFIG_INPUT_LSM303DLH_MAGNETOMETER
#include <linux/lsm303dlh_mag.h>
#endif
#ifdef CONFIG_INPUT_LPS331AP
#include <linux/lps331ap.h>
#endif
#ifdef CONFIG_INPUT_L3G4200D
#include <linux/l3g4200d_gyr.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/gpio-nomadik.h>
#include <plat/i2c.h>
#include <plat/ste_dma40.h>
#include <plat/pincfg.h>

#include <mach/hardware.h>
#include <mach/setup.h>
#include <mach/devices.h>
#include <mach/irqs.h>
#include <mach/ste-dma40-db8500.h>
#ifdef CONFIG_U8500_SIM_DETECT
#include <mach/sim_detect.h>
#endif
#include <mach/crypto-ux500.h>
#include <mach/pm.h>

#ifdef CONFIG_AV8100
#include <video/av8100.h>
#endif

#include "pins-db8500.h"
#include "devices-db8500.h"
#include "board-mop500.h"
#include "board-mop500-regulators.h"
#include "board-mop500-bm.h"
#include "board-mop500-wlan.h"
#include "board-ux500-usb.h"
#include "board-rio-grande-leds.h"
#include "board-rio-grande-keypad.h"

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_SPI
#include <linux/cyttsp.h>
#include <linux/delay.h>
#include <mach/touch_panel.h>
#endif

/* TODO: This is incorrectly switched! */
#ifdef CONFIG_SEMC_RMI4_BUS
#include <linux/rmi4/rmi4.h>
#endif
#if defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR) ||		\
	defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR_MODULE)
#include <linux/rmi4/rmi4_spi.h>
#endif

#ifdef CONFIG_INPUT_APDS9702
#include <linux/apds9702.h>
#endif

#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
#include <mach/simple_remote_ux500_pf.h>
#endif

#ifdef CONFIG_LM3560
#include <linux/lm3560.h>
#endif

#ifdef CONFIG_LM3561
#include <linux/lm3561.h>
#endif

#ifdef CONFIG_INPUT_NOA3402
#include <linux/input/noa3402.h>
#endif

#ifdef CONFIG_LEDS_LM3533
#include <linux/leds-lm3533_ng.h>
#endif

#ifdef CONFIG_NFC_PN544
#include <linux/pn544.h>
#endif

#ifdef CONFIG_INPUT_SO340010_TOUCH_KEY
#include <linux/so340010.h>
#endif

#ifdef CONFIG_SENSORS_TSL2772
#include "../../../drivers/staging/taos/tsl277x.h"
#endif

#define CRASH_LOGS_START 0x1FF00000
#define CRASH_LOGS_SIZE SZ_1M

#ifdef CONFIG_RAMDUMP_CRASH_LOGS
#define RAMDUMP_CRASH_LOGS_SIZE  (128 * SZ_1K)
#define RAMDUMP_CRASH_LOGS_START (CRASH_LOGS_START)
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define RAM_CONSOLE_SIZE    (128 * SZ_1K)
#define RAM_CONSOLE_START   (CRASH_LOGS_START + CRASH_LOGS_SIZE \
			     - RAM_CONSOLE_SIZE)
#endif

#define PRCM_DEBUG_NOPWRDOWN_VAL	0x194
#define ARM_DEBUG_NOPOWER_DOWN_REQ	1

#ifdef CONFIG_AB8500_DENC
static struct ab8500_denc_platform_data ab8500_denc_pdata = {
	.ddr_enable = true,
	.ddr_little_endian = false,
};
#endif


static struct ab8500_gpio_platform_data ab8500_gpio_pdata = {
	.gpio_base		= AB8500_PIN_GPIO(1),
	.irq_base		= MOP500_AB8500_VIR_GPIO_IRQ_BASE,
	/*
	 * config_reg is the initial configuration of ab8500 pins.
	 * The pins can be configured as GPIO or alt functions based
	 * on value present in GpioSel1 to GpioSel6 and AlternatFunction
	 * register. This is the array of 7 configuration settings.
	 * One has to compile time decide these settings. Below is the
	 * explanation of these setting
	 * GpioSel1 = 0x0F => Pin GPIO1 (SysClkReq2)
	 *                    Pin GPIO2 (SysClkReq3)
	 *                    Pin GPIO3 (SysClkReq4)
	 *                    Pin GPIO4 (SysClkReq6) are configured as GPIO
	 * GpioSel2 = 0x9E => Pins GPIO10 to GPIO13 are configured as GPIO
	 * GpioSel3 = 0x80 => Pin GPIO24 (SysClkReq7) is configured as GPIO
	 * GpioSel4 = 0x01 => Pin GPIO25 (SysClkReq8) is configured as GPIO
	 * GpioSel5 = 0x78 => Pin GPIO36 (ApeSpiClk)
	 *		      Pin GPIO37 (ApeSpiCSn)
	 *		      Pin GPIO38 (ApeSpiDout)
	 *		      Pin GPIO39 (ApeSpiDin) are configured as GPIO
	 * GpioSel6 = 0x02 => Pin GPIO42 (SysClkReq5) is configured as GPIO
	 * AlternaFunction = 0x00 => If Pins GPIO10 to 13 are not configured
	 * as GPIO then this register selectes the alternate functions
	 */
	.config_reg     = {0x0F, 0x9E, 0x80, 0x01, 0x78, 0x02, 0x00},

	/*
	 * config_direction allows for the initial GPIO direction to
	 * be set. For Snowball we set GPIO26 to output.
	 */
	.config_direction  = {0x00, 0x00, 0x00, 0x02, 0x00, 0x00},

	/*
	 * config_pullups allows for the intial configuration of the
	 * GPIO pullup/pulldown configuration.
	 */
	.config_pullups    = {0xE0, 0x01, 0x00, 0x00, 0x00, 0x00},
};

static struct ab8500_sysctrl_platform_data ab8500_sysctrl_pdata = {
	/*
	 * SysClkReq1RfClkBuf - SysClkReq8RfClkBuf
	 * The initial values should not be changed because of the way
	 * the system works today
	 */
	.initial_req_buf_config
			= {0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00},
};

#ifdef CONFIG_INPUT_AB8500_ACCDET
static struct abx500_accdet_platform_data ab8500_accdet_pdata = {
	.btn_keycode = KEY_MEDIA,
	.accdet1_dbth = ACCDET1_TH_1200mV | ACCDET1_DB_70ms,
	.accdet2122_th = ACCDET21_TH_1000mV | ACCDET22_TH_1000mV,
	.video_ctrl_gpio = AB8500_PIN_GPIO(35),
};
#endif

#ifdef CONFIG_MODEM_U8500
static struct platform_device u8500_modem_dev = {
	.name = "u8500-modem",
	.id   = 0,
	.dev  = {
		.platform_data = NULL,
	},
};
#endif

#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
static struct resource simple_remote_resources[] = {
	{
		.name = "ACC_DETECT_1DB_F",
		.start = AB8500_INT_ACC_DETECT_1DB_F,
		.end = AB8500_INT_ACC_DETECT_1DB_F,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "ACC_DETECT_1DB_R",
		.start = AB8500_INT_ACC_DETECT_1DB_R,
		.end = AB8500_INT_ACC_DETECT_1DB_R,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "ACC_DETECT_22DB_F",
		.start = AB8500_INT_ACC_DETECT_22DB_F,
		.end = AB8500_INT_ACC_DETECT_22DB_F,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "ACC_DETECT_22DB_R",
		.start = AB8500_INT_ACC_DETECT_22DB_R,
		.end = AB8500_INT_ACC_DETECT_22DB_R,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "ACC_DETECT_21DB_F",
		.start = AB8500_INT_ACC_DETECT_21DB_F,
		.end = AB8500_INT_ACC_DETECT_21DB_F,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "ACC_DETECT_21DB_R",
		.start = AB8500_INT_ACC_DETECT_21DB_R,
		.end = AB8500_INT_ACC_DETECT_21DB_R,
		.flags = IORESOURCE_IRQ,
	}
};
#endif /* CONFIG_SIMPLE_REMOTE_PLATFORM */

struct mfd_cell ab8500_extra_devs[] = {
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
	{
		.name = SIMPLE_REMOTE_PF_NAME,
		.num_resources = ARRAY_SIZE(simple_remote_resources),
		.resources = simple_remote_resources,
		.platform_data = &simple_remote_pf_data,
		.pdata_size = sizeof(simple_remote_pf_data),
	}
#endif /* CONFIG_SIMPLE_REMOTE_PLATFORM */
};

static void ab8500_extra_init(struct ab8500 *ab)
{
	int ret = mfd_add_devices(ab->dev, 0, ab8500_extra_devs,
				  ARRAY_SIZE(ab8500_extra_devs), NULL,
				  ab->irq_base);

	if (ret)
		pr_err("%s - Failed to register extra AB8500 MFD devices\n",
		       __func__);
	else
		pr_info("%s - Successfully registered extra AB8500 MFD "
			"devices\n", __func__);
}

static struct ab8500_platform_data ab8500_platdata = {
	.irq_base	= MOP500_AB8500_IRQ_BASE,
	.regulator	= &ab8500_regulator_plat_data,
#ifdef CONFIG_AB8500_DENC
	.denc		= &ab8500_denc_pdata,
#endif
	.battery	= &ab8500_bm_data,
	.charger	= &ab8500_charger_plat_data,
	.btemp		= &ab8500_btemp_plat_data,
	.fg		= &ab8500_fg_plat_data,
	.chargalg	= &ab8500_chargalg_plat_data,
	.gpio		= &ab8500_gpio_pdata,
	.sysctrl	= &ab8500_sysctrl_pdata,
	.pwmled		= &ab8500_pwmled_plat_data,
#ifdef CONFIG_INPUT_AB8500_ACCDET
	.accdet = &ab8500_accdet_pdata,
#endif
#ifdef CONFIG_PM
	.pm_power_off = true,
#endif
	.thermal_time_out = 20, /* seconds */
#ifdef CONFIG_INPUT_AB8505_MICRO_USB_DETECT
	.iddet = &iddet_adc_val_list,
#endif
	.init = ab8500_extra_init,
/*	.version = AB8500_VERSION_AB8500, */
};

static struct resource ab8500_resources[] = {
	[0] = {
		.start	= IRQ_DB8500_AB8500,
		.end	= IRQ_DB8500_AB8500,
		.flags	= IORESOURCE_IRQ
	}
};

struct platform_device ab8500_device = {
	.name = "ab8500-i2c",
	.id = 0,
	.dev = {
		.platform_data = &ab8500_platdata,
	},
	.num_resources = 1,
	.resource = ab8500_resources,
};

#ifdef CONFIG_AV8100
static int av8100_plat_init(void)
{
	struct ux500_pins *pins;
	int res;

	pins = ux500_pins_get("av8100-hdmi");
	if (!pins) {
		res = -EINVAL;
		goto failed;
	}

	res = ux500_pins_enable(pins);
	if (res != 0)
		goto failed;

	return res;

failed:
	pr_err("%s failed\n", __func__);
	return res;
}

static int av8100_plat_exit(void)
{
	struct ux500_pins *pins;
	int res;

	pins = ux500_pins_get("av8100-hdmi");
	if (!pins) {
		res = -EINVAL;
		goto failed;
	}
	res = ux500_pins_disable(pins);
	if (res != 0)
		goto failed;
	return res;

failed:
	pr_err("%s failed\n", __func__);
	return res;
}

static struct av8100_platform_data av8100_plat_data = {
	.init			= av8100_plat_init,
	.exit			= av8100_plat_exit,
	.irq			= NOMADIK_GPIO_TO_IRQ(192),
	.reset			= MOP500_HDMI_RST_GPIO,
	.inputclk_id		= "sysclk2",
	.regulator_pwr_id	= "hdmi_1v8",
	.alt_powerupseq		= false,
	.mclk_freq		= 3, /* MCLK_RNG_31_38 */
};
#endif

int regulator_enable_handler(struct regulator *r, const char *func_str)
{
	int rc, enabled;

	if (IS_ERR_OR_NULL(r)) {
		rc = r ? PTR_ERR(r) : -EINVAL;
		dev_err(NULL, "%s: regulator invalid",
			func_str ? func_str : "?");
		return rc;
	}

	rc = regulator_enable(r);
	if (!rc)
		return rc;

	enabled = regulator_is_enabled(r);
	if (enabled > 0) {
		dev_warn(NULL, "%s: regulator already enabled",
			func_str ? func_str : "?");
		rc = 0;
	} else if (enabled == 0) {
		dev_err(NULL, "%s: regulator still disabled",
			func_str ? func_str : "?");
	} else {
		dev_err(NULL, "%s: regulator status error %d",
			func_str ? func_str : "?", enabled);
	}
	return rc;
}

int regulator_disable_handler(struct regulator *r, const char *func_str)
{
	int rc, enabled;

	if (IS_ERR_OR_NULL(r)) {
		rc = r ? PTR_ERR(r) : -EINVAL;
		dev_err(NULL, "%s: regulator invalid",
			func_str ? func_str : "?");
		return rc;
	}

	rc = regulator_disable(r);
	if (!rc)
		return rc;

	enabled = regulator_is_enabled(r);
	if (enabled == 0) {
		dev_warn(NULL, "%s: regulator already disabled",
			func_str ? func_str : "?");
		rc = 0;
	} else if (enabled > 0) {
		dev_err(NULL, "%s: regulator still enabled",
			func_str ? func_str : "?");
	} else {
		dev_err(NULL, "%s: regulator status error %d",
			func_str ? func_str : "?", enabled);
	}
	return rc;
}

#ifdef CONFIG_INPUT_LSM303DLH_ACCELEROMETER
static struct lsm303dlh_acc_platform_data lsm303dlh_acc_platform_data = {
	.range = LSM303_RANGE_2G,
	.poll_interval_ms = 100,
	.irq_pad = 75,
	.power_on = NULL,
	.power_off = NULL,
};
#endif

#if defined(CONFIG_INPUT_LSM303DLHC_ACCELEROMETER) || \
	defined(CONFIG_INPUT_LSM303DLHC_ACCELEROMETER_LT) || \
	defined(CONFIG_INPUT_LSM303DLH_MAGNETOMETER) || \
	defined(CONFIG_INPUT_L3G4200D)
static int platform_power_config(struct device *dev, bool enable,
			struct regulator **regulator, char *regulator_id)
{
	int rc = 0;
	dev_dbg(dev, "%s\n", __func__);

	if (enable) {
		if (*regulator == NULL) {
			dev_dbg(dev, "%s: get regulator %s\n",
							__func__, regulator_id);
			*regulator = regulator_get(NULL, regulator_id);
			if (IS_ERR(*regulator)) {
				rc = PTR_ERR(*regulator);
				dev_err(dev, "%s: Failed to get regulator %s\n",
							__func__, regulator_id);
				return rc;
			}
		}
		rc = regulator_set_voltage(*regulator, 2800000, 2800000);
		if (rc) {
			dev_err(dev, "%s: unable to set voltage "
						"rc = %d!\n", __func__, rc);
			goto exit;
		}
	} else {
		goto exit;
	}

	return rc;

exit:
	regulator_put(*regulator);
	*regulator = NULL;
	return rc;
}

#endif

#ifdef CONFIG_INPUT_LSM303DLHC_ACCELEROMETER
static struct regulator *acc_regulator;

static int power_config_acc(struct device *dev, bool enable)
{
	int rc;

	dev_dbg(dev, "%s enable = %d\n", __func__, enable);

	rc = platform_power_config(dev, enable, &acc_regulator, "v-lsm303dlhc");

	return rc;
}

static int power(struct device *dev, enum lsm303dlhc_acc_power_sate pwr_state)
{
	int rc = -ENOSYS;

	dev_dbg(dev, "%s pwr_state = %d\n", __func__, pwr_state);

	if (pwr_state == LSM303DLHC_STANDBY)
		goto exit;
	else if (pwr_state == LSM303DLHC_PWR_ON)
		rc = regulator_enable_handler(acc_regulator, __func__);
	else if (pwr_state == LSM303DLHC_PWR_OFF)
		rc = regulator_disable_handler(acc_regulator, __func__);

exit:
	return rc;
}

static struct lsm303dlhc_acc_platform_data lsm303dlhc_acc_platform_data = {
	.range = 2,
	.poll_interval_ms = 100,
	.mode = MODE_POLL,
	.irq_pad = 75,
	.power = power,
	.power_config = power_config_acc,
};
#endif

#ifdef CONFIG_INPUT_LSM303DLHC_ACCELEROMETER_LT
static struct regulator *acc_regulator;

static int power_config_acc_lt(struct device *dev, bool enable)
{
	int rc;

	dev_dbg(dev, "%s enable = %d\n", __func__, enable);

	rc = platform_power_config(dev, enable, &acc_regulator, "v-lsm303dlhc");

	return rc;
}

static int power_lt(struct device *dev,
			enum lsm303dlhc_acc_lt_power_state pwr_state)
{
	int rc = -ENOSYS;

	dev_dbg(dev, "%s pwr_state = %d\n", __func__, pwr_state);

	if (pwr_state == LSM303DLHC_LT_STANDBY) {
		goto exit;
	} else if (pwr_state == LSM303DLHC_LT_PWR_ON) {
		rc = regulator_enable_handler(acc_regulator, __func__);
		usleep_range(1000, 1000);
	} else if (pwr_state == LSM303DLHC_LT_PWR_OFF) {
		rc = regulator_disable_handler(acc_regulator, __func__);
	}

exit:
	return rc;

}

static struct lsm303dlhc_acc_lt_platform_data
				lsm303dlhc_acc_lt_platform_data = {
	.range = 2,
	.poll_interval_ms = 100,
	.power = power_lt,
	.power_config = power_config_acc_lt,
};
#endif

static struct ab3550_platform_data ab3550_plf_data = {
        .irq = {
                .base = 0,
                .count = 0,
        },
        .dev_data = {
        },
        .dev_data_sz = {
        },
        .init_settings = NULL,
        .init_settings_sz = 0,
};

#if defined(CONFIG_LM3560) || defined(CONFIG_LM3561)
#define LM3560_HW_RESET_GPIO 6
static int lm356x_pwr(struct device *dev, bool request)
{
	dev_dbg(dev, "%s: request %d\n", __func__, request);
	if (request) {
		gpio_set_value(LM3560_HW_RESET_GPIO, 1);
		udelay(20);
	} else
		gpio_set_value(LM3560_HW_RESET_GPIO, 0);
	return 0;
}

static int lm356x_platform_init(struct device *dev, bool request)
{
	int rc;

	if (request) {
		rc = gpio_request(LM3560_HW_RESET_GPIO, "LM356x hw reset");
		if (rc)
			goto err;
	} else {
		rc = 0;
err:
		gpio_free(LM3560_HW_RESET_GPIO);
	}
	if (rc)
		dev_err(dev, "%s: failed rc %d\n", __func__, rc);
	return rc;
}
#endif

#ifdef CONFIG_LM3560
static struct lm3560_platform_data lm3560_platform_data = {
	.power			= lm356x_pwr,
	.platform_init          = lm356x_platform_init,
	.led_nums		= 2,
	.strobe_trigger		= LM3560_STROBE_TRIGGER_EDGE,
	.privacy_terminate	= LM3560_PRIVACY_MODE_TURN_BACK,
	.privacy_led_nums	= 1,
	.privacy_blink_period	= 0, /* No bliking */
	.current_limit		= 2100000, /* uA */
	.flash_sync		= LM3560_SYNC_ON,
	.strobe_polarity	= LM3560_STROBE_POLARITY_HIGH,
	.ledintc_pin_setting	= LM3560_LEDINTC_NTC_THERMISTOR_INPUT,
	.tx1_polarity		= LM3560_TX1_POLARITY_HIGH,
	.tx2_polarity		= LM3560_TX2_POLARITY_HIGH,
	.hw_torch_mode		= LM3560_HW_TORCH_MODE_DISABLE,
};
#endif /* CONFIG_LM3560 */

#ifdef CONFIG_LM3561
static struct lm3561_platform_data lm3561_platform_data = {
	.power			= lm356x_pwr,
	.platform_init          = lm356x_platform_init,
	.led_nums		= 1,
	.strobe_trigger		= LM3561_STROBE_TRIGGER_LEVEL,
	.current_limit		= 1000000, /* uA
				   selectable value are 1500mA or 1000mA.
				   if set other value,
				   it assume current limit is 1000mA.
				*/
	.flash_sync		= LM3561_SYNC_ON,
	.strobe_polarity	= LM3561_STROBE_POLARITY_HIGH,
	.ledintc_pin_setting	= LM3561_LEDINTC_NTC_THERMISTOR_INPUT,
	.tx1_polarity		= LM3561_TX1_POLARITY_HIGH,
	.tx2_polarity		= LM3561_TX2_POLARITY_HIGH,
	.hw_torch_mode		= LM3561_HW_TORCH_MODE_DISABLE,
};
#endif /* CONFIG_LM3561 */

#ifdef CONFIG_INPUT_LSM303DLH_MAGNETOMETER
static struct regulator *mag_regulator;

static int power_config_mag(struct device *dev, bool enable)
{
	int rc;

	dev_dbg(dev, "%s enable = %d\n", __func__, enable);

	rc = platform_power_config(dev, enable, &mag_regulator, "v-lsm303dlh");

	return rc;
}

static int power_on_mag(struct device *dev)
{
	int rc;

	dev_dbg(dev, "%s\n", __func__);

	rc = regulator_enable_handler(mag_regulator, __func__);
	return rc;
}

static int power_off_mag(struct device *dev)
{
	int rc;

	dev_dbg(dev, "%s\n", __func__);

	rc = regulator_disable_handler(mag_regulator, __func__);
	return rc;
}

static struct lsm303dlh_mag_platform_data lsm303dlh_mag_platform_data = {
	.range = LSM303_RANGE_8200mG,
	.poll_interval_ms = 100,
	.power_on = power_on_mag,
	.power_off = power_off_mag,
	.power_config = power_config_mag,
};
#endif

#ifdef CONFIG_INPUT_LPS331AP
static struct lps331ap_prs_platform_data lps331ap_platform_data = {
	.init                 = NULL,
	.exit                 = NULL,
	.power_on             = NULL,
	.power_off            = NULL,
	.poll_interval        = 200,
	.min_interval         = 40,
};
#endif

#ifdef CONFIG_INPUT_L3G4200D
static struct regulator *gyro_regulator;

static int power_config_gyro(struct device *dev, bool enable)
{
	int rc;

	dev_dbg(dev, "%s enable = %d\n", __func__, enable);

	rc = platform_power_config(dev, enable, &gyro_regulator, "v-l3g4200d");

	return rc;
}

static int power_on_gyro(struct device *dev)
{
	int rc;

	dev_dbg(dev, "%s\n", __func__);

	rc = regulator_enable_handler(gyro_regulator, __func__);
	return rc;
}

static int power_off_gyro(struct device *dev)
{
	int rc;

	dev_dbg(dev, "%s\n", __func__);

	rc = regulator_disable_handler(gyro_regulator, __func__);
	return rc;
}

static struct l3g4200d_gyr_platform_data l3g4200d_gyro_platform_data = {
	.poll_interval = 100,
	.min_interval = 40,

	.fs_range = L3G4200D_FS_2000DPS,

	.axis_map_x =  0,
	.axis_map_y = 1,
	.axis_map_z = 2,

	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,

	.init = NULL,
	.exit = NULL,
	.power_on = power_on_gyro,
	.power_off = power_off_gyro,
	.power_config = power_config_gyro,
};
#endif

#ifdef CONFIG_INPUT_APDS9702
#define APDS9702_DOUT_GPIO	89
static int apds9702_gpio_setup(int request)
{
	int rc = 0;

	if (request) {
		rc = gpio_request(APDS9702_DOUT_GPIO, "apds9702_dout");
		if (rc) {
			printk(KERN_ERR "%s: gpio_request failed!", __func__);
			return rc;
		}

		rc = gpio_direction_input(APDS9702_DOUT_GPIO);
		if (rc) {
			printk(KERN_ERR "%s: gpio_direction_input failed!",
				   __func__);
			goto exit;
		}

		/*
		 * request_threaded_irq will be called in the driver,
		 * so free gpio for now...
		 */
	}

exit:
	gpio_free(APDS9702_DOUT_GPIO);
	return rc;
}

static void apds9702_hw_config(int enable)
{
}

static struct apds9702_platform_data apds9702_pdata = {
	.gpio_dout	= APDS9702_DOUT_GPIO,
	.is_irq_wakeup	= 1,
	.hw_config	= apds9702_hw_config,
	.gpio_setup	= apds9702_gpio_setup,
	.ctl_reg = {
		.trg	= 1,
		.pwr	= 1,
		.burst	= 7,
		.frq	= 3,
		.dur	= 2,
		.th		= 15,
		.rfilt	= 0,
	},
	.phys_dev_path	= "/sys/bus/i2c/devices/2-0054",
};
#endif

#ifdef CONFIG_INPUT_NOA3402
#define NOA3402_GPIO	89

static struct noa3402_platform_data noa3402_pdata;

static int noa3402_hw_setup(struct noa3402_chip *chip, bool enable)
{
	int ret = 0;

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (!chip) {
		pr_err("%s: no device\n", __func__);
		return -ENODEV;
	}

	if (enable) {
		ret = gpio_request(noa3402_pdata.gpio, "noa3402");
		if (ret) {
			pr_err("%s: gpio_request failed!", __func__);
			goto exit;
		}

		ret = gpio_direction_input(noa3402_pdata.gpio);
		if (ret) {
			pr_err("%s: gpio_direction_input failed!", __func__);
			goto exit_free_gpio;
		}

		chip->noa_regulator = regulator_get(NULL,
						noa3402_pdata.regulator_id);
		if (IS_ERR(chip->noa_regulator)) {
			ret = PTR_ERR(chip->noa_regulator);
			pr_err("%s: Failed to get reg '%s'\n", __func__,
						noa3402_pdata.regulator_id);
			goto exit_free_gpio;
		}
		goto exit;
	} else {
		regulator_put(chip->noa_regulator);
		chip->noa_regulator = NULL;
	}
exit_free_gpio:
	gpio_free(noa3402_pdata.gpio);
exit:
	return ret;
}

static int noa3402_pwr_enable(struct noa3402_chip *chip, bool enable)
{
	int ret = 0;

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (!chip || !chip->noa_regulator) {
		pr_err("%s: no device\n", __func__);
		return -ENODEV;
	}

	if (enable) {
		pr_debug("%s: enable '%s'\n", __func__,
						noa3402_pdata.regulator_id);
		/* We assume that the regulator has been initialized
		   with the correct voltage since it is shared */
		ret = regulator_enable_handler(chip->noa_regulator, __func__);
	} else {
		pr_debug("%s: disable '%s'\n", __func__,
						noa3402_pdata.regulator_id);
		ret = regulator_disable_handler(chip->noa_regulator, __func__);
	}
	return ret;
}

static struct noa3402_platform_data noa3402_pdata = {
	.gpio				= NOA3402_GPIO,
	.regulator_id			= "v-noa3402",
	.pwm_sensitivity		= PWM_SENSITIVITY_STD,
	.pwm_res			= PWM_RES_8_BIT,
	.pwm_type			= PWM_TYPE_LOG,
	.ps_filter_nbr_correct		= 1,
	.ps_filter_nbr_measurements	= 1,
	.ps_led_current			= LED_CURRENT_MA_TO_REG(160),
	.ps_integration_time		= PS_INTEGRATION_300_US,
	.ps_interval			= PS_INTERVAL_MS_TO_REG(50),
	.als_integration_time		= ALS_INTEGRATION_100_MS,
	.als_interval			= ALS_INTERVAL_MS_TO_REG(500),
	.is_irq_wakeup			= 1,
	.phys_dev_path			= "/sys/bus/i2c/devices/2-0037",
	.pwr_enable			= noa3402_pwr_enable,
	.hw_setup			= noa3402_hw_setup,
};

#endif

#ifdef CONFIG_SENSORS_TSL2772
#define TSL2772_GPIO	89
static struct regulator *tsl2772_vreg;
static int board_tsl2772_init(struct device *dev)
{
	int ret;
	dev_dbg(dev, "%s\n", __func__);
	ret = gpio_request(TSL2772_GPIO, dev_name(dev));
	if (ret) {
		dev_err(dev, "%s: gpio_request failed!", __func__);
		goto exit;
	}
	ret = gpio_direction_input(TSL2772_GPIO);
	if (ret) {
		dev_err(dev, "%s: gpio_direction_input failed!", __func__);
		goto exit_free_gpio;
	}
	tsl2772_vreg = regulator_get(NULL, "v-tsl2772");
	if (IS_ERR(tsl2772_vreg)) {
		ret = PTR_ERR(tsl2772_vreg);
		tsl2772_vreg = NULL;
		dev_err(dev, "%s: Failed to get reg '%s'\n", __func__,
				"v-tsl2772");
		goto exit_free_gpio;
	}
	return 0;
exit_free_gpio:
	gpio_free(TSL2772_GPIO);
exit:
	return ret;
}

static void board_tsl2772_teardown(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	gpio_free(TSL2772_GPIO);
	if (tsl2772_vreg) {
		regulator_put(tsl2772_vreg);
		tsl2772_vreg = NULL;
	}
}

static int board_tsl2772_power(struct device *dev, enum tsl2772_pwr_state state)
{
	int ret = 0;

	dev_dbg(dev, "%s: state %d\n", __func__, state);
	if (state == POWER_ON && tsl2772_vreg)
		ret = regulator_enable_handler(tsl2772_vreg, __func__);
	else if (state == POWER_OFF && tsl2772_vreg)
		ret = regulator_disable_handler(tsl2772_vreg, __func__);
	else
		dev_info(dev, "%s: nothing to do\n", __func__);
	return ret;
}


struct tsl2772_platform_data tsl2772_data = {
	.platform_power = board_tsl2772_power,
	.platform_init = board_tsl2772_init,
	.platform_teardown = board_tsl2772_teardown,
	.prox_name = "tsl2772_proximity",
	.als_name = "tsl2772_als",
	.raw_settings = NULL,
	.parameters = {
		.prox_th_min = 255,
		.prox_th_max = 480,
		.als_gate = 10,
	},
	.als_can_wake = false,
	.proximity_can_wake = true,
};
#endif

#ifdef CONFIG_LEDS_LM3533
#define LM3533_HWEN_GPIO 93
static int lm3533_setup(struct device *dev)
{
	int rc = gpio_request(LM3533_HWEN_GPIO, "lm3533_hwen");
	if (rc)
		dev_err(dev, "failed to request gpio %d\n", LM3533_HWEN_GPIO);
	return rc;
}
static void lm3533_teardown(struct device *dev)
{
	gpio_free(LM3533_HWEN_GPIO);
	return;
}
static int lm3533_power_on(struct device *dev)
{
	gpio_set_value(LM3533_HWEN_GPIO, 1);
	return 0;
}
static int lm3533_power_off(struct device *dev)
{
	gpio_set_value(LM3533_HWEN_GPIO, 0);
	return 0;
}

struct lm3533_startup_brightenss lm3533_start_br[] =  {
	{
		.intf_name = "lcd-backlight",
		.brightness = 255,
	},
	{
		.intf_name = NULL,
	}
};

static struct lm3533_platform_data lm3533_pdata = {
	.b_cnf = {
		[LM3533_CBNKA] = {
			.pwm = 0x3f,
			.ctl = LM3533_HVA_MAP_LIN | LM3533_HVA_BR_CTL,
			.fsc =  I_UA_TO_FSC(20200),
			.iname = "lcd-backlight",
		},
		[LM3533_CBNKB] = {
			.pwm = 0,
			.ctl = LM3533_HVA_MAP_LIN | LM3533_HVB_BR_CTL,
			.fsc =  I_UA_TO_FSC(0),
			.iname = "not-connected",
		},
		[LM3533_CBNKC] = {
			.pwm = 0,
			.ctl = LM3533_LV_MAP_LIN | LM3533_LV_BR_CTL,
			.fsc =  I_UA_TO_FSC(17000),
			.iname = "red",
		},
		[LM3533_CBNKD] = {
			.pwm = 0,
			.ctl = LM3533_LV_MAP_LIN | LM3533_LV_BR_CTL,
			.fsc =  I_UA_TO_FSC(19400),
			.iname = "green",
		},
		[LM3533_CBNKE] = {
			.pwm = 0,
			.ctl = LM3533_LV_MAP_LIN | LM3533_LV_BR_CTL,
			.fsc =  I_UA_TO_FSC(9800),
			.iname = "blue",
		},
		[LM3533_CBNKF] = {
			.pwm = 0,
			.ctl = LM3533_LV_MAP_LIN | LM3533_LV_BR_CTL,
			.fsc =  I_UA_TO_FSC(0),
			.iname = "not-connected",
		},
	},
	.l_cnf = {
		[LM3533_HVLED1] = {
			.connected = true, .cpout = true, .bank =  LM3533_CBNKA,
		},
		[LM3533_HVLED2] = {
			.connected = true, .cpout = true, .bank =  LM3533_CBNKB,
		},
		[LM3533_LVLED1] = {
			.connected = true, .cpout = true, .bank =  LM3533_CBNKC,
		},
		[LM3533_LVLED2] = {
			.connected = true, .cpout = true, .bank =  LM3533_CBNKD,
		},
		[LM3533_LVLED3] = {
			.connected = true, .cpout = true, .bank =  LM3533_CBNKE,
		},
		[LM3533_LVLED4] = {
			.connected = true, .cpout = true, .bank =  LM3533_CBNKF,
		},
		[LM3533_LVLED5] = {
			.connected = true, .cpout = true, .bank =  LM3533_CBNKF,
		},
	},
	.ovp_boost_pwm = LM3533_BOOST_500KHZ | LM3533_OVP_32V | LM3533_PWM_HIGH,
	.setup = lm3533_setup,
	.teardown = lm3533_teardown,
	.power_on = lm3533_power_on,
	.power_off = lm3533_power_off,
	.startup_brightness = lm3533_start_br,
};
#endif

static struct i2c_board_info __initdata pdp_i2c0_devices[] = {
#ifdef CONFIG_AV8100
	{
		I2C_BOARD_INFO("av8100", 0xE0 >> 1),
		.platform_data = &av8100_plat_data,
	},
#endif
};

static struct i2c_board_info __initdata pdp_i2c1_devices[] = {
	{
		/* AB3550 */
		I2C_BOARD_INFO("ab3550", 0x94 >> 1),
		.irq = -1,
		.platform_data = &ab3550_plf_data,
	},
#ifdef CONFIG_INPUT_LPS331AP
	{
		I2C_BOARD_INFO(LPS331AP_PRS_DEV_NAME, 0xB8 >> 1),
		.platform_data = &lps331ap_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_L3G4200D
	{
		I2C_BOARD_INFO(L3G4200D_DEV_NAME, 0xD0 >> 1),
		.platform_data = &l3g4200d_gyro_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_BMP180
	{
		I2C_BOARD_INFO("bmp180", 0xEE >> 1)
	},
#endif
};

#ifdef CONFIG_INPUT_SO340010_TOUCH_KEY

#define SO340010_TOUCH_KEY_IRQ_GPIO 97

#ifdef SO340010_TOUCH_KEY_IRQ_GPIO
#define SO340010_TOUCH_KEY_IRQ GPIO_TO_IRQ(SO340010_TOUCH_KEY_IRQ_GPIO)
#endif

static int so340010_setup(struct device *dev)
{
	int rc;

	dev_dbg(dev, "%s\n", __func__);
#ifdef SO340010_TOUCH_KEY_IRQ_GPIO
	rc = gpio_request(SO340010_TOUCH_KEY_IRQ_GPIO, "so340010_irq");
	if (rc)
		dev_err(dev, "Failded requesting GPIO-%d\n",
				SO340010_TOUCH_KEY_IRQ_GPIO);
#else
	rc = 0;
#endif
	return rc;
}

static void so340010_teardown(struct device *dev)
{
#ifdef SO340010_TOUCH_KEY_IRQ_GPIO
	gpio_free(SO340010_TOUCH_KEY_IRQ_GPIO);
#endif
	dev_dbg(dev, "%s\n", __func__);
}

static struct so340010_config so340010_config = {
	.pin = {
		[0] = {
			.mode = SO340010_LED,
			.u.led.max_level = 31, /* full brightness */
			.u.led.effect = SO340010_RAMP_UP_DN,
			.u.led.name = "so34-led0",
		},
		[1] = {
			.mode = SO340010_LED,
			.u.led.max_level = 31,
			.u.led.effect = SO340010_RAMP_UP_DN,
			.u.led.name = "so34-led1",
		},
		[2] = {
			.mode = SO340010_LED,
			.u.led.max_level = 31,
			.u.led.effect = SO340010_RAMP_UP_DN,
			.u.led.name = "so34-led2",
		},
	},
	.btn = {
		[0] = {
			.enable = 1,
			.sensitivity = 0xd8,
			.code = KEY_MENU,
			.map2gpio = false,
			.suspend = SO340010_SUSPEND_EARLY,
		},
		[1] = {
			.enable = 1,
			.sensitivity = 0xd8,
			.code = KEY_HOME,
			.map2gpio = false,
			.suspend = SO340010_SUSPEND_EARLY,
		},
		[2] = {
			.enable = 1,
			.sensitivity = 0xd8,
			.code = KEY_BACK,
			.map2gpio = false,
			.suspend = SO340010_SUSPEND_EARLY,
		},
	},
	.period_a_x12_5_ms = 5,
	.period_b_x12_5_ms = 5,
	.btn_out_mode = SO340010_ACTIVE_HI,
	.btn_mode = SO340010_INRESTRICTED,
	.btn_map2dir = SO340010_MAP2DATA,
	.gpi_suspend = SO340010_SUSPEND_EARLY,
	.gpi_enable = false,
#ifdef SO340010_TOUCH_KEY_IRQ
	.polling_ms = SO340010_NO_POLLING,
#else
	.polling_ms = 20,
#endif
	.input_name = "so34-buttons",
	.enable_wake = true,
	.platform_setup = so340010_setup,
	.platform_teardown = so340010_teardown,
};
#endif /*CONFIG_INPUT_SO340010_TOUCH_KEY*/

static struct i2c_board_info __initdata pdp_i2c2_devices[] = {
#ifdef CONFIG_INPUT_SO340010_TOUCH_KEY
	{
		I2C_BOARD_INFO(SO340010_DEV_NAME, 0x58 >> 1),
#ifdef SO340010_TOUCH_KEY_IRQ
		.irq		= SO340010_TOUCH_KEY_IRQ,
#else
		.irq		= -1,
#endif
		.platform_data	= &so340010_config,
	},
#endif
#ifdef CONFIG_INPUT_LSM303DLH_ACCELEROMETER
	{
		I2C_BOARD_INFO(LSM303DLH_ACC_DEV_NAME, 0x30 >> 1),
		.platform_data = &lsm303dlh_acc_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_LSM303DLHC_ACCELEROMETER
	{
		I2C_BOARD_INFO(LSM303DLHC_ACC_DEV_NAME, 0x32 >> 1),
		.platform_data = &lsm303dlhc_acc_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_LSM303DLHC_ACCELEROMETER_LT
	{
		I2C_BOARD_INFO(LSM303DLHC_ACC_LT_DEV_NAME, 0x32 >> 1),
		.platform_data = &lsm303dlhc_acc_lt_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_LSM303DLH_MAGNETOMETER
	{
		I2C_BOARD_INFO(LSM303DLH_MAG_DEV_NAME, 0x3c >> 1),
		.platform_data = &lsm303dlh_mag_platform_data,
	},
#endif
#ifdef CONFIG_LEDS_AS3677
	{
		I2C_BOARD_INFO("as3677", 0x80 >> 1),
		.platform_data = &as3677_pdata,
		.irq = 92,
	},
#endif
#ifdef CONFIG_LEDS_AS3676
	{
		I2C_BOARD_INFO("as3676", 0x80 >> 1),
		.platform_data = &as3676_platform_data,
	},
#endif
#ifdef CONFIG_LEDS_AS3676_VENDOR
	{
		I2C_BOARD_INFO("as3676", 0x80 >> 1),
		.platform_data = &as3676_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_APDS9702
	{
		I2C_BOARD_INFO(APDS9702_NAME, 0xA8 >> 1),
		.platform_data = &apds9702_pdata,
		.type = APDS9702_NAME,
	},
#endif
#ifdef CONFIG_INPUT_NOA3402
	{
		I2C_BOARD_INFO(NOA3402_NAME, 0x6E >> 1),
		.platform_data = &noa3402_pdata,
		.type = NOA3402_NAME,
	},
#endif
#ifdef CONFIG_LM3560
	{
		I2C_BOARD_INFO(LM3560_DRV_NAME, 0xA6 >> 1),
		.platform_data = &lm3560_platform_data,
	},
#endif
#ifdef CONFIG_LM3561
	{
		I2C_BOARD_INFO(LM3561_DRV_NAME, 0xA6 >> 1),
		.platform_data = &lm3561_platform_data,
	},
#endif
#ifdef CONFIG_LEDS_LM3533
	{
		I2C_BOARD_INFO(LM3533_DEV_NAME, 0x36),
		.platform_data = &lm3533_pdata,
		.irq = -1,
	},
#endif
#ifdef CONFIG_SENSORS_TSL2772
	{
		I2C_BOARD_INFO("tsl2772", 0x39),
		.platform_data = &tsl2772_data,
		.irq = GPIO_TO_IRQ(TSL2772_GPIO),
	},
#endif
};

#ifdef CONFIG_NFC_PN544
#define PN544_VREG "nfc_1v8"

#ifdef CONFIG_REGULATOR
struct regulator *r;
#endif

static int pn544_driver_opened(void)
{
#ifdef CONFIG_REGULATOR
	int ret;

	if (r == NULL) {
		r = regulator_get(NULL, PN544_VREG);
		if (IS_ERR(r)) {
			pr_err("%s: Not able to find regulator.\n", __func__);
			r = NULL;
			return -ENODEV;
		}
	}
	if (regulator_is_enabled(r)) {
		pr_err("%s: Regulator already enabled.\n", __func__);
		return -ENODEV;
	}

	ret = regulator_enable(r);
	if (ret < 0) {
		pr_err("%s: Not able to enable regulator. Error : %d\n",
		__func__, ret);
		regulator_disable(r);
		return -ENODEV;
	}
#endif
	return 0;
}

static void pn544_driver_closed(void)
{
#ifdef CONFIG_REGULATOR
	if (r != NULL) {
		if (regulator_is_enabled(r))
			regulator_disable(r);
		regulator_put(r);
		r = NULL;
	}
#endif
}

static int pn544_chip_config(enum pn544_state state, void *dev)
{
	switch (state) {
	case PN544_STATE_OFF:
		gpio_set_value(GPIO91_GPIO, 0);
		gpio_set_value(GPIO4_GPIO, 0);
		usleep_range(50000, 50000);
		break;
	case PN544_STATE_ON:
		gpio_set_value(GPIO91_GPIO, 0);
		gpio_set_value(GPIO4_GPIO, 1);
		usleep_range(10000, 10000);
		break;
	case PN544_STATE_FWDL:
		gpio_set_value(GPIO91_GPIO, 1);
		gpio_set_value(GPIO4_GPIO, 0);
		usleep_range(10000, 10000);
		gpio_set_value(GPIO4_GPIO, 1);
		break;
	default:
		pr_err("%s: undefined state %d\n", __func__, state);
		return -EINVAL;
	}
	return 0;
}

static int pn544_gpio_request(void)
{
	int ret;

	ret = gpio_request(GPIO66_GPIO, "pn544_irq");
	if (ret)
		goto err_irq;
	ret = gpio_request(GPIO4_GPIO, "pn544_ven");
	if (ret)
		goto err_ven;
	ret = gpio_request(GPIO91_GPIO, "pn544_fw");
	if (ret)
		goto err_fw;
	return 0;
err_fw:
	gpio_free(GPIO4_GPIO);
err_ven:
	gpio_free(GPIO66_GPIO);
err_irq:
	pr_err("%s: gpio request err %d\n", __func__, ret);
	return ret;
}

static void pn544_gpio_release(void)
{
	gpio_free(GPIO4_GPIO);
	gpio_free(GPIO66_GPIO);
	gpio_free(GPIO91_GPIO);
}

static struct pn544_i2c_platform_data pn544_pdata = {
	.irq_type = IRQF_TRIGGER_RISING,
	.chip_config = pn544_chip_config,
	.driver_loaded = pn544_gpio_request,
	.driver_unloaded = pn544_gpio_release,
	.driver_opened = pn544_driver_opened,
	.driver_closed = pn544_driver_closed,
};
#endif

static struct i2c_board_info __initdata pdp_i2c3_devices[] = {
#ifdef CONFIG_NFC_PN544
	{
		/* Config-spec is 8-bit = 0x50, src-code need 7-bit => 0x28 */
		I2C_BOARD_INFO(PN544_DEVICE_NAME, 0x50 >> 1),
		.platform_data = &pn544_pdata,
		.irq = GPIO_TO_IRQ(GPIO66_GPIO)
	},
#endif
};

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_SPI

#define CYTTSP_IRQ_GPIO  88
#define CYTTSP_XRES_GPIO 94
#define CYTTSP_VREG "v-touch1"
#define CYTTSP_SPI_CS_GPIO 31
#define CYTTSP_VOLTAGE 2800000

static int cyttsp_wakeup(struct device *dev)
{
	int ret;

	ret = gpio_direction_output(CYTTSP_IRQ_GPIO, 0);
	if (ret) {
		pr_err("%s: Failed to request gpio_direction_output\n",
		__func__);
		return ret;
	}
	msleep(50);
	gpio_set_value(CYTTSP_IRQ_GPIO, 0);
	msleep(1);
	gpio_set_value(CYTTSP_IRQ_GPIO, 1);
	msleep(1);
	gpio_set_value(CYTTSP_IRQ_GPIO, 0);
	msleep(1);
	gpio_set_value(CYTTSP_IRQ_GPIO, 1);
	printk(KERN_INFO "%s: wakeup\n", __func__);
	ret = gpio_direction_input(CYTTSP_IRQ_GPIO);
	if (ret) {
		pr_err("%s: Failed to request gpio_direction_input\n",
		__func__);
		return ret;
	}
	msleep(50);
	return 0;
}


static struct regulator *cyttsp_reg;
static int cyttsp_init(int on, struct device *dev)
{
	int ret = 0;
	struct cyttsp_platform_data *data = dev->platform_data;

	if (on && !cyttsp_reg) {
		cyttsp_reg = regulator_get(NULL, CYTTSP_VREG);
		if (IS_ERR(cyttsp_reg)) {
			dev_err(dev, "Failed to get regulator '%s'\n",
					CYTTSP_VREG);
			ret = -ENODEV;
			goto regulator_get_failed;
		}
		ret = regulator_set_voltage(cyttsp_reg, CYTTSP_VOLTAGE,
				CYTTSP_VOLTAGE);
		if (ret < 0) {
			dev_err(dev, "Failed to set voltage on '%s'\n",
			       CYTTSP_VREG);
			ret = -ENODEV;
			goto regulator_set_failed;
		}
		ret = regulator_enable_handler(cyttsp_reg, __func__);
		if (ret < 0) {
			dev_err(dev, "Failed to enable regulator '%s'\n",
			       CYTTSP_VREG);
			ret = -ENODEV;
			goto regulator_set_failed;
		}
		ret = gpio_request(data->irq_gpio, "CYTTSP IRQ GPIO");
		if (ret) {
			dev_err(dev, "%s: Failed to request GPIO %d\n",
				__func__, data->irq_gpio);
			ret = -ENODEV;
			goto irq_gpio_req_failed;
		}
		ret = gpio_request(CYTTSP_XRES_GPIO, "CYTTSP XRES GPIO");
		if (ret) {
			dev_err(dev, "%s: Failed to request GPIO %d\n",
				__func__, CYTTSP_XRES_GPIO);
			ret = -ENODEV;
			goto xres_gpio_req_failed;
		}
		ret = gpio_request(CYTTSP_SPI_CS_GPIO, "CYTTSP_SPI_CS_GPIO");
		if (ret) {
			dev_err(dev, "%s: Failed to request GPIO %d\n",
				__func__, CYTTSP_SPI_CS_GPIO);
			ret = -ENODEV;
			goto cs_gpio_req_failed;
		}
		gpio_direction_input(data->irq_gpio);
		gpio_direction_output(CYTTSP_XRES_GPIO, 1);
		gpio_direction_output(CYTTSP_SPI_CS_GPIO, 1);
	} else if (!on && cyttsp_reg) {
		gpio_free(CYTTSP_SPI_CS_GPIO);
cs_gpio_req_failed:
		gpio_free(CYTTSP_XRES_GPIO);
xres_gpio_req_failed:
		gpio_free(data->irq_gpio);
irq_gpio_req_failed:
		regulator_disable_handler(cyttsp_reg, __func__);
regulator_set_failed:
		regulator_put(cyttsp_reg);
regulator_get_failed:
		cyttsp_reg = NULL;
	} else {
		dev_err(dev, "%s: on %d but cyttsp_reg %p\n", __func__,
				on, cyttsp_reg);
		ret = -EINVAL;
	}
	return ret;
}

static int cyttsp_xres(void)
{
	int polarity;
	int rc;

	rc = gpio_direction_input(CYTTSP_XRES_GPIO);
	if (rc) {
		pr_err("%s: failed to set direction input, %d\n",
		       __func__, rc);
		return -EIO;
	}
	polarity = gpio_get_value(CYTTSP_XRES_GPIO);
	pr_debug("%s: %d\n", __func__, polarity);
	rc = gpio_direction_output(CYTTSP_XRES_GPIO, polarity ^ 1);
	if (rc) {
		pr_err("%s: failed to set direction output, %d\n",
		       __func__, rc);
		return -EIO;
	}
	msleep(1);
	gpio_set_value(CYTTSP_XRES_GPIO, polarity);
	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_KEY
#define TT_KEY_BACK_FLAG	0x01
#define TT_KEY_MENU_FLAG	0x02
#define TT_KEY_HOME_FLAG	0x04

static struct input_dev *input_dev_cyttsp_key;

static int __init cyttsp_key_init(void)
{
	input_dev_cyttsp_key = input_allocate_device();
	if (!input_dev_cyttsp_key) {
		pr_err("Error, unable to alloc cyttsp key device\n");
		return -ENOMEM;
	}
	input_dev_cyttsp_key->name = "cyttsp_key";
	input_set_capability(input_dev_cyttsp_key, EV_KEY, KEY_MENU);
	input_set_capability(input_dev_cyttsp_key, EV_KEY, KEY_BACK);
	input_set_capability(input_dev_cyttsp_key, EV_KEY, KEY_HOME);
	if (input_register_device(input_dev_cyttsp_key)) {
		pr_err("Unable to register cyttsp key device\n");
		input_free_device(input_dev_cyttsp_key);
		return -ENODEV;
	}
	return 0;
}
module_init(cyttsp_key_init);

static int cyttsp_key_rpc_callback(u8 data[], int size)
{
	static u8 last;
	u8 toggled = last ^ data[0];

	if (toggled & TT_KEY_MENU_FLAG)
		input_report_key(input_dev_cyttsp_key, KEY_MENU,
			!!(*data & TT_KEY_MENU_FLAG));

	if (toggled & TT_KEY_BACK_FLAG)
		input_report_key(input_dev_cyttsp_key, KEY_BACK,
			!!(*data & TT_KEY_BACK_FLAG));

	if (toggled & TT_KEY_HOME_FLAG)
		input_report_key(input_dev_cyttsp_key, KEY_HOME,
			!!(*data & TT_KEY_HOME_FLAG));

	input_sync(input_dev_cyttsp_key);
	last = data[0];
	return 0;
}
#endif /* CONFIG_TOUCHSCREEN_CYTTSP_KEY */

static void cyttsp_spi_cs_control(u32 command)
{
	/* set the FRM signal, which is CS  - TODO */
	if (command == SSP_CHIP_SELECT)
		gpio_set_value(CYTTSP_SPI_CS_GPIO, 0);
	else if (command == SSP_CHIP_DESELECT)
		gpio_set_value(CYTTSP_SPI_CS_GPIO, 1);
}

struct pl022_config_chip cyttsp_chip_info = {
	.iface = SSP_INTERFACE_MOTOROLA_SPI,

	/* we can act as master only */
	.hierarchy = SSP_MASTER,

	/* Only valid in slave mode
	.slave_tx_disable = 1,
	*/

	/* clock freq. 133330000 / (cpsdvsr * (scr +1))
	*  cpsdvsr = 165, src=2,  freq=401596
	*  cpsdvsr = 128, src=0,  freq=1041640
	*  If 0 the spi master will calculate a value
	*  based on the slave speed
	*/
	.clk_freq = {
		.cpsdvsr = 0,
		.scr = 0,
	},

	.com_mode = INTERRUPT_TRANSFER,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_8,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	/*
	.clkdelay =
	*/
	.cs_control = cyttsp_spi_cs_control,
};
#endif

#if defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR) ||		\
	defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR_MODULE)

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/rmi4/rmi4_early_suspend.h>
#endif

#define RMI4_SPI_CS_GPIO	31
#define RMI4_SPI_IRQ_GPIO	88
#define RMI4_SPI_XRES_GPIO	94

#define RMI4_SPI_VREG_VDD "vio-touch" /* Shared with display */

#define RMI4_SPI_VDD_VOLTAGE 2800000

static int rmi4_gpio_init(struct device *dev, int on)
{
	int ret = 0;
	int reset;

	if (on) {
		ret = gpio_request(RMI4_SPI_CS_GPIO, "RMI_SPI CHIP SEL");
		if (ret) {
			dev_err(dev,
				"%s: Failed to request CS gpio %d\n", __func__,
				RMI4_SPI_CS_GPIO);
			goto err_cs_request;
		}

		ret = gpio_request(RMI4_SPI_XRES_GPIO, "RMI_SPI XRES");
		if (ret) {
			dev_err(dev, "%s: Failed to request reset gpio %d\n",
				__func__, RMI4_SPI_XRES_GPIO);
			goto err_xres_request;
		}

		/* Pullup is configured per product in *-pins.c */
		gpio_direction_output(RMI4_SPI_CS_GPIO, 1);

		/* Performing chip reset */
		/* This is temporary until we get response from Synaptics on
		 * how this should be done. Currently, pulling the line low once
		 * for 350ms doesn't work, so we kick the reset line a couple of
		 * times. This seems to work */
		for (reset = 0; reset < 10; reset++) {
			dev_dbg(dev, "%s - Resetting chip\n", __func__);
			gpio_direction_output(RMI4_SPI_XRES_GPIO, 0);
			msleep(10);
			gpio_direction_output(RMI4_SPI_XRES_GPIO, 1);
			msleep(10);
			dev_dbg(dev, "%s - Reset done\n", __func__);
		}

	} else {
		gpio_free(RMI4_SPI_XRES_GPIO);
err_xres_request:
		gpio_free(RMI4_SPI_CS_GPIO);
	}

err_cs_request:
	return ret;
}

static int rmi4_reg_init(struct device *dev, int on)
{
	int ret = 0;

	static struct regulator *rmi4_vdd;

	if (!rmi4_vdd)
		rmi4_vdd = regulator_get(NULL, RMI4_SPI_VREG_VDD);

	if (IS_ERR(rmi4_vdd)) {
		dev_err(dev,
			"%s - Failed to get VDD regulator '%s. Error : %ld'\n",
			__func__, RMI4_SPI_VREG_VDD, IS_ERR(rmi4_vdd) * -1);
		return -ENODEV;
	}

	if (on) {
		regulator_set_voltage(rmi4_vdd, RMI4_SPI_VDD_VOLTAGE,
				      RMI4_SPI_VDD_VOLTAGE);
		ret = regulator_enable_handler(rmi4_vdd, __func__);
		if (ret < 0) {
			dev_err(dev,
				"%s - Failed to enable VDD regulator '%s'. "
				"Error : %d\n",
				__func__, RMI4_SPI_VREG_VDD, ret);
		} else {
			msleep(10);
		}
	} else {
		regulator_disable_handler(rmi4_vdd, __func__);
		regulator_put(rmi4_vdd);
		rmi4_vdd = NULL;
	}

	return ret;
}

static int rmi4_gpio_config(struct device *dev, bool configure)
{
	int ret = rmi4_reg_init(dev, configure);
	if (ret)
		return ret;

	ret = rmi4_gpio_init(dev, configure);
	if (ret)
		rmi4_reg_init(dev, 0);

	return ret;
}

static int rmi4_force_chip_sel(struct device *dev, bool assert)
{
	struct rmi4_spi_adapter_platform_data *pdata =
		dev_get_platdata(dev);

	if (true == pdata->assert_level_low)
		assert = !assert;

	return gpio_direction_output(RMI4_SPI_CS_GPIO, assert);
}

static struct rmi4_function_data rmi4_extra_functions[] = {
#ifdef CONFIG_HAS_EARLYSUSPEND
	{
		.func_name = RMI4_3250_E_SUSP_NAME,
		.func_id = 0x01,
		.func_data = NULL,
	},
#endif
};

static struct rmi4_core_device_data rmi4_core_data = {
	.core_name = RMI4_CORE_DRIVER_NAME,
	.attn_gpio = RMI4_SPI_IRQ_GPIO,
	.irq_polarity = IRQF_TRIGGER_FALLING,
	.irq_is_shared = true,
	.num_functions = ARRAY_SIZE(rmi4_extra_functions),
	.func_data = rmi4_extra_functions,

};

static struct rmi4_spi_adapter_platform_data synaptics_platform_data = {
	.attn_gpio = RMI4_SPI_IRQ_GPIO,
	.irq_polarity = IRQF_TRIGGER_FALLING,
	.irq_is_shared = true,
	.gpio_config = rmi4_gpio_config,
	.assert_level_low = true,
	.spi_v2 = {
		.cs_assert = rmi4_force_chip_sel,
		.block_delay_us = 30,
		.split_read_block_delay_us = 30,
		.read_delay_us = 18,
		.write_delay_us = 18,
		.split_read_byte_delay_us = 10,
		.pre_delay_us = 0,
		.post_delay_us = 0,
	},
	.cdev_data = &rmi4_core_data,
};

static void rmi4_spi_cs_control(u32 command)
{
	/* Handled elsewhere to get the required behavior */
	/* This function is here to avoid warning log on SPI *
	 * transactions */
}

static struct pl022_config_chip rmi4_chip_info = {
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy = SSP_MASTER,
	/* clock freq. 133330000 / (cpsdvsr * (scr +1))
	*  cpsdvsr = 165, src=2,  freq=401596
	*  cpsdvsr = 128, src=0,  freq=1041640
	*  If 0 the spi master will calculate a value
	*  based on the slave speed
	*/
	.clk_freq = {
		.cpsdvsr = 0,
		.scr = 0,
	},
	.com_mode = POLLING_TRANSFER,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_8,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control = rmi4_spi_cs_control,
};
#endif /* CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR */

#define U8500_I2C_CONTROLLER(id, _slsu, _tft, _rft, clk, t_out, _sm)	\
static struct nmk_i2c_controller u8500_i2c##id##_data = { \
	/*				\
	 * slave data setup time, which is	\
	 * 250 ns,100ns,10ns which is 14,6,2	\
	 * respectively for a 48 Mhz	\
	 * i2c clock			\
	 */				\
	.slsu		= _slsu,	\
	/* Tx FIFO threshold */		\
	.tft		= _tft,		\
	/* Rx FIFO threshold */		\
	.rft		= _rft,		\
	/* std. mode operation */	\
	.clk_freq	= clk,		\
	/* Slave response timeout(ms) */\
	.timeout	= t_out,	\
	.sm		= _sm,		\
}

/*
 * The board uses 4 i2c controllers, initialize all of
 * them with slave data setup time of 250 ns,
 * Tx & Rx FIFO threshold values as 1 and standard
 * mode of operation
 */
U8500_I2C_CONTROLLER(0, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(1, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(2,	0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(3,	0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);

static void __init mop500_i2c_init(void)
{
	db8500_add_i2c0(&u8500_i2c0_data);
	db8500_add_i2c1(&u8500_i2c1_data);
	db8500_add_i2c2(&u8500_i2c2_data);
	db8500_add_i2c3(&u8500_i2c3_data);
}

/*
 * SPI3
 */
#if defined(CONFIG_TOUCHSCREEN_CYTTSP_SPI) ||			\
	defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR) ||	\
	defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR_MODULE)
#define NUM_SPI3_CLIENTS 1
static struct pl022_ssp_controller pdp_spi3_data = {
	.bus_id		= SPI023_3_CONTROLLER,
	.num_chipselect	= NUM_SPI3_CLIENTS,
};
#endif

static struct spi_board_info u8500_spi_devices[] = {
#ifdef CONFIG_TOUCHSCREEN_CYTTSP_SPI
	{
		.modalias = CY_SPI_NAME,
		/* Need to be 1.4 MHz - lower speed does
		 * not work when APE goes from 100% to 50%
		 */
		.max_speed_hz = 1400000,
		.chip_select = 0,
		.bus_num = SPI023_3_CONTROLLER,
		.controller_data = &cyttsp_chip_info,
		.platform_data = &cyttsp_data,
		.mode = SPI_MODE_0,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_CLEARPAD_SPI
	{
		.modalias = CLEARPADSPI_NAME,
		.max_speed_hz = 2 * 1000 * 1000,
		.chip_select = 0,
		.bus_num = SPI023_3_CONTROLLER,
		.controller_data = &clearpad_chip_info,
		.platform_data = &clearpad_data,
	},
#endif
#if defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR) ||		\
	defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR_MODULE)
	{
		.modalias = "rmi-spi",
		.max_speed_hz = 2000000,
		.chip_select = 0,
		.bus_num = SPI023_3_CONTROLLER,
		.controller_data = &rmi4_chip_info,
		.platform_data = &synaptics_platform_data,
	},
#endif
};

/* Force feedback vibrator device */
static struct platform_device ste_ff_vibra_device = {
	.name = "ste_ff_vibra"
};

#ifdef CONFIG_RAMDUMP_CRASH_LOGS
static struct resource ramdump_crash_logs_resources[] = {
	[0] = {
		.start  = RAMDUMP_CRASH_LOGS_START,
		.end    = RAMDUMP_CRASH_LOGS_START +
			RAMDUMP_CRASH_LOGS_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ramdump_crash_logs_device = {
	.name           = "ramdump_crash_logs",
	.id             = -1,
};
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resources[] = {
	[0] = {
		.start  = RAM_CONSOLE_START,
		.end    = RAM_CONSOLE_START + RAM_CONSOLE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
};
#endif

#if defined(CONFIG_RAMDUMP_CRASH_LOGS) || defined(CONFIG_ANDROID_RAM_CONSOLE)
static void crash_logs_reserve(void)
{
	if (memblock_reserve(CRASH_LOGS_START, CRASH_LOGS_SIZE)) {
		printk(KERN_ERR "Failed to reserve memory for CRASH_LOGS: "
		       "%dM@0x%.8X\n",
		       CRASH_LOGS_SIZE / SZ_1M, CRASH_LOGS_START);
		return;
	}
	memblock_free(CRASH_LOGS_START, CRASH_LOGS_SIZE);
	memblock_remove(CRASH_LOGS_START, CRASH_LOGS_SIZE);

#ifdef CONFIG_RAMDUMP_CRASH_LOGS
	ramdump_crash_logs_device.num_resources =
		ARRAY_SIZE(ramdump_crash_logs_resources);
	ramdump_crash_logs_device.resource =
		ramdump_crash_logs_resources;
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	ram_console_device.num_resources  = ARRAY_SIZE(ram_console_resources);
	ram_console_device.resource       = ram_console_resources;
#endif
}
#endif

#ifdef CONFIG_HSI
static struct hsi_board_info __initdata u8500_hsi_devices[] = {
	{
		.name = "hsi_char",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 1,
			.speed = 200000,
			{.arb_mode = HSI_ARB_RR},
		},
		.rx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 1,
			.speed = 200000,
			{.flow = HSI_FLOW_SYNC},
		},
	},
	{
		.name = "hsi_test",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 2,
			.speed = 100000,
			{.arb_mode = HSI_ARB_RR},
		},
		.rx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 2,
			.speed = 200000,
			{.flow = HSI_FLOW_SYNC},
		},
	},
	{
		.name = "cfhsi_v3_driver",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_STREAM,
			.channels = 2,
			.speed = 20000,
			{.arb_mode = HSI_ARB_RR},
		},
		.rx_cfg = {
			.mode = HSI_MODE_STREAM,
			.channels = 2,
			.speed = 200000,
			{.flow = HSI_FLOW_SYNC},
		},
	},
};
#endif

#ifdef CONFIG_U8500_SIM_DETECT
static struct sim_detect_platform_data sim_detect_pdata = {
	.irq_num		= MOP500_AB8500_VIR_GPIO_IRQ(6),
};
struct platform_device u8500_sim_detect_device = {
	.name	= "sim-detect",
	.id	= 0,
	.dev	= {
			.platform_data          = &sim_detect_pdata,
	},
};
#endif

static struct cryp_platform_data u8500_cryp1_platform_data = {
	.mem_to_engine = {
		.dir = STEDMA40_MEM_TO_PERIPH,
		.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
		.dst_dev_type = DB8500_DMA_DEV48_CAC1_TX,
		.src_info.data_width = STEDMA40_WORD_WIDTH,
		.dst_info.data_width = STEDMA40_WORD_WIDTH,
		.mode = STEDMA40_MODE_LOGICAL,
		.src_info.psize = STEDMA40_PSIZE_LOG_4,
		.dst_info.psize = STEDMA40_PSIZE_LOG_4,
	},
	.engine_to_mem = {
		.dir = STEDMA40_PERIPH_TO_MEM,
		.src_dev_type = DB8500_DMA_DEV48_CAC1_RX,
		.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
		.src_info.data_width = STEDMA40_WORD_WIDTH,
		.dst_info.data_width = STEDMA40_WORD_WIDTH,
		.mode = STEDMA40_MODE_LOGICAL,
		.src_info.psize = STEDMA40_PSIZE_LOG_4,
		.dst_info.psize = STEDMA40_PSIZE_LOG_4,
	}
};

static struct stedma40_chan_cfg u8500_hash_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV50_HAC1_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.mode = STEDMA40_MODE_LOGICAL,
	.src_info.psize = STEDMA40_PSIZE_LOG_16,
	.dst_info.psize = STEDMA40_PSIZE_LOG_16,
};

static struct hash_platform_data u8500_hash1_platform_data = {
	.mem_to_engine = &u8500_hash_dma_cfg_tx,
	.dma_filter = stedma40_filter,
};

/* add any platform devices here - TODO */
static struct platform_device *mop500_platform_devs[] __initdata = {
#ifdef CONFIG_U8500_SIM_DETECT
	&u8500_sim_detect_device,
#endif
	&u8500_shrm_device,
	&ste_ff_vibra_device,
#ifdef CONFIG_U8500_MMIO
	&ux500_mmio_device,
#endif
	&ux500_hwmem_device,
	&ux500_mcde_device,
	&u8500_dsilink_device[0],
	&u8500_dsilink_device[1],
	&u8500_dsilink_device[2],
	&ux500_b2r2_device,
	&ux500_b2r2_blt_device,
#ifdef CONFIG_STE_TRACE_MODEM
	&u8500_trace_modem,
#endif
#ifdef CONFIG_DB8500_MLOADER
	&mloader_fw_device,
#endif
#ifdef CONFIG_HSI
	&u8500_hsi_device,
#endif
#ifdef CONFIG_MODEM_U8500
	&u8500_modem_dev,
#endif
#ifdef CONFIG_RAMDUMP_CRASH_LOGS
	&ramdump_crash_logs_device,
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
};

/*
 * MSP-SPI
 */

#define NUM_MSP_CLIENTS 10

static struct stm_msp_controller pdp_msp2_spi_data = {
	.id		= 2,
	.num_chipselect	= NUM_MSP_CLIENTS,
	.base_addr	= U8500_MSP2_BASE,
	.device_name	= "msp2",
};

static void __init mop500_spi_init(void)
{
	db8500_add_msp2_spi(&pdp_msp2_spi_data);
#if defined(CONFIG_TOUCHSCREEN_CYTTSP_SPI) ||			\
	defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR) ||	\
	defined(CONFIG_SEMC_GENERIC_RMI4_SPI_ADAPTOR_MODULE)
	db8500_add_spi3(&pdp_spi3_data);
#endif
}

#ifdef CONFIG_STE_DMA40_REMOVE
static struct stedma40_chan_cfg uart0_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV13_UART0_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_BYTE_WIDTH,
	.dst_info.data_width = STEDMA40_BYTE_WIDTH,
};

static struct stedma40_chan_cfg uart0_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV13_UART0_TX,
	.src_info.data_width = STEDMA40_BYTE_WIDTH,
	.dst_info.data_width = STEDMA40_BYTE_WIDTH,
};

static struct stedma40_chan_cfg uart1_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV12_UART1_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_BYTE_WIDTH,
	.dst_info.data_width = STEDMA40_BYTE_WIDTH,
};

static struct stedma40_chan_cfg uart1_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV12_UART1_TX,
	.src_info.data_width = STEDMA40_BYTE_WIDTH,
	.dst_info.data_width = STEDMA40_BYTE_WIDTH,
};

static struct stedma40_chan_cfg uart2_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV11_UART2_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_BYTE_WIDTH,
	.dst_info.data_width = STEDMA40_BYTE_WIDTH,
};

static struct stedma40_chan_cfg uart2_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV11_UART2_TX,
	.src_info.data_width = STEDMA40_BYTE_WIDTH,
	.dst_info.data_width = STEDMA40_BYTE_WIDTH,
};
#endif


static pin_cfg_t mop500_pins_uart0[] = {
	GPIO0_U0_CTSn   | PIN_INPUT_PULLUP,
	GPIO1_U0_RTSn   | PIN_OUTPUT_HIGH,
	GPIO2_U0_RXD    | PIN_INPUT_PULLUP,
	GPIO3_U0_TXD    | PIN_OUTPUT_HIGH,
};

#define PRCC_K_SOFTRST_SET      0x18
#define PRCC_K_SOFTRST_CLEAR    0x1C
/* pl011 reset */
static void ux500_uart0_reset(void)
{
	void __iomem *prcc_rst_set, *prcc_rst_clr;

	prcc_rst_set = (void __iomem *)IO_ADDRESS(U8500_CLKRST1_BASE +
			PRCC_K_SOFTRST_SET);
	prcc_rst_clr = (void __iomem *)IO_ADDRESS(U8500_CLKRST1_BASE +
			PRCC_K_SOFTRST_CLEAR);

	/* Activate soft reset PRCC_K_SOFTRST_CLEAR */
	writel((readl(prcc_rst_clr) | 0x1), prcc_rst_clr);
	udelay(1);

	/* Release soft reset PRCC_K_SOFTRST_SET */
	writel((readl(prcc_rst_set) | 0x1), prcc_rst_set);
	udelay(1);
}

static void ux500_uart0_init(void)
{
	int ret;

	ret = nmk_config_pins(mop500_pins_uart0,
			ARRAY_SIZE(mop500_pins_uart0));
	if (ret < 0)
		pr_err("pl011: uart pins_enable failed\n");
}

static void ux500_uart0_exit(void)
{
	int ret;

	ret = nmk_config_pins_sleep(mop500_pins_uart0,
			ARRAY_SIZE(mop500_pins_uart0));
	if (ret < 0)
		pr_err("pl011: uart pins_disable failed\n");
}



static struct amba_pl011_data uart0_plat = {
#ifdef CONFIG_STE_DMA40_REMOVE
	.dma_filter = stedma40_filter,
	.dma_rx_param = &uart0_dma_cfg_rx,
	.dma_tx_param = &uart0_dma_cfg_tx,
#endif
	.init = ux500_uart0_init,
	.exit = ux500_uart0_exit,
	.reset = ux500_uart0_reset,
};

static struct amba_pl011_data uart1_plat = {
#ifdef CONFIG_STE_DMA40_REMOVE
	.dma_filter = stedma40_filter,
	.dma_rx_param = &uart1_dma_cfg_rx,
	.dma_tx_param = &uart1_dma_cfg_tx,
#endif
};

static struct amba_pl011_data uart2_plat = {
#ifdef CONFIG_STE_DMA40_REMOVE
	.dma_filter = stedma40_filter,
	.dma_rx_param = &uart2_dma_cfg_rx,
	.dma_tx_param = &uart2_dma_cfg_tx,
#endif
};

static void __init mop500_uart_init(void)
{
	db8500_add_uart0(&uart0_plat);
	db8500_add_uart1(&uart1_plat);
	db8500_add_uart2(&uart2_plat);
}

#define CONSOLE_NAME "ttyAMA"
#define CONSOLE_IX 2
#define CONSOLE_OPTIONS "115200n8"
static int __init setup_serial_console(char *console_flag)
{
	if (console_flag &&
		strlen(console_flag) >= 2 &&
		(console_flag[0] != '0' || console_flag[1] != '0'))
		add_preferred_console(CONSOLE_NAME,
					CONSOLE_IX,
					CONSOLE_OPTIONS);
	return 1;
}

/*
 * The S1 Boot configuration TA unit can specify that the serial console
 * enable flag will be passed as Kernel boot arg with tag babe09a9.
 */
__setup("semcandroidboot.babe09a9=", setup_serial_console);

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_SPI
static void __init cyttsp_data_set_callbacks(struct cyttsp_platform_data *pdata)
{
#ifdef CONFIG_TOUCHSCREEN_CYTTSP_KEY
	pdata->cust_spec = cyttsp_key_rpc_callback;
#endif /* CONFIG_TOUCHSCREEN_CYTTSP_KEY */
	pdata->wakeup = cyttsp_wakeup;
	pdata->init = cyttsp_init;
	pdata->reset = cyttsp_xres;
}
#endif /* CONFIG_TOUCHSCREEN_CYTTSP_SPI */

static void __init u8500_cryp1_hash1_init(void)
{
	db8500_add_cryp1(&u8500_cryp1_platform_data);
	db8500_add_hash1(&u8500_hash1_platform_data);
}

static void __init mop500_init_machine(void)
{
	u8500_init_devices();

	mop500_pins_init();

	mop500_regulator_init();

	u8500_cryp1_hash1_init();

#ifdef CONFIG_HSI
	hsi_register_board_info(u8500_hsi_devices,
				ARRAY_SIZE(u8500_hsi_devices));
#endif
	platform_add_devices(mop500_platform_devs,
				ARRAY_SIZE(mop500_platform_devs));

	mop500_i2c_init();
	mop500_msp_init();
	mop500_spi_init();
	mop500_uart_init();
	mop500_wlan_init();
	mop500_sdi_init();

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_SPI
	cyttsp_data_set_callbacks(&cyttsp_data);
#endif /* CONFIG_TOUCHSCREEN_CYTTSP_SPI */

#ifdef CONFIG_KEYBOARD_NOMADIK_SKE
	/*
	 * If a hw debugger is detected, do not load the ske driver
	 * since the gpio usage collides.
	 */
	if (!(prcmu_read(PRCM_DEBUG_NOPWRDOWN_VAL) &
	      ARM_DEBUG_NOPOWER_DOWN_REQ))
		db8500_add_ske_keypad(get_ske_keypad_data());
#endif

#ifdef CONFIG_ANDROID_STE_TIMED_VIBRA
	mop500_vibra_init();
#endif
	platform_device_register(&ab8500_device);

	i2c_register_board_info(0, ARRAY_AND_SIZE(pdp_i2c0_devices));
	i2c_register_board_info(1, ARRAY_AND_SIZE(pdp_i2c1_devices));
	i2c_register_board_info(2, ARRAY_AND_SIZE(pdp_i2c2_devices));
	i2c_register_board_info(3, ARRAY_AND_SIZE(pdp_i2c3_devices));

	spi_register_board_info(u8500_spi_devices,
				ARRAY_SIZE(u8500_spi_devices));

	/* This board has full regulator constraints */
	regulator_has_full_constraints();
}

/* Function to reserve memory regions for specific use */
static void __init riogrande_reserve(void)
{
#if defined(CONFIG_RAMDUMP_CRASH_LOGS) || defined(CONFIG_ANDROID_RAM_CONSOLE)
	crash_logs_reserve();
#endif
}

/* Stub function to make board-mop500-wlan.c compile */
int pins_for_u9500(void)
{
	return 0;
}

MACHINE_START(NOMADIK, "riogrande")
	/* Maintainer: Sony Ericsson */
	.boot_params	= 0x00000100,
	.map_io		= u8500_map_io,
	.reserve	= riogrande_reserve,
	.init_irq	= ux500_init_irq,
	.timer		= &ux500_timer,
	.init_machine	= mop500_init_machine,
MACHINE_END
