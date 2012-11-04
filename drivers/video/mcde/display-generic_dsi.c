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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/err.h>

#include <video/mcde_display.h>
#include <video/mcde_display-generic_dsi.h>

#define DSI_HS_FREQ_HZ 420160000
#define DSI_LP_FREQ_HZ 19200000

static int generic_platform_enable(struct mcde_display_device *dev)
{
	struct mcde_display_generic_platform_data *pdata =
		dev->dev.platform_data;

	dev_dbg(&dev->dev, "%s: Reset & power on generic display\n", __func__);

	if (pdata->regulator) {
		if (regulator_enable(pdata->regulator) < 0) {
			dev_err(&dev->dev, "%s:Failed to enable regulator\n"
				, __func__);
			return -EINVAL;
		}
	}
	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, pdata->reset_high);
	mdelay(pdata->reset_delay);
	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, !pdata->reset_high);

	return 0;
}

static int generic_platform_disable(struct mcde_display_device *dev)
{
	struct mcde_display_generic_platform_data *pdata =
		dev->dev.platform_data;

	dev_dbg(&dev->dev, "%s:Reset & power off generic display\n", __func__);

	if (pdata->regulator) {
		if (regulator_disable(pdata->regulator) < 0) {
			dev_err(&dev->dev, "%s:Failed to disable regulator\n"
				, __func__);
			return -EINVAL;
		}
	}
	return 0;
}

static int generic_set_power_mode(struct mcde_display_device *ddev,
	enum mcde_display_power_mode power_mode)
{
	int ret = 0;
	struct mcde_display_generic_platform_data *pdata =
		ddev->dev.platform_data;

	dev_dbg(&ddev->dev, "%s:Set Power mode\n", __func__);

	/* OFF -> STANDBY */
	if (ddev->power_mode == MCDE_DISPLAY_PM_OFF &&
		power_mode != MCDE_DISPLAY_PM_OFF) {

		if (ddev->platform_enable) {
			ret = ddev->platform_enable(ddev);
			if (ret)
				return ret;
		}

		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* STANDBY -> ON */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
		power_mode == MCDE_DISPLAY_PM_ON) {

		ret = mcde_dsi_dcs_write(ddev->chnl_state,
		DCS_CMD_EXIT_SLEEP_MODE, NULL, 0);
		if (ret)
			return ret;

		msleep(pdata->sleep_out_delay);

		ret = mcde_dsi_dcs_write(ddev->chnl_state,
			DCS_CMD_SET_DISPLAY_ON, NULL, 0);
		if (ret)
			return ret;

		ddev->power_mode = MCDE_DISPLAY_PM_ON;
		goto set_power_and_exit;
	}
	/* ON -> STANDBY */
	else if (ddev->power_mode == MCDE_DISPLAY_PM_ON &&
		power_mode <= MCDE_DISPLAY_PM_STANDBY) {
		ret = mcde_dsi_dcs_write(ddev->chnl_state,
			DCS_CMD_SET_DISPLAY_OFF, NULL, 0);
		if (ret)
			return ret;

		ret = mcde_dsi_dcs_write(ddev->chnl_state,
			DCS_CMD_ENTER_SLEEP_MODE, NULL, 0);
		if (ret)
			return ret;

		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* SLEEP -> OFF */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
		power_mode == MCDE_DISPLAY_PM_OFF) {
		if (ddev->platform_disable) {
			ret = ddev->platform_disable(ddev);
			if (ret)
				return ret;
		}
		ddev->power_mode = MCDE_DISPLAY_PM_OFF;
	}

set_power_and_exit:
	mcde_chnl_set_power_mode(ddev->chnl_state, ddev->power_mode);

	return ret;
}

static int __devinit generic_probe(struct mcde_display_device *dev)
{
	int ret = 0;
	struct mcde_display_generic_platform_data *pdata =
		dev->dev.platform_data;
	struct mcde_port *port;

	if (pdata == NULL) {
		dev_err(&dev->dev, "%s:Platform data missing\n", __func__);
		return -EINVAL;
	}

	if (dev->port->type != MCDE_PORTTYPE_DSI) {
		dev_err(&dev->dev,
			"%s:Invalid port type %d\n",
			__func__, dev->port->type);
		return -EINVAL;
	}

	port = dev->port;
	port->phy.dsi.num_data_lanes = 2;
	/* If link2 is connected to e.g. HDMI this freq
	 * should be set to DSI_PLL_FREQ, DSI_PLL_FREQ / 2
	 * or DSI_PLL_FREQ / 4.
	 * All other settings will result in one of these freqs.
	 */
	port->phy.dsi.hs_freq = DSI_HS_FREQ_HZ;
	port->phy.dsi.lp_freq = DSI_LP_FREQ_HZ;

	if (!dev->platform_enable && !dev->platform_disable) {
		pdata->generic_platform_enable = true;
		if (pdata->reset_gpio) {
			ret = gpio_request(pdata->reset_gpio, NULL);
			if (ret) {
				dev_warn(&dev->dev,
					"%s:Failed to request gpio %d\n",
					__func__, pdata->reset_gpio);
				goto gpio_request_failed;
			}
			gpio_direction_output(pdata->reset_gpio,
				!pdata->reset_high);
		}
		if (pdata->regulator_id) {
			pdata->regulator = regulator_get(NULL,
				pdata->regulator_id);
			if (IS_ERR(pdata->regulator)) {
				ret = PTR_ERR(pdata->regulator);
				dev_warn(&dev->dev,
					"%s:Failed to get regulator '%s'\n",
					__func__, pdata->regulator_id);
				pdata->regulator = NULL;
				goto regulator_get_failed;
			}

			if (regulator_set_voltage(pdata->regulator,
					pdata->min_supply_voltage,
					pdata->max_supply_voltage) < 0) {
				int volt;

				dev_warn(&dev->dev,
					"%s:Failed to set voltage '%s'\n",
					__func__, pdata->regulator_id);
				volt = regulator_get_voltage(pdata->regulator);
				dev_warn(&dev->dev,
					"Voltage:%d\n", volt);
			}

			/*
			* When u-boot has display a startup screen.
			* U-boot has turned on display power however the
			* regulator framework does not know about that
			* This is the case here, the display driver has to
			* enable the regulator for the display.
			*/
			if (dev->power_mode == MCDE_DISPLAY_PM_STANDBY) {
				ret = regulator_enable(pdata->regulator);
				if (ret < 0) {
					dev_err(&dev->dev,
					"%s:Failed to enable regulator\n"
					, __func__);
					goto regulator_enable_failed;
				}
			}
		}
	}

	/* TODO: Remove when DSI send command uses interrupts */
	dev->prepare_for_update = NULL;
	dev->platform_enable = generic_platform_enable,
	dev->platform_disable = generic_platform_disable,
	dev->set_power_mode = generic_set_power_mode;

	dev_info(&dev->dev, "Generic display probed\n");

	goto out;
regulator_enable_failed:
regulator_get_failed:
	if (pdata->generic_platform_enable && pdata->reset_gpio)
		gpio_free(pdata->reset_gpio);
gpio_request_failed:
out:
	return ret;
}

static int __devexit generic_remove(struct mcde_display_device *dev)
{
	struct mcde_display_generic_platform_data *pdata =
		dev->dev.platform_data;

	dev->set_power_mode(dev, MCDE_DISPLAY_PM_OFF);

	if (!pdata->generic_platform_enable)
		return 0;

	if (pdata->regulator)
		regulator_put(pdata->regulator);
	if (pdata->reset_gpio) {
		gpio_direction_input(pdata->reset_gpio);
		gpio_free(pdata->reset_gpio);
	}

	return 0;
}

#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
static int generic_resume(struct mcde_display_device *ddev)
{
	int ret;

	/* set_power_mode will handle call platform_enable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_STANDBY);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to resume display\n"
			, __func__);
	return ret;
}

static int generic_suspend(struct mcde_display_device *ddev, pm_message_t state)
{
	int ret;

	/* set_power_mode will handle call platform_disable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to suspend display\n"
			, __func__);
	return ret;
}
#endif

static struct mcde_display_driver generic_driver = {
	.probe	= generic_probe,
	.remove = generic_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
	.suspend = generic_suspend,
	.resume = generic_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
	.driver = {
		.name	= "mcde_disp_generic",
	},
};

/* Module init */
static int __init mcde_display_generic_init(void)
{
	pr_info("%s\n", __func__);

	return mcde_display_driver_register(&generic_driver);
}
module_init(mcde_display_generic_init);

static void __exit mcde_display_generic_exit(void)
{
	pr_info("%s\n", __func__);

	mcde_display_driver_unregister(&generic_driver);
}
module_exit(mcde_display_generic_exit);

MODULE_AUTHOR("Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ST-Ericsson MCDE generic DCS display driver");
