/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson MCDE Sony acx424akp DCS display driver
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
#include <video/mcde_display-sony_acx424akp_dsi.h>

static int display_read_deviceid(struct mcde_display_device *dev, u16 *id)
{
	struct mcde_chnl_state *chnl;

	u8  id1, id2, id3;
	int len = 1;
	int ret = 0;
	int readret = 0;

	dev_dbg(&dev->dev, "%s: Read device id of the display\n", __func__);

	/* Acquire MCDE resources */
	chnl = mcde_chnl_get(dev->chnl_id, dev->fifo, dev->port);
	if (IS_ERR(chnl)) {
		ret = PTR_ERR(chnl);
		dev_warn(&dev->dev, "Failed to acquire MCDE channel\n");
		goto out;
	}

	/* plugnplay: use registers DA, DBh and DCh to detect display */
	readret = mcde_dsi_dcs_read(chnl, 0xDA, (u32 *)&id1, &len);
	if (!readret)
		readret = mcde_dsi_dcs_read(chnl, 0xDB, (u32 *)&id2, &len);
	if (!readret)
		readret = mcde_dsi_dcs_read(chnl, 0xDC, (u32 *)&id3, &len);

	if (readret) {
		dev_info(&dev->dev,
			"mcde_dsi_dcs_read failed to read display ID\n");
		goto read_fail;
	}

	*id = (id3 << 8) | id2;
read_fail:
	/* close  MCDE channel */
	mcde_chnl_put(chnl);
out:
	return 0;
}

static int sony_acx424akp_platform_enable(struct mcde_display_device *dev)
{
	struct mcde_display_sony_acx424akp_platform_data *pdata =
		dev->dev.platform_data;

	dev_dbg(&dev->dev, "%s: Reset & power on sony display\n", __func__);

	if (pdata->regulator) {
		if (regulator_enable(pdata->regulator) < 0) {
			dev_err(&dev->dev, "%s:Failed to enable regulator\n"
				, __func__);
			return -EINVAL;
		}
	}
	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, pdata->reset_high);
	msleep(pdata->reset_delay);	/* as per sony lcd spec */
	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, !pdata->reset_high);
	msleep(pdata->reset_low_delay);	/* as per sony lcd spec */
	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, pdata->reset_high);
	msleep(pdata->reset_delay);	/* as per sony lcd spec */

	return 0;
}

static int sony_acx424akp_platform_disable(struct mcde_display_device *dev)
{
	struct mcde_display_sony_acx424akp_platform_data *pdata =
		dev->dev.platform_data;

	dev_dbg(&dev->dev, "%s:Reset & power off sony display\n", __func__);

	if (pdata->regulator) {
		if (regulator_disable(pdata->regulator) < 0) {
			dev_err(&dev->dev, "%s:Failed to disable regulator\n"
				, __func__);
			return -EINVAL;
		}
	}
	return 0;
}

static int sony_acx424akp_set_power_mode(struct mcde_display_device *ddev,
	enum mcde_display_power_mode power_mode)
{
	int ret = 0;
	struct mcde_display_sony_acx424akp_platform_data *pdata =
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
	ret = mcde_chnl_set_power_mode(ddev->chnl_state, ddev->power_mode);
	return ret;
}

static int __devinit sony_acx424akp_probe(struct mcde_display_device *dev)
{
	int ret = 0;
	u16 id;
	struct mcde_display_sony_acx424akp_platform_data *pdata =
		dev->dev.platform_data;

	if (pdata == NULL || !pdata->reset_gpio) {
		dev_err(&dev->dev, "%s:Platform data missing\n", __func__);
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	port = dev->port;
	di->reset_gpio = pdata->reset_gpio;
	di->port.type = MCDE_PORTTYPE_DSI;
	di->port.dcs_addr_mode.normal = 0x00;
	di->port.dcs_addr_mode.hor_flip = 0x40;
	di->port.mode = MCDE_PORTMODE_CMD;
	di->port.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP;
	di->port.sync_src = dev->port->sync_src;
	if (dev->port->sync_src == MCDE_SYNCSRC_TE0 ||
				dev->port->sync_src == MCDE_SYNCSRC_TE1) {
		di->port.vsync_polarity = VSYNC_ACTIVE_HIGH;
		di->port.vsync_clock_div = 0;
		di->port.vsync_min_duration = 0;
		di->port.vsync_max_duration = 0;
	}
	di->port.frame_trig = dev->port->frame_trig;
	di->port.phy.dsi.num_data_lanes = 2;
	di->port.link = port->link;
	di->port.phy.dsi.host_eot_gen = true;
	/* TODO: Move UI to mcde_hw.c when clk_get_rate(dsi) is done */
	di->port.phy.dsi.ui = 9;

	ret = gpio_request(di->reset_gpio, NULL);
	if (WARN_ON(ret))
		goto gpio_request_failed;

	gpio_direction_output(di->reset_gpio, 1);
	di->regulator = regulator_get(&dev->dev, IO_REGU);
	if (IS_ERR(di->regulator)) {
		ret = PTR_ERR(di->regulator);
		di->regulator = NULL;
		goto regulator_get_failed;
	}

	if (!dev->platform_enable && !dev->platform_disable) {
		pdata->sony_acx424akp_platform_enable = true;
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
			regulator_set_voltage(pdata->regulator,
					pdata->min_supply_voltage,
					pdata->max_supply_voltage);
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
	dev->platform_enable = sony_acx424akp_platform_enable,
	dev->platform_disable = sony_acx424akp_platform_disable,
	dev->set_power_mode = sony_acx424akp_set_power_mode;

	ret = display_read_deviceid(dev, &id);

	switch (id) {
	case DISPLAY_SONY_ACX424AKP:
		pdata->disp_panel = DISPLAY_SONY_ACX424AKP;
		dev_info(&dev->dev, "Sony ACX424AKP display (ID 0x%.4X) \
								probed\n", id);
		goto out;

	default:
		pdata->disp_panel = DISPLAY_NONE;
		dev_warn(&dev->dev, "Display not recognized\n");
		break;
	}

regulator_enable_failed:
regulator_get_failed:
	if (pdata->sony_acx424akp_platform_enable && pdata->reset_gpio)
		gpio_free(pdata->reset_gpio);
gpio_request_failed:
out:
	return ret;
}

static int __devexit sony_acx424akp_remove(struct mcde_display_device *dev)
{
	struct mcde_display_sony_acx424akp_platform_data *pdata =
		dev->dev.platform_data;

	dev->set_power_mode(dev, MCDE_DISPLAY_PM_OFF);

	if (!pdata->sony_acx424akp_platform_enable)
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
static int sony_acx424akp_resume(struct mcde_display_device *ddev)
{
	int ret;

	/* set_power_mode will handle call platform_enable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_STANDBY);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to resume display\n"
			, __func__);
	return ret;
}

static int sony_acx424akp_suspend(struct mcde_display_device *ddev, \
							pm_message_t state)
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

static struct mcde_display_driver sony_acx424akp_driver = {
	.probe	= sony_acx424akp_probe,
	.remove = sony_acx424akp_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
	.suspend = sony_acx424akp_suspend,
	.resume = sony_acx424akp_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
	.driver = {
		.name	= "mcde_disp_sony_acx424akp",
	},
};

/* Module init */
static int __init mcde_display_sony_acx424akp_init(void)
{
	pr_info("%s\n", __func__);

	return mcde_display_driver_register(&sony_acx424akp_driver);
}
module_init(mcde_display_sony_acx424akp_init);

static void __exit mcde_display_sony_acx424akp_exit(void)
{
	pr_info("%s\n", __func__);

	mcde_display_driver_unregister(&sony_acx424akp_driver);
}
module_exit(mcde_display_sony_acx424akp_exit);

MODULE_AUTHOR("Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ST-Ericsson MCDE Sony ACX424AKP DCS display driver");
