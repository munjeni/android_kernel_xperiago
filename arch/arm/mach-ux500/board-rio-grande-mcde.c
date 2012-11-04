/*
 * Copyright (C) ST-Ericsson AB 2010
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/mfd/abx500/ab8500-denc.h>
#include <linux/workqueue.h>
#include <linux/dispdev.h>
#include <linux/compdev.h>
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <mach/devices.h>
#include <video/av8100.h>
#include <video/mcde_display.h>
#include <video/mcde_display-panel_dsi.h>
#include <video/mcde_display-av8100.h>
#include <video/mcde_display-ab8500.h>
#include <video/mcde_fb.h>
#include <video/mcde_dss.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <mach/display_panel.h>
#include <plat/pincfg.h>
#include "pins-db8500.h"
#include "pins.h"

#define DSI_UNIT_INTERVAL_0	0x9
#define DSI_UNIT_INTERVAL_1	0x9
#define DSI_UNIT_INTERVAL_2	0x5

#define DSI_PLL_FREQ_HZ		840320000
/* Based on PLL DDR Freq at 798,72 MHz */
#define HDMI_FREQ_HZ		33280000
#define TV_FREQ_HZ		38400000

#define DSI_HS_FREQ_HZ		420160000
#define DSI_LP_FREQ_HZ		19200000

#define DDR_APE_OPP_TIMER	200
#define DDR_QOS_VALUE		50

#ifdef CONFIG_MCDE_DISPLAY_LOGO
extern int mcde_logo_init(void);
static bool shown_logo;
#endif

#ifdef CONFIG_U8500_TV_OUTPUT_AV8100
/* The initialization of hdmi disp driver must be delayed in order to
 * ensure that inputclk will be available (needed by hdmi hw) */
static struct delayed_work work_dispreg_hdmi;
#define DISPREG_HDMI_DELAY 6000
#endif

enum {
	PRIMARY_DISPLAY_ID,
	SECONDARY_DISPLAY_ID,
	FICTIVE_DISPLAY_ID,
	AV8100_DISPLAY_ID,
	MCDE_NR_OF_DISPLAYS
};

static int display_initialized_during_boot;

static int __init startup_graphics_setup(char *str)
{

	if (get_option(&str, &display_initialized_during_boot) != 1)
		display_initialized_during_boot = 0;

	switch (display_initialized_during_boot) {
	case 1:
		pr_info("Startup graphics support\n");
		break;
	case 0:
	default:
		pr_info("No startup graphics supported\n");
		break;
	};

	return 1;
}
__setup("startup_graphics=", startup_graphics_setup);

#if defined(CONFIG_U8500_TV_OUTPUT_AV8100)
static struct mcde_col_transform rgb_2_yCbCr_transform = {
	.matrix = {
		{0x0042, 0x0081, 0x0019},
		{0xffda, 0xffb6, 0x0070},
		{0x0070, 0xffa2, 0xffee},
	},
	.offset = {0x10, 0x80, 0x80},
};
#endif

static int setup_performance(struct mcde_display_device *ddev)
{
	struct panel_device *dev =
		container_of(ddev, struct panel_device, base);
	struct panel_platform_data *pdata = dev->base.dev.platform_data;
	struct panel_record *rd;
	static int old_ddr_qos_value;
	bool curr_rotation_90;
	s32 ddr_qos_value;

	/* Remove DDR request if no update after DDR_APE_OPP_TIMER ms */
	cancel_delayed_work(&dev->ddr_ape_timeout_work);
	schedule_delayed_work(&dev->ddr_ape_timeout_work,
					msecs_to_jiffies(DDR_APE_OPP_TIMER));

	curr_rotation_90 = mcde_chnl_is_rotated_90(ddev->chnl_state);
	rd = dev_get_drvdata(&dev->base.dev);

	if (rd && rd->panel && rd->panel->ddr_qos_value && curr_rotation_90)
		ddr_qos_value = rd->panel->ddr_qos_value;
	else
		ddr_qos_value = DDR_QOS_VALUE;

	if (!pdata->ddr_is_requested || old_ddr_qos_value != ddr_qos_value) {
		if (pdata->ddr_is_requested)
			prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP,
								"main_LCD");
		/* Continue even if we fail the request */
		if (prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, "main_LCD",
								ddr_qos_value))
			dev_err(&ddev->dev, "DDR OPP %d failed\n",
								ddr_qos_value);
		else
			pdata->ddr_is_requested = true;
	}
	old_ddr_qos_value = ddr_qos_value;

	if (rd && rd->panel && rd->panel->ape_qos_value) {
		if (curr_rotation_90 && !pdata->ape_is_requested) {
			/* Continue even if we fail the request */
			if (prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP,
				"main_LCD_ape", rd->panel->ape_qos_value))
				dev_err(&ddev->dev, "APE OPP %d failed\n",
						rd->panel->ape_qos_value);
			else
				pdata->ape_is_requested = true;
		} else if (pdata->ape_is_requested && !curr_rotation_90) {
			prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP,
								"main_LCD_ape");
			pdata->ape_is_requested = false;
		}
	}
	return 0;
}

static int extern_te_update(struct mcde_display_device *ddev,
							bool tripple_buffer)
{
	int ret = 0;
	bool switched_off_te = false;

	ret = setup_performance(ddev);

	if (ddev->power_mode != MCDE_DISPLAY_PM_ON && ddev->set_power_mode) {
		ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_INTERMEDIATE);
		if (ret < 0) {
			dev_warn(&ddev->dev,
				"%s:Failed to set power mode to intermediate\n",
				__func__);
		}
	}

	/* Temporary set SYNC_SRC to OFF in order to get first refresh out */
	if (ddev->power_mode != MCDE_DISPLAY_PM_ON) {
		mcde_chnl_update_sync_src(ddev->chnl_state, MCDE_SYNCSRC_OFF);
		switched_off_te = true;
		mcde_chnl_set_dirty(ddev->chnl_state);
	}

	ret = mcde_chnl_update(ddev->chnl_state, tripple_buffer);

	/* Set sync_src back to TE0 */
	if (switched_off_te) {
		mcde_chnl_update_sync_src(ddev->chnl_state, MCDE_SYNCSRC_TE0);
		switched_off_te = false;
		mcde_chnl_set_dirty(ddev->chnl_state);
	}

	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to update channel\n", __func__);
		return ret;
	}
	if (ddev->first_update && ddev->on_first_update)
		ddev->on_first_update(ddev);

	if (ddev->power_mode != MCDE_DISPLAY_PM_ON && ddev->set_power_mode) {
		ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_ON);
		if (ret < 0) {
			dev_warn(&ddev->dev,
				"%s:Failed to set power mode to on\n",
				__func__);
			return ret;
		}
	}

	dev_vdbg(&ddev->dev, "Overlay updated, chnl=%d\n", ddev->chnl_id);

	return 0;
}

static int panel_update(struct mcde_display_device *ddev, bool tripple_buffer)
{
	int ret = 0;

	ret = setup_performance(ddev);

	/* TODO: Dirty */
	if (ddev->prepare_for_update) {
		/* TODO: Send dirty rectangle */
		ret = ddev->prepare_for_update(ddev, 0, 0,
			ddev->native_x_res, ddev->native_y_res);
		if (ret < 0) {
			dev_warn(&ddev->dev,
				"%s:Failed to prepare for update\n", __func__);
			return ret;
		}
	}

	if (ddev->power_mode != MCDE_DISPLAY_PM_ON && ddev->set_power_mode) {
		ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_INTERMEDIATE);
		if (ret < 0) {
			dev_warn(&ddev->dev,
				"%s:Failed to set power mode to intermediate\n",
				__func__);
		}
	}

	/* TODO: Calculate & set update rect */
	ret = mcde_chnl_update(ddev->chnl_state, tripple_buffer);
	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to update channel\n", __func__);
		return ret;
	}
	if (ddev->first_update && ddev->on_first_update)
		ddev->on_first_update(ddev);

	if (ddev->power_mode != MCDE_DISPLAY_PM_ON && ddev->set_power_mode) {
		ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_ON);
		if (ret < 0) {
			dev_warn(&ddev->dev,
				"%s:Failed to set power mode to on\n",
				__func__);
			return ret;
		}
	}

	dev_vdbg(&ddev->dev, "Overlay updated, chnl=%d\n", ddev->chnl_id);

	return 0;
}

static int panel_platform_reset(struct mcde_display_device *ddev, bool level)
{
	int ret = 0;
	struct panel_platform_data *pdata = ddev->dev.platform_data;

	dev_dbg(&ddev->dev, "%s: Reset display driver, level = %d\n",
							__func__, level);

	if (pdata->reset_gpio) {
		ret = gpio_request(pdata->reset_gpio, NULL);
		if (ret) {
			dev_warn(&ddev->dev,
				"%s:Failed to request gpio %d\n",
				__func__, pdata->reset_gpio);
			goto out;
		}
		gpio_set_value(pdata->reset_gpio, level);
	}

out:
	if (pdata->reset_gpio)
		gpio_free(pdata->reset_gpio);
	return ret;
}

static int panel_platform_enable(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct panel_device *dev =
		container_of(ddev, struct panel_device, base);
	struct panel_platform_data *pdata = dev->base.dev.platform_data;

	dev_dbg(&ddev->dev, "%s: Reset & power on display driver\n",
								__func__);
	if (pdata->regulator_id && !dev->regulator) {
		dev->regulator = regulator_get(NULL, pdata->regulator_id);
		if (IS_ERR(dev->regulator)) {
			ret = PTR_ERR(dev->regulator);
			dev_err(&ddev->dev,
				"%s:Failed to get regulator '%s'\n",
				__func__, pdata->regulator_id);
			dev->regulator = NULL;
			goto out;
		}
		if (pdata->max_supply_voltage != 0) {
			ret = regulator_set_voltage(dev->regulator,
						    pdata->min_supply_voltage,
						    pdata->max_supply_voltage);
			if (ret < 0) {
				dev_err(&ddev->dev,
						"%s: Failed to set voltage\n",
						__func__);
				goto out1;
			}
		}
		ret = regulator_enable(dev->regulator);
		if (ret < 0) {
			dev_err(&ddev->dev, "%s:Failed to enable regulator\n",
								__func__);
			goto out1;
		}
	}

	if (pdata->io_regulator_id && !dev->io_regulator) {
		dev->io_regulator = regulator_get(NULL, pdata->io_regulator_id);
		if (IS_ERR(dev->io_regulator)) {
			ret = PTR_ERR(dev->io_regulator);
			dev_err(&ddev->dev,
				"%s:Failed to get IO regulator '%s'\n",
				__func__, pdata->io_regulator_id);
			dev->io_regulator = NULL;
			goto out1;
		}
		/* Do not set any voltage. This is an IO regulator used by
		 * many chips. It should not be changed */
		ret = regulator_enable(dev->io_regulator);
		if (ret < 0) {
			dev_err(&ddev->dev,
				"%s:Failed to enable IO regulator\n", __func__);
			goto out2;
		}
	}

	if (pdata->skip_init) {
		dev_info(&ddev->dev,
			"%s: Display already initialized during boot\n",
			__func__);
		pdata->skip_init = false;
	}

	if (dev->base.port->sync_src == MCDE_SYNCSRC_TE0)
		ddev->update = extern_te_update;
	else
		ddev->update = panel_update;

	/* TODO: Remove when DSI send command uses interrupts */
	ddev->prepare_for_update = NULL;
	return 0;
out2:
	regulator_put(dev->io_regulator);
	dev->io_regulator = NULL;
out1:
	regulator_put(dev->regulator);
	dev->regulator = NULL;
out:
	return ret;
}

static int panel_platform_disable(struct mcde_display_device *ddev)
{
	struct panel_device *dev =
		container_of(ddev, struct panel_device, base);
	struct panel_platform_data *pdata = dev->base.dev.platform_data;

	dev_dbg(&ddev->dev, "%s: Reset & power off display driver\n",
								__func__);

	/* Remove DDR and APE request */
	cancel_delayed_work(&dev->ddr_ape_timeout_work);
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, "main_LCD");
	pdata->ddr_is_requested = false;
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, "main_LCD_ape");
	pdata->ape_is_requested = false;

	if (dev->regulator) {
		if (regulator_disable(dev->regulator) < 0)
			dev_err(&ddev->dev, "%s:Failed to disable regulator\n",
								__func__);
		regulator_put(dev->regulator);
		dev->regulator = NULL;
	}
	if (dev->io_regulator) {
		if (regulator_disable(dev->io_regulator) < 0)
			dev_err(&ddev->dev,
				"%s:Failed to disable IO regulator\n",
				__func__);
		regulator_put(dev->io_regulator);
		dev->io_regulator = NULL;
	}
	return 0;
}

static void work_ddr_ape_timeout_function(struct work_struct *work)
{
	struct panel_device *dev = container_of(work,
		struct panel_device, ddr_ape_timeout_work.work);
	struct panel_platform_data *pdata = dev->base.dev.platform_data;

	dev_vdbg(&dev->base.dev, "%s: display update timeout\n", __func__);
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, "main_LCD");
	pdata->ddr_is_requested = false;
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, "main_LCD_ape");
	pdata->ape_is_requested = false;
}

static struct mcde_port panel_port0 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.ifc = DSI_CMD_MODE,
	.link = 0,
	.sync_src = MCDE_SYNCSRC_TE0,
	.update_auto_trig = false,
	.phy = {
		.dsi = {
			.virt_id = 0,
			.num_data_lanes = 2,
			.ui = DSI_UNIT_INTERVAL_0,
			.clk_cont = false,
			.host_eot_gen = true,
			.data_lanes_swap = false,
			.hs_freq = DSI_HS_FREQ_HZ,
			.lp_freq = DSI_LP_FREQ_HZ,
		},
	},
};

struct panel_device panel_display0 = {
	.base = {
		.name = MCDE_DISPLAY_PANEL_NAME,
		.id = PRIMARY_DISPLAY_ID,
		.port = &panel_port0,
		.chnl_id = MCDE_CHNL_A,
		.fifo = MCDE_FIFO_A,
		.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,

		/* These are setup via the panel files */
		.native_x_res = 0,
		.native_y_res = 0,
		.deep_standby_as_power_off = true,
		.platform_reset = panel_platform_reset,
		.platform_enable = panel_platform_enable,
		.platform_disable = panel_platform_disable,
		.dev = {
			.platform_data = &panel_display0_pdata,
		},
	}
};



#ifdef CONFIG_U8500_TV_OUTPUT_AV8100

#if defined(CONFIG_AV8100_HWTRIG_INT)
	#define AV8100_SYNC_SRC MCDE_SYNCSRC_TE0
#elif defined(CONFIG_AV8100_HWTRIG_I2SDAT3)
	#define AV8100_SYNC_SRC MCDE_SYNCSRC_TE1
#elif defined(CONFIG_AV8100_HWTRIG_DSI_TE)
	#define AV8100_SYNC_SRC MCDE_SYNCSRC_TE_POLLING
#else
	#define AV8100_SYNC_SRC MCDE_SYNCSRC_OFF
#endif
static struct mcde_port av8100_port2 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.ifc = 1,
	.link = 2,
	.sync_src = AV8100_SYNC_SRC,
	.update_auto_trig = true,
	.phy = {
		.dsi = {
			.num_data_lanes = 2,
			.ui = DSI_UNIT_INTERVAL_2,
		},
	},
	.hdmi_sdtv_switch = HDMI_SWITCH,
};

static struct mcde_display_hdmi_platform_data av8100_hdmi_pdata = {
	.cvbs_regulator_id = "vcc-N2158",
	.rgb_2_yCbCr_transform = &rgb_2_yCbCr_transform,
};

static struct mcde_display_device av8100_hdmi = {
	.name = "av8100_hdmi",
	.id = AV8100_DISPLAY_ID,
	.port = &av8100_port2,
	.chnl_id = MCDE_CHNL_B,
	.fifo = MCDE_FIFO_B,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
	.native_x_res = 1280,
	.native_y_res = 720,
	.dev = {
		.platform_data = &av8100_hdmi_pdata,
	},
};

static void delayed_work_dispreg_hdmi(struct work_struct *ptr)
{
	if (mcde_display_device_register(&av8100_hdmi))
		pr_warning("Failed to register av8100_hdmi\n");
}
#endif /* CONFIG_U8500_TV_OUTPUT_AV8100 */

#ifdef CONFIG_FB_MCDE

/*
* This function will create the framebuffer for the display that is registered.
*/
static int display_postregistered_callback(struct notifier_block *nb,
	unsigned long event, void *dev)
{
	struct mcde_display_device *ddev = dev;
	u16 width, height;
	u16 virtual_height;
	struct fb_info *fbi;
#if defined(CONFIG_DISPDEV) || defined(CONFIG_COMPDEV)
	struct mcde_fb *mfb;
#endif

	if (event != MCDE_DSS_EVENT_DISPLAY_REGISTERED)
		return 0;

	if (ddev->id < 0 || ddev->id >= MCDE_NR_OF_DISPLAYS)
		return 0;

	mcde_dss_get_native_resolution(ddev, &width, &height);
#ifdef CONFIG_MCDE_DISPLAY_PRIMARY_TRIPPLE_BUFFERED
	if (ddev->id == PRIMARY_DISPLAY_ID)
		virtual_height = height * 3;
	else
#endif
		virtual_height = height * 2;

#ifndef CONFIG_MCDE_DISPLAY_HDMI_FB_AUTO_CREATE
	if (ddev->id == AV8100_DISPLAY_ID)
		goto out;
#endif

	/* Create frame buffer */
	fbi = mcde_fb_create(ddev, width, height, width, virtual_height,
				ddev->default_pixel_format, FB_ROTATE_UR);
	if (IS_ERR(fbi)) {
		dev_warn(&ddev->dev,
			"Failed to create fb for display %s\n", ddev->name);
		goto display_postregistered_callback_err;
	} else {
		dev_info(&ddev->dev, "Framebuffer created (%s)\n", ddev->name);
	}

#ifdef CONFIG_DISPDEV
	mfb = to_mcde_fb(fbi);

	/* Create a dispdev overlay for this display */
	if (dispdev_create(ddev, true, mfb->ovlys[0]) < 0) {
		dev_warn(&ddev->dev,
			"Failed to create disp for display %s\n", ddev->name);
		goto display_postregistered_callback_err;
	} else {
		dev_info(&ddev->dev, "Disp dev created for (%s)\n", ddev->name);
	}
#endif

#ifdef CONFIG_COMPDEV
	/* Only create compdev for the main display */
	if (ddev->id == PRIMARY_DISPLAY_ID) {
		bool mcde_rotation = false;

		/* Use mcde rotation for U8500 only */
		if (cpu_is_u8500())
			mcde_rotation = true;

		mfb = to_mcde_fb(fbi);
		/* Create a compdev overlay for this display */
		if (compdev_create(ddev, mfb->ovlys[0], mcde_rotation,
					NULL) < 0) {
			dev_warn(&ddev->dev,
				"Failed to create compdev for display %s\n",
						ddev->name);
			goto display_postregistered_callback_err;
		} else {
			dev_info(&ddev->dev, "compdev created for (%s)\n",
						ddev->name);
		}
	}
#endif

#ifdef CONFIG_MCDE_DISPLAY_LOGO
	/* We can show the boot logo when we have a frame buffer,
	   but not earlier than that. */
	if (!shown_logo) {
		shown_logo = true;
		mcde_logo_init();
	}
#endif

#ifndef CONFIG_MCDE_DISPLAY_HDMI_FB_AUTO_CREATE
out:
#endif
	return 0;

display_postregistered_callback_err:
	return -1;
}

static struct notifier_block display_nb = {
	.notifier_call = display_postregistered_callback,
};
#endif /* CONFIG_FB_MCDE */

static int __init handle_display_devices_in_u8500(void)
{
	int ret;
	struct mcde_platform_data *pdata = ux500_mcde_device.dev.platform_data;
	pr_debug("%s\n", __func__);

#ifdef CONFIG_FB_MCDE
	(void)mcde_dss_register_notifier(&display_nb);
#endif

	/* Initialize all needed clocks*/
	if (!display_initialized_during_boot) {
		struct clk *clk_dsi_pll;
		struct clk *clk_hdmi;
		struct clk *clk_tv;

		/*
		 * The TV CLK is used as parent for the
		 * DSI LP clock.
		 */
		clk_tv = clk_get(&ux500_mcde_device.dev, "tv");
		if (TV_FREQ_HZ != clk_round_rate(clk_tv, TV_FREQ_HZ))
			pr_warning("%s: TV_CLK freq differs %ld\n", __func__,
					clk_round_rate(clk_tv, TV_FREQ_HZ));
		clk_set_rate(clk_tv, TV_FREQ_HZ);
		clk_put(clk_tv);

		/*
		 * The HDMI CLK is used as parent for the
		 * DSI HS clock.
		 */
		clk_hdmi = clk_get(&ux500_mcde_device.dev, "hdmi");
		if (HDMI_FREQ_HZ != clk_round_rate(clk_hdmi, HDMI_FREQ_HZ))
			pr_warning("%s: HDMI freq differs %ld\n", __func__,
					clk_round_rate(clk_hdmi, HDMI_FREQ_HZ));
		clk_set_rate(clk_hdmi, HDMI_FREQ_HZ);
		clk_put(clk_hdmi);

		/*
		 * The DSI PLL CLK is used as DSI PLL for direct freq for
		 * link 2. Link 0/1 is then divided with 1/2/4 from this freq.
		 */
		clk_dsi_pll = clk_get(&ux500_mcde_device.dev, "dsihs2");
		if (DSI_PLL_FREQ_HZ != clk_round_rate(clk_dsi_pll,
							DSI_PLL_FREQ_HZ))
			pr_warning("%s: DSI_PLL freq differs %ld\n", __func__,
				clk_round_rate(clk_dsi_pll, DSI_PLL_FREQ_HZ));
		clk_set_rate(clk_dsi_pll, DSI_PLL_FREQ_HZ);
		clk_put(clk_dsi_pll);
	}

	/* MCDE pixelfetchwtrmrk levels per overlay */
	pdata->pixelfetchwtrmrk[0] = 48;	/* LCD 32 bpp */
	pdata->pixelfetchwtrmrk[1] = 64;	/* LCD 16 bpp */
	pdata->pixelfetchwtrmrk[2] = 128;	/* HDMI 32 bpp */
	pdata->pixelfetchwtrmrk[3] = 192;	/* HDMI 16 bpp */


	INIT_DELAYED_WORK_DEFERRABLE(&panel_display0.ddr_ape_timeout_work,
						work_ddr_ape_timeout_function);
	((struct panel_platform_data *)panel_display0.base.dev.platform_data)->
						ddr_is_requested = false;
	((struct panel_platform_data *)panel_display0.base.dev.platform_data)->
						ape_is_requested = false;
	/* TODO: enable this code if uboot graphics should be used
	 if (display_initialized_during_boot)
		((struct panel_platform_data *)panel_display0.
		base.dev.platform_data)->skip_init = true;
	 */
	ret = mcde_display_device_register(&panel_display0.base);
	if (ret)
		pr_warning("Failed to register display device 0\n");


#if defined(CONFIG_U8500_TV_OUTPUT_AV8100)
	INIT_DELAYED_WORK_DEFERRABLE(&work_dispreg_hdmi,
			delayed_work_dispreg_hdmi);
	schedule_delayed_work(&work_dispreg_hdmi,
			msecs_to_jiffies(DISPREG_HDMI_DELAY));
#endif

	return 0;
}

#if 0
static int __init handle_display_devices_in_u9540(void)
{
	struct mcde_platform_data *pdata = ux500_mcde_device.dev.platform_data;

	pr_debug("%s\n", __func__);

#ifdef CONFIG_FB_MCDE
	(void)mcde_dss_register_notifier(&display_nb);
#endif*/

	/* Set powermode to STANDBY if startup graphics is executed */
	/*if (display_initialized_during_boot)
		sony_acx424akp_display0.power_mode = MCDE_DISPLAY_PM_ON;

	/* MCDE pixelfetchwtrmrk levels per overlay */
	pdata->pixelfetchwtrmrk[0] = 64;	/* LCD 32 bpp */
	pdata->pixelfetchwtrmrk[1] = 96;	/* LCD 16 bpp */
	pdata->pixelfetchwtrmrk[2] = 192;	/* HDMI 32 bpp */
	pdata->pixelfetchwtrmrk[3] = 256;	/* HDMI 16 bpp */


	sony_acx424akp_display0_pdata.reset_gpio = UIB_9540_DISP1_RST_GPIO;
	(void)mcde_display_device_register(&sony_acx424akp_display0);

	return 0;
}
#endif

static int __init init_display_devices(void)
{
	if (cpu_is_u8500())
		return handle_display_devices_in_u8500();
	/*else if (cpu_is_u9540())
		return handle_display_devices_in_u9540();*/
	else
		return 0;
}
module_init(init_display_devices);
