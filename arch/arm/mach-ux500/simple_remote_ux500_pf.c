/* kernel/arch/arm/mach-ux500/simple_remote_ux500_pf.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Authors: Joachim Holst <joachim.holst@sonyericsson.com>
 *          Torbjorn Eklund <torbjorn.eklund@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#include <linux/simple_remote.h>
#include <mach/simple_remote_ux500_pf.h>
#include <linux/mfd/ab8500.h>
#include <linux/mfd/abx500/ab8500-gpadc.h>
#include <linux/mfd/abx500.h>
#include <sound/ux500_ab8500_ext.h>

#define HSSD_INTERRUPT_ENABLED_BIT	BIT(1)
#define DET_INTERRUPT_ENABLED_BIT	BIT(2)

/* ACC DETECT control registers */
#define VMC_REGU_REG		0x84
#define VAMIC_PD_BIT		0x02

#define GPADC_REG_BASE		0x08
#define ACC_DET22_DEBOUNCE	0x80
#define ACC_DET_THRS_REG	0x81
#define ACC_DET22_THS_SHIFT	0x04
#define ACC_DET22_THS_MASK	0xF0
#define ACC_DET_CTL_REG	0x82

#define LOCK(x)							\
	do {								\
		dev_dbg(loc_dat->dev, "%s - %d Locking mutex\n", __func__, \
			__LINE__);					\
		mutex_lock(x);						\
	} while (0)							\

#define UNLOCK(x)							\
	do {								\
		dev_dbg(loc_dat->dev, "%s - %d Unlocking mutex\n",	\
		       __func__, __LINE__);				\
		mutex_unlock(x);					\
	} while (0)							\

#define TRY_LOCK(x)							\
	do {								\
		dev_dbg(loc_dat->dev, "%s - %d Trying to lock muex\n",	\
			__func__, __LINE__);				\
		mutex_trylock(x);					\
	} while (0)

static unsigned int acc_det_trig_levels[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1500,
	1600,
	1700,
	1800,
};

static unsigned int acc_det_debounce_levels[] = {
	0,
	10,
	20,
	30,
	40,
	50,
	60,
	70,
};

struct btn_irq_data {
	char *name;
	char *desc;
};

static struct btn_irq_data btn_data[] = {
	{
		.name = "ACC_DETECT_22DB_F",
		.desc = "button_press_irq",
	},
	{
		.name = "ACC_DETECT_22DB_R",
		.desc = "button_release_irq",
	},
};

struct local_data {
	struct simple_remote_platform_data *jack_pf;
	long unsigned int simple_remote_pf_flags;
	struct mutex simple_remote_pf_lock;

	struct device *dev;
	struct platform_device *pdev;

	atomic_t mic_bias_enable_counter;

	struct regulator *vamic2_reg;

	bool do_alt_adc;
};

static struct local_data *loc_dat;

static int configure_accdetect_block(int enable);

static int simple_remote_pf_ab8500_reg_mask(u8 bank, u8 reg, u8 mask, u8 bitval)
{
	int err;

	dev_dbg(loc_dat->dev,
		 "%s - bank = 0x%02X, reg = 0x%02X, mask = 0x%02X"
		 " bitval = 0x%02X\n",
		 __func__, bank, reg, mask, bitval);

	err = abx500_mask_and_set(&loc_dat->pdev->dev, bank, reg,
				  mask, bitval);

	if (err)
		dev_err(&loc_dat->pdev->dev,
			"%s - Failed to write data. Error: %d\n",
			__func__, err);

	return err;
}

static int simple_remote_pf_ab8500_reg_write(u8 bank, u8 reg, u8 value)
{
	int ret;

	dev_dbg(&loc_dat->pdev->dev,
		 "%s - bank = 0x%02X, reg = 0x%02X, val = 0x%02X\n",
		 __func__, bank, reg, value);

	ret = abx500_set(&loc_dat->pdev->dev, bank, reg, value);
	if (ret < 0)
		dev_err(loc_dat->dev, "%s prcmu i2c error %d\n", __func__, ret);

	return ret;
}

static int simple_remote_pf_ab8500_reg_read(u8 bank, u8 reg, u8 *value)
{
	int ret;

	dev_dbg(loc_dat->dev, "%s - bank = 0x%02x, reg = 0x%02x",
		 __func__, bank, reg);

	ret = abx500_get(&loc_dat->pdev->dev, bank, reg, value);
	if (ret < 0)
		dev_err(loc_dat->dev, "%s prcmu i2c error %d\n", __func__, ret);

	dev_dbg(loc_dat->dev, "%s - Read value = 0x%02x\n", __func__, *value);

	return ret;
}

static int simple_remote_pf_read_hsd_adc(unsigned int *adc_value)
{
	int ret = 0;
	struct ab8500_gpadc *ab = ab8500_gpadc_get();

	LOCK(&loc_dat->simple_remote_pf_lock);

	if (!loc_dat->do_alt_adc) {
		int iterations = 2;
		int max_deviation = 30;
		int v1, v2, dv;

		v1 = ab8500_gpadc_convert(ab, ACC_DETECT2);
		do {
			usleep_range(1000, 2000);
			--iterations;
			v2 = ab8500_gpadc_convert(ab, ACC_DETECT2);
			dv = abs(v2 - v1);
			v1 = v2;
		} while (iterations > 0 && dv > max_deviation);

		*adc_value = v1;
	} else {
		ret = ux500_ab8500_audio_gpadc_measure(ab, ACC_DETECT2, true,
						     adc_value);
	}

	UNLOCK(&loc_dat->simple_remote_pf_lock);

	dev_dbg(loc_dat->dev,
		"%s - alt_mode = %d, value = %d\n", __func__,
		loc_dat->do_alt_adc, *adc_value);

	return ret;
}

static int simple_remote_pf_enable_mic_bias(unsigned int enable)
{
	int err;
	u8 regu_data;

	mutex_lock(&loc_dat->simple_remote_pf_lock);

	dev_dbg(loc_dat->dev, "%s - %s MIC Bias\n", __func__,
		enable ? "Enabling" : "Disabling");

	simple_remote_pf_ab8500_reg_read(AB8500_REGU_CTRL1, VMC_REGU_REG,
					 &regu_data);
	if (!(regu_data & VAMIC_PD_BIT)) {
		dev_dbg(loc_dat->dev, "Pulldown not enabled. Enabling\n");
		regu_data |= VAMIC_PD_BIT;
		simple_remote_pf_ab8500_reg_mask(
			AB8500_REGU_CTRL1, VMC_REGU_REG, VAMIC_PD_BIT,
			VAMIC_PD_BIT);
	}

	if (enable) {
		err = regulator_enable(loc_dat->vamic2_reg);
		if (err)
			dev_err(loc_dat->dev,
				"%s - Failed to enable MIC Bias\n", __func__);
		else
			dev_dbg(loc_dat->dev, "%s - MIC Bias enabled\n",
				__func__);
	} else {
		err = regulator_disable(loc_dat->vamic2_reg);
		if (!err)
			dev_dbg(loc_dat->dev,
				"%s - MIC Bias disabled\n", __func__);
		else
			dev_dbg(loc_dat->dev,
				"%s - Failed to disable MIC Bias\n",
				__func__);
	}

	mutex_unlock(&loc_dat->simple_remote_pf_lock);

	return err;
}

static int simple_remote_pf_set_period_freq(unsigned int value)
{
	/* Not supported on this platform */
	return -ENODEV;
}

static int simple_remote_pf_set_period_time(unsigned int value)
{
	int ret = -EINVAL;
	int i;
	u8 data;

	dev_dbg(loc_dat->dev, "%s - Setting trig level to %u", __func__, value);

	simple_remote_pf_ab8500_reg_read(GPADC_REG_BASE, ACC_DET22_DEBOUNCE,
					 &data);
	dev_dbg(loc_dat->dev, "%s - Read data = 0x%02X\n", __func__, data);

	if (value >= acc_det_debounce_levels[0] &&
	    value <= acc_det_debounce_levels[
		    ARRAY_SIZE(acc_det_debounce_levels) - 1]) {
		for (i = 0; i < ARRAY_SIZE(acc_det_debounce_levels); i++) {
			if (value <= acc_det_debounce_levels[i]) {
				dev_dbg(loc_dat->dev,
					"%s - Correct value found %u\n",
					__func__, acc_det_debounce_levels[i]);
				data = i;
				goto do_write;
			}
		}
	} else {
		dev_warn(loc_dat->dev, "Trig level out of range\n");
	}

	return ret;

do_write:
	dev_dbg(loc_dat->dev, "%s - Writing data 0x%02X\n",
		__func__, data);

	ret = simple_remote_pf_ab8500_reg_write(GPADC_REG_BASE,
						ACC_DET22_DEBOUNCE,
						data);
	if (ret)
		dev_err(loc_dat->dev,
			"%s - Failed to set ACC_DETECT trig"
			" level\n", __func__);

	return ret;
}

static int simple_remote_pf_set_hysteresis_freq(unsigned int value)
{
	/* Not supported on this platform */
	return -ENODEV;
}

static int simple_remote_pf_set_hysteresis_time(unsigned int value)
{
	/* Not supported on this platform */
	return -ENODEV;
}

static int simple_remote_pf_set_trig_level(unsigned int value)
{
	int ret = -EINVAL;
	int i;
	u8 data;


	dev_dbg(loc_dat->dev, "%s - Setting trig level to %u", __func__,
		value);

	simple_remote_pf_ab8500_reg_read(GPADC_REG_BASE, ACC_DET_THRS_REG,
					 &data);

	dev_dbg(loc_dat->dev, "%s - Read data = 0x%02X\n", __func__, data);
	if (value >= acc_det_trig_levels[0] &&
	    value <= acc_det_trig_levels[ARRAY_SIZE(acc_det_trig_levels) - 1]) {
		/* Currently only accept exact values */
		for (i = 0; i < ARRAY_SIZE(acc_det_trig_levels); i++) {
			if (value <= acc_det_trig_levels[i]) {
				dev_dbg(loc_dat->dev,
					"%s - Correct value found %u\n",
					__func__, acc_det_trig_levels[i]);
				data &= !ACC_DET22_THS_MASK;
				data |= (i << ACC_DET22_THS_SHIFT);
				goto do_write;
			}
		}
	} else {
		dev_warn(loc_dat->dev, "Trig level out of range\n");
	}

	return ret;

do_write:
	ret = simple_remote_pf_ab8500_reg_write(GPADC_REG_BASE,
						ACC_DET_THRS_REG,
						data);
	if (ret)
		dev_err(loc_dat->dev,
			"Failed to set ACC_DETECT trig"
			" level\n");

	return ret;
}

static int simple_remote_pf_get_period_freq(unsigned int *value)
{
	/* Not supported on this platform */
	*value = 0;
	return -ENODEV;
}

static int simple_remote_pf_get_period_time(unsigned int *value)
{
	u8 data;
	int ret = simple_remote_pf_ab8500_reg_read(GPADC_REG_BASE,
						   ACC_DET22_DEBOUNCE,
						   &data);
	if (ret >= 0) {
		*value = acc_det_debounce_levels[data];
		ret = 0;
	} else {
		dev_err(loc_dat->dev, "Failed to read current acc_detect "
			"threshold.\n");
	}
	return ret;
}

static int simple_remote_pf_get_hysteresis_freq(unsigned int *value)
{
	/* Not supported on this platform */
	*value = 0;
	return -ENODEV;
}

static int simple_remote_pf_get_hysteresis_time(unsigned int *value)
{
	/* Not supported on this platform */
	*value = 0;
	return -ENODEV;
}

static int simple_remote_pf_get_trig_level(unsigned int *value)
{
	u8 data;
	int ret = simple_remote_pf_ab8500_reg_read(GPADC_REG_BASE,
						   ACC_DET_THRS_REG,
						   &data);
	if (ret >= 0) {
		*value = acc_det_trig_levels[data >> ACC_DET22_THS_SHIFT];
		ret = 0;
	} else {
		dev_err(loc_dat->dev, "Failed to read current acc_detect "
			"threshold.\n");
	}
	return ret;
}

static int simple_remote_pf_get_current_plug_status(u8 *status)
{
	if (loc_dat->jack_pf->invert_plug_det) {
		dev_dbg(loc_dat->dev, "%s - Invert plug state\n",
			__func__);
		*status = gpio_get_value(
			loc_dat->jack_pf->headset_detect_read_pin);
	} else {
		dev_dbg(loc_dat->dev, "%s - Not inverted plug state\n",
			__func__);
		*status = !gpio_get_value(
			loc_dat->jack_pf->headset_detect_read_pin);
	}

	dev_dbg(loc_dat->dev, "%s - current plug state = %u\n",
		__func__, *status);

	return 0;
}


static int enable_alternate_adc_mode(u8 enable)
{
	LOCK(&loc_dat->simple_remote_pf_lock);
	loc_dat->do_alt_adc = enable;
	UNLOCK(&loc_dat->simple_remote_pf_lock);

	return 0;
}

static int enable_alternate_headset_mode(u8 enable)
{
	int ret = -ENODEV;

	LOCK(&loc_dat->simple_remote_pf_lock);
	if (0 <= loc_dat->jack_pf->headset_mode_switch_pin) {
		u8 status;
		dev_dbg(loc_dat->dev, "%s - We support headset mode switch\n",
			__func__);
		status = gpio_get_value(
			loc_dat->jack_pf->headset_mode_switch_pin);
		gpio_set_value(loc_dat->jack_pf->headset_mode_switch_pin,
			       !status);
		ret = 0;
		goto done;
	}

	dev_info(loc_dat->dev,
		 "%s - This hardware doesn't support headset CTIA/OMTP mode "
		 "switch\n", __func__);

done:
	UNLOCK(&loc_dat->simple_remote_pf_lock);
	return ret;
}

static int simple_remote_pf_register_plug_detect_interrupt(irq_handler_t func,
							   void *data)
{
	int err = -1;
	int irq = 0;

	LOCK(&loc_dat->simple_remote_pf_lock);
	if (test_bit(DET_INTERRUPT_ENABLED_BIT,
		     &loc_dat->simple_remote_pf_flags))
		goto done;

	irq = gpio_to_irq(loc_dat->jack_pf->headset_detect_read_pin);

	if (0 <= irq) {
		err = request_threaded_irq(irq, NULL, func,
					   IRQF_TRIGGER_FALLING |
					   IRQF_TRIGGER_RISING,
					   "simple_remote_plug_detect",
					   data);

		if (err) {
			dev_crit(loc_dat->dev, "Failed to subscribe to plug "
				 "detect interrupt\n");
			goto done;
			}
	} else {
		dev_crit(loc_dat->dev, "Failed to register interrupt for GPIO "
			 "(%d). GPIO Does not exist\n",
			 loc_dat->jack_pf->headset_detect_read_pin);
		err = irq;
		goto done;
	}

	set_bit(DET_INTERRUPT_ENABLED_BIT, &loc_dat->simple_remote_pf_flags);

	err = enable_irq_wake(irq);
	if (err)
		dev_crit(loc_dat->dev,
			 "Failed to enable wakeup on interrupt\n");

done:
	UNLOCK(&loc_dat->simple_remote_pf_lock);
	return err;
}

static void simple_remote_pf_unregister_plug_detect_interrupt(void *data)
{
	int irq;

	LOCK(&loc_dat->simple_remote_pf_lock);
	if (!test_bit(DET_INTERRUPT_ENABLED_BIT,
		      &loc_dat->simple_remote_pf_flags))
		goto done;

	irq = gpio_to_irq(loc_dat->jack_pf->headset_detect_read_pin);
	if (0 <= irq) {
		disable_irq_wake(irq);
		free_irq(irq, data);
		clear_bit(DET_INTERRUPT_ENABLED_BIT,
			  &loc_dat->simple_remote_pf_flags);
	} else {
		dev_crit(loc_dat->dev, "Failed to disable plug detect interrupt"
			 ". GPIO (%d) does not exist\n",
			 loc_dat->jack_pf->headset_detect_read_pin);
	}

done:
	UNLOCK(&loc_dat->simple_remote_pf_lock);
}

/* Note: This function is only called when mutex is already held */
static int configure_accdetect_block(int enable)
{
	u8 reg_data = 0;
	int ret;

	dev_dbg(loc_dat->dev, "%s - Called\n", __func__);

	if (enable)
		reg_data = 0x22;

	/* Enable/Disable accdetect22 comparator */
	ret = simple_remote_pf_ab8500_reg_write(GPADC_REG_BASE,
						ACC_DET_CTL_REG,
						reg_data);
	if (ret < 0)
		dev_err(loc_dat->dev, "%s: Failed to update reg (%d).\n",
				__func__, ret);

	dev_dbg(loc_dat->dev, "%s - %s accdetect_block\n", __func__,
		enable ? "Enabled" : "Disabled");
	dev_dbg(loc_dat->dev, "%s - Wrote data: 0x%X\n",
		__func__, reg_data);

	return ret;
}

/* Note: This function is only called when mutex is already held */
static int request_button_interrupts(int enable, irq_handler_t func, void *data)
{
	int i;
	int irq;
	int err;

	if (NULL == func && enable) {
		dev_err(loc_dat->dev,
			"%s - Tried to enable IRQ's without valid handler",
			__func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(btn_data); i++) {
		irq = platform_get_irq_byname(loc_dat->pdev,
					       btn_data[i].name);
		if (0 > irq) {
			dev_crit(loc_dat->dev,
				"%s - Failed to retrieve correct IRQ.\n",
				__func__);
			if (!enable)
				return irq;
			else
				goto irq_req_fail;
		}

		if (enable) {
			dev_dbg(loc_dat->dev, "%s - Requesting IRQ %d\n",
				__func__, irq);
			err = request_threaded_irq(irq, NULL, func,
						   IRQF_NO_SUSPEND,
						   btn_data[i].desc,
						   data);
			if (err) {
				dev_crit(loc_dat->dev,
					 "%s -  Failed to subscribe to button "
					 "detect interrupt (%s)"
					 ". Error : %d\n", __func__,
					 btn_data[i].name, err);
				goto irq_req_fail;
			}
		} else {
			dev_dbg(loc_dat->dev, "%s - Releasing IRQ %d\n",
				__func__, irq);
			free_irq(irq, data);
		}
	}

	return err;

irq_req_fail:
	for (i = i - 1; i >= 0; i--) {
		irq = platform_get_irq_byname(loc_dat->pdev,
					       btn_data[i].name);
		if (0 <= irq)
			free_irq(irq, data);
	}

	return err;
}

static int simple_remote_pf_register_hssd_button_interrupt(irq_handler_t func,
							   void *data)
{
	int err = 0;

	LOCK(&loc_dat->simple_remote_pf_lock);

	if (test_bit(HSSD_INTERRUPT_ENABLED_BIT,
		     &loc_dat->simple_remote_pf_flags))
		goto done;

	if (configure_accdetect_block(1)) {
		dev_err(loc_dat->dev,
			"%s - Failed to enable ACCDETECT2 IRQ block\n",
			__func__);
		goto done;
	}

	err = request_button_interrupts(1, func, data);
	if (err)
		goto irq_fail;

	set_bit(HSSD_INTERRUPT_ENABLED_BIT,
		&loc_dat->simple_remote_pf_flags);

	goto done;

irq_fail:
	configure_accdetect_block(0);
done:
	UNLOCK(&loc_dat->simple_remote_pf_lock);

	return err;
}

static void simple_remote_pf_unregister_hssd_button_interrupt(void *data)
{
	LOCK(&loc_dat->simple_remote_pf_lock);

	if (!test_bit(HSSD_INTERRUPT_ENABLED_BIT,
		      &loc_dat->simple_remote_pf_flags))
		goto done;

	request_button_interrupts(0, NULL, data);

	configure_accdetect_block(0);

	clear_bit(HSSD_INTERRUPT_ENABLED_BIT, &loc_dat->simple_remote_pf_flags);

done:
	UNLOCK(&loc_dat->simple_remote_pf_lock);
}

static struct simple_remote_pf_interface interface = {
	.read_hs_adc = simple_remote_pf_read_hsd_adc,
	.enable_mic_bias = simple_remote_pf_enable_mic_bias,
	.get_current_plug_status = simple_remote_pf_get_current_plug_status,

	.enable_alternate_adc_mode = enable_alternate_adc_mode,
	.enable_alternate_headset_mode = enable_alternate_headset_mode,

	.set_period_freq = simple_remote_pf_set_period_freq,
	.set_period_time = simple_remote_pf_set_period_time,
	.set_hysteresis_freq = simple_remote_pf_set_hysteresis_freq,
	.set_hysteresis_time = simple_remote_pf_set_hysteresis_time,
	.set_trig_level = simple_remote_pf_set_trig_level,

	.get_period_freq = simple_remote_pf_get_period_freq,
	.get_period_time = simple_remote_pf_get_period_time,
	.get_hysteresis_freq = simple_remote_pf_get_hysteresis_freq,
	.get_hysteresis_time = simple_remote_pf_get_hysteresis_time,
	.get_trig_level = simple_remote_pf_get_trig_level,

	.register_plug_detect_interrupt =
		simple_remote_pf_register_plug_detect_interrupt,

	.unregister_plug_detect_interrupt =
		simple_remote_pf_unregister_plug_detect_interrupt,

	.register_hssd_button_interrupt =
		simple_remote_pf_register_hssd_button_interrupt,

	.unregister_hssd_button_interrupt =
		simple_remote_pf_unregister_hssd_button_interrupt,
};

static struct platform_device simple_remote_device = {
	.name = SIMPLE_REMOTE_NAME,
	.dev = {
		.platform_data = &interface,
	},
};

static int simple_remote_pf_probe(struct platform_device *pdev)
{
	int ret;

	struct platform_device *n_pdev = &simple_remote_device;

	loc_dat = kzalloc(sizeof(*loc_dat), GFP_KERNEL);
	if (!loc_dat)
		return -ENOMEM;

	loc_dat->jack_pf = pdev->dev.platform_data;
	loc_dat->dev = &pdev->dev;
	loc_dat->pdev = pdev;

	ret = loc_dat->jack_pf->initialize(loc_dat->jack_pf);
	if (ret)
		goto out;

	mutex_init(&loc_dat->simple_remote_pf_lock);

	loc_dat->vamic2_reg = regulator_get(NULL, "v-amic2");
	if (IS_ERR(loc_dat->vamic2_reg)) {
		dev_err(loc_dat->dev,
			"%s: Failed to allocate regulator vamic1\n", __func__);
		goto out;
	}

	ret = platform_add_devices(&n_pdev, 1);
	if (ret) {
		dev_err(loc_dat->dev,
			"%s - Failed to register logic layer\n",
			__func__);
		goto out_vamic;
	}

	dev_info(loc_dat->dev, "Successfully registered\n");

	return ret;

out_vamic:
	regulator_put(loc_dat->vamic2_reg);
out:
	kfree(loc_dat);
	dev_err(&pdev->dev, "Failed to register driver\n");
	return ret;
}

static int simple_remote_pf_remove(struct platform_device *pdev)
{
	struct platform_device *n_pdev = &simple_remote_device;

	(void)simple_remote_pf_enable_mic_bias(0);

	platform_device_unregister(n_pdev);

	loc_dat->jack_pf->deinitialize(loc_dat->jack_pf);

	regulator_put(loc_dat->vamic2_reg);

	kfree(loc_dat);
	return 0;
}

static struct platform_driver simple_remote_pf = {
	.probe			= simple_remote_pf_probe,
	.remove		= simple_remote_pf_remove,
	.driver		= {
		.name		= SIMPLE_REMOTE_PF_NAME,
		.owner		= THIS_MODULE,
	},
};

static int __init simple_remote_pf_init(void)
{
	return platform_driver_register(&simple_remote_pf);
}

static void __exit simple_remote_pf_exit(void)
{
	platform_driver_unregister(&simple_remote_pf);
}

module_init(simple_remote_pf_init);
module_exit(simple_remote_pf_exit);

MODULE_AUTHOR("Joachim Holst (joachim.holst@sonyericsson.com)");
MODULE_DESCRIPTION("3.5mm audio jack HW level driver");
MODULE_LICENSE("GPL");
