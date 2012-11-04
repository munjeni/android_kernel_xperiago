/*
 *  Copyright (C) 2009-2011 ST-Ericsson SA
 *  Copyright (C) 2009 STMicroelectronics
 *  Copyright (C) 2012 Sony Mobile Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/mfd/abx500/ux500_sysctrl.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>

#include <plat/gpio-nomadik.h>
#include <plat/pincfg.h>

#include <mach/hardware.h>
#include <linux/mfd/dbx500-prcmu.h>

#include "clock.h"
#include "pins-db8500.h"
#include "product.h"
#include "prcc.h"

static DEFINE_MUTEX(soc0_pll_mutex);
static DEFINE_MUTEX(soc1_pll_mutex);
static DEFINE_MUTEX(sysclk_mutex);
static DEFINE_MUTEX(ab_ulpclk_mutex);
static DEFINE_MUTEX(ab_intclk_mutex);
static DEFINE_MUTEX(clkout0_mutex);
static DEFINE_MUTEX(dsi_pll_mutex);

static struct delayed_work sysclk_disable_work;

/* PLL operations. */

static unsigned long pll_get_rate(struct clk *clk)
{
	return prcmu_clock_rate(clk->cg_sel);
}

static struct clkops pll_ops = {
	.get_rate = pll_get_rate,
};

/* SysClk operations. */

static int request_sysclk(bool enable)
{
	static int requests;

	if ((enable && (requests++ == 0)) || (!enable && (--requests == 0)))
		return prcmu_request_clock(PRCMU_SYSCLK, enable);
	return 0;
}

static int sysclk_enable(struct clk *clk)
{
	static bool swat_enable;
	int r;

	if (!swat_enable) {
		r = ab8500_sysctrl_set(AB8500_SWATCTRL,
			AB8500_SWATCTRL_SWATENABLE);
		if (r)
			return r;

		swat_enable = true;
	}

	r = request_sysclk(true);
	if (r)
		return r;

	if (clk->cg_sel) {
		r = ab8500_sysctrl_set(AB8500_SYSULPCLKCTRL1, (u8)clk->cg_sel);
		if (r)
			(void)request_sysclk(false);
	}
	return r;
}

static void sysclk_disable(struct clk *clk)
{
	int r;

	if (clk->cg_sel) {
		r = ab8500_sysctrl_clear(AB8500_SYSULPCLKCTRL1,
			(u8)clk->cg_sel);
		if (r)
			goto disable_failed;
	}
	r = request_sysclk(false);
	if (r)
		goto disable_failed;
	return;

disable_failed:
	pr_err("clock: failed to disable %s.\n", clk->name);
}

static struct clkops sysclk_ops = {
	.enable = sysclk_enable,
	.disable = sysclk_disable,
};

/* AB8500 UlpClk operations */

static int ab_ulpclk_enable(struct clk *clk)
{
	int err;

	if (clk->regulator == NULL) {
		struct regulator *reg;

		reg = regulator_get(NULL, "v-intcore");
		if (IS_ERR(reg))
			return PTR_ERR(reg);
		clk->regulator = reg;
	}
	err = regulator_set_optimum_mode(clk->regulator, 1500);
	if (unlikely(err < 0))
		goto regulator_enable_error;
	err = regulator_enable(clk->regulator);
	if (unlikely(err))
		goto regulator_enable_error;
	err = ab8500_sysctrl_clear(AB8500_SYSULPCLKCONF,
		AB8500_SYSULPCLKCONF_ULPCLKCONF_MASK);
	if (unlikely(err))
		goto enable_error;
	err = ab8500_sysctrl_set(AB8500_SYSULPCLKCTRL1,
		AB8500_SYSULPCLKCTRL1_ULPCLKREQ);
	if (unlikely(err))
		goto enable_error;
	/* Undocumented PLL locking time. According to AMS team, > 8 ms */
	usleep_range(8000, 9000);
	return 0;

enable_error:
	(void)regulator_disable(clk->regulator);
regulator_enable_error:
	return err;
}

static void ab_ulpclk_disable(struct clk *clk)
{
	int err;

	err = ab8500_sysctrl_clear(AB8500_SYSULPCLKCTRL1,
		AB8500_SYSULPCLKCTRL1_ULPCLKREQ);
	if (unlikely(regulator_disable(clk->regulator) || err))
		goto out_err;

	regulator_set_optimum_mode(clk->regulator, 0);

	return;

out_err:
	pr_err("clock: %s failed to disable %s.\n", __func__, clk->name);
}

static struct clkops ab_ulpclk_ops = {
	.enable = ab_ulpclk_enable,
	.disable = ab_ulpclk_disable,
};

/* AB8500 intclk operations */

enum ab_intclk_parent {
	AB_INTCLK_PARENT_SYSCLK,
	AB_INTCLK_PARENT_ULPCLK,
	AB_INTCLK_PARENTS_END,
	NUM_AB_INTCLK_PARENTS
};

static int ab_intclk_enable(struct clk *clk)
{
	if (clk->parent == clk->parents[AB_INTCLK_PARENT_ULPCLK]) {
		return ab8500_sysctrl_write(AB8500_SYSULPCLKCTRL1,
			AB8500_SYSULPCLKCTRL1_SYSULPCLKINTSEL_MASK,
			(1 << AB8500_SYSULPCLKCTRL1_SYSULPCLKINTSEL_SHIFT));
	}
	return 0;
}

static void ab_intclk_disable(struct clk *clk)
{
	if (clk->parent == clk->parents[AB_INTCLK_PARENT_SYSCLK])
		return;

	if (ab8500_sysctrl_clear(AB8500_SYSULPCLKCTRL1,
		AB8500_SYSULPCLKCTRL1_SYSULPCLKINTSEL_MASK)) {
		pr_err("clock: %s failed to disable %s.\n", __func__,
			clk->name);
	}
}

static int ab_intclk_set_parent(struct clk *clk, struct clk *parent)
{
	int err;

	if (!clk->enabled)
		return 0;

	err = __clk_enable(parent, clk->mutex);

	if (unlikely(err))
		goto parent_enable_error;

	if (parent == clk->parents[AB_INTCLK_PARENT_ULPCLK]) {
		err = ab8500_sysctrl_write(AB8500_SYSULPCLKCTRL1,
			AB8500_SYSULPCLKCTRL1_SYSULPCLKINTSEL_MASK,
			(1 << AB8500_SYSULPCLKCTRL1_SYSULPCLKINTSEL_SHIFT));
	} else {
		err = ab8500_sysctrl_clear(AB8500_SYSULPCLKCTRL1,
			AB8500_SYSULPCLKCTRL1_SYSULPCLKINTSEL_MASK);
	}
	if (unlikely(err))
		goto config_error;

	__clk_disable(clk->parent, clk->mutex);

	return 0;

config_error:
	__clk_disable(parent, clk->mutex);
parent_enable_error:
	return err;
}

static struct clkops ab_intclk_ops = {
	.enable = ab_intclk_enable,
	.disable = ab_intclk_disable,
	.set_parent = ab_intclk_set_parent,
};

/* AB8500 audio clock operations */

static int audioclk_enable(struct clk *clk)
{
	return ab8500_sysctrl_set(AB8500_SYSULPCLKCTRL1,
		AB8500_SYSULPCLKCTRL1_AUDIOCLKENA);
}

static void audioclk_disable(struct clk *clk)
{
	if (ab8500_sysctrl_clear(AB8500_SYSULPCLKCTRL1,
		AB8500_SYSULPCLKCTRL1_AUDIOCLKENA)) {
		pr_err("clock: %s failed to disable %s.\n", __func__,
			clk->name);
	}
}

static struct clkops audioclk_ops = {
	.enable = audioclk_enable,
	.disable = audioclk_disable,
};

/* Primary camera clock operations */
static int clkout0_enable(struct clk *clk)
{
	int r;

	if (clk->regulator == NULL) {
		struct regulator *reg;

		reg = regulator_get(NULL, "v-ape");
		if (IS_ERR(reg))
			return PTR_ERR(reg);
		clk->regulator = reg;
	}
	r = regulator_enable(clk->regulator);
	if (r)
		goto regulator_failed;
	r = prcmu_config_clkout(0, PRCMU_CLKSRC_CLK38M, 4);
	if (r)
		goto config_failed;
	r = nmk_config_pin(GPIO227_CLKOUT1, false);
	if (r)
		goto gpio_failed;
	return r;

gpio_failed:
	(void)prcmu_config_clkout(0, PRCMU_CLKSRC_CLK38M, 0);
config_failed:
	(void)regulator_disable(clk->regulator);
regulator_failed:
	return r;
}

static void clkout0_disable(struct clk *clk)
{
	int r;

	r = nmk_config_pin((GPIO227_GPIO | PIN_OUTPUT_LOW), false);
	if (r)
		goto disable_failed;
	(void)prcmu_config_clkout(0, PRCMU_CLKSRC_CLK38M, 0);
	(void)regulator_disable(clk->regulator);
	return;

disable_failed:
	pr_err("clock: failed to disable %s.\n", clk->name);
}

/* Touch screen/secondary camera clock operations. */
static int clkout1_enable(struct clk *clk)
{
	int r;

	if (clk->regulator == NULL) {
		struct regulator *reg;

		reg = regulator_get(NULL, "v-ape");
		if (IS_ERR(reg))
			return PTR_ERR(reg);
		clk->regulator = reg;
	}
	r = regulator_enable(clk->regulator);
	if (r)
		goto regulator_failed;
	r = prcmu_config_clkout(1, PRCMU_CLKSRC_SYSCLK, 2);
	if (r)
		goto config_failed;
	r = nmk_config_pin(GPIO228_CLKOUT2, false);
	if (r)
		goto gpio_failed;
	return r;

gpio_failed:
	(void)prcmu_config_clkout(1, PRCMU_CLKSRC_SYSCLK, 0);
config_failed:
	(void)regulator_disable(clk->regulator);
regulator_failed:
	return r;
}

static void clkout1_disable(struct clk *clk)
{
	int r;

	r = nmk_config_pin((GPIO228_GPIO | PIN_OUTPUT_LOW), false);
	if (r)
		goto disable_failed;
	(void)prcmu_config_clkout(1, PRCMU_CLKSRC_SYSCLK, 0);
	(void)regulator_disable(clk->regulator);
	return;

disable_failed:
	pr_err("clock: failed to disable %s.\n", clk->name);
}

static struct clkops clkout0_ops = {
	.enable = clkout0_enable,
	.disable = clkout0_disable,
};

static struct clkops clkout1_ops = {
	.enable = clkout1_enable,
	.disable = clkout1_disable,
};

#define DEF_PER1_PCLK(_cg_bit, _name) \
	DEF_PRCC_PCLK(_name, U8500_CLKRST1_BASE, _cg_bit, &per1clk)
#define DEF_PER2_PCLK(_cg_bit, _name) \
	DEF_PRCC_PCLK(_name, U8500_CLKRST2_BASE, _cg_bit, &per2clk)
#define DEF_PER3_PCLK(_cg_bit, _name) \
	DEF_PRCC_PCLK(_name, U8500_CLKRST3_BASE, _cg_bit, &per3clk)
#define DEF_PER5_PCLK(_cg_bit, _name) \
	DEF_PRCC_PCLK(_name, U8500_CLKRST5_BASE, _cg_bit, &per5clk)
#define DEF_PER6_PCLK(_cg_bit, _name) \
	DEF_PRCC_PCLK(_name, U8500_CLKRST6_BASE, _cg_bit, &per6clk)

#define DEF_PER1_KCLK(_cg_bit, _name, _parent) \
	DEF_PRCC_KCLK(_name, U8500_CLKRST1_BASE, _cg_bit, _parent, &per1clk)
#define DEF_PER2_KCLK(_cg_bit, _name, _parent) \
	DEF_PRCC_KCLK(_name, U8500_CLKRST2_BASE, _cg_bit, _parent, &per2clk)
#define DEF_PER3_KCLK(_cg_bit, _name, _parent) \
	DEF_PRCC_KCLK(_name, U8500_CLKRST3_BASE, _cg_bit, _parent, &per3clk)
#define DEF_PER5_KCLK(_cg_bit, _name, _parent) \
	DEF_PRCC_KCLK(_name, U8500_CLKRST5_BASE, _cg_bit, _parent, &per5clk)
#define DEF_PER6_KCLK(_cg_bit, _name, _parent) \
	DEF_PRCC_KCLK(_name, U8500_CLKRST6_BASE, _cg_bit, _parent, &per6clk)

/* Clock sources. */

static struct clk soc0_pll = {
	.name = "soc0_pll",
	.ops = &prcmu_clk_ops,
	.cg_sel = PRCMU_PLLSOC0,
	.mutex = &soc0_pll_mutex,
};

static struct clk soc1_pll = {
	.name = "soc1_pll",
	.ops = &prcmu_clk_ops,
	.cg_sel = PRCMU_PLLSOC1,
	.mutex = &soc1_pll_mutex,
};

static struct clk ddr_pll = {
	.name = "ddr_pll",
	.ops = &pll_ops,
	.cg_sel = PRCMU_PLLDDR,
};

static struct clk ulp38m4 = {
	.name = "ulp38m4",
	.rate = 38400000,
};

static struct clk sysclk = {
	.name = "sysclk",
	.ops = &sysclk_ops,
	.rate = 38400000,
	.mutex = &sysclk_mutex,
};

static struct clk sysclk2 = {
	.name = "sysclk2",
	.ops = &sysclk_ops,
	.cg_sel = AB8500_SYSULPCLKCTRL1_SYSCLKBUF2REQ,
	.rate = 38400000,
	.mutex = &sysclk_mutex,
};

static struct clk sysclk3 = {
	.name = "sysclk3",
	.ops = &sysclk_ops,
	.cg_sel = AB8500_SYSULPCLKCTRL1_SYSCLKBUF3REQ,
	.rate = 38400000,
	.mutex = &sysclk_mutex,
};

static struct clk sysclk4 = {
	.name = "sysclk4",
	.ops = &sysclk_ops,
	.cg_sel = AB8500_SYSULPCLKCTRL1_SYSCLKBUF4REQ,
	.rate = 38400000,
	.mutex = &sysclk_mutex,
};

static struct clk rtc32k = {
	.name = "rtc32k",
	.rate = 32768,
};

static struct clk clkout0 = {
	.name = "clkout0",
	.ops = &clkout0_ops,
	.parent = &ulp38m4,
	.rate = 9600000,
	.mutex = &clkout0_mutex,
};

static struct clk clkout1 = {
	.name = "clkout1",
	.ops = &clkout1_ops,
	.parent = &sysclk,
	.rate = 9600000,
	.mutex = &sysclk_mutex,
};

static struct clk ab_ulpclk = {
	.name = "ab_ulpclk",
	.ops = &ab_ulpclk_ops,
	.rate = 38400000,
	.mutex = &ab_ulpclk_mutex,
};

static struct clk *ab_intclk_parents[NUM_AB_INTCLK_PARENTS] = {
	[AB_INTCLK_PARENT_SYSCLK] = &sysclk,
	[AB_INTCLK_PARENT_ULPCLK] = &ab_ulpclk,
	[AB_INTCLK_PARENTS_END] = NULL,
};

static struct clk ab_intclk = {
	.name = "ab_intclk",
	.ops = &ab_intclk_ops,
	.mutex = &ab_intclk_mutex,
	.parent = &sysclk,
	.parents = ab_intclk_parents,
};

static struct clk audioclk = {
	.name = "audioclk",
	.ops = &audioclk_ops,
	.mutex = &ab_intclk_mutex,
	.parent = &ab_intclk,
};

static DEF_PRCMU_CLK(sgaclk, PRCMU_SGACLK, 320000000);
static DEF_PRCMU_CLK(uartclk, PRCMU_UARTCLK, 38400000);
static DEF_PRCMU_CLK(msp02clk, PRCMU_MSP02CLK, 19200000);
static DEF_PRCMU_CLK(msp1clk, PRCMU_MSP1CLK, 19200000);
static DEF_PRCMU_CLK(i2cclk, PRCMU_I2CCLK, 24000000);
static DEF_PRCMU_CLK(slimclk, PRCMU_SLIMCLK, 19200000);
static DEF_PRCMU_CLK(per1clk, PRCMU_PER1CLK, 133330000);
static DEF_PRCMU_CLK(per2clk, PRCMU_PER2CLK, 133330000);
static DEF_PRCMU_CLK(per3clk, PRCMU_PER3CLK, 133330000);
static DEF_PRCMU_CLK(per5clk, PRCMU_PER5CLK, 133330000);
static DEF_PRCMU_CLK(per6clk, PRCMU_PER6CLK, 133330000);
static DEF_PRCMU_CLK(per7clk, PRCMU_PER7CLK, 100000000);
static DEF_PRCMU_SCALABLE_CLK(lcdclk, PRCMU_LCDCLK);
static DEF_PRCMU_OPP100_CLK(bmlclk, PRCMU_BMLCLK, 200000000);
static DEF_PRCMU_SCALABLE_CLK(hsitxclk, PRCMU_HSITXCLK);
static DEF_PRCMU_SCALABLE_CLK(hsirxclk, PRCMU_HSIRXCLK);
static DEF_PRCMU_SCALABLE_CLK(hdmiclk, PRCMU_HDMICLK);
static DEF_PRCMU_CLK(apeatclk, PRCMU_APEATCLK, 160000000);
static DEF_PRCMU_CLK(apetraceclk, PRCMU_APETRACECLK, 160000000);
static DEF_PRCMU_CLK(mcdeclk, PRCMU_MCDECLK, 160000000);
static DEF_PRCMU_OPP100_CLK(ipi2cclk, PRCMU_IPI2CCLK, 24000000);
static DEF_PRCMU_CLK(dsialtclk, PRCMU_DSIALTCLK, 384000000);
static DEF_PRCMU_CLK(dmaclk, PRCMU_DMACLK, 200000000);
static DEF_PRCMU_CLK(b2r2clk, PRCMU_B2R2CLK, 200000000);
static DEF_PRCMU_SCALABLE_CLK(tvclk, PRCMU_TVCLK);
/* TODO: For SSPCLK, the spec says 24MHz, while the old driver says 48MHz. */
static DEF_PRCMU_CLK(sspclk, PRCMU_SSPCLK, 24000000);
static DEF_PRCMU_CLK(rngclk, PRCMU_RNGCLK, 19200000);
static DEF_PRCMU_CLK(uiccclk, PRCMU_UICCCLK, 48000000);
static DEF_PRCMU_CLK(timclk, PRCMU_TIMCLK, 2400000);
static DEF_PRCMU_CLK(sdmmcclk, PRCMU_SDMMCCLK, 50000000);

static struct clk dsi_pll = {
	.name = "dsi_pll",
	.ops = &prcmu_scalable_clk_ops,
	.cg_sel = PRCMU_PLLDSI,
	.parent = &hdmiclk,
	.mutex = &dsi_pll_mutex,
};

static struct clk dsi0clk = {
	.name = "dsi0clk",
	.ops = &prcmu_scalable_clk_ops,
	.cg_sel = PRCMU_DSI0CLK,
	.parent = &dsi_pll,
	.mutex = &dsi_pll_mutex,
};

static struct clk dsi1clk = {
	.name = "dsi1clk",
	.ops = &prcmu_scalable_clk_ops,
	.cg_sel = PRCMU_DSI1CLK,
	.parent = &dsi_pll,
	.mutex = &dsi_pll_mutex,
};

static struct clk dsi0escclk = {
	.name = "dsi0escclk",
	.ops = &prcmu_scalable_clk_ops,
	.cg_sel = PRCMU_DSI0ESCCLK,
	.parent = &tvclk,
};

static struct clk dsi1escclk = {
	.name = "dsi1escclk",
	.ops = &prcmu_scalable_clk_ops,
	.cg_sel = PRCMU_DSI1ESCCLK,
	.parent = &tvclk,
};

static struct clk dsi2escclk = {
	.name = "dsi2escclk",
	.ops = &prcmu_scalable_clk_ops,
	.cg_sel = PRCMU_DSI2ESCCLK,
	.parent = &tvclk,
};

/* PRCC PClocks */

static DEF_PER1_PCLK(0, p1_pclk0);
static DEF_PER1_PCLK(1, p1_pclk1);
static DEF_PER1_PCLK(2, p1_pclk2);
static DEF_PER1_PCLK(3, p1_pclk3);
static DEF_PER1_PCLK(4, p1_pclk4);
static DEF_PER1_PCLK(5, p1_pclk5);
static DEF_PER1_PCLK(6, p1_pclk6);
static DEF_PER1_PCLK(7, p1_pclk7);
static DEF_PER1_PCLK(8, p1_pclk8);
static DEF_PER1_PCLK(9, p1_pclk9);
static DEF_PER1_PCLK(10, p1_pclk10);
static DEF_PER1_PCLK(11, p1_pclk11);

static DEF_PER2_PCLK(0, p2_pclk0);
static DEF_PER2_PCLK(1, p2_pclk1);
static DEF_PER2_PCLK(2, p2_pclk2);
static DEF_PER2_PCLK(3, p2_pclk3);
static DEF_PER2_PCLK(4, p2_pclk4);
static DEF_PER2_PCLK(5, p2_pclk5);
static DEF_PER2_PCLK(6, p2_pclk6);
static DEF_PER2_PCLK(7, p2_pclk7);
static DEF_PER2_PCLK(8, p2_pclk8);
static DEF_PER2_PCLK(9, p2_pclk9);
static DEF_PER2_PCLK(10, p2_pclk10);
static DEF_PER2_PCLK(11, p2_pclk11);

static DEF_PER3_PCLK(0, p3_pclk0);
static DEF_PER3_PCLK(1, p3_pclk1);
static DEF_PER3_PCLK(2, p3_pclk2);
static DEF_PER3_PCLK(3, p3_pclk3);
static DEF_PER3_PCLK(4, p3_pclk4);
static DEF_PER3_PCLK(5, p3_pclk5);
static DEF_PER3_PCLK(6, p3_pclk6);
static DEF_PER3_PCLK(7, p3_pclk7);
static DEF_PER3_PCLK(8, p3_pclk8);

static DEF_PER5_PCLK(0, p5_pclk0);
static DEF_PER5_PCLK(1, p5_pclk1);

static DEF_PER6_PCLK(0, p6_pclk0);
static DEF_PER6_PCLK(1, p6_pclk1);
static DEF_PER6_PCLK(2, p6_pclk2);
static DEF_PER6_PCLK(3, p6_pclk3);
static DEF_PER6_PCLK(4, p6_pclk4);
static DEF_PER6_PCLK(5, p6_pclk5);
static DEF_PER6_PCLK(6, p6_pclk6);
static DEF_PER6_PCLK(7, p6_pclk7);

/* UART0 */
static DEF_PER1_KCLK(0, p1_uart0_kclk, &uartclk);
static DEF_PER_CLK(p1_uart0_clk, &p1_pclk0, &p1_uart0_kclk);

/* UART1 */
static DEF_PER1_KCLK(1, p1_uart1_kclk, &uartclk);
static DEF_PER_CLK(p1_uart1_clk, &p1_pclk1, &p1_uart1_kclk);

/* I2C1 */
static DEF_PER1_KCLK(2, p1_i2c1_kclk, &i2cclk);
static DEF_PER_CLK(p1_i2c1_clk, &p1_pclk2, &p1_i2c1_kclk);

/* MSP0 */
static DEF_PER1_KCLK(3, p1_msp0_kclk, &msp02clk);
static DEF_PER_CLK(p1_msp0_clk, &p1_pclk3, &p1_msp0_kclk);

/* MSP1 */
static DEF_PER1_KCLK(4, p1_msp1_kclk, &msp1clk);
static DEF_PER_CLK(p1_msp1_clk, &p1_pclk4, &p1_msp1_kclk);

/* SDI0 */
static DEF_PER1_KCLK(5, p1_sdi0_kclk, &sdmmcclk);
static DEF_PER_CLK(p1_sdi0_clk, &p1_pclk5, &p1_sdi0_kclk);

/* I2C2 */
static DEF_PER1_KCLK(6, p1_i2c2_kclk, &i2cclk);
static DEF_PER_CLK(p1_i2c2_clk, &p1_pclk6, &p1_i2c2_kclk);

/* SLIMBUS0 */
static DEF_PER1_KCLK(3, p1_slimbus0_kclk, &slimclk);
static DEF_PER_CLK(p1_slimbus0_clk, &p1_pclk8, &p1_slimbus0_kclk);

/* I2C4 */
static DEF_PER1_KCLK(9, p1_i2c4_kclk, &i2cclk);
static DEF_PER_CLK(p1_i2c4_clk, &p1_pclk10, &p1_i2c4_kclk);

/* MSP3 */
static DEF_PER1_KCLK(10, p1_msp3_kclk, &msp1clk);
static DEF_PER_CLK(p1_msp3_clk, &p1_pclk11, &p1_msp3_kclk);

/* I2C3 */
static DEF_PER2_KCLK(0, p2_i2c3_kclk, &i2cclk);
static DEF_PER_CLK(p2_i2c3_clk, &p2_pclk0, &p2_i2c3_kclk);

/* SDI4 */
static DEF_PER2_KCLK(2, p2_sdi4_kclk, &sdmmcclk);
static DEF_PER_CLK(p2_sdi4_clk, &p2_pclk4, &p2_sdi4_kclk);

/* MSP2 */
static DEF_PER2_KCLK(3, p2_msp2_kclk, &msp02clk);
static DEF_PER_CLK(p2_msp2_clk, &p2_pclk5, &p2_msp2_kclk);

/* SDI1 */
static DEF_PER2_KCLK(4, p2_sdi1_kclk, &sdmmcclk);
static DEF_PER_CLK(p2_sdi1_clk, &p2_pclk6, &p2_sdi1_kclk);

/* SDI3 */
static DEF_PER2_KCLK(5, p2_sdi3_kclk, &sdmmcclk);
static DEF_PER_CLK(p2_sdi3_clk, &p2_pclk7, &p2_sdi3_kclk);

/* HSIR */
static struct clk p2_ssirx_kclk = {
	.name = "p2_ssirx_kclk",
	.ops = &prcc_kclk_rec_ops,
	.io_base = U8500_CLKRST2_BASE,
	.cg_sel = BIT(6),
	.parent = &hsirxclk,
	.clock = &per2clk,
};

/* HSIT */
static struct clk p2_ssitx_kclk = {
	.name = "p2_ssitx_kclk",
	.ops = &prcc_kclk_rec_ops,
	.io_base = U8500_CLKRST2_BASE,
	.cg_sel = BIT(7),
	.parent = &hsitxclk,
	.clock = &per2clk,
};

/* SSP0 */
static DEF_PER3_KCLK(1, p3_ssp0_kclk, &sspclk);
static DEF_PER_CLK(p3_ssp0_clk, &p3_pclk1, &p3_ssp0_kclk);

/* SSP1 */
static DEF_PER3_KCLK(2, p3_ssp1_kclk, &sspclk);
static DEF_PER_CLK(p3_ssp1_clk, &p3_pclk2, &p3_ssp1_kclk);

/* I2C0 */
static DEF_PER3_KCLK(3, p3_i2c0_kclk, &i2cclk);
static DEF_PER_CLK(p3_i2c0_clk, &p3_pclk3, &p3_i2c0_kclk);

/* SDI2 */
static DEF_PER3_KCLK(4, p3_sdi2_kclk, &sdmmcclk);
static DEF_PER_CLK(p3_sdi2_clk, &p3_pclk4, &p3_sdi2_kclk);

/* SKE */
static DEF_PER3_KCLK(5, p3_ske_kclk, &rtc32k);
static DEF_PER_CLK(p3_ske_clk, &p3_pclk5, &p3_ske_kclk);

/* UART2 */
static DEF_PER3_KCLK(6, p3_uart2_kclk, &uartclk);
static DEF_PER_CLK(p3_uart2_clk, &p3_pclk6, &p3_uart2_kclk);

/* SDI5 */
static DEF_PER3_KCLK(7, p3_sdi5_kclk, &sdmmcclk);
static DEF_PER_CLK(p3_sdi5_clk, &p3_pclk7, &p3_sdi5_kclk);

/* RNG */
static DEF_PER6_KCLK(0, p6_rng_kclk, &rngclk);
static DEF_PER_CLK(p6_rng_clk, &p6_pclk0, &p6_rng_kclk);

/* MTU:S */

/* MTU0 */
static DEF_PER_CLK(p6_mtu0_clk, &p6_pclk6, &timclk);

/* MTU1 */
static DEF_PER_CLK(p6_mtu1_clk, &p6_pclk7, &timclk);

/*
 * TODO: Ensure names match with devices and then remove unnecessary entries
 * when all drivers use the clk API.
 */

static struct clk_lookup u8500_clocks[] = {
	CLK_LOOKUP(soc0_pll, NULL, "soc0_pll"),
	CLK_LOOKUP(soc1_pll, NULL, "soc1_pll"),
	CLK_LOOKUP(ddr_pll, NULL, "ddr_pll"),
	CLK_LOOKUP(ulp38m4, NULL, "ulp38m4"),
	CLK_LOOKUP(sysclk, NULL, "sysclk"),
	CLK_LOOKUP(rtc32k, NULL, "clk32k"),
	CLK_LOOKUP(rtc32k, "rtc-pl031", NULL),
	CLK_LOOKUP(sysclk, "ab8500-usb.0", "sysclk"),
	CLK_LOOKUP(sysclk, "ab8500-codec.0", "sysclk"),
	CLK_LOOKUP(ab_ulpclk, "ab8500-codec.0", "ulpclk"),
	CLK_LOOKUP(ab_intclk, "ab8500-codec.0", "intclk"),
	CLK_LOOKUP(audioclk, "ab8500-codec.0", "audioclk"),
	CLK_LOOKUP(ab_intclk, "ab8500-pwm.1", NULL),

	CLK_LOOKUP(clkout0, "pri-cam", NULL),
	CLK_LOOKUP(clkout1, "3-005c", NULL),
	CLK_LOOKUP(clkout1, "3-005d", NULL),
	CLK_LOOKUP(clkout1, "sec-cam", NULL),

	/* prcmu */
	CLK_LOOKUP(sgaclk, "mali", NULL),
	CLK_LOOKUP(uartclk, "UART", NULL),
	CLK_LOOKUP(msp02clk, "MSP02", NULL),
	CLK_LOOKUP(i2cclk, "I2C", NULL),
	CLK_LOOKUP(sdmmcclk, "sdmmc", NULL),
	CLK_LOOKUP(slimclk, "slim", NULL),
	CLK_LOOKUP(per1clk, "PERIPH1", NULL),
	CLK_LOOKUP(per2clk, "PERIPH2", NULL),
	CLK_LOOKUP(per3clk, "PERIPH3", NULL),
	CLK_LOOKUP(per5clk, "PERIPH5", NULL),
	CLK_LOOKUP(per6clk, "PERIPH6", NULL),
	CLK_LOOKUP(per7clk, "PERIPH7", NULL),
	CLK_LOOKUP(lcdclk, "lcd", NULL),
	CLK_LOOKUP(bmlclk, "bml", NULL),
	CLK_LOOKUP(p2_ssitx_kclk, "ste_hsi.0", "hsit_hsitxclk"),
	CLK_LOOKUP(p2_ssirx_kclk, "ste_hsi.0", "hsir_hsirxclk"),
	CLK_LOOKUP(lcdclk, "mcde", "lcd"),
	CLK_LOOKUP(hdmiclk, "hdmi", NULL),
	CLK_LOOKUP(hdmiclk, "mcde", "hdmi"),
	CLK_LOOKUP(apeatclk, "apeat", NULL),
	CLK_LOOKUP(apetraceclk, "apetrace", NULL),
	CLK_LOOKUP(mcdeclk, "mcde", NULL),
	CLK_LOOKUP(mcdeclk, "mcde", "mcde"),
	CLK_LOOKUP(mcdeclk, "dsilink.0", "dsisys"),
	CLK_LOOKUP(mcdeclk, "dsilink.1", "dsisys"),
	CLK_LOOKUP(mcdeclk, "dsilink.2", "dsisys"),
	CLK_LOOKUP(ipi2cclk, "ipi2", NULL),
	CLK_LOOKUP(dmaclk, "dma40.0", NULL),
	CLK_LOOKUP(b2r2clk, "b2r2", NULL),
	CLK_LOOKUP(b2r2clk, "b2r2_core", NULL),
	CLK_LOOKUP(b2r2clk, "U8500-B2R2.0", NULL),
	CLK_LOOKUP(tvclk, "tv", NULL),
	CLK_LOOKUP(tvclk, "mcde", "tv"),
	CLK_LOOKUP(dsi0escclk, "dsilink.0", "dsilp0"),
	CLK_LOOKUP(dsi1escclk, "dsilink.1", "dsilp1"),
	CLK_LOOKUP(dsi2escclk, "dsilink.2", "dsilp2"),
	CLK_LOOKUP(msp1clk, "MSP1", NULL),
	CLK_LOOKUP(dsialtclk, "dsialt", NULL),
	CLK_LOOKUP(sspclk, "SSP", NULL),
	CLK_LOOKUP(rngclk, "rngclk", NULL),
	CLK_LOOKUP(uiccclk, "uicc", NULL),
	CLK_LOOKUP(dsi0clk, "mcde", "dsihs0"),
	CLK_LOOKUP(dsi1clk, "mcde", "dsihs1"),
	CLK_LOOKUP(dsi_pll, "mcde", "dsihs2"),
	CLK_LOOKUP(dsi0escclk, "mcde", "dsilp0"),
	CLK_LOOKUP(dsi1escclk, "mcde", "dsilp1"),
	CLK_LOOKUP(dsi2escclk, "mcde", "dsilp2"),
	CLK_LOOKUP(dsi0clk, "dsilink.0", "dsihs0"),
	CLK_LOOKUP(dsi1clk, "dsilink.1", "dsihs1"),
	CLK_LOOKUP(dsi_pll, "dsilink.2", "dsihs2"),

	/* PERIPH 1 */
	CLK_LOOKUP(p1_msp3_clk, "msp3", NULL),
	CLK_LOOKUP(p1_msp3_clk, "ux500-msp-i2s.3", NULL),
	CLK_LOOKUP(p1_msp3_kclk, "ab8500-codec.0", "msp3-kernel"),
	CLK_LOOKUP(p1_pclk11, "ab8500-codec.0", "msp3-bus"),
	CLK_LOOKUP(p1_uart0_clk, "uart0", NULL),
	CLK_LOOKUP(p1_uart1_clk, "uart1", NULL),
	CLK_LOOKUP(p1_i2c1_clk, "nmk-i2c.1", NULL),
	CLK_LOOKUP(p1_msp0_clk, "msp0", NULL),
	CLK_LOOKUP(p1_msp0_clk, "ux500-msp-i2s.0", NULL),
	CLK_LOOKUP(p1_sdi0_clk, "sdi0", NULL),
	CLK_LOOKUP(p1_i2c2_clk, "nmk-i2c.2", NULL),
	CLK_LOOKUP(p1_slimbus0_clk, "slimbus0", NULL),
	CLK_LOOKUP(p1_pclk9, "gpio.0", NULL),
	CLK_LOOKUP(p1_pclk9, "gpio.1", NULL),
	CLK_LOOKUP(p1_pclk9, "gpioblock0", NULL),
	CLK_LOOKUP(p1_msp1_clk, "msp1", NULL),
	CLK_LOOKUP(p1_msp1_clk, "ux500-msp-i2s.1", NULL),
	CLK_LOOKUP(p1_msp1_kclk, "ab8500-codec.0", "msp1-kernel"),
	CLK_LOOKUP(p1_pclk4, "ab8500-codec.0", "msp1-bus"),
	CLK_LOOKUP(p1_pclk7, "spi3", NULL),
	CLK_LOOKUP(p1_i2c4_clk, "nmk-i2c.4", NULL),

	/* PERIPH 2 */
	CLK_LOOKUP(p2_i2c3_clk, "nmk-i2c.3", NULL),
	CLK_LOOKUP(p2_pclk1, "spi2", NULL),
	CLK_LOOKUP(p2_pclk2, "spi1", NULL),
	CLK_LOOKUP(p2_pclk3, "pwl", NULL),
	CLK_LOOKUP(p2_sdi4_clk, "sdi4", NULL),
	CLK_LOOKUP(p2_msp2_clk, "msp2", NULL),
	CLK_LOOKUP(p2_msp2_clk, "ux500-msp-i2s.2", NULL),
	CLK_LOOKUP(p2_sdi1_clk, "sdi1", NULL),
	CLK_LOOKUP(p2_sdi3_clk, "sdi3", NULL),
	CLK_LOOKUP(p2_pclk8, "spi0", NULL),
	CLK_LOOKUP(p2_pclk9, "ste_hsi.0", "hsir_hclk"),
	CLK_LOOKUP(p2_pclk10, "ste_hsi.0", "hsit_hclk"),
	CLK_LOOKUP(p2_pclk11, "gpio.6", NULL),
	CLK_LOOKUP(p2_pclk11, "gpio.7", NULL),
	CLK_LOOKUP(p2_pclk11, "gpioblock1", NULL),

	/* PERIPH 3 */
	CLK_LOOKUP(p3_pclk0, "fsmc", NULL),
	CLK_LOOKUP(p3_i2c0_clk, "nmk-i2c.0", NULL),
	CLK_LOOKUP(p3_sdi2_clk, "sdi2", NULL),
	CLK_LOOKUP(p3_ske_clk, "ske", NULL),
	CLK_LOOKUP(p3_ske_clk, "nmk-ske-keypad", NULL),
	CLK_LOOKUP(p3_uart2_clk, "uart2", NULL),
	CLK_LOOKUP(p3_sdi5_clk, "sdi5", NULL),
	CLK_LOOKUP(p3_pclk8, "gpio.2", NULL),
	CLK_LOOKUP(p3_pclk8, "gpio.3", NULL),
	CLK_LOOKUP(p3_pclk8, "gpio.4", NULL),
	CLK_LOOKUP(p3_pclk8, "gpio.5", NULL),
	CLK_LOOKUP(p3_pclk8, "gpioblock2", NULL),
	CLK_LOOKUP(p3_ssp0_clk, "ssp0", NULL),
	CLK_LOOKUP(p3_ssp1_clk, "ssp1", NULL),

	/* PERIPH 5 */
	CLK_LOOKUP(p5_pclk1, "gpio.8", NULL),
	CLK_LOOKUP(p5_pclk1, "gpioblock3", NULL),
	CLK_LOOKUP(p5_pclk0, "musb-ux500.0", "usb"),

	/* PERIPH 6 */
	CLK_LOOKUP(p6_pclk1, "cryp0", NULL),
	CLK_LOOKUP(p6_pclk2, "hash0", NULL),
	CLK_LOOKUP(p6_pclk3, "pka", NULL),
	CLK_LOOKUP(p6_pclk5, "cfgreg", NULL),
	CLK_LOOKUP(p6_mtu0_clk, "mtu0", NULL),
	CLK_LOOKUP(p6_mtu1_clk, "mtu1", NULL),
	CLK_LOOKUP(p6_pclk4, "hash1", NULL),
	CLK_LOOKUP(p6_pclk1, "cryp1", NULL),
	CLK_LOOKUP(p6_rng_clk, "rng", NULL),

};

static struct clk_lookup u8500_v2_sysclks[] = {
	CLK_LOOKUP(sysclk2, NULL, "sysclk2"),
	CLK_LOOKUP(sysclk3, NULL, "sysclk3"),
	CLK_LOOKUP(sysclk4, NULL, "sysclk4"),
};

static void sysclk_init_disable(struct work_struct *not_used)
{
	int i;

	mutex_lock(&sysclk_mutex);

	/* Enable SWAT  */
	if (ab8500_sysctrl_set(AB8500_SWATCTRL, AB8500_SWATCTRL_SWATENABLE))
		goto err_swat;

	for (i = 0; i < ARRAY_SIZE(u8500_v2_sysclks); i++) {
		struct clk *clk = u8500_v2_sysclks[i].clk;

		/* Disable sysclks */
		if (!clk->enabled && clk->cg_sel) {
			if (ab8500_sysctrl_clear(AB8500_SYSULPCLKCTRL1,
				(u8)clk->cg_sel))
				goto err_sysclk;
		}
	}
	goto unlock_and_exit;

err_sysclk:
	pr_err("clock: Disable %s failed", u8500_v2_sysclks[i].clk->name);
	ab8500_sysctrl_clear(AB8500_SWATCTRL, AB8500_SWATCTRL_SWATENABLE);
	goto unlock_and_exit;

err_swat:
	pr_err("clock: Enable SWAT failed");

unlock_and_exit:
	mutex_unlock(&sysclk_mutex);
}

static struct clk *db8500_dbg_clks[] __initdata = {
	/* Clock sources */
	&soc0_pll,
	&soc1_pll,
	&ddr_pll,
	&ulp38m4,
	&sysclk,
	&rtc32k,
	/* PRCMU clocks */
	&sgaclk,
	&uartclk,
	&msp02clk,
	&msp1clk,
	&i2cclk,
	&sdmmcclk,
	&slimclk,
	&per1clk,
	&per2clk,
	&per3clk,
	&per5clk,
	&per6clk,
	&per7clk,
	&lcdclk,
	&bmlclk,
	&hsitxclk,
	&hsirxclk,
	&hdmiclk,
	&apeatclk,
	&apetraceclk,
	&mcdeclk,
	&ipi2cclk,
	&dsialtclk,
	&dsi_pll,
	&dsi0clk,
	&dsi1clk,
	&dsi0escclk,
	&dsi1escclk,
	&dsi2escclk,
	&dmaclk,
	&b2r2clk,
	&tvclk,
	&sspclk,
	&rngclk,
	&uiccclk,
	&sysclk2,
	&clkout0,
	&clkout1,
	&p1_pclk0,
	&p1_pclk1,
	&p1_pclk2,
	&p1_pclk3,
	&p1_pclk4,
	&p1_pclk5,
	&p1_pclk6,
	&p1_pclk7,
	&p1_pclk8,
	&p1_pclk9,
	&p1_pclk10,
	&p1_pclk11,
	&p2_pclk0,
	&p2_pclk1,
	&p2_pclk2,
	&p2_pclk3,
	&p2_pclk4,
	&p2_pclk5,
	&p2_pclk6,
	&p2_pclk7,
	&p2_pclk8,
	&p2_pclk9,
	&p2_pclk10,
	&p2_pclk11,
	&p3_pclk0,
	&p3_pclk1,
	&p3_pclk2,
	&p3_pclk3,
	&p3_pclk4,
	&p3_pclk5,
	&p3_pclk6,
	&p3_pclk7,
	&p3_pclk8,
	&p5_pclk0,
	&p5_pclk1,
	&p6_pclk0,
	&p6_pclk1,
	&p6_pclk2,
	&p6_pclk3,
	&p6_pclk4,
	&p6_pclk5,
	&p6_pclk6,
	&p6_pclk7,
};

/* List of clocks which might be enabled from the bootloader */

/*
 * SOC settings enable bus + kernel clocks of all periphs without
 * properly configuring the parents of the kernel clocks for all units.
 * Enable and Disable them all to get them into a known and working state.
 */
static struct clk *loader_enabled_clk[] __initdata = {
	/* periph 1 */
	&p1_uart0_clk,
	&p1_uart1_clk,
	&p1_i2c1_clk,
	&p1_msp0_clk,
	&p1_msp1_clk,
	&p1_sdi0_clk,
	&p1_i2c2_clk,
	&p1_pclk7,		/* spi3 */
	&p1_pclk9,		/* gpioctrl */
	&p1_i2c4_clk,

	/* periph 2 */
	&p2_i2c3_clk,
	&p2_pclk1,		/* spi2 */
	&p2_pclk2,		/* spi1 */
	/* pwl has an unknown kclk parent, ignore it */
	&p2_sdi4_clk,
	&p2_msp2_clk,
	&p2_sdi1_clk,
	&p2_sdi3_clk,
	&p2_pclk8,		/* spi0 */
	&p2_ssirx_kclk,		/* hsir kernel */
	&p2_ssitx_kclk,		/* hsit kernel */
	&p2_pclk9,		/* hsir bus */
	&p2_pclk10,		/* hsit bus */
	&p2_pclk11,		/* gpioctrl */
	/* periph 3 */
	&p3_pclk0,		/* fsmc */
	&p3_ssp0_clk,
	&p3_ssp1_clk,
	&p3_i2c0_clk,
	&p3_sdi2_clk,
	&p3_ske_clk,
	&p3_uart2_clk,
	&p3_sdi5_clk,
	&p3_pclk8,		/* gpio */
	/* periph 5 */
	&p5_pclk0,		/* usb */
	&p5_pclk1,		/* gpio */
	/* periph 6 */
	/* Leave out rng, cryp0, hash0 and pka */
	&p6_pclk4,		/* hash1 */
	&p6_pclk5,		/* cr */
	&p6_mtu0_clk,
	&p6_mtu1_clk,
	/* periph 7 */
	&per7clk,		/* PERIPH7 */

	&bmlclk,		/* BML */
	&dsialtclk,		/* dsialt */
	&hsirxclk,		/* hsirx */
	&hsitxclk,		/* hsitx */
	&ipi2cclk,		/* ipi2 */
	&lcdclk,		/* mcde */
	&b2r2clk,		/* b2r2_bus */
};

static int __init init_clock_states(void)
{
	unsigned int i;
	/*
	 * Disable peripheral clocks enabled by bootloader/default
	 * but without drivers
	 */
	for (i = 0; i < ARRAY_SIZE(loader_enabled_clk); i++)
		if (!clk_enable(loader_enabled_clk[i]))
			clk_disable(loader_enabled_clk[i]);

	/*
	 * APEATCLK and APETRACECLK are enabled at boot and needed
	 * in order to debug with Lauterbach
	 */
	if (!clk_enable(&apeatclk)) {
		if (!ux500_jtag_enabled())
			clk_disable(&apeatclk);
	}
	if (!clk_enable(&apetraceclk)) {
		if (!ux500_jtag_enabled())
			clk_disable(&apetraceclk);
	}

	INIT_DELAYED_WORK(&sysclk_disable_work, sysclk_init_disable);
	schedule_delayed_work(&sysclk_disable_work, 10 * HZ);

	return 0;
}
late_initcall(init_clock_states);

static void __init configure_c2_clocks(void)
{
	sgaclk.parent = &soc0_pll;
	sgaclk.mutex = &soc0_pll_mutex;
}

int __init db8500_clk_init(void)
{
	struct prcmu_fw_version *fw_version;

	/*
	 * Disable pwl's and slimbus' bus and kernel clocks without touching
	 * any parents. Because for slimbus, the prcmu fw has not correctly
	 * configured the clocks at boot and for pwl the kclk parent
	 * is unknown.
	 */

	/* slimbus' bus and kernel clocks */
	writel(1 << 8, __io_address(U8500_CLKRST1_BASE) + PRCC_PCKDIS);
	writel(1 << 8, __io_address(U8500_CLKRST1_BASE) + PRCC_KCKDIS);
	/* pwl's bus and kernel clocks */
	writel(1 << 3, __io_address(U8500_CLKRST2_BASE) + PRCC_PCKDIS);
	writel(1 << 1, __io_address(U8500_CLKRST2_BASE) + PRCC_KCKDIS);

	fw_version = prcmu_get_fw_version();
	if (fw_version != NULL)
		switch (fw_version->project) {
		case PRCMU_FW_PROJECT_U8500_C2:
		case PRCMU_FW_PROJECT_U9500_C2:
		case PRCMU_FW_PROJECT_U8520:
		case PRCMU_FW_PROJECT_U8420:
			configure_c2_clocks();
			break;
		default:
			break;
		}
	clkdev_add_table(u8500_v2_sysclks,
		      ARRAY_SIZE(u8500_v2_sysclks));
	clkdev_add_table(u8500_clocks,
		      ARRAY_SIZE(u8500_clocks));
	return 0;
}

int __init db8500_clk_debug_init(void)
{
	return dbx500_clk_debug_init(db8500_dbg_clks,
				     ARRAY_SIZE(db8500_dbg_clks));
}
