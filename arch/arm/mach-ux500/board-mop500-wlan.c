/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <mach/irqs.h>
#include "pins.h"
#include <mach/cw1200_plat.h>
#include <linux/clk.h>

static void cw1200_release(struct device *dev);
static int cw1200_clk_ctrl(const struct cw1200_platform_data *pdata,
		bool enable);

static struct resource cw1200_pdp_resources[] = {
	{
		.start = 20,
		.end = 20,
		.flags = IORESOURCE_IO,
		.name = "cw1200_reset",
	},
	{
		.start = GPIO_TO_IRQ(169),
		.end = GPIO_TO_IRQ(169),
		.flags = IORESOURCE_IRQ,
		.name = "cw1200_irq",
	},
};


static struct cw1200_platform_data cw1200_platform_data = {
	.clk_ctrl = cw1200_clk_ctrl,
};

static struct platform_device cw1200_device = {
	.name = "cw1200_wlan",
	.dev = {
		.platform_data = &cw1200_platform_data,
		.release = cw1200_release,
		.init_name = "cw1200_wlan",
	},
};

static struct clk *clk_dev;

const struct cw1200_platform_data *cw1200_get_platform_data(void)
{
	return &cw1200_platform_data;
}
EXPORT_SYMBOL_GPL(cw1200_get_platform_data);

static int cw1200_clk_ctrl(const struct cw1200_platform_data *pdata,
		bool enable)
{
	static const char *clock_name = "sys_clk_out";
	int ret = 0;

	if (enable) {
		clk_dev = clk_get(&cw1200_device.dev, clock_name);
		if (IS_ERR(clk_dev)) {
			ret = PTR_ERR(clk_dev);
			dev_warn(&cw1200_device.dev,
				"%s: Failed to get clk '%s': %d\n",
				__func__, clock_name, ret);
		} else {
			ret = clk_enable(clk_dev);
			if (ret) {
				clk_put(clk_dev);
				dev_warn(&cw1200_device.dev,
					"%s: Failed to enable clk '%s': %d\n",
					__func__, clock_name, ret);
			}
		}
	} else {
		clk_disable(clk_dev);
		clk_put(clk_dev);
	}

	return ret;
}

int __init mop500_wlan_init(void)
{
	cw1200_device.num_resources = ARRAY_SIZE(cw1200_pdp_resources);
	cw1200_device.resource = cw1200_pdp_resources;

	cw1200_platform_data.mmc_id = "mmc2";

	cw1200_platform_data.reset = &cw1200_device.resource[0];
	cw1200_platform_data.irq = &cw1200_device.resource[1];

	cw1200_device.dev.release = cw1200_release;

	return WARN_ON(platform_device_register(&cw1200_device));
}

static void cw1200_release(struct device *dev)
{

}
