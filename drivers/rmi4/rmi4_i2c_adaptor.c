/*
 * RMI4 bus driver.
 * drivers/rmi4/rmi4_i2c_adaptor.c
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 *
 * Author: Joachim Holst <joachim.holst@sonyericsson.com>
 *
 * Based on rmi_bus by Synaptics and Unixphere.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/rmi4/rmi4.h>
#include <linux/platform_device.h>

/* INFO:
 * This is an RMI4 adapter. This is responsible for low-level communication
 * with the connected hardware.
 *
 * This is just a skeleton. In real life, this will probably
 * be an I2C or I2C driver with it's own probe functionality.
 * In that probe function, everything should be initialized as
 * required and the adapter should be registered on the RMI4 bus in
 * that function. Since we don't have access to all that, we hack
 * around a bit in this example driver to get the required
 * functionlity. */

static int rmi4_i2c_adapter_write_data(struct rmi4_core_device *dev, u16 addr,
				       u8 *buf, int len)
{
	dev_info(dev->dev.parent, "%s - Called\n", __func__);
	return 0;
}

static int rmi4_i2c_adapter_read_data(struct rmi4_core_device *dev, u16 addr,
				      u8 *buf, int len)
{
	dev_info(dev->dev.parent, "%s - Called\n", __func__);
	return 0;
}

static void rmi4_i2c_adapter_release(struct device *dev)
{
	dev_info(dev, "%s - Called\n", __func__);
}

static struct platform_device dev = {
	.name = "rmi4_i2c_adapeter",
	.dev = {
		.release = rmi4_i2c_adapter_release,
	},
};

struct rmi4_function_data fdata[] = {
	{
		.func_name = "fn01",
		.func_id = 0x01,
	},
	{
		.func_name = "fn34",
		.func_id = 34,
	}
};

static struct rmi4_core_device_data ddata = {
	.core_name = RMI4_CORE_DRIVER_NAME,
	.attn_gpio = 67,
	.num_functions = ARRAY_SIZE(fdata),
	.func_data = fdata,
};

static int __devinit rmi4_i2c_adapter_init(void)
{
	int ret;
	struct rmi4_comm_ops ops = {
		.chip_read = rmi4_i2c_adapter_write_data,
		.chip_write = rmi4_i2c_adapter_read_data,
	};

	pr_debug("%s - Called\n", __func__);

	ret = platform_device_register(&dev);
	if (ret)
		pr_info("%s - Failed to register base device\n",
			__func__);
	else
		pr_info("%s - base device registered\n", __func__);

	return rmi4_bus_register_adapter(&dev.dev, &ops, &ddata);
}

static void __devexit rmi4_i2c_adapter_exit(void)
{
	pr_debug("%s - Called\n", __func__);
	rmi4_bus_unregister_adapter(&dev.dev);
	platform_device_unregister(&dev);
}

module_init(rmi4_i2c_adapter_init);
module_exit(rmi4_i2c_adapter_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst@sonyerisson.com>");
MODULE_DESCRIPTION("RMI4 i2c adapter");
MODULE_LICENSE("GPL");
