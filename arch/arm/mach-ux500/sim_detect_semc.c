/* arch/arm/mach-ux500/sim_detect_semc.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/input.h>

static atomic_t	sim_status;
static struct input_dev *input_dev;

static ssize_t attr_simstatus_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&sim_status));
}

static ssize_t attr_simstatus_store(
			struct class *class,
			struct class_attribute *attr,
			const char *buf,
			size_t count)
{
	long status = 0;
	static int init = 1;
	ssize_t ret = -EINVAL;

	if (0 == count) {
		ret = 0;
		goto err_exit;
	}

	if (strict_strtol(buf, 10, &status))
		goto err_exit;

	if ((1 < status) || (status < 0))
		goto err_exit;

	atomic_set(&sim_status, status);
	pr_info("%s called, sim status %d\n", __func__, status);

	if (init) {
		init = 0;
		if (!status)
			__change_bit(SW_JACK_PHYSICAL_INSERT, input_dev->sw);
	}
	input_event(input_dev, EV_SW,
		    SW_JACK_PHYSICAL_INSERT, status);
	input_sync(input_dev);

	return count;

err_exit:
	pr_err("%s called, error parameter\n", __func__);
	return ret;
}

static struct class *sim_class;
static struct class_attribute sim_attributes =
	__ATTR(sim_status, 0660, attr_simstatus_show, attr_simstatus_store);

static int __init sim_detect_init(void)
{
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: input_allocate_device failed\n",
			__func__);
		err = -ENOMEM;
		goto out;
	}

	input_dev->name = "simdetect";
	input_set_capability(input_dev, EV_SW,
					     SW_JACK_PHYSICAL_INSERT);

	err = input_register_device(input_dev);
	if (err) {
		pr_err("%s: input_register_device failed!\n",
			__func__);
		input_free_device(input_dev);
		goto out;
	}

	sim_class = class_create(THIS_MODULE, "sim_card");
	if (IS_ERR(sim_class)) {
		err = PTR_ERR(sim_class);
		pr_err("%s: cannot create sim_card class\n", __func__);
		goto out_input_device;
	}

	err = class_create_file(sim_class, &sim_attributes);
	if (err) {
		pr_err("%s: cannot create sysfs file\n", __func__);
		goto out_class;
	}

	return 0;

out_class:
	class_destroy(sim_class);
out_input_device:
	input_unregister_device(input_dev);
out:
	pr_err("%s: cannot initialize, error %d\n", __func__, err);
	return err;
}
module_init(sim_detect_init);

static void __exit sim_detect_exit(void)
{
	class_remove_file(sim_class, &sim_attributes);
	class_destroy(sim_class);
	input_unregister_device(input_dev);
}
module_exit(sim_detect_exit);

MODULE_AUTHOR("NEIL GAO <neil.gao@sonyericsson.com>");
MODULE_DESCRIPTION("Detects SIM");
MODULE_ALIAS("SIM DETECT virtual driver");
MODULE_LICENSE("GPL v2");
