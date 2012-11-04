/* drivers/input/misc/ab8500-forcecrash.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Michael Ambrus <michael.ambrus@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysrq.h>
#include <linux/time.h>
#include <linux/sysrq.h>
#include <linux/reboot.h>
#include <linux/ab8500-ponkey.h>
#include <mach/reboot_reasons.h>

/* Ponkey time control bits */
#define FORCED_KEY_TIMEOUT_MS	4500LL
#define EXTERN_KEY_TIMEOUT_MS	100LL
#define MS_TO_NS(T)		((T * NSEC_PER_MSEC) % NSEC_PER_SEC)
#define MS_TO_S(T)		((T * NSEC_PER_MSEC) / NSEC_PER_SEC)
#define SYSRQ_KEY		'g'

static bool forcecrash_on;
static struct platform_device *ppdev;

/*
 * Functions for enable/disable feature in runtime though
 * /sys/bus/platform/drivers/module/module.0/forcecrash_on
 */
static ssize_t forcecrash_on_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", (unsigned int)forcecrash_on);
}

static ssize_t forcecrash_on_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long i = 0;
	int rc;

	rc = strict_strtoul(buf, 10, &i);
	if (rc)
		return -EINVAL;

	forcecrash_on = (i != 0);
	return count;
}

static DEVICE_ATTR(forcecrash_on, S_IRUGO|S_IWUSR, forcecrash_on_show,
			forcecrash_on_store);

/*
 * Practicaly a copy & paste of Linux original sysrq handler for crashes
 * for key 'c'(sysrq_handle_crash). Only difference is setting of restart
 * reason.
 *
 * Usage of panic handler mechanism is important for SoMC core dump mechanism.
 * Therefore keep this as close to sysrq_handle_crash as possible please.
 */
static void sysrq_handle_forced_crash(int key)
{
	char *killer = NULL;

	printk(KERN_EMERG "Force crash triggered!\n");
	reboot_reason_preset("forced-crash");
	panic_on_oops = '1';

	wmb();
	*killer = 1;
}

static struct sysrq_key_op sysrq_forced_crash_op = {
	.handler	= sysrq_handle_forced_crash,
	.help_msg	= "ForceCrash",
	.action_msg	= "Trigger a forced crash",
	.enable_mask	= SYSRQ_ENABLE_DUMP,
};

/*
 * Forced crash detector
 *
 * Detect intention to force a crash and if satisfied actuate it.
 *
 * The function does not assume that key-press and key-release IRQ occur in a
 * certain order. Any such case is taken care of by pre-setting the default
 * time-stamp for key-down to an unreasonable long future time.
 */
void ab8500_forced_key_detect(enum ab8500_forced_key key)
{
	static const struct timespec t_timeout = {
		.tv_sec		= MS_TO_S(FORCED_KEY_TIMEOUT_MS),
		.tv_nsec	= MS_TO_NS(FORCED_KEY_TIMEOUT_MS)
	};
	static const struct timespec t_timeout2 = {
		.tv_sec		= MS_TO_S(EXTERN_KEY_TIMEOUT_MS),
		.tv_nsec	= MS_TO_NS(EXTERN_KEY_TIMEOUT_MS)
	};
	static struct timespec t_up = {
		.tv_sec		= 0,
		.tv_nsec	= 0
	};
	static struct timespec t_down = {
		.tv_sec		= LONG_MAX,
		.tv_nsec	= LONG_MAX
	};
	int deadline_passed;
	struct timespec t_diff;

	if (key == AB8500_PON_PRESSED) {
		ktime_get_ts(&t_down);
		dev_dbg(&(ppdev->dev),
			"PON key DOWN at ktime: [%ld.%ld]\n",
			t_down.tv_sec, t_down.tv_nsec);
	} else {
		ktime_get_ts(&t_up);
		dev_dbg(&(ppdev->dev),
			"PON key UP at ktime: [%ld.%ld]\n",
			t_up.tv_sec, t_up.tv_nsec);
		t_diff = timespec_sub(t_up, t_down);
		if (key == AB8500_EXTERNAL_EVENT)
			deadline_passed =
				timespec_compare(&t_diff, &t_timeout2);
		else
			deadline_passed =
				timespec_compare(&t_diff, &t_timeout);

		if (deadline_passed > 0) {
			if (!forcecrash_on)
				return;
			printk(KERN_WARNING
				"PON generates forced-crash [%ld.%ld]\n",
				t_diff.tv_sec, t_diff.tv_nsec);
			handle_sysrq('s');
			handle_sysrq('u');
			handle_sysrq('g');
		} else {
			dev_dbg(&(ppdev->dev),
				"PON key skipping forced-crash [%ld.%ld]\n",
				t_diff.tv_sec, t_diff.tv_nsec);
			t_down.tv_sec = LONG_MAX;
			t_down.tv_nsec = LONG_MAX;
			t_up.tv_sec = 0;
			t_up.tv_nsec = 0;
		}
	}
}

/*
 * Externally exposed public API to be used when PON key-combo should force a
 * crash.
 *
 * This function will act when PON-key is pressed down, and another stimuli is
 * intended in conjunction (for example another key).
 */
void ab8500_forced_key_combo_detect(void)
{
	ab8500_forced_key_detect(AB8500_EXTERNAL_EVENT);
}

int ab8500_forcecrash_init(struct platform_device *pdev)
{
	int ret;
	ppdev = pdev;

	ret = register_sysrq_key(SYSRQ_KEY, &sysrq_forced_crash_op);
	if (ret) {
		dev_err(&(pdev->dev),
			"PON registred crash-handler at key '%c' failed\n",
			SYSRQ_KEY);
		return ret;
	}
	ret = device_create_file(&pdev->dev, &dev_attr_forcecrash_on);
	return ret;
}

void ab8500_forcecrash_exit(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_forcecrash_on);
	if (unregister_sysrq_key(SYSRQ_KEY, &sysrq_forced_crash_op) != 0)
		dev_err(&(pdev->dev),
			"PON failed to un-register crash-handler at key '%c'\n",
			SYSRQ_KEY);
	ppdev = NULL;
}
