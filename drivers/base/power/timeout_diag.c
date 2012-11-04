/*
 * Copyright (C) 2012 Sony Ericsson Mobile Communications AB.
 * Copyright (c) 2012 Sony Mobile Communications AB
 *
 * Author: Michael Ambrus <michael.ambrus@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <asm/unwind.h>
#include <linux/delay.h>

#include "../base.h"
#include "power.h"

#define BUF_LEN 1024

static int suspend_resume_timeout;
static char msg_buf[BUF_LEN];
static void (*dpm_drv_timeout_orig_fun)(unsigned long data);
static struct dpm_drv_wd_data *wd_data;

/*
 * Return task_struct for the first process or thread matched by name
 * @name: Name to search for
 * @returns: task_struct or NULL if not found.
 */
static struct task_struct *find_ktask_by_name(char *name)
{
	struct task_struct *p, *t, *ret = NULL;
	for_each_process(p)  {
		t = p;
		do {
			/* Kernel threads do not have resources. Save some time
			 * by only considering these.
			 */
			if ((t->mm == NULL)) {
				if (strncmp(name,  t->comm, TASK_COMM_LEN) == 0)
					ret = t;
			}
			if (ret != NULL)
				break;

		} while_each_thread(p, t);
		if (ret != NULL)
			break;
	}
	return ret;
}

static int swapper_diagnostics(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	struct task_struct *ts;

	if (suspend_resume_timeout) {
#if NR_CPUS > 1
		int n_tries = 10;
		/*
		 * In a SMP system, other CPU's have just been ordered to stop
		 * via smp_send_stop() in panic(). Give those CPU's some time
		 * to complete and to put kernel_structs in consistent state
		 * before continuing. Note that GB and ICS implementation
		 * differ. This mechanism cover both and should be tolerant
		 * for future changes.
		 */

		for (n_tries = 10;
			n_tries && (num_online_cpus() > 1);
			n_tries--)
			mdelay(MSEC_PER_SEC/10);

		if (!n_tries)
			printk(KERN_ERR "Other CPU:s stuck while suspend/"
				"resume timeout. This crash will be partly"
				" unflushed/corrupt\n");
#endif

		printk(KERN_EMERG "%s", msg_buf);
		memset(msg_buf, 0, BUF_LEN);

		if (wd_data->tsk)
			ts = wd_data->tsk;
		else
			ts = find_ktask_by_name("suspend");

		if (ts != NULL) {
			printk(KERN_INFO "Device suspend/resume timeout "
				"detected. Backtrace follows. Thread: %d, "
				"groupleader: %d, name: %s\n",
				ts->pid, ts->tgid, ts->comm);
			unwind_backtrace(NULL, ts);
		} else {
			printk(KERN_ERR "Device suspend/resume timeout "
				"detected but suspend thread is missing "
				"probably killed. Can't back-trace.\n");
		}
		suspend_resume_timeout = 0;
	}
	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = swapper_diagnostics,
	.priority = (INT_MIN + 1) /* Make sure it's late, but not last */
};

/**
 * dpm_drv_timeout to be used instead of the default. Mimic the original
 * function but write in buffer instead for post-poned output to log.
 */
static void semc_dpm_drv_timeout(unsigned long data)
{
	struct device *dev;

	wd_data = (struct dpm_drv_wd_data *)data;
	dev = wd_data->dev;

	snprintf(msg_buf, BUF_LEN,
		"**** DPM device timeout: %s (%s): [<%08lx>]\n",
		dev_name(dev), (dev->driver ? dev->driver->name : "no driver"),
		(long unsigned int)dev
		);
	msg_buf[BUF_LEN-1] = 0;
	suspend_resume_timeout = 1;
	BUG();
}

static int __init crash_swapper_init(void)
{
	int rc;
	suspend_resume_timeout = 0;
	memset(msg_buf, 0, BUF_LEN);
	wd_data = NULL;

	rc = atomic_notifier_chain_register(
		&panic_notifier_list, &panic_block);
	if (rc == 0) {
		dpm_drv_timeout_orig_fun =
			device_pm_set_timout_handler(semc_dpm_drv_timeout);
	}

	return rc;
}

static void __exit crash_swapper_exit(void)
{
	device_pm_set_timout_handler(dpm_drv_timeout_orig_fun);
	atomic_notifier_chain_unregister(
		&panic_notifier_list, &panic_block);
}

module_init(crash_swapper_init)
module_exit(crash_swapper_exit)

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Michael Ambrus <michael.ambrus@sonymobile.com>");
MODULE_DESCRIPTION("Swapper suspend/resume extended self-analysis");
