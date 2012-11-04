/*
 * Copyright (C) 2009 ST-Ericsson.
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <linux/mfd/dbx500-prcmu.h>
#include <mach/reboot_reasons.h>

static inline void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
#ifdef CONFIG_UX500_SOC_DB8500
	unsigned short reset_code;
	unsigned short preset_code;

	preset_code = reboot_reason_get_preset();

	if (preset_code != SW_RESET_CRASH)
		prcmu_system_reset(preset_code);
	else {
		reset_code = reboot_reason_code(cmd);
		prcmu_system_reset(reset_code);
	}
#endif
}

#endif
