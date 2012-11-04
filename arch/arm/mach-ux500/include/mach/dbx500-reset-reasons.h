/*
 * Copyright (C) ST Ericsson SA 2011
 * Author: Shreshtha Kumar SAHU <shreshthakumar.sahu@stericsson.com>
 * for ST Ericsson.
 *
 * License terms: GNU General Public License (GPL) version 2
 *
 * This file defines the hardware reset reasons.
 * The reboot reasons will be saved to a secure location in TCDM memory and
 * can be read at bootup by e.g. the bootloader, or at a later stage userspace
 * since the code is exposed through sysfs.
 */

#ifndef _DBX500_REBOOT_REASONS_H
#define _DBX500_REBOOT_REASONS_H

#include "reboot_reasons.h"

/*
 * These defines contains the hardware reset event that was the source
 * of the system reset. These values are exposed through a sysfs
 * entry under /sys/socinfo, see mach-ux500/cpu-db*500.c
 */
#define HW_RESET_MODEM		0x0100
#define HW_RESET_APE_RESTART	0x0080
#define HW_RESET_A9_RESTART	0x0040
#define HW_RESET_POR		0x0020
#define HW_RESET_SECURE_WD	0x0010
#define HW_RESET_APE		0x0008
#define HW_RESET_APE_SOFTWARE	0x0004
#define HW_RESET_A9_CPU1_WD	0x0002
#define HW_RESET_A9_CPU0_WD	0x0001

extern struct reboot_reason reboot_reasons_hw[];
extern unsigned int reboot_reasons_hw_size;

#endif
