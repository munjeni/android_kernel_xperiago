/*
 * Copyright (C) ST-Ericsson SA 2011
 * Author: Rickard Evertsson <rickard.evertsson@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL) version 2
 *
 * Use this file to customize your reboot / sw reset reasons. Add, remove or
 * modify reasons in reboot_reasons_sw[].
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <mach/reboot_reasons.h>
#include <mach/dbx500-reset-reasons.h>

struct reboot_reason reboot_reasons_sw[] = {
	{"panic", SW_RESET_CRASH},
	{"factory-reset", SW_RESET_FACTORY_RESET},
	{"recovery", SW_RESET_RECOVERY},
	{"charging", SW_RESET_CHARGING},
	{"coldstart", SW_RESET_COLDSTART},
	{"none", SW_RESET_NO_ARGUMENT}, /* Normal Boot */
	{"chgonly-exit", SW_RESET_CHGONLY_EXIT}, /* Exit Charge Only Mode */
	{"forced-crash", SW_RESET_FORCED},
	{"", SW_RESET_NORMAL},  /* Normal Boot */
	{"bootloader", SW_RESET_BOOTLOADER},
	{"eraseflash", SW_RESET_ERASEFLASH},
};

unsigned int reboot_reasons_sw_size = ARRAY_SIZE(reboot_reasons_sw);
static u16 preset_reboot_reason_code = SW_RESET_CRASH;

/*
 * The reboot reason string can be 255 characters long and the memory
 * in which we save the sw reset reason is 2 bytes. Therefore we need to
 * convert the string into a 16 bit pattern.
 *
 * See files reboot_reasons.h and dbx400-reset-reasons.h for conversion.
 */
u16 reboot_reason_code(const char *cmd)
{
	int i;

	if (cmd == NULL) {
		if (oops_in_progress) {
			/* if we're in an oops assume it's a crash */
			return SW_RESET_CRASH;
		} else {
			/* normal reboot w/o argument */
			return SW_RESET_NORMAL;
		}
	}

	/* Search through software reboot reason list */
	for (i = 0; i < reboot_reasons_sw_size; i++) {
		if (!strcmp(reboot_reasons_sw[i].reason, cmd))
			return reboot_reasons_sw[i].code;
	}

	/* Search through hardware reboot reason list */
	for (i = 0; i < reboot_reasons_hw_size; i++) {
		if (!strcmp(reboot_reasons_hw[i].reason, cmd))
			return reboot_reasons_hw[i].code;
	}

	/*
	 * No luck so far, let's try the OEMs
	 * If it starts with "oem-", let's assume oem-00..oem-ff
	 */
	if (!strncmp(cmd, "oem-", 4))
		return SW_RESET_OEM_00 + (simple_strtoul(cmd + 4, 0, 16)
					  & 0xff);

	/* No valid reboot reason found */
	printk(KERN_ERR "No valid Reboot Reason found for \"%s\"\n", cmd);
	return SW_RESET_NO_ARGUMENT;
}

/*
 * Preset a restart reason when this can't be done in a linear call-chain.
 * Typically this would be the case when a sysrq handler calls panic()
 * indirectly via the *killer = 1 mechanism to enforce the crash.
 */
void reboot_reason_preset(const char *cmd)
{
	preset_reboot_reason_code = reboot_reason_code(cmd);
}

/*
 * Read preset reason (if any).
 */
u16 reboot_reason_get_preset(void)

{
	return preset_reboot_reason_code;
}

/*
 * The saved sw reset reason is a 2 byte code that is translated into
 * a reboot reason string which is up to 255 characters long by this
 * function.
 *
 * See file reboot_reasons.h for conversion.
 */
const char *reboot_reason_string(u16 code)
{
	int i;

	/* Search through software reboot reason list */
	for (i = 0; i < reboot_reasons_sw_size; i++) {
		if (reboot_reasons_sw[i].code == code)
			return reboot_reasons_sw[i].reason;
	}

	/* Search through hardware reboot reason list */
	for (i = 0; i < reboot_reasons_hw_size; i++) {
		if (reboot_reasons_hw[i].code == code)
			return reboot_reasons_hw[i].reason;
	}

	/* No valid reboot reason code found */
	return "unknown";
}
