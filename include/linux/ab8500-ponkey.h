/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Author: Michael Ambrus <michael.ambrus@sonyericsson.com>
 *
 * AB8500 Power-On Key handler
 */

#ifndef __AB8500_PONKEY_H__
#define __AB8500_PONKEY_H__

#include <linux/platform_device.h>
enum ab8500_forced_key {
        AB8500_PON_RELEASED,
        AB8500_PON_PRESSED,
	AB8500_EXTERNAL_EVENT,
};

#ifdef CONFIG_INPUT_AB8500_FORCECRASH

extern int ab8500_forcecrash_init(struct platform_device *pdev);
extern void ab8500_forcecrash_exit(struct platform_device *pdev);
extern void ab8500_forced_key_detect(enum ab8500_forced_key);
extern void ab8500_forced_key_combo_detect(void);
#else
static inline int ab8500_forcecrash_init(struct platform_device *pdev) { return 0; }
static inline void ab8500_forcecrash_exit(struct platform_device *pdev) {}
static inline void ab8500_forced_key_detect(int dummy) {}
static inline void ab8500_forced_key_combo_detect(void) {}
#endif

#endif /*__AB8500_PONKEY_H__*/
