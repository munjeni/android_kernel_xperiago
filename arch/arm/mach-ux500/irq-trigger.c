/* kernel/arch/arm/mach-ux500/irq-trigger.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Krzysztof Antonowicz <krzysztof.antonowicz@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/errno.h>
#include <mach/hardware.h>
#include <asm/hardware/gic.h>

#define MIN_SPI_INTID	(32)
#define MAX_SPI_INTID	(255)

int irq_trigger_set_gic_spi_pending_interrupt(unsigned int int_id)
{

	if (int_id < MIN_SPI_INTID || int_id > MAX_SPI_INTID)
		return -EINVAL;

	writel_relaxed(1 << (int_id % 32),
		__io_address(U8500_GIC_DIST_BASE) +
		GIC_DIST_PENDING_SET + 4 * (int_id / 32));

	return 0;
}

