/* kernel/arch/arm/mach-ux500/include/mach/irq-trigger.h
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

#ifndef IRQ_TRIGGER_H
#define IRQ_TRIGGER_H

/**
 * irq_trigger_set_gic_spi_pending_interrupt(unsigned int int_id)
 *
 * @int_id: INTID for SPI (Shared Peripheral Interrupt).
 *          Minimum INTID for SPI is 32 and maximum 255.
 *          The Interrupt Distributor ignores writes to unused INTIDs.
 *
 * Set an interrupt to the pending or active-and-pending state.
 * Returns 0 if write succeeded.
 */
int irq_trigger_set_gic_spi_pending_interrupt(unsigned int int_id);

#endif
