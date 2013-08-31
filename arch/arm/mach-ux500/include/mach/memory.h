/*
 * Copyright (C) 2009 ST-Ericsson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#define PLAT_PHYS_OFFSET	UL(0x00000000)
#define BUS_OFFSET	UL(0x00000000)

#if defined(CONFIG_MACH_U8500_LOTUS) || defined(CONFIG_MACH_U8500_PEPPER) || defined(CONFIG_MACH_U8500_KUMQUAT) || defined(CONFIG_MACH_U8500_NYPON)
#define END_MEM		            UL(0x1FDFFFFF)
#endif

#if defined(CONFIG_KEXEC_HARDBOOT)
#if defined(CONFIG_MACH_U8500_LOTUS) || defined(CONFIG_MACH_U8500_PEPPER) || defined(CONFIG_MACH_U8500_KUMQUAT) || defined(CONFIG_MACH_U8500_NYPON)
#define KEXEC_HB_PAGE_ADDR		UL(0x1FE00000)
#else
#error "Adress for kexec hardboot page not defined"
#endif
#endif

#ifdef CONFIG_UX500_PASR
#define PASR_SECTION_SZ_BITS	26 /* 64MB sections */
#define PASR_SECTION_SZ	(1 << PASR_SECTION_SZ_BITS)
#define PASR_MAX_DIE_NR		4
#define PASR_MAX_SECTION_NR_PER_DIE	8 /* 32 * 64MB = 2GB */
#endif

#ifdef CONFIG_UX500_SOC_DB8500
/*
 * STE NMF CM driver only used on the U8500 allocate using dma_alloc_coherent:
 *    8M for SIA and SVA data + 2M for SIA code + 2M for SVA code
 */
#define CONSISTENT_DMA_SIZE SZ_16M
#endif

#endif
