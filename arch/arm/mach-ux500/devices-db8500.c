/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <plat/pincfg.h>
#include <plat/gpio-nomadik.h>

#include <plat/ste_dma40.h>

#include <mach/devices.h>
#include <mach/hardware.h>
#include <mach/setup.h>
#include <mach/pm.h>
#include <video/mcde.h>
#include <video/nova_dsilink.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <mach/hsi.h>
#include <mach/ste-dma40-db8500.h>
#include <video/b2r2_blt.h>

#include "pins-db8500.h"

#define GPIO_DATA(_name, first, num)					\
	{								\
		.name		= _name,				\
		.first_gpio	= first,				\
		.first_irq	= NOMADIK_GPIO_TO_IRQ(first),		\
		.num_gpio	= num,					\
		.get_secondary_status = ux500_pm_gpio_read_wake_up_status, \
		.set_ioforce	= ux500_pm_prcmu_set_ioforce,		\
		.supports_sleepmode = true,				\
	}

#define GPIO_RESOURCE(block)						\
	{								\
		.start	= U8500_GPIOBANK##block##_BASE,			\
		.end	= U8500_GPIOBANK##block##_BASE + 127,		\
		.flags	= IORESOURCE_MEM,				\
	},								\
	{								\
		.start	= IRQ_DB8500_GPIO##block,			\
		.end	= IRQ_DB8500_GPIO##block,			\
		.flags	= IORESOURCE_IRQ,				\
	},								\
	{								\
		.start	= IRQ_PRCMU_GPIO##block,			\
		.end	= IRQ_PRCMU_GPIO##block,			\
		.flags	= IORESOURCE_IRQ,				\
	}

#define GPIO_DEVICE(block)						\
	{								\
		.name		= "gpio",				\
		.id		= block,				\
		.num_resources	= 3,					\
		.resource	= &u8500_gpio_resources[block * 3],	\
		.dev = {						\
			.platform_data = &u8500_gpio_data[block],	\
		},							\
	}

static struct nmk_gpio_platform_data u8500_gpio_data[] = {
	GPIO_DATA("GPIO-0-31", 0, 32),
	GPIO_DATA("GPIO-32-63", 32, 5), /* 37..63 not routed to pin */
	GPIO_DATA("GPIO-64-95", 64, 32),
	GPIO_DATA("GPIO-96-127", 96, 2), /* 98..127 not routed to pin */
	GPIO_DATA("GPIO-128-159", 128, 32),
	GPIO_DATA("GPIO-160-191", 160, 12), /* 172..191 not routed to pin */
	GPIO_DATA("GPIO-192-223", 192, 32),
	GPIO_DATA("GPIO-224-255", 224, 7), /* 231..255 not routed to pin */
	GPIO_DATA("GPIO-256-288", 256, 12), /* 268..288 not routed to pin */
};

static struct resource u8500_gpio_resources[] = {
	GPIO_RESOURCE(0),
	GPIO_RESOURCE(1),
	GPIO_RESOURCE(2),
	GPIO_RESOURCE(3),
	GPIO_RESOURCE(4),
	GPIO_RESOURCE(5),
	GPIO_RESOURCE(6),
	GPIO_RESOURCE(7),
	GPIO_RESOURCE(8),
};

struct platform_device u8500_gpio_devs[] = {
	GPIO_DEVICE(0),
	GPIO_DEVICE(1),
	GPIO_DEVICE(2),
	GPIO_DEVICE(3),
	GPIO_DEVICE(4),
	GPIO_DEVICE(5),
	GPIO_DEVICE(6),
	GPIO_DEVICE(7),
	GPIO_DEVICE(8),
};

static struct resource u8500_shrm_resources[] = {
	[0] = {
		.start = U8500_SHRM_GOP_INTERRUPT_BASE,
		.end = U8500_SHRM_GOP_INTERRUPT_BASE + ((4*4)-1),
		.name = "shrm_gop_register_base",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CA_WAKE_REQ_V1,
		.end = IRQ_CA_WAKE_REQ_V1,
		.name = "ca_irq_wake_req",
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_AC_READ_NOTIFICATION_0_V1,
		.end = IRQ_AC_READ_NOTIFICATION_0_V1,
		.name = "ac_read_notification_0_irq",
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_AC_READ_NOTIFICATION_1_V1,
		.end = IRQ_AC_READ_NOTIFICATION_1_V1,
		.name = "ac_read_notification_1_irq",
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_CA_MSG_PEND_NOTIFICATION_0_V1,
		.end = IRQ_CA_MSG_PEND_NOTIFICATION_0_V1,
		.name = "ca_msg_pending_notification_0_irq",
		.flags = IORESOURCE_IRQ,
	},
	[5] = {
		.start = IRQ_CA_MSG_PEND_NOTIFICATION_1_V1,
		.end = IRQ_CA_MSG_PEND_NOTIFICATION_1_V1,
		.name = "ca_msg_pending_notification_1_irq",
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device u8500_shrm_device = {
	.name = "u8500_shrm",
	.id = 0,
	.dev = {
		.init_name = "shrm_bus",
		.coherent_dma_mask = ~0,
	},

	.num_resources = ARRAY_SIZE(u8500_shrm_resources),
	.resource = u8500_shrm_resources
};

static struct resource u8500_dsilink_resources[] = {
	[0] = {
		.name  = DSI_IO_AREA,
		.start = U8500_DSI_LINK1_BASE,
		.end   = U8500_DSI_LINK1_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = DSI_IO_AREA,
		.start = U8500_DSI_LINK2_BASE,
		.end   = U8500_DSI_LINK2_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = DSI_IO_AREA,
		.start = U8500_DSI_LINK3_BASE,
		.end   = U8500_DSI_LINK3_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device u8500_dsilink_device[] = {
	[0] = {
		.name = "dsilink",
		.id = 0,
		.num_resources = 1,
		.resource = &u8500_dsilink_resources[0],
	},
	[1] = {
		.name = "dsilink",
		.id = 1,
		.num_resources = 1,
		.resource = &u8500_dsilink_resources[1],
	},
	[2] = {
		.name = "dsilink",
		.id = 2,
		.num_resources = 1,
		.resource = &u8500_dsilink_resources[2],
	},
};

static struct resource mcde_resources[] = {
	[0] = {
		.name  = MCDE_IO_AREA,
		.start = U8500_MCDE_BASE,
		.end   = U8500_MCDE_BASE + U8500_MCDE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = MCDE_IO_AREA,
		.start = U8500_DSI_LINK1_BASE,
		.end   = U8500_DSI_LINK1_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = MCDE_IO_AREA,
		.start = U8500_DSI_LINK2_BASE,
		.end   = U8500_DSI_LINK2_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.name  = MCDE_IO_AREA,
		.start = U8500_DSI_LINK3_BASE,
		.end   = U8500_DSI_LINK3_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		.name  = MCDE_IRQ,
		.start = IRQ_DB8500_DISP,
		.end   = IRQ_DB8500_DISP,
		.flags = IORESOURCE_IRQ,
	},
};

static int mcde_platform_enable_dsipll(void)
{
	return prcmu_enable_dsipll();
}

static int mcde_platform_disable_dsipll(void)
{
	return prcmu_disable_dsipll();
}

static int mcde_platform_set_display_clocks(void)
{
	return prcmu_set_display_clocks();
}

static struct mcde_platform_data mcde_pdata = {
	/*
	 * [0] = 3: 24 bits DPI: connect LSB Ch B to D[0:7]
	 * [3] = 4: 24 bits DPI: connect MID Ch B to D[24:31]
	 * [4] = 5: 24 bits DPI: connect MSB Ch B to D[32:39]
	 *
	 * [1] = 3: TV out     : connect LSB Ch B to D[8:15]
	 */
#define DONT_CARE 0
	.outmux = { 3, 3, DONT_CARE, 4, 5 },
#undef DONT_CARE
	.syncmux = 0x00,  /* DPI channel A and B on output pins A and B resp */
#ifdef CONFIG_MCDE_DISPLAY_DSI
	.regulator_vana_id = "vdddsi1v2",
#endif
	.regulator_mcde_epod_id = "vsupply",
	.regulator_esram_epod_id = "v-esram34",
#ifdef CONFIG_MCDE_DISPLAY_DSI
	.clock_dsi_id = "hdmi",
	.clock_dsi_lp_id = "tv",
#endif
	.clock_dpi_id = "lcd",
	.clock_mcde_id = "mcde",
	.platform_set_clocks = mcde_platform_set_display_clocks,
	.platform_enable_dsipll = mcde_platform_enable_dsipll,
	.platform_disable_dsipll = mcde_platform_disable_dsipll,
	/* TODO: Remove rotation buffers once ESRAM driver is completed */
	.rotbuf1 = U8500_ESRAM_BASE + 0x20000 * 4 + 0x2000,
	.rotbuf2 = U8500_ESRAM_BASE + 0x20000 * 4 + 0x11000,
	.rotbufsize = 0xF000,
	.pixelfetchwtrmrk = {MCDE_PIXFETCH_WTRMRKLVL_OVL0,
				MCDE_PIXFETCH_WTRMRKLVL_OVL1,
				MCDE_PIXFETCH_WTRMRKLVL_OVL2,
				MCDE_PIXFETCH_WTRMRKLVL_OVL3,
				MCDE_PIXFETCH_WTRMRKLVL_OVL4,
				MCDE_PIXFETCH_WTRMRKLVL_OVL5},
};

struct platform_device ux500_mcde_device = {
	.name = "mcde",
	.id = -1,
	.dev = {
		.platform_data = &mcde_pdata,
	},
	.num_resources = ARRAY_SIZE(mcde_resources),
	.resource = mcde_resources,
};

struct platform_device ux500_b2r2_blt_device = {
	.name	= "b2r2_blt",
	.id	= 0,
	.dev	= {
		.init_name = "b2r2_blt_init",
		.coherent_dma_mask = ~0,
	},
};

static struct b2r2_platform_data b2r2_platform_data = {
	.regulator_id = "vsupply",
	.clock_id = "b2r2",
};

static struct resource b2r2_resources[] = {
	[0] = {
		.start	= U8500_B2R2_BASE,
		.end	= U8500_B2R2_BASE + ((4*1024)-1),
		.name	= "b2r2_base",
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name  = "B2R2_IRQ",
		.start = IRQ_DB8500_B2R2,
		.end   = IRQ_DB8500_B2R2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ux500_b2r2_device = {
	.name	= "b2r2",
	.id	= 0,
	.dev	= {
		.init_name = "b2r2_core",
		.platform_data = &b2r2_platform_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources	= ARRAY_SIZE(b2r2_resources),
	.resource	= b2r2_resources,
};


/*
 * HSI
 */
#define HSI0_CAWAKE { \
	.start = IRQ_PRCMU_HSI0, \
	.end   = IRQ_PRCMU_HSI0, \
	.flags = IORESOURCE_IRQ, \
	.name = "hsi0_cawake" \
}

#define HSI0_ACWAKE { \
	.start = 226, \
	.end   = 226, \
	.flags = IORESOURCE_IO, \
	.name = "hsi0_acwake" \
}

#define HSIR_OVERRUN(num) {			    \
	.start  = IRQ_DB8500_HSIR_CH##num##_OVRRUN, \
	.end    = IRQ_DB8500_HSIR_CH##num##_OVRRUN, \
	.flags  = IORESOURCE_IRQ,		    \
	.name   = "hsi_rx_overrun_ch"#num	    \
}

#define STE_HSI_PORT0_TX_CHANNEL_CFG(n) { \
	.dir = STEDMA40_MEM_TO_PERIPH,	\
	.high_priority = true,	\
	.mode = STEDMA40_MODE_LOGICAL, \
	.mode_opt = STEDMA40_LCHAN_SRC_LOG_DST_LOG, \
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY, \
	.dst_dev_type = n,\
	.src_info.big_endian = false,\
	.src_info.data_width = STEDMA40_WORD_WIDTH,\
	.dst_info.big_endian = false,\
	.dst_info.data_width = STEDMA40_WORD_WIDTH,\
},

#define STE_HSI_PORT0_RX_CHANNEL_CFG(n) { \
	.dir = STEDMA40_PERIPH_TO_MEM, \
	.high_priority = true, \
	.mode = STEDMA40_MODE_LOGICAL, \
	.mode_opt = STEDMA40_LCHAN_SRC_LOG_DST_LOG, \
	.src_dev_type = n,\
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY, \
	.src_info.big_endian = false,\
	.src_info.data_width = STEDMA40_WORD_WIDTH,\
	.dst_info.big_endian = false,\
	.dst_info.data_width = STEDMA40_WORD_WIDTH,\
},

static struct resource u8500_hsi_resources[] = {
       {
		.start  = U8500_HSIR_BASE,
		.end    = U8500_HSIR_BASE + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
		.name   = "hsi_rx_base"
       },
       {
		.start  = U8500_HSIT_BASE,
		.end    = U8500_HSIT_BASE + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
		.name   = "hsi_tx_base"
       },
       {
		.start  = IRQ_DB8500_HSIRD0,
		.end    = IRQ_DB8500_HSIRD0,
		.flags  = IORESOURCE_IRQ,
		.name   = "hsi_rx_irq0"
       },
       {
		.start  = IRQ_DB8500_HSITD0,
		.end    = IRQ_DB8500_HSITD0,
		.flags  = IORESOURCE_IRQ,
		.name   = "hsi_tx_irq0"
       },
       {
		.start  = IRQ_DB8500_HSIR_EXCEP,
		.end    = IRQ_DB8500_HSIR_EXCEP,
		.flags  = IORESOURCE_IRQ,
		.name   = "hsi_rx_excep0"
       },
       HSIR_OVERRUN(0),
       HSIR_OVERRUN(1),
       HSIR_OVERRUN(2),
       HSIR_OVERRUN(3),
       HSIR_OVERRUN(4),
       HSIR_OVERRUN(5),
       HSIR_OVERRUN(6),
       HSIR_OVERRUN(7),
       HSI0_CAWAKE,
       HSI0_ACWAKE,
};

#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg ste_hsi_port0_dma_tx_cfg[] = {
       STE_HSI_PORT0_TX_CHANNEL_CFG(DB8500_DMA_DEV20_SLIM0_CH0_TX_HSI_TX_CH0)
       STE_HSI_PORT0_TX_CHANNEL_CFG(DB8500_DMA_DEV21_SLIM0_CH1_TX_HSI_TX_CH1)
       STE_HSI_PORT0_TX_CHANNEL_CFG(DB8500_DMA_DEV22_SLIM0_CH2_TX_HSI_TX_CH2)
       STE_HSI_PORT0_TX_CHANNEL_CFG(DB8500_DMA_DEV23_SLIM0_CH3_TX_HSI_TX_CH3)
};

static struct stedma40_chan_cfg ste_hsi_port0_dma_rx_cfg[] = {
       STE_HSI_PORT0_RX_CHANNEL_CFG(DB8500_DMA_DEV20_SLIM0_CH0_RX_HSI_RX_CH0)
       STE_HSI_PORT0_RX_CHANNEL_CFG(DB8500_DMA_DEV21_SLIM0_CH1_RX_HSI_RX_CH1)
       STE_HSI_PORT0_RX_CHANNEL_CFG(DB8500_DMA_DEV22_SLIM0_CH2_RX_HSI_RX_CH2)
       STE_HSI_PORT0_RX_CHANNEL_CFG(DB8500_DMA_DEV23_SLIM0_CH3_RX_HSI_RX_CH3)
};
#endif

static struct ste_hsi_port_cfg ste_hsi_port0_cfg = {
#ifdef CONFIG_STE_DMA40
	.dma_filter = stedma40_filter,
	.dma_tx_cfg = ste_hsi_port0_dma_tx_cfg,
	.dma_rx_cfg = ste_hsi_port0_dma_rx_cfg
#endif
};

struct ste_hsi_platform_data u8500_hsi_platform_data = {
	.num_ports = 1,
	.use_dma = 1,
	.port_cfg = &ste_hsi_port0_cfg,
};

struct platform_device u8500_hsi_device = {
	.dev = {
		.platform_data = &u8500_hsi_platform_data,
	},
	.name = "ste_hsi",
	.id = 0,
	.resource = u8500_hsi_resources,
	.num_resources = ARRAY_SIZE(u8500_hsi_resources)
};

/*
 * Thermal Sensor
 */



struct resource keypad_resources[] = {
	[0] = {
		.start = U8500_SKE_BASE,
		.end = U8500_SKE_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_DB8500_KB,
		.end = IRQ_DB8500_KB,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device u8500_ske_keypad_device = {
	.name = "nmk-ske-keypad",
	.id = -1,
	.num_resources = ARRAY_SIZE(keypad_resources),
	.resource = keypad_resources,
};
