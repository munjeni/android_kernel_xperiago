/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Mian Yousaf Kaukab <mian.yousaf.kaukab@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */
#ifndef __ASM_ARCH_USB_H
#define __ASM_ARCH_USB_H

#include <linux/dmaengine.h>

#define UX500_MUSB_DMA_NUM_RX_CHANNELS 8
#define UX500_MUSB_DMA_NUM_TX_CHANNELS 8

struct musb;

struct ux500_musb_board_data {
	void	**dma_rx_param_array;
	void	**dma_tx_param_array;
	u32	num_rx_channels;
	u32	num_tx_channels;
	bool (*dma_filter)(struct dma_chan *chan, void *filter_param);
};

void ux500_add_usb(resource_size_t base, int irq, int *dma_rx_cfg,
	int *dma_tx_cfg);

/* Only used for u5500 */
struct abx500_usbgpio_platform_data {
	int (*get)(struct device *device);
	void (*enable)(void);
	void (*disable)(void);
	void (*put)(void);
	int usb_cs;
};
void ux500_restore_context(struct musb *musb);

/*
 * ULPI-specific Registers
 */
#define OTG_UVBCTRL	0x70	/* OTG ULPI VBUS Control Register*/
#define OTG_UCKIT	0x71	/* OTG ULPI CarKit Control Register*/
#define OTG_UINTMASK	0x72	/* OTG ULPI INT Mask Register*/
#define OTG_UINTSRC	0x73	/* OTG ULPI INT Source Register*/
#define OTG_UREGDATA	0x74	/* OTG ULPI Reg Data Register*/
#define OTG_UREGADDR	0x75	/* OTG ULPI Reg Address Register*/
#define OTG_UREGCTRL	0x76	/* OTG ULPI Reg Control Register*/
#define OTG_URAWDATA	0x77	/* OTG ULPI Raw Data Register*/

/*
 * OTG Top Control Register Bits
 */
#define OTG_TOPCTRL_MODE_ULPI	(1 << 0)/* Activate ULPI interface*/
#define OTG_TOPCTRL_UDDR	(1 << 1)/* Activate ULPI double-data rate mode */
#define OTG_TOPCTRL_SRST	(1 << 2)/* OTG core soft reset*/
#define OTG_TOPCTRL_XGATE	(1 << 3)/* Activate transceiver clock*/
#define OTG_TOPCTRL_I2C_OFF	(1 << 4)/* Switch off I2C controller*/
#define OTG_TOPCTRL_HDEV	(1 << 5)/* Select host mode with FS interface*/
#define OTG_TOPCTRL_VBUSLO	(1 << 6)/* Enable VBUS for FS transceivers*/

/*
 * OTG ULPI VBUS Control Register Bits
 */
#define OTG_UVBCTRL_EXTVB	(1 << 0)/* Use External VBUS*/
#define OTG_UVBCTRL_EXTVI	(1 << 1)/* Use External VBUS Indicator*/

/*
 * OTG ULPI Reg Control Register Bits
 */
#define OTG_UREGCTRL_REGREQ	(1 << 0)/* Request ULPI register access */
#define OTG_UREGCTRL_REGCMP	(1 << 1)/* ULPI register access completion flag */
#define OTG_UREGCTRL_URW	(1 << 2)/* Request read from register access */


#endif
