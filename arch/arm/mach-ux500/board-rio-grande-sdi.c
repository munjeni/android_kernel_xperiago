/*
 * Copyright (C) ST-Ericsson SA 2010
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Hanumath Prasad <hanumath.prasad@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <plat/ste_dma40.h>
#include <mach/devices.h>
#include <mach/hardware.h>
#include <mach/ste-dma40-db8500.h>

#include "devices-db8500.h"
#include "board-mop500.h"

/*
 * SDI1 (SDIO WLAN)
 */
#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg sdi1_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV2_SD_MMC1_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 0,
};

static struct stedma40_chan_cfg sdi1_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV2_SD_MMC1_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 0,
};
#endif

#define SDIO_WLAN_ENABLE	20

static void sdi1_configure(void)
{
	gpio_request(SDIO_WLAN_ENABLE, "wlan");
	gpio_direction_output(SDIO_WLAN_ENABLE, 0);
}

static int mop500_sdi1_ios_handler(struct device *dev, struct mmc_ios *ios, enum rpm_status pm)
{
	static int power_mode = -1;

	dev_dbg(dev, "%s: mode %d\n", __func__, ios->power_mode);

	if (power_mode == ios->power_mode)
		return 0;

	switch (ios->power_mode) {
	case MMC_POWER_ON:
		dev_dbg(dev, "powering up\n");
		gpio_direction_output(SDIO_WLAN_ENABLE, 1);
		/* It is not stated in the datasheet, but at least some devices
		 * have problems with reset if this stage is omited. */
		msleep(50);
		gpio_direction_output(SDIO_WLAN_ENABLE, 0);
		/* A valid reset shall be obtained by maintaining WRESETN
		 * active (low) for at least two cycles of LP_CLK after VDDIO
		 * is stable within it operating range. */
		mdelay(2);
		gpio_set_value(SDIO_WLAN_ENABLE, 1);
		/* The host should wait 32 ms after the WRESETN release
		 * for the on-chip LDO to stabilize */
		msleep(50);
	case MMC_POWER_UP:
		break;
	case MMC_POWER_OFF:
		dev_dbg(dev, "powering off\n");
		gpio_set_value(SDIO_WLAN_ENABLE, 0);
		break;
	default:
		dev_warn(dev, "Unknown power_mode: %d\n", ios->power_mode);
	}

	power_mode = ios->power_mode;
	return 0;
}

static struct mmci_platform_data mop500_sdi1_data = {
	.ios_handler	= mop500_sdi1_ios_handler,
	.ocr_mask	= MMC_VDD_29_30,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD |
				MMC_CAP_NONREMOVABLE,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &sdi1_dma_cfg_rx,
	.dma_tx_param	= &sdi1_dma_cfg_tx,
#endif
};


/*
 * SDI 2 (POP eMMC, not on DB8500ed)
 */
#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg mop500_sdi2_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV28_SD_MM2_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg mop500_sdi2_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV28_SD_MM2_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data mop500_sdi2_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_8_BIT_DATA,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &mop500_sdi2_dma_cfg_rx,
	.dma_tx_param	= &mop500_sdi2_dma_cfg_tx,
#endif
};

#ifdef CONFIG_U8500_SD_MMC_CARD_SUPPORT

#define CARD_INS_DETECT_N		84
#define MMC_PIN_CARD_PWR_ENABLE		85
#define MMC_PIN_LEVELSHIFTER_ENABLE	139
#define MMC_PIN_SDMMC_1V8_3V_SEL	217

/*
 * SDI 3 (SD/MMC card)
 */
static void sdi3_configure(void)
{
	int ret;

	ret = gpio_request(MMC_PIN_LEVELSHIFTER_ENABLE, "levelshifter_en");
	if (ret)
		goto err_out;

	ret = gpio_request(MMC_PIN_SDMMC_1V8_3V_SEL, "1v8_3v_sel");
	if (ret)
		goto err_out;

#ifdef CONFIG_U8500_SD_MMC_CARD_GPIO_PWR
	ret = gpio_request(MMC_PIN_CARD_PWR_ENABLE, "pwr_enable");
#endif
	if (ret)
		goto err_out;


	/* Set the SDMMC_1V8_3V_SEL signal to high to
	 * choose 3V card by default
	 */
	gpio_set_value(MMC_PIN_SDMMC_1V8_3V_SEL, 0);

	/* Card power and level shifter is powered off by default */
#ifdef CONFIG_U8500_SD_MMC_CARD_GPIO_PWR
	gpio_set_value(MMC_PIN_CARD_PWR_ENABLE, 0);
#endif
	gpio_set_value(MMC_PIN_LEVELSHIFTER_ENABLE, 0);

    return;

err_out:
	printk(KERN_WARNING "unable to config gpios for levelshift.\n");
}
static int mop500_sdi3_ios_handler(struct device *dev, struct mmc_ios *ios,
				enum rpm_status pm)
{
	static int power_mode = -1;

	if (power_mode == ios->power_mode)
		goto do_pm;

	switch (ios->power_mode) {
	case MMC_POWER_UP:
	case MMC_POWER_ON:
#ifdef CONFIG_U8500_SD_MMC_CARD_GPIO_PWR
		gpio_set_value(MMC_PIN_CARD_PWR_ENABLE, 1);
#endif
		gpio_set_value(MMC_PIN_LEVELSHIFTER_ENABLE, 1);
		udelay(100);
		break;
	case MMC_POWER_OFF:
#ifdef CONFIG_U8500_SD_MMC_CARD_GPIO_PWR
		gpio_set_value(MMC_PIN_CARD_PWR_ENABLE, 0);
#endif
		gpio_set_value(MMC_PIN_LEVELSHIFTER_ENABLE, 0);
		break;
	default:
		printk(KERN_WARNING "%s: Unknown power_mode: %d\n",
		       __func__, power_mode);
	}

	power_mode = ios->power_mode;

do_pm:
	if ((pm == RPM_SUSPENDING) && (power_mode == MMC_POWER_ON)) {
		/* Disable levelshifter to save power */
#ifdef CONFIG_U8500_SD_MMC_CARD_GPIO_PWR
		gpio_set_value(MMC_PIN_CARD_PWR_ENABLE, 0);
#endif
		gpio_set_value(MMC_PIN_LEVELSHIFTER_ENABLE, 0);
	} else if ((pm == RPM_RESUMING) && (power_mode == MMC_POWER_ON)) {
		/* Re-enable levelshifter. */
#ifdef CONFIG_U8500_SD_MMC_CARD_GPIO_PWR
		gpio_set_value(MMC_PIN_CARD_PWR_ENABLE, 1);
#endif
		gpio_set_value(MMC_PIN_LEVELSHIFTER_ENABLE, 1);
		/* Max settling time according to ST6G3244ME spec is 100 us. */
		udelay(100);
	}

	return 0;
}

#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg mop500_sdi3_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV41_SD_MM3_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 4,
};

static struct stedma40_chan_cfg mop500_sdi3_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV41_SD_MM3_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 4,
};
#endif

static struct mmci_platform_data mop500_sdi3_data = {
	.ios_handler	= mop500_sdi3_ios_handler,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_MMC_HIGHSPEED,
	.gpio_cd	= CARD_INS_DETECT_N,
	.gpio_wp	= -1,
	.levelshifter	= true,
	.sigdir		= MCI_ST_FBCLKEN |
				MCI_ST_CMDDIREN |
				MCI_ST_DATA0DIREN |
				MCI_ST_DATA2DIREN,
	.cd_invert	= 1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &mop500_sdi3_dma_cfg_rx,
	.dma_tx_param	= &mop500_sdi3_dma_cfg_tx,
#endif
};

#endif

/*
 * SDI 4 (on-board eMMC)
 */

#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg mop500_sdi4_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV42_SD_MM4_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 5,
};

static struct stedma40_chan_cfg mop500_sdi4_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV42_SD_MM4_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 5,
};
#endif

static struct mmci_platform_data mop500_sdi4_data = {
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_8_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &mop500_sdi4_dma_cfg_rx,
	.dma_tx_param	= &mop500_sdi4_dma_cfg_tx,
#endif
};

void __init mop500_sdi_init(void)
{
	u32 periphid = 0x10480180;

	/* POPed eMMC */
	db8500_add_sdi2(&mop500_sdi2_data, periphid);

	/* On-board eMMC */
	db8500_add_sdi4(&mop500_sdi4_data, periphid);

	/* WLAN */
	sdi1_configure();
	db8500_add_sdi1(&mop500_sdi1_data, periphid);

#ifdef CONFIG_U8500_SD_MMC_CARD_SUPPORT
	/* SD/MMC card */
	sdi3_configure();
	db8500_add_sdi3(&mop500_sdi3_data, periphid);
#endif
}
