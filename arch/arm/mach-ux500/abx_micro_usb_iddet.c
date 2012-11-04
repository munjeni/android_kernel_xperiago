/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/mfd/abx500.h>
#include <linux/input/ab8505_micro_usb_iddet.h>
#include <plat/gpio-nomadik.h>
#include <mach/id.h>

static int u9540_uart_cable(struct usb_accessory_state *accessory,
		bool connected)
{
	int ret;
	static u8 read_back;
	struct device *dev = accessory->dev;

	if (connected) {
		ret = abx500_get(dev, AB8505_USB, USBPHYCTRL, &read_back);
		if (ret < 0) {
			dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
			return ret;
		}
	}

	/* GPIO266, connected ? U2_RXD_f : USB_DAT1 */
	ret = nmk_gpio_set_mode(266, connected ? NMK_GPIO_ALT_B :
			NMK_GPIO_ALT_A);
	if (ret < 0) {
		dev_err(dev, "%s Alternate function enable failed %d",
				__func__, __LINE__);
		return ret;
	}

	/* GPIO267, connected ? U2_TXD_e : USB_DAT0 */
	ret = nmk_gpio_set_mode(267, connected ? NMK_GPIO_ALT_B :
			NMK_GPIO_ALT_A);
	if (ret < 0) {
		dev_err(dev, "%s Alternate function enable failed %d",
				__func__, __LINE__);
		return ret;
	}

	/* force UART on ULPI whatever if ULPI command */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
			REGIDDETCTRL4, USBUARTULPIENA,
			connected ? USBUARTULPIENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	ret = abx500_set(dev, AB8505_USB, USBPHYCTRL,
			connected ? USBDEVICEMODEENA : read_back);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	return ret;
}

static int u8520_uart_cable(struct usb_accessory_state *accessory,
		bool connected)
{
	int ret;
	static u8 read_back;
	struct device *dev = accessory->dev;

	if (connected) {
		ret = abx500_get(dev, AB8505_USB, USBPHYCTRL, &read_back);
		if (ret < 0) {
			dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
			return ret;
		}

		ret = abx500_set(dev, AB8505_USB, USBPHYCTRL, 0x00);
		if (ret < 0) {
			dev_err(dev, "%s write failed %d\n", __func__,
					__LINE__);
			return ret;
		}
	}

	/* Select UARTTX data on pad GPIO13 */
	ret = abx500_mask_and_set(dev, AB8505_GPIO,
				ALTERNATFUNCTION, SETALTERUSBVDATULPIUARTTX,
				connected ? UARTTXDATA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Select UARTRxData function on pad Gpio50 */
	ret = abx500_mask_and_set(dev, AB8505_GPIO,
				ALTERNATFUNCTION, SETALTERULPIUARTRX,
				connected ? SETALTERULPIUARTRX : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/*
	 * force UART on ULPI whatever if ULPI command and Enable
	 * UART on GPIO pads.
	 */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
			REGIDDETCTRL4, USBUARTULPIENA | USBUARTNONULPIENA,
			connected ? USBUARTULPIENA | USBUARTNONULPIENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	ret = abx500_set(dev, AB8505_USB, USBPHYCTRL,
			connected ? USBDEVICEMODEENA : read_back);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Set alternate function Gpio13Sel */
	ret = abx500_mask_and_set(dev, AB8505_GPIO, GPIOSEL2,
			GPIO13SEL_ALT, connected ? 0x0 : GPIO13SEL_ALT);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Set alternate function on Gpio50Sel */
	ret = abx500_mask_and_set(dev, AB8505_GPIO, GPIOSEL7,
			GPIO50SEL_ALT, connected ? 0x0 : GPIO50SEL_ALT);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	return ret;
}

static int __init abx_micro_usb_iddet(void)
{
	if (cpu_is_u9540())
		iddet_adc_val_list.uart_cable = u9540_uart_cable;
	else
		iddet_adc_val_list.uart_cable = u8520_uart_cable;

	return 0;
}

module_init(abx_micro_usb_iddet);
