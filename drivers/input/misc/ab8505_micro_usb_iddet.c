/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * License terms: GNU General Public License (GPL) version 2
 * Author: Etienne CARRIERE <etienne.carriere@stericsson.com>
 * Author: Virupax Sadashivpetimath <virupax.sadashivpetimath@stericsson.com>
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mfd/abx500.h>
#include <linux/hrtimer.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/abx500/ab8500-gpadc.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/input/ab8505_micro_usb_iddet.h>
#include <sound/jack.h>

#define KEY_DEBOUCE_TIME_12MS		0x00
#define KEY_DEBOUCE_TIME_25MS		0x01
#define KEY_DEBOUCE_TIME_37MS		0x02
#define KEY_DEBOUCE_TIME_50MS		0x03
#define KEY_DEBOUCE_TIME_62MS		0x04
#define KEY_DEBOUCE_TIME_75MS		0x05
#define KEY_DEBOUCE_TIME_87MS		0x06
#define KEY_DEBOUCE_TIME_100MS		0x07
#define KEY_DEBOUCE_TIME_0MS		0x08

#define KEY_PRESS_TIME_100MS		0x00
#define KEY_PRESS_TIME_200MS		0x01
#define KEY_PRESS_TIME_300MS		0x02
#define KEY_PRESS_TIME_400MS		0x03
#define KEY_PRESS_TIME_500MS		0x04
#define KEY_PRESS_TIME_600MS		0x05
#define KEY_PRESS_TIME_700MS		0x06
#define KEY_PRESS_TIME_800MS		0x07
#define KEY_PRESS_TIME_900MS		0x08
#define KEY_PRESS_TIME_1000MS		0x09
#define KEY_PRESS_TIME_1100MS		0x0a
#define KEY_PRESS_TIME_1200MS		0x0b
#define KEY_PRESS_TIME_1300MS		0x0c
#define KEY_PRESS_TIME_1400MS		0x0d
#define KEY_PRESS_TIME_1500MS		0x0e

#define IDHOSTENA			0x02
#define IDDEVENA			0x01
#define IDDETSWCTRLENA			0x80
#define IDDETPU1ENA			0x20
#define PLUGDETCOMPENA			0x08
#define IDDETPLUGDETCOMP		0x08
#define IDDETPU200K18VENA		0x40
#define IDDETADCENA			0x01
#define ITSOURCE2			0x01
#define VUSBENA				0x01
#define VBUSDET				0x80
#define VBUSVALIDENA			0x20
#define USBSWCTRL			0x04
#define USBCPMICSUBCLKENA		0x08
#define USBHOSTMODEENA			0x01
#define REGIDDETKEYDEGLITCH		0xAF
#define REGIDDETKEYTIMER1		0xB0
#define IDDETKEYDEGLITCH		0x0F
#define IDDETKEYPRESSTIME		0xF0
#define IDDETKEYLONGTIME		0x0F
#define ENUHSR				0x01
#define ENUHSL				0x02
#define USBMICSEL			0x30
#define ENCKLOL				0x80
#define ENCKLOR				0x40
#define ENCKLOLDM			0x20
#define ENCKLOLDP			0x10
#define ROUTE_CARKIT_HEADSET		(ENCKLOL | ENCKLOR | ENCKLOLDM |\
		ENCKLOLDP)
#define VUSBCTRL			0x82
#define USBOTGCTRL			0x87
#define REGIDDETCTRL1			0xA0
#define REGIDDETCTRL2			0xA1
#define REGIDDETCTRL3			0xA2
#define REGIDDETSTATE			0xA7
#define REGIDDETKEYLEVEL		0xA4
#define MISSKEYPRESS			0xA5
#define REGIDDETKEYTIMER2		0xB1
#define REGIDDETKEYTIMER3		0xB2
#define USBLINK1STATUS			0x94
#define USBLINECTRL1			0x81
#define USBDRVCTRL			0x74
#define USBGAINMICSEL			0x75
#define DAPATHCONF			0x09
#define ENDACHSL			0x20
#define ENDACHSR			0x10
#define ENABLE				0x01
#define DISABLE				0x00
#define AB8505_SUPPLY_CONTROL		0x03
#define LINK1_STATUS_MASK		0xF8
#define REGIDDETVTH			0xA6
#define IDDETFMDETCOMPHIENA		0x01
#define IDDETFMDETLOENA			0x02
#define IDDETFMDETREFPUENA		0x04
#define IDDETPU200KVISENA		0x80
#define FMCOMPENA			(IDDETFMDETCOMPHIENA | IDDETFMDETLOENA\
		| IDDETFMDETREFPUENA)
#define IDDETVTHDET619KENA		0x40
#define IDDETVTHDET523KENA		0x20
#define IDDETVTHDET440KENA		0x10
#define IDDETVTHDET301KENA		0x08
#define IDDETVTHDET255KENA		0x04
#define IDDETVTHDET200KENA		0x02
#define IDDETFMDETCOMPHI		0x40
#define IDDETFMDETCOMPLO		0x20
#define IDDETFMCOMPSTAT			(IDDETFMDETCOMPHI | IDDETFMDETCOMPLO)
#define SERVICESWCTRLENA		0x80
#define SERVICEFORCEHZENA		0x40
#define USBHSGAIN			0x73
#define VUSBFLP				0x02

#define IDDETKEYCOMPPENA		0x01
#define KEYPADSINGLECOMPMODE		0x02
#define IDDETKEYRDREQ			0x08
#define REGIDDETKEYTIMER3_MASK		0x03
#define IDDETPU10K1V8ENA		0x08
#define IDDETPU10KVADCENA		0x10
#define IDDETPU1UENA			0x20
#define IDIDETPLUGDETCOMPENA		0x08
#define IDDETKEYCOMPNENA		0x02
#define IDDETKEYNLEVEL			0x0F
#define IDDETKEYPLEVEL			0xF0
#define PLEVEL_500MV			0x30
#define NLEVEL_200MV			0x01
#define KEYPADENAFLANK			0x01
#define IDDETKEYNEGPOSON		0x04

#define DETECTION_INTERVAL_MS		500
#define BTN_DETECTION_RESET_INTERVAL_MS	800

struct key_interrupt_types {
	char *name;
	irqreturn_t (*function) (int, void*);
	int irq;
};

static int (*acessory_func_list[])(struct usb_accessory_state *, bool);
static int init_button_press_detection(struct device *dev, int connected);

static struct key_interrupt_types key_press_interrupts[];
struct workqueue_struct *btn_detection_work_queue;
static struct input_dev *usb_button;

static BLOCKING_NOTIFIER_HEAD(micro_usb_notifier_list);

static int key_glitch_time =  KEY_DEBOUCE_TIME_50MS;
static int key_press_time = KEY_PRESS_TIME_500MS;
static int key_log_press_time = KEY_PRESS_TIME_1000MS;

int micro_usb_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&micro_usb_notifier_list, nb);
}
EXPORT_SYMBOL(micro_usb_register_notifier);

int micro_usb_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&micro_usb_notifier_list, nb);
}
EXPORT_SYMBOL(micro_usb_unregister_notifier);

void get_iddet_adc_val(struct kthread_work *work)
{
	int id_voltage;
	struct usb_accessory_state *accessory
		= container_of(work, struct usb_accessory_state,
				read_adc_work);
	struct device *dev = accessory->dev;
	struct button_param_list *tmp = accessory->btn_param_list;

	id_voltage = ab8500_gpadc_convert(accessory->gpadc, USB_ID);
	if (id_voltage < 0) {
		dev_err(dev, "GPADC read failed %d\n", id_voltage);
		return;
	}

	dev_dbg(dev, "GPADC USB_ID read %d\n", id_voltage);

	while (tmp->vmax != 0) {
		if ((id_voltage <= tmp->vmax) && (id_voltage >= tmp->vmin)) {
			dev_info(dev, "Button ID %d name %s\n", tmp->btn_id,
					tmp->btn_name);
			break;
		}
		tmp++;
	}
}

static irqreturn_t key_glitch_interrupt_handler(int irq, void *data)
{
	int ret;
	struct usb_accessory_state *accessory = data;
	struct device *dev = accessory->dev;

	input_report_key(usb_button, KEY_MEDIA, 1);
	input_sync(usb_button);
	input_report_key(usb_button, KEY_MEDIA, 0);
	input_sync(usb_button);

	/* IdDetKeyCompPEna Ena */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, MISSKEYPRESS,
			IDDETKEYCOMPPENA, IDDETKEYCOMPPENA);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		goto handled;
	}

	/* KeypadSingleCompMode Dis */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, REGIDDETCTRL3,
			KEYPADSINGLECOMPMODE, 0x00);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		goto handled;
	}

	queue_kthread_work(&accessory->kworker, &accessory->read_adc_work);

handled:
	return IRQ_HANDLED;
}

static irqreturn_t key_press_interrupt_handler(int irq, void *data)
{
	int ret;
	struct usb_accessory_state *accessory = data;
	struct device *dev = accessory->dev;
	u8 tmp1, tmp2;
	u16 iddetkeytime;

	/* IdDetKeyRdReq Ena */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, MISSKEYPRESS,
			IDDETKEYRDREQ, IDDETKEYRDREQ);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		goto handled;
	}

	/* Read IdDetKeyTime[7:0] */
	ret = abx500_get(dev, AB8505_CHARGER, REGIDDETKEYTIMER2, &tmp1);
	if (ret < 0) {
		dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
		goto handled;
	}

	/* Read IdDetKeyTime[9:8] */
	ret = abx500_get(dev, AB8505_CHARGER, REGIDDETKEYTIMER3, &tmp2);
	if (ret < 0) {
		dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
		goto handled;
	}

	/* IdDetKeyRdReq Dis */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, MISSKEYPRESS,
			IDDETKEYRDREQ, 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		goto handled;
	}

	/* IdDetKeyTime[7:0] | IdDetKeyTime[9:8] */
	iddetkeytime = tmp1 | ((tmp2 & REGIDDETKEYTIMER3_MASK) << 8);

	dev_dbg(dev, "Key Press Time read (%d * 12.5)ms\n", iddetkeytime);
handled:
	return IRQ_HANDLED;
}

/*TODO: Task needs to be defined for the below 3 handlers */
static irqreturn_t long_key_press_interrupt_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t ikr_interrupt_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t key_stuck_interrupt_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static struct key_interrupt_types key_press_interrupts[] = {
	{"KeyDeglitch", &key_glitch_interrupt_handler},
	{"KP", &key_press_interrupt_handler},
	{"IKP", &long_key_press_interrupt_handler},
	{"IKR", &ikr_interrupt_handler},
	{"KeyStuck", &key_stuck_interrupt_handler},
};

static int key_press_interrupt(bool connected)
{
	int i;

	for (i = 0; i < sizeof(key_press_interrupts)/
			sizeof(key_press_interrupts[0]); i++) {
		if (connected)
			enable_irq(key_press_interrupts[i].irq);
		else
			disable_irq(key_press_interrupts[i].irq);
	}

	return 0;
};

static int uart_boot_off(struct usb_accessory_state *accessory, bool connected)
{
	struct device *dev = accessory->dev;

	dev_info(dev, "UART Boot-OFF %s\n",
			connected ? "PLUGGED" : "UNPLUGGED");
	acessory_func_list[USBSWITCH_UART](accessory, connected);
	return 0;
}

static int uart_boot_on(struct usb_accessory_state *accessory, bool connected)
{
	struct device *dev = accessory->dev;

	dev_info(dev, "UART Boot-ON %s\n", connected ? "PLUGGED" : "UNPLUGGED");
	acessory_func_list[USBSWITCH_UART](accessory, connected);
	return 0;
}

static int usb_boot_on(struct usb_accessory_state *accessory, bool connected)
{
	int ret;
	struct device *dev = accessory->dev;

	dev_info(dev, "USB Boot-ON %s\n", connected ? "PLUGGED" : "UNPLUGGED");

	/* Service controlled by i2c */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL4, SERVICESWCTRLENA,
				connected ? SERVICESWCTRLENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Service is forced Hig hZ */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL4, SERVICEFORCEHZENA,
				connected ? SERVICEFORCEHZENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	blocking_notifier_call_chain(&micro_usb_notifier_list,
			connected ? USB_BOOT_ON_PLUGGED :
			USB_BOOT_ON_UNPLUGGED, NULL);

	return ret;
}

static int usb_boot_off(struct usb_accessory_state *accessory, bool connected)
{
	int ret;
	struct device *dev = accessory->dev;

	dev_info(dev, "USB Boot-OFF %s\n", connected ? "PLUGGED" : "UNPLUGGED");

	/* Service controlled by i2c */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL4, SERVICESWCTRLENA,
				connected ? SERVICESWCTRLENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Service is forced Hig hZ */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL4, SERVICEFORCEHZENA,
				0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	blocking_notifier_call_chain(&micro_usb_notifier_list,
			connected ? USB_BOOT_OFF_PLUGGED :
			USB_BOOT_OFF_UNPLUGGED, NULL);

	return ret;
}

static int tty_converter(struct usb_accessory_state *accessory, bool connected)
{
	int ret;
	struct device *dev = accessory->dev;
	static unsigned char usbhsgain;
	static unsigned char eargainmicsel;

	dev_info(dev, "TTY Device %s\n",  connected ? "PLUGGED" : "UNPLUGGED");

	/* Vbus OVP external switch force opened */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, REGIDDETCTRL3, USBSWCTRL,
				connected ? USBSWCTRL : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	if (connected) {
		/* Take backup and Set Left,Right Usb Headset analog gain */
		ret = abx500_get(dev, AB8505_AUDIO, USBHSGAIN, &usbhsgain);
		if (ret < 0) {
			dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
			return ret;
		}

		ret = abx500_set(dev, AB8505_AUDIO, USBHSGAIN, 0x0);
		if (ret < 0) {
			dev_err(dev, "%s write failed %d\n", __func__,
					__LINE__);
			return ret;
		}
	} else {
		ret = abx500_set(dev, AB8505_AUDIO, USBHSGAIN, usbhsgain);
		if (ret < 0) {
			dev_err(dev, "%s write failed %d\n", __func__,
					__LINE__);
			return ret;
		}
	}

	/* Enable Left and Right USB Headset */
	ret = abx500_set(dev, AB8505_AUDIO, USBDRVCTRL,
			connected ? ENUHSR | ENUHSL : 0xC);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	if (connected) {
		ret = abx500_get(dev, AB8505_AUDIO, USBGAINMICSEL,
				&eargainmicsel);
		if (ret < 0) {
			dev_err(dev, "%s get failed %d\n", __func__, __LINE__);
			return ret;
		}

		/* DP ball is selected as usb microphone input */
		ret = abx500_mask_and_set(dev, AB8505_AUDIO,
				USBGAINMICSEL, USBMICSEL, 0x20);
		if (ret < 0) {
			dev_err(dev, "%s write failed %d\n", __func__,
					__LINE__);
			return ret;
		}
	} else {
		ret = abx500_set(dev, AB8505_AUDIO, USBGAINMICSEL,
				eargainmicsel);
		if (ret < 0) {
			dev_err(dev, "%s write failed %d\n", __func__,
					__LINE__);
			return ret;
		}
	}

	/* Enable Vusb */
	ret = abx500_mask_and_set(dev, AB8505_SUPPLY_CONTROL,
				VUSBCTRL, VUSBENA | VUSBFLP,
				connected ? VUSBENA | VUSBFLP : 0x0);
	if (ret < 0)
		dev_err(dev, "%s write failed %d\n", __func__,
				__LINE__);

	return ret;
}

static int init_button_press_detection(struct device *dev, int connected)
{
	int ret;

	/* IDDetPu10k1v8Ena Ena & IDDetPu10kVadcEna Dis & IDDetPu1uEna Ena */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, REGIDDETCTRL2,
			IDDETPU10K1V8ENA | IDDETPU10KVADCENA | IDDETPU1UENA,
			connected ? IDDETPU1UENA | IDDETPU10K1V8ENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* IDDetPlugDetCompEna Ena */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, REGIDDETCTRL1,
			IDIDETPLUGDETCOMPENA, connected ?
			IDIDETPLUGDETCOMPENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* IDDetKeyCompNEna Ena & IDDetKeyCompPEna Dis */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, MISSKEYPRESS,
			IDDETKEYCOMPNENA | IDDETKEYCOMPPENA, connected ?
			IDDETKEYCOMPNENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* IdDetKeyNlevel[3:0] &  IdDetKeyPlevel[7:4] */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, REGIDDETKEYLEVEL,
			IDDETKEYNLEVEL | IDDETKEYPLEVEL, connected ?
			PLEVEL_500MV | NLEVEL_200MV : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* IdDetKeyPressTime[7:4] */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETKEYTIMER1, IDDETKEYPRESSTIME,
				key_press_time << 4);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* IdDetKeyLongTime[3:0] */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETKEYTIMER1, IDDETKEYLONGTIME,
				key_log_press_time);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* set key glitch time */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETKEYDEGLITCH, IDDETKEYDEGLITCH,
				key_glitch_time);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* KeypadSingleCompMode Ena & KeypadEnaFlank Dis */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, REGIDDETCTRL3,
			KEYPADSINGLECOMPMODE | KEYPADENAFLANK,
			KEYPADSINGLECOMPMODE);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* IdDetKeyNegPosOn Dis */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, MISSKEYPRESS,
			IDDETKEYNEGPOSON, 0x0);
	if (ret < 0)
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);

	return ret;
}

static void button_detection_function(struct work_struct *work)
{
	int ret;
	u8 id_voltage;
	struct usb_accessory_state *accessory = container_of(work,
		struct usb_accessory_state, detect_button.work);
	struct device *dev = accessory->dev;

	/* IdDetKeyNegPosOn Dis */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, MISSKEYPRESS,
			IDDETKEYNEGPOSON, 0x00);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		goto btn_det_restart;
	}

	/* IDDetPu10k1v8Ena Dis Here */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, REGIDDETCTRL2,
			IDDETPU10K1V8ENA, 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		goto btn_det_restart;
	}

	/* read Iddet comparator state */
	ret = abx500_get(dev, AB8505_CHARGER,
				REGIDDETSTATE, &id_voltage);
	if (ret < 0) {
		dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
		goto btn_det_restart;
	}

	/* Check if the cable was UnPluged */
	if (!(id_voltage & IDDETPLUGDETCOMP)) {
		queue_delayed_work(accessory->iddet_workqueue,
				&accessory->cable_detection,
			msecs_to_jiffies(DETECTION_INTERVAL_MS));

		acessory_func_list[accessory->cable_last_detected]
			(accessory, false);

		accessory->cable_last_detected = USBSWITCH_NONE;
		return;
	}

	/*TODO:ForABcut2: IDDetPu10k1v8Ena Ena Here */

	/* IdDetKeyNegPosOn Ena */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, MISSKEYPRESS,
			IDDETKEYNEGPOSON, IDDETKEYNEGPOSON);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		goto btn_det_restart;
	}

btn_det_restart:
	queue_delayed_work(btn_detection_work_queue, &accessory->detect_button,
			msecs_to_jiffies(BTN_DETECTION_RESET_INTERVAL_MS));
}

static int audio_dev_type1(struct usb_accessory_state *accessory,
		bool connected)
{
	int ret;
	struct device *dev = accessory->dev;
	static unsigned char eargainmicsel;

	dev_info(dev, "Audio Device %s\n", connected ? "PLUGGED" : "UNPLUGGED");

	/* Vbus OVP external switch force opened */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
			REGIDDETCTRL3, USBSWCTRL | USBCPMICSUBCLKENA,
			connected ? USBSWCTRL | USBCPMICSUBCLKENA : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Enable Left and Right USB Headset */
	ret = abx500_set(dev, AB8505_AUDIO, USBDRVCTRL,
				connected ? ENUHSR | ENUHSL : 0xC);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	if (connected) {
		ret = abx500_get(dev, AB8505_AUDIO,
				USBGAINMICSEL, &eargainmicsel);
		if (ret < 0) {
			dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
			return ret;
		}

		/* DP ball is selected as usb microphone input */
		ret = abx500_mask_and_set(dev, AB8505_AUDIO,
				USBGAINMICSEL, USBMICSEL, 0x20);
		if (ret < 0) {
			dev_err(dev, "%s write failed %d\n", __func__,
					__LINE__);
			return ret;
		}
	} else {
		ret = abx500_set(dev, AB8505_AUDIO,
				USBGAINMICSEL, eargainmicsel);
		if (ret < 0) {
			dev_err(dev, "%s write failed %d\n", __func__,
					__LINE__);
			return ret;
		}
	}

	/* Enable VusbEna and VusbFlp */
	ret = abx500_mask_and_set(dev, AB8505_SUPPLY_CONTROL,
				VUSBCTRL, VUSBENA | VUSBFLP,
				connected ? VUSBENA | VUSBFLP : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__,
				__LINE__);
		return ret;
	}

	set_android_switch_state(connected);

	init_button_press_detection(dev, connected);

	key_press_interrupt(connected);

	/* IdDetKeyNegPosOn Ena */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER, MISSKEYPRESS,
			IDDETKEYNEGPOSON, connected ? IDDETKEYNEGPOSON : 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	if (connected)
		queue_delayed_work(btn_detection_work_queue,
				&accessory->detect_button,
				msecs_to_jiffies
				(BTN_DETECTION_RESET_INTERVAL_MS));
	return ret;
}

static int cable_unknown(struct usb_accessory_state *accessory, bool connected)
{
	struct device *dev = accessory->dev;
	dev_warn(dev, "Unknown Cable %s\n", connected ? "PLUGGED" :
			"UNPLUGGED");
	return 0;
}

static int legacy_charger(struct usb_accessory_state *accessory, bool connected)
{
	struct device *dev = accessory->dev;
	dev_info(dev, "Legacy charger %s\n", connected ? "PLUGGED" :
			"UNPLUGGED");
	blocking_notifier_call_chain(&micro_usb_notifier_list,
			connected ? LEGACY_CHARGER_PLUGGED :
			LEGACY_CHARGER_UNPLUGGED, NULL);
	return 0;
}

/* Callbacks for the type of cable connected */
static int (*acessory_func_list[])(struct usb_accessory_state *, bool) = {
	[USBSWITCH_UART_BOOT_ON] = uart_boot_on,
	[USBSWITCH_UART_BOOT_OFF] = uart_boot_off,
	[USBSWITCH_USB_BOOT_ON] = usb_boot_on,
	[USBSWITCH_USB_BOOT_OFF] = usb_boot_off,
	[USBSWITCH_AUDIODEV_TYPE1] = audio_dev_type1,
	[USBSWITCH_TTY_CONV] = tty_converter,
	[USBSWITCH_UART] = NULL,
	[USBSWITCH_LEGACY_CHARGER] = legacy_charger,
	[USBSWITCH_UNKNOWN] = cable_unknown,
};

/*
 * Detec the cable connected depending on the voltage across the
 * ID line.
 */
static int detect_depending_on_id_resistance(struct usb_accessory_state
		*accessory)
{
	int ret;
	struct cust_rid_adcid *p;
	int id_voltage;
	struct device *dev = accessory->dev;

	/* 1microAmp current source and 200k pull up enable  */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL2, IDDETPU200K18VENA,
				IDDETPU200K18VENA);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Connect ID line to GPADC input */
	ret = abx500_mask_and_set(dev, AB8505_USB,
				USBLINECTRL1, IDDETADCENA, IDDETADCENA);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	id_voltage = ab8500_gpadc_convert(accessory->gpadc, USB_ID);
	dev_dbg(dev, "USB-ID %d\n", id_voltage);
	if (id_voltage) {
		p = accessory->cables_param_list;
		while (p->max != 0) {
			if ((id_voltage >= p->min) && (id_voltage <= p->max)) {
				dev_dbg(dev, "Matched %d\n", p->cable_id);
				accessory->cable_detected = p->cable_id;
				goto detected;
			}
			p++;
		}
	}

	accessory->cable_detected = USBSWITCH_UNKNOWN;

detected:
	/* 1microAmp current source and 200k pull up disable */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL2, IDDETPU200K18VENA, 0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Disconnect ID line to GPADC input */
	ret = abx500_mask_and_set(dev, AB8505_USB,
				USBLINECTRL1, IDDETADCENA, 0);
	if (ret < 0)
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);

	return ret;
}

/*
 * set the registers for scaning the USB device connected
 * here HW detection of the device connected is disabled.
 */
static int usb_switch_init(struct device *dev)
{
	int ret;

	/* Disable host, device detection and enable VBUS Valid comporator */
	ret = abx500_mask_and_set(dev, AB8505_USB,
				USBOTGCTRL, IDHOSTENA | IDDEVENA |
				VBUSVALIDENA, 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Set iddet IP SW controllable */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL3, IDDETSWCTRLENA, IDDETSWCTRLENA);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Turn off USB PHY */
	ret = abx500_mask_and_set(dev, AB8505_USB, USBPHYCTRL,
			USBDEVICEMODEENA | USBHOSTMODEENA, 0x0);
	if (ret < 0)
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);

	return ret;
}

/* Check if the connected accessory has ID-USB resistance */
static int plug_unplug_monitor_init(struct device *dev)
{
	int ret;
	unsigned char id_voltage;

	/* 1microAmp current source pull up enable */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL2, IDDETPU1ENA, IDDETPU1ENA);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Enable ID detect comparator */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL1, PLUGDETCOMPENA, PLUGDETCOMPENA);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* read Iddet comparator state */
	ret = abx500_get(dev, AB8505_CHARGER,
				REGIDDETSTATE, &id_voltage);
	if (ret < 0) {
		dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* 1microAmp current source pull up disable */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL2, IDDETPU1ENA, 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	/* Disable ID detect comparator */
	ret = abx500_mask_and_set(dev, AB8505_CHARGER,
				REGIDDETCTRL1, PLUGDETCOMPENA, 0x0);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		return ret;
	}

	dev_dbg(dev, "Anything connected with ID-USB resistance? %s\n",
			(id_voltage & IDDETPLUGDETCOMP) ?
			"true" : "false");

	/* Return ID-USB resistance detected or not */
	return (ret < 0) ? ret : !!(id_voltage & IDDETPLUGDETCOMP);
}

/*
 * Work function to detect the uUSB cable plug/unplug.
 * On detecting a cable, cable specific call back is called to do the
 * cable specific initializations and on unplug of the same cable the
 * initializations done are undone.
 */
static void micro_usb_accessory_detect(struct work_struct *work)
{
	int ret;
	unsigned char vbusdet;
	bool restore_regs = true;
	static unsigned char usbotgctrl, usbphyctrl;
	unsigned char usblink1status;
	struct usb_accessory_state *accessory
		= container_of(work, struct usb_accessory_state,
				cable_detection.work);
	struct device *dev = accessory->dev;

	ret = abx500_get(dev, AB8505_USB, USBLINK1STATUS, &usblink1status);
	if (ret < 0) {
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
		goto restart;
	}

	/* Skip detection if UsbLink1Status[4:0] is set & != 0x80 */
	if ((usblink1status & 0xF8) && (usblink1status != 0x80))
		goto restart;

	/*
	 * On Audio device connection, stop the cable unplug detection.
	 * Audio device routine, will do the unplg detection and will
	 * queue iddet_workqueue on Audio device Unplug.
	 */
	if (accessory->cable_last_detected == USBSWITCH_AUDIODEV_TYPE1)
		return;

	if (accessory->cable_last_detected == USBSWITCH_UART_BOOT_ON ||
			accessory->cable_last_detected ==
			USBSWITCH_UART_BOOT_OFF ||
			accessory->cable_last_detected ==
			USBSWITCH_UART)
		return;

	/*
	 * If already a cable was detected, skip to id resistance change
	 * or vbus change detection direclty, i.e for unplug detection.
	 */
	if (accessory->cable_last_detected != USBSWITCH_NONE) {
		restore_regs = false;
		goto detect_unplug;
	}

	accessory->cable_detected = USBSWITCH_NONE;

	/* Take a backup of the registers, about to be changed */
	ret = abx500_get(dev, AB8505_USB, USBOTGCTRL, &usbotgctrl);
	if (ret < 0) {
		dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
		goto restart;
	}

	ret = abx500_get(dev, AB8505_USB, USBPHYCTRL, &usbphyctrl);
	if (ret < 0) {
		dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
		goto restart;
	}

	ret = usb_switch_init(dev);
	if (ret < 0)
		goto restore;

detect_unplug:
	/* read VbusDet state */
	ret = abx500_get(dev, AB8505_INTERRUPT, ITSOURCE2, &vbusdet);
	if (ret < 0) {
		dev_err(dev, "%s read failed %d\n", __func__, __LINE__);
		goto restore;
	}

	vbusdet &= VBUSDET;

	ret = plug_unplug_monitor_init(dev);
	if (!ret) {
		/*
		 * No ID Pin resistance detected.
		 * This can happen when
		 * 1 ) Connected cable is removed. cable_last_detected
		 *   is not NONE.
		 * 2 ) Cable was never connected.
		 * 3 ) Cable with no resistance is conencted and is not
		 *   detected by the link1status registor. If vbus
		 *   is detected then it is a carkit charger, with D+, D-,
		 *   ID pin floating.
		 */
		if (vbusdet) {
			accessory->cable_detected =
				USBSWITCH_LEGACY_CHARGER;
			if (accessory->cable_last_detected !=
					USBSWITCH_LEGACY_CHARGER)
				goto done;
			else
				goto restart;
		}

		if (accessory->cable_last_detected != USBSWITCH_NONE) {
			acessory_func_list[accessory->cable_last_detected]
				(accessory, false);
			accessory->cable_last_detected = USBSWITCH_NONE;
		}

		goto restore;
	}

	ret = detect_depending_on_id_resistance(accessory);
	if (ret < 0)
		goto restore;

done:
	dev_dbg(dev, "Cable ID Detected present %d last %d\n",
			accessory->cable_detected,
			accessory->cable_last_detected);

	if (accessory->cable_detected == accessory->cable_last_detected)
		goto restart;

	if (accessory->cable_last_detected != USBSWITCH_NONE)
		/* Call restore callback for the last connected device */
		acessory_func_list[accessory->cable_last_detected](accessory,
				false);

	accessory->cable_last_detected = accessory->cable_detected;

	/* Do cable specific plug related work */
	acessory_func_list[accessory->cable_detected](accessory, true);

	goto restart;

restore:
	/* Restore back registers */
	ret = abx500_set(dev, AB8505_USB, USBOTGCTRL, usbotgctrl);
	if (ret < 0)
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);

	ret = abx500_set(dev, AB8505_USB, USBPHYCTRL, usbphyctrl);
	if (ret < 0)
		dev_err(dev, "%s write failed %d\n", __func__, __LINE__);
restart:
	queue_delayed_work(accessory->iddet_workqueue,
			&accessory->cable_detection,
			msecs_to_jiffies(DETECTION_INTERVAL_MS));
}

static int init_key_press(struct platform_device *pdev,
		struct usb_accessory_state *accessory)
{
	int i;
	int irq;
	int ret;

	for (i = 0; i < sizeof(key_press_interrupts)/
			sizeof(key_press_interrupts[0]); i++) {

		irq = platform_get_irq_byname(pdev,
				key_press_interrupts[i].name);
		if (irq < 0) {
			dev_err(&pdev->dev,
				"%s: Failed to get irq %s\n", __func__,
				key_press_interrupts[i].name);
			return irq;
		}

		ret = request_threaded_irq(irq, NULL,
					key_press_interrupts[i].function,
					IRQF_TRIGGER_FALLING,
					key_press_interrupts[i].name,
					accessory);
		if (ret != 0) {
			dev_err(&pdev->dev,
				"%s: Failed to claim irq %s (%d)\n",
				__func__,
				key_press_interrupts[i].name,
				ret);
			return ret;
		}

		key_press_interrupts[i].irq = irq;
		disable_irq(key_press_interrupts[i].irq);
	}

	return 0;
}

/*
 * create input device for button press reporting
 */
static int init_button(struct usb_accessory_state *accessory)
{
	int err;

	usb_button = input_allocate_device();
	if (!usb_button) {
		dev_err(accessory->dev, "Input device alloc failed.\n");
		return -ENOMEM;
	}

	input_set_capability(usb_button, EV_KEY, KEY_MEDIA);

	usb_button->name = "uUSB button";
	usb_button->uniq = "uUSBbtn";
	usb_button->dev.parent = accessory->dev;

	err = input_register_device(usb_button);
	if (err) {
		dev_err(accessory->dev, "Input device registration failed %d",
			       err);
		input_free_device(usb_button);
		usb_button = NULL;
	}

	return err;
}

static int __devinit ab8505_iddet_probe(struct platform_device *pdev)
{
	int ret = 0;
	/* Set the thread as a priority one, with  MAX_RT_PRIO-5 */
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-5 };
	struct ab8500_platform_data *plat;
	struct ab8505_iddet_platdata *pdata;
	struct usb_accessory_state  *accessory;

	accessory = kzalloc(sizeof(*accessory), GFP_KERNEL);
	if (!accessory) {
		dev_err(&pdev->dev, "alloc failed\n");
		return -ENOMEM;
	}

	accessory->gpadc = ab8500_gpadc_get();
	if (!accessory->gpadc) {
		ret = -ENODEV;
		goto gpadc_get_fail;

	}

	accessory->dev = &pdev->dev;
	plat = dev_get_platdata(pdev->dev.parent);
	pdata = plat->iddet;
	accessory->cables_param_list = pdata->adc_id_list;
	acessory_func_list[USBSWITCH_UART] = pdata->uart_cable;
	accessory->btn_param_list = pdata->btn_list;

	ret = init_key_press(pdev, accessory);
	if (ret < 0) {
		dev_err(&pdev->dev, "key press interrupt init failed\n");
		goto key_init_fail;
	}

	ret = init_button(accessory);
	if (ret < 0) {
		dev_err(&pdev->dev, "Button init failed %d\n", ret);
		goto btn_init_fail;
	}

	/* Cable Plug/Unplug detection thread */
	INIT_DELAYED_WORK(&accessory->cable_detection,
			micro_usb_accessory_detect);

	accessory->iddet_workqueue =
		create_singlethread_workqueue("iddet_btn_detect");
	if (!accessory->iddet_workqueue) {
		dev_err(&pdev->dev, "%s: Failed to create wq\n", __func__);
		ret = -ENOMEM;
		goto create_iddet_worqueue_fail;
	}

	INIT_DELAYED_WORK(&accessory->detect_button, button_detection_function);

	btn_detection_work_queue =
		create_singlethread_workqueue("iddet_btn_detect");
	if (!btn_detection_work_queue) {
		dev_err(&pdev->dev, "%s: Failed to create wq\n", __func__);
		ret = -ENOMEM;
		goto create_btn_detection_fail;
	}

	/*
	 * Thread to read the gpadc value, needed to detect the button on
	 * the headset pressed. Since the GPADC read depends on a AB interrupt,
	 * the GPADC cannot be read here in the key_press AB interrupt. So a
	 * HIGH Priotiry thread is created.
	 */
	init_kthread_worker(&accessory->kworker);
	accessory->gpadc_read_thread = kthread_run(kthread_worker_fn,
					&accessory->kworker,
					dev_name(accessory->dev));
	if (IS_ERR(accessory->gpadc_read_thread)) {
		dev_err(&pdev->dev,
			"failed to create id detect thread\n");
		ret = -ENOMEM;
		goto kthread_err;
	}

	/*
	 * The key press may last for a very small time, it needs to be read
	 * quickly so make GPADC reading thread HIGHer priority one.
	 */
	sched_setscheduler(accessory->gpadc_read_thread, SCHED_FIFO, &param);

	init_kthread_work(&accessory->read_adc_work,
			get_iddet_adc_val);

	accessory->cable_last_detected = USBSWITCH_NONE;

	platform_set_drvdata(pdev, accessory);

	queue_delayed_work(accessory->iddet_workqueue,
			&accessory->cable_detection,
			msecs_to_jiffies(DETECTION_INTERVAL_MS));

	return 0;

kthread_err:
	flush_workqueue(btn_detection_work_queue);
	destroy_workqueue(btn_detection_work_queue);
create_btn_detection_fail:
	flush_workqueue(accessory->iddet_workqueue);
	destroy_workqueue(accessory->iddet_workqueue);
create_iddet_worqueue_fail:
btn_init_fail:
key_init_fail:
gpadc_get_fail:
	kfree(accessory);
	return ret;
}

static int __devexit ab8505_id_remove(struct platform_device *pdev)
{
	struct usb_accessory_state *accessory;

	accessory = platform_get_drvdata(pdev);
	hrtimer_cancel(&accessory->timer);
	flush_kthread_worker(&accessory->kworker);
	kthread_stop(accessory->gpadc_read_thread);
	kfree(accessory);

	return 0;
}

static struct platform_driver ab8505_iddet_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ab-iddet",
	},
	.probe = ab8505_iddet_probe,
	.remove = __devexit_p(ab8505_id_remove),
};

static int __init ab8505_iddet_init(void)
{
	return platform_driver_register(&ab8505_iddet_driver);
}

static void __exit ab8505_iddet_exit(void)
{
	platform_driver_unregister(&ab8505_iddet_driver);
}

late_initcall(ab8505_iddet_init);
module_exit(ab8505_iddet_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("micro usb accessory detection");
