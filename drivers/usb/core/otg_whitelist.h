/*
 * drivers/usb/core/otg_whitelist.h
 *
 * Copyright (C) 2004 Texas Instruments
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * This OTG Whitelist is the OTG "Targeted Peripheral List".  It should
 * mostly use of USB_DEVICE() or USB_DEVICE_VER() entries..
 *
 * YOU _SHOULD_ CHANGE THIS LIST TO MATCH YOUR PRODUCT AND ITS TESTING!
 */

static struct usb_device_id whitelist_table [] = {

/* hubs are optional in OTG, but very handy ... */
{ USB_DEVICE_INFO(USB_CLASS_HUB, 0, 0), },
{ USB_DEVICE_INFO(USB_CLASS_HUB, 0, 1), },

#ifdef	CONFIG_USB_PRINTER		/* ignoring nonstatic linkage! */
/* FIXME actually, printers are NOT supposed to use device classes;
 * they're supposed to use interface classes...
 */
{ USB_DEVICE_INFO(7, 1, 1) },
{ USB_DEVICE_INFO(7, 1, 2) },
{ USB_DEVICE_INFO(7, 1, 3) },
#endif

#ifdef	CONFIG_USB_NET_CDCETHER
/* Linux-USB CDC Ethernet gadget */
{ USB_DEVICE(0x0525, 0xa4a1), },
/* Linux-USB CDC Ethernet + RNDIS gadget */
{ USB_DEVICE(0x0525, 0xa4a2), },
#endif

#if	defined(CONFIG_USB_TEST) || defined(CONFIG_USB_TEST_MODULE)
/* gadget zero, for testing */
{ USB_DEVICE(0x0525, 0xa4a0), },
#endif

#ifdef CONFIG_USB_OTG_20
{ USB_DEVICE_INFO(USB_CLASS_HID, 1, 1) },    /* Keyboard Devices */
{ USB_DEVICE_INFO(USB_CLASS_HID, 1, 2) },    /* Mouse Devices */
{ USB_DEVICE_INFO(USB_CLASS_HID, 0, 0) },    /* Gamepad Devices */

#ifdef CONFIG_JOYSTICK_XPAD
{ USB_DEVICE_INFO(USB_CLASS_VENDOR_SPEC, 0x5d, 0x81), },
{ USB_DEVICE_INFO(USB_CLASS_VENDOR_SPEC, 0x5d, 0x01), },
#endif

/* Test Devices */
{ USB_DEVICE(0x1A0A, 0x0101), },/* Test_SE0_NAK */
{ USB_DEVICE(0x1A0A, 0x0102), },/* Test_J */
{ USB_DEVICE(0x1A0A, 0x0103), },/* Test_K */
{ USB_DEVICE(0x1A0A, 0x0104), },/* Test_Packet */
{ USB_DEVICE(0x1A0A, 0x0106), },/* HS_HOST_PORT_SUSPEND_RESUME */
{ USB_DEVICE(0x1A0A, 0x0107), },/* SINGLE_STEP_GET_DEV_DESC */
{ USB_DEVICE(0x1A0A, 0x0108), },/* SINGLE_STEP_ GET_DEV_DESC_DATA*/
{ USB_DEVICE(0x1A0A, 0x0201), },/*  OTG 2 TEST DEVICE*/
#endif
{ }	/* Terminating entry */
};

/* The TEST_MODE Definition for OTG as per 6.4 of OTG Rev 2.0 */

#ifdef CONFIG_USB_OTG_20
#define USB_OTG_TEST_MODE_VID                          0x1A0A
#define USB_OTG_TEST_SE0_NAK_PID                       0x0101
#define USB_OTG_TEST_J_PID		               0x0102
#define USB_OTG_TEST_K_PID		               0x0103
#define USB_OTG_TEST_PACKET_PID	                       0x0104
#define USB_OTG_TEST_HS_HOST_PORT_SUSPEND_RESUME_PID   0x0106
#define USB_OTG_TEST_SINGLE_STEP_GET_DEV_DESC_PID      0x0107
#define USB_OTG_TEST_SINGLE_STEP_GET_DEV_DESC_DATA_PID 0x0108
#define USB_OTG_TEST_SE0_NAK                           0x01
#define USB_OTG_TEST_J		                       0x02
#define USB_OTG_TEST_K		                       0x03
#define USB_OTG_TEST_PACKET	                       0x04
/* For A_HNP and B_HNP test cases PET identifies itself
 * with a PID of 0x0200
 */
#define USB_OTG_PET_TEST_HNP			       0x0200

#endif

static int is_targeted(struct usb_device *dev)
{
	struct usb_device_id	*id = whitelist_table;
#ifdef CONFIG_USB_OTG_20
	u8 nifc = dev->config ? dev->config->desc.bNumInterfaces : 0;
	u8 i;
#endif

	/* possible in developer configs only! */
	if (!dev->bus->otg_port)
		return 1;

	/* HNP test device is _never_ targeted (see OTG spec 6.6.6) */
	if ((le16_to_cpu(dev->descriptor.idVendor) == 0x1a0a &&
	     le16_to_cpu(dev->descriptor.idProduct) == 0xbadd))
		return 0;

	/* NOTE: can't use usb_match_id() since interface caches
	 * aren't set up yet. this is cut/paste from that code.
	 */
	for (id = whitelist_table; id->match_flags; id++) {
		if ((id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
		    id->idVendor != le16_to_cpu(dev->descriptor.idVendor))
			continue;

		if ((id->match_flags & USB_DEVICE_ID_MATCH_PRODUCT) &&
		    id->idProduct != le16_to_cpu(dev->descriptor.idProduct))
			continue;

		/* No need to test id->bcdDevice_lo != 0, since 0 is never
		   greater than any unsigned number. */
		if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_LO) &&
		    (id->bcdDevice_lo > le16_to_cpu(dev->descriptor.bcdDevice)))
			continue;

		if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_HI) &&
		    (id->bcdDevice_hi < le16_to_cpu(dev->descriptor.bcdDevice)))
			continue;

		if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_CLASS) &&
		    (id->bDeviceClass != dev->descriptor.bDeviceClass))
			continue;

		if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_SUBCLASS) &&
		    (id->bDeviceSubClass != dev->descriptor.bDeviceSubClass))
			continue;

		if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_PROTOCOL) &&
		    (id->bDeviceProtocol != dev->descriptor.bDeviceProtocol))
			continue;

/* add other match criteria here ... */
#ifdef CONFIG_USB_OTG_20
		/* Checking class,subclass and protocal at interface level */
		for (i = 0; i < nifc; i++) {
			if (!dev->config->intf_cache[i])
				continue;

			if ((id->match_flags &
				USB_DEVICE_ID_MATCH_INT_CLASS)
				&& (id->bInterfaceClass !=
				dev->config->intf_cache[i]
				->altsetting[0].desc.bInterfaceClass))
				continue;

			if ((id->match_flags &
				USB_DEVICE_ID_MATCH_INT_SUBCLASS)
				&& (id->bInterfaceSubClass !=
				dev->config->intf_cache[i]
				->altsetting[0].desc.bInterfaceSubClass))
				continue;

			if ((id->match_flags &
				USB_DEVICE_ID_MATCH_INT_PROTOCOL)
				&& (id->bInterfaceProtocol !=
				dev->config->intf_cache[i]
				->altsetting[0].desc.bInterfaceProtocol))
				continue;

			break;
		}

		if (nifc && i == nifc)
			continue;
#endif
		return 1;
	}


	/* OTG MESSAGE: report errors here, customize to match your product */
	dev_err(&dev->dev, "device v%04x p%04x is not supported\n",
		le16_to_cpu(dev->descriptor.idVendor),
		le16_to_cpu(dev->descriptor.idProduct));
#ifdef	CONFIG_USB_OTG_WHITELIST
	return 0;
#else
	return 1;
#endif
}

