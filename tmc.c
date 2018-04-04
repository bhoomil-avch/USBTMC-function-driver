/*
 * tmc.c -- TMC gadget driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/byteorder.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

USB_GADGET_COMPOSITE_OPTIONS();

#define DRIVER_DESC			"USBTMC Test & Measurement Device"
#define DRIVER_SHORTNAME		"tmc"
#define DRIVER_AUTHOR			"Bhoomil Chavda"
#define DRIVER_LICENSE			"GPL"
#define DRIVER_VERSION			"2018 FEB 27"

#define DRIVER_USB_GADGET_PRODUCT	DRIVER_DESC
#define DRIVER_USB_GADGET_MANUFACTURER	"XYZ Inc."
#define DRIVER_USB_GADGET_SERIAL	"1234ABCD"
#define DRIVER_USB_GADGET_LANGUAGE	0x0409	/* en-us */

#include "u_tmc.h"

/*-------------------------------------------------------------------------*/

/* DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures.
 */

/* Agilent VID/PID (only for experiment)
 */
#define DEFAULT_VENDOR_NUM	0x0957		/* Aglient Technologies, Inc. */
#define DEFAULT_PRODUCT_NUM	0x1745		/* Aglient Technologies Test and Measurement Device (IVI) */

/* Some systems will want different product identifiers published in the
 * device descriptor, either numbers or strings or both.  These string
 * parameters are in UTF-8 (superset of ASCII's 7 bit characters).
 */


static struct usb_function_instance *fi_tmc;
static struct usb_function *f_tmc;

/*-------------------------------------------------------------------------*/

/*
 * DESCRIPTORS ... most are static, but strings and (full) configuration
 * descriptors are built on demand.
 */

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	.idVendor =		cpu_to_le16(DEFAULT_VENDOR_NUM),
	.idProduct =		cpu_to_le16(DEFAULT_PRODUCT_NUM),
	.bcdDevice =		cpu_to_le16(0x0100),
	.bNumConfigurations =	1
};

static const struct usb_descriptor_header *otg_desc[2];

/*-------------------------------------------------------------------------*/

/* descriptors that are built on-demand */

/* static strings, in UTF-8 */
static struct usb_string		strings [] = {
	[USB_GADGET_MANUFACTURER_IDX].s = DRIVER_USB_GADGET_MANUFACTURER,
	[USB_GADGET_PRODUCT_IDX].s = DRIVER_USB_GADGET_PRODUCT,
	[USB_GADGET_SERIAL_IDX].s = DRIVER_USB_GADGET_SERIAL,
	{  }		/* end of list */
};

static struct usb_gadget_strings	stringtab_dev = {
	.language	= DRIVER_USB_GADGET_LANGUAGE,	/* en-us */
	.strings	= strings,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_configuration tmc_cfg_driver = {
	.label			= DRIVER_SHORTNAME,
	.bConfigurationValue	= 1,
	.bmAttributes		= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
};

static int tmc_do_config(struct usb_configuration *c){

	struct usb_gadget	*gadget = c->cdev->gadget;
	int			status = 0;

	usb_ep_autoconfig_reset(gadget);

	usb_gadget_set_selfpowered(gadget);

	if (gadget_is_otg(gadget)) {
		tmc_cfg_driver.descriptors = otg_desc;
		tmc_cfg_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	f_tmc = usb_get_function(fi_tmc);
	if (IS_ERR(f_tmc))
		return PTR_ERR(f_tmc);

	status = usb_add_function(c, f_tmc);
	if (status < 0)
		usb_put_function(f_tmc);

	return status;
}

static int tmc_bind(struct usb_composite_dev *cdev){

	struct f_tmc_opts *opts;
	int ret, len;

	fi_tmc = usb_get_function_instance("tmc");
	if (IS_ERR(fi_tmc))
		return PTR_ERR(fi_tmc);

	opts = container_of(fi_tmc, struct f_tmc_opts, func_inst);
	opts->minor = 0;

	ret = usb_string_ids_tab(cdev, strings);
	if (ret < 0)
		goto fail_put_func_inst;

	device_desc.iManufacturer = strings[USB_GADGET_MANUFACTURER_IDX].id;
	device_desc.iProduct = strings[USB_GADGET_PRODUCT_IDX].id;
	device_desc.iSerialNumber = strings[USB_GADGET_SERIAL_IDX].id;

	if (gadget_is_otg(cdev->gadget) && !otg_desc[0]) {
		struct usb_descriptor_header *usb_desc;

		usb_desc = usb_otg_descriptor_alloc(cdev->gadget);
		if (!usb_desc) {
			ret = -ENOMEM;
			goto fail_put_func_inst;
		}
		usb_otg_descriptor_init(cdev->gadget, usb_desc);
		otg_desc[0] = usb_desc;
		otg_desc[1] = NULL;
	}


	ret = usb_add_config(cdev, &tmc_cfg_driver, tmc_do_config);
	if (ret)
		goto fail_free_otg_desc;

	usb_composite_overwrite_options(cdev, &coverwrite);
	return ret;

fail_free_otg_desc:
	kfree(otg_desc[0]);
	otg_desc[0] = NULL;
fail_put_func_inst:
	usb_put_function_instance(fi_tmc);
	return ret;
}

static int tmc_unbind(struct usb_composite_dev *cdev){

	usb_put_function(f_tmc);
	usb_put_function_instance(fi_tmc);

	kfree(otg_desc[0]);
	otg_desc[0] = NULL;

	return 0;
}

static struct usb_composite_driver tmc_driver = {
	.name           = DRIVER_SHORTNAME,
	.dev            = &device_desc,
	.strings        = dev_strings,
	.max_speed      = USB_SPEED_SUPER,
	.bind		= tmc_bind,
	.unbind		= tmc_unbind,
};

module_usb_composite_driver(tmc_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE(DRIVER_LICENSE);

