/*
 * composite.c
 *
 * Copyright 2008 (C) Palm, Inc.
 *
 * Copyright 2006 (C) Instituto Nokia de Tecnologia - INdT
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/moduleparam.h>

#include <asm/byteorder.h>
#include <asm/system.h>
#include <asm/unaligned.h>

#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>
#include <linux/usb/gadget_event.h>

#include "../gadget_chips.h"

#include "composite.h"
#include "charger.h"

#if 0
#define dprintk(format, args...)  \
       printk("%s %d: " format , __FUNCTION__, __LINE__, ## args)
#else
#define dprintk(format, args...)
#endif

/*-------------------------------------------------------------------------*/
#define COMPOSITE_DESC		"Composite Driver"
#define COMPOSITE_VERSION	"v1.0-alpha"

//#define COMPOSITE_VERSION_NUM	0x0202
#define COMPOSITE_VERSION_NUM	0x0200
#define COMPOSITE_NAME		"composite"

static const char shortname [] = COMPOSITE_NAME;
static const char driver_desc [] = COMPOSITE_DESC;

/* descriptors that are built on-demand */
static char serial_number [64] = {"0"};
static char manufacturer [64] = {"Palm Inc."};
static char product_name [64] = {"Pre"};

#define DEFAULT_VENDOR_ID	0x0830 /* Palm, Inc. */

#if defined(CONFIG_MACH_BRISKET)
#define DEFAULT_PRODUCT_ID	0xc002
#elif defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
#define DEFAULT_PRODUCT_ID	0x8004
#else
#error "unknown machine type"
#endif

static unsigned int composite_version_num = COMPOSITE_VERSION_NUM;
static unsigned int vendor = DEFAULT_VENDOR_ID; /* module parm */
static unsigned int product = DEFAULT_PRODUCT_ID; /* module parm */

#define NDUID_AS_SERIAL_NUMBER

/* USB String IDs */
#define COMPOSITE_MANUFACTURER_ID 	1
#define COMPOSITE_PRODUCT_ID 		2
#define COMPOSITE_SERIALNUMBER_ID 	3

#define COMPOSITE_CONFIG_500MA_ID 	4
#define COMPOSITE_CONFIG_100MA_ID 	5

/* USB Strings, UTF8 */

static struct usb_string composite_strings[] = {
	{ COMPOSITE_MANUFACTURER_ID, manufacturer },
	{ COMPOSITE_PRODUCT_ID, product_name },
	{ COMPOSITE_SERIALNUMBER_ID, serial_number },
	{ COMPOSITE_CONFIG_500MA_ID, "Composite 500mA" },
	{ COMPOSITE_CONFIG_100MA_ID, "Composite 100mA" },
	{  } /* end */
};
static struct usb_gadget_strings composite_stringtable = {
	.language		= 0x0409, /* en-US */
	.strings		= composite_strings,
};
static struct usb_device_descriptor composite_device_desc = {
	.bLength		= USB_DT_DEVICE_SIZE,
	.bDescriptorType	= USB_DT_DEVICE,
	.bcdUSB			= __constant_cpu_to_le16(0x0200),
	.bDeviceClass		= USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass	= USB_CLASS_PER_INTERFACE,
	.bDeviceProtocol	= 0,
	.idVendor		= __constant_cpu_to_le16(DEFAULT_VENDOR_ID),
	.idProduct		= __constant_cpu_to_le16(DEFAULT_PRODUCT_ID),
	.iManufacturer		= COMPOSITE_MANUFACTURER_ID,
	.iProduct		= COMPOSITE_PRODUCT_ID,
	.iSerialNumber		= COMPOSITE_SERIALNUMBER_ID,
	.bNumConfigurations	= NUM_COMPOSITE_CONFIGS,
};
static struct usb_config_descriptor composite_500ma_config_desc = {
	.bLength		= USB_DT_CONFIG_SIZE,
	.bDescriptorType	= USB_DT_CONFIG,
	/* wTotalLenght computed dynamically */
	.bNumInterfaces		= MAX_COMPOSITE_INTERFACES,
	.bConfigurationValue	= COMPOSITE_500MA_CONFIG_VALUE,
	.iConfiguration		= COMPOSITE_CONFIG_500MA_ID,
	.bmAttributes 		= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER ,
	.bMaxPower		= USB_CHARGE_CURRENT_500MA,
};
static struct usb_config_descriptor composite_100ma_config_desc = {
	.bLength		= USB_DT_CONFIG_SIZE,
	.bDescriptorType	= USB_DT_CONFIG,
	/* wTotalLenght computed dynamically */
	.bNumInterfaces		= MAX_COMPOSITE_INTERFACES,
	.bConfigurationValue	= COMPOSITE_100MA_CONFIG_VALUE,
	.iConfiguration		= COMPOSITE_CONFIG_100MA_ID,
	.bmAttributes 		= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER ,
	.bMaxPower		= USB_CHARGE_CURRENT_100MA,
};
static struct usb_qualifier_descriptor composite_qualifier_desc = {
	.bLength =		sizeof composite_qualifier_desc,
	.bDescriptorType	= USB_DT_DEVICE_QUALIFIER,
	.bcdUSB			= __constant_cpu_to_le16(0x0200),
	.bDeviceClass		= USB_CLASS_PER_INTERFACE,
	.bNumConfigurations	= NUM_COMPOSITE_CONFIGS,
};

/*
 * usb_composite_driver (cdev->driver)
 */
static struct usb_composite_driver composite_drv = {
	.dev			= &composite_device_desc,
	.strings		= &composite_stringtable,
	.functions		= LIST_HEAD_INIT(composite_drv.functions),
};

static struct usb_composite_dev	*the_cdev = NULL;

/*-------------------------------------------------------------------------*/

static void parameter_update(void)
{
	struct usb_composite_driver *d = &composite_drv;
	struct usb_composite_dev	*cdev = the_cdev;
	int gcnum;

	d->vendor_id = vendor;
	d->product_id = product;

	composite_device_desc.idVendor = cpu_to_le16(d->vendor_id);
	composite_device_desc.idProduct = cpu_to_le16(d->product_id);

	cdev->dev.idVendor = cpu_to_le16(d->vendor_id);
	cdev->dev.idProduct = cpu_to_le16(d->product_id);

	gcnum = usb_gadget_controller_number(cdev->gadget);
	/* FIXME */
	if (gcnum >= 0)
		cdev->dev.bcdDevice =
				cpu_to_le16(composite_version_num| gcnum);
	else {
		dprintk("controller '%s' not recognized\n", cdev->gadget->name);
		/* unrecognized, but safe unless bulk is REALLY quirky */
		cdev->dev.bcdDevice =
			__constant_cpu_to_le16(composite_version_num|0x0099);
	}
}

static ssize_t show_product_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", product);
}

static ssize_t store_product_id(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int product_id;
	if (sscanf(buf, "%x", &product_id) != 1)
		return -EINVAL;
	product = product_id;	
	parameter_update();
	return count;
}

static ssize_t show_vendor_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", vendor);
}

static ssize_t store_vendor_id(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int vendor_id;
	if (sscanf(buf, "%x", &vendor_id) != 1)
		return -EINVAL;
	vendor = vendor_id;	
	parameter_update();
	return count;
}

static ssize_t show_serial_number(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", serial_number);
}

static ssize_t store_serial_number(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int len = count;

	if (buf[len-1] == '\n')
		--len;
	if (len >= 64)
		return -EINVAL;
	memcpy(serial_number, buf, len);
	serial_number[len] = '\0';

	parameter_update();
	return count;
}

static ssize_t show_composite_version_num(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", composite_version_num);
}

static ssize_t store_composite_version_num(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (sscanf(buf, "%x", &composite_version_num) != 1)
		return -EINVAL;
	parameter_update();
	return count;
}

static ssize_t show_manufacturer(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", manufacturer);
}

static ssize_t store_manufacturer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int len = count;

	if (buf[len-1] == '\n')
		--len;
	if (len >= 64)
		return -EINVAL;
	memcpy(manufacturer, buf, len);
	manufacturer[len] = '\0';

	parameter_update();
	return count;
}

static ssize_t show_product_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", product_name);
}

static ssize_t store_product_name(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int len = count;

	if (buf[len-1] == '\n')
		--len;
	if (len >= 64)
		return -EINVAL;
	memcpy(product_name, buf, len);
	product_name[len] = '\0';

	parameter_update();
	return count;
}

static ssize_t show_dump(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0;
	len += sprintf(buf+len, "Vendor ID %x\n", composite_device_desc.idVendor);
	len += sprintf(buf+len, "Product ID %x\n", composite_device_desc.idProduct);
	len += sprintf(buf+len, "CDEV Vendor ID %x\n", the_cdev->dev.idVendor);
	len += sprintf(buf+len, "CDEV Product ID %x\n", the_cdev->dev.idProduct);

	return len;
}

static ssize_t store_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(product_id, 0644, show_product_id, store_product_id);
static DEVICE_ATTR(vendor_id, 0644, show_vendor_id, store_vendor_id);
static DEVICE_ATTR(composite_version_num, 0644, show_composite_version_num, store_composite_version_num);
static DEVICE_ATTR(serial_number, 0644, show_serial_number, store_serial_number);
static DEVICE_ATTR(manufacturer, 0644, show_manufacturer, store_manufacturer);
static DEVICE_ATTR(product_name, 0644, show_product_name, store_product_name);
static DEVICE_ATTR(dump, 0644, show_dump, store_dump);


/*-------------------------------------------------------------------------*/
/*
 * config_buf()
 * composite_set_config()
 * composite_reset_config()
 */

static int
config_buf(struct usb_composite_dev *cdev, void *buf, u8 type,
	   unsigned index, int is_otg)
{
	struct usb_config_descriptor	*c = buf;
	void				*next = buf + USB_DT_CONFIG_SIZE;
	int				len = COMPOSITE_BUFSIZ - USB_DT_CONFIG_SIZE;
	int				hs;
	struct usb_function		*f;
	int config_value;
	int status;
	int n;
	int i;
	int next_interface;
	int try_maxpower;

	switch (index) {
	case 0:
		cdev->config = composite_500ma_config_desc;
		config_value = COMPOSITE_500MA_CONFIG_VALUE;
		break;
	case 1:
		cdev->config = composite_100ma_config_desc;
		config_value = COMPOSITE_100MA_CONFIG_VALUE;
		break;
	default:
		dprintk("choosing an unknown config\n");
		return -EINVAL;
	}

	try_maxpower = charger_detection_try_maxpower();
	if (try_maxpower > 0)
		cdev->config.bMaxPower = try_maxpower;

	if (cdev->gadget->is_dualspeed) {
		hs = (cdev->gadget->speed == USB_SPEED_HIGH);
		if (type == USB_DT_OTHER_SPEED_CONFIG)
			hs = !hs;
	} else
		hs = 0;

	next_interface = 0;
	list_for_each_entry (f, &cdev->driver->functions, function) {
		f->first_interface = next_interface;
		if (f->set_descriptors(config_value, is_otg) < 0) {
			dprintk("function %s set_descriptors error\n", f->name);
			return -EINVAL;
		}
		next_interface += f->num_interfaces;
	}

	/* set the function for each interface */
	n = 0;
	list_for_each_entry (f, &cdev->driver->functions, function) {
		if (f->num_interfaces <= 0) {
			dprintk("function %s has an invalid number of interfaces (%d)\n",
				f->name, f->num_interfaces);
			return -EINVAL;
		}
		for (i = 0; i < f->num_interfaces; i++)
			cdev->interface[n++] = f;
	}
        cdev->config.bNumInterfaces = n;
	dprintk("total %d interfaces\n", n);

	*c = cdev->config;
	c->bLength = USB_DT_CONFIG_SIZE;
	c->bDescriptorType = type;

	/* add each function's descriptors */
	list_for_each_entry (f, &cdev->driver->functions, function) {
		status = usb_descriptor_fillbuf(next, len,
				hs ? f->hs_descriptors : f->descriptors);
		if (status < 0) {
			dprintk("error status=%d\n", status);
			return status;
		}
		len -= status;
		next += status;
	}

	len = next - buf;
	c->wTotalLength = cpu_to_le16(len);

	return len;
}

static int
composite_set_config(struct usb_composite_dev *cdev, unsigned number)
{
	int			result = 0;
	struct usb_gadget	*gadget = cdev->gadget;
	struct usb_ctrlrequest	req;
	struct usb_function	*f;
	int			delayed_status = 0;
	int			try_maxpower = 0;

	memset(&req, 0, sizeof req);

	switch (number) {
	default:
		result = -EINVAL;
		dprintk("unknown config number=%d\n", number);
		/* FALLTHROUGH */
	case 0:
		cdev->config.bConfigurationValue = 0;
		usb_gadget_vbus_draw(gadget, gadget->is_otg ? 8 : 100);
		break;
	case COMPOSITE_500MA_CONFIG_VALUE:
	case COMPOSITE_100MA_CONFIG_VALUE:
		try_maxpower = charger_detection_try_maxpower();
		charger_detection_stop(cdev);

		req.bRequestType = USB_DIR_OUT
				| USB_TYPE_STANDARD
				| USB_RECIP_DEVICE;
		req.bRequest	= USB_REQ_SET_CONFIGURATION;
		req.wValue	= number;

		list_for_each_entry (f, &cdev->driver->functions, function) {
			cdev->current_func = f;
			result = f->setup(gadget, &req);
			if (result == DELAYED_STATUS)
				delayed_status = 1;
			if (result < 0)
				dprintk("set function configuration %s --> %d\n",
					f->name, result);
		}

		cdev->config.bConfigurationValue = number;

		if (try_maxpower > 0) {
			cdev->config.bMaxPower = try_maxpower;
		} else {
			switch (number) {
			case COMPOSITE_500MA_CONFIG_VALUE:
				cdev->config.bMaxPower = USB_CHARGE_CURRENT_500MA;
				break;
			case COMPOSITE_100MA_CONFIG_VALUE:
				cdev->config.bMaxPower = USB_CHARGE_CURRENT_100MA;
				break;
			default:
				dprintk("choosing an unknown config\n");
				return -EINVAL;
			}
		}
		usb_gadget_vbus_draw(cdev->gadget, 2 * cdev->config.bMaxPower);
		break;
	}

	printk(KERN_INFO "composite: %s speed config #%d\n",
		({ char *speed;
		switch (gadget->speed) {
		case USB_SPEED_LOW:	speed = "low"; break;
		case USB_SPEED_FULL:	speed = "full"; break;
		case USB_SPEED_HIGH:	speed = "high"; break;
		default:		speed = "?"; break;
		} ; speed; }), number);

	if (result >= 0 && delayed_status)
		return DELAYED_STATUS;
	else
		return result;
}

/*-------------------------------------------------------------------------*/
/* 
 * composite_setup()
 * composite_disconnect()
 */

static void
composite_collect_langs(struct usb_gadget_strings *sp, __le16 *buf)
{
	const struct usb_gadget_strings	*s;
	u16				language;
	__le16				*tmp;

	if (sp) {
		s = sp;
		language = cpu_to_le16(s->language);
		for (tmp = buf; *tmp && tmp < &buf[126]; tmp++) {
			if (*tmp == language)
				return;
		}
		*tmp++ = language;
	}
}

static int
composite_check_string(struct usb_gadget_strings *sp, void *buf,
		       u16 language, int id)
{
	struct usb_gadget_strings *s;
	int				value;

	if (sp) {
		s = sp;
		if (s->language != language)
			return -EINVAL;
		value = usb_gadget_get_string(s, id, buf);
		if (value > 0)
			return value;
	}
	return -EINVAL;
}

static int
composite_lookup_string(struct usb_composite_dev *cdev, void *buf, 
			u16 language, int id)
{
	struct usb_function		*f;
	int				len;

	/* 0 == report all available language codes */
	if (id == 0) {
		struct usb_string_descriptor	*s = buf;
		struct usb_gadget_strings *sp;

		memset(s, 0, COMPOSITE_BUFSIZ);
		s->bDescriptorType = USB_DT_STRING;

		sp = cdev->driver->strings;
		if (sp)
			composite_collect_langs(sp, s->wData);

		list_for_each_entry (f, &cdev->driver->functions, function) {
			sp = f->strings;
			if (sp)
				composite_collect_langs(sp, s->wData);
		}

		for (len = 0; s->wData[len] && len <= 126; len++)
			continue;
		if (!len)
			return -EINVAL;

		s->bLength = 2 * (len + 1);
		return s->bLength;
	}

	/* otherwise, look up and return a specified string */
	if (cdev->driver->strings) {
		len = composite_check_string(cdev->driver->strings,
				buf, language, id);
		if (len > 0)
			return len;
	}
	list_for_each_entry (f, &cdev->driver->functions, function) {
		if (!f->strings)
			continue;
		len = composite_check_string(f->strings, buf, language, id);
		if (len > 0)
			return len;
	}
	return -EINVAL;
}

static void
composite_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
	dprintk("enter\n");

	if (req->status || req->actual != req->length)
		dprintk("setup complete --> %d, %d/%d\n",
				req->status, req->actual, req->length);
}

static int
composite_setup_standard(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	int				value = -EOPNOTSUPP;
	u16				wIndex = le16_to_cpu(ctrl->wIndex);
	u16				wValue = le16_to_cpu(ctrl->wValue);
	u16				wLength = le16_to_cpu(ctrl->wLength);

	switch (ctrl->bRequest) {

	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_DIR_IN) {
			dprintk("GET_DESC: not DIR_IN!?\n");
			goto unknown;
		}
		switch (wValue >> 8) {

		case USB_DT_DEVICE:
			dprintk("GET_DESC(%d)-DEVICE(%d): wIndex=%d wLength=%d\n",
				ctrl->bRequest, wValue & 0xff, wIndex, wLength);
			value = min(wLength, (u16) sizeof cdev->dev);
			memcpy(req->buf, &cdev->dev, value);
			break;
#ifdef CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			dprintk("GET_DESC(%d)-DEVICE_QUAL(%d): wIndex=%d wLength=%d\n",
				ctrl->bRequest, wValue & 0xff, wIndex, wLength);
			if (!gadget->is_dualspeed)
				break;
			value = min(wLength, (u16) sizeof cdev->qual);
			memcpy(req->buf, &cdev->qual, value);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			dprintk("GET_DESC(%d)-OTHER_SPEED(%d): wIndex=%d wLength=%d\n",
				ctrl->bRequest, wValue & 0xff, wIndex, wLength);
			if (!gadget->is_dualspeed)
				break;
			// FALLTHROUGH
			goto fall_through;
#endif
		case USB_DT_CONFIG:
			dprintk("GET_DESC(%d)-CONFIG(%d): wIndex=%d wLength=%d\n",
				ctrl->bRequest, wValue & 0xff, wIndex, wLength);
#ifdef CONFIG_USB_GADGET_DUALSPEED
		fall_through:
#endif
			value = config_buf(cdev, req->buf, wValue >> 8,
					   wValue & 0xff, gadget->is_otg);
			if (value >= 0)
				value = min(wLength, (u16) value);

			charger_detection_start(cdev);

			break;
		case USB_DT_STRING:
			dprintk("GET_DESC(%d)-STRING(%d): wIndex=%d wLength=%d\n",
				ctrl->bRequest, wValue & 0xff, wIndex, wLength);
			value = composite_lookup_string(cdev, req->buf, wIndex,
						wValue & 0xff);
			if (value >= 0)
				value = min(wLength, (u16) value);
			break;
		}
		break;

	/* currently one config, two speeds */
	case USB_REQ_SET_CONFIGURATION:
		dprintk("SET_CONFIG(%d): wIndex=%d wValue=%d wLength=%d\n", 
			ctrl->bRequest, wIndex, wValue, wLength);
		if (ctrl->bRequestType != 0)
			goto unknown;
		if (gadget->a_hnp_support)
			dprintk("HNP available\n");
		else if (gadget->a_alt_hnp_support)
			dprintk("HNP needs a different root port\n");
		else
			dprintk("HNP inactive\n");
		spin_lock(&cdev->lock);
		value = composite_set_config(cdev, wValue);
		spin_unlock(&cdev->lock);
		break;
	case USB_REQ_GET_CONFIGURATION:
		dprintk("GET_CONFIG(%d): wIndex=%d wValue=%d wLength=%d\n",
			ctrl->bRequest, wIndex, wValue, wLength);
		if (ctrl->bRequestType != USB_DIR_IN)
			goto unknown;
		*(u8 *)req->buf = cdev->config.bConfigurationValue;
		value = min(wLength, (u16) 1);
		break;

	/* function drivers must handle get/set altsetting */
	case USB_REQ_SET_INTERFACE:
		dprintk("SET_INTERFACE(%d): wIndex=%d wValue=%d wLength=%d\n",
			ctrl->bRequest, wIndex, wValue, wLength);
		if (ctrl->bRequestType != USB_RECIP_INTERFACE)
			goto unknown;
		if (!cdev->config.bConfigurationValue
		    || wIndex >= MAX_COMPOSITE_INTERFACES
		    || !cdev->interface[wIndex]) {
			dprintk("SET_INTERFACE Error\n");
			break;
		}
		spin_lock(&cdev->lock);
		cdev->current_func = cdev->interface[wIndex];
		value = cdev->current_func->setup(gadget, ctrl);
		spin_unlock(&cdev->lock);
		break;
	case USB_REQ_GET_INTERFACE:
		dprintk("GET_INTERFACE(%d): wIndex=%d wValue=%d wLength=%d\n",
			ctrl->bRequest, wIndex, wValue, wLength);
		if (ctrl->bRequestType !=(USB_DIR_IN|USB_RECIP_INTERFACE))
			goto unknown;
		if (!cdev->config.bConfigurationValue
		    || wIndex >= MAX_COMPOSITE_INTERFACES
		    || !cdev->interface[wIndex]) {
			dprintk("GET_INTERFACE Error\n");
			break;
		}
		spin_lock(&cdev->lock);
		/* function must set cdev->req->buf[0] */
		cdev->current_func = cdev->interface[wIndex];
		value = cdev->current_func->setup(gadget, ctrl);
		spin_unlock(&cdev->lock);
		value = min(wLength, (u16) 1);
		break;
	default:
unknown:
		dprintk("unknown control req: type=%d req=%d wIndex=%d wValue=%d wLenght=%d\n",
			ctrl->bRequestType, ctrl->bRequest, wIndex, wValue, wLength);
	}
	return value;
}

static int
composite_setup_class(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	int				value = -EOPNOTSUPP;
	u16				wIndex = le16_to_cpu(ctrl->wIndex);

	dprintk("enter\n");
	if (wIndex >= MAX_COMPOSITE_INTERFACES ||
			!cdev->interface[wIndex])
		return value;

	spin_lock(&cdev->lock);
	/* function must set cdev->req->buf[0] */
	cdev->current_func = cdev->interface[wIndex];
	value = cdev->current_func->setup(gadget, ctrl);
	spin_unlock(&cdev->lock);
	if (value < 0 || value == DELAYED_STATUS)
		dprintk("return %d\n", value);
	return value;
}

static int
composite_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	int				value = -EOPNOTSUPP;
	//u16				wIndex = le16_to_cpu(ctrl->wIndex);
	//u16				wValue = le16_to_cpu(ctrl->wValue);
	u16				wLength = le16_to_cpu(ctrl->wLength);

	dprintk("enter\n");

	/* this is required to clean up leftover complete changes that may have
  	 * been done by other modules being loaded/removed */
	req->complete = composite_setup_complete;
	req->zero = 0;
	switch (ctrl->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_STANDARD:
		value = composite_setup_standard(gadget, ctrl);
		break;
	case USB_TYPE_CLASS:
		value = composite_setup_class(gadget, ctrl);
		break;
	default:
		dprintk("unknown control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			wValue, wIndex, wLength);
	}

	if (value == DELAYED_STATUS)
		dprintk("DELAYED_STATUS returned\n");

	/* respond with data transfer before status phase? */
	if (value >= 0 && value != DELAYED_STATUS) {
		req->length = value;
		req->zero = value < wLength;
		dprintk("ep0 queue length=%d\n", req->length);
		value = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			dprintk("ep_queue --> %d\n", value);
			req->status = 0;
			composite_setup_complete(gadget->ep0, req);
		}
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static void
composite_disconnect(struct usb_gadget *gadget)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_function *f;

	dprintk("calling each function's disconnect\n");

	list_for_each_entry (f, &cdev->driver->functions, function) {
		cdev->current_func = f;
		if (f->disconnect)
			f->disconnect(gadget);
	}
	charger_detection_enable(cdev);
}

/*-------------------------------------------------------------------------*/
/* 
 * composite_bind()
 * composite_unbind()
 */
static struct usb_gadget_driver the_composite_driver;

static void
composite_unbind(struct usb_gadget *gadget)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_function		*f;

	dprintk("enter\n");

	device_remove_file(&gadget->dev, &dev_attr_product_id);
	device_remove_file(&gadget->dev, &dev_attr_vendor_id);
	device_remove_file(&gadget->dev, &dev_attr_composite_version_num);
	device_remove_file(&gadget->dev, &dev_attr_serial_number);
	device_remove_file(&gadget->dev, &dev_attr_manufacturer);
	device_remove_file(&gadget->dev, &dev_attr_product_name);
	device_remove_file(&gadget->dev, &dev_attr_dump);

	list_for_each_entry (f, &cdev->driver->functions, function) {
		cdev->current_func = f;
		if (f->unbind)
			f->unbind(gadget);
		//if (f == cdev->current_func)
			//break;
	}
	charger_detection_unbind(cdev);

	if (cdev->req) {
		kfree(cdev->req->buf);
		usb_ep_free_request(gadget->ep0, cdev->req);
	}
	kfree(cdev);
	set_gadget_data(gadget, NULL);
}

static int
composite_bind(struct usb_gadget *gadget)
{
	struct usb_composite_dev	*cdev;
	struct usb_function		*f;
	int				status = -ENOMEM;
	int				gcnum;

	dprintk("enter\n");

	cdev = kzalloc(sizeof *cdev, GFP_KERNEL);
	if (!cdev)
		return status;

	spin_lock_init(&cdev->lock);
	cdev->gadget = gadget;
	set_gadget_data(gadget, cdev);

	/* automatic power adjustment */
	charger_detection_bind(cdev);
	charger_detection_enable(cdev);

	/* preallocate control response and buffer */
	cdev->req = usb_ep_alloc_request(gadget->ep0, GFP_KERNEL);
	if (!cdev->req)
		goto fail;
	cdev->req->buf = kmalloc(COMPOSITE_BUFSIZ, GFP_KERNEL);
	if (!cdev->req->buf)
		goto fail;

	cdev->req->complete = composite_setup_complete;
	gadget->ep0->driver_data = cdev;

	cdev->driver = &composite_drv;
	cdev->dev = *composite_drv.dev; /* copy */
	cdev->dev.bMaxPacketSize0 = gadget->ep0->maxpacket;

	cdev->gadget_driver = &the_composite_driver; /* REVISIT */

	usb_gadget_set_selfpowered(gadget);/* REVISIT */

	gcnum = usb_gadget_controller_number(gadget);
	/* FIXME */
	if (gcnum >= 0)
		cdev->dev.bcdDevice =
				cpu_to_le16(composite_version_num| gcnum);
	else {
		dprintk("controller '%s' not recognized\n", gadget->name);
		/* unrecognized, but safe unless bulk is REALLY quirky */
		cdev->dev.bcdDevice =
			__constant_cpu_to_le16(composite_version_num|0x0099);
	}

	/* REVISIT Code from serial gadget? */
	if (gadget->is_otg) {
		composite_500ma_config_desc.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
		composite_100ma_config_desc.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	usb_ep_autoconfig_reset(gadget);

	list_for_each_entry (f, &cdev->driver->functions, function) {
		cdev->current_func = f;
		status = f->bind(gadget);
		if (status < 0) {
			dprintk("%s bind failed\n", f->name);
			goto fail;
		}
	}

	cdev->current_func = NULL;

	/* fix qual */
	cdev->qual = composite_qualifier_desc;
	if (gadget->is_dualspeed) {
		cdev->qual.bLength = sizeof cdev->qual;
		cdev->qual.bDescriptorType = USB_DT_DEVICE_QUALIFIER;

		cdev->qual.bcdUSB = cdev->dev.bcdUSB;
		cdev->qual.bDeviceClass = cdev->dev.bDeviceClass;
		cdev->qual.bDeviceProtocol = cdev->dev.bDeviceProtocol;

		/* assume ep0 uses the same value for both speeds ... */
		cdev->qual.bMaxPacketSize0 = cdev->dev.bMaxPacketSize0;

		cdev->qual.bNumConfigurations = cdev->dev.bNumConfigurations;
	}

	if ((status = device_create_file(&gadget->dev,
							&dev_attr_product_id)) != 0 ||
		(status = device_create_file(&gadget->dev,
							&dev_attr_vendor_id)) != 0 ||
		(status = device_create_file(&gadget->dev,
							&dev_attr_composite_version_num)) != 0 ||
		(status = device_create_file(&gadget->dev,
							&dev_attr_serial_number)) != 0 ||
		(status = device_create_file(&gadget->dev,
							&dev_attr_manufacturer)) != 0 ||
		(status = device_create_file(&gadget->dev,
							&dev_attr_product_name)) != 0 ||
		(status = device_create_file(&gadget->dev, &dev_attr_dump)) != 0) {

		printk(KERN_ERR "%s could not register devattrs.\n",
					cdev->driver->name);
		goto fail;
	}

	the_cdev = cdev;

	printk(KERN_INFO "%s ready\n", cdev->driver->name);
	return 0;

fail:
	composite_unbind(gadget);
	return status;
}

/*-------------------------------------------------------------------------*/
/* 
 * composite_suspend()
 * composite_rusume()
 */

static void
composite_suspend(struct usb_gadget *gadget)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_function		*f;

	dprintk("suspend\n");

	/* revisit -- iterate cdev->interface? */
	list_for_each_entry (f, &cdev->driver->functions, function) {
		cdev->current_func = f;
		if (!f->suspend)
			continue;
		f->suspend(gadget);
	}
}

static void
composite_resume(struct usb_gadget *gadget)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_function		*f;

	dprintk("resume\n");

	/* revisit -- iterate cdev->interface? */
	list_for_each_entry (f, &cdev->driver->functions, function) {
			cdev->current_func = f;
		if (!f->resume)
			continue;
		f->resume(gadget);
	}
}

/*-------------------------------------------------------------------------*/

static struct usb_gadget_driver the_composite_driver = {
	.speed		= USB_SPEED_HIGH,

	.bind		= composite_bind,
	.unbind		=/* __exit_p */(composite_unbind),

	.setup		= composite_setup,
	.disconnect	= composite_disconnect,

	.suspend	= composite_suspend,
	.resume		= composite_resume,

	.driver	= {
		.owner		= THIS_MODULE,
		.name 		= (char *) shortname,
	},
};

/*-------------------------------------------------------------------------*/

static int usb_function_register(struct usb_function *g_func)
{
	struct usb_composite_driver *d = &composite_drv;

	if (!g_func->name)
		g_func->name = "Gadget";

	if (g_func->init) {
		int retval;
		retval = g_func->init();
		if (retval < 0) {
			printk(KERN_ERR "USB Function (%s) failed to register.\n",
			       g_func->name);
			return retval;
		}
	}

	list_add_tail(&g_func->function, &d->functions);

	printk(KERN_INFO "USB Function (%s) registered.\n", g_func->name);

	return 0;
}

static void usb_function_unregister_all(void)
{
	struct usb_composite_driver *d = &composite_drv;
	struct list_head *entry, *tmp;
	struct usb_function *g_func;

	list_for_each_safe (entry, tmp, &d->functions) {
		g_func = list_entry (entry, struct usb_function, function);
		if (g_func->exit)
			g_func->exit();
		printk(KERN_INFO "USB Function (%s) unregistered.\n", g_func->name);
		list_del_init (&g_func->function);
	}
}

/*-------------------------------------------------------------------------*/

/* REVISIT */
int usb_composite_ep_reset (struct usb_ep *ep)
{
	if (!ep)
		return -EINVAL;

	ep->driver_data = NULL;

	return 0;
}
EXPORT_SYMBOL(usb_composite_ep_reset);

/*-------------------------------------------------------------------------*/
/*
 * composite_assemble_product()
 * composite_disassemble_product()
 */

extern struct usb_function eth_usb_function;
extern struct usb_function gs4_usb_function;
extern struct usb_function fsg_usb_function;
extern struct usb_function gadgetfs_usb_function;

int
composite_assemble_product(struct usb_composite_driver *d)
{
#ifdef NDUID_AS_SERIAL_NUMBER
	extern char *nduid_string_get(void);
#endif

#ifdef NDUID_AS_SERIAL_NUMBER
	if (nduid_string_get()) {
		strncpy(serial_number, nduid_string_get(),
			sizeof(serial_number)-1);
		printk(KERN_INFO "%s: serial_number=%s\n", __func__,
		       serial_number);
	} else {
		printk(KERN_INFO "%s: can't get nduid\n", __func__);
	}
#else
	snprintf(serial_number, sizeof(serial_number), "%08x%08x",
		 system_serial_high, system_serial_low);
#endif

	composite_device_desc.idVendor = cpu_to_le16(d->vendor_id);
	composite_device_desc.idProduct = cpu_to_le16(d->product_id);
	/*
	 * REVISIT should fix .iManufacturer, .iProduct ?
	 */

	switch (d->product_id) {
	case 0x100:
		composite_device_desc.iSerialNumber = 0;
		usb_function_register(&eth_usb_function);
		usb_function_register(&gs4_usb_function);
		break;
	case 0x101:
		composite_device_desc.iSerialNumber = COMPOSITE_SERIALNUMBER_ID;
		usb_function_register(&eth_usb_function);
		usb_function_register(&fsg_usb_function);
		usb_function_register(&gadgetfs_usb_function);
		break;
	case 0x8002:
	case 0xc002:
		composite_device_desc.iSerialNumber = COMPOSITE_SERIALNUMBER_ID;
		usb_function_register(&fsg_usb_function);
		usb_function_register(&gadgetfs_usb_function);
		break;
	case 0x8003:
	case 0xc003:
		composite_device_desc.iSerialNumber = 0;
		usb_function_register(&gs4_usb_function);
		/* usb_function_register(&gadgetfs_usb_function); */
		break;
	case 0x8004: //Castle retail
		composite_device_desc.iSerialNumber = COMPOSITE_SERIALNUMBER_ID;
		usb_function_register(&fsg_usb_function);
		break;
	default:
		printk("%s: unknown product\n", __FUNCTION__);
		return -EINVAL;
	}

	return 0;
}

int
composite_disassemble_product(struct usb_composite_driver *d)
{
	usb_function_unregister_all();

	return 0;
}

/*-------------------------------------------------------------------------*/

static int composite_init(void)
{
	int retval = 0;
	struct usb_composite_driver *d = &composite_drv;

	if (!d->name)
		d->name = shortname;

	the_composite_driver.function = (char *) d->name;
	the_composite_driver.driver.name = d->name;

	d->vendor_id = vendor;
	d->product_id = product;
	printk(KERN_INFO "%s: vendor_id=%x product_id=%x\n", 
	       __FUNCTION__, d->vendor_id, d->product_id);

	retval = composite_assemble_product(d);
	if (retval < 0)
		goto fail;

	retval = usb_gadget_register_driver(&the_composite_driver);
	if (retval < 0) {
		printk(KERN_ERR "%s: usb_gadget_register_driver failed\n",
		       __FUNCTION__);
		goto fail;
	}

	return 0;

fail:
	composite_disassemble_product(d);
	return retval;
}

static void composite_exit(void)
{
	struct usb_composite_driver *d = &composite_drv;

	usb_gadget_unregister_driver(&the_composite_driver);

	composite_disassemble_product(d);

	printk(KERN_INFO "composite_exit: %s %s unloaded\n",
		COMPOSITE_DESC, COMPOSITE_VERSION);
}
module_init(composite_init);
module_exit(composite_exit);

module_param(vendor, uint, S_IRUGO);
MODULE_PARM_DESC(vendor, "vendor id");

module_param(product, uint, S_IRUGO);
MODULE_PARM_DESC(product, "product id");


MODULE_DESCRIPTION(COMPOSITE_DESC);
MODULE_AUTHOR("Felipe Balbi, Ragner Magalhaes, Toshi Kikuchi");
MODULE_LICENSE("GPL");
