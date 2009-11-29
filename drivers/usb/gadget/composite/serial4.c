/*
 * serial4.c -- USB gadget serial driver
 *
 * Copyright 2008 (C) Palm, Inc.
 *
 * Copyright 2003 (C) Al Borchers (alborchers@steinerpoint.com)
 *
 * This code is based in part on the Gadget Zero driver, which
 * is Copyright (C) 2003 by David Brownell, all rights reserved.
 *
 * This code also borrows from usbserial.c, which is
 * Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2000 Peter Berger (pberger@brimson.com)
 * Copyright (C) 2000 Al Borchers (alborchers@steinerpoint.com)
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * version 2 of that License.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>

#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>

#include <linux/usb/passthru.h>

#include "../gadget_chips.h"

#include "composite.h"

#if 0
#define dprintk(format, args...)  \
       printk("%s %d: " format , __FUNCTION__, __LINE__, ## args)
#else
#define dprintk(format, args...)
#endif

/*-------------------------------------------------------------------------*/

/* Defines */

#define GS4_VERSION_STR			"v2.2"
#define GS4_VERSION_NUM			0x0202

#define GS4_LONG_NAME			"Gadget Serial multiport"
#define GS4_SHORT_NAME			"g_serial4"

#define GS4_MAJOR			127
#define GS4_MINOR_START			0

#define GS4_MAX_NUM_PORTS		4
#define GS4_DEFAULT_NUM_PORTS		2

#define GS4_NUM_CONFIGS			1
#define GS4_NO_CONFIG_ID		0
#define GS4_BULK_CONFIG_ID		1
#define GS4_ACM_CONFIG_ID		2
#define GS4_QC_CONFIG_ID		3

#define GS4_MAX_NUM_INTERFACES		3
#define GS4_BULK_INTERFACE_ID		0
#define GS4_CONTROL_INTERFACE_ID	0
#define GS4_DATA_INTERFACE_ID		1
#define GS4_QC_INTERFACE_ID		0

#define GS4_MAX_DESC_LEN		256

#define GS4_DEFAULT_READ_Q_SIZE		32
#define GS4_DEFAULT_WRITE_Q_SIZE	32

#define GS4_DEFAULT_WRITE_BUF_SIZE	8192
#define GS4_TMP_BUF_SIZE		8192

#define GS4_CLOSE_TIMEOUT		15

#define GS4_DEFAULT_USE_ACM		0
#define GS4_DEFAULT_USE_QC		1

#define GS4_DEFAULT_DTE_RATE		480000000
#define GS4_DEFAULT_DATA_BITS		8
#define GS4_DEFAULT_PARITY		USB_CDC_NO_PARITY
#define GS4_DEFAULT_CHAR_FORMAT		USB_CDC_1_STOP_BITS

/* Output control lines */
#define ACM_CTRL_DTR		0x01
#define ACM_CTRL_RTS		0x02

/* Input control lines */
#define ACM_CTRL_DCD		0x01
#define ACM_CTRL_DSR		0x02
#define ACM_CTRL_BRK		0x04
#define ACM_CTRL_RI		0x08
#define ACM_CTRL_FRAMING	0x10
#define ACM_CTRL_PARITY		0x20
#define ACM_CTRL_OVERRUN	0x40

/* select highspeed/fullspeed, hiding highspeed if not configured */
#ifdef CONFIG_USB_GADGET_DUALSPEED
#define GS4_SPEED_SELECT(is_hs,hs,fs) ((is_hs) ? (hs) : (fs))
#else
#define GS4_SPEED_SELECT(is_hs,hs,fs) (fs)
#endif /* CONFIG_USB_GADGET_DUALSPEED */

/* Thanks to NetChip Technologies for donating this product ID.
 *
 * DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures.
 */
#define GS4_VENDOR_ID			0x0525	/* NetChip */
#define GS4_PRODUCT_ID			0xa4a6	/* Linux-USB Serial Gadget */
#define GS4_CDC_PRODUCT_ID		0xa4a7	/* ... as CDC-ACM */
#define GS4_QC_PRODUCT_ID		0xa4a8	/* ... as QC emulation */

#define GS4_LOG2_NOTIFY_INTERVAL	7	/* 1 << 7 == 128 msec */
#define GS4_NOTIFY_MAXPACKET		16


/* Structures */

struct gs4_dev;

/* circular buffer */
struct gs4_buf {
	unsigned int		buf_size;
	char			*buf_buf;
	char			*buf_get;
	char			*buf_put;
};

/* list of requests */
struct gs4_req_entry {
	struct list_head	re_entry;
	struct usb_request	*re_req;
};

struct gs4_ctlreq_entry {
	struct list_head		list;
	struct pioc_cdc_control_request	ctlreq;
};

#define GS4_NCTLREQ	16

/* the port structure holds info for each port, one for each minor number */
struct gs4_port {
	struct gs4_dev 		*port_dev;	/* pointer to device struct */
	struct tty_struct	*port_tty;	/* pointer to tty struct */
	spinlock_t		port_lock;
	int 			port_num;
	int			port_open_count;
	int			port_in_use;	/* open/close in progress */
	wait_queue_head_t	port_write_wait;/* waiting to write */
	struct gs4_buf		*port_write_buf;
	struct usb_cdc_line_coding	port_line_coding;

	struct gs4_ctlreq_entry	port_ctlreq_entry[GS4_NCTLREQ];
	struct list_head	port_spare_ctlreq_entries;
	struct list_head	port_filled_ctlreq_entries;
	wait_queue_head_t	port_ctlreq_wait;
};

/* the device structure holds info for the USB device */
struct gs4_dev {
	struct usb_gadget	*dev_gadget;	/* gadget device pointer */
	spinlock_t		dev_lock;	/* lock for set/reset config */
	int			dev_config;	/* configuration number */
	struct usb_ep		*dev_notify_ep[GS4_MAX_NUM_PORTS];	/* address of notify endpoint */
	struct usb_ep		*dev_in_ep [GS4_MAX_NUM_PORTS] ;	/* address of in endpoint */
	struct usb_ep		*dev_out_ep [GS4_MAX_NUM_PORTS] ;	/* address of out endpoint */
	struct usb_endpoint_descriptor		/* descriptor of notify ep */
				*dev_notify_ep_desc[GS4_MAX_NUM_PORTS];
	struct usb_endpoint_descriptor		/* descriptor of in endpoint */
				*dev_in_ep_desc[GS4_MAX_NUM_PORTS];
	struct usb_endpoint_descriptor		/* descriptor of out endpoint */
				*dev_out_ep_desc[GS4_MAX_NUM_PORTS];
	struct usb_request	*dev_ctrl_req;	/* control request */
	struct usb_request	*dev_notify_req;/* notification request */
	int			dev_notify_req_avail;
	struct list_head	dev_req_list [GS4_MAX_NUM_PORTS] ;	/* list of write requests */
	//volatile int		dev_write_req_pending;
	int			dev_sched_port;	/* round robin port scheduled */
	struct gs4_port		*dev_port[GS4_MAX_NUM_PORTS]; /* the ports */
	//unsigned                suspended:1;
	wait_queue_head_t	dev_notify_req_wait;
};


/* Functions */

/* REVISIT module */
#ifdef USB_COMPOSITE_DEVICE
static int gs4_init(void);
static void gs4_exit(void);
#else
static int gs4_init(void);
static void gs4_exit(void);
#endif

/* tty driver */
static int gs4_open(struct tty_struct *tty, struct file *file);
static void gs4_close(struct tty_struct *tty, struct file *file);
static int gs4_write(struct tty_struct *tty,
	const unsigned char *buf, int count);
static void gs4_put_char(struct tty_struct *tty, unsigned char ch);
static void gs4_flush_chars(struct tty_struct *tty);
static int gs4_write_room(struct tty_struct *tty);
static int gs4_chars_in_buffer(struct tty_struct *tty);
static void gs4_throttle(struct tty_struct * tty);
static void gs4_unthrottle(struct tty_struct * tty);
static void gs4_break(struct tty_struct *tty, int break_state);
static int  gs4_ioctl(struct tty_struct *tty, struct file *file,
	unsigned int cmd, unsigned long arg);
static void gs4_set_termios(struct tty_struct *tty, struct ktermios *old);

static void gs4_ioc_send_complete(struct usb_ep *ep, struct usb_request *req);
static int gs4_ioc_send_notification(struct gs4_port *port, struct pioc_cdc_notification __user *u_notif);
static int gs4_ioc_recv_ctrl_request(struct gs4_port *port, struct pioc_cdc_control_request __user *u_req);

static int gs4_send(struct gs4_dev *dev, int port_num);
static int gs4_send_packet(struct gs4_dev *dev, char *packet,
	unsigned int size, int port_num);
static int gs4_recv_packet(struct gs4_dev *dev, char *packet,
	unsigned int size, int port_num);
static void gs4_read_complete0(struct usb_ep *ep, struct usb_request *req);
static void gs4_write_complete0(struct usb_ep *ep, struct usb_request *req);
static void gs4_read_complete1(struct usb_ep *ep, struct usb_request *req);
static void gs4_write_complete1(struct usb_ep *ep, struct usb_request *req);
static void gs4_read_complete2(struct usb_ep *ep, struct usb_request *req);
static void gs4_write_complete2(struct usb_ep *ep, struct usb_request *req);
static void gs4_read_complete3(struct usb_ep *ep, struct usb_request *req);
static void gs4_write_complete3(struct usb_ep *ep, struct usb_request *req);

/* gadget driver */
static int gs4_bind(struct usb_gadget *gadget);
static void gs4_unbind(struct usb_gadget *gadget);
#ifdef USB_COMPOSITE_DEVICE
static int gs4_set_descriptors(int config, int is_otg);
#endif
static int gs4_setup(struct usb_gadget *gadget,
	const struct usb_ctrlrequest *ctrl);
static int gs4_setup_standard(struct usb_gadget *gadget,
	const struct usb_ctrlrequest *ctrl);
static int gs4_setup_class(struct usb_gadget *gadget,
	const struct usb_ctrlrequest *ctrl);
static void gs4_disconnect(struct usb_gadget *gadget);
//static void gs4_suspend (struct usb_gadget *gadget);
//static void gs4_resume (struct usb_gadget *gadget);
static int gs4_set_config(struct gs4_dev *dev, unsigned config);
static void gs4_reset_config(struct gs4_dev *dev);
#ifndef USB_COMPOSITE_DEVICE
static void gs4_setup_complete(struct usb_ep *ep, struct usb_request *req);
static int gs4_build_config_buf(u8 *buf, enum usb_device_speed speed,
		u8 type, unsigned int index, int is_otg);
#endif /* !USB_COMPOSITE_DEVICE */

static struct usb_request *gs4_alloc_req(struct usb_ep *ep, unsigned int len,
	gfp_t kmalloc_flags);
static void gs4_free_req(struct usb_ep *ep, struct usb_request *req);

static struct gs4_req_entry *gs4_alloc_req_entry(struct usb_ep *ep, unsigned len,
	gfp_t kmalloc_flags);
static void gs4_free_req_entry(struct usb_ep *ep, struct gs4_req_entry *req);

static int gs4_alloc_ports(struct gs4_dev *dev, gfp_t kmalloc_flags);
static void gs4_free_ports(struct gs4_dev *dev);

/* circular buffer */
static struct gs4_buf *gs4_buf_alloc(unsigned int size, gfp_t kmalloc_flags);
static void gs4_buf_free(struct gs4_buf *gb);
static void gs4_buf_clear(struct gs4_buf *gb);
static unsigned int gs4_buf_data_avail(struct gs4_buf *gb);
static unsigned int gs4_buf_space_avail(struct gs4_buf *gb);
static unsigned int gs4_buf_put(struct gs4_buf *gb, const char *buf,
	unsigned int count);
static unsigned int gs4_buf_get(struct gs4_buf *gb, char *buf,
	unsigned int count);

/* external functions */
extern int net2280_set_fifo_mode(struct usb_gadget *gadget, int mode);

//int gs4_write_req_pending(struct tty_struct *tty) {
//	struct gs4_port *port = tty->driver_data;
//
//	if ( port == NULL ) {
//		printk(KERN_ERR "%s: NULL port pointer\n", __FUNCTION__);
//		return -1;
//	}
//	return port->port_dev->dev_write_req_pending;
//}

/* Globals */

static struct gs4_dev *gs4_device;

static const char *EP_IN_NAME[GS4_MAX_NUM_PORTS];
static const char *EP_OUT_NAME[GS4_MAX_NUM_PORTS];
static const char *EP_NOTIFY_NAME[GS4_MAX_NUM_PORTS];

static struct semaphore	gs4_open_close_sem[GS4_MAX_NUM_PORTS];

static unsigned int read_q_size = GS4_DEFAULT_READ_Q_SIZE;
static unsigned int write_q_size = GS4_DEFAULT_WRITE_Q_SIZE;

static unsigned int write_buf_size = GS4_DEFAULT_WRITE_BUF_SIZE;

static unsigned int use_acm = GS4_DEFAULT_USE_ACM;
static unsigned int use_qc = GS4_DEFAULT_USE_QC;

static unsigned int num_ports = GS4_DEFAULT_NUM_PORTS;

/* tty driver struct */
static const struct tty_operations gs4_tty_ops = {
	.open =			gs4_open,
	.close =		gs4_close,
	.write =		gs4_write,
	.put_char =		gs4_put_char,
	.flush_chars =		gs4_flush_chars,
	.write_room =		gs4_write_room,
	.ioctl =		gs4_ioctl,
	.set_termios =		gs4_set_termios,
	.throttle =		gs4_throttle,
	.unthrottle =		gs4_unthrottle,
	.break_ctl =		gs4_break,
	.chars_in_buffer =	gs4_chars_in_buffer,
};
static struct tty_driver *gs4_tty_driver;
static struct usb_gadget_strings gs4_string_table;

/* USB_FUNCTION */
struct usb_function gs4_usb_function = {
	.name		= GS4_LONG_NAME,
	.strings	= &gs4_string_table,
	.init		= gs4_init,
	.exit		= gs4_exit,
	.bind		= gs4_bind,
	.unbind		= gs4_unbind,
	.set_descriptors= gs4_set_descriptors,
	.setup		= gs4_setup,
	.disconnect	= gs4_disconnect,
//	.suspend	= gs4_suspend,
//	.resume		= gs4_resume,
};
EXPORT_SYMBOL(gs4_usb_function);

#ifndef USB_COMPOSITE_DEVICE
/* gadget driver struct */
static struct usb_gadget_driver gs4_gadget_driver = {
#ifdef CONFIG_USB_GADGET_DUALSPEED
	.speed =		USB_SPEED_HIGH,
#else
	.speed =		USB_SPEED_FULL,
#endif /* CONFIG_USB_GADGET_DUALSPEED */
	.function =		GS4_LONG_NAME,
	.bind =			gs4_bind,
	.unbind =		gs4_unbind,
	.setup =		gs4_setup,
	.disconnect =		gs4_disconnect,
	.driver = {
		.name =		GS4_SHORT_NAME,
		/* .shutdown = ... */
		/* .suspend = ...  */
		/* .resume = ...   */
	},
};
#endif /* USB_COMPOSITE_DEVICE */


/* USB descriptors */

#define GS4_CONTROL_STR_ID	11 /* REVISIT */
#define GS4_DATA_STR_ID		12
#define GS4_FUNCTION_STR_ID0	13
#define GS4_FUNCTION_STR_ID1	14
#define GS4_FUNCTION_STR_ID2	15
#define GS4_FUNCTION_STR_ID3	16

/* static strings, in UTF-8 */
static char manufacturer[50];
static struct usb_string gs4_strings[] = {
	{ GS4_CONTROL_STR_ID, "Serial Control" },
	{ GS4_DATA_STR_ID, "Serial Data" },
	{ GS4_FUNCTION_STR_ID0, "USB Serial Multiport Function0" },
	{ GS4_FUNCTION_STR_ID1, "USB Serial Multiport Function1" },
	{ GS4_FUNCTION_STR_ID2, "USB Serial Multiport Function2" },
	{ GS4_FUNCTION_STR_ID3, "USB Serial Multiport Function3" },
	{  } /* end of list */
};

static struct usb_gadget_strings gs4_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		gs4_strings,
};

////////////////////////////////////////////////////////////////////////////

/////////                     Device Descriptor                    /////////

#ifndef USB_COMPOSITE_DEVICE
/* if defined composite, use the device descriptor
 * from composite for all gadgets.
 */
static struct usb_device_descriptor gs4_device_desc = {
	.bLength =		USB_DT_DEVICE_SIZE,
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceSubClass =	2,
	.bDeviceProtocol =	1,
	.idVendor =		__constant_cpu_to_le16(GS4_VENDOR_ID),
	.idProduct =		__constant_cpu_to_le16(GS4_PRODUCT_ID),
	.iManufacturer =	GS4_MANUFACTURER_STR_ID,
	.iProduct =		GS4_PRODUCT_STR_ID,
	.iSerialNumber =	GS4_SERIAL_STR_ID,
	.bNumConfigurations =	GS4_NUM_CONFIGS,
};
#endif /* !USB_COMPOSITE_DEVICE */
////////////////////////////////////////////////////////////////////////////



static struct usb_otg_descriptor gs4_otg_descriptor = {
	.bLength =		sizeof(gs4_otg_descriptor),
	.bDescriptorType =	USB_DT_OTG,
	.bmAttributes =		USB_OTG_SRP,
};

static struct usb_interface_assoc_descriptor gs4_iad_descriptor[] = {
	{
	.bLength =		sizeof gs4_iad_descriptor[0],
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface =	0 + 0,	// composite interface will bump this
	.bInterfaceCount = 	2,	// control + data
	.bFunctionClass =	USB_CLASS_COMM, // 2
	.bFunctionSubClass =	USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	.iFunction =		GS4_FUNCTION_STR_ID0,
	}
	,{
	.bLength =		sizeof gs4_iad_descriptor[1],
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface =	0 + 2,	// composite interface will bump this
	.bInterfaceCount = 	2,	// control + data
	.bFunctionClass =	USB_CLASS_COMM, // 2
	.bFunctionSubClass =	USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	.iFunction =		GS4_FUNCTION_STR_ID1,
	}
	,{
	.bLength =		sizeof gs4_iad_descriptor[2],
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface =	0 + 4,	// composite interface will bump this
	.bInterfaceCount = 	2,	// control + data
	.bFunctionClass =	USB_CLASS_COMM, // 2
	.bFunctionSubClass =	USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	.iFunction =		GS4_FUNCTION_STR_ID2,
	}
	,{
	.bLength =		sizeof gs4_iad_descriptor[3],
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface =	0 + 6,	// composite interface will bump this
	.bInterfaceCount = 	2,	// control + data
	.bFunctionClass =	USB_CLASS_COMM, // 2
	.bFunctionSubClass =	USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	.iFunction =		GS4_FUNCTION_STR_ID3,
	}
};

#ifndef USB_COMPOSITE_DEVICE
/* if defined composite, use the config descriptor
 * from composite for all gadgets.
 */
static struct usb_config_descriptor gs4_bulk_config_desc = {
	.bLength =		USB_DT_CONFIG_SIZE,
	.bDescriptorType =	USB_DT_CONFIG,
	/* .wTotalLength computed dynamically */
	.bNumInterfaces =	GS4_MAX_NUM_PORTS,
	.bConfigurationValue =	GS4_BULK_CONFIG_ID,
	.iConfiguration =	GS4_BULK_CONFIG_STR_ID,
	.bmAttributes =		USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower =		1,
};

static struct usb_config_descriptor gs4_acm_config_desc = {
	.bLength =		USB_DT_CONFIG_SIZE,
	.bDescriptorType =	USB_DT_CONFIG,
	/* .wTotalLength computed dynamically */
	.bNumInterfaces =	 2 * GS4_MAX_NUM_PORTS,
	.bConfigurationValue =	GS4_ACM_CONFIG_ID,
	.iConfiguration =	GS4_ACM_CONFIG_STR_ID,
	.bmAttributes =		USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower =		1,
};

static struct usb_config_descriptor gs4_qc_config_desc = {
	.bLength =		USB_DT_CONFIG_SIZE,
	.bDescriptorType =	USB_DT_CONFIG,
	/* .wTotalLength computed dynamically */
	.bNumInterfaces =	GS4_MAX_NUM_PORTS,
	.bConfigurationValue =	GS4_QC_CONFIG_ID,
	.iConfiguration =	GS4_QC_CONFIG_STR_ID,
	.bmAttributes =		USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower =		1,
};
#endif /* !USB_COMPOSITE_DEVICE */

static struct usb_interface_descriptor gs4_bulk_interface_desc[] = {
	{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_BULK_INTERFACE_ID,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_BULK_INTERFACE_ID+1,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_BULK_INTERFACE_ID+2,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_BULK_INTERFACE_ID+3,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		GS4_DATA_STR_ID,
	}
};

static struct usb_interface_descriptor gs4_qc_interface_desc[] = {
	{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_QC_INTERFACE_ID,
	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_QC_INTERFACE_ID+1,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_QC_INTERFACE_ID+2,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_QC_INTERFACE_ID+3,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
	.iInterface =		GS4_DATA_STR_ID,
	}
};

static struct usb_interface_descriptor gs4_control_interface_desc[] = {
	{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_CONTROL_INTERFACE_ID + 0,
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM, // 2
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	.iInterface =		GS4_CONTROL_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_CONTROL_INTERFACE_ID + 2,
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM, // 2
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	.iInterface =		GS4_CONTROL_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_CONTROL_INTERFACE_ID + 4,
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM, // 2
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	.iInterface =		GS4_CONTROL_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_CONTROL_INTERFACE_ID + 6,
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM, // 2
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	.iInterface =		GS4_CONTROL_STR_ID,
	}
};

static struct usb_interface_descriptor gs4_data_interface_desc[] = {
	{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_DATA_INTERFACE_ID+0,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_DATA_INTERFACE_ID+2,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_DATA_INTERFACE_ID+4,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		GS4_DATA_STR_ID,
	}
	,{
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	GS4_DATA_INTERFACE_ID+6,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		GS4_DATA_STR_ID,
	}
};

static const struct usb_cdc_header_desc gs4_header_desc = {
	.bLength =		sizeof(gs4_header_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		__constant_cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor gs4_call_mgmt_descriptor[] = {
	{
	.bLength =  		sizeof(gs4_call_mgmt_descriptor[0]),
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubType = 	USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities = 	0,
	.bDataInterface = 	GS4_DATA_INTERFACE_ID + 0,
	}
	,{
	.bLength =  		sizeof(gs4_call_mgmt_descriptor[1]),
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubType = 	USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities = 	0,
	.bDataInterface = 	GS4_DATA_INTERFACE_ID + 2,
	}
	,{
	.bLength =  		sizeof(gs4_call_mgmt_descriptor[2]),
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubType = 	USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities = 	0,
	.bDataInterface = 	GS4_DATA_INTERFACE_ID + 4,
	}
	,{
	.bLength =  		sizeof(gs4_call_mgmt_descriptor[3]),
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubType = 	USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities = 	0,
	.bDataInterface = 	GS4_DATA_INTERFACE_ID + 6,
	}
};

static struct usb_cdc_acm_descriptor gs4_acm_descriptor = {
	.bLength =  		sizeof(gs4_acm_descriptor),
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubType = 	USB_CDC_ACM_TYPE,
	.bmCapabilities = 	0,
};

static struct usb_cdc_union_desc gs4_union_desc[] = {
	{
	.bLength =		sizeof gs4_union_desc[0],
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	.bMasterInterface0 =	GS4_CONTROL_INTERFACE_ID + 0,
	.bSlaveInterface0 =	GS4_CONTROL_INTERFACE_ID + 1,
	}
	,{
	.bLength =		sizeof gs4_union_desc[1],
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	.bMasterInterface0 =	GS4_CONTROL_INTERFACE_ID + 2,
	.bSlaveInterface0 =	GS4_CONTROL_INTERFACE_ID + 3,
	}
	,{
	.bLength =		sizeof gs4_union_desc[2],
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	.bMasterInterface0 =	GS4_CONTROL_INTERFACE_ID + 4,
	.bSlaveInterface0 =	GS4_CONTROL_INTERFACE_ID + 5,
	}
	,{
	.bLength =		sizeof gs4_union_desc[3],
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	.bMasterInterface0 =	GS4_CONTROL_INTERFACE_ID + 6,
	.bSlaveInterface0 =	GS4_CONTROL_INTERFACE_ID + 7,
	}
};

static struct usb_endpoint_descriptor gs4_fullspeed_notify_desc[] = {
	{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS4_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS4_LOG2_NOTIFY_INTERVAL,
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS4_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS4_LOG2_NOTIFY_INTERVAL,
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS4_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS4_LOG2_NOTIFY_INTERVAL,
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS4_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS4_LOG2_NOTIFY_INTERVAL,
	}
};

static struct usb_endpoint_descriptor gs4_fullspeed_in_desc[] = {
	{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	}
};

static struct usb_endpoint_descriptor gs4_fullspeed_out_desc[] = {
	{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	}
};

/////////////////////////////////////////////////////////////////////////////////////


static const struct usb_descriptor_header *gs4_bulk_fullspeed_function[] = {
	(struct usb_descriptor_header *) &gs4_otg_descriptor,
	(struct usb_descriptor_header *) &gs4_bulk_interface_desc[0],
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[0],
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[0],
	(struct usb_descriptor_header *) &gs4_bulk_interface_desc[1],
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[1],
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[1],
	(struct usb_descriptor_header *) &gs4_bulk_interface_desc[2],
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[2],
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[2],
	(struct usb_descriptor_header *) &gs4_bulk_interface_desc[3],
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[3],
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[3],
	NULL,
};

static const struct usb_descriptor_header *gs4_acm_fullspeed_function[] = {
	(struct usb_descriptor_header *) &gs4_otg_descriptor,
	(struct usb_descriptor_header *) &gs4_iad_descriptor[0],
	(struct usb_descriptor_header *) &gs4_control_interface_desc[0], // IF Desc
	(struct usb_descriptor_header *) &gs4_header_desc,
	(struct usb_descriptor_header *) &gs4_call_mgmt_descriptor[0],
	(struct usb_descriptor_header *) &gs4_acm_descriptor,
	(struct usb_descriptor_header *) &gs4_union_desc[0],
	(struct usb_descriptor_header *) &gs4_fullspeed_notify_desc[0],//  EP desc
	(struct usb_descriptor_header *) &gs4_data_interface_desc[0],    // IF desc
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[0],      //  EP desc
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[0],     //  EP desc
	(struct usb_descriptor_header *) &gs4_iad_descriptor[1],
	(struct usb_descriptor_header *) &gs4_control_interface_desc[1],// IF Desc
	(struct usb_descriptor_header *) &gs4_header_desc,
	(struct usb_descriptor_header *) &gs4_call_mgmt_descriptor[1],
	(struct usb_descriptor_header *) &gs4_acm_descriptor,
	(struct usb_descriptor_header *) &gs4_union_desc[1],
	(struct usb_descriptor_header *) &gs4_fullspeed_notify_desc[1],//  EP desc
	(struct usb_descriptor_header *) &gs4_data_interface_desc[1],    // IF desc
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[1],      //  EP desc
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[1],     //  EP desc
	(struct usb_descriptor_header *) &gs4_iad_descriptor[2],
	(struct usb_descriptor_header *) &gs4_control_interface_desc[2], // IF Desc
	(struct usb_descriptor_header *) &gs4_header_desc,
	(struct usb_descriptor_header *) &gs4_call_mgmt_descriptor[2],
	(struct usb_descriptor_header *) &gs4_acm_descriptor,
	(struct usb_descriptor_header *) &gs4_union_desc[2],
	(struct usb_descriptor_header *) &gs4_fullspeed_notify_desc[2],//  EP desc
	(struct usb_descriptor_header *) &gs4_data_interface_desc[2],    // IF desc
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[2],      //  EP desc
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[2],     //  EP desc
	(struct usb_descriptor_header *) &gs4_iad_descriptor[3],
	(struct usb_descriptor_header *) &gs4_control_interface_desc[3], // IF Desc
	(struct usb_descriptor_header *) &gs4_header_desc,
	(struct usb_descriptor_header *) &gs4_call_mgmt_descriptor[3],
	(struct usb_descriptor_header *) &gs4_acm_descriptor,
	(struct usb_descriptor_header *) &gs4_union_desc[3],
	(struct usb_descriptor_header *) &gs4_fullspeed_notify_desc[3],//  EP desc
	(struct usb_descriptor_header *) &gs4_data_interface_desc[3],    // IF desc
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[3],      //  EP desc
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[3],     //  EP desc
	NULL,
};

static const struct usb_descriptor_header *gs4_qc_fullspeed_function[] = {
	(struct usb_descriptor_header *) &gs4_otg_descriptor,
	(struct usb_descriptor_header *) &gs4_qc_interface_desc[0],
	(struct usb_descriptor_header *) &gs4_fullspeed_notify_desc[0],
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[0],
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[0],
	(struct usb_descriptor_header *) &gs4_qc_interface_desc[1],
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[1],
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[1],
	(struct usb_descriptor_header *) &gs4_qc_interface_desc[2],
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[2],
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[2],
	(struct usb_descriptor_header *) &gs4_qc_interface_desc[3],
	(struct usb_descriptor_header *) &gs4_fullspeed_in_desc[3],
	(struct usb_descriptor_header *) &gs4_fullspeed_out_desc[3],
	NULL,
};

#ifdef CONFIG_USB_GADGET_DUALSPEED
static struct usb_endpoint_descriptor gs4_highspeed_notify_desc[] = {
	{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS4_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS4_LOG2_NOTIFY_INTERVAL,
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS4_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS4_LOG2_NOTIFY_INTERVAL,
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS4_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS4_LOG2_NOTIFY_INTERVAL,
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS4_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS4_LOG2_NOTIFY_INTERVAL,
	}
};

static struct usb_endpoint_descriptor gs4_highspeed_in_desc[] = {
	{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	}
};

static struct usb_endpoint_descriptor gs4_highspeed_out_desc[] = {
	{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	}
	,{
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	}
};

#ifndef USB_COMPOSITE_DEVICE
static struct usb_qualifier_descriptor gs4_qualifier_desc = {
	.bLength =		sizeof(struct usb_qualifier_descriptor),
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,
	.bcdUSB =		__constant_cpu_to_le16 (0x0200),
	/* assumes ep0 uses the same value for both speeds ... */
	.bNumConfigurations =	GS4_NUM_CONFIGS,
};
#endif /* !USB_COMPOSITE_DEVICE */

static const struct usb_descriptor_header *gs4_bulk_highspeed_function[] = {
	(struct usb_descriptor_header *) &gs4_otg_descriptor,
	(struct usb_descriptor_header *) &gs4_bulk_interface_desc[0],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[0],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[0],
	(struct usb_descriptor_header *) &gs4_bulk_interface_desc[1],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[1],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[1],
	(struct usb_descriptor_header *) &gs4_bulk_interface_desc[2],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[2],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[2],
	(struct usb_descriptor_header *) &gs4_bulk_interface_desc[3],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[3],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[3],
	NULL,
};

static const struct usb_descriptor_header *gs4_acm_highspeed_function[] = {
	(struct usb_descriptor_header *) &gs4_otg_descriptor,
	(struct usb_descriptor_header *) &gs4_iad_descriptor[0],
	(struct usb_descriptor_header *) &gs4_control_interface_desc[0],
	(struct usb_descriptor_header *) &gs4_header_desc,
	(struct usb_descriptor_header *) &gs4_call_mgmt_descriptor[0],
	(struct usb_descriptor_header *) &gs4_acm_descriptor,
	(struct usb_descriptor_header *) &gs4_union_desc[0],
	(struct usb_descriptor_header *) &gs4_highspeed_notify_desc[0],
	(struct usb_descriptor_header *) &gs4_data_interface_desc[0],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[0],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[0],
	(struct usb_descriptor_header *) &gs4_iad_descriptor[1],
	(struct usb_descriptor_header *) &gs4_control_interface_desc[1],
	(struct usb_descriptor_header *) &gs4_header_desc,
	(struct usb_descriptor_header *) &gs4_call_mgmt_descriptor[1],
	(struct usb_descriptor_header *) &gs4_acm_descriptor,
	(struct usb_descriptor_header *) &gs4_union_desc[1],
	(struct usb_descriptor_header *) &gs4_highspeed_notify_desc[1],
	(struct usb_descriptor_header *) &gs4_data_interface_desc[1],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[1],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[1],
	(struct usb_descriptor_header *) &gs4_iad_descriptor[2],
	(struct usb_descriptor_header *) &gs4_control_interface_desc[2],
	(struct usb_descriptor_header *) &gs4_header_desc,
	(struct usb_descriptor_header *) &gs4_call_mgmt_descriptor[2],
	(struct usb_descriptor_header *) &gs4_acm_descriptor,
	(struct usb_descriptor_header *) &gs4_union_desc[2],
	(struct usb_descriptor_header *) &gs4_highspeed_notify_desc[2],
	(struct usb_descriptor_header *) &gs4_data_interface_desc[2],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[2],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[2],
	(struct usb_descriptor_header *) &gs4_iad_descriptor[3],
	(struct usb_descriptor_header *) &gs4_control_interface_desc[3],
	(struct usb_descriptor_header *) &gs4_header_desc,
	(struct usb_descriptor_header *) &gs4_call_mgmt_descriptor[3],
	(struct usb_descriptor_header *) &gs4_acm_descriptor,
	(struct usb_descriptor_header *) &gs4_union_desc[3],
	(struct usb_descriptor_header *) &gs4_highspeed_notify_desc[3],
	(struct usb_descriptor_header *) &gs4_data_interface_desc[3],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[3],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[3],
	NULL,
};

static const struct usb_descriptor_header *gs4_qc_highspeed_function[] = {
	(struct usb_descriptor_header *) &gs4_otg_descriptor,
	(struct usb_descriptor_header *) &gs4_qc_interface_desc[0],
	(struct usb_descriptor_header *) &gs4_highspeed_notify_desc[0],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[0],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[0],
	(struct usb_descriptor_header *) &gs4_qc_interface_desc[1],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[1],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[1],
	(struct usb_descriptor_header *) &gs4_qc_interface_desc[2],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[2],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[2],
	(struct usb_descriptor_header *) &gs4_qc_interface_desc[3],
	(struct usb_descriptor_header *) &gs4_highspeed_in_desc[3],
	(struct usb_descriptor_header *) &gs4_highspeed_out_desc[3],
	NULL,
};
#endif /* CONFIG_USB_GADGET_DUALSPEED */


#ifdef USB_COMPOSITE_DEVICE
module_param(num_ports, uint, S_IRUGO);
MODULE_PARM_DESC(num_ports, "number of serial ports, default=2");
#else
/* Module */
MODULE_DESCRIPTION(GS4_LONG_NAME);
MODULE_AUTHOR("Al Borchers, Martin Rogers, Toshi Kikuchi");
MODULE_LICENSE("GPL");

module_param(num_ports, uint, S_IRUGO);
MODULE_PARM_DESC(num_ports, "number of serial ports, default=2");

module_param(read_q_size, uint, S_IRUGO);
MODULE_PARM_DESC(read_q_size, "Read request queue size, default=32");

module_param(write_q_size, uint, S_IRUGO);
MODULE_PARM_DESC(write_q_size, "Write request queue size, default=32");

module_param(write_buf_size, uint, S_IRUGO);
MODULE_PARM_DESC(write_buf_size, "Write buffer size, default=8192");

module_param(use_acm, uint, S_IRUGO);
MODULE_PARM_DESC(use_acm, "Use CDC ACM, 0=no, 1=yes, default=no");

module_param(use_qc, uint, S_IRUGO);
MODULE_PARM_DESC(use_qc, "Use Qualcomm modem emulation, 0=no, 1=yes, default=yes");

module_init(gs4_module_init);
module_exit(gs4_module_exit);
#endif

//#ifdef CONFIG_PM_DEVICE_LOCKOUT
//
//static wait_queue_head_t gs4_suspend_wq;
//
//static int gs4_lockout(void)
//{
//	if (gs4_device->suspended) {
//		int err;
//		err = wait_event_interruptible(gs4_suspend_wq,
//				       gs4_device->suspended == 0);
//		if (err < 0)
//			return err;
//	}
//	return 0;
//}
//#else
//static inline int gs4_lockout(void)
//{	return 0;
//}
//#endif

/*
*  gs4_module_init
*
*  Register as a USB gadget driver and a tty driver.
*/
static int gs4_init(void)
{
	int i;
	int retval;

	if (num_ports > GS4_MAX_NUM_PORTS) {
		printk(KERN_ERR "numports too big\n");
		return 1;
	}
	
#ifndef USB_COMPOSITE_DEVICE
	gs4_bulk_config_desc.bNumInterfaces = num_ports;
	gs4_acm_config_desc.bNumInterfaces = 2 * num_ports;
	gs4_qc_config_desc.bNumInterfaces = num_ports;
#else
	gs4_strings[num_ports+7].id=0;
	gs4_strings[num_ports+7].s=NULL;
#endif

	gs4_bulk_fullspeed_function[num_ports * 3 + 1] = NULL;
	gs4_acm_fullspeed_function[num_ports * 10 + 1] = NULL;
	gs4_qc_fullspeed_function[4 + (num_ports - 1) * 3 + 1] = NULL;

#ifdef CONFIG_USB_GADGET_DUALSPEED
	gs4_bulk_highspeed_function[num_ports * 3 + 1] = NULL;
	gs4_acm_highspeed_function[num_ports * 10 + 1] = NULL;
	gs4_qc_highspeed_function[4 + (num_ports - 1) * 3 + 1] = NULL;
#endif

#ifndef USB_COMPOSITE_DEVICE
	retval = usb_gadget_register_driver(&gs4_gadget_driver);
	if (retval) {
		printk(KERN_ERR "gs4_module_init: cannot register gadget"
            " driver, ret=%d\n", retval);
		return retval;
	}
#endif /* USB_COMPOSITE_DEVICE */

	gs4_tty_driver = alloc_tty_driver(num_ports);
	if (!gs4_tty_driver)
		return -ENOMEM;
	gs4_tty_driver->owner = THIS_MODULE;
	gs4_tty_driver->driver_name = GS4_SHORT_NAME;
	gs4_tty_driver->name = "ttygs";
	gs4_tty_driver->major = GS4_MAJOR;
	gs4_tty_driver->minor_start = GS4_MINOR_START;
	gs4_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	gs4_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	gs4_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	gs4_tty_driver->init_termios = tty_std_termios;
	gs4_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(gs4_tty_driver, &gs4_tty_ops);

	for (i=0; i < num_ports; i++)
		sema_init(&gs4_open_close_sem[i], 1);

	retval = tty_register_driver(gs4_tty_driver);
	if (retval) {
#ifndef USB_COMPOSITE_DEVICE
		usb_gadget_unregister_driver(&gs4_gadget_driver);
#endif /* USB_COMPOSITE_DEVICE */
		put_tty_driver(gs4_tty_driver);
		printk(KERN_ERR "gs4_module_init: cannot register tty driver,"
            " ret=%d\n", retval);
		return retval;
	}
//#ifdef CONFIG_PM_DEVICE_LOCKOUT
//	init_waitqueue_head(&gs4_suspend_wq);
//#endif

	printk(KERN_INFO "gs4_module_init: %s %s loaded; num ports=%d\n",
        	GS4_LONG_NAME, GS4_VERSION_STR, num_ports);
	return 0;
}

/*
* gs4_module_exit
*
* Unregister as a tty driver and a USB gadget driver.
*/
static void gs4_exit(void)
{
	tty_unregister_driver(gs4_tty_driver);
	put_tty_driver(gs4_tty_driver);
#ifndef USB_COMPOSITE_DEVICE
	usb_gadget_unregister_driver(&gs4_gadget_driver);
#endif /* USB_COMPOSITE_DEVICE */

	printk(KERN_INFO "gs4_module_exit: %s %s unloaded\n", GS4_LONG_NAME, GS4_VERSION_STR);
}

/* TTY Driver */

#if 0
static int is_config_set(struct gs4_dev *dev)
{
	int config;
	unsigned long flags;

	spin_lock_irqsave(&dev->dev_lock, flags);
	config = dev->dev_config;
	spin_unlock_irqrestore(&dev->dev_lock, flags);

	return (config != GS4_NO_CONFIG_ID);
}
#endif

/*
 * gs4_open
 */
static int gs4_open(struct tty_struct *tty, struct file *file)
{
	int port_num;
	unsigned long flags;
	struct gs4_port *port;
	struct gs4_dev *dev;
	struct gs4_buf *buf;
	struct semaphore *sem;
	int ret;

	port_num = tty->index;

	dprintk("port %d,tty %p,file %p\n", port_num, tty, file);

	if (port_num < 0 || port_num >= num_ports) {
		printk(KERN_ERR "gs4_open: (%d,%p,%p) invalid port number\n",
			port_num, tty, file);
		return -ENODEV;
	}

	/* block process if suspended */
	//if ((ret = gs4_lockout())) {
	//	dprintk("suspend lockout returned %i\n", ret);
	//	return ret;
	//}

	dev = gs4_device;

	if (dev == NULL) {
		printk(KERN_ERR "gs4_open: (%d,%p,%p) NULL device pointer\n",
			port_num, tty, file);
		return -ENODEV;
	}

	sem = &gs4_open_close_sem[port_num];
	if (down_interruptible(sem)) {
		printk(KERN_ERR
		"gs4_open: (%d,%p,%p) interrupted waiting for semaphore\n",
			port_num, tty, file);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&dev->dev_lock, flags);

	if (dev->dev_config == GS4_NO_CONFIG_ID) {
		printk(KERN_DEBUG
			"gs4_open: (%d,%p,%p) device is not connected\n",
			port_num, tty, file);
		ret = -ENODEV;
		goto exit_unlock_dev;
	}

	port = dev->dev_port[port_num];

	if (port == NULL) {
		printk(KERN_DEBUG "gs4_open: (%d,%p,%p) NULL port pointer\n",
			port_num, tty, file);
		ret = -ENODEV;
		goto exit_unlock_dev;
	}

	spin_lock(&port->port_lock);
	spin_unlock(&dev->dev_lock);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "gs4_open: (%d,%p,%p) port disconnected (1)\n",
			port_num, tty, file);
		ret = -EIO;
		goto exit_unlock_port;
	}

	if (port->port_open_count > 0) {
		++port->port_open_count;
		dprintk("%d,%p,%p already open\n",
			port_num, tty, file);
		ret = 0;
		goto exit_unlock_port;
	}

	tty->driver_data = NULL;
	tty->low_latency = 1;

	/* mark port as in use, we can drop port lock and sleep if necessary */
	port->port_in_use = 1;

	/* allocate write buffer on first open */
	if (port->port_write_buf == NULL) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		buf = gs4_buf_alloc(write_buf_size, GFP_KERNEL);
		spin_lock_irqsave(&port->port_lock, flags);

		/* might have been disconnected while asleep, check */
		if (port->port_dev == NULL) {
			printk(KERN_ERR
				"gs4_open: (%d,%p,%p) port disconnected (2)\n",
				port_num, tty, file);
			port->port_in_use = 0;
			ret = -EIO;
			goto exit_unlock_port;
		}

		if ((port->port_write_buf=buf) == NULL) {
			printk(KERN_ERR "gs4_open: (%d,%p,%p) cannot allocate port write buffer\n",
				port_num, tty, file);
			port->port_in_use = 0;
			ret = -ENOMEM;
			goto exit_unlock_port;
		}

	}

	/* wait for carrier detect (not implemented) */

	/* might have been disconnected while asleep, check */
	if (port->port_dev == NULL) {
		printk(KERN_ERR "gs4_open: (%d,%p,%p) port disconnected (3)\n",
			port_num, tty, file);
		port->port_in_use = 0;
		ret = -EIO;
		goto exit_unlock_port;
	}

	tty->driver_data = port;
	port->port_tty = tty;
	port->port_open_count = 1;
	port->port_in_use = 0;

	dprintk("%d,%p,%p completed\n", port_num, tty, file);

	ret = 0;

exit_unlock_port:
	spin_unlock_irqrestore(&port->port_lock, flags);
	up(sem);
	return ret;

exit_unlock_dev:
	spin_unlock_irqrestore(&dev->dev_lock, flags);
	up(sem);
	return ret;

}

/*
 * gs4_close
 */

#define GS4_WRITE_FINISHED_EVENT_SAFELY(p)			\
({								\
	int cond;						\
	unsigned long flags;					\
								\
	spin_lock_irqsave(&(p)->port_lock, flags);		\
	cond = !(p)->port_dev || !gs4_buf_data_avail((p)->port_write_buf); \
	spin_unlock_irqrestore(&(p)->port_lock, flags);		\
	cond;							\
})

static void gs4_close(struct tty_struct *tty, struct file *file)
{
	struct gs4_port *port = tty->driver_data;
	struct semaphore *sem;
	//int ret;
	unsigned long flags;
	int i;

	if (port == NULL) {
		printk(KERN_DEBUG "gs4_close: NULL port pointer\n");
		return;
	}

	dprintk("port %d,tty %p, tty->index %d\n", port->port_num, tty, tty->index);

	/* block process if suspended */
	//if ((ret = gs4_lockout())) {
	//	dprintk("suspend lockout returned %i\n", ret);
	//	return;
	//}

	sem = &gs4_open_close_sem[port->port_num];
	down(sem);

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_open_count == 0) {
		printk(KERN_ERR
			"gs4_close: (%d,%p,%p) port is already closed\n",
			port->port_num, tty, file);
		goto exit;
	}

	if (port->port_open_count > 1) {
		--port->port_open_count;
		goto exit;
	}

	/* free disconnected port on final close */
	if (port->port_dev == NULL) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		kfree(port);
		goto exit_sem;
	}

	/* mark port as closed but in use, we can drop port lock */
	/* and sleep if necessary */
	port->port_in_use = 1;
	port->port_open_count = 0;

	/* wait for write buffer to drain, or */
	/* at most GS4_CLOSE_TIMEOUT seconds */
	if (gs4_buf_data_avail(port->port_write_buf) > 0) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		wait_event_interruptible_timeout(port->port_write_wait,
					GS4_WRITE_FINISHED_EVENT_SAFELY(port),
					GS4_CLOSE_TIMEOUT * HZ);
		spin_lock_irqsave(&port->port_lock, flags);
	}

	spin_unlock_irqrestore(&port->port_lock, flags);
	INIT_LIST_HEAD(&port->port_spare_ctlreq_entries);
	INIT_LIST_HEAD(&port->port_filled_ctlreq_entries);
	for (i = 0; i < GS4_NCTLREQ; i++)
		list_add(&(port->port_ctlreq_entry[i].list), &port->port_spare_ctlreq_entries);
	spin_lock_irqsave(&port->port_lock, flags);

	/* free disconnected port on final close */
	/* (might have happened during the above sleep) */
	if (port->port_dev == NULL) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		kfree(port);
		goto exit_sem;
	}

	gs4_buf_clear(port->port_write_buf);

	tty->driver_data = NULL;
	port->port_tty = NULL;
	port->port_in_use = 0;

	dprintk("%d,%p,%p completed\n", port->port_num, tty, file);

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
exit_sem:
	up(sem);
}

/*
 * gs4_write
 */
static int gs4_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	unsigned long flags;
	struct gs4_port *port = tty->driver_data;
	int ret = 0;

	if (port == NULL) {
		printk(KERN_ERR "gs4_write: NULL port pointer\n");
		return -EIO;
	}

	//dprintk("%d,%p writing %d bytes\n", port->port_num, tty, count);

	/* block process if suspended */
	//if ((ret = gs4_lockout())) {
	//	dprintk("suspend lockout returned %i\n", ret);
	//	return ret;
	//}

	if (count == 0)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "gs4_write: (%d,%p) port is not connected\n",
			port->port_num, tty);
		ret = -EIO;
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "gs4_write: (%d,%p) port is closed\n",
			port->port_num, tty);
		ret = -EBADF;
		goto exit;
	}

	count = gs4_buf_put(port->port_write_buf, buf, count);

	spin_unlock_irqrestore(&port->port_lock, flags);

	gs4_send(gs4_device, port->port_num);

	//dprintk("%d,%p wrote %d bytes\n", port->port_num, tty, count);

	return count;

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
	return ret;
}

/*
 * gs4_put_char
 */
static void gs4_put_char(struct tty_struct *tty, unsigned char ch)
{
	unsigned long flags;
	struct gs4_port *port = tty->driver_data;
	//int ret;

	if (port == NULL) {
		printk(KERN_ERR "gs4_put_char: NULL port pointer\n");
		return;
	}

	dprintk("%d,%p char=0x%x, called from %p\n", port->port_num, tty, ch, __builtin_return_address(0));

	/* block process if suspended */
	//if ((ret = gs4_lockout())) {
	//	dprintk("suspend lockout returned %i\n", ret);
	//	return;
	//}

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "gs4_put_char: (%d,%p) port is not connected\n",
			port->port_num, tty);
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "gs4_put_char: (%d,%p) port is closed\n",
			port->port_num, tty);
		goto exit;
	}

	gs4_buf_put(port->port_write_buf, &ch, 1);

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * gs4_flush_chars
 */
static void gs4_flush_chars(struct tty_struct *tty)
{
	unsigned long flags;
	struct gs4_port *port = tty->driver_data;
	//int ret;

	if (port == NULL) {
		printk(KERN_ERR "gs4_flush_chars: NULL port pointer\n");
		return;
	}

	dprintk("%d,%p\n", port->port_num, tty);

	/* block process if suspended */
	//if ((ret = gs4_lockout())) {
	//	dprintk("suspend lockout returned %i\n", ret);
	//	return;
	//}

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR
			"gs4_flush_chars: (%d,%p) port is not connected\n",
			port->port_num, tty);
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "gs4_flush_chars: (%d,%p) port is closed\n",
			port->port_num, tty);
		goto exit;
	}

	spin_unlock_irqrestore(&port->port_lock, flags);

	gs4_send(gs4_device, port->port_num);

	return;

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * gs4_write_room
 */
static int gs4_write_room(struct tty_struct *tty)
{

	int room = 0;
	unsigned long flags;
	struct gs4_port *port = tty->driver_data;


	if (port == NULL)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev != NULL && port->port_open_count > 0
	&& port->port_write_buf != NULL)
		room = gs4_buf_space_avail(port->port_write_buf);

	spin_unlock_irqrestore(&port->port_lock, flags);

	//dprintk("%d,%p room=%d\n", port->port_num, tty, room);

	return room;
}

/*
 * gs4_chars_in_buffer
 */
static int gs4_chars_in_buffer(struct tty_struct *tty)
{
	int chars = 0;
	unsigned long flags;
	struct gs4_port *port = tty->driver_data;

	if (port == NULL)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev != NULL && port->port_open_count > 0
	&& port->port_write_buf != NULL)
		chars = gs4_buf_data_avail(port->port_write_buf);

	spin_unlock_irqrestore(&port->port_lock, flags);

	//dprintk("%d,%p chars=%d\n", port->port_num, tty, chars);

	return chars;
}

/*
 * gs4_throttle
 */
static void gs4_throttle(struct tty_struct *tty)
{
}

/*
 * gs4_unthrottle
 */
static void gs4_unthrottle(struct tty_struct *tty)
{
}

/*
 * gs4_break
 */
static void gs4_break(struct tty_struct *tty, int break_state)
{
}

/*
 * gs4_ioctl
 */
static int gs4_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct gs4_port *port = tty->driver_data;
	int ret = 0;
	void __user *uarg = (void __user *)arg;

	if (port == NULL) {
		printk(KERN_ERR "gs4_ioctl: NULL port pointer\n");
		return -EIO;
	}

	if (port->port_open_count == 0) {
		printk(KERN_DEBUG "gs4_ioctl: port=%d, port is closed\n",
			port->port_num);
		return -EIO;
	}

	dprintk("%d,%p,%p cmd=0x%4.4x, arg=%lu\n", port->port_num, tty, file, cmd, arg);

	/* block process if suspended */
	//if ((ret = gs4_lockout())) {
	//	dprintk("suspend lockout returned %i\n", ret);
	//	return ret;
	//}

	/* handle ioctls */
	switch (cmd) {
	case PIOCSENDNOTIF:
		ret = gs4_ioc_send_notification(port, uarg);
		break;
	case PIOCRECVCTLREQ:
		ret = gs4_ioc_recv_ctrl_request(port, uarg);
		break;
	default:
		ret = -ENOIOCTLCMD;	/* could not handle ioctl */
	}
	return ret;
}

/*
 * gs4_set_termios
 */
static void gs4_set_termios(struct tty_struct *tty, struct ktermios *old)
{
}

/*
 * gs4_ioc_send_notification
 */
static int gs4_ioc_send_notification(struct gs4_port *port,
				     struct pioc_cdc_notification __user *u_notif)
{
	struct gs4_dev *dev;
	struct usb_request *req;
	struct pioc_cdc_notification notif;
	u16 wIndex;
	int retval;
	unsigned long flags;

	if (copy_from_user(&notif, u_notif, sizeof(notif)))
		return -EFAULT;

	if (port == NULL) {
		printk(KERN_ERR "%s: NULL port pointer\n", __FUNCTION__);
		return -EINVAL;
	}

	if (port->port_dev == NULL) {
		printk(KERN_INFO "%s: port->port_dev == NULL\n", __func__);
		return -EIO;
	}

	dev = port->port_dev;
	req = dev->dev_notify_req;

	if (!use_qc) {
		printk("%s: only works with use_qc\n", __FUNCTION__);
		return -EINVAL;
	}
	wIndex = gs4_qc_interface_desc[port->port_num].bInterfaceNumber;

	if (req == NULL) {
		if (dev->dev_notify_ep[0] == 0) {
			printk(KERN_ERR "%s: no notify ep\n", __FUNCTION__);
			return -EINVAL;
		}
		req = gs4_alloc_req(dev->dev_notify_ep[0], sizeof(notif), GFP_KERNEL);
		if (req == NULL) {
			printk(KERN_ERR "%s: can't allocalte req\n", __FUNCTION__);
			return -ENOMEM;
		}
		dev->dev_notify_req = req;
		dev->dev_notify_req_avail = 1;
		init_waitqueue_head(&dev->dev_notify_req_wait);
	}

	notif.wValue = __constant_cpu_to_le16(notif.wValue);
	notif.wIndex = __constant_cpu_to_le16(wIndex);
	notif.wLength = __constant_cpu_to_le16(notif.wLength);

	memcpy(req->buf, &notif, sizeof(notif));
	req->length = sizeof(notif) - PIOC_NOTIF_DATA_SIZE + notif.wLength;
	req->complete = gs4_ioc_send_complete;
	req->context = dev;

	spin_lock_irqsave(&dev->dev_lock, flags);
	if (!dev->dev_notify_req_avail) {
		spin_unlock_irqrestore(&dev->dev_lock, flags);
		printk(KERN_ERR "%s: dev_notify_req is not available\n", __func__);
		return -EINVAL;
	}
	retval = usb_ep_queue (dev->dev_notify_ep[0], req, GFP_ATOMIC); /* REVISIT: assuming intf 0 */
	if (retval < 0) {
		spin_unlock_irqrestore(&dev->dev_lock, flags);
		printk(KERN_ERR "%s: notify queue --> %d\n", __FUNCTION__, retval);
		return retval;
	}
	dev->dev_notify_req_avail = 0;
	spin_unlock_irqrestore(&dev->dev_lock, flags);

	printk(KERN_DEBUG "%s: type: 0x%02x val: %#x idx: %d len: %#x result: %d\n",
	       __func__, notif.bNotificationType, notif.wValue, notif.wIndex, notif.wLength, retval);

	retval = wait_event_interruptible(dev->dev_notify_req_wait, dev->dev_notify_req_avail > 0);
	if (retval < 0)
		printk(KERN_ERR "%s: wait_event returned %d\n", __func__, retval);

	return retval < 0 ? retval : 0;
}

static void gs4_ioc_send_complete (struct usb_ep *ep, struct usb_request *req)
{
	struct gs4_dev *dev = ep->driver_data;

	if (req->status < 0)
		printk(KERN_ERR "%s: req->status=%d\n", __func__, req->status);

	spin_lock(&dev->dev_lock);
	dev->dev_notify_req_avail = 1;
	spin_unlock(&dev->dev_lock);
	wake_up_interruptible(&dev->dev_notify_req_wait);
}

/*
 * gs4_ioc_recv_ctrl_request
 */
static int gs4_ioc_recv_ctrl_request(struct gs4_port *port, struct pioc_cdc_control_request __user *u_req)
{
	DECLARE_WAITQUEUE(wait, current);
	int ret = 0;
	unsigned long flags;
	struct gs4_ctlreq_entry *ctlreq_entry;

	if (port == NULL) {
		printk(KERN_ERR "%s: NULL port pointer\n", __FUNCTION__);
		return -EINVAL;
	}

	add_wait_queue(&port->port_ctlreq_wait, &wait);

	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_irqsave(&port->port_lock, flags);
		if (!list_empty(&port->port_filled_ctlreq_entries)) {
			spin_unlock_irqrestore(&port->port_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&port->port_lock, flags);

		schedule();

		spin_lock_irqsave(&port->port_lock, flags);
		if (port->port_dev == NULL) { /* this port is being freed */
			spin_unlock_irqrestore(&port->port_lock, flags);
			printk(KERN_INFO "%s: port->port_dev == NULL\n", __func__);
			ret = -EIO;
			break;
		}
		spin_unlock_irqrestore(&port->port_lock, flags);

		/* see if a signal did it */
		if (signal_pending(current)) {
			printk(KERN_INFO "%s: got a signal\n", __func__);
			ret = -EINTR;
			break;
		}
	}

	current->state = TASK_RUNNING;
	remove_wait_queue(&port->port_ctlreq_wait, &wait);

	if (ret < 0)
		return ret;

	spin_lock_irqsave(&port->port_lock, flags);
	ctlreq_entry = list_entry(port->port_filled_ctlreq_entries.next, struct gs4_ctlreq_entry, list);
	list_del(&ctlreq_entry->list);
	spin_unlock_irqrestore(&port->port_lock, flags);

	ret = copy_to_user(u_req, &ctlreq_entry->ctlreq, sizeof(*u_req));
	if (ret < 0)
		ret = -EFAULT; /* fall through */

	spin_lock_irqsave(&port->port_lock, flags);
	list_add_tail(&ctlreq_entry->list, &port->port_spare_ctlreq_entries);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return ret;
}

/*
* gs4_send
*
* This function finds available write requests, calls
* gs4_send_packet to fill these packets with data, and
* continues until either there are no more write requests
* available or no more data to send.  This function is
* run whenever data arrives or write requests are available.
*/
static int gs4_send(struct gs4_dev *dev, int port_num)
{
	int ret,len;
	unsigned long flags;
	struct usb_ep *ep;
	struct usb_request *req;
	struct gs4_req_entry *req_entry;

	//dprintk("%d,%p\n", port_num, dev);

	if (dev == NULL) {
		printk(KERN_ERR "gs4_send: NULL device pointer\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&dev->dev_lock, flags);

	ep = dev->dev_in_ep[port_num];

	while(!list_empty(&dev->dev_req_list[port_num])) {

		req_entry = list_entry(dev->dev_req_list[port_num].next,
			struct gs4_req_entry, re_entry);

		req = req_entry->re_req;

		len = gs4_send_packet(dev, req->buf, ep->maxpacket, port_num);

		if (len > 0) {
			list_del(&req_entry->re_entry);
			req->length = len;
//			dev->dev_write_req_pending++;
			spin_unlock_irqrestore(&dev->dev_lock, flags);
			if ((ret=usb_ep_queue(ep, req, GFP_ATOMIC))) {
				printk(KERN_ERR
				"gs4_send: cannot queue read request, ret=%d\n",
					ret);
				spin_lock_irqsave(&dev->dev_lock, flags);
				break;
			}
			spin_lock_irqsave(&dev->dev_lock, flags);
		} else {
			break;
		}

	}

	spin_unlock_irqrestore(&dev->dev_lock, flags);

	return 0;
}

/*
 * gs4_send_packet
 *
 * If there is data to send, a packet is built in the given
 * buffer and the size is returned.  If there is no data to
 * send, 0 is returned.  If there is any error a negative
 * error number is returned.
 *
 * Called during USB completion routine, on interrupt time.
 *
 * We assume that disconnect will not happen until all completion
 * routines have completed, so we can assume that the dev_port
 * array does not change during the lifetime of this function.
 */
static int gs4_send_packet(struct gs4_dev *dev, char *packet, unsigned int size, int port_num)
{
	unsigned int len;
	struct gs4_port *port;

	port = dev->dev_port[port_num];

	if (port == NULL) {
		printk(KERN_ERR
			"gs4_send_packet: port=%d, NULL port pointer\n",
			0);
		return -EIO;
	}

	spin_lock(&port->port_lock);

	len = gs4_buf_data_avail(port->port_write_buf);
	if (len < size)
		size = len;

	if (size == 0)
		goto exit;

	size = gs4_buf_get(port->port_write_buf, packet, size);

	if (port->port_tty)
		wake_up_interruptible(&port->port_tty->write_wait);

exit:
	spin_unlock(&port->port_lock);
	//dprintk("%d,%p %d bytes\n", port_num, dev, size);
	return size;
}

/*
 * gs4_recv_packet
 *
 * Called for each USB packet received.  Reads the packet
 * header and stuffs the data in the appropriate tty buffer.
 * Returns 0 if successful, or a negative error number.
 *
 * Called during USB completion routine, on interrupt time.
 *
 * We assume that disconnect will not happen until all completion
 * routines have completed, so we can assume that the dev_port
 * array does not change during the lifetime of this function.
 */
static int gs4_recv_packet(struct gs4_dev *dev, char *packet, unsigned int size, int port_num)
{
	unsigned int len;
	struct gs4_port *port;
	int ret;

	port = dev->dev_port[port_num];

	if (port == NULL) {
		printk(KERN_ERR "gs4_recv_packet: port=%d, NULL port pointer\n",
			port_num);
		return -EIO;
	}

	spin_lock(&port->port_lock);

	if (port->port_open_count == 0) {
		printk(KERN_DEBUG "gs4_recv_packet: port=%d, port is closed\n",
			port->port_num);
		ret = -EIO;
		goto exit;
	}

	if (port->port_tty == NULL) {
		printk(KERN_ERR "gs4_recv_packet: port=%d, NULL tty pointer\n",
			port->port_num);
		ret = -EIO;
		goto exit;
	}

	if (port->port_tty->magic != TTY_MAGIC) {
		printk(KERN_ERR "gs4_recv_packet: port=%d, bad tty magic\n",
			port->port_num);
		ret = -EIO;
		goto exit;
	}

	len = tty_buffer_request_room(port->port_tty, size);
	if (len > 0) {
		tty_insert_flip_string(port->port_tty, packet, len);
		tty_flip_buffer_push(port->port_tty);
		wake_up_interruptible(&port->port_tty->read_wait);
	}
	ret = 0;

exit:
	spin_unlock(&port->port_lock);
	return ret;
}

/*
* gs4_read_complete
*/
static void gs4_read_complete(struct usb_ep *ep, struct usb_request *req, int port_num)
{
	int ret;
	struct gs4_dev *dev = ep->driver_data;

	if (dev == NULL) {
		printk(KERN_ERR "gs4_read_complete%d: NULL device pointer\n", port_num);
		return;
	}

	switch(req->status) {
	case 0:
 		/* normal completion */
		gs4_recv_packet(dev, req->buf, req->actual, port_num);
requeue:
		req->length = ep->maxpacket;
		if ((ret=usb_ep_queue(ep, req, GFP_ATOMIC))) {
			printk(KERN_ERR
			"gs4_read_complete%d: cannot queue read request, ret=%d\n",
				port_num, ret);
		}
		break;

	case -ESHUTDOWN:
		/* disconnect */
		//dprintk("%d: shutdown\n", port_num);
		gs4_free_req(ep, req);
		break;

	default:
		/* unexpected */
		printk(KERN_ERR
		"gs4_read_complete%d: unexpected status error, status=%d\n",
			port_num, req->status);
		goto requeue;
		break;
	}
}

static void gs4_read_complete0(struct usb_ep *ep, struct usb_request *req)
{
	gs4_read_complete(ep, req, 0);
}
static void gs4_read_complete1(struct usb_ep *ep, struct usb_request *req)
{
	gs4_read_complete(ep, req, 1);
}
static void gs4_read_complete2(struct usb_ep *ep, struct usb_request *req)
{
	gs4_read_complete(ep, req, 2);
}
static void gs4_read_complete3(struct usb_ep *ep, struct usb_request *req)
{
	gs4_read_complete(ep, req, 3);
}

/*
* gs4_write_complete
*/
static void gs4_write_complete(struct usb_ep *ep, struct usb_request *req,
				int port_num)
{
	struct gs4_dev *dev = ep->driver_data;
	struct gs4_req_entry *gs4_req = req->context;

	if (dev == NULL) {
		printk(KERN_ERR "gs4_write_complete%d: NULL device pointer\n",
			port_num);
		return;
	}

	switch(req->status) {
	case 0:
		/* normal completion */
requeue:
		if (gs4_req == NULL) {
			printk(KERN_ERR
				"gs4_write_complete%d: NULL request pointer\n",
				port_num);
			return;
		}

		spin_lock(&dev->dev_lock);
		list_add(&gs4_req->re_entry, &dev->dev_req_list[port_num]);
		//dev->dev_write_req_pending--;
		spin_unlock(&dev->dev_lock);

		gs4_send(dev, port_num);

		break;

	case -ESHUTDOWN:
		/* disconnect */
		//dprintk("%d: shutdown\n", port_num);
		gs4_free_req(ep, req);
		break;

	default:
		printk(KERN_ERR
		"gs4_write_complete%d: unexpected status error, status=%d\n",
			port_num, req->status);
		goto requeue;
		break;
	}
}

static void gs4_write_complete0(struct usb_ep *ep, struct usb_request *req)
{
	gs4_write_complete(ep, req, 0);
}
static void gs4_write_complete1(struct usb_ep *ep, struct usb_request *req)
{
	gs4_write_complete(ep, req, 1);
}
static void gs4_write_complete2(struct usb_ep *ep, struct usb_request *req)
{
	gs4_write_complete(ep, req, 2);
}
static void gs4_write_complete3(struct usb_ep *ep, struct usb_request *req)
{
	gs4_write_complete(ep, req, 3);
}

/* Gadget Driver */

/*
 * gs4_bind
 *
 * Called on module load.  Allocates and initializes the device
 * structure and a control request.
 */
static int gs4_bind(struct usb_gadget *gadget)
{
#ifdef USB_COMPOSITE_DEVICE
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
#else
	int gcnum;
#endif /* USB_COMPOSITE_DEVICE */
	int ret, i;
	struct usb_ep *ep;
	struct gs4_dev *dev;

	/* Some controllers can't support CDC ACM:
	 * - sh doesn't support multiple interfaces or configs;
	 * - sa1100 doesn't have a third interrupt endpoint
	 */
	dprintk("enter\n");

	if (gadget_is_sh(gadget) || gadget_is_sa1100(gadget))
		use_acm = 0;

#ifdef USB_COMPOSITE_DEVICE
#else
	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		gs4_device_desc.bcdDevice =
				cpu_to_le16(GS4_VERSION_NUM | gcnum);
	else {
		printk(KERN_WARNING "gs4_bind: controller '%s' not recognized\n",
			gadget->name);
		/* unrecognized, but safe unless bulk is REALLY quirky */
		gs4_device_desc.bcdDevice =
			__constant_cpu_to_le16(GS4_VERSION_NUM|0x0099);
	}

	usb_ep_autoconfig_reset(gadget);
#endif /* USB_COMPOSITE_DEVICE */

	for (i=0; i<num_ports; i++)
	{
		ep = usb_ep_autoconfig(gadget, &gs4_fullspeed_in_desc[i]);
		if (!ep) {
			printk("in%d fail\n", i);
			goto autoconf_fail;
		}
		EP_IN_NAME[i] = ep->name;
		ep->driver_data = ep;	/* claim the endpoint */

		ep = usb_ep_autoconfig(gadget, &gs4_fullspeed_out_desc[i]);
		if (!ep) {
			printk("out%d fail\n", i);
			goto autoconf_fail;
		}
		EP_OUT_NAME[i] = ep->name;
		ep->driver_data = ep;	/* claim the endpoint */
	}

	if (use_acm) {
#ifndef USB_COMPOSITE_DEVICE
		gs4_device_desc.idProduct = __constant_cpu_to_le16(
			GS4_CDC_PRODUCT_ID);
#endif /* !USB_COMPOSITE_DEVICE */
		for (i=0; i<num_ports; i++) {
			ep = usb_ep_autoconfig(gadget, &gs4_fullspeed_notify_desc[i]);
			if (!ep) {
				printk("acm fail on port %d\n", i);
				printk(KERN_ERR "gs4_bind: cannot run ACM on %s\n",
					gadget->name);
				goto autoconf_fail;
			}
			EP_NOTIFY_NAME[i] = ep->name;
			ep->driver_data = ep;	/* claim the endpoint */
		}
	} else if (use_qc) {
#ifndef USB_COMPOSITE_DEVICE
		gs4_device_desc.idProduct = __constant_cpu_to_le16(
			GS4_QC_PRODUCT_ID);
#endif /* !USB_COMPOSITE_DEVICE */
		ep = usb_ep_autoconfig(gadget, &gs4_fullspeed_notify_desc[0]);
		if (!ep) {
			printk("acm fail on port 0\n");
			printk(KERN_ERR "gs4_bind: cannot run ACM on %s\n",
			       gadget->name);
			goto autoconf_fail;
		}
		EP_NOTIFY_NAME[0] = ep->name;
		ep->driver_data = ep;	/* claim the endpoint */
	}

#ifndef USB_COMPOSITE_DEVICE
	gs4_device_desc.bDeviceClass = use_acm
		? USB_CLASS_COMM : USB_CLASS_VENDOR_SPEC;
	gs4_device_desc.bMaxPacketSize0 = gadget->ep0->maxpacket;
#endif /* !USB_COMPOSITE_DEVICE */

#ifdef CONFIG_USB_GADGET_DUALSPEED
#ifndef USB_COMPOSITE_DEVICE
	gs4_qualifier_desc.bDeviceClass = use_acm
		? USB_CLASS_COMM : USB_CLASS_VENDOR_SPEC;
	/* assume ep0 uses the same packet size for both speeds */
	gs4_qualifier_desc.bMaxPacketSize0 = gs4_device_desc.bMaxPacketSize0;
#endif /* !USB_COMPOSITE_DEVICE */
	/* assume endpoints are dual-speed */
	for (i=0; i<num_ports; i++) {
		gs4_highspeed_notify_desc[i].bEndpointAddress =
			gs4_fullspeed_notify_desc[i].bEndpointAddress;
		gs4_highspeed_in_desc[i].bEndpointAddress =
			gs4_fullspeed_in_desc[i].bEndpointAddress;
		gs4_highspeed_out_desc[i].bEndpointAddress =
			gs4_fullspeed_out_desc[i].bEndpointAddress;
	}
#endif /* CONFIG_USB_GADGET_DUALSPEED */

#ifndef USB_COMPOSITE_DEVICE
	usb_gadget_set_selfpowered(gadget);
#endif /* !USB_COMPOSITE_DEVICE */

	if (gadget->is_otg) { /* REVISIT */
#ifdef USB_COMPOSITE_DEVICE
		gs4_otg_descriptor.bmAttributes |= USB_OTG_HNP;
#else
		gs4_otg_descriptor.bmAttributes |= USB_OTG_HNP,
		gs4_bulk_config_desc.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
		gs4_acm_config_desc.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
		gs4_qc_config_desc.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
#endif /* !USB_COMPOSITE_DEVICE */
	}

	gs4_device = dev = kzalloc(sizeof(struct gs4_dev), GFP_KERNEL);
	if (dev == NULL)
		return -ENOMEM;

	snprintf(manufacturer, sizeof(manufacturer), "%s %s with %s",
		init_utsname()->sysname, init_utsname()->release,
		gadget->name);

	memset(dev, 0, sizeof(struct gs4_dev));
	dev->dev_gadget = gadget;
	spin_lock_init(&dev->dev_lock);

	for (i=0; i < num_ports; i++) {
		INIT_LIST_HEAD(&dev->dev_req_list[i]);
	}

	//dev->dev_write_req_pending=0;
#ifdef USB_COMPOSITE_DEVICE
	set_composite_data(cdev, dev);
#else
	set_gadget_data(gadget, dev);
#endif /* USB_COMPOSITE_DEVICE */
	//dev->suspended = 0;

	if ((ret=gs4_alloc_ports(dev, GFP_KERNEL)) != 0) {
		printk(KERN_ERR "gs4_bind: cannot allocate ports\n");
		gs4_unbind(gadget);
		return ret;
	}

	/* preallocate control response and buffer */
#ifdef USB_COMPOSITE_DEVICE
	dev->dev_ctrl_req = cdev->req;
#else
	dev->dev_ctrl_req = gs4_alloc_req(gadget->ep0, GS4_MAX_DESC_LEN,
		GFP_KERNEL);
	if (dev->dev_ctrl_req == NULL) {
		gs4_unbind(gadget);
		return -ENOMEM;
	}
	dev->dev_ctrl_req->complete = gs4_setup_complete;

	gadget->ep0->driver_data = dev;
#endif /* USB_COMPOSITE_DEVICE */

	printk(KERN_INFO "gs4_bind: %s %s bound- %d ports\n",
		GS4_LONG_NAME, GS4_VERSION_STR, num_ports);
	for (i=0; i < num_ports; i++) {
		if (EP_NOTIFY_NAME[i])
			printk(KERN_INFO "using %s, OUT %s IN %s STATUS %s\n",
			       gadget->name, EP_OUT_NAME[i], EP_IN_NAME[i],
			       EP_NOTIFY_NAME[i]);
		else
			printk(KERN_INFO "using %s, OUT %s IN %s\n",
			       gadget->name, EP_OUT_NAME[i], EP_IN_NAME[i]);
	}

	return 0;

autoconf_fail:
	printk(KERN_ERR "gs4_bind: cannot autoconfigure on %s\n", gadget->name);
	return -ENODEV;
}

/*
 * gs4_unbind
 *
 * Called on module unload.  Frees the control request and device
 * structure.
 */
static void gs4_unbind(struct usb_gadget *gadget)
{
#ifdef USB_COMPOSITE_DEVICE
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct gs4_dev *dev = get_composite_data(cdev);
	int i;
#else
	struct gs4_dev *dev = get_gadget_data(gadget);
#endif /* USB_COMPOSITE_DEVICE */

	gs4_device = NULL;

	/* read/write requests already freed, only control request remains */
	if (dev != NULL) {
#ifdef USB_COMPOSITE_DEVICE
		for (i=0; i < num_ports; i++) {
			usb_composite_ep_reset(dev->dev_notify_ep[i]);
			usb_composite_ep_reset(dev->dev_in_ep[i]);
			usb_composite_ep_reset(dev->dev_out_ep[i]);
		}
		gs4_reset_config(dev);

		if (dev->dev_notify_req != NULL) {
			gs4_free_req(dev->dev_notify_ep[0], dev->dev_notify_req);
			dev->dev_notify_req = NULL;
		}
		gs4_free_ports(dev);
		kfree(dev);
		set_composite_data(cdev, NULL);
#else
		if (dev->dev_ctrl_req != NULL) {
			gs4_free_req(gadget->ep0, dev->dev_ctrl_req);
			dev->dev_ctrl_req = NULL;
		}
		if (dev->dev_notify_req != NULL) {
			gs4_free_req(dev->dev_notify_ep[0], dev->dev_notify_req);
			dev->dev_notify_req = NULL;
		}
		gs4_free_ports(dev);
		kfree(dev);
		set_gadget_data(gadget, NULL);
#endif /* USB_COMPOSITE_DEVICE */
	}

	printk(KERN_INFO "gs4_unbind: %s %s unbound\n", GS4_LONG_NAME,
		GS4_VERSION_STR);
}

/*
 * gs4_set_descriptors
 */
static int gs4_set_descriptors(int config, int is_otg)
{
	int first_interface = gs4_usb_function.first_interface;

	dprintk("first_interface=%d\n", first_interface);

	/* fix interface numbers */
	gs4_iad_descriptor[0].bFirstInterface = first_interface;
	gs4_iad_descriptor[1].bFirstInterface = first_interface+2;
	gs4_iad_descriptor[2].bFirstInterface = first_interface+4;
	gs4_iad_descriptor[3].bFirstInterface = first_interface+6;

	gs4_bulk_interface_desc[0].bInterfaceNumber = first_interface;
	gs4_bulk_interface_desc[1].bInterfaceNumber = first_interface+1;
	gs4_bulk_interface_desc[2].bInterfaceNumber = first_interface+2;
	gs4_bulk_interface_desc[3].bInterfaceNumber = first_interface+3;

	gs4_qc_interface_desc[0].bInterfaceNumber = first_interface;
	gs4_qc_interface_desc[1].bInterfaceNumber = first_interface+1;
	gs4_qc_interface_desc[2].bInterfaceNumber = first_interface+2;
	gs4_qc_interface_desc[3].bInterfaceNumber = first_interface+3;

	gs4_control_interface_desc[0].bInterfaceNumber = first_interface;
	gs4_control_interface_desc[1].bInterfaceNumber = first_interface+2;
	gs4_control_interface_desc[2].bInterfaceNumber = first_interface+4;
	gs4_control_interface_desc[3].bInterfaceNumber = first_interface+6;

	gs4_data_interface_desc[0].bInterfaceNumber = first_interface+1;
	gs4_data_interface_desc[1].bInterfaceNumber = first_interface+3;
	gs4_data_interface_desc[2].bInterfaceNumber = first_interface+5;
	gs4_data_interface_desc[3].bInterfaceNumber = first_interface+7;

	gs4_call_mgmt_descriptor[0].bDataInterface = first_interface+1;
	gs4_call_mgmt_descriptor[1].bDataInterface = first_interface+3;
	gs4_call_mgmt_descriptor[2].bDataInterface = first_interface+5;
	gs4_call_mgmt_descriptor[3].bDataInterface = first_interface+7;

	gs4_union_desc[0].bMasterInterface0 = first_interface;
	gs4_union_desc[0].bSlaveInterface0 = first_interface+1;
	gs4_union_desc[1].bMasterInterface0 = first_interface+2;
	gs4_union_desc[1].bSlaveInterface0 = first_interface+3;
	gs4_union_desc[2].bMasterInterface0 = first_interface+4;
	gs4_union_desc[2].bSlaveInterface0 = first_interface+5;
	gs4_union_desc[3].bMasterInterface0 = first_interface+6;
	gs4_union_desc[3].bSlaveInterface0 = first_interface+7;

	/* set descriptors */
	gs4_usb_function.descriptors = use_acm ? gs4_acm_fullspeed_function :
		use_qc ? gs4_qc_fullspeed_function : gs4_bulk_fullspeed_function;
#ifdef CONFIG_USB_GADGET_DUALSPEED
	gs4_usb_function.hs_descriptors = use_acm ? gs4_acm_highspeed_function :
		use_qc ? gs4_qc_highspeed_function : gs4_bulk_highspeed_function;
#endif
	gs4_usb_function.num_interfaces = use_acm ? 2 * num_ports :
		use_qc ? num_ports : num_ports;

	if (!is_otg) {
		/* skip over the otg descriptor if it's not being used */
		gs4_usb_function.descriptors++;
#ifdef CONFIG_USB_GADGET_DUALSPEED
		gs4_usb_function.hs_descriptors++;
#endif
	}
	return 0;
}

/*
 * gs4_setup
 *
 * Implements all the control endpoint functionality that's not
 * handled in hardware or the hardware driver.
 *
 * Returns the size of the data sent to the host, or a negative
 * error number.
 */
static int gs4_setup(struct usb_gadget *gadget,
	const struct usb_ctrlrequest *ctrl)
{
	int ret = -EOPNOTSUPP;
#ifndef USB_COMPOSITE_DEVICE
	struct gs4_dev *dev = get_gadget_data(gadget);
	struct usb_request *req = dev->dev_ctrl_req;
#endif /* USB_COMPOSITE_DEVICE */
	u16 wIndex = le16_to_cpu(ctrl->wIndex);
	u16 wValue = le16_to_cpu(ctrl->wValue);
	u16 wLength = le16_to_cpu(ctrl->wLength);

	dprintk("wIndex %d; case ctrl->bRequest=%#x\n", wIndex, ctrl->bRequest);

	switch (ctrl->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_STANDARD:
		ret = gs4_setup_standard(gadget,ctrl);
		break;

	case USB_TYPE_CLASS:
		ret = gs4_setup_class(gadget,ctrl);
		break;

	default:
		printk(KERN_ERR "gs4_setup: unknown request, type=%02x, request=%02x, "
            "value=%04x, wIndex=%04x, length=%d\n",	ctrl->bRequestType,
            ctrl->bRequest, wValue, wIndex, wLength);
		break;
	}

#ifndef USB_COMPOSITE_DEVICE
	/* respond with data transfer before status phase? */
	if (ret >= 0) {
		req->length = ret;
		req->zero = ret < wLength
				&& (ret % gadget->ep0->maxpacket) == 0;
		ret = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
		if (ret < 0) {
			printk(KERN_ERR "gs4_setup: cannot queue response, ret=%d\n",
				ret);
			req->status = 0;
			gs4_setup_complete(gadget->ep0, req);
		}
	}
#endif /* !USB_COMPOSITE_DEVICE */

	/* device either stalls (ret < 0) or reports success */
	return ret;
}

static int gs4_setup_standard(struct usb_gadget *gadget,
	const struct usb_ctrlrequest *ctrl)
{
	int ret = -EOPNOTSUPP;
#ifdef USB_COMPOSITE_DEVICE
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct gs4_dev *dev = get_composite_data(cdev);
#else
	struct gs4_dev *dev = get_gadget_data(gadget);
#endif
	struct usb_request *req = dev->dev_ctrl_req;
	u16 wIndex = le16_to_cpu(ctrl->wIndex);
	u16 wValue = le16_to_cpu(ctrl->wValue);
	u16 wLength = le16_to_cpu(ctrl->wLength);
	int idx;
	int first_interface = gs4_usb_function.first_interface;

	switch (ctrl->bRequest) {

#ifndef USB_COMPOSITE_DEVICE
	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_DIR_IN)
			break;

		switch (wValue >> 8) {
		case USB_DT_DEVICE:
			ret = min(wLength,
				(u16)sizeof(struct usb_device_descriptor));
			memcpy(req->buf, &gs4_device_desc, ret);
			break;

#ifdef CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			if (!gadget->is_dualspeed)
				break;
			ret = min(wLength,
				(u16)sizeof(struct usb_qualifier_descriptor));
			memcpy(req->buf, &gs4_qualifier_desc, ret);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			if (!gadget->is_dualspeed)
				break;
			/* fall through */
#endif /* CONFIG_USB_GADGET_DUALSPEED */
		case USB_DT_CONFIG:
			ret = gs4_build_config_buf(req->buf, gadget->speed,
				wValue >> 8, wValue & 0xff,
				gadget->is_otg);
			if (ret >= 0)
				ret = min(wLength, (u16)ret);
			break;

		case USB_DT_STRING:
			/* wIndex == language code. */
			ret = usb_gadget_get_string(&gs4_string_table,
				wValue & 0xff, req->buf);
			if (ret >= 0)
				ret = min(wLength, (u16)ret);
			break;
		}
		break;
#endif /* !USB_COMPOSITE_DEVICE */

	case USB_REQ_SET_CONFIGURATION:
		if (ctrl->bRequestType != 0)
			break;
#ifdef USB_COMPOSITE_DEVICE
		/* REVISIT */
		if (wValue == COMPOSITE_500MA_CONFIG_VALUE ||
		    wValue == COMPOSITE_100MA_CONFIG_VALUE) {
			wValue = use_acm ? GS4_ACM_CONFIG_ID :
				use_qc ? GS4_QC_CONFIG_ID : GS4_BULK_CONFIG_ID;
		}
#endif /* USB_COMPOSITE_DEVICE */
		dprintk("set CONFIG wIndex %xh\n", wIndex);
		spin_lock(&dev->dev_lock);
		ret = gs4_set_config(dev, wValue);
		spin_unlock(&dev->dev_lock);
		break;

#ifndef USB_COMPOSITE_DEVICE
	case USB_REQ_GET_CONFIGURATION:
		if (ctrl->bRequestType != USB_DIR_IN)
			break;
		*(u8 *)req->buf = dev->dev_config;
		ret = min(wLength, (u16)1);
		break;
#endif /* !USB_COMPOSITE_DEVICE */

	case USB_REQ_SET_INTERFACE:
		if (ctrl->bRequestType != USB_RECIP_INTERFACE
				|| !dev->dev_config
		    || wIndex >= (first_interface + GS4_MAX_NUM_INTERFACES)) {
			dprintk("error\n");
			break;
		}
		dprintk("set IF wIndex %xh\n", wIndex);

		if (dev->dev_config == GS4_BULK_CONFIG_ID &&
		    ((first_interface > 0 && wIndex < first_interface) ||
		     wIndex >= (first_interface + num_ports))) {
			dprintk("wIndex range error\n");
			break;
		}
#ifndef USB_COMPOSITE_DEVICE
		/* REVISIT */
		/* no alternate interface settings */
		if (wValue != 0) {
			dprintk("no alternate interface settings\n");
			break;
		}
#endif
		spin_lock(&dev->dev_lock);
		/* PXA hardware partially handles SET_INTERFACE;
		 * we need to kluge around that interference.  */
		if (gadget_is_pxa(gadget)) {
			ret = gs4_set_config(dev, use_acm ? GS4_ACM_CONFIG_ID :
				use_qc ? GS4_QC_CONFIG_ID : GS4_BULK_CONFIG_ID);
			goto set_interface_done;
		}
		idx = wIndex - first_interface;
		if (dev->dev_config == GS4_ACM_CONFIG_ID &&
		    (wIndex == first_interface     ||
                     wIndex == (first_interface+2) ||
                     wIndex == (first_interface+4) ||
                     wIndex == (first_interface+6))) {
			if (dev->dev_notify_ep[idx]) {
				usb_ep_disable(dev->dev_notify_ep[idx]);
				usb_ep_enable(dev->dev_notify_ep[idx],
					      dev->dev_notify_ep_desc[idx]);
			}
		} else if (dev->dev_config == GS4_QC_CONFIG_ID) {
			if (dev->dev_notify_ep[idx]) {
				usb_ep_disable(dev->dev_notify_ep[idx]);
				usb_ep_enable(dev->dev_notify_ep[idx],
					      dev->dev_notify_ep_desc[idx]);
			}
			usb_ep_disable(dev->dev_in_ep[idx]);
			usb_ep_disable(dev->dev_out_ep[idx]);
			usb_ep_enable(dev->dev_in_ep[idx], dev->dev_in_ep_desc[idx]);
			usb_ep_enable(dev->dev_out_ep[idx], dev->dev_out_ep_desc[idx]);
		} else {
			usb_ep_disable(dev->dev_in_ep[idx]);
			usb_ep_disable(dev->dev_out_ep[idx]);
			usb_ep_enable(dev->dev_in_ep[idx], dev->dev_in_ep_desc[idx]);
			usb_ep_enable(dev->dev_out_ep[idx], dev->dev_out_ep_desc[idx]);
		}
		ret = 0;
set_interface_done:
		spin_unlock(&dev->dev_lock);
		break;

	case USB_REQ_GET_INTERFACE:
		if (ctrl->bRequestType != (USB_DIR_IN|USB_RECIP_INTERFACE)
		    || dev->dev_config == GS4_NO_CONFIG_ID) {
			dprintk("error\n");
			break;
		}
		if (wIndex >= GS4_MAX_NUM_INTERFACES
		    || (dev->dev_config == GS4_BULK_CONFIG_ID &&
			((first_interface > 0 && wIndex < first_interface) ||
			 wIndex >= (first_interface+num_ports)))) {
			ret = -EDOM;
			dprintk("wIndex range error\n");
			break;
		}
		/* no alternate interface settings */
		*(u8 *)req->buf = 0;
		ret = min(wLength, (u16)1);
		break;

	default:
		printk(KERN_ERR "gs4_setup: unknown standard request, type=%02x,"
                       " request=%02x, value=%04x, index=%04x, length=%d\n",
		       ctrl->bRequestType, ctrl->bRequest, wValue, wIndex, wLength);
		break;
	}

	dprintk("returning %d\n",ret);
	return ret;
}

static int gs4_setup_class(struct usb_gadget *gadget,
	const struct usb_ctrlrequest *ctrl)
{
	int ret = -EOPNOTSUPP;
#ifdef USB_COMPOSITE_DEVICE
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct gs4_dev *dev = get_composite_data(cdev);
#else
	struct gs4_dev *dev = get_gadget_data(gadget);
#endif
	struct gs4_port *port = dev->dev_port[0];	// REVISIT: assuming intf 0
	struct usb_request *req = dev->dev_ctrl_req;
	u16 wIndex = le16_to_cpu(ctrl->wIndex);
	u16 wValue = le16_to_cpu(ctrl->wValue);
	u16 wLength = le16_to_cpu(ctrl->wLength);
	struct gs4_ctlreq_entry *ctlreq_entry;
#ifdef FORWARD_SETUP_DATA
	unsigned char *data;

	data = (unsigned char *)(ctrl + 1); /* REVISIT: This is not correct */
#endif
	printk(KERN_INFO "%s: ctlreq %#x received: windex %d wValue %d wLength %d\n",
	       __func__, ctrl->bRequest, wIndex, wValue, wLength);
#ifdef FORWARD_SETUP_DATA
	if (wLength == 7)
		printk(KERN_INFO "%s: data: %x %x %x %x %x %x %x\n",
		       __func__, data[0], data[1], data[2], data[3], data[4],
		       data[5], data[6], data[7]);
#endif
	spin_lock(&port->port_lock);
	if (list_empty(&port->port_spare_ctlreq_entries)) {
		printk(KERN_ERR "%s: spare ctlreq list is empty\n", __func__);
		spin_unlock(&port->port_lock);
		goto exit;
	}
	ctlreq_entry = list_entry(port->port_spare_ctlreq_entries.next, struct gs4_ctlreq_entry, list);
	list_del(&ctlreq_entry->list);
	spin_unlock(&port->port_lock);

	ctlreq_entry->ctlreq.bmRequestType = ctrl->bRequestType;
	ctlreq_entry->ctlreq.bRequest = ctrl->bRequest;
	ctlreq_entry->ctlreq.wValue = wValue;
	ctlreq_entry->ctlreq.wIndex = wIndex;
	ctlreq_entry->ctlreq.wLength = wLength;

#ifdef FORWARD_SETUP_DATA
	if (wLength > PIOC_REQ_DATA_SIZE) {
		printk(KERN_ERR "%s: control request datasize is too big (%d bytes)\n",
			       __func__, wLength);
		spin_lock(&port->port_lock);
		list_add(&ctlreq_entry->list, &port->port_spare_ctlreq_entries);
		spin_unlock(&port->port_lock);
		goto exit;
	}
	if (wLength > 0)
		memcpy(ctlreq_entry->ctlreq.data, data, wLength);
#endif
	spin_lock(&port->port_lock);
	list_add_tail(&ctlreq_entry->list, &port->port_filled_ctlreq_entries);
	spin_unlock(&port->port_lock);

	wake_up_interruptible(&port->port_ctlreq_wait);

exit:

	switch (ctrl->bRequest) {
	case USB_CDC_REQ_SET_LINE_CODING:
		ret = min(wLength,
			(u16)sizeof(struct usb_cdc_line_coding));
		break;

	case USB_CDC_REQ_GET_LINE_CODING:
		port = dev->dev_port[0];	/* REVISIT: assuming intf 0 */
		ret = min(wLength,
			(u16)sizeof(struct usb_cdc_line_coding));
		if (port) {
			spin_lock(&port->port_lock);
			memcpy(req->buf, &port->port_line_coding, ret);
			spin_unlock(&port->port_lock);
		}
		break;

	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		/* printk("gs4_setup: SET_CONTROL_LINE_STATE value=%#x\n", wValue); */
		port = dev->dev_port[0];	/* REVISIT: assuming intf 0 */

		ret = 0;
		break;

	default:
		printk(KERN_ERR "gs4_setup: unknown class request, type=%02x,"
                       " request=%02x, value=%04x, index=%04x, length=%d\n",
		       ctrl->bRequestType, ctrl->bRequest, wValue, wIndex, wLength);
		break;
	}

	return ret;
}

#ifndef USB_COMPOSITE_DEVICE
/*
 * gs4_setup_complete
 */
static void gs4_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status || req->actual != req->length) {
		printk(KERN_ERR "gs4_setup_complete: status error, status=%d,"
		       " actual=%d, length=%d\n", req->status, req->actual, req->length);
	}
}
#endif /* !USB_COMPOSITE_DEVICE */

/*
 * gs4_disconnect
 *
 * Called when the device is disconnected.  Frees the closed
 * ports and disconnects open ports.  Open ports will be freed
 * on close.  Then reallocates the ports for the next connection.
 */
static void gs4_disconnect(struct usb_gadget *gadget)
{
	unsigned long flags;
#ifdef USB_COMPOSITE_DEVICE
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct gs4_dev *dev = get_composite_data(cdev);
#else
	struct gs4_dev *dev = get_gadget_data(gadget);
#endif /* USB_COMPOSITE_DEVICE */

	spin_lock_irqsave(&dev->dev_lock, flags);

	gs4_reset_config(dev);

	/* free closed ports and disconnect open ports */
	/* (open ports will be freed when closed) */
	gs4_free_ports(dev);

	/* re-allocate ports for the next connection */
	if (gs4_alloc_ports(dev, GFP_ATOMIC) != 0)
		printk(KERN_ERR "gs4_disconnect: cannot re-allocate ports\n");

	spin_unlock_irqrestore(&dev->dev_lock, flags);

	printk(KERN_INFO "gs4_disconnect: %s disconnected\n", GS4_LONG_NAME);
}

/*-------------------------------------------------------------------------*/

//static void
//gs4_suspend (struct usb_gadget *gadget)
//{
//#ifdef USB_COMPOSITE_DEVICE
//	struct usb_composite_dev *cdev = get_gadget_data(gadget);
//	struct gs4_dev		 *dev = get_composite_data(cdev);
//#else
//	struct gs4_dev		 *dev = get_gadget_data (gadget);
//#endif
//
//	dprintk(dev, "suspend\n");
//	dev->suspended = 1;
//}
//
//static void
//gs4_resume (struct usb_gadget *gadget)
//{
//#ifdef USB_COMPOSITE_DEVICE
//	struct usb_composite_dev *cdev = get_gadget_data(gadget);
//	struct gs4_dev		 *dev = get_composite_data(cdev);
//#else
//	struct gs4_dev		 *dev = get_gadget_data (gadget);
//#endif
//
//	dprintk(dev, "resume\n");
//	dev->suspended = 0;
//#ifdef CONFIG_PM_DEVICE_LOCKOUT
//	wake_up_all(&gs4_suspend_wq);
//#endif
//}


/*
 * gs4_set_config
 *
 * Configures the device by enabling device specific
 * optimizations, setting up the endpoints, allocating
 * read and write requests and queuing read requests.
 *
 * The device lock must be held when calling this function.
 */
static int gs4_set_config(struct gs4_dev *dev, unsigned config)
{
	int port_num;
	int i;
	int ret = 0;
	struct usb_gadget *gadget;
	struct usb_ep *ep;
	struct usb_endpoint_descriptor *ep_desc;
	struct usb_request *req;
	struct gs4_req_entry *req_entry;

	if (dev == NULL) {
		printk(KERN_ERR "gs4_set_config: NULL device pointer\n");
		return 0;
	}

	gadget = dev->dev_gadget;

	dprintk("config=%u\n", config);

	if (config == dev->dev_config)
		return 0;

	gs4_reset_config(dev);

	switch (config) {
	case GS4_NO_CONFIG_ID:
		return 0;
	case GS4_BULK_CONFIG_ID:
		if (use_acm)
			return -EINVAL;
		/* device specific optimizations */
		if (gadget_is_net2280(gadget))
			net2280_set_fifo_mode(gadget, 1);
		break;
	case GS4_ACM_CONFIG_ID:
		if (!use_acm)
			return -EINVAL;
		/* device specific optimizations */
		if (gadget_is_net2280(gadget))
			net2280_set_fifo_mode(gadget, 1);
		break;
	case GS4_QC_CONFIG_ID:
		if (!use_qc)
			return -EINVAL;
		/* device specific optimizations */
		if (gadget_is_net2280(gadget))
			net2280_set_fifo_mode(gadget, 1);
		break;
	default:
		return -EINVAL;
	}

	dev->dev_config = config;

	gadget_for_each_ep(ep, gadget) {
		for (port_num=0; port_num<num_ports && port_num<GS4_MAX_NUM_PORTS; port_num++)
		{
			if (EP_NOTIFY_NAME[port_num] &&
			    strcmp(ep->name, EP_NOTIFY_NAME[port_num]) == 0) {
				ep_desc = GS4_SPEED_SELECT(
					gadget->speed == USB_SPEED_HIGH,
					&gs4_highspeed_notify_desc[port_num],
					&gs4_fullspeed_notify_desc[port_num]);
				ret = usb_ep_enable(ep,ep_desc);
				if (ret == 0) {
					ep->driver_data = dev;
					dev->dev_notify_ep[port_num] = ep;
					dev->dev_notify_ep_desc[port_num] = ep_desc;
				} else {
					printk(KERN_ERR "gs4_set_config: cannot enable notify endpoint"
					       " %s, ret=%d, port %d\n", ep->name, ret, port_num);
					goto exit_reset_config;
				}
			}
			else if (strcmp(ep->name, EP_IN_NAME[port_num]) == 0) {
				ep_desc = GS4_SPEED_SELECT(
					gadget->speed == USB_SPEED_HIGH,
					&gs4_highspeed_in_desc[port_num],
					&gs4_fullspeed_in_desc[port_num]);
				ret = usb_ep_enable(ep,ep_desc);
				if (ret == 0) {
					ep->driver_data = dev;
					dev->dev_in_ep[port_num] = ep;
					dev->dev_in_ep_desc[port_num] = ep_desc;
				} else {
					printk(KERN_ERR "gs4_set_config %d: cannot enable in endpoint"
					       " %s, ret=%d, port %d\n",
						__LINE__,ep->name, ret, port_num);
					goto exit_reset_config;
				}
			}

			else if (strcmp(ep->name, EP_OUT_NAME[port_num]) == 0) {
				ep_desc = GS4_SPEED_SELECT(
					gadget->speed == USB_SPEED_HIGH,
					&gs4_highspeed_out_desc[port_num],
					&gs4_fullspeed_out_desc[port_num]);
				ret = usb_ep_enable(ep,ep_desc);
				if (ret == 0) {
					ep->driver_data = dev;
					dev->dev_out_ep[port_num] = ep;
					dev->dev_out_ep_desc[port_num] = ep_desc;
				} else {
					printk(KERN_ERR "gs4_set_config %d: cannot enable out endpoint"
					       " %s, ret=%d, port %d\n",
						__LINE__,ep->name, ret, port_num);
					goto exit_reset_config;
				}
			}
		}
	}

	for (port_num=0; port_num < num_ports; port_num++) {
		if (dev->dev_in_ep[port_num] == NULL || dev->dev_out_ep[port_num] == NULL
			|| (config == GS4_ACM_CONFIG_ID && dev->dev_notify_ep[port_num] == NULL)
			|| (config == GS4_QC_CONFIG_ID && port_num == 0 && dev->dev_notify_ep[port_num] == NULL)) {
			printk(KERN_ERR "gs4_set_config: cannot find endpoints\n");
			ret = -ENODEV;
			goto exit_reset_config;
		}
	}

	/* allocate and queue read requests */
	for (port_num=0; port_num < num_ports; port_num++) {
		ep = dev->dev_out_ep[port_num];
		for (i=0; i<read_q_size && ret == 0; i++) {
			if ((req=gs4_alloc_req(ep, ep->maxpacket, GFP_ATOMIC))) {
				switch (port_num) {
				case 0: req->complete = gs4_read_complete0; break;
				case 1: req->complete = gs4_read_complete1; break;
				case 2: req->complete = gs4_read_complete2; break;
				case 3: req->complete = gs4_read_complete3; break;
				}
				if ((ret=usb_ep_queue(ep, req, GFP_ATOMIC))) {
					printk(KERN_ERR "gs4_set_config: cannot queue read request, ret=%d\n",
						ret);
				}
			} else {
				printk(KERN_ERR "gs4_set_config: cannot allocate read requests\n");
				ret = -ENOMEM;
				goto exit_reset_config;
			}
		}
	}

	/* allocate write requests, and put on free list */
	for (port_num=0; port_num < num_ports; port_num++)
	{
		ep = dev->dev_in_ep[port_num];
		for (i=0; i<write_q_size; i++) {
			if ((req_entry=gs4_alloc_req_entry(ep, ep->maxpacket, GFP_ATOMIC))) {
				switch (port_num) {
				    case 0: req_entry->re_req->complete =
						gs4_write_complete0;
					break;
				    case 1: req_entry->re_req->complete =
						gs4_write_complete1;
					break;
				    case 2: req_entry->re_req->complete =
						gs4_write_complete2;
					break;
				    case 3: req_entry->re_req->complete =
						gs4_write_complete3;
					break;
				}
				list_add(&req_entry->re_entry, &dev->dev_req_list[port_num]);
			} else {
				printk(KERN_ERR "gs4_set_config: cannot allocate write requests\n");
				ret = -ENOMEM;
				goto exit_reset_config;
			}
		}
	}


#ifndef USB_COMPOSITE_DEVICE
	printk(KERN_INFO "gs4_set_config: %s configured, %s speed %s config\n",
	       GS4_LONG_NAME,
	       gadget->speed == USB_SPEED_HIGH ? "high" : "full",
	       config == GS4_BULK_CONFIG_ID ? "BULK" :
	       config == GS4_ACM_CONFIG_ID ? "CDC-ACM" :
	       config == GS4_QC_CONFIG_ID ? "QUALCOMM" : "Unknown");
#endif /* !USB_COMPOSITE_DEVICE */

	return 0;

exit_reset_config:
	gs4_reset_config(dev);
	return ret;
}

/*
 * gs4_reset_config
 *
 * Mark the device as not configured, disable all endpoints,
 * which forces completion of pending I/O and frees queued
 * requests, and free the remaining write requests on the
 * free list.
 *
 * The device lock must be held when calling this function.
 */
static void gs4_reset_config(struct gs4_dev *dev)
{
	struct gs4_req_entry *req_entry;
	int i;

	if (dev == NULL) {
		printk(KERN_ERR "gs4_reset_config: NULL device pointer\n");
		return;
	}

	if (dev->dev_config == GS4_NO_CONFIG_ID)
		return;

	dev->dev_config = GS4_NO_CONFIG_ID;

	/* free write requests on the free list */
	for (i=0; i < num_ports; i++) {
		while (!list_empty(&dev->dev_req_list[i])) {
			req_entry = list_entry(dev->dev_req_list[i].next,
				struct gs4_req_entry, re_entry);
			list_del(&req_entry->re_entry);
			gs4_free_req_entry(dev->dev_in_ep[i], req_entry);
		}
	}

	/* disable endpoints, forcing completion of pending i/o; */
	/* completion handlers free their requests in this case */

	for (i=0; i < num_ports; i++) {
		if (dev->dev_notify_ep[i]) {
		    //printk("reset_config: disable notify_ep[%d]\n", i);
		    usb_ep_disable(dev->dev_notify_ep[i]);
		    dev->dev_notify_ep[i] = NULL;
		}
		if (dev->dev_in_ep[i]) {
		    //printk("reset_config: disable in_ep[%d]\n", i);
		    usb_ep_disable(dev->dev_in_ep[i]);
		    dev->dev_in_ep[i] = NULL;
		}
		if (dev->dev_out_ep[i]) {
		    //printk("reset_config: disable out_ep[%d]\n", i);
		    usb_ep_disable(dev->dev_out_ep[i]);
		    dev->dev_out_ep[i] = NULL;
		}
	}
}

/* if defined composite, build the config
 * buf for all gadgets in the composite.
 */
#ifndef USB_COMPOSITE_DEVICE
/*
 * gs4_build_config_buf
 *
 * Builds the config descriptors in the given buffer and returns the
 * length, or a negative error number.
 */
static int gs4_build_config_buf(u8 *buf, enum usb_device_speed speed,
	u8 type, unsigned int index, int is_otg)
{
	int len;
	int high_speed;
	const struct usb_config_descriptor *config_desc;
	/* const */ struct usb_descriptor_header **function;

	dprintk("enter\n");

	if (index >= gs4_device_desc.bNumConfigurations)
		return -EINVAL;

	/* other speed switches high and full speed */
	high_speed = (speed == USB_SPEED_HIGH);
	if (type == USB_DT_OTHER_SPEED_CONFIG)
		high_speed = !high_speed;

	if (use_acm) {
		config_desc = &gs4_acm_config_desc;
		function = GS4_SPEED_SELECT(high_speed,
			gs4_acm_highspeed_function,
			gs4_acm_fullspeed_function);
	} else if (use_qc) {
		config_desc = &gs4_qc_config_desc;
		function = GS4_SPEED_SELECT(high_speed,
			gs4_qc_highspeed_function,
			gs4_qc_fullspeed_function);
	} else {
		config_desc = &gs4_bulk_config_desc;
		function = GS4_SPEED_SELECT(high_speed,
			gs4_bulk_highspeed_function,
			gs4_bulk_fullspeed_function);
	}

	/* for now, don't advertise srp-only devices */
	if (!is_otg)
		function++;

	len = usb_gadget_config_buf(config_desc, buf, GS4_MAX_DESC_LEN, function);
	if (len < 0)
		return len;

	((struct usb_config_descriptor *)buf)->bDescriptorType = type;

	return len;
}
#endif /* !USB_COMPOSITE_DEVICE */

/*
 * gs4_alloc_req
 *
 * Allocate a usb_request and its buffer.  Returns a pointer to the
 * usb_request or NULL if there is an error.
 */
static struct usb_request *
gs4_alloc_req(struct usb_ep *ep, unsigned int len, gfp_t kmalloc_flags)
{
	struct usb_request *req;

	if (ep == NULL)
		return NULL;

	req = usb_ep_alloc_request(ep, kmalloc_flags);

	if (req != NULL) {
		req->length = len;
		req->buf = kmalloc(len, kmalloc_flags);
		if (req->buf == NULL) {
			usb_ep_free_request(ep, req);
			return NULL;
		}
	}

	return req;
}

/*
 * gs4_free_req
 *
 * Free a usb_request and its buffer.
 */
static void gs4_free_req(struct usb_ep *ep, struct usb_request *req)
{
	if (ep != NULL && req != NULL) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

/*
 * gs4_alloc_req_entry
 *
 * Allocates a request and its buffer, using the given
 * endpoint, buffer len, and kmalloc flags.
 */
static struct gs4_req_entry *
gs4_alloc_req_entry(struct usb_ep *ep, unsigned len, gfp_t kmalloc_flags)
{
	struct gs4_req_entry	*req;

	req = kmalloc(sizeof(struct gs4_req_entry), kmalloc_flags);
	if (req == NULL)
		return NULL;

	req->re_req = gs4_alloc_req(ep, len, kmalloc_flags);
	if (req->re_req == NULL) {
		kfree(req);
		return NULL;
	}

	req->re_req->context = req;

	return req;
}

/*
 * gs4_free_req_entry
 *
 * Frees a request and its buffer.
 */
static void gs4_free_req_entry(struct usb_ep *ep, struct gs4_req_entry *req)
{
	if (ep != NULL && req != NULL) {
		if (req->re_req != NULL)
			gs4_free_req(ep, req->re_req);
		kfree(req);
	}
}

/*
 * gs4_alloc_ports
 *
 * Allocate all ports and set the gs4_dev struct to point to them.
 * Return 0 if successful, or a negative error number.
 *
 * The device lock is normally held when calling this function.
 */
static int gs4_alloc_ports(struct gs4_dev *dev, gfp_t kmalloc_flags)
{
	int i;
	struct gs4_port *port;
	int j;

	if (dev == NULL)
		return -EIO;

	for (i=0; i<num_ports; i++) {
		if ((port=(struct gs4_port *)kzalloc(sizeof(struct gs4_port), kmalloc_flags)) == NULL)
			return -ENOMEM;

		port->port_dev = dev;
		port->port_num = i;
		port->port_line_coding.dwDTERate = cpu_to_le32(GS4_DEFAULT_DTE_RATE);
		port->port_line_coding.bCharFormat = GS4_DEFAULT_CHAR_FORMAT;
		port->port_line_coding.bParityType = GS4_DEFAULT_PARITY;
		port->port_line_coding.bDataBits = GS4_DEFAULT_DATA_BITS;
		spin_lock_init(&port->port_lock);
		init_waitqueue_head(&port->port_write_wait);

		INIT_LIST_HEAD(&port->port_spare_ctlreq_entries);
		INIT_LIST_HEAD(&port->port_filled_ctlreq_entries);
		for (j = 0; j < GS4_NCTLREQ; j++)
			list_add(&(port->port_ctlreq_entry[j].list), &port->port_spare_ctlreq_entries);
		init_waitqueue_head(&port->port_ctlreq_wait);

		dev->dev_port[i] = port;
	}

	return 0;
}

/*
 * gs4_free_ports
 *
 * Free all closed ports.  Open ports are disconnected by
 * freeing their write buffers, setting their device pointers
 * and the pointers to them in the device to NULL.  These
 * ports will be freed when closed.
 *
 * The device lock is normally held when calling this function.
 */
static void gs4_free_ports(struct gs4_dev *dev)
{
	int i;
	unsigned long flags;
	struct gs4_port *port;

	if (dev == NULL)
		return;

	for (i=0; i<num_ports; i++) {
		if ((port=dev->dev_port[i]) != NULL) {
			dev->dev_port[i] = NULL;

			spin_lock_irqsave(&port->port_lock, flags);

			if (port->port_write_buf != NULL) {
				gs4_buf_free(port->port_write_buf);
				port->port_write_buf = NULL;
			}

			if (port->port_open_count > 0 || port->port_in_use) {
				port->port_dev = NULL;
				wake_up_interruptible(&port->port_write_wait);
				wake_up_interruptible(&port->port_ctlreq_wait);
				if (port->port_tty) {
					dprintk("hangup port #%d\n", i);
					tty_hangup(port->port_tty);
					wake_up_interruptible(&port->port_tty->read_wait);
					wake_up_interruptible(&port->port_tty->write_wait);
				}
				spin_unlock_irqrestore(&port->port_lock, flags);
			} else {
				spin_unlock_irqrestore(&port->port_lock, flags);
				kfree(port);
			}

		}
	}
}

/* Circular Buffer */

/*
 * gs4_buf_alloc
 *
 * Allocate a circular buffer and all associated memory.
 */
static struct gs4_buf *gs4_buf_alloc(unsigned int size, gfp_t kmalloc_flags)
{
	struct gs4_buf *gb;

	if (size == 0)
		return NULL;

	gb = (struct gs4_buf *)kmalloc(sizeof(struct gs4_buf), kmalloc_flags);
	if (gb == NULL)
		return NULL;

	gb->buf_buf = kmalloc(size, kmalloc_flags);
	if (gb->buf_buf == NULL) {
		kfree(gb);
		return NULL;
	}

	gb->buf_size = size;
	gb->buf_get = gb->buf_put = gb->buf_buf;

	return gb;
}

/*
 * gs4_buf_free
 *
 * Free the buffer and all associated memory.
 */
void gs4_buf_free(struct gs4_buf *gb)
{
	if (gb) {
		kfree(gb->buf_buf);
		kfree(gb);
	}
}

/*
 * gs4_buf_clear
 *
 * Clear out all data in the circular buffer.
 */
void gs4_buf_clear(struct gs4_buf *gb)
{
	if (gb != NULL)
		gb->buf_get = gb->buf_put;
		/* equivalent to a get of all data available */
}

/*
 * gs4_buf_data_avail
 *
 * Return the number of bytes of data available in the circular
 * buffer.
 */
unsigned int gs4_buf_data_avail(struct gs4_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_put - gb->buf_get) % gb->buf_size;
	else
		return 0;
}

/*
 * gs4_buf_space_avail
 *
 * Return the number of bytes of space available in the circular
 * buffer.
 */
unsigned int gs4_buf_space_avail(struct gs4_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_get - gb->buf_put - 1) % gb->buf_size;
	else
		return 0;
}

/*
 * gs4_buf_put
 *
 * Copy data data from a user buffer and put it into the circular buffer.
 * Restrict to the amount of space available.
 *
 * Return the number of bytes copied.
 */
unsigned int gs4_buf_put(struct gs4_buf *gb, const char *buf, unsigned int count)
{
	unsigned int len;

	if (gb == NULL)
		return 0;

	len  = gs4_buf_space_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_put;
	if (count > len) {
		memcpy(gb->buf_put, buf, len);
		memcpy(gb->buf_buf, buf+len, count - len);
		gb->buf_put = gb->buf_buf + count - len;
	} else {
		memcpy(gb->buf_put, buf, count);
		if (count < len)
			gb->buf_put += count;
		else /* count == len */
			gb->buf_put = gb->buf_buf;
	}

	return count;
}

/*
 * gs4_buf_get
 *
 * Get data from the circular buffer and copy to the given buffer.
 * Restrict to the amount of data available.
 *
 * Return the number of bytes copied.
 */
unsigned int gs4_buf_get(struct gs4_buf *gb, char *buf, unsigned int count)
{
	unsigned int len;

	if (gb == NULL)
		return 0;

	len = gs4_buf_data_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_get;
	if (count > len) {
		memcpy(buf, gb->buf_get, len);
		memcpy(buf+len, gb->buf_buf, count - len);
		gb->buf_get = gb->buf_buf + count - len;
	} else {
		memcpy(buf, gb->buf_get, count);
		if (count < len)
			gb->buf_get += count;
		else /* count == len */
			gb->buf_get = gb->buf_buf;
	}

	return count;
}
