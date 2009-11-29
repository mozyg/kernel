/*
 * inode.c -- user mode filesystem api for usb gadget controllers
 *
 * Copyright (C) 2003-2004 David Brownell
 * Copyright (C) 2003 Agilent Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


// #define	DEBUG			/* data to help fault diagnosis */
// #define	VERBOSE		/* extra debug messages (success too) */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/uts.h>
#include <linux/wait.h>
#include <linux/compiler.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/poll.h>

#include <linux/device.h>
#include <linux/moduleparam.h>

#include <linux/usb/gadgetfs.h>
#include <linux/usb/gadget.h>

#include "composite.h"

#if 0
#define dprintk(format, args...)  \
       printk("%s %d: " format , __FUNCTION__, __LINE__, ## args)
#else
#define dprintk(format, args...)
#endif

/*
 * The gadgetfs API maps each endpoint to a file descriptor so that you
 * can use standard synchronous read/write calls for I/O.  There's some
 * O_NONBLOCK and O_ASYNC/FASYNC style i/o support.  Example usermode
 * drivers show how this works in practice.  You can also use AIO to
 * eliminate I/O gaps between requests, to help when streaming data.
 *
 * Key parts that must be USB-specific are protocols defining how the
 * read/write operations relate to the hardware state machines.  There
 * are two types of files.  One type is for the device, implementing ep0.
 * The other type is for each IN or OUT endpoint.  In both cases, the
 * user mode driver must configure the hardware before using it.
 *
 * - First, dev_config() is called when /dev/gadget/$CHIP is configured
 *   (by writing configuration and device descriptors).  Afterwards it
 *   may serve as a source of device events, used to handle all control
 *   requests other than basic enumeration.
 *
 * - Then, after a SET_CONFIGURATION control request, ep_config() is
 *   called when each /dev/gadget/ep* file is configured (by writing
 *   endpoint descriptors).  Afterwards these files are used to write()
 *   IN data or to read() OUT data.  To halt the endpoint, a "wrong
 *   direction" request is issued (like reading an IN endpoint).
 *
 * Unlike "usbfs" the only ioctl()s are for things that are rare, and maybe
 * not possible on all hardware.  For example, precise fault handling with
 * respect to data left in endpoint fifos after aborted operations; or
 * selective clearing of endpoint halts, to implement SET_INTERFACE.
 */

#define	DRIVER_DESC	"USB Gadget filesystem"
#define	DRIVER_VERSION	"24 Aug 2004"

static const char driver_desc [] = DRIVER_DESC;
static const char shortname [] = "gadgetfs";

MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_AUTHOR ("David Brownell");
MODULE_LICENSE ("GPL");

/*----------------------------------------------------------------------*/

#define	STRINGID_INTERFACE	20 /* REVISIT */

static struct usb_interface_descriptor
source_sink_intf = {
	.bLength =		sizeof source_sink_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	0x47,
	.bInterfaceProtocol =	0x11,
	.iInterface =		STRINGID_INTERFACE,
};

/* Full speed configurations are used for full-speed only devices as
 * well as dual-speed ones (the only kind with high speed support).
 */

static struct usb_endpoint_descriptor
fs_source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* NOTE some controllers may need FS bulk max packet size
	 * to be smaller.  it would be a chip-specific option.
	 */
	.wMaxPacketSize =	__constant_cpu_to_le16 (64),
};

static struct usb_endpoint_descriptor
fs_sink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (64),
};

/* some devices can handle other status packet sizes */
#define STATUS_MAXPACKET	8
#define	LOG2_STATUS_POLL_MSEC	3

static struct usb_endpoint_descriptor
fs_status_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16 (STATUS_MAXPACKET),
	.bInterval =		(1 << LOG2_STATUS_POLL_MSEC),
};


/* High speed configurations are used only in addition to a full-speed
 * ones ... since all high speed devices support full speed configs.
 * Of course, not all hardware supports high speed configurations.
 */

static struct usb_endpoint_descriptor
hs_source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (512),
};

static struct usb_endpoint_descriptor
hs_sink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (512),
	.bInterval =		1,
};

static struct usb_endpoint_descriptor
hs_status_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16 (STATUS_MAXPACKET),
	.bInterval =		LOG2_STATUS_POLL_MSEC + 3,
};

static const struct usb_descriptor_header *fs_function [] = {
	(struct usb_descriptor_header *) &source_sink_intf,
	(struct usb_descriptor_header *) &fs_source_desc,
	(struct usb_descriptor_header *) &fs_sink_desc,
	(struct usb_descriptor_header *) &fs_status_desc,
	NULL,
};

#ifdef CONFIG_USB_GADGET_DUALSPEED
static const struct usb_descriptor_header *hs_function [] = {
	(struct usb_descriptor_header *) &source_sink_intf,
	(struct usb_descriptor_header *) &hs_source_desc,
	(struct usb_descriptor_header *) &hs_sink_desc,
	(struct usb_descriptor_header *) &hs_status_desc,
	NULL,
};
#endif

/*----------------------------------------------------------------------*/

static struct usb_string strings [] = {
	{ STRINGID_INTERFACE,	"novacom linux", },
};

static struct usb_gadget_strings stringtab = {
	.language =	0x0409,		/* "en-us" */
	.strings =	strings,
};

/*----------------------------------------------------------------------*/

#define GADGETFS_MAGIC		0xaee71ee7
#define DMA_ADDR_INVALID	(~(dma_addr_t)0)

/* /dev/gadget/$CHIP represents ep0 and the whole device */
enum ep0_state {
	STATE_EP0_UNCONNECTED,
	STATE_EP0_CONNECTED,
};

enum dev_state {
	STATE_DEV_DISABLED = 0,
	STATE_DEV_CLOSED,
	STATE_DEV_OPENED,
	STATE_DEV_UNBOUND,
};

/* enough for the whole queue: most events invalidate others */
#define	N_EVENT			5

static enum ep0_state		ep0_state = STATE_EP0_UNCONNECTED;

struct dev_data {
	spinlock_t			lock;
	atomic_t			count;
	enum dev_state			state;	/* P: lock */
	struct usb_gadgetfs_event	event [N_EVENT];
	unsigned			ev_next;
	struct fasync_struct		*fasync;
	u8				current_config;
	struct usb_ctrlrequest		last_setconfig_ctrl; /* REVISIT */

	/* drivers reading ep0 MUST handle control requests (SETUP)
	 * reported that way; else the host will time out.
	 */
	unsigned			usermode_setup : 1,
					setup_in : 1,
					setup_can_stall : 1,
					setup_out_ready : 1,
					setup_out_error : 1,
					setup_abort : 1;
	unsigned			setup_wLength;

	/* the rest is basically write-once */
	struct usb_config_descriptor	*config, *hs_config;
	struct usb_device_descriptor	*dev;
	struct usb_request		*req;
	struct usb_gadget		*gadget;
	struct list_head		epfiles;
	void				*buf;
	wait_queue_head_t		wait;
	struct super_block		*sb;
	struct dentry			*dentry;

	struct usb_ep			*in_ep, *out_ep, *status_ep;
	const struct usb_endpoint_descriptor
					*in_desc, *out_desc, *status_desc;

	/* except this scratch i/o buffer for ep0 */
	u8				rbuf [256];
};

static inline void get_dev (struct dev_data *data)
{
	atomic_inc (&data->count);
}

static void put_dev (struct dev_data *data)
{
	if (likely (!atomic_dec_and_test (&data->count)))
		return;
	/* needs no more cleanup */
	BUG_ON (waitqueue_active (&data->wait));
	kfree (data);
}

static struct dev_data *dev_new (void)
{
	struct dev_data		*dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	dprintk("STATE_DEV_DISABLED\n");
	dev->state = STATE_DEV_DISABLED;
	atomic_set (&dev->count, 1);
	spin_lock_init (&dev->lock);
	INIT_LIST_HEAD (&dev->epfiles);
	init_waitqueue_head (&dev->wait);
	return dev;
}

/*----------------------------------------------------------------------*/

/* other /dev/gadget/$ENDPOINT files represent endpoints */
enum ep_state {
	STATE_EP_DISABLED = 0,
	STATE_EP_ENABLED,
};

struct ep_data {
	struct semaphore		lock;
	enum ep_state			state;
	atomic_t			count;
	struct dev_data			*dev;
	/* must hold dev->lock before accessing ep or req */
	struct usb_ep			*ep;
	struct usb_request		*req;
	ssize_t				status;
	char				name [16];
	struct usb_endpoint_descriptor	desc, hs_desc;
	struct list_head		epfiles;
	wait_queue_head_t		wait;
	struct dentry			*dentry;
	struct inode			*inode;
};

static inline void get_ep (struct ep_data *data)
{
	atomic_inc (&data->count);
}

static void put_ep (struct ep_data *data)
{
	if (likely (!atomic_dec_and_test (&data->count)))
		return;
	put_dev (data->dev);
	/* needs no more cleanup */
	BUG_ON (!list_empty (&data->epfiles));
	BUG_ON (waitqueue_active (&data->wait));
	kfree (data);
}

/*----------------------------------------------------------------------*/

/* most "how to use the hardware" policy choices are in userspace:
 * mapping endpoint roles (which the driver needs) to the capabilities
 * which the usb controller has.  most of those capabilities are exposed
 * implicitly, starting with the driver name and then endpoint names.
 */

static const char *CHIP;

/*----------------------------------------------------------------------*/

/* NOTE:  don't use dev_printk calls before binding to the gadget
 * at the end of ep0 configuration, or after unbind.
 */

/* too wordy: dev_printk(level , &(d)->gadget->dev , fmt , ## args) */
#define xprintk(d,level,fmt,args...) \
	printk(level "%s: " fmt , shortname , ## args)

#ifdef DEBUG
#define DBG(dev,fmt,args...) \
	xprintk(dev , KERN_DEBUG , fmt , ## args)
#else
#define DBG(dev,fmt,args...) \
	do { } while (0)
#endif /* DEBUG */

#ifdef VERBOSE
#define VDEBUG	DBG
#else
#define VDEBUG(dev,fmt,args...) \
	do { } while (0)
#endif /* DEBUG */

#define ERROR(dev,fmt,args...) \
	xprintk(dev , KERN_ERR , fmt , ## args)
#define WARN(dev,fmt,args...) \
	xprintk(dev , KERN_WARNING , fmt , ## args)
#define INFO(dev,fmt,args...) \
	xprintk(dev , KERN_INFO , fmt , ## args)

/*----------------------------------------------------------------------*/

#ifdef USB_COMPOSITE_DEVICE
static struct dev_data		*the_device;
#endif

/*----------------------------------------------------------------------*/

/* SYNCHRONOUS ENDPOINT OPERATIONS (bulk/intr/iso)
 *
 * After opening, configure non-control endpoints.  Then use normal
 * stream read() and write() requests; and maybe ioctl() to get more
 * precise FIFO status when recovering from cancellation.
 */

static void epio_complete (struct usb_ep *ep, struct usb_request *req)
{
	struct ep_data	*epdata = ep->driver_data;

	dprintk("enter\n");

	BUG_ON (ep == the_device->gadget->ep0);

	if (!req->context)
		return;
	if (req->status)
		epdata->status = req->status;
	else
		epdata->status = req->actual;
	complete ((struct completion *)req->context);
}

/* tasklock endpoint, returning when it's connected.
 * still need dev->lock to use epdata->ep.
 */
static int
get_ready_ep (unsigned f_flags, struct ep_data *epdata)
{
	int	val;

	if (f_flags & O_NONBLOCK) {
		if (down_trylock (&epdata->lock) != 0)
			goto nonblock;
		if (epdata->state != STATE_EP_ENABLED) {
			up (&epdata->lock);
nonblock:
			val = -EAGAIN;
		} else
			val = 0;
		return val;
	}

	if ((val = down_interruptible (&epdata->lock)) < 0)
		return val;

	switch (epdata->state) {
	case STATE_EP_ENABLED:
		break;
	case STATE_EP_DISABLED:
	default:				/* error! */
		dprintk ("%s: ep %p not available, state %d\n",
				shortname, epdata, epdata->state);
		val = -ENODEV;
		up (&epdata->lock);
	}
	return val;
}

static ssize_t
ep_io (struct ep_data *epdata, void *buf, unsigned len)
{
	DECLARE_COMPLETION_ONSTACK (done);
	int value;

	dprintk("enter len=%d\n", len);

	spin_lock_irq (&epdata->dev->lock);
	if (likely (epdata->ep != NULL)) {
		struct usb_request	*req = epdata->req;

		req->context = &done;
		req->complete = epio_complete;
		req->buf = buf;
		req->length = len;
		value = usb_ep_queue (epdata->ep, req, GFP_ATOMIC);
	} else
		value = -ENODEV;
	spin_unlock_irq (&epdata->dev->lock);

	if (likely (value == 0)) {
		value = wait_event_interruptible (done.wait, done.done);
		if (value != 0) {
			spin_lock_irq (&epdata->dev->lock);
			if (likely (epdata->ep != NULL)) {
				dprintk("%s i/o interrupted\n", epdata->name);
				DBG (epdata->dev, "%s i/o interrupted\n",
						epdata->name);
				usb_ep_dequeue (epdata->ep, epdata->req);
				spin_unlock_irq (&epdata->dev->lock);

				wait_event (done.wait, done.done);
				if (epdata->status == -ECONNRESET)
					epdata->status = -EINTR;
			} else {
				spin_unlock_irq (&epdata->dev->lock);

				dprintk("endpoint gone\n");
				DBG (epdata->dev, "endpoint gone\n");
				epdata->status = -ENODEV;
			}
		}
		return epdata->status;
	}
	return value;
}


/* handle a synchronous OUT bulk/intr/iso transfer */
static ssize_t
ep_read (struct file *fd, char __user *buf, size_t len, loff_t *ptr)
{
	struct ep_data		*data = fd->private_data;
	void			*kbuf;
	ssize_t			value;

	dprintk("enter\n");

	if ((value = get_ready_ep (fd->f_flags, data)) < 0)
		return value;

	/* halt any endpoint by doing a "wrong direction" i/o call */
	if (data->desc.bEndpointAddress & USB_DIR_IN) {
		if ((data->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_ISOC)
			return -EINVAL;
		DBG (data->dev, "%s halt\n", data->name);
		spin_lock_irq (&data->dev->lock);
		if (likely (data->ep != NULL))
			usb_ep_set_halt (data->ep);
		spin_unlock_irq (&data->dev->lock);
		up (&data->lock);
		return -EBADMSG;
	}

	/* FIXME readahead for O_NONBLOCK and poll(); careful with ZLPs */

	value = -ENOMEM;
	/* HACK:
 	 * kmalloc() with size 0 returns invalid ZERO_SIZEPTR.
 	 * dma_cache_maint() will BUG_ON() it -- tk
	 */
	if (len == 0)
		kbuf = kmalloc (1, GFP_KERNEL);
	else
		kbuf = kmalloc (len, GFP_KERNEL);
	if (unlikely (!kbuf))
		goto free1;

	value = ep_io (data, kbuf, len);
	VDEBUG (data->dev, "%s read %zu OUT, status %d\n",
		data->name, len, (int) value);
	if (value >= 0 && copy_to_user (buf, kbuf, value))
		value = -EFAULT;

free1:
	up (&data->lock);
	kfree (kbuf);
	return value;
}

/* handle a synchronous IN bulk/intr/iso transfer */
static ssize_t
ep_write (struct file *fd, const char __user *buf, size_t len, loff_t *ptr)
{
	struct ep_data		*data = fd->private_data;
	void			*kbuf;
	ssize_t			value;

	dprintk("enter\n");

	if ((value = get_ready_ep (fd->f_flags, data)) < 0)
		return value;

	/* halt any endpoint by doing a "wrong direction" i/o call */
	if (!(data->desc.bEndpointAddress & USB_DIR_IN)) {
		if ((data->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_ISOC)
			return -EINVAL;
		DBG (data->dev, "%s halt\n", data->name);
		spin_lock_irq (&data->dev->lock);
		if (likely (data->ep != NULL))
			usb_ep_set_halt (data->ep);
		spin_unlock_irq (&data->dev->lock);
		up (&data->lock);
		return -EBADMSG;
	}

	/* FIXME writebehind for O_NONBLOCK and poll(), qlen = 1 */
	value = -ENOMEM;

	/* HACK:
 	 * kmalloc() with size 0 returns invalid ZERO_SIZEPTR.
 	 * dma_cache_maint() will BUG_ON() it -- tk
	 */
	if (len == 0)
		kbuf = kmalloc (1, GFP_KERNEL);
	else
		kbuf = kmalloc (len, GFP_KERNEL);
	if (!kbuf)
		goto free1;
	if (copy_from_user (kbuf, buf, len)) {
		value = -EFAULT;
		goto free1;
	}

	value = ep_io (data, kbuf, len);
	dprintk ("%s write %zu IN, status %d\n", data->name, len, (int) value);
	VDEBUG (data->dev, "%s write %zu IN, status %d\n",
		data->name, len, (int) value);
free1:
	up (&data->lock);
	kfree (kbuf);
	return value;
}

static void disable_ep (struct ep_data *data);

static int
ep_release (struct inode *inode, struct file *fd)
{
	struct ep_data		*data = fd->private_data;

	dprintk("enter\n");

	disable_ep (data);
	put_ep (data);
	return 0;
}

static int ep_ioctl (struct inode *inode, struct file *fd,
		unsigned code, unsigned long value)
{
	struct ep_data		*data = fd->private_data;
	int			status;

	dprintk("enter\n");

	if ((status = get_ready_ep (fd->f_flags, data)) < 0)
		return status;

	spin_lock_irq (&data->dev->lock);
	if (likely (data->ep != NULL)) {
		switch (code) {
		case GADGETFS_FIFO_STATUS:
			status = usb_ep_fifo_status (data->ep);
			break;
		case GADGETFS_FIFO_FLUSH:
			usb_ep_fifo_flush (data->ep);
			break;
		case GADGETFS_CLEAR_HALT:
			status = usb_ep_clear_halt (data->ep);
			break;
		default:
			status = -ENOTTY;
		}
	} else
		status = -ENODEV;
	spin_unlock_irq (&data->dev->lock);
	up (&data->lock);
	return status;
}

/*----------------------------------------------------------------------*/

/* ASYNCHRONOUS ENDPOINT I/O OPERATIONS (bulk/intr/iso) */

struct kiocb_priv {
	struct usb_request	*req;
	struct ep_data		*epdata;
	void			*buf;
	const struct iovec	*iv;
	unsigned long		nr_segs;
	unsigned		actual;
};

static int ep_aio_cancel(struct kiocb *iocb, struct io_event *e)
{
	struct kiocb_priv	*priv = iocb->private;
	struct ep_data		*epdata;
	int			value;

	local_irq_disable();
	epdata = priv->epdata;
	// spin_lock(&epdata->dev->lock);
	kiocbSetCancelled(iocb);
	if (likely(epdata && epdata->ep && priv->req))
		value = usb_ep_dequeue (epdata->ep, priv->req);
	else
		value = -EINVAL;
	// spin_unlock(&epdata->dev->lock);
	local_irq_enable();

	aio_put_req(iocb);
	return value;
}

static ssize_t ep_aio_read_retry(struct kiocb *iocb)
{
	struct kiocb_priv	*priv = iocb->private;
	ssize_t			len, total;
	void			*to_copy;
	int			i;

	/* we "retry" to get the right mm context for this: */

	/* copy stuff into user buffers */
	total = priv->actual;
	len = 0;
	to_copy = priv->buf;
	for (i=0; i < priv->nr_segs; i++) {
		ssize_t this = min((ssize_t)(priv->iv[i].iov_len), total);

		if (copy_to_user(priv->iv[i].iov_base, to_copy, this)) {
			if (len == 0)
				len = -EFAULT;
			break;
		}

		total -= this;
		len += this;
		to_copy += this;
		if (total == 0)
			break;
	}
	kfree(priv->buf);
	kfree(priv);
	return len;
}

static void ep_aio_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct kiocb		*iocb = req->context;
	struct kiocb_priv	*priv = iocb->private;
	struct ep_data		*epdata = priv->epdata;

	/* lock against disconnect (and ideally, cancel) */
	spin_lock(&epdata->dev->lock);
	priv->req = NULL;
	priv->epdata = NULL;

	/* if this was a write or a read returning no data then we
	 * don't need to copy anything to userspace, so we can
	 * complete the aio request immediately.
	 */
	if (priv->iv == NULL || unlikely(req->actual == 0)) {
		kfree(req->buf);
		kfree(priv);
		iocb->private = NULL;
		/* aio_complete() reports bytes-transferred _and_ faults */
		aio_complete(iocb, req->actual ? req->actual : req->status,
				req->status);
	} else {
		/* retry() won't report both; so we hide some faults */
		if (unlikely(0 != req->status))
			DBG(epdata->dev, "%s fault %d len %d\n",
				ep->name, req->status, req->actual);

		priv->buf = req->buf;
		priv->actual = req->actual;
		kick_iocb(iocb);
	}
	spin_unlock(&epdata->dev->lock);

	usb_ep_free_request(ep, req);
	put_ep(epdata);
}

static ssize_t
ep_aio_rwtail(
	struct kiocb	*iocb,
	char		*buf,
	size_t		len,
	struct ep_data	*epdata,
	const struct iovec *iv,
	unsigned long	nr_segs
)
{
	struct kiocb_priv	*priv;
	struct usb_request	*req;
	ssize_t			value;

	priv = kmalloc(sizeof *priv, GFP_KERNEL);
	if (!priv) {
		value = -ENOMEM;
fail:
		kfree(buf);
		return value;
	}
	iocb->private = priv;
	priv->iv = iv;
	priv->nr_segs = nr_segs;

	value = get_ready_ep(iocb->ki_filp->f_flags, epdata);
	if (unlikely(value < 0)) {
		kfree(priv);
		goto fail;
	}

	iocb->ki_cancel = ep_aio_cancel;
	get_ep(epdata);
	priv->epdata = epdata;
	priv->actual = 0;

	/* each kiocb is coupled to one usb_request, but we can't
	 * allocate or submit those if the host disconnected.
	 */
	spin_lock_irq(&epdata->dev->lock);
	if (likely(epdata->ep)) {
		req = usb_ep_alloc_request(epdata->ep, GFP_ATOMIC);
		if (likely(req)) {
			priv->req = req;
			req->buf = buf;
			req->length = len;
			req->complete = ep_aio_complete;
			req->context = iocb;
			value = usb_ep_queue(epdata->ep, req, GFP_ATOMIC);
			if (unlikely(0 != value))
				usb_ep_free_request(epdata->ep, req);
		} else
			value = -EAGAIN;
	} else
		value = -ENODEV;
	spin_unlock_irq(&epdata->dev->lock);

	up(&epdata->lock);

	if (unlikely(value)) {
		kfree(priv);
		put_ep(epdata);
	} else
		value = (iv ? -EIOCBRETRY : -EIOCBQUEUED);
	return value;
}

static ssize_t
ep_aio_read(struct kiocb *iocb, const struct iovec *iov,
		unsigned long nr_segs, loff_t o)
{
	struct ep_data		*epdata = iocb->ki_filp->private_data;
	char			*buf;

	if (unlikely(epdata->desc.bEndpointAddress & USB_DIR_IN))
		return -EINVAL;

	buf = kmalloc(iocb->ki_left, GFP_KERNEL);
	if (unlikely(!buf))
		return -ENOMEM;

	iocb->ki_retry = ep_aio_read_retry;
	return ep_aio_rwtail(iocb, buf, iocb->ki_left, epdata, iov, nr_segs);
}

static ssize_t
ep_aio_write(struct kiocb *iocb, const struct iovec *iov,
		unsigned long nr_segs, loff_t o)
{
	struct ep_data		*epdata = iocb->ki_filp->private_data;
	char			*buf;
	size_t			len = 0;
	int			i = 0;

	if (unlikely(!(epdata->desc.bEndpointAddress & USB_DIR_IN)))
		return -EINVAL;

	buf = kmalloc(iocb->ki_left, GFP_KERNEL);
	if (unlikely(!buf))
		return -ENOMEM;

	for (i=0; i < nr_segs; i++) {
		if (unlikely(copy_from_user(&buf[len], iov[i].iov_base,
				iov[i].iov_len) != 0)) {
			kfree(buf);
			return -EFAULT;
		}
		len += iov[i].iov_len;
	}
	return ep_aio_rwtail(iocb, buf, len, epdata, NULL, 0);
}

/*----------------------------------------------------------------------*/

static int ep_open (struct inode *inode, struct file *fd);

/* used after endpoint configuration */
static const struct file_operations ep_io_operations = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,

	.open =		ep_open,
	.read =		ep_read,
	.write =	ep_write,
	.ioctl =	ep_ioctl,
	.release =	ep_release,

	.aio_read =	ep_aio_read,
	.aio_write =	ep_aio_write,
};

static int
ep_open (struct inode *inode, struct file *fd)
{
	struct ep_data		*data = inode->i_private;
	int			value = -EBUSY;

	dprintk("enter\n");

	if (down_interruptible (&data->lock) != 0)
		return -EINTR;
	spin_lock_irq (&data->dev->lock);
	if (data->state == STATE_EP_ENABLED) {
		value = 0;
		get_ep (data);
		fd->private_data = data;
		dprintk("%s opened\n", data->name);
		VDEBUG (data->dev, "%s opened\n", data->name);
	} else {
		dprintk("%s busy (data->state=%d)\n", data->name, data->state);
		DBG (data->dev, "%s state %d\n",
			data->name, data->state);
	}
	spin_unlock_irq (&data->dev->lock);
	up (&data->lock);
	return value;
}

/*----------------------------------------------------------------------*/

/* EP0 IMPLEMENTATION can be partly in userspace.
 *
 * Drivers that use this facility receive various events, including
 * control requests the kernel doesn't handle.  Drivers that don't
 * use this facility may be too simple-minded for real applications.
 */

static inline void ep0_readable (struct dev_data *dev)
{
	dprintk("enter\n");

	wake_up (&dev->wait);
	kill_fasync (&dev->fasync, SIGIO, POLL_IN);
}

#if 0
static void clean_req (struct usb_ep *ep, struct usb_request *req)
{
	struct dev_data		*dev = the_device;

	BUG_ON (ep != the_device->gadget->ep0);

	if (req->buf != dev->rbuf) {
		usb_ep_free_buffer (ep, req->buf, req->dma, req->length);
		req->buf = dev->rbuf;
		req->dma = DMA_ADDR_INVALID;
	}
	req->complete = epio_complete;
	dev->setup_out_ready = 0;
}

static void ep0_complete (struct usb_ep *ep, struct usb_request *req)
{
	struct dev_data		*dev = the_device;
	unsigned long		flags;
	int			free = 1;

	BUG_ON (ep != the_device->gadget->ep0);

	/* for control OUT, data must still get to userspace */
	spin_lock_irqsave(&dev->lock, flags);
	if (!dev->setup_in) {
		dev->setup_out_error = (req->status != 0);
		if (!dev->setup_out_error)
			free = 0;
		dev->setup_out_ready = 1;
		ep0_readable (dev);
	}

	/* clean up as appropriate */
	if (free && req->buf != &dev->rbuf)
		clean_req (ep, req);
	req->complete = epio_complete;
	spin_unlock_irqrestore(&dev->lock, flags);
}
#endif

static ssize_t
ep0_read (struct file *fd, char __user *buf, size_t len, loff_t *ptr)
{
	struct dev_data			*dev = fd->private_data;
	ssize_t				retval;
	enum ep0_state			state;

	dprintk("enter\n");

	spin_lock_irq (&dev->lock);

	state = ep0_state;

	if (len < sizeof dev->event [0]) {
		retval = -EINVAL;
		goto done;
	}
	len -= len % sizeof (struct usb_gadgetfs_event);

scan:
	/* return queued events right away */
	if (dev->ev_next != 0) {
		unsigned		n;

		n = len / sizeof (struct usb_gadgetfs_event);
		if (dev->ev_next < n)
			n = dev->ev_next;

		spin_unlock_irq (&dev->lock);
		len = n * sizeof (struct usb_gadgetfs_event);
		if (copy_to_user (buf, &dev->event, len))
			retval = -EFAULT;
		else
			retval = len;
		if (len > 0) {
			/* NOTE this doesn't guard against broken drivers;
			 * concurrent ep0 readers may lose events.
			 */
			spin_lock_irq (&dev->lock);
			if (dev->ev_next > n) {
				memmove(&dev->event[0], &dev->event[n],
					sizeof (struct usb_gadgetfs_event)
						* (dev->ev_next - n));
			}
			dev->ev_next -= n;
			spin_unlock_irq (&dev->lock);
		}
		return retval;
	}
	if (fd->f_flags & O_NONBLOCK) {
		retval = -EAGAIN;
		goto done;
	}

	switch (state) {
	default:
		dprintk("fail state=%d\n", state);
		DBG (dev, "fail %s, state %d\n", __FUNCTION__, state);
		retval = -ESRCH;
		break;
	case STATE_EP0_UNCONNECTED:
	case STATE_EP0_CONNECTED:
		spin_unlock_irq (&dev->lock);
		dprintk("wait\n");
		DBG (dev, "%s wait\n", __FUNCTION__);

		/* wait for events */
		retval = wait_event_interruptible (dev->wait,
				dev->ev_next != 0);
		if (retval < 0)
			return retval;
		spin_lock_irq (&dev->lock);
		goto scan;
	}

done:
	spin_unlock_irq (&dev->lock);
	return retval;
}

static struct usb_gadgetfs_event *
next_event (struct dev_data *dev, enum usb_gadgetfs_event_type type)
{
	struct usb_gadgetfs_event	*event;
	unsigned			i;

	dprintk("enter\n");

	switch (type) {
	/* these events purge the queue */
	case GADGETFS_DISCONNECT:
	case GADGETFS_CONNECT:
		dev->ev_next = 0;
		break;
	case GADGETFS_SETUP:		/* previous request timed out */
	case GADGETFS_SUSPEND:		/* same effect */
		/* these events can't be repeated */
		for (i = 0; i != dev->ev_next; i++) {
			if (dev->event [i].type != type)
				continue;
			DBG(dev, "discard old event[%d] %d\n", i, type);
			dev->ev_next--;
			if (i == dev->ev_next)
				break;
			/* indices start at zero, for simplicity */
			memmove (&dev->event [i], &dev->event [i + 1],
				sizeof (struct usb_gadgetfs_event)
					* (dev->ev_next - i));
		}
		break;
	default:
		BUG ();
	}
	dprintk("event[%d] = %d\n", dev->ev_next, type);
	VDEBUG(dev, "event[%d] = %d\n", dev->ev_next, type);
	event = &dev->event [dev->ev_next++];
	BUG_ON (dev->ev_next > N_EVENT);
	memset (event, 0, sizeof *event);
	event->type = type;
	return event;
}

static ssize_t
ep0_write (struct file *fd, const char __user *buf, size_t len, loff_t *ptr)
{
	//struct dev_data		*dev = fd->private_data;
	ssize_t			retval = -ESRCH;

	dprintk("do nothing\n");

	return retval;
}

static int
ep0_fasync (int f, struct file *fd, int on)
{
	struct dev_data		*dev = fd->private_data;
	// caller must F_SETOWN before signal delivery happens
	VDEBUG (dev, "%s %s\n", __FUNCTION__, on ? "on" : "off");
	return fasync_helper (f, fd, on, &dev->fasync);
}

static int activate_ep_files (struct dev_data *dev);
static void destroy_ep_files (struct dev_data *dev);

static int enable_ep (struct ep_data *data);

static int
dev_open (struct inode *inode, struct file *fd)
{
	struct dev_data		*dev = inode->i_private;
	int			value = -EBUSY;

	dprintk("enter dev=%p\n", dev);

	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_DEV_CLOSED) {
		if (activate_ep_files (dev) < 0)
			goto fail;
		
		/* dev->ev_next = 0; */
		dprintk("STATE_DEV_OPENED\n");
		dev->state = STATE_DEV_OPENED;
		fd->private_data = dev;
		get_dev (dev);
		value = 0;

		if (ep0_state == STATE_EP0_CONNECTED) {
			struct ep_data	*data;
			struct usb_gadgetfs_event	*event;

			list_for_each_entry (data, &dev->epfiles, epfiles) {
				value = enable_ep (data);
				if (value != 0) {
					printk("can't enable %s\n", data->name);
					goto fail;
				}
			}

			if (dev->current_config != 0) {
				/* REVISIT - fake connect & set_config */
				INFO (dev, "fake connect\n");
				event = next_event (dev, GADGETFS_CONNECT);
				event->u.speed = dev->gadget->speed;
				INFO (dev, "fake setup\n");
				event = next_event (dev, GADGETFS_SETUP);
				event->u.setup = dev->last_setconfig_ctrl;
				ep0_readable (dev);
			}
		}
	}
fail:
	spin_unlock_irq(&dev->lock);
	return value;
}

static int
dev_release (struct inode *inode, struct file *fd)
{
	struct dev_data		*dev = fd->private_data;
	struct ep_data  *data;

	/* closing ep0 === shutdown all */

	list_for_each_entry (data, &dev->epfiles, epfiles) {
		disable_ep (data);
	}

	destroy_ep_files (dev);

	/* at this point "good" hardware has disconnected the
	 * device from USB; the host won't see it any more.
	 * alternatively, all host requests will time out.
	 */

	fasync_helper (-1, fd, 0, &dev->fasync);
	kfree (dev->buf);
	dev->buf = NULL;
	put_dev (dev);

	/* other endpoints were all decoupled from this device */
	spin_lock_irq(&dev->lock);
	dprintk("STATE_DEV_CLOSED\n");
	dev->state = STATE_DEV_CLOSED;
	spin_unlock_irq(&dev->lock);
	return 0;
}

static unsigned int
ep0_poll (struct file *fd, poll_table *wait)
{
       struct dev_data         *dev = fd->private_data;
       int                     mask = 0;

       dprintk("enter\n");

       poll_wait(fd, &dev->wait, wait);

       spin_lock_irq (&dev->lock);

       /* report fd mode change before acting on it */
       if (dev->setup_abort) {
               dev->setup_abort = 0;
               mask = POLLHUP;
               goto out;
       }

       dprintk("dev->ev_next=%d\n", dev->ev_next);

       if (dev->ev_next != 0)
	       mask = POLLIN;
out:
       spin_unlock_irq(&dev->lock);
       return mask;
}

static int dev_ioctl (struct inode *inode, struct file *fd,
		unsigned code, unsigned long value)
{
	struct dev_data		*dev = fd->private_data;
	struct usb_gadget	*gadget = dev->gadget;

	if (gadget->ops->ioctl)
		return gadget->ops->ioctl (gadget, code, value);
	return -ENOTTY;
}

/* used after device configuration */
static const struct file_operations ep0_io_operations = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,

	.open =		dev_open,
	.read =		ep0_read,
	.write =	ep0_write,
	.fasync =	ep0_fasync,
	.poll =		ep0_poll,
	.ioctl =	dev_ioctl,
	.release =	dev_release,
};

/*----------------------------------------------------------------------*/

/* The in-kernel gadget driver handles most ep0 issues, in particular
 * enumerating the single configuration (as provided from user space).
 *
 * Unrecognized ep0 requests may be handled in user space.
 */

static int
enable_ep (struct ep_data *data)
{
	struct usb_ep *ep = data->ep;
#ifdef CONFIG_USB_GADGET_DUALSPEED
	struct dev_data *dev = data->dev;
	struct usb_gadget *gadget = dev->gadget;
	const struct usb_endpoint_descriptor *hs_desc = &data->hs_desc;
#endif
	const struct usb_endpoint_descriptor *desc = &data->desc;
	int value;

	dprintk("enter %s\n", ep->name);

	if (data->state == STATE_EP_DISABLED) {
#ifdef CONFIG_USB_GADGET_DUALSPEED
		if (gadget->speed == USB_SPEED_HIGH)
			desc = hs_desc;
#endif
		
		value = usb_ep_enable (ep, desc);
		if (value != 0) {
			dprintk("usb_ep_enable %s failed --> %d\n",
				ep->name, value);
			return value;
		}

		dprintk("%s: STATE_EP_ENABLED\n", data->name);
		data->state = STATE_EP_ENABLED;
	}
	return 0;
}

static void
disable_ep (struct ep_data *data)
{
	if (data->state == STATE_EP_ENABLED) {
		(void) usb_ep_disable (data->ep);
		dprintk("%s: STATE_EP_DISABLED\n", data->name);
		data->state = STATE_EP_DISABLED;
	}
}

static int set_config (struct dev_data *dev)
{
	struct ep_data	*data;
	int value = 0;

	dprintk("enter\n");
	
	list_for_each_entry (data, &dev->epfiles, epfiles) {
		value = enable_ep (data);
		if (value != 0) {
			printk("can't enable %s\n", data->name);
			goto fail;
		}
	}
	//INFO (dev, "configuration #%d\n", dev->current_config);
fail:
	return value;
}

static void reset_config (struct dev_data *dev)
{
	struct ep_data	*data;

	dprintk("enter\n");

	list_for_each_entry (data, &dev->epfiles, epfiles) {
		disable_ep (data);
	}
}

static int
gadgetfs_setup (struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
#ifdef USB_COMPOSITE_DEVICE
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
#endif
	struct dev_data			*dev = the_device;
	struct usb_request		*req;
	int				value = -EOPNOTSUPP;
	struct usb_gadgetfs_event	*event;
	u16				w_value = le16_to_cpu(ctrl->wValue);
	u16				w_length = le16_to_cpu(ctrl->wLength);

	if (!dev)
		return -EINVAL;
	dprintk("enter dev->state=%d\n", dev->state);

	req = dev->req;

	spin_lock (&dev->lock);
	dev->setup_abort = 0;

	if (ep0_state == STATE_EP0_UNCONNECTED) {
		dprintk("STATE_EP0_CONNECTED\n");
		ep0_state = STATE_EP0_CONNECTED;

		INFO (dev, "connected\n");

		if (dev->state == STATE_DEV_OPENED) {
			event = next_event (dev, GADGETFS_CONNECT);
			event->u.speed = dev->gadget->speed;
			ep0_readable (dev);
		}

	/* host may have given up waiting for response.  we can miss control
	 * requests handled lower down (device/endpoint status and features);
	 * then ep0_{read,write} will report the wrong status. controller
	 * driver will have aborted pending i/o.
	 */
	}

	req->buf = dev->rbuf;
	req->dma = DMA_ADDR_INVALID;
	req->context = NULL;
	value = -EOPNOTSUPP;
	switch (ctrl->bRequest) {

	/* currently one config, two speeds */
	case USB_REQ_SET_CONFIGURATION:
		dprintk("SET_CONFIGURATION(%d): wIndex=%d wValue=%d\n", ctrl->bRequest, le16_to_cpu(ctrl->wIndex), w_value);
		if (ctrl->bRequestType != 0)
			break;
		if (dev->current_config != 0) {
			dprintk("reset config\n");
			reset_config(dev);
		}
		dev->current_config = (u8) w_value;

		if (dev->current_config != 0) {
			value = set_config(dev);
		}
		/* REVISIT - if user program is not running... */
		event = next_event (dev, GADGETFS_SETUP);
		event->u.setup = *ctrl;
		ep0_readable(dev);

		dev->last_setconfig_ctrl = *ctrl; // REVISIT
		break;

	case USB_REQ_SET_INTERFACE:
		dprintk("SET_INTERFACE(%d): wIndex=%d wValue=%d\n", ctrl->bRequest, le16_to_cpu(ctrl->wIndex), w_value);

		reset_config(dev);
		value = set_config(dev);

		event = next_event (dev, GADGETFS_SETUP);
		event->u.setup = *ctrl;
		ep0_readable(dev);
		break;
#ifdef USB_COMPOSITE_DEVICE
	case USB_REQ_GET_INTERFACE:
		dprintk("GET_INTERFACE(%d): wIndex=%d wValue=%d\n", ctrl->bRequest, le16_to_cpu(ctrl->wIndex), w_value);
		*(u8 *)cdev->req->buf = 0; /* REVISIT */
		value = 1;
		break;
#endif
	default:
		dprintk("fail req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, le16_to_cpu(ctrl->wIndex), w_length);
		VDEBUG (dev, "%s req%02x.%02x v%04x i%04x l%d\n",
			dev->usermode_setup ? "delegate" : "fail",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, le16_to_cpu(ctrl->wIndex), w_length);
	}

#ifndef USB_COMPOSITE_DEVICE
	/* proceed with data transfer and status phases? */
	if (value >= 0 && dev->state != STATE_DEV_SETUP) {
		req->length = value;
		req->zero = value < w_length;
		value = usb_ep_queue (gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			DBG (dev, "ep_queue --> %d\n", value);
			req->status = 0;
		}
	}
#endif /* !USB_COMPOSITE_DEVICE */

	/* device stalls when value < 0 */
	spin_unlock (&dev->lock);
	return value;
}

static void destroy_ep_files (struct dev_data *dev)
{
	struct ep_data	*data;

	dprintk("enter\n");

	DBG (dev, "%s %d\n", __FUNCTION__, dev->state);

	list_for_each_entry (data, &dev->epfiles, epfiles) {
		struct inode	*parent;
		struct dentry	*dentry;

		if (!data->dentry)
			continue;

		dprintk("delete file for %s\n", data->ep->name);

		/* break link to FS */
		dentry = data->dentry;
		data->dentry = NULL;
		parent = dentry->d_parent->d_inode;

		/* break link to dcache */
		mutex_lock (&parent->i_mutex);
		d_delete (dentry);
		dput (dentry);
		mutex_unlock (&parent->i_mutex);

		/* REVISIT fds may still be open? */
		//goto restart;
	}
}


static struct inode *
gadgetfs_create_file (struct super_block *sb, char const *name,
		void *data, const struct file_operations *fops,
		struct dentry **dentry_p);

static int activate_ep_files (struct dev_data *dev)
{
	struct ep_data	*data;

	dprintk("enter\n");

	list_for_each_entry (data, &dev->epfiles, epfiles) {
		dprintk("create file for %s\n", data->ep->name);

		data->inode = gadgetfs_create_file (dev->sb, data->name,
				data, &ep_io_operations,
				&data->dentry);
		if (!data->inode)
			goto enomem0;
	}
	return 0;

enomem0:
	DBG (dev, "%s enomem\n", __FUNCTION__);
	destroy_ep_files (dev);
	return -ENOMEM;
}

struct usb_function gadgetfs_usb_function;

static int
gadgetfs_set_descriptors(int config, int is_otg)
{
	int interface = gadgetfs_usb_function.first_interface;

	source_sink_intf.bInterfaceNumber = interface;
	source_sink_intf.bNumEndpoints = 3;

	gadgetfs_usb_function.descriptors = fs_function;
#ifdef CONFIG_USB_GADGET_DUALSPEED
	gadgetfs_usb_function.hs_descriptors = hs_function;
#endif
	gadgetfs_usb_function.num_interfaces = 1;

	return 0;
}

static int
alloc_ep_data (struct dev_data *dev,
	       struct usb_ep *ep,
	       const struct usb_endpoint_descriptor *hs_desc,
	       const struct usb_endpoint_descriptor *desc)
{
	struct ep_data	*data;

	dprintk("enter %s\n", ep->name);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		goto enomem0;
	
	dprintk("%s: STATE_EP_DISABLED\n", ep->name);
	data->state = STATE_EP_DISABLED;
	init_MUTEX (&data->lock);
	init_waitqueue_head (&data->wait);
	
	strncpy (data->name, ep->name, sizeof (data->name) - 1);
	atomic_set (&data->count, 1);
	data->dev = dev;
	get_dev (dev);
	
	data->ep = ep;
	ep->driver_data = data;
	
	data->req = usb_ep_alloc_request (ep, GFP_KERNEL);
	if (!data->req) {
		dprintk("usb_ep_alloc_request failed\n");
		goto enomem1;
	}
	
	/* REVISIT - we don't have to copy */
	data->hs_desc = *hs_desc;
	data->desc = *desc;
	
	return 0;

enomem1:
	put_ep (data); /* putdev(data->dev) + kfree(data) */
enomem0:
	return -ENOMEM;
}

static void
destroy_ep_data (struct ep_data *data)
{
	usb_ep_free_request (data->ep, data->req);
	data->req = NULL;
	put_ep (data);
}

static void
gadgetfs_unbind (struct usb_gadget *gadget)
{
	struct dev_data		*dev = the_device;
	struct list_head	*entry, *tmp;

	DBG (dev, "%s\n", __FUNCTION__);

	spin_lock_irq (&dev->lock);
	dprintk("STATE_DEV_UNBOUND\n");
	dev->state = STATE_DEV_UNBOUND;
	spin_unlock_irq (&dev->lock);

	/* destroy ep_data */
	list_for_each_safe (entry, tmp, &dev->epfiles) {
		struct ep_data	*data;

		data = list_entry (entry, struct ep_data, epfiles);
		dprintk("destroy epdata for %s\n", data->name);
		list_del_init (&data->epfiles);
		disable_ep(data);
		destroy_ep_data (data);
	}	

	/* we've already been disconnected ... no i/o is active */
	if (dev->req)
		usb_ep_free_request (gadget->ep0, dev->req);
	DBG (dev, "%s done\n", __FUNCTION__);
	put_dev (dev);
	the_device = NULL;
}

static int
gadgetfs_bind (struct usb_gadget *gadget)
{
	struct dev_data		*dev;
	struct ep_data		*data;

	dprintk("enter\n");
	CHIP = gadget->name;

	/* create dev */
	dev = dev_new ();
	if (!dev)
		return -ENOMEM;
	dev->gadget = gadget;
	the_device = dev; /* REVISIT : should use cdev_data?  */

	/* preallocate control response and buffer */
	dev->req = usb_ep_alloc_request (gadget->ep0, GFP_KERNEL);
	if (!dev->req)
		goto enomem;
	dev->req->context = NULL;
	dev->req->complete = epio_complete;

	/* config in-ep */
	dev->in_ep = usb_ep_autoconfig (gadget, &fs_source_desc);
	if (!dev->in_ep) {
		printk("can't autoconfigure in_ep\n");
		goto enomem;
	}
	hs_source_desc.bEndpointAddress = fs_source_desc.bEndpointAddress;
	if (alloc_ep_data (dev, dev->in_ep, 
			   &hs_source_desc, &fs_source_desc) != 0) {
		printk("can't alloc ep_data for in_ep\n");
		goto enomem;
	}
	data = dev->in_ep->driver_data;
	list_add_tail (&data->epfiles, &dev->epfiles);

	/* config out-ep */
	dev->out_ep = usb_ep_autoconfig (gadget, &fs_sink_desc);
	if (!dev->out_ep) {
		printk("can't autoconfigure out_ep\n");
		goto enomem;
	}
	hs_sink_desc.bEndpointAddress = fs_sink_desc.bEndpointAddress;
	if (alloc_ep_data (dev, dev->out_ep, 
			   &hs_sink_desc, &fs_sink_desc) != 0) {
		printk("can't alloc ep_data for out_ep\n");
		goto enomem;
	}
	data = dev->out_ep->driver_data;
	list_add_tail (&data->epfiles, &dev->epfiles);

	/* config status-ep */
	dev->status_ep = usb_ep_autoconfig (gadget, &fs_status_desc);
	if (!dev->status_ep) {
		printk("can't autoconfigure status_ep\n");
		goto enomem;
	}
	hs_status_desc.bEndpointAddress = fs_status_desc.bEndpointAddress;
	if (alloc_ep_data (dev, dev->status_ep, 
			   &hs_status_desc, &fs_status_desc) != 0) {
		printk("can't alloc ep_data for status_ep\n");
		goto enomem;
	}
	data = dev->status_ep->driver_data;
	list_add_tail (&data->epfiles, &dev->epfiles);

	INFO (dev, "using %s, OUT %s IN %s%s%s\n", gadget->name,
	      dev->out_ep->name, dev->in_ep->name,
	      dev->status_ep ? " STATUS " : "",
	      dev->status_ep ? dev->status_ep->name : ""
		);

	dprintk("STATE_DEV_CLOSED\n");
	dev->state = STATE_DEV_CLOSED;

	return 0;

enomem:
	gadgetfs_unbind (gadget);
	return -ENOMEM;
}

static void
gadgetfs_disconnect (struct usb_gadget *gadget)
{
	struct dev_data		*dev = the_device;

	dprintk("enter\n");

	spin_lock (&dev->lock);
	if (ep0_state == STATE_EP0_UNCONNECTED)
		goto exit;
	dprintk("STATE_EP0_UNCONNECTED\n");
	ep0_state = STATE_EP0_UNCONNECTED;

	INFO (dev, "disconnected\n");
	next_event (dev, GADGETFS_DISCONNECT);
	ep0_readable (dev);
exit:
	spin_unlock (&dev->lock);
}

static void
gadgetfs_suspend (struct usb_gadget *gadget)
{
	struct dev_data		*dev = the_device;

	INFO (dev, "suspended from state %d\n", ep0_state);
	spin_lock (&dev->lock);
	switch (ep0_state) {
	case STATE_EP0_CONNECTED:
	case STATE_EP0_UNCONNECTED:
		next_event (dev, GADGETFS_SUSPEND);
		ep0_readable (dev);
		/* FALLTHROUGH */
	default:
		break;
	}
	spin_unlock (&dev->lock);
}

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

/* FILESYSTEM AND SUPERBLOCK OPERATIONS
 *
 * Mounting the filesystem creates a controller file, used first for
 * device configuration then later for event monitoring.
 */


/* FIXME PAM etc could set this security policy without mount options
 * if epfiles inherited ownership and permissons from ep0 ...
 */

static unsigned default_uid;
static unsigned default_gid;
static unsigned default_perm = S_IRUSR | S_IWUSR;

module_param (default_uid, uint, 0644);
module_param (default_gid, uint, 0644);
module_param (default_perm, uint, 0644);


static struct inode *
gadgetfs_make_inode (struct super_block *sb,
		void *data, const struct file_operations *fops,
		int mode)
{
	struct inode *inode = new_inode (sb);

	if (inode) {
		inode->i_mode = mode;
		inode->i_uid = default_uid;
		inode->i_gid = default_gid;
		inode->i_blocks = 0;
		inode->i_atime = inode->i_mtime = inode->i_ctime
				= CURRENT_TIME;
		inode->i_private = data;
		inode->i_fop = fops;
	}
	return inode;
}

/* creates in fs root directory, so non-renamable and non-linkable.
 * so inode and dentry are paired, until device reconfig.
 */
static struct inode *
gadgetfs_create_file (struct super_block *sb, char const *name,
		void *data, const struct file_operations *fops,
		struct dentry **dentry_p)
{
	struct dentry	*dentry;
	struct inode	*inode;

	dentry = d_alloc_name(sb->s_root, name);
	if (!dentry)
		return NULL;

	inode = gadgetfs_make_inode (sb, data, fops,
			S_IFREG | (default_perm & S_IRWXUGO));
	if (!inode) {
		dput(dentry);
		return NULL;
	}
	d_add (dentry, inode);
	*dentry_p = dentry;
	return inode;
}

static struct super_operations gadget_fs_operations = {
	.statfs =	simple_statfs,
	.drop_inode =	generic_delete_inode,
};

static int
gadgetfs_fill_super (struct super_block *sb, void *opts, int silent)
{
	struct inode	*inode;
	struct dentry	*d;
	struct dev_data	*dev;

	dprintk("enter CHIP=%s\n", CHIP);

	/* superblock */
	sb->s_blocksize = PAGE_CACHE_SIZE;
	sb->s_blocksize_bits = PAGE_CACHE_SHIFT;
	sb->s_magic = GADGETFS_MAGIC;
	sb->s_op = &gadget_fs_operations;
	sb->s_time_gran = 1;

	/* root inode */
	inode = gadgetfs_make_inode (sb,
			NULL, &simple_dir_operations,
			S_IFDIR | S_IRUGO | S_IXUGO);
	if (!inode)
		goto enomem0;
	inode->i_op = &simple_dir_inode_operations;
	if (!(d = d_alloc_root (inode)))
		goto enomem1;
	sb->s_root = d;

	/* the ep0 file is named after the controller we expect;
	 * user mode code can use it for sanity checks, like we do.
	 */
	dev = the_device;
	if (!dev)
		goto enomem2;

	dev->sb = sb;
	if (!gadgetfs_create_file (sb, CHIP,
				dev, &ep0_io_operations,
				&dev->dentry))
		goto enomem3;

	return 0;

enomem3:
	//put_dev (dev); // REVISIT
enomem2:
	dput (d);
enomem1:
	iput (inode);
enomem0:
	return -ENOMEM;
}

/* "mount -t gadgetfs path /dev/gadget" ends up here */
static int
gadgetfs_get_sb (struct file_system_type *t, int flags,
		const char *path, void *opts, struct vfsmount *mnt)
{
	return get_sb_single (t, flags, opts, gadgetfs_fill_super, mnt);
}

static void
gadgetfs_kill_sb (struct super_block *sb)
{
	kill_litter_super (sb);
}

/*----------------------------------------------------------------------*/

static struct file_system_type gadgetfs_type = {
	.owner		= THIS_MODULE,
	.name		= shortname,
	.get_sb		= gadgetfs_get_sb,
	.kill_sb	= gadgetfs_kill_sb,
};

/*----------------------------------------------------------------------*/

static int /* __init */ init (void)
{
	int status;

	status = register_filesystem (&gadgetfs_type);
	if (status == 0)
		pr_info ("%s: %s, version " DRIVER_VERSION "\n",
			shortname, driver_desc);
	return status;
}
/* module_init (init); */

static void /* __exit */ cleanup (void)
{
	pr_debug ("unregister %s\n", shortname);
	unregister_filesystem (&gadgetfs_type);
}
/* module_exit (cleanup); */

/* USB_FUNCTION */
struct usb_function gadgetfs_usb_function = {
	.name		= shortname,
	.strings	= &stringtab,
	.init		= init,
	.exit		= cleanup,
	.bind		= gadgetfs_bind,
	.unbind		= gadgetfs_unbind,
	.set_descriptors= gadgetfs_set_descriptors,
	.setup		= gadgetfs_setup,
	.disconnect	= gadgetfs_disconnect,
	.suspend	= gadgetfs_suspend,
	//.resume	= gadgetfs_resume,
};
EXPORT_SYMBOL(gadgetfs_usb_function);
