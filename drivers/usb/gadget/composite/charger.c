/*
 * charger.c
 *
 * Copyright 2008 (C) Palm,Inc.
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/moduleparam.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/gadget_event.h>

#include "composite.h"
#include "charger.h"

static int auto_maxpower = 0; /* disabled by default */
static int set_config_timeout = 700; /* ms */

static struct usb_composite_charger {
	spinlock_t lock;
	int detection_pending;
	int detection_enabled;
	struct delayed_work 	detection_work;
        struct workqueue_struct *detection_queue;
	struct usb_composite_dev * cdev;
	int try_current;
} charger;

static void re_enum(struct usb_composite_dev *cdev)
{
#if defined(CONFIG_TWL4030_USB_FS_3_PIN) && defined(CONFIG_ARCH_OMAP24XX)
	transceiver_reconnect();
#else
	usb_gadget_disconnect(cdev->gadget);
	msleep(20);
	usb_gadget_connect(cdev->gadget);
#endif
}

void charger_detection_start(struct usb_composite_dev *cdev)
{
	unsigned long flags;

	spin_lock_irqsave(&charger.lock, flags);
	if (charger.detection_pending)
		cancel_delayed_work(&charger.detection_work);

	if (!charger.detection_enabled) {
		spin_unlock_irqrestore(&charger.lock, flags);
		return;
	}
	if (charger.try_current == USB_CHARGE_CURRENT_100MA) {
		spin_unlock_irqrestore(&charger.lock, flags);
		return;
	}
	queue_delayed_work(charger.detection_queue,
			   &charger.detection_work,
			   msecs_to_jiffies(set_config_timeout));
	charger.detection_pending = 1;
	spin_unlock_irqrestore(&charger.lock, flags);
}
EXPORT_SYMBOL(charger_detection_start);

void charger_detection_stop(struct usb_composite_dev *cdev)
{
	unsigned long flags;

	spin_lock_irqsave(&charger.lock, flags);
	if (charger.detection_pending) {
		charger.detection_pending = 0;
		cancel_delayed_work(&charger.detection_work);
	}
	charger.try_current = USB_CHARGE_CURRENT_500MA;
	charger.detection_enabled = 0;
	spin_unlock_irqrestore(&charger.lock, flags);
}
EXPORT_SYMBOL(charger_detection_stop);

static void charger_detection_timeout(struct work_struct *work)
{
	struct usb_composite_dev *cdev = charger.cdev;
	unsigned long flags;

	spin_lock_irqsave(&charger.lock, flags);
	if (!charger.detection_pending) {
		/* already stopped */
		spin_unlock_irqrestore(&charger.lock, flags);
		return;
	}
	charger.detection_pending = 0;
	spin_unlock_irqrestore(&charger.lock, flags);

	/* detection failed */
	if (charger.try_current == USB_CHARGE_CURRENT_500MA) {
		charger.try_current = USB_CHARGE_CURRENT_100MA;
		printk(KERN_INFO "usb charging: re-enum for 100mA\n");
		re_enum(cdev); /* restart the detection */
	} else {
		charger.try_current = USB_CHARGE_CURRENT_500MA;
		charger.detection_enabled = 0;
	}
}

void charger_detection_enable(struct usb_composite_dev *cdev)
{
	unsigned long flags;

	if (!auto_maxpower) {
		return;
	}
	printk(KERN_INFO "%s: enabled\n", __func__);

	spin_lock_irqsave(&charger.lock, flags);
	charger.detection_enabled = 1;
	spin_unlock_irqrestore(&charger.lock, flags);
}
EXPORT_SYMBOL(charger_detection_enable);

int charger_detection_try_maxpower(void)
{
	return charger.detection_enabled ? charger.try_current : 0;
}
EXPORT_SYMBOL(charger_detection_try_maxpower);

static ssize_t show_auto_maxpower(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", auto_maxpower);
}

static ssize_t store_auto_maxpower(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%d", &i) != 1)
		return -EINVAL;

	auto_maxpower = !!i;
	printk(KERN_INFO "%s: auto_maxpower=%d\n", __func__, auto_maxpower);

	return count;
}

static DEVICE_ATTR(auto_maxpower, 0644, show_auto_maxpower, store_auto_maxpower);

static ssize_t show_set_config_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", set_config_timeout);
}

static ssize_t store_set_config_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%d", &i) != 1)
		return -EINVAL;

	set_config_timeout = i;
	printk(KERN_INFO "%s: set_config_timeout=%d\n", __func__, set_config_timeout);

	return count;
}

static DEVICE_ATTR(set_config_timeout, 0644, show_set_config_timeout, store_set_config_timeout);

int charger_detection_bind(struct usb_composite_dev *cdev)
{
	int rc;

	charger.detection_queue = create_singlethread_workqueue("usb_charger");
	if (!charger.detection_queue) {
		printk(KERN_ERR "%s: no memory\n", __func__);
                return -ENOMEM;
	}
	INIT_DELAYED_WORK(&charger.detection_work, charger_detection_timeout);
	charger.cdev = cdev;
	charger.detection_pending = 0;
	charger.detection_enabled = 0;
	charger.try_current = USB_CHARGE_CURRENT_500MA;

	rc = device_create_file(&cdev->gadget->dev, &dev_attr_auto_maxpower);
	if (rc != 0) {
		goto end;
	}
	rc = device_create_file(&cdev->gadget->dev, &dev_attr_set_config_timeout);
	if (rc != 0) {
		device_remove_file(&cdev->gadget->dev, &dev_attr_auto_maxpower);
		goto end;
	}
end:
	return rc;
}
EXPORT_SYMBOL(charger_detection_bind);

int charger_detection_unbind(struct usb_composite_dev *cdev)
{
	unsigned long flags;

	device_remove_file(&cdev->gadget->dev, &dev_attr_set_config_timeout);
	device_remove_file(&cdev->gadget->dev, &dev_attr_auto_maxpower);

	spin_lock_irqsave(&charger.lock, flags);
	if (charger.detection_pending) {
		charger.detection_pending = 0;
		cancel_delayed_work(&charger.detection_work);
	}
	charger.detection_enabled = 0;
	charger.cdev = NULL;
	spin_unlock_irqrestore(&charger.lock, flags);
	destroy_workqueue(charger.detection_queue);
	return 0;
}
EXPORT_SYMBOL(charger_detection_unbind);
