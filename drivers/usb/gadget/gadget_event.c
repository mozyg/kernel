/*
 * drivers/misc/gadget_event.c
 *
 * Copyright (C) 2008 Palm, Inc.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/version.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/gadget_event.h>

#define CHARGING_DEBUG

enum {
	SOURCE_NONE = 0,
	SOURCE_BUS,
	SOURCE_CHARGER,
};

struct usb_gadget_event_state {
	/* mass storage */
	int				host_connected_1;
	int				host_connected;
	struct work_struct		host_connected_work;
	int				media_loaded_1;
	int				media_loaded;
	struct work_struct		media_loaded_work;
	int				media_requested_1;
	int				media_requested;
	struct work_struct		media_requested_work;
	struct work_struct		reconnect_work;
	/* charger */
	int				enabled;
	int				vbus;
	int				mA;
	int				source;
	int				new_mA;
	struct workqueue_struct		*work_queue;
	struct work_struct		disable_work;
	struct work_struct		vbus_presence_work;
	struct work_struct		vbus_draw_work;
	struct work_struct		suspend_work;
	/* REVISIT */
	struct usb_gadget		*gadget;
};

static struct usb_gadget_event_state *the_gadget_event_state = NULL;

#define PULLUP_POLLING_LIMIT			(100)
#define SINGLE_ENDED_STATE_POLLING_LIMIT	(5000)

static struct platform_device usb_gadget_event_device = {
	.name = USB_GADGET_EVENT_NAME,
	.id = -1,
};

/*********************************************************************
 * 'host_connected' file
 */
static ssize_t
show_host_connected(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	int host_connected = 0;

	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state)
		host_connected = state->host_connected;
	return snprintf(buf, PAGE_SIZE, "%d\n", host_connected);
}

static DEVICE_ATTR(host_connected, S_IRUGO, show_host_connected, NULL);

/*********************************************************************
 * 'reconnect' file
 */

static void
reconnect(void)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state && state->gadget) {
#if defined(CONFIG_TWL4030_USB_FS_3_PIN) && defined(CONFIG_ARCH_OMAP24XX)
		transceiver_reconnect();
#else
		usb_gadget_disconnect(state->gadget);
		msleep(20);
		usb_gadget_connect(state->gadget);
#endif
	}
}

static ssize_t
store_reconnect(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	if (buf[0] == '1')
		reconnect();

	return count;
}

static DEVICE_ATTR(reconnect, S_IWUSR, NULL, store_reconnect);

static void
usb_gadget_event_reconnect_callback(struct work_struct *work)
{
	printk(KERN_INFO "%s:\n", __func__);
	reconnect();
}

int usb_gadget_event_reconnect(void)
{
	int ret = -ENODEV;
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state) {
		queue_work(state->work_queue,
			   &state->reconnect_work);
		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(usb_gadget_event_reconnect);

/*********************************************************************
 * 'media_loaded' file
 */
static ssize_t
show_media_loaded(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	int media_loaded = 0;

	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state)
		media_loaded = state->media_loaded;
	return snprintf(buf, PAGE_SIZE, "%d\n", media_loaded);
}

static DEVICE_ATTR(media_loaded, S_IRUGO, show_media_loaded, NULL);

static void
usb_gadget_event_media_loaded_callback(struct work_struct *work)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state->media_loaded != state->media_loaded_1) {
		char *var = state->media_loaded_1 ? 
			"G_MEDIA_LOADED=1" : "G_MEDIA_LOADED=0";
		char *envp[] = {
			"G_SUBSYSTEM=storage", 
			"G_ACTION=MEDIA_STATE_CHANGED",
			var,
			NULL
		};

		state->media_loaded = state->media_loaded_1;
		printk(KERN_INFO "%s: UEVENT media_loaded=%d\n",
		       __func__, state->media_loaded);
		kobject_uevent_env(&usb_gadget_event_device.dev.kobj,
				   KOBJ_CHANGE, envp);
	}
}

int usb_gadget_event_media_loaded(int loaded)
{
	int ret = -ENODEV;
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state) {
		state->media_loaded_1 = loaded;
		smp_mb();
		queue_work(state->work_queue,
			   &state->media_loaded_work);
		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(usb_gadget_event_media_loaded);

/*********************************************************************
 * 'media_requested' file
 */
static ssize_t
show_media_requested(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	int media_requested = 0;

	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state)
		media_requested = state->media_requested;
	return snprintf(buf, PAGE_SIZE, "%d\n", media_requested);
}

static ssize_t
store_media_requested(struct device *dev, struct device_attribute *attr,
		      const char *buf, size_t count)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;
	int i;

	if (sscanf(buf, "%d", &i) != 1)
		return -EINVAL;

	if (state)
		state->media_requested = !!i;

	return count;
}

static DEVICE_ATTR(media_requested, S_IRUGO|S_IWUSR, show_media_requested, store_media_requested);

static void
usb_gadget_event_media_requested_callback(struct work_struct *work)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state->media_requested != state->media_requested_1) {
		char *var = state->media_requested_1 ?
			"G_MEDIA_REQUESTED=1" : "G_MEDIA_REQUESTED=0";
		char *envp[] = {
			"G_SUBSYSTEM=storage", 
			"G_ACTION=MEDIA_REQUEST_STATE_CHANGED",
			var,
			NULL
		};

		state->media_requested = state->media_requested_1;
		printk(KERN_INFO "%s: UEVENT media_requested=%d\n",
		       __func__, state->media_requested);
		kobject_uevent_env(&usb_gadget_event_device.dev.kobj,
				   KOBJ_CHANGE, envp);
	}
}

int usb_gadget_event_media_requested(int requested)
{
	int ret = -ENODEV;
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state) {
		state->media_requested_1 = requested;
		smp_mb();
		queue_work(state->work_queue,
			   &state->media_requested_work);
		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(usb_gadget_event_media_requested);

/*********************************************************************
 * 'cuurent_mA' file
 */
static ssize_t
show_current_mA(struct device *dev, struct device_attribute *attr,
	     char *buf)
{
	int mA = 0;
	int retval;
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state)
		mA = state->mA;

	retval = snprintf(buf, PAGE_SIZE, "%d\n", mA);
	return retval;
}

static DEVICE_ATTR(current_mA, S_IRUGO, show_current_mA, NULL);

/*********************************************************************
 * 'source' file
 */
static char*
source_to_string(int source)
{
	switch (source) {
	case SOURCE_NONE:
		return ("none");
	case SOURCE_BUS:
		return ("bus");
	case SOURCE_CHARGER:
		return ("charger");
	default:
		return ("unknown");
	}
}

static ssize_t
show_source(struct device *dev, struct device_attribute *attr,
	    char *buf)
{
	int source = SOURCE_NONE;
	int retval;
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state)
		source = state->source;

	retval = snprintf(buf, PAGE_SIZE, "%s\n", source_to_string(source));
	return retval;
}

static DEVICE_ATTR(source, S_IRUGO, show_source, NULL);

/*********************************************************************
 * usb_gadget_event_vbus_presence()
 */
static int
detect_wall_charger(void)
{
	unsigned long start;
	unsigned int samples;
	int elapsed;
	int pullup = 0;
	int mA = 0;
	int dplus = 1;
	int dminus = 0;

	/*
	 * Poll the pull-up state.
	 * high-speed controller won't attach the D+ pull-up
	 * until a vbus presence is detected.
	 */
	start = jiffies;
	samples = 0;
	for (;;) {
		transceiver_is_pullup_attached(&pullup);
		elapsed = jiffies_to_msecs(jiffies - start);
		samples++;
		if (pullup) {
			break;
		}
		if (elapsed >= PULLUP_POLLING_LIMIT)
			break;
		msleep(1);
	}

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s: pullup=%d (samples=%u elapsed=%u)\n",
	       __func__, pullup, samples, elapsed);
#endif
	if (!pullup)
		return 0; /* not detected */

	/*
	 * Poll the data line state.
	 */
	start = jiffies;
	samples = 0;
	for (;;) {
		transceiver_single_ended_state(&dplus, &dminus);
		elapsed = jiffies_to_msecs(jiffies - start);
		samples++;
		if (dplus && dminus) {
			/* both D+ and D- are high */
			mA = 1000;
			break;
		}
		if (!dplus && !dminus)
			break;
		if (elapsed >= SINGLE_ENDED_STATE_POLLING_LIMIT) {
			if (dplus && !dminus) {
				/* This charger doesn't conform to the USB battery 
				 * charging spec. Charge at 500 mA. There shouldn't be
				 * danger of overcurrenting a host because a host is
				 * required to ground D+ and D-. */
				mA = 500;
			}
			break;
		}
		msleep(5);
	}

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s: mA=%d D+=%d D-=%d (samples=%u elapsed=%u)\n",
	       __func__, mA, dplus, dminus, samples, elapsed);
#endif
	return mA;
}

extern void musb_g_disconnect_HACK(struct usb_gadget *gadget);

static void
usb_gadget_event_vbus_presence_callback(struct work_struct *work)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;
	int prev_vbus = state->vbus;
	int prev_mA = state->mA;
	int new_vbus = 0;

	if (!state->enabled) {
		printk(KERN_INFO "%s: not enabled. ignored\n", __func__);
		return;
	}

	transceiver_vbus_presence(&new_vbus);
#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s: prev_vbus=%d new_vbus=%d\n", __func__,
	       prev_vbus, new_vbus);
#endif
	state->vbus = new_vbus;
	state->mA = 0;
	state->source = SOURCE_NONE;

	if (!prev_vbus && new_vbus) {
		int new_mA = detect_wall_charger();
		if (new_mA) {
			state->mA = new_mA;
			state->source = SOURCE_CHARGER;
		}
	}

	/* Potentially fire the host_connected=0 event here. This
	 * function modifies state->source so the condition to check
	 * the previous source == bus in
	 * usb_gadget_event_vbus_draw_callback() will always fail,
	 * and the host_connected=0 event will never fire.
	 * Not sure if this is the best way to do this. Probably need
	 * significant rethinking of gadget event to clean up... */
	if (!new_vbus && state->host_connected) {
		char *var = "G_HOST_CONNECTED=0";
		char *envp[] = {
			"G_SUBSYSTEM=storage",
			"G_ACTION=HOST_STATE_CHANGED",
			var,
			NULL
		};

		state->host_connected = 0;
		printk(KERN_INFO "%s: UEVENT host_connected=%d\n",
		       __func__, state->host_connected);
		kobject_uevent_env(&usb_gadget_event_device.dev.kobj,
				   KOBJ_CHANGE, envp);
	}

	if (prev_mA != state->mA) {
		char var_source[32];
		char var_current[32];
		char *envp[] = {
			"G_SUBSYSTEM=power",
			"G_ACTION=POWER_STATE_CHANGED",
			var_source,
			var_current,
			NULL
		};
		sprintf(var_source,
			"G_POWER_SOURCE=%s", source_to_string(state->source));
		sprintf(var_current, "G_CURRENT_MA=%d", state->mA);

		printk(KERN_INFO "%s: UEVENT source=%s mA=%d\n",
		       __func__, source_to_string(state->source), state->mA);
		kobject_uevent_env(&usb_gadget_event_device.dev.kobj,
				   KOBJ_CHANGE, envp);
	}

	/* UGH! Hack alert!
	 * It seems like if the PHY was suspended before the musb
	 * disconnect interrupt, the musb interrupt will happen when the
	 * PHY is resumed again. There is a race condition between the PHY
	 * disconnect interrupt, upon which we suspend the PHY, and the musb
	 * disconnect interrupt. Due to the way the drivers are structured,
	 * there's no way for the PHY to tell the musb, and the gadgets bound
	 * to the musb, of the disconnect, therefore I wrote this terrible 
	 * hack. If you think of a better way to do this, PLEASE fix it. */
	if (prev_vbus && !new_vbus) {
		musb_g_disconnect_HACK(state->gadget);
	}
}

int
usb_gadget_event_vbus_presence(void)
{
	int ret = -ENODEV;
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state) {
		queue_work(state->work_queue, &state->vbus_presence_work);
		ret = 0;
	}

	return (ret);
}
EXPORT_SYMBOL(usb_gadget_event_vbus_presence);

/*********************************************************************
 * usb_gadget_event_enable()
 */
static void
usb_gadget_event_disable_callback(struct work_struct *work)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;
	int prev_mA = state->mA;

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s: prev_mA=%d\n", __func__, prev_mA);
#endif
	state->vbus = 0;
	state->mA = 0;
	state->source = SOURCE_NONE;

	if (prev_mA != state->mA) {
		char var_source[32];
		char var_current[32];
		char *envp[] = {
			"G_SUBSYSTEM=power",
			"G_ACTION=POWER_STATE_CHANGED",
			var_source,
			var_current,
			NULL
		};
		sprintf(var_source,
			"G_POWER_SOURCE=%s", source_to_string(state->source));
		sprintf(var_current, "G_CURRENT_MA=%d", state->mA);

		printk(KERN_INFO "%s: UEVENT source=%s mA=%d\n",
		       __func__, source_to_string(state->source), state->mA);
		kobject_uevent_env(&usb_gadget_event_device.dev.kobj,
				   KOBJ_CHANGE, envp);
	}

	state->enabled = 0;
}

int
usb_gadget_event_enable(int enable)
{
	int ret = -ENODEV;
	struct usb_gadget_event_state *state = the_gadget_event_state;

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s: enable=%d\n", __func__, enable);
#endif
	if (state) {
		if (!state->enabled && enable) {
			/* Enabled. Check vbus presence */
			state->enabled = 1;
			queue_work(state->work_queue,
				   &state->vbus_presence_work);
		} else if (state->enabled && !enable) {
			/* Disabled. state->enabled will be 0 */
			queue_work(state->work_queue, &state->disable_work);
		}
		ret = 0;
	}

	return (ret);
}
EXPORT_SYMBOL(usb_gadget_event_enable);

/*********************************************************************
 * usb_gadget_event_vbus_draw()
 */
static void
usb_gadget_event_vbus_draw_callback(struct work_struct *work)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;
	int prev_mA = state->mA;
	int prev_source = state->source;
	int new_mA = state->new_mA;

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s: new_mA=%d\n", __func__, new_mA);
#endif
	if (!state->enabled) {
		printk(KERN_INFO "%s: not enabled. ignored\n", __func__);
		return;
	}
	if (!state->vbus) {
		printk(KERN_INFO "%s: vbus not presence. new mA is ignored\n",
		       __func__);
		return;
	}
	if ((prev_source != SOURCE_BUS) && (new_mA == 0)) {
		/*
		 * Ignore attempts to disable charging if source is not bus
		 */
		printk(KERN_INFO "%s: source is not bus. new mA=0 is ignored\n",
		       __func__);
		return;
	}

	state->mA = new_mA;
	state->source = new_mA > 0 ? SOURCE_BUS : SOURCE_NONE;

	if ((prev_source == SOURCE_BUS && state->source != SOURCE_BUS)
		|| (prev_source != SOURCE_BUS && state->source == SOURCE_BUS)) {
		char *var = (state->source == SOURCE_BUS) ? 
			"G_HOST_CONNECTED=1" : "G_HOST_CONNECTED=0";
		char *envp[] = {
			"G_SUBSYSTEM=storage", 
			"G_ACTION=HOST_STATE_CHANGED",
			var,
			NULL
		};

		state->host_connected = (state->source == SOURCE_BUS);
		printk(KERN_INFO "%s: UEVENT host_connected=%d\n",
		       __func__, state->host_connected);
		kobject_uevent_env(&usb_gadget_event_device.dev.kobj,
				   KOBJ_CHANGE, envp);
	}

	if (prev_mA != state->mA) {
		char var_source[32];
		char var_current[32];
		char *envp[] = {
			"G_SUBSYSTEM=power",
			"G_ACTION=POWER_STATE_CHANGED",
			var_source,
			var_current,
			NULL
		};
		sprintf(var_source,
			"G_POWER_SOURCE=%s", source_to_string(state->source));
		sprintf(var_current, "G_CURRENT_MA=%d", state->mA);

		printk(KERN_INFO "%s: UEVENT source=%s mA=%d\n",
		       __func__, source_to_string(state->source), state->mA);
		kobject_uevent_env(&usb_gadget_event_device.dev.kobj,
				   KOBJ_CHANGE, envp);
	}
}

int
usb_gadget_event_vbus_draw(unsigned mA)
{
	int ret = -ENODEV;
	struct usb_gadget_event_state *state = the_gadget_event_state;

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s: mA=%d\n", __func__, mA);
#endif
	if (state) {
		state->new_mA = mA;
		queue_work(state->work_queue, &state->vbus_draw_work);
		ret = 0;
	}

	return (ret);
}
EXPORT_SYMBOL(usb_gadget_event_vbus_draw);

/*********************************************************************
 * usb_gadget_event_driver functions
 */
static int
usb_gadget_event_probe(struct platform_device *pdev)
{
	return (0);
}

static void
usb_gadget_event_suspend_callback(struct work_struct *work)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;
	int prev_mA = state->mA;
	int prev_source = state->source;

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s: prev_mA=%d\n", __func__, prev_mA);
#endif
	if (!state->enabled) {
		printk(KERN_INFO "%s: not enabled. ignored\n", __func__);
		return;
	}

	if (prev_source == SOURCE_BUS) {
		state->mA = 0;
		state->source = SOURCE_NONE;
	}

	if (prev_mA != state->mA) {
		char var_source[32];
		char var_current[32];
		char *envp[] = {
			"G_SUBSYSTEM=power",
			"G_ACTION=POWER_STATE_CHANGED",
			var_source,
			var_current,
			NULL
		};
		sprintf(var_source,
			"G_POWER_SOURCE=%s", source_to_string(state->source));
		sprintf(var_current, "G_CURRENT_MA=%d", state->mA);

		printk(KERN_INFO "%s: UEVENT source=%s mA=%d\n",
		       __func__, source_to_string(state->source), state->mA);
		kobject_uevent_env(&usb_gadget_event_device.dev.kobj,
				   KOBJ_CHANGE, envp);
	}
}

static int
usb_gadget_event_suspend(struct platform_device *pdev, pm_message_t message)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	if ((message.event == PM_EVENT_SUSPEND) && state) {
		// flush the queue to ensure that a notification can happen
		// before the suspend.
		queue_work(state->work_queue, &state->suspend_work);
		flush_workqueue(state->work_queue);
	}

	return 0;
}

static int
usb_gadget_event_resume(struct platform_device *pdev)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;

#ifdef CHARGING_DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	if (state) {
		queue_work(state->work_queue, &state->vbus_presence_work);
	}

	return (0);
}

static struct platform_driver usb_gadget_event_driver = {
	.driver = {
		.name		= USB_GADGET_EVENT_NAME,
		.bus		= &platform_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe		= usb_gadget_event_probe,
	.suspend	= usb_gadget_event_suspend,
	.resume		= usb_gadget_event_resume,
};

/*********************************************************************
 * usb_gadget_event_bind()
 */

void
usb_gadget_event_bind(void *arg)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;
	struct usb_gadget *gadget = (struct usb_gadget *)arg;

	if (state) {
		state->gadget = gadget;
	}
}
EXPORT_SYMBOL(usb_gadget_event_bind);

/*********************************************************************
 * usb_gadget_event_unbind()
 */

void
usb_gadget_event_unbind(void)
{
	struct usb_gadget_event_state *state = the_gadget_event_state;

	if (state) {
		state->gadget = NULL;
	}
}
EXPORT_SYMBOL(usb_gadget_event_unbind);

/*********************************************************************
 * usb_gadget_event_init()
 */

static void __init
usb_gadget_event_state_destroy(struct usb_gadget_event_state *state)
{
	if (state->work_queue)
		destroy_workqueue(state->work_queue);

	kfree(state);
}

static inline struct usb_gadget_event_state *
usb_gadget_event_state_create(void)
{
	struct usb_gadget_event_state *state = NULL;

	state = kcalloc(1, sizeof(struct usb_gadget_event_state), GFP_KERNEL);

	if (!state)
		return (NULL);

	state->work_queue =
		create_singlethread_workqueue("gadget_event");

	if (!state->work_queue)
	{
		usb_gadget_event_state_destroy(state);
		return (NULL);
	}

	INIT_WORK(&state->disable_work, usb_gadget_event_disable_callback);
	INIT_WORK(&state->vbus_presence_work, usb_gadget_event_vbus_presence_callback);
	INIT_WORK(&state->vbus_draw_work, usb_gadget_event_vbus_draw_callback);
	INIT_WORK(&state->suspend_work, usb_gadget_event_suspend_callback);
	INIT_WORK(&state->media_loaded_work, usb_gadget_event_media_loaded_callback);
	INIT_WORK(&state->media_requested_work, usb_gadget_event_media_requested_callback);
	INIT_WORK(&state->reconnect_work, usb_gadget_event_reconnect_callback);

	return (state);
}

static int __init
usb_gadget_event_init(void)
{
	int ret = 0;
	struct usb_gadget_event_state *state;

	// Create the USB gadget_event state.
	if (!(state = usb_gadget_event_state_create()))
		return (-ENOMEM);

	// Register USB gadget_event device.
	if ((ret = platform_device_register(&usb_gadget_event_device)))
		goto platform_device_register_fail;

	// Create sysfs files for USB gadget_event device.
	if ((ret = device_create_file(&usb_gadget_event_device.dev,
				      &dev_attr_current_mA)))
		goto device_create_file_fail1;

	if ((ret = device_create_file(&usb_gadget_event_device.dev,
				      &dev_attr_source)))
		goto device_create_file_fail2;

	if ((ret = device_create_file(&usb_gadget_event_device.dev,
				      &dev_attr_media_loaded)))
		goto device_create_file_fail4;

	if ((ret = device_create_file(&usb_gadget_event_device.dev,
				      &dev_attr_media_requested)))
		goto device_create_file_fail5;

	if ((ret = device_create_file(&usb_gadget_event_device.dev,
				      &dev_attr_host_connected)))
		goto device_create_file_fail6;

	if ((ret = device_create_file(&usb_gadget_event_device.dev,
				      &dev_attr_reconnect)))
		goto device_create_file_fail7;

	// Register the USB gadget_event driver.
	if ((ret = platform_driver_register(&usb_gadget_event_driver)))
		goto driver_register_fail;

	// Set the USB gadget_event state singleton.
	the_gadget_event_state = state;
	smp_mb();

	goto done;

driver_register_fail:
	device_remove_file(&usb_gadget_event_device.dev, &dev_attr_reconnect);
device_create_file_fail7:
	device_remove_file(&usb_gadget_event_device.dev, &dev_attr_host_connected);
device_create_file_fail6:
	device_remove_file(&usb_gadget_event_device.dev, &dev_attr_media_requested);
device_create_file_fail5:
	device_remove_file(&usb_gadget_event_device.dev, &dev_attr_media_loaded);
device_create_file_fail4:
	device_remove_file(&usb_gadget_event_device.dev, &dev_attr_current_mA);
device_create_file_fail2:
	device_remove_file(&usb_gadget_event_device.dev, &dev_attr_source);
device_create_file_fail1:
	platform_device_unregister(&usb_gadget_event_device);
platform_device_register_fail:
	usb_gadget_event_state_destroy(state);
done:

	return (ret);
}

module_init(usb_gadget_event_init);

MODULE_AUTHOR("Palm, Inc.");
MODULE_DESCRIPTION("USB gadget event driver");
MODULE_LICENSE("GPL");
