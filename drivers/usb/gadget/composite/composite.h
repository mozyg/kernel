#ifndef	_COMPOSITE_H_
#define	_COMPOSITE_H_

/* defines shared with gadgets */
#define USB_COMPOSITE_DEVICE
/* big enough to hold our biggest descriptor */
#define COMPOSITE_BUFSIZ 	512
/* For file storage */
#define DELAYED_STATUS		(COMPOSITE_BUFSIZ + 999)

/* configurations */
#define NUM_COMPOSITE_CONFIGS		1 /* 2 */

#define COMPOSITE_500MA_CONFIG_VALUE	1
#define COMPOSITE_100MA_CONFIG_VALUE	2

/**
 * struct usb_function - describes one function of a composite device
 * @name: for diagnostics, identifies the function
 * @strings: tables of strings, keyed by identifiers assigned during bind()
 *	and language IDs provided in control requests
 * @descriptors: table of low/full speed descriptors, using interface and
 *	string identifiers assigned during bind()
 * @hs_descriptors: table of high speed descriptors, using interface and
 *	string identifiers assigned during bind(); or null
 * @function: each function is on the usb_composite.function list when the
 *	composite gadget is initialized, and then places itself in the right
 *	position(s) in usb_composite.functions[] during bind()
 * @bind: before the gadget can register, all of its functions bind() to the
 *	available resources including identifiers for strings, interfaces,
 *	and endpoints
 * @unbind: before the gadget can unregister, all of its functions unbind()
 *	the resources they have allocated.
 * @setup: used for SET_INTERFACE, SET_CONFIGURATION, and interface-specific
 *	control requests
 * @disconnect: used to notify functions when the host has disconnected
 * @suspend: notifies functions when the host stops sending USB traffic
 * @resume: notifies functions when the host reactivates the USB link
 *
 * A single USB function uses one or more interfaces, and supports dual speed
 * operation on appropriate hardware.
 */
struct usb_function {
	const char				*name;
	struct usb_gadget_strings		*strings;
	const struct usb_descriptor_header	**descriptors;
	const struct usb_descriptor_header	**hs_descriptors;
	int					first_interface;
	int					num_interfaces;

	struct list_head			function;

	/* data private to the driver */
	void			*driver_data;

	int			(*init)(void);
	void			(*exit)(void);
	int			(*bind)(struct usb_gadget *);
	void			(*unbind)(struct usb_gadget *);
	int			(*set_descriptors)(int config, int is_otg);
	int			(*setup)(struct usb_gadget *,
					const struct usb_ctrlrequest *);
	void			(*disconnect)(struct usb_gadget *);
	void			(*suspend)(struct usb_gadget *);
	void			(*resume)(struct usb_gadget *);
};

/**
 * struct usb_composite_driver -- groups functions into one gadget driver
 * @name: for diagnostics, identifies the gadget driver
 * @dev: descriptor for the device
 * @strings: tables of strings, keyed by identifiers assigned during bind()
 *	and language IDs provided in control requests
 * @functions: each function on this list when usb_composite_init() is called
 *	will be bound and initialized
 */
struct usb_composite_driver {
	const char				*name;
	const struct usb_device_descriptor	*dev;
	struct usb_gadget_strings		*strings;

	unsigned int vendor_id;
	unsigned int product_id;

	/* REVISIT want a general "add more descriptors for config N"
	 * hook; OTG would fall out naturally
	 */

	struct list_head	functions;
};

extern int usb_composite_ep_reset (struct usb_ep *);

#define	MAX_COMPOSITE_INTERFACES		16	/* max 16 */

/**
 * struct usb_composite_device - represents one composite usb gadget
 * @gadget: read-only, abstracts the gadget's usb peripheral controller
 * @req: used for control responses; buffer is pre-allocated
 * @dev: device descriptor
 * @config: configuration descriptor; if the device is not configured,
 *	its bConfigurationValue is zero and other fields are ignored
 */
struct usb_composite_dev {
	spinlock_t			lock;
	struct usb_gadget		*gadget;
	struct usb_request		*req;
	struct usb_device_descriptor	dev;
	struct usb_config_descriptor	config;

	/* REVISIT need per-function state hook ... maybe a
	 * (void*) returned from bind() and passed to callbacks?
	 * arguably, interface descriptors are part of that...
	 */

	/* INTERNALS -- not for function drivers */
	struct usb_function	*current_func;
	struct usb_function	*interface[MAX_COMPOSITE_INTERFACES];

	struct usb_composite_driver	*driver;
	struct usb_gadget_driver	*gadget_driver;// REVISIT
	struct usb_qualifier_descriptor	qual;
};

static inline void set_composite_data (struct usb_composite_dev *cdev, void *data)
	{ cdev->current_func->driver_data = data; }
static inline void *get_composite_data (struct usb_composite_dev *cdev)
	{ return cdev->current_func->driver_data; }

#endif	/* _COMPOSITE_H_ */
