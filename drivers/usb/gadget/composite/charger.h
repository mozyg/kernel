#ifndef	_CHARGER_H_
#define	_CHARGER_H_

/*
 * Charger
 */
#define USB_CHARGE_CURRENT_100MA	50
#define USB_CHARGE_CURRENT_500MA	250
#define USB_CHARGE_CURRENT_1A		500

extern void charger_detection_start(struct usb_composite_dev *cdev);
extern void charger_detection_stop(struct usb_composite_dev *cdev);

extern void charger_detection_enable(struct usb_composite_dev *cdev);
extern int charger_detection_try_maxpower(void);

extern int charger_detection_bind(struct usb_composite_dev *cdev);
extern int charger_detection_unbind(struct usb_composite_dev *cdev);

#endif /* _CHARGER_H_ */
