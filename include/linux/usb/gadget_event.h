/*
 *  gadget_event.h
 *
 *  Copyright (C) 2008-2009  Palm, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _LINUX_USB_GADGET_EVENT_H
#define _LINUX_USB_GADGET_EVENT_H

#define USB_GADGET_EVENT_NAME	"usb_gadget"

extern int usb_gadget_event_enable(int enable);
extern int usb_gadget_event_vbus_presence(void);
extern int usb_gadget_event_vbus_draw(unsigned mA);
extern void usb_gadget_event_bind(void *arg);
extern void usb_gadget_event_unbind(void);

extern int usb_gadget_event_media_loaded(int loaded);
extern int usb_gadget_event_media_requested(int requested);
extern int usb_gadget_event_reconnect(void);

extern int transceiver_vbus_presence(int *presence);
extern int transceiver_is_pullup_attached(int *attached);
extern int transceiver_single_ended_state(int *dplus, int *dminus);
#if defined(CONFIG_TWL4030_USB_FS_3_PIN) && defined(CONFIG_ARCH_OMAP24XX)
extern void transceiver_reconnect(void);
#endif

#endif // _LINUX_USB_GADGET_EVENT_H
