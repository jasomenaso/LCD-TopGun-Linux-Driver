/*
 *  EMS LCD TopGun support
 *
 *  (c) 2006 Christophe Thibault <chris@aegis-corp.org>
 *      2008 Adolfo R. Brandes <arbrandes@gmail.com>
 *
 *  Based on GunCon2 linux driver by Brian Goines
 *  and the Xpad linux driver by Marko Friedemann
 *
 *  History:
 *
 *  2008-04-14 - 0.2: (Adolfo R. Brandes) General update
 *   - Compiles and runs on newer kernels (tested up to 2.6.24).
 *   - Rewrote the setting of bits, based on xpad360.
 *   - Added module option debug=1
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/config.h>
#endif
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17)
#include <linux/usb_input.h>
#else
#include <linux/usb/input.h>
#endif

static unsigned long debug = 0;
module_param(debug, ulong, 0444);
MODULE_PARM_DESC(debug, "Debugging");

/*
 * Version Information
 */
#define DRIVER_VERSION "v0.2"
#define DRIVER_AUTHOR "Christophe Thibault <chris@aegis-corp.org>"
#define DRIVER_DESC "USB EMS LCD TopGun driver"
#define DRIVER_LICENSE "GPL"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

struct usb_topgun {
	char name[128];
	char phys[65];
	struct usb_device *usbdev;
	struct input_dev *dev;
	struct urb *irq;
	int open;

	unsigned char *data;
	dma_addr_t data_dma;
};

static const signed short topgun_btn[] = {
	BTN_TRIGGER, BTN_A, BTN_B, BTN_C,	/* Main buttons */
	BTN_START, BTN_SELECT,			/* Start, Select */
	-1					/* Terminating entry */
};

static const signed short topgun_abs[] = {
	ABS_X, ABS_Y,				/* Aiming */
	ABS_HAT0X, ABS_HAT0Y,			/* POV Hat */
	-1					/* Terminating entry */
};

static struct usb_device_id usb_topgun_id_table [] = {
	{match_flags: USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT,	idVendor: 0xb9a, idProduct: 0x16a},
	{ } /* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, usb_topgun_id_table);

static void usb_topgun_irq(struct urb *urb
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	, struct pt_regs *regs
#endif
)
{
	struct usb_topgun *topgun = urb->context;
	unsigned char *data = topgun->data;
	struct input_dev *dev = topgun->dev;
	int status;
	int i, x = 0, y = 0;
	unsigned int gun;

	switch (urb->status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	/* -EPIPE:  should clear the halt */
	default:		/* error */
		goto resubmit;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	input_regs(dev, regs);
#endif

	if (debug) {
		printk(KERN_INFO "topgun_debug: data :");
		for(i = 0; i < 20; i++) {
			printk("0x%02x ", data[i]);
		}
		printk("\n");
	}

	input_report_key(dev, BTN_TRIGGER, !(data[1] & 0x20));
	input_report_key(dev, BTN_A,       !(data[0] & 0x04));
	input_report_key(dev, BTN_B,       !(data[0] & 0x08));
	input_report_key(dev, BTN_C,       !(data[0] & 0x02));
	input_report_key(dev, BTN_START,   !(data[1] & 0x80));
	input_report_key(dev, BTN_SELECT,  !(data[1] & 0x40));

	if(!(data[0] & 0x10)) x += -1;
	if(!(data[0] & 0x40)) x	+= 1;
	if(!(data[0] & 0x80)) y	+= -1;
	if(!(data[0] & 0x20)) y	+= 1;

	gun = data[3];
	gun <<= 8;
	gun |= data[2];

	/* stick */
	input_report_abs(dev, ABS_X, gun);
	input_report_abs(dev, ABS_Y, data[4]);

	/* digital pad */
	input_report_abs(dev, ABS_HAT0X, x);
	input_report_abs(dev, ABS_HAT0Y, y);

	input_sync(dev);

resubmit:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status)
		err("can't resubmit intr, %s-%s/input0, status %d",
				topgun->usbdev->bus->bus_name,
				topgun->usbdev->devpath, status);
}

static int usb_topgun_open(struct input_dev *dev)
{
	struct usb_topgun *topgun = dev->private;
	int status;

	if (topgun->open++)
		return 0;

	topgun->irq->dev = topgun->usbdev;
	if ((status = usb_submit_urb(topgun->irq, GFP_KERNEL))) {
		err("open input urb failed: %d", status);
		topgun->open--;
		return -EIO;
	}

	return 0;
}

static void usb_topgun_close(struct input_dev *dev)
{
	struct usb_topgun *topgun = dev->private;

	if (!--topgun->open)
		usb_unlink_urb(topgun->irq);
}

static int usb_topgun_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct usb_topgun *topgun;
	struct input_dev *input_dev;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_host_interface *interface;
	int pipe, maxp, i;
	char path[64];
	char *buf;

	interface = intf->cur_altsetting;

	if (interface->desc.bNumEndpoints != 1) 
		return -ENODEV;

	endpoint = &interface->endpoint[0].desc;
	if (!(endpoint->bEndpointAddress & 0x80)) 
		return -ENODEV;
	if ((endpoint->bmAttributes & 3) != 3) 
		return -ENODEV;

	pipe = usb_rcvintpipe(usbdev, endpoint->bEndpointAddress);
	maxp = usb_maxpacket(usbdev, pipe, usb_pipeout(pipe));

	topgun = kzalloc(sizeof(struct usb_topgun), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!topgun || !input_dev) {
		input_free_device(input_dev);
		kfree(topgun);
		return -ENOMEM;
	}

	topgun->data = usb_buffer_alloc(usbdev, 8, GFP_ATOMIC, &topgun->data_dma);
	if (!topgun->data) {
		input_free_device(input_dev);
		kfree(topgun);
		return -ENOMEM;
	}

	topgun->irq = usb_alloc_urb(0, GFP_KERNEL);
	if (!topgun->irq) {
		usb_buffer_free(usbdev, 8, topgun->data, topgun->data_dma);
		input_free_device(input_dev);
		kfree(topgun);
		return -ENODEV;
	}

	topgun->usbdev = usbdev;
	topgun->dev = input_dev;

	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);

	for (i = 0; topgun_btn[i] >= 0; ++i)
		set_bit(topgun_btn[i], input_dev->keybit);

	for (i = 0; topgun_abs[i] >= 0; ++i) {
		signed short t = topgun_abs[i];
		set_bit(t, input_dev->absbit);
		switch (t) {
			case ABS_X: input_set_abs_params(input_dev, t, 160, 672, 0, 0); break;
			case ABS_Y: input_set_abs_params(input_dev, t, 32, 224, 0, 0); break;
			case ABS_HAT0X:
			case ABS_HAT0Y: input_set_abs_params(input_dev, t, -1, 1, 0, 0); break;
		}
	}

	usb_make_path(usbdev, path, 64);
	sprintf(topgun->phys, "%s/input0", path);

	input_dev->name = topgun->name;
	input_dev->phys = topgun->phys;
	usb_to_input_id(usbdev, &input_dev->id);
	input_dev->cdev.dev = &intf->dev;
	input_dev->private = topgun;
	input_dev->open = usb_topgun_open;
	input_dev->close = usb_topgun_close;

	/* Start name manipulation. */
	if (!(buf = kmalloc(63, GFP_KERNEL))) {
		usb_buffer_free(usbdev, 8, topgun->data, topgun->data_dma);
		kfree(topgun);
		return -ENOMEM;
	}

	if (usbdev->descriptor.iManufacturer &&
		usb_string(usbdev, usbdev->descriptor.iManufacturer, buf, 63) > 0)
			strcat(topgun->name, buf);

	if (usbdev->descriptor.iProduct &&
		usb_string(usbdev, usbdev->descriptor.iProduct, buf, 63) > 0)
			sprintf(topgun->name, "%s %s", topgun->name, buf);

	if (!strlen(topgun->name))
		sprintf(topgun->name, "EMS LCD TopGun %04x:%04x",
			input_dev->id.vendor, input_dev->id.product);

	kfree(buf);
	/* End name manipulation. */

	usb_fill_int_urb(topgun->irq, usbdev, pipe, topgun->data,
			 (maxp > 8 ? 8 : maxp),
			 usb_topgun_irq, topgun, endpoint->bInterval);
	topgun->irq->transfer_dma = topgun->data_dma;
	topgun->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	input_register_device(topgun->dev);

	if (debug)
		printk(KERN_INFO "input: %s on %s\n", topgun->name, path);

	usb_set_intfdata(intf, topgun);

	return 0;
}

static void usb_topgun_disconnect(struct usb_interface *intf)
{
	struct usb_topgun *topgun = usb_get_intfdata (intf);
	
	usb_set_intfdata(intf, NULL);
	if (topgun) {
		usb_unlink_urb(topgun->irq);
		input_unregister_device(topgun->dev);
		usb_free_urb(topgun->irq);
		usb_buffer_free(interface_to_usbdev(intf), 8, topgun->data, topgun->data_dma);
		kfree(topgun);
	}
}

static struct usb_driver usb_topgun_driver = {
	.name		= "lcdtopgun",
	.probe		= usb_topgun_probe,
	.disconnect	= usb_topgun_disconnect,
	.id_table	= usb_topgun_id_table,
};

static int __init usb_topgun_init(void)
{
	int retval = usb_register(&usb_topgun_driver);
	if (retval == 0) 
		info(DRIVER_DESC " " DRIVER_VERSION " initialized" );
	return retval;
}

static void __exit usb_topgun_exit(void)
{
	usb_deregister(&usb_topgun_driver);
}

module_init(usb_topgun_init);
module_exit(usb_topgun_exit);
