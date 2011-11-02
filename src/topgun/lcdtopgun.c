/*
 *  EMS LCD TopGun support
 *
 *  (c) 2006 Christophe Thibault <chris@aegis-corp.org>
 *
 *  Based on GunCon2 linux driver by Brian Goines
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>

/*
 * Version Information
 */
#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Christophe Thibault <chris@aegis-corp.org>"
#define DRIVER_DESC "USB EMS LCD TopGun driver"
#define DRIVER_LICENSE "GPL"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

struct usb_topgun {
	char name[128];
	char phys[64];
	struct usb_device *usbdev;
	struct input_dev dev;
	struct urb *irq;
	int open;

	signed char *data;
	dma_addr_t data_dma;
};

static void usb_topgun_irq(struct urb *urb, struct pt_regs *regs)
{
	struct usb_topgun *topgun = urb->context;
	unsigned char *data = topgun->data;
	struct input_dev *dev = &topgun->dev;
	int status;
	int x = 0,y = 0;
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

	input_regs(dev, regs);

	input_report_key(dev, BTN_TRIGGER, !(data[1] & 0x20));
	input_report_key(dev, BTN_A,       !(data[0] & 0x04));
	input_report_key(dev, BTN_B,       !(data[0] & 0x08));
	input_report_key(dev, BTN_C,       !(data[0] & 0x02));
	input_report_key(dev, BTN_START,   !(data[1] & 0x80));
	input_report_key(dev, BTN_SELECT,  !(data[1] & 0x40));

	if(!(data[0] & 0x10)) x+=-1;
	if(!(data[0] & 0x40)) x+=1;
	if(!(data[0] & 0x80)) y+=-1;
	if(!(data[0] & 0x20)) y+=1;

	gun=data[3]; gun <<= 8; gun |=data[2];

	/* stick */
	input_report_abs(dev, ABS_X,     gun);
	input_report_abs(dev, ABS_Y,     data[4]);

	/* digital pad */
	input_report_abs(dev, ABS_HAT0X, x);
	input_report_abs(dev, ABS_HAT0Y, y);

	input_sync(dev);

resubmit:
	status = usb_submit_urb (urb, SLAB_ATOMIC);
	if (status)
		err ("can't resubmit intr, %s-%s/input0, status %d",
				topgun->usbdev->bus->bus_name,
				topgun->usbdev->devpath, status);
}

static int usb_topgun_open(struct input_dev *dev)
{
	struct usb_topgun *topgun = dev->private;

	if (topgun->open++)
		return 0;

	topgun->irq->dev = topgun->usbdev;
	if (usb_submit_urb(topgun->irq, GFP_KERNEL)) {
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

static int usb_topgun_probe(struct usb_interface * intf, const struct usb_device_id * id)
{
	struct usb_device * dev = interface_to_usbdev(intf);
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_topgun *topgun;
	int pipe, maxp;
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

	pipe = usb_rcvintpipe(dev, endpoint->bEndpointAddress);
	maxp = usb_maxpacket(dev, pipe, usb_pipeout(pipe));

	if (!(topgun = kmalloc(sizeof(struct usb_topgun), GFP_KERNEL))) 
		return -ENOMEM;
	memset(topgun, 0, sizeof(struct usb_topgun));

	topgun->data = usb_buffer_alloc(dev, 8, SLAB_ATOMIC, &topgun->data_dma);
	if (!topgun->data) {
		kfree(topgun);
		return -ENOMEM;
	}

	topgun->irq = usb_alloc_urb(0, GFP_KERNEL);
	if (!topgun->irq) {
		usb_buffer_free(dev, 8, topgun->data, topgun->data_dma);
		kfree(topgun);
		return -ENODEV;
	}

	topgun->usbdev = dev;

	topgun->dev.evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	topgun->dev.absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_HAT0X) | BIT(ABS_HAT0Y);
	topgun->dev.keybit[LONG(BTN_GAMEPAD)]=BIT(BTN_A) | BIT(BTN_B) | BIT(BTN_C);
	topgun->dev.keybit[LONG(BTN_GAMEPAD)]|=BIT(BTN_START) | BIT(BTN_SELECT);
	topgun->dev.keybit[LONG(BTN_JOYSTICK)]|=BIT(BTN_TRIGGER);

	topgun->dev.absmin[ABS_X] =  160;
	topgun->dev.absmax[ABS_X] =  672;
	topgun->dev.absflat[ABS_X] = 0;
	topgun->dev.absfuzz[ABS_X] = 0;
	topgun->dev.absmin[ABS_Y] = 32;
	topgun->dev.absmax[ABS_Y] = 224;
	topgun->dev.absflat[ABS_Y] = 0;
	topgun->dev.absfuzz[ABS_Y] = 0;

	topgun->dev.absmax[ABS_HAT0X] =  1;
	topgun->dev.absmin[ABS_HAT0X] = -1;
	topgun->dev.absmax[ABS_HAT0Y] =  1;
	topgun->dev.absmin[ABS_HAT0Y] = -1;

	topgun->dev.private = topgun;
	topgun->dev.open = usb_topgun_open;
	topgun->dev.close = usb_topgun_close;

	usb_make_path(dev, path, 64);
	sprintf(topgun->phys, "%s/input0", path);

	topgun->dev.name = topgun->name;
	topgun->dev.phys = topgun->phys;
	topgun->dev.id.bustype = BUS_USB;
	topgun->dev.id.vendor = dev->descriptor.idVendor;
	topgun->dev.id.product = dev->descriptor.idProduct;
	topgun->dev.id.version = dev->descriptor.bcdDevice;
	topgun->dev.dev = &intf->dev;

	if (!(buf = kmalloc(63, GFP_KERNEL))) {
		usb_buffer_free(dev, 8, topgun->data, topgun->data_dma);
		kfree(topgun);
		return -ENOMEM;
	}

	if (dev->descriptor.iManufacturer &&
		usb_string(dev, dev->descriptor.iManufacturer, buf, 63) > 0)
			strcat(topgun->name, buf);
	if (dev->descriptor.iProduct &&
		usb_string(dev, dev->descriptor.iProduct, buf, 63) > 0)
			sprintf(topgun->name, "%s %s", topgun->name, buf);

	if (!strlen(topgun->name))
		sprintf(topgun->name, "EMS LCD TopGun %04x:%04x",
			topgun->dev.id.vendor, topgun->dev.id.product);

	kfree(buf);

	usb_fill_int_urb(topgun->irq, dev, pipe, topgun->data,
			 (maxp > 8 ? 8 : maxp),
			 usb_topgun_irq, topgun, endpoint->bInterval);
	topgun->irq->transfer_dma = topgun->data_dma;
	topgun->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	input_register_device(&topgun->dev);
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
		input_unregister_device(&topgun->dev);
		usb_free_urb(topgun->irq);
		usb_buffer_free(interface_to_usbdev(intf), 8, topgun->data, topgun->data_dma);
		kfree(topgun);
	}
}

static struct usb_device_id usb_topgun_id_table [] = {
	{match_flags: USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT,	idVendor: 0xb9a, idProduct: 0x16a},
    { }						/* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, usb_topgun_id_table);

static struct usb_driver usb_topgun_driver = {
	.owner		= THIS_MODULE,
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
