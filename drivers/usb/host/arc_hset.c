/*
 * HS USB Host-mode HSET driver for ARC USB controller
 * Copyright 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Zhang Yan <jasper.zhang@freescale.com>
 * Peter Chen <peter.chen@freescale.com>  */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define EHCI_IAA_MSECS		10		/* arbitrary */
#include <linux/usb/hcd.h>
#include "../core/usb.h"
#include "ehci-fsl.h"
#include <linux/uaccess.h>
#include <linux/io.h>
#include <mach/arc_otg.h>
#include "ehci.h"

#define rh_level 1
#define SINGLE_STEP_SLEEP_COUNT  15000
#define USB_HIGH_SPEED		0x01
#define USB_FULL_SPEED		0x02
#define USB_LOW_SPEED		0x03

/* ARC port test mode */
#define PORTSC_PTC_TEST_MODE_DISABLE	0x00
#define PORTSC_PTC_TEST_J		0x01
#define PORTSC_PTC_TEST_K		0x02
#define PORTSC_PTC_SE0_NAK		0x03
#define PORTSC_PTC_TEST_PACKET		0x04
#define PORTSC_PTC_FORCE_ENABLE_HS	0x05
#define PORTSC_PTC_FORCE_ENABLE_FS	0x06
#define PORTSC_PTC_FORCE_ENABLE_LS	0x07

/* Other test */
#define HSET_TEST_SUSPEND_RESUME	0x08
#define HSET_TEST_GET_DEV_DESC		0x09
#define HSET_TEST_GET_DEV_DESC_DATA	0x0A


#define TEST_DEVICE_VID	 0x1A0A
static struct usb_device_id archset_table[] = {
	{ USB_DEVICE(TEST_DEVICE_VID, 0x0101) },	/* Test_SE0_NAK */
	{ USB_DEVICE(TEST_DEVICE_VID, 0x0102) },	/* Test_J */
	{ USB_DEVICE(TEST_DEVICE_VID, 0x0103) },	/* Test_K */
	{ USB_DEVICE(TEST_DEVICE_VID, 0x0104) },	/* Test_Packet */
	{ USB_DEVICE(TEST_DEVICE_VID, 0x0105) },	/* Force enable */
	{ USB_DEVICE(TEST_DEVICE_VID, 0x0106) },	/* HS_HOST_PORT_SUSPEND_RESUME */
	{ USB_DEVICE(TEST_DEVICE_VID, 0x0107) },	/* SINGLE_STEP_GET_DEV_DESC */
	{ USB_DEVICE(TEST_DEVICE_VID, 0x0108) },	/* SINGLE_STEP_GET_DEV_DESC_DATA */
	{ }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, archset_table);

/* Private data */
struct usb_archset {
	struct usb_device    *udev;	/* the usb device for this device */
	struct usb_interface *interface;	/* the interface for this device */
	struct kref           kref;
	struct ehci_hcd       *ehci;	/* the usb controller */
	char                  ptcname[24];
};

 void set_single_step_desc_data_on(void);
 void clear_single_step_desc_data_on(void);

static void arc_kill_per_sched(struct ehci_hcd *ehci)
{
	u32 command = 0;
	command = ehci_readl(ehci, &ehci->regs->command);
	command &= ~(CMD_PSE);
	ehci_writel(ehci, command, &ehci->regs->command);
	printk(KERN_INFO "+++ %s: Periodic Schedule Stopped", __func__);
}

static void arc_hset_test(struct ehci_hcd *ehci, int mode)
{
	u32 portsc = 0;

	portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);
	portsc &= ~(0xf << 16);
	portsc |= (mode << 16);
	ehci_writel(ehci, portsc, &ehci->regs->port_status[0]);
}

static inline void hub_set_testmode(struct usb_device *udev, u8
			test_mode)
{
	struct usb_device *hub_dev;
	int i;
	int max_port;
	int status;
	u8 level = udev->level;

	if (level == 0)
		return;
	else {
		printk(KERN_INFO "run %s at tie %d hub\n", __func__, level);
		/* get the port number of parent */
		if (udev->parent != NULL) {
			hub_dev = udev->parent;
			max_port = hub_dev->maxchild;
		} else {
			printk(KERN_INFO "device don't have parent hub\n");
			return;
		}

		for (i = 0; i < max_port; i++) {
			status = usb_control_msg(hub_dev, usb_sndctrlpipe(udev, 0),
				USB_REQ_SET_FEATURE, USB_RT_PORT,
				USB_PORT_FEAT_SUSPEND, i+1,
				NULL, 0, 1000);
			if (status >= 0)
				printk(KERN_INFO "send port_suspend to port %d\n", i+1);
			else {
				printk(KERN_INFO "send port_suspend error %d to port %d\n",
					status, i+1);
				continue;
			}
		}
		msleep(1000);
		status = usb_control_msg(hub_dev, usb_sndctrlpipe(udev, 0),
				USB_REQ_SET_FEATURE/*USB_REQ_CLEAR_FEATURE*/, USB_RT_PORT,
				USB_PORT_FEAT_TEST, (__u16) ((test_mode << 8) | udev->portnum),
				NULL, 0, 1000);
		if (status >= 0)
			printk(KERN_INFO "send cmd %d to port %d\n", test_mode, udev->portnum);
		else
			printk(KERN_INFO "send cmd %d error %d to port %d\n", test_mode, status, udev->portnum);
	}
}

static inline void test_j(struct usb_archset *hset)
{
	struct usb_device *udev = hset->udev;
	u8 level = udev->level;
	if (level == rh_level) {
		printk(KERN_INFO "%s at tie %d hub\n", __func__, level);
		arc_hset_test(hset->ehci, PORTSC_PTC_TEST_J);
	} else
		hub_set_testmode(udev, PORTSC_PTC_TEST_J);
}

static inline void test_k(struct usb_archset *hset)
{
	struct usb_device *udev = hset->udev;
	u8 level = udev->level;
	printk(KERN_INFO "%s at tie %d hub\n", __func__, level);
	if (level == rh_level)
		arc_hset_test(hset->ehci, PORTSC_PTC_TEST_K);
	else
		hub_set_testmode(udev, PORTSC_PTC_TEST_K);
}

static inline void test_se0_nak(struct usb_archset *hset)
{
	struct usb_device *udev = hset->udev;
	u8 level = udev->level;
	printk(KERN_INFO "%s at tie %d hub\n", __func__, level);
	if (level == rh_level)
		arc_hset_test(hset->ehci, PORTSC_PTC_SE0_NAK);
	else
		hub_set_testmode(udev, PORTSC_PTC_SE0_NAK);
}

static inline void test_packet(struct usb_archset *hset)
{
	struct usb_device *udev = hset->udev;
	u8 level = udev->level;
	printk(KERN_INFO "%s at tie %d hub\n", __func__, level);
	if (level == rh_level)
		arc_hset_test(hset->ehci, PORTSC_PTC_TEST_PACKET);
	else
		hub_set_testmode(udev, PORTSC_PTC_TEST_PACKET);
}

static inline void test_force_enable(struct usb_archset *hset, int
			forcemode)
{
	struct usb_device *udev = hset->udev;
	u8 level = udev->level;
	int ptc_fmode = 0;
	printk(KERN_INFO "%s at tie %d hub\n", __func__, level);
	if (level == 0) {
		switch (forcemode) {
		case USB_HIGH_SPEED:
			ptc_fmode = PORTSC_PTC_FORCE_ENABLE_HS;
			break;
		case USB_FULL_SPEED:
			ptc_fmode = PORTSC_PTC_FORCE_ENABLE_FS;
			break;
		case USB_LOW_SPEED:
			ptc_fmode = PORTSC_PTC_FORCE_ENABLE_LS;
			break;
		default:
			printk(KERN_ERR "unknown speed mode %d\n", forcemode);
			return;
		}
		arc_hset_test(hset->ehci, ptc_fmode);
	} else
		hub_set_testmode(udev, PORTSC_PTC_FORCE_ENABLE_HS);
}

static void suspend(struct usb_archset *hset)
{
	struct ehci_hcd *ehci = hset->ehci;
	u32 portsc = 0;

	portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);
	portsc |= PORT_SUSPEND;	/* Set suspend */
	ehci_writel(ehci, portsc, &ehci->regs->port_status[0]);
}


static void resume(struct usb_archset *hset)
{
	struct ehci_hcd *ehci = hset->ehci;
	u32 portsc = 0;

	portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);
	portsc |= PORT_RESUME;
	ehci_writel(ehci, portsc, &ehci->regs->port_status[0]);
}

static void test_suspend_resume(struct usb_archset *hset)
{
	printk(KERN_INFO "%s\n", __func__);

	suspend(hset);
	msleep(15000);	/* Wait for 15s */
	resume(hset);
}

static void test_single_step_get_dev_desc(struct usb_archset *hset)
{
	struct ehci_hcd *ehci = hset->ehci;
	struct usb_bus *bus = hcd_to_bus(ehci_to_hcd(ehci));
	struct usb_device *udev = hset->udev;
	int    result;

	if (!ehci || !bus || !bus->root_hub) {
		printk(KERN_ERR "Host controller not ready!\n");
		return;
	}

	if (!udev) {
		printk(KERN_ERR "No device connected.\n");
		return;
	}

	/* Stop Periodic schedule to prevent polling of hubs */
	arc_kill_per_sched(hset->ehci);

	msleep(SINGLE_STEP_SLEEP_COUNT);	/* SOF for 15s */

	result = usb_get_device_descriptor(udev, sizeof(struct usb_device_descriptor));
	if (result < 0 && result != -ETIMEDOUT)
		printk(KERN_ERR "usb_get_device_descriptor failed %d\n", result);
	else
		printk(KERN_INFO "usb_get_device_descriptor passed\n");
}
static void test_single_step_get_dev_desc_data(struct usb_archset *hset)
{
	struct usb_device *udev = hset->udev;
	struct usb_device_descriptor *desc;
	int    result;
	unsigned int size = sizeof(struct usb_device_descriptor);
	printk(KERN_INFO "%s size = %d\n", __func__, size);
	desc = kmalloc(sizeof(*desc), GFP_NOIO);
	if (!desc) {
		printk(KERN_ERR "alloc desc failed\n");
		return ;
	}

	set_single_step_desc_data_on();

	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
					USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
					(0x01 << 8) + 0, 0, desc, size, USB_CTRL_GET_TIMEOUT);
	if (result < 0 && result != -ETIMEDOUT)
		printk(KERN_ERR "the setup transaction failed %d\n", result);
	else
		printk(KERN_INFO "the setup transaction passed\n");


	/* Stop Periodic schedule to prevent polling of hubs */
	arc_kill_per_sched(hset->ehci);

	msleep(SINGLE_STEP_SLEEP_COUNT);

	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
					USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
					(0x01 << 8) + 0, 0, desc, size,	USB_CTRL_GET_TIMEOUT);
	if (result <= 0 && result != -ETIMEDOUT)
		printk(KERN_ERR "the data transaction failed %d\n", result);
	else
		printk(KERN_INFO "the data transaction passed\n");

	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
					USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
					(0x01 << 8) + 0, 0, desc, size, USB_CTRL_GET_TIMEOUT);
	if (result < 0 && result != -ETIMEDOUT)
		printk(KERN_ERR "the status transaction failed %d\n", result);
	else
		printk(KERN_INFO "the status transaction passed\n");

	clear_single_step_desc_data_on();

	kfree(desc);

	msleep(SINGLE_STEP_SLEEP_COUNT);	/* SOF for 15s */
	printk(KERN_INFO "test_single_step_get_dev_desc_data finished\n");
}


void test_single_step_set_feature(struct usb_archset *hset)
{
	struct ehci_hcd *ehci = hset->ehci;
	struct usb_bus *bus = hcd_to_bus(ehci_to_hcd(ehci));
	struct usb_device *udev;

	if (!ehci || !bus || !bus->root_hub) {
		printk(KERN_ERR "Host controller not ready!\n");
		return;
	}
	udev = bus->root_hub->children[0];
	if (!udev) {
		printk(KERN_ERR "No device connected.\n");
		return;
	}
	usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
				USB_REQ_SET_FEATURE, 0, 0, 0,
				NULL, 0, USB_CTRL_SET_TIMEOUT);
}

static struct usb_archset *the_hset[4];
static struct usb_archset *init_hset_dev(void *controller)
{
	struct ehci_hcd *ehci = (struct ehci_hcd *)controller;
	struct usb_archset *hset = NULL;
	int ctrid = 0;

	ctrid = ehci_to_hcd(ehci)->self.busnum-1;
	if (the_hset[ctrid]) {
		kref_get(&the_hset[ctrid]->kref);
		return the_hset[ctrid];
	}

	hset = kzalloc(sizeof(struct usb_archset), GFP_KERNEL);
	if (hset == NULL) {
		printk(KERN_ERR "Out of memory!\n");
		return NULL;
	}
	hset->ehci = (struct ehci_hcd *)controller;
	kref_init(&hset->kref);
	the_hset[ctrid] = hset;

	return hset;
}


static void hset_delete(struct kref *kref)
{
	struct usb_archset *hset = container_of(kref, struct usb_archset,
								kref);

	kfree(hset);
}

static int archset_probe(struct usb_interface *iface,
		const struct usb_device_id *id)
{
	struct usb_archset *hset;
	struct usb_device *udev;
	struct usb_hcd *hcd;

	udev = usb_get_dev(interface_to_usbdev(iface));
	hcd = bus_to_hcd(udev->bus);
	hset = init_hset_dev((void *)hcd_to_ehci(hcd));
	if (hset == NULL)
		return -ENOMEM;

	hset->udev = udev;
	usb_set_intfdata(iface, hset);
	switch (id->idProduct) {
	case 0x0101:
		test_se0_nak(hset);
		break;
	case 0x0102:
		test_j(hset);
		break;
	case 0x0103:
		test_k(hset);
		break;
	case 0x0104:
		test_packet(hset);
		break;
	case 0x0105:
		printk(KERN_INFO "Force FS/FS/LS ?\n");
		test_force_enable(hset, USB_HIGH_SPEED);
		break;
	case 0x0106:
		test_suspend_resume(hset);
		break;
	case 0x0107:
		printk(KERN_INFO "Begin SINGLE_STEP_GET_DEVICE_DESCRIPTOR\n");
		test_single_step_get_dev_desc(hset);
		break;
	case 0x0108:
		printk(KERN_INFO "Begin SINGLE_STEP_GET_DEVICE_DESCRIPTOR_DATA\n");
		test_single_step_get_dev_desc_data(hset);
		break;
	}

	return 0;
}

static void archset_disconnect(struct usb_interface *iface)
{
	struct usb_archset *hset;

	hset = usb_get_intfdata(iface);
	usb_set_intfdata(iface, NULL);

	usb_put_dev(hset->udev);
	kref_put(&hset->kref, hset_delete);
}


static struct usb_driver archset_driver = {
	.name = "arc hset",
	.probe = archset_probe,
	.disconnect = archset_disconnect,
	.id_table = archset_table,
};

static int __init usb_hset_init(void)
{
	int ret = 0;
	printk(KERN_ERR "usb register error with %d\n", ret);
	/* register driver to usb subsystem */
	ret = usb_register(&archset_driver);
    if (ret)
		printk(KERN_ERR "usb register error with %d\n", ret);
	else
		printk(KERN_INFO "usb hset init succeed\n");

	return ret;
}

static void __exit usb_hset_exit(void)
{
	usb_deregister(&archset_driver);
}

module_init(usb_hset_init);
module_exit(usb_hset_exit);
MODULE_DESCRIPTION("Freescale USB Host Test Mode Driver");
MODULE_AUTHOR("Jasper Zhang");
MODULE_LICENSE("GPL");

