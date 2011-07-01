/*
 *  acpi_drivers.h  ($Revision: 31 $)
 *
 *  Copyright (C) 2001, 2002 Andy Grover <andrew.grover@intel.com>
 *  Copyright (C) 2001, 2002 Paul Diefenbaugh <paul.s.diefenbaugh@intel.com>
 *
 *  Copyright (C) 2011 Freescale Semiconductor, Inc.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef __ANATOP_DRIVERS_H__
#define __ANATOP_DRIVERS_H__


typedef void *anatop_handle;	/* Actually a ptr to a NS Node */

/* Device */
#define ACPI_MAX_HANDLES	10
struct anatop_handle_list {
	u32 count;
	anatop_handle handles[ACPI_MAX_HANDLES];
};

struct anatop_device {
	struct device dev;
	anatop_handle handle;		/* no handle for fixed hardware */
	char name[40];
	int id;
	void *driver_data;
};
struct anatop_device_id {
	__u8 id[ACPI_ID_LEN];
	kernel_ulong_t driver_data;
};
typedef int (*anatop_op_add) (struct anatop_device *device);
typedef int (*anatop_op_remove) (struct anatop_device *device, int type);
typedef int (*anatop_op_start) (struct anatop_device *device);
typedef int (*anatop_op_suspend) (struct anatop_device *device,
				pm_message_t state);
typedef int (*anatop_op_resume) (struct anatop_device *device);
typedef int (*anatop_op_bind) (struct anatop_device *device);
typedef int (*anatop_op_unbind) (struct anatop_device *device);
typedef void (*anatop_op_notify) (struct anatop_device *device, u32 event);

struct anatop_ops {
	u32 anatop_op_add:1;
	u32 anatop_op_start:1;
};

struct anatop_device_ops {
	anatop_op_add add;
	anatop_op_remove remove;
	anatop_op_start start;
	anatop_op_suspend suspend;
	anatop_op_resume resume;
	anatop_op_bind bind;
	anatop_op_unbind unbind;
	anatop_op_notify notify;
};

struct anatop_driver {
	char name[80];
	char class[80];
	const struct acpi_device_id *ids; /* Supported Hardware IDs */
	unsigned int flags;
	struct anatop_device_ops ops;
	struct device_driver drv;
	struct module *owner;
};



typedef u32 anatop_status;	/* All ANATOP Exceptions */

#define AT_OK                   (anatop_status) 0x0000
#define AT_ERROR                (anatop_status) 0x0001


#define ANATOP_MAX_STRING			80

/*
 * Please update drivers/acpi/debug.c and Documentation/acpi/debug.txt
 * if you add to this list.
 */
#define ANATOP_BUS_COMPONENT		0x00010000
#define ANATOP_AC_COMPONENT		0x00020000
#define ANATOP_BATTERY_COMPONENT		0x00040000
#define ANATOP_BUTTON_COMPONENT		0x00080000
#define ANATOP_SBS_COMPONENT		0x00100000
#define ANATOP_FAN_COMPONENT		0x00200000
#define ANATOP_PCI_COMPONENT		0x00400000
#define ANATOP_POWER_COMPONENT		0x00800000
#define ANATOP_CONTAINER_COMPONENT	0x01000000
#define ANATOP_SYSTEM_COMPONENT		0x02000000
#define ANATOP_THERMAL_COMPONENT		0x04000000
#define ANATOP_MEMORY_DEVICE_COMPONENT	0x08000000
#define ANATOP_VIDEO_COMPONENT		0x10000000
#define ANATOP_PROCESSOR_COMPONENT	0x20000000

/*
 * _HID definitions
 * HIDs must conform to ACPI spec(6.1.4)
 * Linux specific HIDs do not apply to this and begin with LNX:
 */

#define ANATOP_POWER_HID			"LNXPOWER"
#define ANATOP_PROCESSOR_OBJECT_HID	"LNXCPU"
#define ANATOP_SYSTEM_HID			"LNXSYSTM"
#define ANATOP_THERMAL_HID		"LNXTHERM"
#define ANATOP_BUTTON_HID_POWERF		"LNXPWRBN"
#define ANATOP_BUTTON_HID_SLEEPF		"LNXSLPBN"
#define ANATOP_VIDEO_HID			"LNXVIDEO"
#define ANATOP_BAY_HID			"LNXIOBAY"
#define ANATOP_DOCK_HID			"LNXDOCK"
/* Quirk for broken IBM BIOSes */
#define ANATOP_SMBUS_IBM_HID		"SMBUSIBM"

/*
 * For fixed hardware buttons, we fabricate acpi_devices with HID
 * ACPI_BUTTON_HID_POWERF or ACPI_BUTTON_HID_SLEEPF.  Fixed hardware
 * signals only an event; it doesn't supply a notification value.
 * To allow drivers to treat notifications from fixed hardware the
 * same as those from real devices, we turn the events into this
 * notification value.
 */
#define ANATOP_FIXED_HARDWARE_EVENT	0x100

static inline void *anatop_driver_data(struct anatop_device *d)
{
	return d->driver_data;
}

#endif /*__ANATOP_DRIVERS_H__*/
