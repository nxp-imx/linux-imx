/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __IMXUSB6_CHARGER_H
#define __IMXUSB6_CHARGER_H

#include <linux/power_supply.h>
enum battery_charging_spec {
	BATTERY_CHARGING_SPEC_NONE = 0,
	BATTERY_CHARGING_SPEC_UNKNOWN,
	BATTERY_CHARGING_SPEC_1_0,
	BATTERY_CHARGING_SPEC_1_1,
	BATTERY_CHARGING_SPEC_1_2,
};

struct usb_charger {
	/* The anatop regmap */
	struct regmap		*anatop;
	/* USB controller */
	struct device		*dev;
	struct power_supply	psy;
	struct mutex		lock;

	/* Compliant with Battery Charging Specification version (if any) */
	enum battery_charging_spec	bc;

	/* properties */
	unsigned		present:1;
	unsigned		online:1;
	unsigned		max_current;
	int	(*connect)(struct usb_charger *charger);
	int	(*disconnect)(struct usb_charger *charger);
	int	(*set_power)(struct usb_charger *charger, unsigned mA);

	int	(*detect)(struct usb_charger *charger);
};

#ifdef CONFIG_IMX6_USB_CHARGER
extern void imx6_usb_remove_charger(struct usb_charger *charger);
extern int imx6_usb_create_charger(struct usb_charger *charger,
		const char *name);
extern int imx6_usb_vbus_disconnect(struct usb_charger *charger);
extern int imx6_usb_vbus_connect(struct usb_charger *charger);
extern int imx6_usb_charger_detect_post(struct usb_charger *charger);
#else
void imx6_usb_remove_charger(struct usb_charger *charger)
{

}

int imx6_usb_create_charger(struct usb_charger *charger,
		const char *name)
{
	return -ENODEV;
}

int imx6_usb_vbus_disconnect(struct usb_charger *charger)
{
	return -ENODEV;
}

int imx6_usb_vbus_connect(struct usb_charger *charger)
{
	return -ENODEV;
}
int imx6_usb_charger_detect_post(struct usb_charger *charger)
{
	return -ENODEV;
}
#endif

#endif /* __IMXUSB6_CHARGER_H */
