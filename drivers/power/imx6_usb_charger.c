/*
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/power/imx6_usb_charger.h>
#include <linux/regmap.h>

#define HW_ANADIG_REG_3P0_SET	(0x00000124)
#define HW_ANADIG_REG_3P0_CLR	(0x00000128)
#define BM_ANADIG_REG_3P0_ENABLE_ILIMIT 0x00000004
#define BM_ANADIG_REG_3P0_ENABLE_LINREG 0x00000001

#define HW_ANADIG_USB1_CHRG_DETECT_SET	(0x000001b4)
#define HW_ANADIG_USB1_CHRG_DETECT_CLR	(0x000001b8)

#define BM_ANADIG_USB1_CHRG_DETECT_EN_B 0x00100000
#define BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B 0x00080000
#define BM_ANADIG_USB1_CHRG_DETECT_CHK_CONTACT 0x00040000

#define HW_ANADIG_USB1_VBUS_DET_STAT	(0x000001c0)

#define BM_ANADIG_USB1_VBUS_DET_STAT_VBUS_VALID 0x00000008

#define HW_ANADIG_USB1_CHRG_DET_STAT	(0x000001d0)

#define BM_ANADIG_USB1_CHRG_DET_STAT_DM_STATE 0x00000004
#define BM_ANADIG_USB1_CHRG_DET_STAT_CHRG_DETECTED 0x00000002
#define BM_ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT 0x00000001

static char *imx6_usb_charger_supplied_to[] = {
	"imx6_usb_charger",
};

static enum power_supply_property imx6_usb_charger_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,	/* Charger detected */
	POWER_SUPPLY_PROP_ONLINE,	/* VBUS online */
	POWER_SUPPLY_PROP_CURRENT_MAX,	/* Maximum current in mA */
};

static int imx6_usb_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct usb_charger *charger =
		container_of(psy, struct usb_charger, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = charger->present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->online;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = charger->max_current;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void disable_charger_detector(struct regmap *regmap)
{
	regmap_write(regmap, HW_ANADIG_USB1_CHRG_DETECT_SET,
		BM_ANADIG_USB1_CHRG_DETECT_EN_B |
		BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B);
}

static void disable_current_limiter(struct regmap *regmap)
{
	/* Disable the vdd3p0 current limiter */
	regmap_write(regmap, HW_ANADIG_REG_3P0_CLR,
			BM_ANADIG_REG_3P0_ENABLE_ILIMIT);
}

/* Return value if the charger is present */
static int imx6_usb_charger_detect(struct usb_charger *charger)
{
	struct regmap *regmap = charger->anatop;
	u32 val;
	int i, data_pin_contact_count = 0;

	/* Enable the vdd3p0 curret limiter */
	regmap_write(regmap, HW_ANADIG_REG_3P0_SET,
			BM_ANADIG_REG_3P0_ENABLE_LINREG |
			BM_ANADIG_REG_3P0_ENABLE_ILIMIT);

	/* check if vbus is valid */
	regmap_read(regmap, HW_ANADIG_USB1_VBUS_DET_STAT, &val);
	if (!(val & BM_ANADIG_USB1_VBUS_DET_STAT_VBUS_VALID)) {
		dev_err(charger->dev, "vbus is error\n");
		disable_current_limiter(regmap);
		return -EINVAL;
	}

	/* Enable charger detector */
	regmap_write(regmap, HW_ANADIG_USB1_CHRG_DETECT_CLR,
			BM_ANADIG_USB1_CHRG_DETECT_EN_B);
	/*
	 * - Do not check whether a charger is connected to the USB port
	 * - Check whether the USB plug has been in contact with each other
	 */
	regmap_write(regmap, HW_ANADIG_USB1_CHRG_DETECT_SET,
			BM_ANADIG_USB1_CHRG_DETECT_CHK_CONTACT |
			BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B);

	/* Check if plug is connected */
	for (i = 0; i < 100; i = i + 1) {
		regmap_read(regmap, HW_ANADIG_USB1_CHRG_DET_STAT, &val);
		if (val & BM_ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT) {
			if (data_pin_contact_count++ > 5)
			/* Data pin makes contact */
				break;
			else
				usleep_range(5000, 10000);
		} else {
			data_pin_contact_count = 0;
			msleep(20);
		}
	}

	if (i == 100) {
		dev_err(charger->dev,
			"VBUS is coming from a dedicated power supply.\n");
		disable_current_limiter(regmap);
		disable_charger_detector(regmap);
		return -ENXIO;
	}

	/*
	 * - Do check whether a charger is connected to the USB port
	 * - Do not Check whether the USB plug has been in contact with
	 * each other
	 */
	regmap_write(regmap, HW_ANADIG_USB1_CHRG_DETECT_CLR,
			BM_ANADIG_USB1_CHRG_DETECT_CHK_CONTACT |
			BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B);
	msleep(100);

	/* Check if it is a charger */
	regmap_read(regmap, HW_ANADIG_USB1_CHRG_DET_STAT, &val);
	if (!(val & BM_ANADIG_USB1_CHRG_DET_STAT_CHRG_DETECTED)) {
		dev_dbg(charger->dev, "It is a stardard downstream port\n");
		charger->psy.type = POWER_SUPPLY_TYPE_USB;
		charger->max_current = 500;
		disable_charger_detector(regmap);
	} else {
		/* It is a charger */
		disable_charger_detector(regmap);
		msleep(45);
	}

	disable_current_limiter(regmap);

	return 0;
}

/*
 * imx6_usb_vbus_connect - inform about VBUS connection
 * @charger: the usb charger
 *
 * Inform the charger VBUS is connected, vbus detect supplier should call it.
 * Besides, the USB device controller is expected to keep the dataline
 * pullups disabled.
 */
int imx6_usb_vbus_connect(struct usb_charger *charger)
{
	int ret;

	charger->online = 1;

	mutex_lock(&charger->lock);

	/* Start the 1st period charger detection. */
	ret = imx6_usb_charger_detect(charger);
	if (ret)
		dev_err(charger->dev,
				"Error occurs during detection: %d\n",
				ret);
	else
		charger->present = 1;

	mutex_unlock(&charger->lock);

	return ret;
}
EXPORT_SYMBOL(imx6_usb_vbus_connect);

/*
 * It must be called after dp is pulled up (from USB controller driver),
 * That is used to differentiate DCP and CDP
 */
int imx6_usb_charger_detect_post(struct usb_charger *charger)
{
	struct regmap *regmap = charger->anatop;
	int val;

	mutex_lock(&charger->lock);

	msleep(40);

	regmap_read(regmap, HW_ANADIG_USB1_CHRG_DET_STAT, &val);
	if (val & BM_ANADIG_USB1_CHRG_DET_STAT_DM_STATE) {
		dev_dbg(charger->dev, "It is a dedicate charging port\n");
		charger->psy.type = POWER_SUPPLY_TYPE_USB_DCP;
		charger->max_current = 1500;
	} else {
		dev_dbg(charger->dev, "It is a charging downstream port\n");
		charger->psy.type = POWER_SUPPLY_TYPE_USB_CDP;
		charger->max_current = 900;
	}

	power_supply_changed(&charger->psy);

	mutex_unlock(&charger->lock);

	return 0;
}
EXPORT_SYMBOL(imx6_usb_charger_detect_post);

/*
 * imx6_usb_vbus_disconnect - inform about VBUS disconnection
 * @charger: the usb charger
 *
 * Inform the charger that VBUS is disconnected. The charging will be
 * stopped and the charger properties cleared.
 */
int imx6_usb_vbus_disconnect(struct usb_charger *charger)
{
	charger->online = 0;
	charger->present = 0;
	charger->max_current = 0;
	charger->psy.type = POWER_SUPPLY_TYPE_MAINS;

	power_supply_changed(&charger->psy);

	return 0;
}
EXPORT_SYMBOL(imx6_usb_vbus_disconnect);

/*
 * imx6_usb_create_charger - create a USB charger
 * @charger: the charger to be initialized
 * @name: name for the power supply

 * Registers a power supply for the charger. The USB Controller
 * driver will call this after filling struct usb_charger.
 */
int imx6_usb_create_charger(struct usb_charger *charger,
		const char *name)
{
	struct power_supply	*psy = &charger->psy;

	if (!charger->dev)
		return -EINVAL;

	if (name)
		psy->name = name;
	else
		psy->name = "imx6_usb_charger";

	charger->bc = BATTERY_CHARGING_SPEC_1_2;
	mutex_init(&charger->lock);

	psy->type		= POWER_SUPPLY_TYPE_MAINS;
	psy->properties		= imx6_usb_charger_power_props;
	psy->num_properties	= ARRAY_SIZE(imx6_usb_charger_power_props);
	psy->get_property	= imx6_usb_charger_get_property;
	psy->supplied_to	= imx6_usb_charger_supplied_to;
	psy->num_supplicants	= sizeof(imx6_usb_charger_supplied_to)
		/ sizeof(char *);

	return power_supply_register(charger->dev, psy);
}
EXPORT_SYMBOL(imx6_usb_create_charger);

/*
 * imx6_usb_remove_charger - remove a USB charger
 * @charger: the charger to be removed
 *
 * Unregister the chargers power supply.
 */
void imx6_usb_remove_charger(struct usb_charger *charger)
{
	power_supply_unregister(&charger->psy);
}
EXPORT_SYMBOL(imx6_usb_remove_charger);
