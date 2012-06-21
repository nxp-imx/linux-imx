/*
 * Copyright 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * NOTICE: Currently, it only supports i.mx6q usb charger detect
 */

static void my_batt_ext_power_changed(struct power_supply *psy)
{
	int mA;
	union power_supply_propval ret = {0,};
	printk(KERN_INFO "my_batt_ext_power_changed!\n");
	if (!power_supply_am_i_supplied(psy)) {
		/* stop charging */
		printk(KERN_ERR "It is not usb supply!\n");
		return;
	}
	power_supply_get_supplier_property(psy,
		POWER_SUPPLY_PROP_ONLINE, &ret);

	printk(KERN_INFO "imx6 usb charger online:%d\n", ret.intval);

	power_supply_get_supplier_property(psy,
		POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		/* maximum milliamps we are allowed to draw from VBUS */
	mA = ret.intval;
	printk(KERN_INFO "imx6 usb charger limit mA: %d\n", mA);
}


static int usb_charger_get_property(struct power_supply *psy,
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

static void usb_charger_work(struct work_struct *data)
{
	int			ret;
	struct usb_charger	*charger =
			container_of(data, struct usb_charger, work);

	if (!charger->online)
		return;

	mutex_lock(&charger->lock);

	/* Start the primary charger detection. */
	if (charger->detect) {
		ret = charger->detect(charger);
		if (ret <= 0)
			dev_err(charger->dev, "Error occurs during usb charger \
					detection\n");
		else
			charger->present = ret;
	}

	switch (charger->psy.type) {
	case POWER_SUPPLY_TYPE_USB_DCP:
		charger->max_current = 1500;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		charger->max_current = 900;
		break;
	case POWER_SUPPLY_TYPE_USB: /* SDP */
		charger->max_current = 500;
	default:
		if (charger->dp_pullup)
			charger->dp_pullup(true);
		break;
	}

	power_supply_changed(&charger->psy);

	mutex_unlock(&charger->lock);
}

/* Return value if the charger is present */
static int usb_charger_detect(struct usb_charger *charger)
{
	void __iomem *addr = charger->charger_base_addr;
	int i;

	BUG_ON(!addr);

	/* Enable the vdd3p0 curret limiter */
	writel(BM_ANADIG_REG_3P0_ENABLE_LINREG |
			BM_ANADIG_REG_3P0_ENABLE_ILIMIT,
			addr + HW_ANADIG_REG_3P0_SET);

	/* check if vbus is valid */
	if (!(readl(addr + HW_ANADIG_USB1_VBUS_DET_STAT) &
			BM_ANADIG_USB1_VBUS_DET_STAT_VBUS_VALID)) {
		dev_err(charger->dev, "vbus is error\n");
		return 0;
	}

	/* Enable charger detector */
	writel(BM_ANADIG_USB1_CHRG_DETECT_EN_B,
			addr + HW_ANADIG_USB1_CHRG_DETECT_CLR);
	/*
	 * - Do not check whether a charger is connected to the USB port
	 * - Check whether the USB plug has been in contact with each other
	 */
	writel(BM_ANADIG_USB1_CHRG_DETECT_CHK_CONTACT
			| BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
			addr + HW_ANADIG_USB1_CHRG_DETECT_SET);

	/* Check if plug is connected */
	for (i = 0; i < 1000; i = i + 1) {
		if (readl(addr + HW_ANADIG_USB1_CHRG_DET_STAT) &
			BM_ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT) {
			dev_dbg(charger->dev, "Plug Contact = 1\n");
			break;
		} else if (i > 800) {
			dev_err(charger->dev, "VBUS is coming from a dedicated power supply.\n");
			return 0;
		} else
			msleep(1);
	}

	/*
	 * - Do check whether a charger is connected to the USB port
	 * - Do not Check whether the USB plug has been in contact with each other
	 */
	writel(BM_ANADIG_USB1_CHRG_DETECT_CHK_CONTACT
			| BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
			addr + HW_ANADIG_USB1_CHRG_DETECT_CLR);

	msleep(40);

	/* Check if it is a charger */
	if (!(readl(addr + HW_ANADIG_USB1_CHRG_DET_STAT) &
			BM_ANADIG_USB1_CHRG_DET_STAT_CHRG_DETECTED)) {
		dev_info(charger->dev, "It is a stardard downstream port\n");
		charger->psy.type = POWER_SUPPLY_TYPE_USB;

		/* Disable Charger detector */
		writel(BM_ANADIG_USB1_CHRG_DETECT_EN_B
			| BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
			addr + HW_ANADIG_USB1_CHRG_DETECT);

		/* Disable the vdd3p0 curret limiter */
		writel(BM_ANADIG_REG_3P0_ENABLE_ILIMIT,
				addr + HW_ANADIG_REG_3P0_CLR);
		return 1;
	}

	/* Begin to detect CDP and DCP */

	/* Disable Charger detector */
	writel(BM_ANADIG_USB1_CHRG_DETECT_EN_B
		| BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
		addr + HW_ANADIG_USB1_CHRG_DETECT);

	msleep(40);

	/* Disable the vdd3p0 curret limiter */
	writel(BM_ANADIG_REG_3P0_ENABLE_ILIMIT,
			addr + HW_ANADIG_REG_3P0_CLR);

	/* pull up dp */
	if (charger->dp_pullup)
		charger->dp_pullup(true);

	msleep(40);

	if ((readl(&dr_regs->portsc1) & PORTSCX_LINE_STATUS_KSTATE)) {
		dev_info(charger->dev, "It is a dedicate charging port\n");
		charger->psy.type = POWER_SUPPLY_TYPE_USB_DCP;
	} else {
		dev_info(charger->dev, "It is a charging downstream port\n");
		charger->psy.type = POWER_SUPPLY_TYPE_USB_CDP;
	}

	return 1;
}

static void usb_charger_init(struct usb_charger *charger)
{
	charger->bc = BATTERY_CHARGING_SPEC_1_2;
	charger->detect = usb_charger_detect;
}

/*
 * imx_usb_create_charger - create a USB charger
 * @charger: the charger to be initialized
 * @name: name for the power supply

 * Registers a power supply for the charger. The PHY/UDC driver will
 * call this after filling struct usb_charger. All the users are
 * expected to be in the supplied_to parameter.
 */
int imx_usb_create_charger(struct usb_charger *charger,
		const char *name)
{
	int			ret = 0;
	struct power_supply	*psy = &charger->psy;

	if (!charger->dev)
		return -EINVAL;

	if (name)
		psy->name = name;
	else
		psy->name = "usb_charger";

	usb_charger_init(charger);

	psy->type		= POWER_SUPPLY_TYPE_USB;
	psy->properties		= power_props;
	psy->num_properties	= ARRAY_SIZE(power_props);
	psy->get_property	= usb_charger_get_property;

	psy->supplied_to	= usb_charger_supplied_to;
	psy->num_supplicants	= sizeof(usb_charger_supplied_to)/sizeof(char *);
	psy->external_power_changed = my_batt_ext_power_changed;

	ret = power_supply_register(charger->dev, psy);
	if (ret)
		goto fail;

	mutex_init(&charger->lock);
	INIT_WORK(&charger->work, usb_charger_work);

fail:
	return ret;
}
EXPORT_SYMBOL(imx_usb_create_charger);

/*
 * imx_usb_remove_charger - remove a USB charger
 * @charger: the charger to be removed
 *
 * Unregister the chargers power supply.
 */
void imx_usb_remove_charger(struct usb_charger *charger)
{
	power_supply_unregister(&charger->psy);
}
EXPORT_SYMBOL(imx_usb_remove_charger);

/*
 * imx_usb_set_power - Set the maximum power allowed to draw
 * @charger: the usb charger
 * @mA: maximum current in milliamps
 *
 * Called from the controller after enumeration to inform the maximum
 * power from the configuration, and after bus suspend and resume.
 */
int imx_usb_set_power(struct usb_charger *charger, unsigned mA)
{
	if (!charger->online)
		return 0;

	if (charger->max_current == mA)
		return 0;

	charger->max_current = mA;

	power_supply_changed(&charger->psy);

	return 0;
}
EXPORT_SYMBOL(imx_usb_set_power);

/*
 * imx_usb_vbus_connect - inform about VBUS connection
 * @charger: the usb charger
 *
 * Inform the charger VBUS is connected. The USB device controller is
 * expected to keep the dataline pullups disabled until dp_pullup()
 * is called.
 */
int imx_usb_vbus_connect(struct usb_charger *charger)
{
	/* if charger is disabled, set usbcmd.rs=1 and return directly */
	if (!charger->enable) {
		if (charger->dp_pullup)
			charger->dp_pullup(true);
		return 0;
	}

	charger->online = 1;
	schedule_work(&charger->work);

	return 0;
}
EXPORT_SYMBOL(imx_usb_vbus_connect);

/*
 * imx_usb_vbus_disconnect - inform about VBUS disconnection
 * @charger: the usb charger
 *
 * Inform the charger that VBUS is disconnected. The charging will be
 * stopped and the charger properties cleared.
 */
int imx_usb_vbus_disconnect(struct usb_charger *charger)
{
	if (!charger->enable)
		return 0;

	/* in case, the charger detect is doing or pending */
	cancel_work_sync(&charger->work);

	charger->online = 0;
	charger->present = 0;
	charger->max_current = 0;
	charger->psy.type = POWER_SUPPLY_TYPE_USB;

	if (charger->disconnect)
		charger->disconnect(charger);

	power_supply_changed(&charger->psy);

	return 0;
}
EXPORT_SYMBOL(imx_usb_vbus_disconnect);
