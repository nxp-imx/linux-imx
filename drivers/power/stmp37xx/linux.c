/*
 * Linux glue to STMP3xxx battery state machine.
 *
 * Author: Steve Longerbeam <stevel@embeddedalley.com>
 *
 * Copyright (C) 2008 EmbeddedAlley Solutions Inc.
 * Copyright 2008-2009 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <mach/ddi_bc.h>
#include "ddi_bc_internal.h"
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <mach/regulator.h>
#include <mach/regs-power.h>
#include <mach/regs-usbphy.h>
#include <mach/platform.h>
#include <linux/delay.h>

#include <linux/interrupt.h>


struct stmp3xxx_info {
	struct device *dev;
	struct regulator *regulator;

	struct power_supply bat;
	struct power_supply ac;
	struct power_supply usb;

	ddi_bc_Cfg_t *sm_cfg;
	struct mutex sm_lock;
	struct timer_list sm_timer;
	struct work_struct sm_work;
	struct resource *vdd5v_irq;
	int is_ac_online;
#define USB_ONLINE      0x01
#define USB_REG_SET     0x02
#define USB_SM_RESTART  0x04
#define USB_SHUTDOWN    0x08
#define USB_N_SEND      0x10
	int is_usb_online;
};

#define to_stmp3xxx_info(x) container_of((x), struct stmp3xxx_info, bat)

/* There is no direct way to detect wall power presence, so assume the AC
 * power source is valid if 5V presents and USB device is disconnected.
 * If USB device is connected then assume that AC is offline and USB power
 * is online.
 */
#define is_usb_plugged()(__raw_readl(REGS_USBPHY_BASE + HW_USBPHY_STATUS) & \
		BM_USBPHY_STATUS_DEVPLUGIN_STATUS)
#define is_ac_online()	\
		(ddi_power_Get5vPresentFlag() ? (!is_usb_plugged()) : 0)
#define is_usb_online()	\
		(ddi_power_Get5vPresentFlag() ? (!!is_usb_plugged()) : 0)

/*
 * Power properties
 */
static enum power_supply_property stmp3xxx_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int stmp3xxx_power_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			/* ac online */
			val->intval = is_ac_online();
		else
			/* usb online */
			val->intval = is_usb_online();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
/*
 * Battery properties
 */
static enum power_supply_property stmp3xxx_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int stmp3xxx_bat_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct stmp3xxx_info *info = to_stmp3xxx_info(psy);
	ddi_bc_State_t state;
	ddi_bc_BrokenReason_t reason;
	int temp_alarm;
	int16_t temp_lo, temp_hi;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		state = ddi_bc_GetState();
		switch (state) {
		case DDI_BC_STATE_CONDITIONING:
		case DDI_BC_STATE_CHARGING:
		case DDI_BC_STATE_TOPPING_OFF:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case DDI_BC_STATE_DISABLED:
			val->intval = ddi_power_Get5vPresentFlag() ?
				POWER_SUPPLY_STATUS_NOT_CHARGING :
			POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		default:
			/* TODO: detect full */
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* is battery present */
		state = ddi_bc_GetState();
		switch (state) {
		case DDI_BC_STATE_WAITING_TO_CHARGE:
		case DDI_BC_STATE_DCDC_MODE_WAITING_TO_CHARGE:
		case DDI_BC_STATE_CONDITIONING:
		case DDI_BC_STATE_CHARGING:
		case DDI_BC_STATE_TOPPING_OFF:
		case DDI_BC_STATE_DISABLED:
			val->intval = 1;
			break;
		case DDI_BC_STATE_BROKEN:
			val->intval = !(ddi_bc_GetBrokenReason() ==
					DDI_BC_BROKEN_NO_BATTERY_DETECTED);
			break;
		default:
			val->intval = 0;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		temp_alarm = ddi_bc_RampGetDieTempAlarm();
		if (temp_alarm) {
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			state = ddi_bc_GetState();
			switch (state) {
			case DDI_BC_STATE_BROKEN:
				reason = ddi_bc_GetBrokenReason();
				val->intval =
				   (reason == DDI_BC_BROKEN_CHARGING_TIMEOUT) ?
					POWER_SUPPLY_HEALTH_DEAD :
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
				break;
			case DDI_BC_STATE_UNINITIALIZED:
				val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
				break;
			default:
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
				break;
			}
		}
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* uV */
		val->intval = ddi_power_GetBattery() * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* uA */
		val->intval = ddi_power_GetMaxBatteryChargeCurrent() * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&info->sm_lock);
		ddi_power_GetDieTemp(&temp_lo, &temp_hi);
		mutex_unlock(&info->sm_lock);
		val->intval = temp_lo + (temp_hi - temp_lo) / 2;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void state_machine_timer(unsigned long data)
{
	struct stmp3xxx_info *info = (struct stmp3xxx_info *)data;
	ddi_bc_Cfg_t *cfg = info->sm_cfg;
	int ret;

	/* schedule next call to state machine */
	mod_timer(&info->sm_timer,
		  jiffies + msecs_to_jiffies(cfg->u32StateMachinePeriod));

	ret = schedule_work(&info->sm_work);
	if (!ret)
		dev_dbg(info->dev, "state machine failed to schedule\n");

}
/*
 * Assumtion:
 * AC power can't be switched to USB w/o system reboot
 * and vice-versa
 */
static void state_machine_work(struct work_struct *work)
{
	struct stmp3xxx_info *info =
		container_of(work, struct stmp3xxx_info, sm_work);

	mutex_lock(&info->sm_lock);

	if (info->is_usb_online & USB_SHUTDOWN) {
		info->is_usb_online = 0;
		if (!info->regulator)
			goto out;
		regulator_set_current_limit(info->regulator, 0, 0);
		goto out;
	}

	if (is_ac_online()) {
		if (info->is_ac_online)
			goto done;

		/* ac supply connected */
		dev_info(info->dev, "changed power connection to ac/5v \n");

		info->is_ac_online = 1;
		info->is_usb_online = 0;
		ddi_bc_SetCurrentLimit(600 /*mA*/);
		ddi_power_execute_battery_to_5v_handoff();
		ddi_power_enable_5v_to_battery_handoff();
		goto done;
	}

	if (!is_usb_online())
		goto out;

	if (info->is_usb_online & USB_REG_SET)
		goto done;

	info->is_ac_online = 0;
	info->is_usb_online |= USB_ONLINE;

	if (!info->regulator) {
		info->regulator = regulator_get(NULL, "charger-1");
		if (!info->regulator || IS_ERR(info->regulator)) {
			dev_err(info->dev,
				"%s: failed to get regulator\n", __func__);
			info->regulator = NULL;
			ddi_bc_SetCurrentLimit(350 /*mA*/);
			ddi_power_execute_battery_to_5v_handoff();
			ddi_power_enable_5v_to_battery_handoff();
			goto done;
		} else
			regulator_set_mode(info->regulator,
					   REGULATOR_MODE_FAST);
	}

	if (!(info->is_usb_online & USB_N_SEND)) {
		info->is_usb_online |= USB_N_SEND;
	}

	if (regulator_set_current_limit(info->regulator, 150000, 150000)) {
		dev_err(info->dev, "reg_set_current(150000) failed\n");

		ddi_bc_SetCurrentLimit(0 /*mA*/);
		dev_dbg(info->dev, "charge current set to 0\n");
		mod_timer(&info->sm_timer, jiffies + msecs_to_jiffies(1000));
		goto done;
	}

	dev_dbg(info->dev, "%s: charge current set to 100mA\n", __func__);
	ddi_bc_SetCurrentLimit(100 /*mA*/);
	regulator_set_current_limit(info->regulator, 100000, 100000);
	if (info->is_usb_online & USB_SM_RESTART) {
		info->is_usb_online &= ~USB_SM_RESTART;
		ddi_bc_SetEnable();
	}

	info->is_usb_online |= USB_REG_SET;

	dev_info(info->dev, "changed power connection to usb/5v present\n");
	ddi_power_execute_battery_to_5v_handoff();
	ddi_power_enable_5v_to_battery_handoff();
	ddi_bc_SetEnable();

done:
	ddi_bc_StateMachine();
out:
	mutex_unlock(&info->sm_lock);
}

static int bc_sm_restart(struct stmp3xxx_info *info)
{
	ddi_bc_Status_t bcret;
	int ret = 0;

	mutex_lock(&info->sm_lock);

	/* ungate power clk */
	ddi_power_SetPowerClkGate(0);

	/*
	 * config battery charger state machine and move it to the Disabled
	 * state. This must be done before starting the state machine.
	 */
	bcret = ddi_bc_Init(info->sm_cfg);
	if (bcret != DDI_BC_STATUS_SUCCESS) {
		dev_err(info->dev, "state machine init failed: %d\n", bcret);
		ret = -EIO;
		goto out;
	}

	/*
	 * Check what power supply options we have right now. If
	 * we're able to do any battery charging, then set the
	 * appropriate current limit and enable. Otherwise, leave
	 * the battery charger disabled.
	 */
	if (is_ac_online()) {
		/* ac supply connected */
		dev_info(info->dev, "ac/5v present, enabling state machine\n");

		info->is_ac_online = 1;
		info->is_usb_online = 0;
		ddi_bc_SetCurrentLimit(600 /*mA*/);
		ddi_bc_SetEnable();
	} else if (is_usb_online()) {
		/* usb supply connected */
		dev_info(info->dev, "usb/5v present, enabling state machine\n");

		info->is_ac_online = 0;
		info->is_usb_online = USB_ONLINE | USB_SM_RESTART;
	} else {
		/* not powered */
		dev_info(info->dev, "%s: 5v not present\n", __func__);

		info->is_ac_online = 0;
		info->is_usb_online = 0;
		ddi_bc_SetDisable();
	}

	/* schedule first call to state machine */
	mod_timer(&info->sm_timer, jiffies + 1);
out:
	mutex_unlock(&info->sm_lock);
	return ret;
}

static irqreturn_t stmp3xxx_vdd5v_irq(int irq, void *cookie)
{
	struct stmp3xxx_info *info = (struct stmp3xxx_info *)cookie;

	if (ddi_power_Get5vPresentFlag()) {
		dev_info(info->dev, "5v present, reenable state machine\n");

		ddi_bc_SetEnable();

		/*
		 * We only ack/negate the interrupt here,
		 * as we can't decide yet if we really can
		 * switch to 5V (USB bits not ready)
		 */
		ddi_power_enable_5v_to_battery_handoff();
	} else {
		dev_info(info->dev, "5v went away, disabling state machine\n");

		ddi_bc_SetDisable();

		info->is_ac_online = 0;
		if (info->is_usb_online)
			info->is_usb_online = USB_SHUTDOWN;

		ddi_power_execute_5v_to_battery_handoff();
		ddi_power_enable_battery_to_5v_handoff();
		mod_timer(&info->sm_timer, jiffies + 1);

	}

	return IRQ_HANDLED;
}

static int stmp3xxx_bat_probe(struct platform_device *pdev)
{
	struct stmp3xxx_info *info;
	int ret = 0;

	if (!pdev->dev.platform_data) {
		printk(KERN_ERR "%s: missing platform data\n", __func__);
		return -ENODEV;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->vdd5v_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (info->vdd5v_irq == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}

	platform_set_drvdata(pdev, info);

	info->dev    = &pdev->dev;
	info->sm_cfg = pdev->dev.platform_data;

	/* initialize bat power_supply struct */
	info->bat.name           = "battery";
	info->bat.type           = POWER_SUPPLY_TYPE_BATTERY;
	info->bat.properties     = stmp3xxx_bat_props;
	info->bat.num_properties = ARRAY_SIZE(stmp3xxx_bat_props);
	info->bat.get_property   = stmp3xxx_bat_get_property;

	/* initialize ac power_supply struct */
	info->ac.name           = "ac";
	info->ac.type           = POWER_SUPPLY_TYPE_MAINS;
	info->ac.properties     = stmp3xxx_power_props;
	info->ac.num_properties = ARRAY_SIZE(stmp3xxx_power_props);
	info->ac.get_property   = stmp3xxx_power_get_property;

	/* initialize usb power_supply struct */
	info->usb.name           = "usb";
	info->usb.type           = POWER_SUPPLY_TYPE_USB;
	info->usb.properties     = stmp3xxx_power_props;
	info->usb.num_properties = ARRAY_SIZE(stmp3xxx_power_props);
	info->usb.get_property   = stmp3xxx_power_get_property;

	init_timer(&info->sm_timer);
	info->sm_timer.data = (unsigned long)info;
	info->sm_timer.function = state_machine_timer;

	mutex_init(&info->sm_lock);
	INIT_WORK(&info->sm_work, state_machine_work);

	/* init LRADC channels to measure battery voltage and die temp */
	ddi_power_init_battery();
	__raw_writel(BM_POWER_5VCTRL_ENABLE_LINREG_ILIMIT,
		REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	ret = bc_sm_restart(info);
	if (ret)
		goto free_info;

	ret = request_irq(info->vdd5v_irq->start,
			stmp3xxx_vdd5v_irq, IRQF_DISABLED | IRQF_SHARED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}

	ret = power_supply_register(&pdev->dev, &info->bat);
	if (ret) {
		dev_err(info->dev, "failed to register battery\n");
		goto free_irq;
	}

	ret = power_supply_register(&pdev->dev, &info->ac);
	if (ret) {
		dev_err(info->dev, "failed to register ac power supply\n");
		goto unregister_bat;
	}

	ret = power_supply_register(&pdev->dev, &info->usb);
	if (ret) {
		dev_err(info->dev, "failed to register usb power supply\n");
		goto unregister_ac;
	}

	/* enable usb device presence detection */
	__raw_writel(BM_USBPHY_CTRL_ENDEVPLUGINDETECT,
			REGS_USBPHY_BASE + HW_USBPHY_CTRL_SET);

	return 0;

unregister_ac:
	power_supply_unregister(&info->ac);
unregister_bat:
	power_supply_unregister(&info->bat);
free_irq:
	free_irq(info->vdd5v_irq->start, pdev);
stop_sm:
	ddi_bc_ShutDown();
free_info:
	kfree(info);
	return ret;
}

static int stmp3xxx_bat_remove(struct platform_device *pdev)
{
	struct stmp3xxx_info *info = platform_get_drvdata(pdev);

	if (info->regulator)
		regulator_put(info->regulator);
	free_irq(info->vdd5v_irq->start, pdev);
	ddi_bc_ShutDown();
	power_supply_unregister(&info->usb);
	power_supply_unregister(&info->ac);
	power_supply_unregister(&info->bat);
	return 0;
}

static void stmp3xxx_bat_shutdown(struct platform_device *pdev)
{
	ddi_bc_ShutDown();
}


#ifdef CONFIG_PM

static int stmp3xxx_bat_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct stmp3xxx_info *info = platform_get_drvdata(pdev);

	mutex_lock(&info->sm_lock);

	/* disable 5v irq */
	__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	ddi_bc_SetDisable();
	/* cancel state machine timer */
	del_timer_sync(&info->sm_timer);

	mutex_unlock(&info->sm_lock);
	return 0;
}

static int stmp3xxx_bat_resume(struct platform_device *pdev)
{
	struct stmp3xxx_info *info = platform_get_drvdata(pdev);
	ddi_bc_Cfg_t *cfg = info->sm_cfg;

	mutex_lock(&info->sm_lock);

	if (is_ac_online()) {
		/* ac supply connected */
		dev_info(info->dev, "ac/5v present, enabling state machine\n");

		info->is_ac_online = 1;
		info->is_usb_online = 0;
		ddi_bc_SetCurrentLimit(600 /*mA*/);
		ddi_bc_SetEnable();
	} else if (is_usb_online()) {
		/* usb supply connected */
		dev_info(info->dev, "usb/5v present, enabling state machine\n");

		info->is_ac_online = 0;
		info->is_usb_online = 1;
		ddi_bc_SetCurrentLimit(350 /*mA*/);
		ddi_bc_SetEnable();
	} else {
		/* not powered */
		dev_info(info->dev, "%s: 5v not present\n", __func__);

		info->is_ac_online = 0;
		info->is_usb_online = 0;
	}

	/* enable 5v irq */
	__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);

	/* reschedule calls to state machine */
	mod_timer(&info->sm_timer,
		  jiffies + msecs_to_jiffies(cfg->u32StateMachinePeriod));

	mutex_unlock(&info->sm_lock);
	return 0;
}

#else
#define stmp3xxx_bat_suspend NULL
#define stmp3xxx_bat_resume  NULL
#endif

static struct platform_driver stmp3xxx_batdrv = {
	.probe		= stmp3xxx_bat_probe,
	.remove		= stmp3xxx_bat_remove,
	.shutdown       = stmp3xxx_bat_shutdown,
	.suspend	= stmp3xxx_bat_suspend,
	.resume		= stmp3xxx_bat_resume,
	.driver		= {
		.name	= "stmp3xxx-battery",
		.owner	= THIS_MODULE,
	},
};

static int __init stmp3xxx_bat_init(void)
{
	return platform_driver_register(&stmp3xxx_batdrv);
}

static void __exit stmp3xxx_bat_exit(void)
{
	platform_driver_unregister(&stmp3xxx_batdrv);
}

module_init(stmp3xxx_bat_init);
module_exit(stmp3xxx_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steve Longerbeam <stevel@embeddedalley.com>");
MODULE_DESCRIPTION("Linux glue to STMP3xxx battery state machine");
