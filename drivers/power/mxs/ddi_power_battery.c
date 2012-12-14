/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


/* addtogroup ddi_power */
/*  @{ */
/*  */
/* Copyright(C) 2005 SigmaTel, Inc. */
/*  */
/* file ddi_power_battery.c */
/* brief Implementation file for the power driver battery charger. */
/*  */

/*   Includes and external references */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/processor.h> /* cpu_relax */
#include <mach/hardware.h>
#include <mach/ddi_bc.h>
#include <mach/lradc.h>
#include <mach/regs-power.h>
#include <mach/regs-lradc.h>
#include <mach/lradc.h>
#include "ddi_bc_internal.h"

/* brief Base voltage to start battery calculations for LiIon */
#define BATT_BRWNOUT_LIION_BASE_MV 2800
/* brief Constant to help with determining whether to round up or */
/*  not during calculation */
#define BATT_BRWNOUT_LIION_CEILING_OFFSET_MV 39
/* brief Number of mV to add if rounding up in LiIon mode */
#define BATT_BRWNOUT_LIION_LEVEL_STEP_MV 40
/* brief Constant value to be calculated by preprocessing */
#define BATT_BRWNOUT_LIION_EQN_CONST \
	(BATT_BRWNOUT_LIION_BASE_MV - BATT_BRWNOUT_LIION_CEILING_OFFSET_MV)
/* brief Base voltage to start battery calculations for Alkaline/NiMH */
#define BATT_BRWNOUT_ALKAL_BASE_MV 800
/* brief Constant to help with determining whether to round up or */
/*  not during calculation */
#define BATT_BRWNOUT_ALKAL_CEILING_OFFSET_MV 19
/* brief Number of mV to add if rounding up in Alkaline/NiMH mode */
#define BATT_BRWNOUT_ALKAL_LEVEL_STEP_MV 20
/* brief Constant value to be calculated by preprocessing */
#define BATT_BRWNOUT_ALKAL_EQN_CONST \
	(BATT_BRWNOUT_ALKAL_BASE_MV - BATT_BRWNOUT_ALKAL_CEILING_OFFSET_MV)

#define GAIN_CORRECTION 1012    /* 1.012 */

#define VBUSVALID_THRESH_2_90V		0x0
#define VBUSVALID_THRESH_4_00V		0x1
#define VBUSVALID_THRESH_4_10V		0x2
#define VBUSVALID_THRESH_4_20V		0x3
#define VBUSVALID_THRESH_4_30V		0x4
#define VBUSVALID_THRESH_4_40V		0x5
#define VBUSVALID_THRESH_4_50V		0x6
#define VBUSVALID_THRESH_4_60V		0x7

#define LINREG_OFFSET_STEP_BELOW	0x2
#define BP_POWER_BATTMONITOR_BATT_VAL	16
#define BP_POWER_CHARGE_BATTCHRG_I	0
#define BP_POWER_CHARGE_STOP_ILIMIT	8

#define VDD4P2_ENABLED

#define DDI_POWER_BATTERY_XFER_THRESHOLD_MV 3200


#ifndef BATTERY_VOLTAGE_CMPTRIP100_THRESHOLD_MV
#define BATTERY_VOLTAGE_CMPTRIP100_THRESHOLD_MV 4000
#endif

#ifndef BATTERY_VOLTAGE_CMPTRIP105_THRESHOLD_MV
#define BATTERY_VOLTAGE_CMPTRIP105_THRESHOLD_MV 3800
#endif

/* #define DEBUG_IRQS */

/* to be re-enabled once FIQ functionality is added */
#define DISABLE_VDDIO_BO_PROTECTION

#ifdef CONFIG_ARCH_MX28
#define BM_POWER_STS_VBUSVALID BM_POWER_STS_VBUSVALID0
#endif

/* Globals & Variables */



/* Select your 5V Detection method */

static ddi_power_5vDetection_t DetectionMethod =
			DDI_POWER_5V_VDD5V_GT_VDDIO;
/* static ddi_power_5vDetection_t DetectionMethod = DDI_POWER_5V_VBUSVALID; */


/* Code */


#if 0
static void dump_regs(void)
{
	printk("HW_POWER_CHARGE      0x%08x\n", __raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE));
	printk("HW_POWER_STS         0x%08x\n", __raw_readl(REGS_POWER_BASE + HW_POWER_STS));
	printk("HW_POWER_BATTMONITOR 0x%08x\n", __raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR));
}
#endif

/*  This array maps bit numbers to current increments, as used in the register */
/*  fields HW_POWER_CHARGE.STOP_ILIMIT and HW_POWER_CHARGE.BATTCHRG_I. */
static const uint16_t currentPerBit[] = {  10,  20,  50, 100, 200, 400 };

uint16_t ddi_power_convert_current_to_setting(uint16_t u16Current)
{
	int       i;
	uint16_t  u16Mask;
	uint16_t  u16Setting = 0;

	/* Scan across the bit field, adding in current increments. */
	u16Mask = (0x1 << 5);

	for (i = 5; (i >= 0) && (u16Current > 0); i--, u16Mask >>= 1) {
		if (u16Current >= currentPerBit[i]) {
			u16Current -= currentPerBit[i];
			u16Setting |= u16Mask;
		}
	}

	/* Return the result. */
	return u16Setting;
}


/*  See hw_power.h for details. */

uint16_t ddi_power_convert_setting_to_current(uint16_t u16Setting)
{
	int       i;
	uint16_t  u16Mask;
	uint16_t  u16Current = 0;

	/* Scan across the bit field, adding in current increments. */
	u16Mask = (0x1 << 5);

	for (i = 5; i >= 0; i--, u16Mask >>= 1) {
		if (u16Setting & u16Mask)
			u16Current += currentPerBit[i];
	}

	/* Return the result. */
	return u16Current;
}

void ddi_power_Enable5vDetection(void)
{
	u32 val;
	/* Disable hardware power down when 5V is inserted or removed */
	__raw_writel(BM_POWER_5VCTRL_PWDN_5VBRNOUT,
		REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	/* Enabling VBUSVALID hardware detection even if VDD5V_GT_VDDIO
	 * is the detection method being used for 5V status (hardware
	 * or software).  This is in case any other drivers (such as
	 * USB) are specifically monitoring VBUSVALID status
	 */
	__raw_writel(BM_POWER_5VCTRL_VBUSVALID_5VDETECT,
			REGS_POWER_BASE + HW_POWER_5VCTRL_SET);

	/* Set 5V detection threshold to 4.3V for VBUSVALID. */
	__raw_writel(
		BF_POWER_5VCTRL_VBUSVALID_TRSH(VBUSVALID_THRESH_4_30V),
			REGS_POWER_BASE + HW_POWER_5VCTRL_SET);

	/* gotta set LINREG_OFFSET to STEP_BELOW according to manual */
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	val &= ~(BM_POWER_VDDIOCTRL_LINREG_OFFSET);
	val |= BF_POWER_VDDIOCTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);

	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL);
	val &= ~(BM_POWER_VDDACTRL_LINREG_OFFSET);
	val |= BF_POWER_VDDACTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDACTRL);

	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	val &= ~(BM_POWER_VDDDCTRL_LINREG_OFFSET);
	val |= BF_POWER_VDDDCTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	/* Clear vbusvalid interrupt flag */
	__raw_writel(BM_POWER_CTRL_VBUSVALID_IRQ,
				REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ,
				REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	/* enable vbusvalid irq */


	/* enable 5V Detection interrupt vbusvalid irq */
	switch (DetectionMethod) {
	case DDI_POWER_5V_VBUSVALID:
		/* Check VBUSVALID for 5V present */
		__raw_writel(BM_POWER_CTRL_ENIRQ_VBUS_VALID,
				REGS_POWER_BASE + HW_POWER_CTRL_SET);
		break;
	case DDI_POWER_5V_VDD5V_GT_VDDIO:
		/* Check VDD5V_GT_VDDIO for 5V present */
		__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO,
				REGS_POWER_BASE + HW_POWER_CTRL_SET);
		break;
	}
}

/*
 * This function prepares the hardware for a 5V-to-battery handoff. It assumes
 * the current configuration is using 5V as the power source.  The 5V
 * interrupt will be set up for a 5V removal.
 */
void ddi_power_enable_5v_to_battery_handoff(void)
{
	/* Clear vbusvalid interrupt flag */
	__raw_writel(BM_POWER_CTRL_VBUSVALID_IRQ,
				REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ,
				REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	/* detect 5v unplug */
	__raw_writel(BM_POWER_CTRL_POLARITY_VBUSVALID,
				REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO,
				REGS_POWER_BASE + HW_POWER_CTRL_CLR);

#ifndef VDD4P2_ENABLED
	/* Enable automatic transition to DCDC */
	__raw_writel(BM_POWER_5VCTRL_DCDC_XFER,
				REGS_POWER_BASE + HW_POWER_5VCTRL_SET);
#endif
}

/*
 * This function will handle all the power rail transitions necesarry to power
 * the chip from the battery when it was previously powered from the 5V power
 * source.
 */
void ddi_power_execute_5v_to_battery_handoff(void)
{
	int val;
#ifdef VDD4P2_ENABLED
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2);
	val &= ~(BM_POWER_DCDC4P2_ENABLE_DCDC | BM_POWER_DCDC4P2_ENABLE_4P2);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_DCDC4P2);

	__raw_writel(BM_POWER_5VCTRL_PWD_CHARGE_4P2,
				REGS_POWER_BASE + HW_POWER_5VCTRL_SET);

	/* make VBUSVALID_TRSH 4400mV and set PWD_CHARGE_4P2 */
	__raw_writel(BM_POWER_5VCTRL_VBUSVALID_TRSH,
		REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	__raw_writel(BF_POWER_5VCTRL_VBUSVALID_TRSH(VBUSVALID_THRESH_4_40V),
		REGS_POWER_BASE + HW_POWER_5VCTRL_SET);

#else
	/* VDDD has different configurations depending on the battery type */
	/* and battery level. */

	/* For LiIon battery, we will use the DCDC to power VDDD. */
	/* Use LinReg offset for DCDC mode. */
	__raw_writel(BF_POWER_VDDDCTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW),
				HW_POWER_BASE + HW_POWER_VDDDCTRL_SET);
	/* Turn on the VDDD DCDC output and turn off the VDDD LinReg output. */
	__raw_writel(BM_POWER_VDDDCTRL_DISABLE_FET,
				HW_POWER_BASE + HW_POWER_VDDDCTRL_CLR);

	__raw_writel(BM_POWER_VDDDCTRL_ENABLE_LINREG,
				HW_POWER_BASE + HW_POWER_VDDDCTRL_CLR);
	/* Make sure stepping is enabled when using DCDC. */
	__raw_writel(BM_POWER_VDDDCTRL_DISABLE_STEPPING,
				HW_POWER_BASE + HW_POWER_VDDDCTRL_CLR);

	/* Power VDDA and VDDIO from the DCDC. */

	/* Use LinReg offset for DCDC mode. */
	__raw_writel(BF_POWER_VDDACTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW),
				HW_POWER_BASE + HW_POWER_VDDACTRL_SET);
	/* Turn on the VDDA DCDC converter output and turn off LinReg output. */
	__raw_writel(BM_POWER_VDDACTRL_DISABLE_FET,
				HW_POWER_BASE + HW_POWER_VDDACTRL_CLR);
	__raw_writel(BM_POWER_VDDACTRL_ENABLE_LINREG,
				HW_POWER_BASE + HW_POWER_VDDACTRL_CLR);

	/* Make sure stepping is enabled when using DCDC. */
	__raw_writel(BM_POWER_VDDACTRL_DISABLE_STEPPING,
				HW_POWER_BASE + HW_POWER_VDDACTRL_CLR);

	/* Use LinReg offset for DCDC mode. */
	__raw_writel(BF_POWER_VDDIOCTRL_LINREG_OFFSET(
					LINREG_OFFSET_STEP_BELOW
						),
				HW_POWER_BASE + HW_POWER_VDDIOCTRL_SET);

	/* Turn on the VDDIO DCDC output and turn on the LinReg output.*/
	__raw_writel(BM_POWER_VDDIOCTRL_DISABLE_FET,
				HW_POWER_BASE + HW_POWER_VDDIOCTRL_CLR);

	__raw_writel(BM_POWER_5VCTRL_ILIMIT_EQ_ZERO,
				HW_POWER_BASE + HW_POWER_5VCTRL_CLR_CLR);

	/* Make sure stepping is enabled when using DCDC. */
	__raw_writel(BM_POWER_VDDIOCTRL_DISABLE_STEPPING,
				HW_POWER_BASE + HW_POWER_VDDIOCTRL_CLR);
#endif

}

/*
 * This function sets up battery-to-5V handoff. The power switch from
 * battery to 5V is automatic. This funtion enables the 5V present detection
 * such that the 5V interrupt can be generated if it is enabled. (The interrupt
 * handler can inform software the 5V present event.) To deal with noise or
 * a high current, this function enables DCDC1/2 based on the battery mode.
 */
void ddi_power_enable_battery_to_5v_handoff(void)
{
	/* Clear vbusvalid interrupt flag */
	__raw_writel(BM_POWER_CTRL_VBUSVALID_IRQ,
				REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ,
				REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	/* detect 5v plug-in */
	__raw_writel(BM_POWER_CTRL_POLARITY_VBUSVALID,
				REGS_POWER_BASE + HW_POWER_CTRL_SET);
	__raw_writel(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO,
				REGS_POWER_BASE + HW_POWER_CTRL_SET);

#ifndef VDD4P2_ENABLED
	/* Force current from 5V to be zero by disabling its entry source. */
	__raw_writel(BM_POWER_5VCTRL_ILIMIT_EQ_ZERO,
				REGS_POWER_BASE + HW_POWER_5VCTRL_SET);
#endif
	/* Allow DCDC be to active when 5V is present. */
	__raw_writel(BM_POWER_5VCTRL_ENABLE_DCDC,
				REGS_POWER_BASE + HW_POWER_5VCTRL_SET);
}

/* This function handles the transitions on each of theVDD5V_GT_VDDIO power
 * rails necessary to power the chip from the 5V power supply when it was
 * previously powered from the battery power supply.
 */
void ddi_power_execute_battery_to_5v_handoff(void)
{

#ifdef VDD4P2_ENABLED
	ddi_power_Enable4p2(450);
#else
	/* Disable the DCDC during 5V connections. */
	__raw_writel(BM_POWER_5VCTRL_ENABLE_DCDC,
				HW_POWER_BAE + HW_POWER_5VCTRL_CLR);

	/* Power the VDDD/VDDA/VDDIO rail from the linear regulator.  The DCDC */
	/* is ready to automatically power the chip when 5V is removed. */
	/* Use this configuration when powering from 5V */

	/* Use LinReg offset for LinReg mode */
	__raw_writel(BF_POWER_VDDDCTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW),
				HW_POWER_BAE + HW_POWER_VDDDCTRL_SET);

	/* Turn on the VDDD LinReg and turn on the VDDD DCDC output.  The */
	/* ENABLE_DCDC must be cleared to avoid LinReg and DCDC conflict. */
	__raw_writel(BM_POWER_VDDDCTRL_ENABLE_LINREG,
				HW_POWER_BAE + HW_POWER_VDDDCTRL_SET);
	__raw_writel(BM_POWER_VDDDCTRL_DISABLE_FET,
				HW_POWER_BAE + HW_POWER_VDDDCTRL_CLR);

	/* Make sure stepping is disabled when using linear regulators */
	__raw_writel(BM_POWER_VDDDCTRL_DISABLE_STEPPING,
				HW_POWER_BAE + HW_POWER_VDDDCTRL_SET);

	/* Use LinReg offset for LinReg mode */
	__raw_writel(BM_POWER_VDDACTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW),
				HW_POWER_BAE + HW_POWER_VDDACTRL_SET);


	/* Turn on the VDDA LinReg output and prepare the DCDC for transfer. */
	/* ENABLE_DCDC must be clear to avoid DCDC and LinReg conflict. */
	stmp3xxx_set(BM_POWER_VDDACTRL_ENABLE_LINREG,
				HW_POWER_BASE + HW_POWER_VDDACTRL_SET);
	__raw_writel(BM_POWER_VDDACTRL_DISABLE_FET,
				HW_POWER_BASE + HW_POWER_VDDACTRL_CLR);

	/* Make sure stepping is disabled when using linear regulators */
	__raw_writel(BM_POWER_VDDACTRL_DISABLE_STEPPING,
				 HW_POWER_BASE + HW_POWER_VDDACTRL_SET);

	/* Use LinReg offset for LinReg mode. */
	__raw_writel(BF_POWER_VDDIOCTRL_LINREG_OFFSET(
					LINREG_OFFSET_STEP_BELOW),
				HW_POWER_BASE + HW_POWER_VDDIOCTRL_SET);

	/* Turn on the VDDIO LinReg output and prepare the VDDIO DCDC output. */
	/* ENABLE_DCDC must be cleared to prevent DCDC and LinReg conflict. */
	__raw_writel(BM_POWER_VDDIOCTRL_DISABLE_FET,
				HW_POWER_BASE + HW_POWER_VDDIOCTRL_CLR);
	__raw_writel(BM_POWER_5VCTRL_ILIMIT_EQ_ZERO,
			REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	/* Make sure stepping is disabled when using DCDC. */
	__raw_writel(BM_POWER_VDDIOCTRL_DISABLE_STEPPING,
			REGS_POWER_BASE + HW_POWER_VDDIOCTRL_SET);
#endif
}


void ddi_power_Start4p2Dcdc(bool battery_ready)
{
	uint32_t temp_reg, old_values;
	bool vdda_pwdn = false, vddd_pwdn = false, vddio_pwdn = false;

#ifndef CONFIG_ARCH_MX28
	/* set vbusvalid threshold to 2.9V because of errata */
	__raw_writel(BM_POWER_5VCTRL_VBUSVALID_TRSH,
			REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);
#endif

#if 0
	if (battery_ready)
		ddi_power_EnableBatteryIrq();
	else
		enable_4p2_fiq_shutdown();
#endif

	/* enable hardware shutdown on battery brownout */
	__raw_writel(
			BM_POWER_BATTMONITOR_PWDN_BATTBRNOUT |
			__raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR),
			REGS_POWER_BASE + HW_POWER_BATTMONITOR);

	/* set VBUS DROOP threshold to 4.3V */
	__raw_writel(BM_POWER_5VCTRL_VBUSDROOP_TRSH,
			REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	/* turn of vbus valid detection.  Part of errate
	 * workaround. */
	__raw_writel(BM_POWER_5VCTRL_PWRUP_VBUS_CMPS,
			REGS_POWER_BASE + HW_POWER_5VCTRL_SET);

	__raw_writel(BM_POWER_5VCTRL_VBUSVALID_5VDETECT,
			REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	if (__raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL)
		& BM_POWER_VDDIOCTRL_PWDN_BRNOUT)
		vddio_pwdn = true;

	if (__raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL)
		& BM_POWER_VDDDCTRL_PWDN_BRNOUT)
		vddd_pwdn = true;

	if (__raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL)
		& BM_POWER_VDDACTRL_PWDN_BRNOUT)
		vdda_pwdn = true;

	__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL)
	& (~BM_POWER_VDDACTRL_PWDN_BRNOUT),
	REGS_POWER_BASE + HW_POWER_VDDACTRL);

	__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL)
	& (~BM_POWER_VDDDCTRL_PWDN_BRNOUT),
	REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL)
	& (~BM_POWER_VDDIOCTRL_PWDN_BRNOUT),
	REGS_POWER_BASE + HW_POWER_VDDIOCTRL);

	if ((__raw_readl(REGS_POWER_BASE + HW_POWER_STS)
		& BM_POWER_STS_VDDIO_BO) == 0)
		__raw_writel(BM_POWER_CTRL_VDDIO_BO_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	if ((__raw_readl(REGS_POWER_BASE + HW_POWER_STS)
		& BM_POWER_STS_VDDD_BO) == 0)
		__raw_writel(BM_POWER_CTRL_VDDD_BO_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	if ((__raw_readl(REGS_POWER_BASE + HW_POWER_STS)
		& BM_POWER_STS_VDDA_BO) == 0)
		__raw_writel(BM_POWER_CTRL_VDDA_BO_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	temp_reg = (BM_POWER_CTRL_ENIRQ_VDDD_BO |
		BM_POWER_CTRL_ENIRQ_VDDA_BO |
		BM_POWER_CTRL_ENIRQ_VDDIO_BO |
		BM_POWER_CTRL_ENIRQ_VDD5V_DROOP |
		BM_POWER_CTRL_ENIRQ_VBUS_VALID);

	/* save off old brownout enable values */
	old_values = __raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
		temp_reg;

	/* disable irqs affected by errata */
	__raw_writel(temp_reg, REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	/* Enable DCDC from 4P2 */
	__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2) |
			BM_POWER_DCDC4P2_ENABLE_DCDC,
			REGS_POWER_BASE + HW_POWER_DCDC4P2);

	/* give a delay to check for errate noise problem */
	mdelay(1);

	temp_reg = (BM_POWER_CTRL_VDDD_BO_IRQ |
		BM_POWER_CTRL_VDDA_BO_IRQ |
		BM_POWER_CTRL_VDDIO_BO_IRQ |
		BM_POWER_CTRL_VDD5V_DROOP_IRQ |
		BM_POWER_CTRL_VBUSVALID_IRQ);

	__raw_writel(temp_reg, REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	/* stay in this loop until the false brownout indciations
	 * no longer occur or until 5V actually goes away
	 */
	while ((__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) & temp_reg) &&
		!(__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
				BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ)) {
		__raw_writel(temp_reg, REGS_POWER_BASE + HW_POWER_CTRL_CLR);

		mdelay(1);
	}
	/* revert to previous enable irq values */
	__raw_writel(old_values, REGS_POWER_BASE + HW_POWER_CTRL_SET);

	if (vdda_pwdn)
		__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL)
		| BM_POWER_VDDACTRL_PWDN_BRNOUT,
		REGS_POWER_BASE + HW_POWER_VDDACTRL);

	if (vddd_pwdn)
		__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL)
		| BM_POWER_VDDDCTRL_PWDN_BRNOUT,
		REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	if (vddio_pwdn)
		__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL)
		| BM_POWER_VDDIOCTRL_PWDN_BRNOUT,
		REGS_POWER_BASE + HW_POWER_VDDIOCTRL);

	if (DetectionMethod == DDI_POWER_5V_VBUSVALID)
		__raw_writel(BM_POWER_5VCTRL_VBUSVALID_5VDETECT,
			REGS_POWER_BASE + HW_POWER_5VCTRL_SET);
}


/* set the optimal CMPTRIP for the best possible 5V
 * disconnection handling but without drawing power
 * from the power on a stable 4p2 rails (at 4.2V).
 */
void ddi_power_handle_cmptrip(void)
{
	enum ddi_power_5v_status pmu_5v_status;
	uint32_t temp = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2);
	temp &= ~(BM_POWER_DCDC4P2_CMPTRIP);

	pmu_5v_status = ddi_power_GetPmu5vStatus();

	/* CMPTRIP should remain at 31 when 5v is disconnected
	 * or 5v is connected but hasn't been handled yet
	 */
	if (pmu_5v_status != existing_5v_connection)
		temp |= (31 << BP_POWER_DCDC4P2_CMPTRIP);
	else if (ddi_power_GetBattery() >
			BATTERY_VOLTAGE_CMPTRIP100_THRESHOLD_MV)
		temp |= (1 << BP_POWER_DCDC4P2_CMPTRIP);
	else if (ddi_power_GetBattery() >
			BATTERY_VOLTAGE_CMPTRIP105_THRESHOLD_MV)
		temp |= (24 << BP_POWER_DCDC4P2_CMPTRIP);
	else
		temp |= (31 << BP_POWER_DCDC4P2_CMPTRIP);


	__raw_writel(temp, REGS_POWER_BASE + HW_POWER_DCDC4P2);
}

void ddi_power_Init4p2Params(void)
{
	uint32_t temp;

	ddi_power_handle_cmptrip();

	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2);

	/* DROPOUT CTRL to 10, TRG to 0 */
	temp &= ~(BM_POWER_DCDC4P2_TRG | BM_POWER_DCDC4P2_DROPOUT_CTRL);
	temp |= (0xa << BP_POWER_DCDC4P2_DROPOUT_CTRL);

	__raw_writel(temp, REGS_POWER_BASE + HW_POWER_DCDC4P2);


	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL);

	/* HEADROOM_ADJ to 4, CHARGE_4P2_ILIMIT to 0 */
	temp &= ~(BM_POWER_5VCTRL_HEADROOM_ADJ |
			BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT);
	temp |= (4 << BP_POWER_5VCTRL_HEADROOM_ADJ);

}

bool ddi_power_IsBattRdyForXfer(void)
{
	uint16_t u16BatteryVoltage = ddi_power_GetBattery();

	if (u16BatteryVoltage > DDI_POWER_BATTERY_XFER_THRESHOLD_MV)
		return true;
	else
		return false;
}

void ddi_power_EnableVbusDroopIrq(void)
{

	__raw_writel(BM_POWER_CTRL_VDD5V_DROOP_IRQ,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_DROOP,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);

}


void ddi_power_Enable4p2(uint16_t target_current_limit_ma)
{

	uint16_t u16BatteryVoltage;
	uint32_t temp_reg;

	ddi_power_Init4p2Params();
	/* disable 4p2 rail brownouts for now. (they
	 * should have already been off at this point) */
	__raw_writel(BM_POWER_CTRL_ENIRQ_DCDC4P2_BO,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	u16BatteryVoltage = ddi_power_GetBattery();

	if (ddi_power_IsBattRdyForXfer()) {

		/* PWD_CHARGE_4P2 should already be set but just in case... */
		__raw_writel(BM_POWER_5VCTRL_PWD_CHARGE_4P2,
				REGS_POWER_BASE + HW_POWER_5VCTRL_SET);

		/* set CMPTRIP to DCDC_4P2 pin >= BATTERY pin */
		temp_reg = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2);
		temp_reg &= ~(BM_POWER_DCDC4P2_CMPTRIP);
		temp_reg |= (31 << BP_POWER_DCDC4P2_CMPTRIP);
		__raw_writel(temp_reg, REGS_POWER_BASE + HW_POWER_DCDC4P2);

		/* since we have a good battery, we can go ahead
		 * and turn on the Dcdcing from the 4p2 source.
		 * This is helpful in working around the chip
		 * errata.
		 */
		ddi_power_Start4p2Dcdc(true);

		/* Enable VbusDroopIrq to handle errata */

		/* set vbus droop detection level to 4.3V */
		__raw_writel(BM_POWER_5VCTRL_VBUSDROOP_TRSH,
				REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

		ddi_power_EnableVbusDroopIrq();
		/* now that the DCDC4P2 problems are cleared,
		 * turn on and ramp up the 4p2 regulator
		 */
		temp_reg = ddi_power_BringUp4p2Regulator(
			target_current_limit_ma, true);

		/* if we still have our 5V connection, we can disable
		 * battery brownout interrupt.  This is because the
		 * VDD5V DROOP IRQ handler will also shutdown if battery
		 * is browned out and it will enable the battery brownout
		 * and bring VBUSVALID_TRSH level back to a normal level
		 * which caused the hardware battery brownout shutdown
		 * to be enabled.  The benefit of this is that device
		 * that have detachable batteries (or devices going through
		 * the assembly line and running this firmware to test
		 *  with) can avoid shutting down if 5V is present and
		 *  battery voltage goes away.
		 */
		if (!(__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
			(BM_POWER_CTRL_VBUSVALID_IRQ |
			BM_POWER_CTRL_VDD5V_DROOP_IRQ))) {
			ddi_power_EnableBatteryBoInterrupt(false);
		}



		printk(KERN_DEBUG "4P2 rail started.  5V current limit\
			set to %dmA\n",	temp_reg);

	} else {

		printk(KERN_ERR "4P2 rail was attempted to be started \
			from a system\
			with a very low battery voltage.  This is not\
			yet handled by the kernel driver, only by the\
			bootlets.  Remaining on battery power.\n");

		if ((__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) &&
					BM_POWER_5VCTRL_ENABLE_DCDC))
			ddi_power_EnableBatteryBoInterrupt(true);

#if 0
		/* enable hardware shutdown (if 5v disconnected)
		 * on battery brownout */
		__raw_writel(
			BM_POWER_BATTMONITOR_PWDN_BATTBRNOUT |
			__raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR),
			REGS_POWER_BASE + HW_POWER_BATTMONITOR);

		/* turn on and ramp up the 4p2 regulator */
		temp_reg = ddi_power_BringUp4p2Regulator(
			target_current_limit_ma, false);

		Configure4p2FiqShutdown();

		SetVbusValidThresh(0);
#endif
	}

}

/* enable and ramp up 4p2 regulator */
uint16_t ddi_power_BringUp4p2Regulator(
		uint16_t target_current_limit_ma,
		bool b4p2_dcdc_enabled)
{
	uint32_t temp_reg;
	uint16_t charge_4p2_ilimit = 0;

	/* initial current limit to 0 */
	__raw_writel(BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT,
		REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2) |
		BM_POWER_DCDC4P2_ENABLE_4P2,
		REGS_POWER_BASE + HW_POWER_DCDC4P2);

	/* set 4p2 target voltage to zero */
	temp_reg = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2);
	temp_reg &= (~BM_POWER_DCDC4P2_TRG);
	__raw_writel(temp_reg, REGS_POWER_BASE + HW_POWER_DCDC4P2);

	/* Enable 4P2 regulator*/
	__raw_writel(BM_POWER_5VCTRL_PWD_CHARGE_4P2,
			REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	if (target_current_limit_ma > 780)
		target_current_limit_ma = 780;

	ddi_power_Set4p2BoLevel(4150);

	/* possibly not necessary but recommended for unloaded
	 * 4p2 rail
	 */
	__raw_writel(BM_POWER_CHARGE_ENABLE_LOAD,
		REGS_POWER_BASE + HW_POWER_CHARGE_SET);

	while (charge_4p2_ilimit < target_current_limit_ma) {

		if (__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
			(BM_POWER_CTRL_VBUSVALID_IRQ |
			BM_POWER_CTRL_VDD5V_DROOP_IRQ))
			break;


		charge_4p2_ilimit += 100;
		if (charge_4p2_ilimit > target_current_limit_ma)
			charge_4p2_ilimit = target_current_limit_ma;

		ddi_power_set_4p2_ilimit(charge_4p2_ilimit);

		/* dcdc4p2 enable_dcdc must be enabled for
		 * 4p2 bo indication to function.  If not enabled,
		 * skip using bo level detection
		 */
		if (!(b4p2_dcdc_enabled))
			msleep(1);
		else if	(__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
			BM_POWER_STS_DCDC_4P2_BO)
			msleep(1);
		else {
			charge_4p2_ilimit = target_current_limit_ma;
			ddi_power_set_4p2_ilimit(charge_4p2_ilimit);
		}
	}

	ddi_power_Set4p2BoLevel(3600);

	__raw_writel(BM_POWER_CTRL_DCDC4P2_BO_IRQ,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	/* rail should now be up and loaded.  Extra
	 * internal load is not necessary.
	 */
	__raw_writel(BM_POWER_CHARGE_ENABLE_LOAD,
		REGS_POWER_BASE + HW_POWER_CHARGE_CLR);

	return charge_4p2_ilimit;

}


void ddi_power_Set4p2BoLevel(uint16_t bo_voltage_mv)
{
	uint16_t bo_reg_value;
	uint32_t temp;

	if (bo_voltage_mv < 3600)
		bo_voltage_mv = 3600;
	else if (bo_voltage_mv > 4375)
		bo_voltage_mv = 4375;

	bo_reg_value = (bo_voltage_mv - 3600) / 25;

	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2);
	temp &= (~BM_POWER_DCDC4P2_BO);
	temp |= (bo_reg_value << BP_POWER_DCDC4P2_BO);
	__raw_writel(temp, REGS_POWER_BASE + HW_POWER_DCDC4P2);
}



void ddi_power_init_handoff(void)
{
	int val;
	/* The following settings give optimal power supply capability */

	/* enable 5v presence detection */
	ddi_power_Enable5vDetection();

	if (ddi_power_Get5vPresentFlag())
		/* It's 5V mode, enable 5V-to-battery handoff */
		ddi_power_enable_5v_to_battery_handoff();
	else
		/* It's battery mode, enable battery-to-5V handoff */
		ddi_power_enable_battery_to_5v_handoff();

	/* Finally enable the battery adjust */
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR);
	val |= BM_POWER_BATTMONITOR_EN_BATADJ;
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_BATTMONITOR);
}


void ddi_power_EnableBatteryInterrupt(bool enable)
{

	__raw_writel(BM_POWER_CTRL_BATT_BO_IRQ,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	__raw_writel(BM_POWER_CTRL_ENIRQBATT_BO,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);

}


#define REGS_LRADC_BASE IO_ADDRESS(LRADC_PHYS_ADDR)

int ddi_power_init_battery(void)
{

	int ret = 0;

	if (!(__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) &&
			BM_POWER_5VCTRL_ENABLE_DCDC)) {
		printk(KERN_ERR "WARNING: Power Supply not\
			initialized correctly by \
			pre-kernel bootlets. HW_POWER_5VCTRL \
			ENABLE_DCDC should already be set.  Kernel \
			power driver behavior may not be reliable \n");
		ret = 1;
	}
	if ((__raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR) &
		BM_POWER_BATTMONITOR_BATT_VAL) == 0) {
		ret = 1;
		printk(KERN_INFO "WARNING : No battery connected !\r\n");
		return ret;
	}

	/* the following code to enable automatic battery measurement
	 * should have already been enabled in the boot prep files.  Not
	 * sure if this is necessary or possibly susceptible to
	 * mis-coordination
	 */


	ret = !hw_lradc_present(BATTERY_VOLTAGE_CH);

	if (ret) {
		printk(KERN_ERR "%s: hw_lradc_present failed\n", __func__);
		return -ENODEV;
	} else {
		uint16_t wait_time = 0;

		hw_lradc_configure_channel(BATTERY_VOLTAGE_CH, 0 /* div2 */ ,
					   0 /* acc */ ,
					   0 /* num_samples */);

		/* Setup the trigger loop forever */
		hw_lradc_set_delay_trigger(LRADC_DELAY_TRIGGER_BATTERY,
			1 << BATTERY_VOLTAGE_CH,
			1 << LRADC_DELAY_TRIGGER_BATTERY,
			0, 200);

		/* Clear the accumulator & NUM_SAMPLES */
		__raw_writel(0xFFFFFFFF,
			REGS_LRADC_BASE + HW_LRADC_CHn_CLR(BATTERY_VOLTAGE_CH));

		/* clear previous "measurement performed" status */
		__raw_writel(1 << BATTERY_VOLTAGE_CH,
			REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);

		/* set to LiIon scale factor */
		__raw_writel(BM_LRADC_CONVERSION_SCALE_FACTOR,
			REGS_LRADC_BASE + HW_LRADC_CONVERSION_SET);

		/* kick off the trigger */
		hw_lradc_set_delay_trigger_kick(
				LRADC_DELAY_TRIGGER_BATTERY, 1);


		/* wait for 1st converstion to be complete before
		 * enabling automatic copy to power supply
		 * peripheral
		 */
		while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1) &
			1 << BATTERY_VOLTAGE_CH) &&
				(wait_time < 10)) {
			wait_time++;
			mdelay(1);
		}

		__raw_writel(BM_LRADC_CONVERSION_AUTOMATIC,
			REGS_LRADC_BASE + HW_LRADC_CONVERSION_SET);
#ifdef CONFIG_ARCH_MX28
		/* workaround for mx28 lradc result incorrect in the
		first several ms */
		for (wait_time = 0; wait_time < 20; wait_time++)
			if (ddi_bc_hwGetBatteryVoltage() < 1000) {
				pr_info("ddi_bc_hwGetBatteryVoltage=%u\n",
				ddi_bc_hwGetBatteryVoltage());
				mdelay(100);
			} else
				break;
#endif
	}

#ifndef VDD4P2_ENABLED
	/* prepare handoff */
	ddi_power_init_handoff();
#endif
	return ret;
}

/*
 * Use the the lradc channel
 * get the die temperature from on-chip sensor.
 */
uint16_t MeasureInternalDieTemperature(void)
{
	uint32_t  ch8Value, ch9Value, lradc_irq_mask, channel;

	channel = g_ddi_bc_Configuration.u8DieTempChannel;
	lradc_irq_mask = 1 << channel;

	/* power up internal tep sensor block */
	__raw_writel(BM_LRADC_CTRL2_TEMPSENSE_PWD,
			REGS_LRADC_BASE + HW_LRADC_CTRL2_CLR);

	/* mux to the lradc 8th temp channel */
	__raw_writel((0xF << (4 * channel)),
			REGS_LRADC_BASE + HW_LRADC_CTRL4_CLR);
	__raw_writel((8 << (4 * channel)),
			REGS_LRADC_BASE + HW_LRADC_CTRL4_SET);

	/* Clear the interrupt flag */
	__raw_writel(lradc_irq_mask,
			REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);
	__raw_writel(BF_LRADC_CTRL0_SCHEDULE(1 << channel),
			REGS_LRADC_BASE + HW_LRADC_CTRL0_SET);

	/* Wait for conversion complete*/
	while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1)
			& lradc_irq_mask))
		cpu_relax();

	/* Clear the interrupt flag again */
	__raw_writel(lradc_irq_mask,
			REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);

	/* read temperature value and clr lradc */
	ch8Value = __raw_readl(REGS_LRADC_BASE +
			HW_LRADC_CHn(channel)) & BM_LRADC_CHn_VALUE;


	__raw_writel(BM_LRADC_CHn_VALUE,
			REGS_LRADC_BASE + HW_LRADC_CHn_CLR(channel));

	/* mux to the lradc 9th temp channel */
	__raw_writel((0xF << (4 * channel)),
			REGS_LRADC_BASE + HW_LRADC_CTRL4_CLR);
	__raw_writel((9 << (4 * channel)),
			REGS_LRADC_BASE + HW_LRADC_CTRL4_SET);

	/* Clear the interrupt flag */
	__raw_writel(lradc_irq_mask,
			REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);
	__raw_writel(BF_LRADC_CTRL0_SCHEDULE(1 << channel),
			REGS_LRADC_BASE + HW_LRADC_CTRL0_SET);
	/* Wait for conversion complete */
	while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1)
			& lradc_irq_mask))
		cpu_relax();

	/* Clear the interrupt flag */
	__raw_writel(lradc_irq_mask,
			REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);
	/* read temperature value */
	ch9Value = __raw_readl(
			REGS_LRADC_BASE + HW_LRADC_CHn(channel))
		  & BM_LRADC_CHn_VALUE;


	__raw_writel(BM_LRADC_CHn_VALUE,
			REGS_LRADC_BASE + HW_LRADC_CHn_CLR(channel));

	/* power down temp sensor block */
	__raw_writel(BM_LRADC_CTRL2_TEMPSENSE_PWD,
			REGS_LRADC_BASE + HW_LRADC_CTRL2_SET);


	return (uint16_t)((ch9Value-ch8Value)*GAIN_CORRECTION/4000);
}


/*
 * Use the the lradc channel to get the battery temperature.
 * A thermistor is used for external temperature sensing
 * which attached to LRADC0. This function returns the thermister
 * resistance value in ohm. Please check the specifiction of the
 * thermister to convert the resistance value to temperature.
 */

#define NUM_TEMP_READINGS_TO_AVG 3

uint16_t MeasureInternalBatteryTemperature(void)
{
	uint32_t  value, lradc_irq_mask, channel, sum = 0;
  uint16_t  out_value;
  int i;

	channel = g_ddi_bc_Configuration.u8BatteryTempChannel;
	lradc_irq_mask = 1 << channel;

  /* Enable the temperature sensor. */
	__raw_writel(BM_LRADC_CTRL2_TEMPSENSE_PWD,
			REGS_LRADC_BASE + HW_LRADC_CTRL2_CLR);


  /*Setup the temperature sensor for current measurement.
   100uA is used for thermistor  */

	__raw_writel(BF_LRADC_CTRL2_TEMP_ISRC0(BV_LRADC_CTRL2_TEMP_ISRC0__100),
			REGS_LRADC_BASE + HW_LRADC_CTRL2_SET);

	__raw_writel(BM_LRADC_CTRL2_TEMP_SENSOR_IENABLE0,
			REGS_LRADC_BASE + HW_LRADC_CTRL2_SET);

  /* Wait while the current ramps up.  */
  msleep(1);


  /* mux analog input lradc 0 for conversion on LRADC channel . */

  __raw_writel((0xF << (4 * channel)),
      REGS_LRADC_BASE + HW_LRADC_CTRL4_CLR);
  __raw_writel((0 << (4 * channel)),
      REGS_LRADC_BASE + HW_LRADC_CTRL4_SET);


	for (i = 0; i < NUM_TEMP_READINGS_TO_AVG; i++) {
		/* Clear the interrupt flag */
    __raw_writel(lradc_irq_mask,
			REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);
    __raw_writel(BF_LRADC_CTRL0_SCHEDULE(1 << channel),
			REGS_LRADC_BASE + HW_LRADC_CTRL0_SET);

    /* Wait for conversion complete*/
    while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1) & lradc_irq_mask))
				cpu_relax();

    /* Clear the interrupt flag again */
    __raw_writel(lradc_irq_mask, REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);

    /* read temperature value and clr lradc */
    value = __raw_readl(REGS_LRADC_BASE +
			HW_LRADC_CHn(channel)) & BM_LRADC_CHn_VALUE;

    __raw_writel(BM_LRADC_CHn_VALUE,
			REGS_LRADC_BASE + HW_LRADC_CHn_CLR(channel));

    sum += value;
  }

  /* Turn off the current to the temperature sensor to save power */

	__raw_writel(BF_LRADC_CTRL2_TEMP_ISRC0(BV_LRADC_CTRL2_TEMP_ISRC0__ZERO),
			REGS_LRADC_BASE + HW_LRADC_CTRL2_SET);

	__raw_writel(BM_LRADC_CTRL2_TEMP_SENSOR_IENABLE0,
			REGS_LRADC_BASE + HW_LRADC_CTRL2_CLR);


	/* power down temp sensor block */
	__raw_writel(BM_LRADC_CTRL2_TEMPSENSE_PWD,
			REGS_LRADC_BASE + HW_LRADC_CTRL2_SET);


  /* Take the voltage average.  */
  value = sum/NUM_TEMP_READINGS_TO_AVG;

  /* convert the voltage to thermister resistance value in 10ohm.  */
  /* ohm = (ADC value) * 1.85/(2^12) / current 100uA  */
  value = value * 18500/4096;

  out_value = value/10;

	return out_value;
}


/*  Name: ddi_power_GetBatteryMode */
/*  */
/* brief */

ddi_power_BatteryMode_t ddi_power_GetBatteryMode(void)
{
	return DDI_POWER_BATT_MODE_LIION;
}


/*  Name: ddi_power_GetBatteryChargerEnabled */
/*  */
/* brief */

bool ddi_power_GetBatteryChargerEnabled(void)
{
#if 0
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_STS) & BM_POWER_STS_BATT_CHRG_PRESENT) ? 1 : 0;
#endif
	return 1;
}


/*  */
/* brief Report if the charger hardware power is on. */
/*  */
/* fntype Function */
/*  */
/*  This function reports if the charger hardware power is on. */
/*  */
/* retval  Zero if the charger hardware is not powered. Non-zero otherwise. */
/*  */
/*  Note that the bit we're looking at is named PWD_BATTCHRG. The "PWD" */
/*  stands for "power down". Thus, when the bit is set, the battery charger */
/*  hardware is POWERED DOWN. */

bool ddi_power_GetChargerPowered(void)
{
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_PWD_BATTCHRG) ? 0 : 1;
}


/*  */
/* brief Turn the charging hardware on or off. */
/*  */
/* fntype Function */
/*  */
/*  This function turns the charging hardware on or off. */
/*  */
/* param[in]  on  Indicates whether the charging hardware should be on or off. */
/*  */
/*  Note that the bit we're looking at is named PWD_BATTCHRG. The "PWD" */
/*  stands for "power down". Thus, when the bit is set, the battery charger */
/*  hardware is POWERED DOWN. */

void ddi_power_SetChargerPowered(bool bPowerOn)
{
	/* Hit the battery charge power switch. */
	if (bPowerOn) {
		__raw_writel(BM_POWER_CHARGE_PWD_BATTCHRG,
			REGS_POWER_BASE + HW_POWER_CHARGE_CLR);
		__raw_writel(BM_POWER_5VCTRL_PWD_CHARGE_4P2,
			REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);
	} else {
		__raw_writel(BM_POWER_CHARGE_PWD_BATTCHRG,
			REGS_POWER_BASE + HW_POWER_CHARGE_SET);
#ifndef VDD4P2_ENABLED
		__raw_writel(BM_POWER_5VCTRL_PWD_CHARGE_4P2,
			REGS_POWER_BASE + HW_POWER_5VCTRL_SET);
#endif
	}

/* #ifdef CONFIG_POWER_SUPPLY_DEBUG */
#if 0
	printk("Battery charger: charger %s\n", bPowerOn ? "ON!" : "OFF");
	dump_regs();
#endif
}


/*  */
/* brief Reports if the charging current has fallen below the threshold. */
/*  */
/* fntype Function */
/*  */
/*  This function reports if the charging current that the battery is accepting */
/*  has fallen below the threshold. */
/*  */
/*  Note that this bit is regarded by the hardware guys as very slightly */
/*  unreliable. They recommend that you don't believe a value of zero until */
/*  you've sampled it twice. */
/*  */
/* retval  Zero if the battery is accepting less current than indicated by the */
/*           charging threshold. Non-zero otherwise. */
/*  */

int ddi_power_GetChargeStatus(void)
{
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_STS) & BM_POWER_STS_CHRGSTS) ? 1 : 0;
}


/* Battery Voltage */



/*  */
/* brief Report the voltage across the battery. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the voltage across the battery. Should return a */
/*  value in range ~3000 - 4200 mV. */
/*  */
/* retval The voltage across the battery, in mV. */
/*  */


/* brief Constant value for 8mV steps used in battery translation */
#define BATT_VOLTAGE_8_MV 8

uint16_t ddi_power_GetBattery(void)
{
	uint32_t    u16BattVolt;

	/* Get the raw result of battery measurement */
	u16BattVolt = __raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR);
	u16BattVolt &= BM_POWER_BATTMONITOR_BATT_VAL;
	u16BattVolt >>= BP_POWER_BATTMONITOR_BATT_VAL;

	/* Adjust for 8-mV LSB resolution and return */
	u16BattVolt *= BATT_VOLTAGE_8_MV;

/* #ifdef CONFIG_POWER_SUPPLY_DEBUG */
#if 0
	printk("Battery charger: %u mV\n", u16BattVolt);
#endif

	return u16BattVolt;
}

#if 0

/*  */
/* brief Report the voltage across the battery. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the voltage across the battery. */
/*  */
/* retval The voltage across the battery, in mV. */
/*  */

uint16_t ddi_power_GetBatteryBrownout(void)
{
	uint32_t    u16BatteryBrownoutLevel;

	/* Get battery brownout level */
	u16BatteryBrownoutLevel = __raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR);
	u16BatteryBrownoutLevel &= BM_POWER_BATTMONITOR_BRWNOUT_LVL;
	u16BatteryBrownoutLevel >>= BP_POWER_BATTMONITOR_BRWNOUT_LVL;

	/* Calculate battery brownout level */
	switch (ddi_power_GetBatteryMode()) {
	case DDI_POWER_BATT_MODE_LIION:
		u16BatteryBrownoutLevel *= BATT_BRWNOUT_LIION_LEVEL_STEP_MV;
		u16BatteryBrownoutLevel += BATT_BRWNOUT_LIION_BASE_MV;
		break;
	case DDI_POWER_BATT_MODE_ALKALINE_NIMH:
		u16BatteryBrownoutLevel *= BATT_BRWNOUT_ALKAL_LEVEL_STEP_MV;
		u16BatteryBrownoutLevel += BATT_BRWNOUT_ALKAL_BASE_MV;
		break;
	default:
		u16BatteryBrownoutLevel = 0;
		break;
	}
	return u16BatteryBrownoutLevel;
}


/*  */
/* brief Set battery brownout level */
/*  */
/* fntype     Reentrant Function */
/*  */
/*  This function sets the battery brownout level in millivolt. It transforms the */
/*  input brownout value from millivolts to the hardware register bit field value */
/*  taking the ceiling value in the calculation. */
/*  */
/* param[in]  u16BattBrownout_mV      Battery battery brownout level in mV */
/*  */
/* return     SUCCESS */
/*  */

int ddi_power_SetBatteryBrownout(uint16_t u16BattBrownout_mV)
{
	int16_t i16BrownoutLevel;
	int ret = 0;

	/* Calculate battery brownout level */
	switch (ddi_power_GetBatteryMode()) {
	case DDI_POWER_BATT_MODE_LIION:
		i16BrownoutLevel  = u16BattBrownout_mV -
			BATT_BRWNOUT_LIION_EQN_CONST;
		i16BrownoutLevel /= BATT_BRWNOUT_LIION_LEVEL_STEP_MV;
		break;
	case DDI_POWER_BATT_MODE_ALKALINE_NIMH:
		i16BrownoutLevel  = u16BattBrownout_mV -
			BATT_BRWNOUT_ALKAL_EQN_CONST;
		i16BrownoutLevel /= BATT_BRWNOUT_ALKAL_LEVEL_STEP_MV;
		break;
	default:
		return -EINVAL;
	}

	/* Do a check to make sure nothing went wrong. */
	if (i16BrownoutLevel <= 0x0f) {
		/* Write the battery brownout level */
		__raw_writel(
			BF_POWER_BATTMONITOR_BRWNOUT_LVL(i16BrownoutLevel),
				REGS_POWER_BASE + HW_POWER_BATTMONITOR_SET);
	} else
		ret = -EINVAL;

	return ret;
}
#endif


/* Currents */




/*  Name: ddi_power_SetMaxBatteryChargeCurrent */
/*  */
/* brief */

uint16_t ddi_power_SetMaxBatteryChargeCurrent(uint16_t u16MaxCur)
{
	uint32_t   u16OldSetting;
	uint32_t   u16NewSetting;
	uint32_t   u16ToggleMask;

	/* Get the old setting. */
	u16OldSetting = (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_BATTCHRG_I) >>
		BP_POWER_CHARGE_BATTCHRG_I;

	/* Convert the new threshold into a setting. */
	u16NewSetting = ddi_power_convert_current_to_setting(u16MaxCur);

	/* Compute the toggle mask. */
	u16ToggleMask = u16OldSetting ^ u16NewSetting;

	/* Write to the toggle register.*/
	__raw_writel(u16ToggleMask << BP_POWER_CHARGE_BATTCHRG_I,
			REGS_POWER_BASE + HW_POWER_CHARGE_TOG);

	/* Tell the caller what current we're set at now. */
	return ddi_power_convert_setting_to_current(u16NewSetting);
}


/*  Name: ddi_power_GetMaxBatteryChargeCurrent */
/*  */
/* brief */

uint16_t ddi_power_GetMaxBatteryChargeCurrent(void)
{
	uint32_t u8Bits;

	/* Get the raw data from register */
	u8Bits = (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_BATTCHRG_I) >>
		BP_POWER_CHARGE_BATTCHRG_I;

	/* Translate raw data to current (in mA) and return it */
	return ddi_power_convert_setting_to_current(u8Bits);
}


/*  Name: ddi_power_GetMaxChargeCurrent */
/*  */
/* brief */

uint16_t ddi_power_SetBatteryChargeCurrentThreshold(uint16_t u16Thresh)
{
	uint32_t   u16OldSetting;
	uint32_t   u16NewSetting;
	uint32_t   u16ToggleMask;

	/* ------------------------------------------------------------------- */
	/* See ddi_power_SetMaxBatteryChargeCurrent for an explanation of */
	/* why we're using the toggle register here. */
	/*  */
	/* Since this function doesn't have any major hardware effect, */
	/* we could use the usual macros for writing to this bit field. But, */
	/* for the sake of parallel construction and any potentially odd */
	/* effects on the status bit, we use the toggle register in the same */
	/* way as ddi_bc_hwSetMaxCurrent. */
	/* ------------------------------------------------------------------- */

	/* ------------------------------------------------------------------- */
	/* The threshold hardware can't express as large a range as the max */
	/* current setting, but we can use the same functions as long as we */
	/* add an extra check here. */
	/*  */
	/* Thresholds larger than 180mA can't be expressed. */
	/* ------------------------------------------------------------------- */

	if (u16Thresh > 180)
		u16Thresh = 180;


	/* Create the mask */


	/* Get the old setting. */
	u16OldSetting = (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_STOP_ILIMIT) >>
		BP_POWER_CHARGE_STOP_ILIMIT;

	/* Convert the new threshold into a setting. */
	u16NewSetting = ddi_power_convert_current_to_setting(u16Thresh);

	/* Compute the toggle mask. */
	u16ToggleMask = u16OldSetting ^ u16NewSetting;


	/* Write to the register */


	/* Write to the toggle register. */
	__raw_writel(BF_POWER_CHARGE_STOP_ILIMIT(u16ToggleMask),
			REGS_POWER_BASE + HW_POWER_CHARGE_TOG);

	/* Tell the caller what current we're set at now. */
	return ddi_power_convert_setting_to_current(u16NewSetting);
}


/*  Name: ddi_power_GetBatteryChargeCurrentThreshold */
/*  */
/* brief */

uint16_t ddi_power_GetBatteryChargeCurrentThreshold(void)
{
	uint32_t u16Threshold;

	u16Threshold = (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_STOP_ILIMIT) >>
		BP_POWER_CHARGE_STOP_ILIMIT;

	return ddi_power_convert_setting_to_current(u16Threshold);
}


/* Conversion */



/*  */
/* brief Compute the actual current expressible in the hardware. */
/*  */
/* fntype Function */
/*  */
/*  Given a desired current, this function computes the actual current */
/*  expressible in the hardware. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 780mA (see the data sheet for details). If the given */
/*  current cannot be expressed exactly, then the largest expressible smaller */
/*  value will be used. */
/*  */
/* param[in]  u16Current  The current of interest. */
/*  */
/* retval  The corresponding current in mA. */
/*  */

uint16_t ddi_power_ExpressibleCurrent(uint16_t u16Current)
{
	return ddi_power_convert_setting_to_current(
		ddi_power_convert_current_to_setting(u16Current));
}


/*  Name: ddi_power_Get5VPresent */
/*  */
/* brief */


bool ddi_power_Get5vPresentFlag(void)
{
	switch (DetectionMethod) {
	case DDI_POWER_5V_VBUSVALID:
		/* Check VBUSVALID for 5V present */
		return ((__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
			BM_POWER_STS_VBUSVALID) != 0);
	case DDI_POWER_5V_VDD5V_GT_VDDIO:
		/* Check VDD5V_GT_VDDIO for 5V present */
		return ((__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
			BM_POWER_STS_VDD5V_GT_VDDIO) != 0);
	default:
		break;
	}

	return 0;
}




/*  */
/* brief Report on the die temperature. */
/*  */
/* fntype Function */
/*  */
/*  This function reports on the die temperature. */
/*  */
/* param[out]  pLow   The low  end of the temperature range. */
/* param[out]  pHigh  The high end of the temperature range. */
/*  */

/* Temperature constant */
#define TEMP_READING_ERROR_MARGIN 5
#define KELVIN_TO_CELSIUS_CONST 273

void ddi_power_GetDieTemp(int16_t *pLow, int16_t *pHigh)
{
	int16_t i16High, i16Low;
	uint16_t u16Reading;

	/* Get the reading in Kelvins */
	u16Reading = MeasureInternalDieTemperature();

	/* Adjust for error margin */
	i16High = u16Reading + TEMP_READING_ERROR_MARGIN;
	i16Low  = u16Reading - TEMP_READING_ERROR_MARGIN;

	/* Convert to Celsius */
	i16High -= KELVIN_TO_CELSIUS_CONST;
	i16Low  -= KELVIN_TO_CELSIUS_CONST;

/* #ifdef CONFIG_POWER_SUPPLY_DEBUG */
#if 0
	printk("Battery charger: Die temp %d to %d C\n", i16Low, i16High);
#endif
	/* Return the results */
	*pHigh = i16High;
	*pLow  = i16Low;
}


void ddi_power_GetBatteryTemp(uint16_t *pReading)
{
	/* Get the reading */
	*pReading = MeasureInternalBatteryTemperature();
}

/*  */
/* brief Checks to see if the DCDC has been manually enabled */
/*  */
/* fntype Function */
/*  */
/* retval  true if DCDC is ON, false if DCDC is OFF. */
/*  */

bool ddi_power_IsDcdcOn(void)
{
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) & BM_POWER_5VCTRL_ENABLE_DCDC) ? 1 : 0;
}



/*  See hw_power.h for details. */

void ddi_power_SetPowerClkGate(bool bGate)
{
	/* Gate/Ungate the clock to the power block */
#ifndef CONFIG_ARCH_MX28
	if (bGate) {
		__raw_writel(BM_POWER_CTRL_CLKGATE,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
	} else {
		__raw_writel(BM_POWER_CTRL_CLKGATE,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	}
#endif
}


/*  See hw_power.h for details. */

bool ddi_power_GetPowerClkGate(void)
{
#ifdef CONFIG_ARCH_MX28
	return 0;
#else
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) & BM_POWER_CTRL_CLKGATE) ? 1 : 0;
#endif
}


enum ddi_power_5v_status ddi_power_GetPmu5vStatus(void)
{

	if (DetectionMethod == DDI_POWER_5V_VDD5V_GT_VDDIO) {

		if (__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
			BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO) {
			if ((__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
				BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ) ||
				ddi_power_Get5vPresentFlag())
				return new_5v_connection;
			else
				return existing_5v_disconnection;
		} else {
			if ((__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
				BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ) ||
				!ddi_power_Get5vPresentFlag() ||
				ddi_power_Get5vDroopFlag())
				return new_5v_disconnection;
			else
				return existing_5v_connection;
		}
	} else {

		if (__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
			BM_POWER_CTRL_POLARITY_VBUSVALID) {
			if ((__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
				BM_POWER_CTRL_VBUSVALID_IRQ) ||
				ddi_power_Get5vPresentFlag())
				return new_5v_connection;
			else
				return existing_5v_disconnection;
		} else {
			if ((__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
				BM_POWER_CTRL_VBUSVALID_IRQ) ||
				!ddi_power_Get5vPresentFlag() ||
				ddi_power_Get5vDroopFlag())
				return new_5v_disconnection;
			else
				return existing_5v_connection;
		}

	}
}

void ddi_power_disable_5v_connection_irq(void)
{

	__raw_writel((BM_POWER_CTRL_ENIRQ_VBUS_VALID |
		BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO),
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
}

void ddi_power_enable_5v_disconnect_detection(void)
{
	__raw_writel(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO |
		BM_POWER_CTRL_POLARITY_VBUSVALID,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	__raw_writel(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ |
		BM_POWER_CTRL_VBUSVALID_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	if (DetectionMethod == DDI_POWER_5V_VDD5V_GT_VDDIO) {
		__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
	} else {
		__raw_writel(BM_POWER_CTRL_ENIRQ_VBUS_VALID,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
	}
}

void ddi_power_enable_5v_connect_detection(void)
{
	__raw_writel(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO |
		BM_POWER_CTRL_POLARITY_VBUSVALID,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);

	__raw_writel(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ |
		BM_POWER_CTRL_VBUSVALID_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	if (DetectionMethod == DDI_POWER_5V_VDD5V_GT_VDDIO) {
		__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
	} else {
		__raw_writel(BM_POWER_CTRL_ENIRQ_VBUS_VALID,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
	}
}

void ddi_power_EnableBatteryBoInterrupt(bool bEnable)
{
	if (bEnable) {

		__raw_writel(BM_POWER_CTRL_BATT_BO_IRQ,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
		__raw_writel(BM_POWER_CTRL_ENIRQBATT_BO,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
		/* todo: make sure the battery brownout comparator
		 * is enabled in HW_POWER_BATTMONITOR
		 */
	} else {
		__raw_writel(BM_POWER_CTRL_ENIRQBATT_BO,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	}
}

void ddi_power_EnableDcdc4p2BoInterrupt(bool bEnable)
{
	if (bEnable) {

		__raw_writel(BM_POWER_CTRL_DCDC4P2_BO_IRQ,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
		__raw_writel(BM_POWER_CTRL_ENIRQ_DCDC4P2_BO,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
	} else {
		__raw_writel(BM_POWER_CTRL_ENIRQ_DCDC4P2_BO,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	}
}

void ddi_power_EnableVdd5vDroopInterrupt(bool bEnable)
{
	if (bEnable) {

		__raw_writel(BM_POWER_CTRL_VDD5V_DROOP_IRQ,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
		__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_DROOP,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
	} else {
		__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_DROOP,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	}
}


void ddi_power_Enable5vDisconnectShutdown(bool bEnable)
{
	if (bEnable) {
		__raw_writel(BM_POWER_5VCTRL_PWDN_5VBRNOUT,
			REGS_POWER_BASE + HW_POWER_5VCTRL_SET);
	} else {
		__raw_writel(BM_POWER_5VCTRL_PWDN_5VBRNOUT,
			REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);
	}
}


void ddi_power_enable_5v_to_battery_xfer(bool bEnable)
{
	if (bEnable) {
		/* order matters */

		/* we can enable this in in vbus droop or 4p2 fiq handler
		 * ddi_power_EnableBatteryBoInterrupt(true);
		 */
		ddi_power_Enable5vDisconnectShutdown(false);
	} else {
		/* order matters */
		ddi_power_Enable5vDisconnectShutdown(true);
		ddi_power_EnableBatteryBoInterrupt(false);
	}
}


void ddi_power_init_4p2_protection(void)
{
	/* set vbus droop detection level to 4.3V */
	__raw_writel(BM_POWER_5VCTRL_VBUSDROOP_TRSH,
			REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	/* VBUSDROOP THRESHOLD to 4.3V */
	__raw_writel(BM_POWER_5VCTRL_VBUSDROOP_TRSH,
		REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	ddi_power_EnableVbusDroopIrq();

#ifndef CONFIG_ARCH_MX28
	/* VBUSVALID THRESH = 2.9V */
	__raw_writel(BM_POWER_5VCTRL_VBUSVALID_TRSH,
		REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);
#endif

}

/* determine if all the bits are in a 'DCDC 4P2 Enabled' state. */
bool ddi_power_check_4p2_bits(void)
{


	uint32_t temp;

	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) &
		BM_POWER_5VCTRL_PWD_CHARGE_4P2;

	/* if PWD_CHARGE_4P2 = 1, 4p2 is disabled */
	if (temp)
		return false;

	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2) &
		BM_POWER_DCDC4P2_ENABLE_DCDC;

	if (!temp)
		return false;

	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2) &
			BM_POWER_DCDC4P2_ENABLE_4P2;

	if (temp)
		return true;
	else
		return false;

}

uint16_t ddi_power_set_4p2_ilimit(uint16_t ilimit)
{
	uint32_t temp_reg;

	if (ilimit > 780)
		ilimit = 780;
	temp_reg = __raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL);
	temp_reg &= (~BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT);
	temp_reg |= BF_POWER_5VCTRL_CHARGE_4P2_ILIMIT(
		ddi_power_convert_current_to_setting(
				ilimit));
	__raw_writel(temp_reg, REGS_POWER_BASE + HW_POWER_5VCTRL);

	return ilimit;
}

void ddi_power_shutdown(void)
{
	__raw_writel(0x3e770001, REGS_POWER_BASE + HW_POWER_RESET);
}

void ddi_power_handle_dcdc4p2_bo(void)
{
	ddi_power_EnableBatteryBoInterrupt(true);
	ddi_power_EnableDcdc4p2BoInterrupt(false);
}

void ddi_power_enable_vddio_interrupt(bool enable)
{
	if (enable) {
		__raw_writel(BM_POWER_CTRL_VDDIO_BO_IRQ,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
#ifndef DISABLE_VDDIO_BO_PROTECTION
		__raw_writel(BM_POWER_CTRL_ENIRQ_VDDIO_BO,
			REGS_POWER_BASE + HW_POWER_CTRL_SET);
#endif
	} else {
		__raw_writel(BM_POWER_CTRL_ENIRQ_VDDIO_BO,
			REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	}

}


void ddi_power_handle_vddio_brnout(void)
{
	if (ddi_power_GetPmu5vStatus() == new_5v_connection ||
		(ddi_power_GetPmu5vStatus() == new_5v_disconnection)) {
		ddi_power_enable_vddio_interrupt(false);
	} else {
#ifdef DEBUG_IRQS
		ddi_power_enable_vddio_interrupt(false);
		printk(KERN_ALERT "VDDIO BO TRIED TO SHUTDOWN!!!\n");
		return;
#else
		ddi_power_shutdown();
#endif
	}
}

void ddi_power_handle_vdd5v_droop(void)
{
	uint32_t temp;

	/* handle errata */
	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_DCDC4P2);
	temp |= (BF_POWER_DCDC4P2_CMPTRIP(31) | BM_POWER_DCDC4P2_TRG);
	__raw_writel(temp, REGS_POWER_BASE + HW_POWER_DCDC4P2);


	/* if battery is below brownout level, shutdown asap */
	if (__raw_readl(REGS_POWER_BASE + HW_POWER_STS) & BM_POWER_STS_BATT_BO)
		ddi_power_shutdown();

	/* due to 5v connect vddio bo chip bug, we need to
	 * disable vddio interrupts until we reset the 5v
	 * detection for 5v connect detect.  We want to allow
	 * some debounce time before enabling connect detection.
	 */
	ddi_power_enable_vddio_interrupt(false);

	ddi_power_EnableBatteryBoInterrupt(true);
	ddi_power_EnableDcdc4p2BoInterrupt(false);
	ddi_power_EnableVdd5vDroopInterrupt(false);

}

void ddi_power_InitOutputBrownouts(void)
{
	uint32_t temp;

	__raw_writel(BM_POWER_CTRL_VDDD_BO_IRQ |
		BM_POWER_CTRL_VDDA_BO_IRQ |
		BM_POWER_CTRL_VDDIO_BO_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

	__raw_writel(BM_POWER_CTRL_ENIRQ_VDDD_BO |
		BM_POWER_CTRL_ENIRQ_VDDA_BO |
		BM_POWER_CTRL_ENIRQ_VDDIO_BO,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);

	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	temp &= ~BM_POWER_VDDDCTRL_PWDN_BRNOUT;
	__raw_writel(temp, REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL);
	temp &= ~BM_POWER_VDDACTRL_PWDN_BRNOUT;
	__raw_writel(temp, REGS_POWER_BASE + HW_POWER_VDDACTRL);

	temp = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	temp &= ~BM_POWER_VDDIOCTRL_PWDN_BRNOUT;
	__raw_writel(temp, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
}

/* used for debugging purposes only */
void ddi_power_disable_power_interrupts(void)
{
	__raw_writel(BM_POWER_CTRL_ENIRQ_DCDC4P2_BO |
		BM_POWER_CTRL_ENIRQ_VDD5V_DROOP |
		BM_POWER_CTRL_ENIRQ_PSWITCH |
		BM_POWER_CTRL_ENIRQ_DC_OK |
		BM_POWER_CTRL_ENIRQBATT_BO |
		BM_POWER_CTRL_ENIRQ_VDDIO_BO |
		BM_POWER_CTRL_ENIRQ_VDDA_BO |
		BM_POWER_CTRL_ENIRQ_VDDD_BO |
		BM_POWER_CTRL_ENIRQ_VBUS_VALID |
		BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);

}

bool ddi_power_Get5vDroopFlag(void)
{
	if (__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
		BM_POWER_STS_VDD5V_DROOP)
		return true;
	else
		return false;
}


/* End of file */

/*  @} */
