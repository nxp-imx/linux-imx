/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

////////////////////////////////////////////////////////////////////////////////
//! \addtogroup ddi_power
//! @{
//
// Copyright(C) 2005 SigmaTel, Inc.
//
//! \file ddi_power_battery.c
//! \brief Implementation file for the power driver battery charger.
//!
////////////////////////////////////////////////////////////////////////////////
//   Includes and external references
////////////////////////////////////////////////////////////////////////////////
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <asm/processor.h> /* cpu_relax */
#include <mach/hardware.h>
#include <mach/ddi_bc.h>
#include <mach/lradc.h>
#include <mach/regs-power.h>
#include <mach/regs-lradc.h>
#include <mach/lradc.h>
#include "ddi_bc_internal.h"
#include <mach/platform.h>

//! \brief Base voltage to start battery calculations for LiIon
#define BATT_BRWNOUT_LIION_BASE_MV 2800
//! \brief Constant to help with determining whether to round up or
//! not during calculation
#define BATT_BRWNOUT_LIION_CEILING_OFFSET_MV 39
//! \brief Number of mV to add if rounding up in LiIon mode
#define BATT_BRWNOUT_LIION_LEVEL_STEP_MV 40
//! \brief Constant value to be calculated by preprocessing
#define BATT_BRWNOUT_LIION_EQN_CONST \
	(BATT_BRWNOUT_LIION_BASE_MV - BATT_BRWNOUT_LIION_CEILING_OFFSET_MV)
//! \brief Base voltage to start battery calculations for Alkaline/NiMH
#define BATT_BRWNOUT_ALKAL_BASE_MV 800
//! \brief Constant to help with determining whether to round up or
//! not during calculation
#define BATT_BRWNOUT_ALKAL_CEILING_OFFSET_MV 19
//! \brief Number of mV to add if rounding up in Alkaline/NiMH mode
#define BATT_BRWNOUT_ALKAL_LEVEL_STEP_MV 20
//! \brief Constant value to be calculated by preprocessing
#define BATT_BRWNOUT_ALKAL_EQN_CONST \
	(BATT_BRWNOUT_ALKAL_BASE_MV - BATT_BRWNOUT_ALKAL_CEILING_OFFSET_MV)

#define GAIN_CORRECTION 1012    // 1.012

/* NOTE: the below define is different for 37xx and 378x */
#define VBUSVALID_THRESH_4_30V		0x4
#define LINREG_OFFSET_STEP_BELOW	0x2
#define BP_POWER_BATTMONITOR_BATT_VAL	16
#define BP_POWER_CHARGE_BATTCHRG_I	0
#define BP_POWER_CHARGE_STOP_ILIMIT	8

////////////////////////////////////////////////////////////////////////////////
// Globals & Variables
////////////////////////////////////////////////////////////////////////////////

// FIXME
/* We cant use VBUSVALID signal for VDD5V detection, since setting in
 * USB driver POWER_DEBUG.VBUSVALIDPIOLOCK bit locks the POWER_STS.VBUSVALID to
 * active state for all power states (even if the 5v went away). The
 * POWER_CTRL.VBUSVALID_IRQ is also affected and it's impossible to get
 * valid information about 5v presence.
 */
/* static ddi_power_5vDetection_t DetectionMethod =
			DDI_POWER_5V_VDD5V_GT_VDDIO; */
static ddi_power_5vDetection_t DetectionMethod = DDI_POWER_5V_VBUSVALID;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if 0
static void dump_regs(void)
{
	printk("HW_POWER_CHARGE      0x%08x\n", __raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE));
	printk("HW_POWER_STS         0x%08x\n", __raw_readl(REGS_POWER_BASE + HW_POWER_STS));
	printk("HW_POWER_BATTMONITOR 0x%08x\n", __raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR));
}
#endif

//! This array maps bit numbers to current increments, as used in the register
//! fields HW_POWER_CHARGE.STOP_ILIMIT and HW_POWER_CHARGE.BATTCHRG_I.
static const uint16_t currentPerBit[] = {  10,  20,  50, 100, 200, 400 };

uint16_t ddi_power_convert_current_to_setting(uint16_t u16Current)
{
	int       i;
	uint16_t  u16Mask;
	uint16_t  u16Setting = 0;

	// Scan across the bit field, adding in current increments.
	u16Mask = (0x1 << 5);

	for (i = 5; (i >= 0) && (u16Current > 0); i--, u16Mask >>= 1) {
		if (u16Current >= currentPerBit[i]) {
			u16Current -= currentPerBit[i];
			u16Setting |= u16Mask;
		}
	}

	// Return the result.
	return(u16Setting);
}

////////////////////////////////////////////////////////////////////////////////
//! See hw_power.h for details.
////////////////////////////////////////////////////////////////////////////////
uint16_t ddi_power_convert_setting_to_current(uint16_t u16Setting)
{
	int       i;
	uint16_t  u16Mask;
	uint16_t  u16Current = 0;

	// Scan across the bit field, adding in current increments.
	u16Mask = (0x1 << 5);

	for (i = 5; i >= 0; i--, u16Mask >>= 1) {
		if (u16Setting & u16Mask) u16Current += currentPerBit[i];
	}

	// Return the result.
	return(u16Current);
}

void ddi_power_Enable5vDetection(void)
{
	u32 val;
	// Disable hardware power down when 5V is inserted or removed
	stmp3xxx_clearl(BM_POWER_5VCTRL_PWDN_5VBRNOUT, REGS_POWER_BASE + HW_POWER_5VCTRL);

	/*
	 * Prepare the hardware for the detection method.  We used to set
	 * and clear the VBUSVALID_5VDETECT bit, but that is also used for
	 * the DCDC 5V detection.  It is sufficient to just check the status
	 * bits to see if 5V is present.
	 *
	 * Use VBUSVALID for DCDC 5V detection.  The DCDC's detection is
	 * different than the USB/5V detection used to switch profiles.  This
	 * is used to determine when a handoff should occur.
	 */
	stmp3xxx_setl(BM_POWER_5VCTRL_VBUSVALID_5VDETECT, REGS_POWER_BASE + HW_POWER_5VCTRL);

	// Set 5V detection threshold to 4.3V for VBUSVALID.
	stmp3xxx_setl(
		BF(VBUSVALID_THRESH_4_30V, POWER_5VCTRL_VBUSVALID_TRSH), REGS_POWER_BASE + HW_POWER_5VCTRL);

	// gotta set LINREG_OFFSET to STEP_BELOW according to manual
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	val &= ~(BM_POWER_VDDIOCTRL_LINREG_OFFSET);
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDIOCTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);

	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL);
	val &= ~(BM_POWER_VDDACTRL_LINREG_OFFSET);
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDACTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDACTRL);

	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	val &= ~(BM_POWER_VDDDCTRL_LINREG_OFFSET);
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDDCTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	/* Clear vbusvalid interrupt flag */
//	stmp3xxx_clearl(BM_POWER_CTRL_VBUSVALID_IRQ, REGS_POWER_BASE + HW_POWER_CTRL);
	stmp3xxx_clearl(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ, REGS_POWER_BASE + HW_POWER_CTRL);
	/* enable vbusvalid irq */
//	stmp3xxx_setl(BM_POWER_CTRL_ENIRQ_VBUS_VALID, REGS_POWER_BASE + HW_POWER_CTRL);
	stmp3xxx_setl(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO, REGS_POWER_BASE + HW_POWER_CTRL);
}

/*
 * This function prepares the hardware for a 5V-to-battery handoff. It assumes
 * the current configuration is using 5V as the power source.  The 5V
 * interrupt will be set up for a 5V removal.
 */
void ddi_power_enable_5v_to_battery_handoff(void)
{
	/* Clear vbusvalid interrupt flag */
//	stmp3xxx_clearl(BM_POWER_CTRL_VBUSVALID_IRQ, REGS_POWER_BASE + HW_POWER_CTRL);
	stmp3xxx_clearl(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ, REGS_POWER_BASE + HW_POWER_CTRL);

	/* detect 5v unplug */
//	stmp3xxx_clearl(BM_POWER_CTRL_POLARITY_VBUSVALID, REGS_POWER_BASE + HW_POWER_CTRL);
	stmp3xxx_clearl(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO, REGS_POWER_BASE + HW_POWER_CTRL);

	// Enable automatic transition to DCDC
	stmp3xxx_setl(BM_POWER_5VCTRL_DCDC_XFER, REGS_POWER_BASE + HW_POWER_5VCTRL);
}

/*
 * This function will handle all the power rail transitions necesarry to power
 * the chip from the battery when it was previously powered from the 5V power
 * source.
 */
void ddi_power_execute_5v_to_battery_handoff(void)
{
	u32 val;
	// VDDD has different configurations depending on the battery type
	// and battery level.

	// For LiIon battery, we will use the DCDC to power VDDD.
	// Use LinReg offset for DCDC mode.
	// Turn on the VDDD DCDC output and turn off the VDDD LinReg output.
	// Make sure stepping is enabled when using DCDC.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	val &= ~(BM_POWER_VDDDCTRL_DISABLE_FET | BM_POWER_VDDDCTRL_ENABLE_LINREG |
			BM_POWER_VDDDCTRL_DISABLE_STEPPING | BM_POWER_VDDDCTRL_LINREG_OFFSET);
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDDCTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	// Power VDDA and VDDIO from the DCDC.
	// Use LinReg offset for DCDC mode.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL);
	val &= ~(BM_POWER_VDDACTRL_LINREG_OFFSET);
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDACTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDACTRL);

	// Turn on the VDDA DCDC converter output and turn off LinReg output.
	// Make sure stepping is enabled when using DCDC.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL);
	val &= ~(BM_POWER_VDDACTRL_DISABLE_FET |
			BM_POWER_VDDACTRL_ENABLE_LINREG |
			BM_POWER_VDDACTRL_DISABLE_STEPPING);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDACTRL);

	// Use LinReg offset for DCDC mode.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	val &= ~(BM_POWER_VDDIOCTRL_LINREG_OFFSET);
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDIOCTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	// Turn on the VDDIO DCDC output and turn on the LinReg output.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	val &= ~(BM_POWER_VDDIOCTRL_DISABLE_FET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);

	stmp3xxx_clearl(BM_POWER_5VCTRL_ILIMIT_EQ_ZERO, REGS_POWER_BASE + HW_POWER_5VCTRL);
	// Make sure stepping is enabled when using DCDC.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	val &= ~(BM_POWER_VDDIOCTRL_DISABLE_STEPPING);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
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
//	stmp3xxx_clearl(BM_POWER_CTRL_VBUSVALID_IRQ, REGS_POWER_BASE + HW_POWER_CTRL);
	stmp3xxx_clearl(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ, REGS_POWER_BASE + HW_POWER_CTRL);

	/* detect 5v plug-in */
//	stmp3xxx_setl(BM_POWER_CTRL_POLARITY_VBUSVALID, REGS_POWER_BASE + HW_POWER_CTRL);
	stmp3xxx_setl(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO, REGS_POWER_BASE + HW_POWER_CTRL);

	// Force current from 5V to be zero by disabling its entry source.
	stmp3xxx_setl(BM_POWER_5VCTRL_ILIMIT_EQ_ZERO, REGS_POWER_BASE + HW_POWER_5VCTRL);

	// Allow DCDC be to active when 5V is present.
	stmp3xxx_setl(BM_POWER_5VCTRL_ENABLE_DCDC, REGS_POWER_BASE + HW_POWER_5VCTRL);
}

/* This function handles the transitions on each of the power rails necessary
 * to power the chip from the 5V power supply when it was previously powered
 * from the battery power supply.
 */
void ddi_power_execute_battery_to_5v_handoff(void)
{
	u32 val;
	// Disable the DCDC during 5V connections.
	stmp3xxx_clearl(BM_POWER_5VCTRL_ENABLE_DCDC, REGS_POWER_BASE + HW_POWER_5VCTRL);

	// Power the VDDD/VDDA/VDDIO rail from the linear regulator.  The DCDC
	// is ready to automatically power the chip when 5V is removed.
	// Use this configuration when powering from 5V

	// Use LinReg offset for LinReg mode
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	val &= ~BM_POWER_VDDDCTRL_LINREG_OFFSET;
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDDCTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	// Turn on the VDDD LinReg and turn on the VDDD DCDC output.  The
	// ENABLE_DCDC must be cleared to avoid LinReg and DCDC conflict.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	val &= ~(BM_POWER_VDDDCTRL_ENABLE_LINREG | BM_POWER_VDDDCTRL_DISABLE_FET);
	val |= BM_POWER_VDDDCTRL_ENABLE_LINREG;
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	// Make sure stepping is disabled when using linear regulators
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	val &= ~(BM_POWER_VDDDCTRL_DISABLE_STEPPING);
	val |= BM_POWER_VDDDCTRL_DISABLE_STEPPING;
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	// Use LinReg offset for LinReg mode
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL);
	val &= ~(BM_POWER_VDDACTRL_LINREG_OFFSET);
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDACTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDACTRL);

	// Turn on the VDDA LinReg output and prepare the DCDC for transfer.
	// ENABLE_DCDC must be clear to avoid DCDC and LinReg conflict.
	// Make sure stepping is disabled when using linear regulators
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL);
	val &= ~(BM_POWER_VDDACTRL_ENABLE_LINREG | BM_POWER_VDDACTRL_DISABLE_FET |
			BM_POWER_VDDACTRL_DISABLE_STEPPING);
	val |= BM_POWER_VDDACTRL_ENABLE_LINREG | BM_POWER_VDDACTRL_DISABLE_STEPPING;
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDACTRL);

	// Use LinReg offset for LinReg mode.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	val &= ~(BM_POWER_VDDIOCTRL_LINREG_OFFSET);
	val |= BF(LINREG_OFFSET_STEP_BELOW, POWER_VDDIOCTRL_LINREG_OFFSET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);

	// Turn on the VDDIO LinReg output and prepare the VDDIO DCDC output.
        // ENABLE_DCDC must be cleared to prevent DCDC and LinReg conflict.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	val &= ~(BM_POWER_VDDIOCTRL_DISABLE_FET);
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);

	stmp3xxx_clearl(BM_POWER_5VCTRL_ILIMIT_EQ_ZERO, REGS_POWER_BASE + HW_POWER_5VCTRL);
	// Make sure stepping is disabled when using DCDC.
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	val |= BM_POWER_VDDIOCTRL_DISABLE_STEPPING;
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
}

void ddi_power_init_handoff(void)
{
	u32 val;
	/*
	 * The following settings give optimal power supply capability and
	 * efficiency.  Extreme loads will need HALF_FETS cleared and
	 * possibly DOUBLE_FETS set.  The below setting are probably also
	 * the best for alkaline mode also but more characterization is
	 * needed to know for sure.
	 */
	// Increase the RCSCALE_THRESHOLD
	stmp3xxx_setl(BM_POWER_LOOPCTRL_RCSCALE_THRESH,
			REGS_POWER_BASE + HW_POWER_LOOPCTRL);
	// Increase the RCSCALE level for quick DCDC response to dynamic load
	stmp3xxx_setl(BF(3, POWER_LOOPCTRL_EN_RCSCALE),
			REGS_POWER_BASE + HW_POWER_LOOPCTRL); // 8x

	// Enable half fets for increase efficiency.
	stmp3xxx_setl(BM_POWER_MINPWR_HALF_FETS, REGS_POWER_BASE + HW_POWER_MINPWR);

	// enable 5v presence detection
	ddi_power_Enable5vDetection();

	if (ddi_power_Get5vPresentFlag())
		/* It's 5V mode, enable 5V-to-battery handoff */
		ddi_power_enable_5v_to_battery_handoff();
	else
		/* It's battery mode, enable battery-to-5V handoff */
		ddi_power_enable_battery_to_5v_handoff();

	// Finally enable the battery adjust
	val = __raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR);
	val |= BM_POWER_BATTMONITOR_EN_BATADJ;
	__raw_writel(val, REGS_POWER_BASE + HW_POWER_BATTMONITOR);
}

int ddi_power_init_battery(void)
{
	int ret;

	// Init LRADC channel 7
	ret = hw_lradc_init_ladder(BATTERY_VOLTAGE_CH,
				   LRADC_DELAY_TRIGGER_BATTERY,
				   200);
	if (ret) {
		printk(KERN_ERR "%s: hw_lradc_init_ladder failed\n", __func__);
		return ret;
	}

	stmp3xxx_setl(BM_LRADC_CONVERSION_AUTOMATIC, REGS_LRADC_BASE + HW_LRADC_CONVERSION);

	// Set li-ion mode
	stmp3xxx_setl(BF(2, LRADC_CONVERSION_SCALE_FACTOR), REGS_LRADC_BASE + HW_LRADC_CONVERSION);

	// Turn off divide-by-two - we already have a divide-by-four
	// as part of the hardware
	stmp3xxx_clearl(
		BF(1 << BATTERY_VOLTAGE_CH, LRADC_CTRL2_DIVIDE_BY_TWO), REGS_LRADC_BASE + HW_LRADC_CTRL2);

	stmp3xxx_setl(BM_POWER_CHARGE_ENABLE_FAULT_DETECT, REGS_POWER_BASE + HW_POWER_CHARGE);

	// kick off the trigger
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_BATTERY, 1);

	/* prepare handoff */
	ddi_power_init_handoff();

	return 0;
}

/*
 * Use the the lradc7 channel dedicated for battery voltage measurement to
 * get the die temperature from on-chip sensor.
 */
uint16_t MeasureInternalDieTemperature(void)
{
	uint32_t  ch8Value, ch9Value;

	/* power up internal tep sensor block */
	stmp3xxx_clearl(BM_LRADC_CTRL2_TEMPSENSE_PWD, REGS_LRADC_BASE + HW_LRADC_CTRL2);

	/* mux to the lradc 8th temp channel */
	stmp3xxx_clearl(BF(0xF, LRADC_CTRL4_LRADC7SELECT), REGS_LRADC_BASE + HW_LRADC_CTRL4);
	stmp3xxx_setl(BF(8, LRADC_CTRL4_LRADC7SELECT), REGS_LRADC_BASE + HW_LRADC_CTRL4);

	/* Clear the interrupt flag */
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC7_IRQ, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	stmp3xxx_setl(BF(1 << BATTERY_VOLTAGE_CH, LRADC_CTRL0_SCHEDULE), REGS_LRADC_BASE + HW_LRADC_CTRL0);
	// Wait for conversion complete
        while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1) & BM_LRADC_CTRL1_LRADC7_IRQ))
                cpu_relax();
	/* Clear the interrupt flag again */
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC7_IRQ, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	// read temperature value and clr lradc
	ch8Value = __raw_readl(REGS_LRADC_BASE + HW_LRADC_CHn(BATTERY_VOLTAGE_CH)) & BM_LRADC_CHn_VALUE;
	stmp3xxx_clearl(BM_LRADC_CHn_VALUE, REGS_LRADC_BASE + HW_LRADC_CHn(BATTERY_VOLTAGE_CH));

	/* mux to the lradc 9th temp channel */
	stmp3xxx_clearl(BF(0xF, LRADC_CTRL4_LRADC7SELECT), REGS_LRADC_BASE + HW_LRADC_CTRL4);
	stmp3xxx_setl(BF(9, LRADC_CTRL4_LRADC7SELECT), REGS_LRADC_BASE + HW_LRADC_CTRL4);

	/* Clear the interrupt flag */
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC7_IRQ, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	stmp3xxx_setl(BF(1 << BATTERY_VOLTAGE_CH, LRADC_CTRL0_SCHEDULE), REGS_LRADC_BASE + HW_LRADC_CTRL0);
	// Wait for conversion complete
        while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1) & BM_LRADC_CTRL1_LRADC7_IRQ))
                cpu_relax();
	/* Clear the interrupt flag */
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC7_IRQ, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	// read temperature value
	ch9Value = __raw_readl(REGS_LRADC_BASE + HW_LRADC_CHn(BATTERY_VOLTAGE_CH)) & BM_LRADC_CHn_VALUE;
	stmp3xxx_clearl(BM_LRADC_CHn_VALUE, REGS_LRADC_BASE + HW_LRADC_CHn(BATTERY_VOLTAGE_CH));

	/* power down temp sensor block */
	stmp3xxx_setl(BM_LRADC_CTRL2_TEMPSENSE_PWD, REGS_LRADC_BASE + HW_LRADC_CTRL2);

	/* mux back to the lradc 7th battery voltage channel */
	stmp3xxx_clearl(BF(0xF, LRADC_CTRL4_LRADC7SELECT), REGS_LRADC_BASE + HW_LRADC_CTRL4);
	stmp3xxx_setl(BF(7, LRADC_CTRL4_LRADC7SELECT), REGS_LRADC_BASE + HW_LRADC_CTRL4);

	/* Clear the interrupt flag */
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC7_IRQ, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	stmp3xxx_setl(BF(1 << BATTERY_VOLTAGE_CH, LRADC_CTRL0_SCHEDULE), REGS_LRADC_BASE + HW_LRADC_CTRL0);
	// Wait for conversion complete
        while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1) & BM_LRADC_CTRL1_LRADC7_IRQ))
                cpu_relax();
	/* Clear the interrupt flag */
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC7_IRQ, REGS_LRADC_BASE + HW_LRADC_CTRL1);

	return (uint16_t)((ch9Value-ch8Value)*GAIN_CORRECTION/4000);
}


////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_GetBatteryMode
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
ddi_power_BatteryMode_t ddi_power_GetBatteryMode(void)
{
#if 0
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_STS) & BM_POWER_STS_MODE) ?
		DDI_POWER_BATT_MODE_ALKALINE_NIMH :
		DDI_POWER_BATT_MODE_LIION;
#endif
	return DDI_POWER_BATT_MODE_LIION;
}

////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_GetBatteryChargerEnabled
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
bool ddi_power_GetBatteryChargerEnabled(void)
{
#if 0
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_STS) & BM_POWER_STS_BATT_CHRG_PRESENT) ? 1 : 0;
#endif
	return 1;
}

////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Report if the charger hardware power is on.
//!
//! \fntype Function
//!
//! This function reports if the charger hardware power is on.
//!
//! \retval  Zero if the charger hardware is not powered. Non-zero otherwise.
//!
//! Note that the bit we're looking at is named PWD_BATTCHRG. The "PWD"
//! stands for "power down". Thus, when the bit is set, the battery charger
//! hardware is POWERED DOWN.
////////////////////////////////////////////////////////////////////////////////
bool ddi_power_GetChargerPowered(void)
{
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_PWD_BATTCHRG) ? 0 : 1;
}

////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Turn the charging hardware on or off.
//!
//! \fntype Function
//!
//! This function turns the charging hardware on or off.
//!
//! \param[in]  on  Indicates whether the charging hardware should be on or off.
//!
//! Note that the bit we're looking at is named PWD_BATTCHRG. The "PWD"
//! stands for "power down". Thus, when the bit is set, the battery charger
//! hardware is POWERED DOWN.
////////////////////////////////////////////////////////////////////////////////
void ddi_power_SetChargerPowered(bool bPowerOn)
{
	// Hit the battery charge power switch.
	if (bPowerOn) {
		stmp3xxx_clearl(BM_POWER_CHARGE_PWD_BATTCHRG, REGS_POWER_BASE + HW_POWER_CHARGE);
		stmp3xxx_clearl(BM_POWER_5VCTRL_PWD_CHARGE_4P2, REGS_POWER_BASE + HW_POWER_5VCTRL);
	} else {
		stmp3xxx_setl(BM_POWER_CHARGE_PWD_BATTCHRG, REGS_POWER_BASE + HW_POWER_CHARGE);
		stmp3xxx_setl(BM_POWER_5VCTRL_PWD_CHARGE_4P2, REGS_POWER_BASE + HW_POWER_5VCTRL);
	}

//#ifdef CONFIG_POWER_SUPPLY_DEBUG
#if 0
	printk("Battery charger: charger %s\n", bPowerOn ? "ON!" : "OFF");
	dump_regs();
#endif
}

////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Reports if the charging current has fallen below the threshold.
//!
//! \fntype Function
//!
//! This function reports if the charging current that the battery is accepting
//! has fallen below the threshold.
//!
//! Note that this bit is regarded by the hardware guys as very slightly
//! unreliable. They recommend that you don't believe a value of zero until
//! you've sampled it twice.
//!
//! \retval  Zero if the battery is accepting less current than indicated by the
//!          charging threshold. Non-zero otherwise.
//!
////////////////////////////////////////////////////////////////////////////////
int ddi_power_GetChargeStatus(void)
{
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_STS) & BM_POWER_STS_CHRGSTS) ? 1 : 0;
}

////////////////////////////////////////////////////////////////////////////////
// Battery Voltage
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Report the voltage across the battery.
//!
//! \fntype Function
//!
//! This function reports the voltage across the battery. Should return a
//! value in range ~3000 - 4200 mV.
//!
//! \retval The voltage across the battery, in mV.
//!
////////////////////////////////////////////////////////////////////////////////

//! \brief Constant value for 8mV steps used in battery translation
#define BATT_VOLTAGE_8_MV 8

uint16_t ddi_power_GetBattery(void)
{
	uint32_t    u16BattVolt;

	// Get the raw result of battery measurement
	u16BattVolt = __raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR);
	u16BattVolt &= BM_POWER_BATTMONITOR_BATT_VAL;
	u16BattVolt >>= BP_POWER_BATTMONITOR_BATT_VAL;

	// Adjust for 8-mV LSB resolution and return
	u16BattVolt *= BATT_VOLTAGE_8_MV;

//#ifdef CONFIG_POWER_SUPPLY_DEBUG
#if 0
	printk("Battery charger: %u mV\n", u16BattVolt);
#endif

	return u16BattVolt;
}

#if 0
////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Report the voltage across the battery.
//!
//! \fntype Function
//!
//! This function reports the voltage across the battery.
//!
//! \retval The voltage across the battery, in mV.
//!
////////////////////////////////////////////////////////////////////////////////
uint16_t ddi_power_GetBatteryBrownout(void)
{
	uint32_t    u16BatteryBrownoutLevel;

	// Get battery brownout level
	u16BatteryBrownoutLevel = __raw_readl(REGS_POWER_BASE + HW_POWER_BATTMONITOR);
	u16BatteryBrownoutLevel &= BM_POWER_BATTMONITOR_BRWNOUT_LVL;
	u16BatteryBrownoutLevel >>= BP_POWER_BATTMONITOR_BRWNOUT_LVL;

	// Calculate battery brownout level
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

////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Set battery brownout level
//!
//! \fntype     Reentrant Function
//!
//! This function sets the battery brownout level in millivolt. It transforms the
//! input brownout value from millivolts to the hardware register bit field value
//! taking the ceiling value in the calculation.
//!
//! \param[in]  u16BattBrownout_mV      Battery battery brownout level in mV
//!
//! \return     SUCCESS
//!
////////////////////////////////////////////////////////////////////////////////
int ddi_power_SetBatteryBrownout(uint16_t u16BattBrownout_mV)
{
	int16_t i16BrownoutLevel;
	int ret = 0;

	// Calculate battery brownout level
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

	// Do a check to make sure nothing went wrong.
	if (i16BrownoutLevel <= 0x0f) {
		//Write the battery brownout level
		stmp3xxx_setl(
			BF(i16BrownoutLevel, POWER_BATTMONITOR_BRWNOUT_LVL), REGS_POWER_BASE + HW_POWER_BATTMONITOR);
	} else
		ret = -EINVAL;

	return ret;
}
#endif

////////////////////////////////////////////////////////////////////////////////
// Currents
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_SetBiasCurrentSource
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
int ddi_power_SetBiasCurrentSource(ddi_power_BiasCurrentSource_t eSource)
{
	switch (eSource) {
	case DDI_POWER_INTERNAL_BIAS_CURRENT:
		stmp3xxx_setl(BM_POWER_CHARGE_USE_EXTERN_R, REGS_POWER_BASE + HW_POWER_CHARGE);
		break;
	case DDI_POWER_EXTERNAL_BIAS_CURRENT:
		stmp3xxx_clearl(BM_POWER_CHARGE_USE_EXTERN_R, REGS_POWER_BASE + HW_POWER_CHARGE);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_GetBiasCurrentSource
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
ddi_power_BiasCurrentSource_t ddi_power_GetBiasCurrentSource(void)
{
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_USE_EXTERN_R) ?
		DDI_POWER_INTERNAL_BIAS_CURRENT :
		DDI_POWER_EXTERNAL_BIAS_CURRENT;
}

////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_SetMaxBatteryChargeCurrent
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
uint16_t ddi_power_SetMaxBatteryChargeCurrent(uint16_t u16MaxCur)
{
	uint32_t   u16OldSetting;
	uint32_t   u16NewSetting;
	uint32_t   u16ToggleMask;

	// Get the old setting.
	u16OldSetting = (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_BATTCHRG_I) >>
		BP_POWER_CHARGE_BATTCHRG_I;

	// Convert the new threshold into a setting.
	u16NewSetting = ddi_power_convert_current_to_setting(u16MaxCur);

	// Write to the toggle register.
	u16ToggleMask = __raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE);
	u16ToggleMask &= ~BM_POWER_CHARGE_BATTCHRG_I;
	u16ToggleMask |= u16NewSetting << BP_POWER_CHARGE_BATTCHRG_I;

	__raw_writel(u16ToggleMask, REGS_POWER_BASE + HW_POWER_CHARGE);

	// Tell the caller what current we're set at now.
	return ddi_power_convert_setting_to_current(u16NewSetting);
}

////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_GetMaxBatteryChargeCurrent
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
uint16_t ddi_power_GetMaxBatteryChargeCurrent(void)
{
	uint32_t u8Bits;

	// Get the raw data from register
	u8Bits = (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_BATTCHRG_I) >>
		BP_POWER_CHARGE_BATTCHRG_I;

	// Translate raw data to current (in mA) and return it
	return ddi_power_convert_setting_to_current(u8Bits);
}

////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_GetMaxChargeCurrent
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
uint16_t ddi_power_SetBatteryChargeCurrentThreshold(uint16_t u16Thresh)
{
	uint32_t   u16OldSetting;
	uint32_t   u16NewSetting;
	uint32_t   u16ToggleMask;

	//-------------------------------------------------------------------
	// See ddi_power_SetMaxBatteryChargeCurrent for an explanation of
	// why we're using the toggle register here.
	//
	// Since this function doesn't have any major hardware effect,
	// we could use the usual macros for writing to this bit field. But,
	// for the sake of parallel construction and any potentially odd
	// effects on the status bit, we use the toggle register in the same
	// way as ddi_bc_hwSetMaxCurrent.
	//-------------------------------------------------------------------

	//-------------------------------------------------------------------
	// The threshold hardware can't express as large a range as the max
	// current setting, but we can use the same functions as long as we
	// add an extra check here.
	//
	// Thresholds larger than 180mA can't be expressed.
	//-------------------------------------------------------------------

	if (u16Thresh > 180)
		u16Thresh = 180;

	////////////////////////////////////////
	// Create the mask
	////////////////////////////////////////

	// Get the old setting.
	u16OldSetting = (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_STOP_ILIMIT) >>
		BP_POWER_CHARGE_STOP_ILIMIT;

	// Convert the new threshold into a setting.
	u16NewSetting = ddi_power_convert_current_to_setting(u16Thresh);

	// Compute the toggle mask.
	u16ToggleMask = u16OldSetting ^ u16NewSetting;

	/////////////////////////////////////////
	// Write to the register
	/////////////////////////////////////////

	// Write to the toggle register.
	u16ToggleMask = __raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE);
	u16ToggleMask &= ~BM_POWER_CHARGE_STOP_ILIMIT;
	u16ToggleMask |= u16NewSetting << BP_POWER_CHARGE_STOP_ILIMIT;

	// Tell the caller what current we're set at now.
	return ddi_power_convert_setting_to_current(u16NewSetting);
}

////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_GetBatteryChargeCurrentThreshold
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
uint16_t ddi_power_GetBatteryChargeCurrentThreshold(void)
{
	uint32_t u16Threshold;

	u16Threshold = (__raw_readl(REGS_POWER_BASE + HW_POWER_CHARGE) & BM_POWER_CHARGE_STOP_ILIMIT) >>
		BP_POWER_CHARGE_STOP_ILIMIT;

	return ddi_power_convert_setting_to_current(u16Threshold);
}

////////////////////////////////////////////////////////////////////////////////
// Conversion
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Compute the actual current expressible in the hardware.
//!
//! \fntype Function
//!
//! Given a desired current, this function computes the actual current
//! expressible in the hardware.
//!
//! Note that the hardware has a minimum resolution of 10mA and a maximum
//! expressible value of 780mA (see the data sheet for details). If the given
//! current cannot be expressed exactly, then the largest expressible smaller
//! value will be used.
//!
//! \param[in]  u16Current  The current of interest.
//!
//! \retval  The corresponding current in mA.
//!
////////////////////////////////////////////////////////////////////////////////
uint16_t ddi_power_ExpressibleCurrent(uint16_t u16Current)
{
	return ddi_power_convert_setting_to_current(
		ddi_power_convert_current_to_setting(u16Current));
}

////////////////////////////////////////////////////////////////////////////////
//! Name: ddi_power_Get5VPresent
//!
//! \brief
////////////////////////////////////////////////////////////////////////////////
bool ddi_power_Get5vPresentFlag(void)
{
	switch (DetectionMethod) {
	case DDI_POWER_5V_VBUSVALID:
		// Check VBUSVALID for 5V present
		return ((__raw_readl(REGS_POWER_BASE + HW_POWER_STS) & BM_POWER_STS_VBUSVALID) != 0);
	case DDI_POWER_5V_VDD5V_GT_VDDIO:
		// Check VDD5V_GT_VDDIO for 5V present
		return ((__raw_readl(REGS_POWER_BASE + HW_POWER_STS) & BM_POWER_STS_VDD5V_GT_VDDIO) != 0);
	default:
		break;
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Report on the die temperature.
//!
//! \fntype Function
//!
//! This function reports on the die temperature.
//!
//! \param[out]  pLow   The low  end of the temperature range.
//! \param[out]  pHigh  The high end of the temperature range.
//!
////////////////////////////////////////////////////////////////////////////////
// Temperature constant
#define TEMP_READING_ERROR_MARGIN 5
#define KELVIN_TO_CELSIUS_CONST 273

void ddi_power_GetDieTemp(int16_t * pLow, int16_t * pHigh)
{
	int16_t i16High, i16Low;
	uint16_t u16Reading;

	// Get the reading in Kelvins
	u16Reading = MeasureInternalDieTemperature();

	// Adjust for error margin
	i16High = u16Reading + TEMP_READING_ERROR_MARGIN;
	i16Low  = u16Reading - TEMP_READING_ERROR_MARGIN;

	// Convert to Celsius
	i16High -= KELVIN_TO_CELSIUS_CONST;
	i16Low  -= KELVIN_TO_CELSIUS_CONST;

//#ifdef CONFIG_POWER_SUPPLY_DEBUG
#if 0
	printk("Battery charger: Die temp %d to %d C\n", i16Low, i16High);
#endif
	// Return the results
	*pHigh = i16High;
	*pLow  = i16Low;
}

///////////////////////////////////////////////////////////////////////////////
//!
//! \brief Checks to see if the DCDC has been manually enabled
//!
//! \fntype Function
//!
//! \retval  true if DCDC is ON, false if DCDC is OFF.
//!
////////////////////////////////////////////////////////////////////////////////
bool ddi_power_IsDcdcOn(void)
{
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) & BM_POWER_5VCTRL_ENABLE_DCDC) ? 1 : 0;
}


////////////////////////////////////////////////////////////////////////////////
//! See hw_power.h for details.
////////////////////////////////////////////////////////////////////////////////
void ddi_power_SetPowerClkGate(bool bGate)
{
	// Gate/Ungate the clock to the power block
	if (bGate) {
		stmp3xxx_setl(BM_POWER_CTRL_CLKGATE, REGS_POWER_BASE + HW_POWER_CTRL);
	} else {
		stmp3xxx_clearl(BM_POWER_CTRL_CLKGATE, REGS_POWER_BASE + HW_POWER_CTRL);
	}
}

////////////////////////////////////////////////////////////////////////////////
//! See hw_power.h for details.
////////////////////////////////////////////////////////////////////////////////
bool ddi_power_GetPowerClkGate(void)
{
	return (__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) & BM_POWER_CTRL_CLKGATE) ? 1 : 0;
}


////////////////////////////////////////////////////////////////////////////////
// End of file
////////////////////////////////////////////////////////////////////////////////
//! @}
