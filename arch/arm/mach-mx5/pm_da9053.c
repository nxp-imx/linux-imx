/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/mfd/da9052/reg.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/i2c.h>
#include "pmic.h"

/** Defines ********************************************************************
*******************************************************************************/
/* have to hard-code the preset voltage here for they share the register
as the normal setting on Da9053 */
/* preset buck core to 850 mv */
#define BUCKCORE_SUSPEND_PRESET 0xCE
/* preset buck core to 950 mv */
#define BUCKPRO_SUSPEND_PRESET 0xD2
/* preset ldo6 to 1200 mv */
#define LDO6_SUSPEND_PRESET 0xC0
/* preset ldo10 to 1200 mv */
#define iLDO10_SUSPEND_PRESET 0xC0
/* set VUSB 2V5 active during suspend */
#define BUCKPERI_SUSPEND_SW_STEP 0x50
/* restore VUSB 2V5 active after suspend */
#define BUCKPERI_RESTORE_SW_STEP 0x55
/* restore VUSB 2V5 power supply after suspend */
#define SUPPLY_RESTORE_VPERISW_EN 0x20
#define CONF_BIT 0x80

#define DA9053_SLEEP_DELAY 0x1f
#define DA9052_CONTROLC_SMD_SET 0x62
#define DA9052_GPIO0809_SMD_SET 0x18
#define DA9052_ID1415_SMD_SET   0x1
#define DA9052_GPI9_IRQ_MASK    0x2
#define DA9052_ALARM_IRQ_EN     (0x1<<6)
#define DA9052_SEQ_RDY_IRQ_MASK (0x1<<6)

static u8 volt_settings[DA9052_LDO10_REG - DA9052_BUCKCORE_REG + 1];

static void pm_da9053_read_reg(u8 reg, u8 *value)
{
	unsigned char buf[2] = {0, 0};
	struct i2c_msg i2cmsg[2];
	buf[0] = reg;
	i2cmsg[0].addr  = 0x48 ;
	i2cmsg[0].len   = 1;
	i2cmsg[0].buf   = &buf[0];

	i2cmsg[0].flags = 0;

	i2cmsg[1].addr  = 0x48 ;
	i2cmsg[1].len   = 1;
	i2cmsg[1].buf   = &buf[1];

	i2cmsg[1].flags = I2C_M_RD;

	pm_i2c_imx_xfer(i2cmsg, 2);
	*value = buf[1];
}

static void pm_da9053_write_reg(u8 reg, u8 value)
{
	unsigned char buf[2] = {0, 0};
	struct i2c_msg i2cmsg[2];
	buf[0] = reg;
	buf[1] = value;
	i2cmsg[0].addr  = 0x48 ;
	i2cmsg[0].len   = 2;
	i2cmsg[0].buf   = &buf[0];
	i2cmsg[0].flags = 0;
	pm_i2c_imx_xfer(i2cmsg, 1);
}

static void pm_da9053_preset_voltage(void)
{
	u8 reg, data;
	for (reg = DA9052_BUCKCORE_REG;
		reg <= DA9052_LDO10_REG; reg++) {
		pm_da9053_read_reg(reg, &data);
		volt_settings[reg - DA9052_BUCKCORE_REG] = data;
		data |= CONF_BIT;
		pm_da9053_write_reg(reg, data);
	}
	pm_da9053_write_reg(DA9052_BUCKCORE_REG, BUCKCORE_SUSPEND_PRESET);
	pm_da9053_write_reg(DA9052_BUCKPRO_REG, BUCKPRO_SUSPEND_PRESET);
	pm_da9053_write_reg(DA9052_LDO6_REG, LDO6_SUSPEND_PRESET);
	pm_da9053_write_reg(DA9052_LDO10_REG, iLDO10_SUSPEND_PRESET);
	pm_da9053_write_reg(DA9052_ID1213_REG, BUCKPERI_SUSPEND_SW_STEP);
}

#if 0
static void pm_da9053_dump(int start, int end)
{
	u8 reg, data;
	for (reg = start; reg <= end; reg++) {
		pm_da9053_read_reg(reg, &data);
		pr_info("reg %u = 0x%2x\n",
			reg, data);
	}
}
#endif

int da9053_suspend_cmd_sw(void)
{
	unsigned char buf[2] = {0, 0};
	struct clk *i2c_clk;
	u8 data;
	buf[0] = 29;

	i2c_clk = clk_get(NULL, "i2c_clk");
	if (IS_ERR(i2c_clk)) {
		pr_err("unable to get i2c clk\n");
		return PTR_ERR(i2c_clk);
	}
	clk_enable(i2c_clk);

	pm_da9053_preset_voltage();

	pm_da9053_read_reg(DA9052_ID01_REG, &data);
	data &= ~(DA9052_ID01_DEFSUPPLY | DA9052_ID01_nRESMODE);
	pm_da9053_write_reg(DA9052_ID01_REG, data);

	pm_da9053_write_reg(DA9052_SEQB_REG, DA9053_SLEEP_DELAY);

	pm_da9053_read_reg(DA9052_CONTROLB_REG, &data);
	data |= DA9052_CONTROLB_DEEPSLEEP;
	pm_da9053_write_reg(DA9052_CONTROLB_REG, data);

	clk_disable(i2c_clk);
	clk_put(i2c_clk);
	return 0;
}

int da9053_suspend_cmd_hw(void)
{
	unsigned char buf[2] = {0, 0};
	struct clk *i2c_clk;
	u8 data;
	buf[0] = 29;

	i2c_clk = clk_get(NULL, "i2c_clk");
	if (IS_ERR(i2c_clk)) {
		pr_err("unable to get i2c clk\n");
		return PTR_ERR(i2c_clk);
	}
	clk_enable(i2c_clk);

	pm_da9053_preset_voltage();
	pm_da9053_write_reg(DA9052_CONTROLC_REG,
				DA9052_CONTROLC_SMD_SET);

	pm_da9053_read_reg(DA9052_ID01_REG, &data);
	data &= ~(DA9052_ID01_DEFSUPPLY | DA9052_ID01_nRESMODE);
	pm_da9053_write_reg(DA9052_ID01_REG, data);

	pm_da9053_write_reg(DA9052_GPIO0809_REG,
			DA9052_GPIO0809_SMD_SET);
	pm_da9053_read_reg(DA9052_IRQMASKD_REG, &data);
	data |= DA9052_GPI9_IRQ_MASK;
	pm_da9053_write_reg(DA9052_IRQMASKD_REG, data);
#ifdef CONFIG_RTC_DRV_DA9052
	pm_da9053_read_reg(DA9052_ALARMY_REG, &data);
	data |= DA9052_ALARM_IRQ_EN;
	pm_da9053_write_reg(DA9052_ALARMY_REG, data);
#endif
	/* Mask SEQ_RDY_IRQ to avoid some suspend/resume issues */
	pm_da9053_read_reg(DA9052_IRQMASKA_REG, &data);
	data |= DA9052_SEQ_RDY_IRQ_MASK;
	pm_da9053_write_reg(DA9052_IRQMASKA_REG, data);

	pm_da9053_read_reg(DA9052_ID1415_REG, &data);
	data &= 0xf0;
	data |= DA9052_ID1415_SMD_SET;
	pm_da9053_write_reg(DA9052_ID1415_REG, data);

	pm_da9053_write_reg(DA9052_SEQTIMER_REG, 0);
	/* pm_da9053_write_reg(DA9052_SEQB_REG, 0x1f); */

	clk_disable(i2c_clk);
	clk_put(i2c_clk);
	return 0;
}

int da9053_restore_volt_settings(void)
{
	u8 data;
	struct clk *i2c_clk;

	i2c_clk = clk_get(NULL, "i2c_clk");
	if (IS_ERR(i2c_clk)) {
		pr_err("unable to get i2c clk\n");
		return PTR_ERR(i2c_clk);
	}
	clk_enable(i2c_clk);

	pm_da9053_write_reg(DA9052_ID1213_REG, BUCKPERI_RESTORE_SW_STEP);
	pm_da9053_read_reg(DA9052_SUPPLY_REG, &data);
	data |= SUPPLY_RESTORE_VPERISW_EN;
	pm_da9053_write_reg(DA9052_SUPPLY_REG, data);

	clk_disable(i2c_clk);
	clk_put(i2c_clk);
	return 0;
}
