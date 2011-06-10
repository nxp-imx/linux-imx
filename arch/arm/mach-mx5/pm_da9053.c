/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

/** Defines ********************************************************************
*******************************************************************************/


/* IMX I2C registers */
#define IMX_I2C_IADR	0x00	/* i2c slave address */
#define IMX_I2C_IFDR	0x04	/* i2c frequency divider */
#define IMX_I2C_I2CR	0x08	/* i2c control */
#define IMX_I2C_I2SR	0x0C	/* i2c status */
#define IMX_I2C_I2DR	0x10	/* i2c transfer data */

/* Bits of IMX I2C registers */
#define I2SR_RXAK	0x01
#define I2SR_IIF	0x02
#define I2SR_SRW	0x04
#define I2SR_IAL	0x10
#define I2SR_IBB	0x20
#define I2SR_IAAS	0x40
#define I2SR_ICF	0x80
#define I2CR_RSTA	0x04
#define I2CR_TXAK	0x08
#define I2CR_MTX	0x10
#define I2CR_MSTA	0x20
#define I2CR_IIEN	0x40
#define I2CR_IEN	0x80

static void __iomem		*base;
static int stopped;

/** Functions for IMX I2C adapter driver ***************************************
*******************************************************************************/

static int pm_i2c_imx_bus_busy(int for_busy)
{
	unsigned int temp;

	while (1) {
		temp = readb(base + IMX_I2C_I2SR);
		if (for_busy && (temp & I2SR_IBB))
			break;
		if (!for_busy && !(temp & I2SR_IBB))
			break;
		pr_debug("waiting bus busy=%d\n", for_busy);
	}

	return 0;
}

static int pm_i2c_imx_trx_complete(void)
{
	unsigned int temp;
	while (!((temp = readb(base + IMX_I2C_I2SR)) & I2SR_IIF))
		pr_debug("waiting or I2SR_IIF\n");
	temp &= ~I2SR_IIF;
	writeb(temp, base + IMX_I2C_I2SR);

	return 0;
}

static int pm_i2c_imx_acked(void)
{
	if (readb(base + IMX_I2C_I2SR) & I2SR_RXAK) {
		pr_info("<%s> No ACK\n", __func__);
		return -EIO;  /* No ACK */
	}
	return 0;
}

static int pm_i2c_imx_start(void)
{
	unsigned int temp = 0;
	int result;

	/* Enable I2C controller */
	writeb(0, base + IMX_I2C_I2SR);
	writeb(I2CR_IEN, base + IMX_I2C_I2CR);

	/* Wait controller to be stable */
	udelay(50);

	/* Start I2C transaction */
	temp = readb(base + IMX_I2C_I2CR);
	temp |= I2CR_MSTA;
	writeb(temp, base + IMX_I2C_I2CR);
	result = pm_i2c_imx_bus_busy(1);

	temp |= I2CR_IIEN | I2CR_MTX | I2CR_TXAK;
	writeb(temp, base + IMX_I2C_I2CR);
	return result;
}

static void pm_i2c_imx_stop(void)
{
	unsigned int temp = 0;

	/* Stop I2C transaction */
	temp = readb(base + IMX_I2C_I2CR);
	temp &= ~(I2CR_MSTA | I2CR_MTX);
	writeb(temp, base + IMX_I2C_I2CR);

	pm_i2c_imx_bus_busy(0);

	/* Disable I2C controller */
	writeb(0, base + IMX_I2C_I2CR);
}

static int pm_i2c_imx_write(struct i2c_msg *msgs)
{
	int i, result;

	/* write slave address */
	writeb(msgs->addr << 1, base + IMX_I2C_I2DR);
	result = pm_i2c_imx_trx_complete();
	if (result)
		return result;
	result = pm_i2c_imx_acked();
	if (result)
		return result;

	/* write data */
	for (i = 0; i < msgs->len; i++) {
		writeb(msgs->buf[i], base + IMX_I2C_I2DR);
		result = pm_i2c_imx_trx_complete();
		if (result)
			return result;
		result = pm_i2c_imx_acked();
		if (result)
			return result;
	}
	return 0;
}

static int pm_i2c_imx_read(struct i2c_msg *msgs)
{
	int i, result;
	unsigned int temp;

	/* write slave address */
	writeb((msgs->addr << 1) | 0x01, base + IMX_I2C_I2DR);
	result = pm_i2c_imx_trx_complete();
	if (result)
		return result;
	result = pm_i2c_imx_acked();
	if (result)
		return result;

	/* setup bus to read data */
	temp = readb(base + IMX_I2C_I2CR);
	temp &= ~I2CR_MTX;
	if (msgs->len - 1)
		temp &= ~I2CR_TXAK;
	writeb(temp, base + IMX_I2C_I2CR);
	readb(base + IMX_I2C_I2DR); /* dummy read */

	/* read data */
	for (i = 0; i < msgs->len; i++) {
		result = pm_i2c_imx_trx_complete();
		if (result)
			return result;
		if (i == (msgs->len - 1)) {
			/* It must generate STOP before read I2DR to prevent
			   controller from generating another clock cycle */
			temp = readb(base + IMX_I2C_I2CR);
			temp &= ~(I2CR_MSTA | I2CR_MTX);
			writeb(temp, base + IMX_I2C_I2CR);
			pm_i2c_imx_bus_busy(0);
			stopped = 1;
		} else if (i == (msgs->len - 2)) {
			temp = readb(base + IMX_I2C_I2CR);
			temp |= I2CR_TXAK;
			writeb(temp, base + IMX_I2C_I2CR);
		}
		msgs->buf[i] = readb(base + IMX_I2C_I2DR);
	}
	return 0;
}

int pm_i2c_imx_xfer(struct i2c_msg *msgs, int num)
{
	unsigned int i, temp;
	int result;

	/* Start I2C transfer */
	result = pm_i2c_imx_start();
	if (result)
		goto fail0;

	/* read/write data */
	for (i = 0; i < num; i++) {
		if (i) {
			temp = readb(base + IMX_I2C_I2CR);
			temp |= I2CR_RSTA;
			writeb(temp, base + IMX_I2C_I2CR);
			result =  pm_i2c_imx_bus_busy(1);
			if (result)
				goto fail0;
		}
		/* write/read data */
		if (msgs[i].flags & I2C_M_RD)
			result = pm_i2c_imx_read(&msgs[i]);
		else
			result = pm_i2c_imx_write(&msgs[i]);
		if (result)
			goto fail0;
	}

fail0:
	/* Stop I2C transfer */
	pm_i2c_imx_stop();

	return (result < 0) ? result : num;
}

void pm_da9053_i2c_init(u32 base_addr)
{
	base = ioremap(base_addr, SZ_4K);
}

void pm_da9053_i2c_deinit(void)
{
	iounmap(base);
}

void pm_da9053_read_reg(u8 reg, u8 *value)
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

void pm_da9053_write_reg(u8 reg, u8 value)
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

static u8 volt_settings[DA9052_LDO10_REG - DA9052_BUCKCORE_REG + 1];
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

void pm_da9053_dump(int start, int end)
{
	u8 reg, data;
	for (reg = start; reg <= end; reg++) {
		pm_da9053_read_reg(reg, &data);
		pr_info("reg %u = 0x%2x\n",
			reg, data);
	}
}

#define DA9053_SLEEP_DELAY 0x1f

#define DA9052_CONTROLC_SMD_SET 0x62
#define DA9052_GPIO0809_SMD_SET 0x18
#define DA9052_ID1415_SMD_SET   0x1
#define DA9052_GPI9_IRQ_MASK    0x2

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

int da9053_poweroff_cmd(void)
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

	pm_da9053_read_reg(DA9052_CONTROLB_REG, &data);
	data |= DA9052_CONTROLB_SHUTDOWN;
	pm_da9053_write_reg(DA9052_CONTROLB_REG, data);

	clk_disable(i2c_clk);
	clk_put(i2c_clk);
	return 0;
}

int da9053_resume_dump(void)
{
	unsigned char buf[2] = {0, 0};
	struct clk *i2c_clk;
	buf[0] = 29;

	i2c_clk = clk_get(NULL, "i2c_clk");
	if (IS_ERR(i2c_clk)) {
		pr_err("unable to get i2c clk\n");
		return PTR_ERR(i2c_clk);
	}
	clk_enable(i2c_clk);

	pm_da9053_dump(46, 59);
	pm_da9053_dump(25, 25);
	pm_da9053_dump(29, 29);
	pm_da9053_dump(36, 36);

	clk_disable(i2c_clk);
	clk_put(i2c_clk);
	return 0;
}

