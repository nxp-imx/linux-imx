/**
 * @file pca9450.h  NXP PCA9450 header file
 *
 * Copyright 2019 NXP
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_PCA9450_H
#define __LINUX_MFD_PCA9450_H

#include <linux/regmap.h>

enum {
	PCA9450_BUCK1	=	0,
	PCA9450_BUCK2,
	PCA9450_BUCK3,
	PCA9450_BUCK4,
	PCA9450_BUCK5,
	PCA9450_BUCK6,
	PCA9450_LDO1,
	PCA9450_LDO2,
	PCA9450_LDO3,
	PCA9450_LDO4,
	PCA9450_LDO5,
	PCA9450_REGULATOR_CNT,
};

#define PCA9450_SUPPLY_STATE_ENABLED    0x1

#define PCA9450_BUCK1_VOLTAGE_NUM	0x80
#define PCA9450_BUCK2_VOLTAGE_NUM	0x80
#define PCA9450_BUCK3_VOLTAGE_NUM	0x80
#define PCA9450_BUCK4_VOLTAGE_NUM	0x80

#define PCA9450_BUCK5_VOLTAGE_NUM	0x80
#define PCA9450_BUCK6_VOLTAGE_NUM	0x80

#define PCA9450_LDO1_VOLTAGE_NUM	0x08
#define PCA9450_LDO2_VOLTAGE_NUM	0x08
#define PCA9450_LDO3_VOLTAGE_NUM	0x20
#define PCA9450_LDO4_VOLTAGE_NUM	0x20
#define PCA9450_LDO5_VOLTAGE_NUM	0x10

enum {
	PCA9450_REG_DEV_ID      = 0x00,
	PCA9450_INT1            = 0x01,
	PCA9450_INT1_MSK        = 0x02,
	PCA9450_STATUS1         = 0x03,
	PCA9450_STATUS2         = 0x04,
	PCA9450_PWRON_STAT      = 0x05,
	PCA9450_SW_RST          = 0x06,
	PCA9450_PWR_CTRL        = 0x07,
	PCA9450_RESET_CTRL      = 0x08,
	PCA9450_CONFIG1         = 0x09,
	PCA9450_CONFIG2         = 0x0A,
	PCA9450_BUCK123_DVS     = 0x0C,
	PCA9450_BUCK1OUT_LIMIT  = 0x0D,
	PCA9450_BUCK2OUT_LIMIT  = 0x0E,
	PCA9450_BUCK3OUT_LIMIT  = 0x0F,
	PCA9450_BUCK1CTRL       = 0x10,
	PCA9450_BUCK1OUT_DVS0   = 0x11,
	PCA9450_BUCK1OUT_DVS1   = 0x12,
	PCA9450_BUCK2CTRL       = 0x13,
	PCA9450_BUCK2OUT_DVS0   = 0x14,
	PCA9450_BUCK2OUT_DVS1   = 0x15,
	PCA9450_BUCK3CTRL       = 0x16,
	PCA9450_BUCK3OUT_DVS0   = 0x17,
	PCA9450_BUCK3OUT_DVS1   = 0x18,
	PCA9450_BUCK4CTRL       = 0x19,
	PCA9450_BUCK4OUT        = 0x1A,
	PCA9450_BUCK5CTRL       = 0x1B,
	PCA9450_BUCK5OUT        = 0x1C,
	PCA9450_BUCK6CTRL       = 0x1D,
	PCA9450_BUCK6OUT        = 0x1E,
	PCA9450_LDO_AD_CTRL     = 0x20,
	PCA9450_LDO1CTRL        = 0x21,
	PCA9450_LDO2CTRL        = 0x22,
	PCA9450_LDO3CTRL        = 0x23,
	PCA9450_LDO4CTRL        = 0x24,
	PCA9450_LDO5CTRL_L      = 0x25,
	PCA9450_LDO5CTRL_H      = 0x26,
	PCA9450_LOADSW_CTRL     = 0x2A,
	PCA9450_VRFLT1_STS      = 0x2B,
	PCA9450_VRFLT2_STS      = 0x2C,
	PCA9450_VRFLT1_MASK     = 0x2D,
	PCA9450_VRFLT2_MASK     = 0x2E,
	PCA9450_MAX_REGISTER	= 0x2F,
};

/* PCA9450 BUCK ENMODE bits */
#define BUCK_ENMODE_OFF			0x00
#define BUCK_ENMODE_ONREQ		0x01
#define BUCK_ENMODE_ONREQ_STBYREQ	0x02
#define BUCK_ENMODE_ON			0x03

/* PCA9450_REG_BUCK1_CTRL bits */
#define BUCK1_RAMP_MASK			0xC0
#define BUCK1_RAMP_25MV			0x0
#define BUCK1_RAMP_12P5MV		0x1
#define BUCK1_RAMP_6P25MV		0x2
#define BUCK1_RAMP_3P125MV		0x3
#define BUCK1_DVS_CTRL			0x10
#define BUCK1_AD			0x08
#define BUCK1_FPWM			0x04
#define BUCK1_ENMODE_MASK		0x03

/* PCA9450_REG_BUCK2_CTRL bits */
#define BUCK2_RAMP_MASK			0xC0
#define BUCK2_RAMP_25MV			0x0
#define BUCK2_RAMP_12P5MV		0x1
#define BUCK2_RAMP_6P25MV		0x2
#define BUCK2_RAMP_3P125MV		0x3
#define BUCK2_DVS_CTRL			0x10
#define BUCK2_AD			0x08
#define BUCK2_FPWM			0x04
#define BUCK2_ENMODE_MASK		0x03

/* PCA9450_REG_BUCK3_CTRL bits */
#define BUCK3_RAMP_MASK			0xC0
#define BUCK3_RAMP_25MV			0x0
#define BUCK3_RAMP_12P5MV		0x1
#define BUCK3_RAMP_6P25MV		0x2
#define BUCK3_RAMP_3P125MV		0x3
#define BUCK3_DVS_CTRL			0x10
#define BUCK3_AD			0x08
#define BUCK3_FPWM			0x04
#define BUCK3_ENMODE_MASK		0x03

/* PCA9450_REG_BUCK4_CTRL bits */
#define BUCK4_AD			0x08
#define BUCK4_FPWM			0x04
#define BUCK4_ENMODE_MASK		0x03

/* PCA9450_REG_BUCK5_CTRL bits */
#define BUCK5_AD			0x08
#define BUCK5_FPWM			0x04
#define BUCK5_ENMODE_MASK		0x03

/* PCA9450_REG_BUCK6_CTRL bits */
#define BUCK6_AD			0x08
#define BUCK6_FPWM			0x04
#define BUCK6_ENMODE_MASK		0x03

/* PCA9450_BUCK1OUT_DVS0 bits */
#define BUCK1OUT_DVS0_MASK		0x7F
#define BUCK1OUT_DVS0_DEFAULT		0x14

/* PCA9450_BUCK1OUT_DVS1 bits */
#define BUCK1OUT_DVS1_MASK		0x7F
#define BUCK1OUT_DVS1_DEFAULT		0x14

/* PCA9450_BUCK2OUT_DVS0 bits */
#define BUCK2OUT_DVS0_MASK		0x7F
#define BUCK2OUT_DVS0_DEFAULT		0x14

/* PCA9450_BUCK2OUT_DVS1 bits */
#define BUCK2OUT_DVS1_MASK		0x7F
#define BUCK2OUT_DVS1_DEFAULT		0x14

/* PCA9450_BUCK3OUT_DVS0 bits */
#define BUCK3OUT_DVS0_MASK		0x7F
#define BUCK3OUT_DVS0_DEFAULT		0x14

/* PCA9450_BUCK3OUT_DVS1 bits */
#define BUCK3OUT_DVS1_MASK		0x7F
#define BUCK3OUT_DVS1_DEFAULT		0x14

/* PCA9450_REG_BUCK4OUT bits */
#define BUCK4OUT_MASK			0x7F
#define BUCK4OUT_DEFAULT		0x6C

/* PCA9450_REG_BUCK5OUT bits */
#define BUCK5OUT_MASK			0x7F
#define BUCK5OUT_DEFAULT		0x30

/* PCA9450_REG_BUCK6OUT bits */
#define BUCK6OUT_MASK			0x7F
#define BUCK6OUT_DEFAULT		0x14

/* PCA9450_REG_IRQ bits */
#define IRQ_PWRON			0x80
#define IRQ_WDOGB			0x40
#define IRQ_VR_FLT1			0x10
#define IRQ_VR_FLT2			0x08
#define IRQ_LOWVSYS			0x04
#define IRQ_THERM_105			0x02
#define IRQ_THERM_125			0x01

/* PCA9450 interrupt masks */
enum {
	PCA9450_INT_MASK	=	0xDF,
};
/* PCA9450 interrupt irqs */
enum {
	PCA9450_IRQ		=	0x0,
};

/* PCA9450_REG_LDO1_VOLT bits */
#define LDO1_EN_MASK			0xC0
#define LDO1OUT_MASK			0x07

/* PCA9450_REG_LDO2_VOLT bits */
#define LDO2_EN_MASK			0xC0
#define LDO2OUT_MASK			0x07

/* PCA9450_REG_LDO3_VOLT bits */
#define LDO3_EN_MASK			0xC0
#define LDO3OUT_MASK			0x1F

/* PCA9450_REG_LDO4_VOLT bits */
#define LDO4_EN_MASK			0xC0
#define LDO4OUT_MASK			0x1F

/* PCA9450_REG_LDO5_VOLT bits */
#define LDO5L_EN_MASK			0xC0
#define LDO5LOUT_MASK			0x0F

#define LDO5H_EN_MASK			0xC0
#define LDO5HOUT_MASK			0x0F

/*
 * @brief Board platform data may be used to initialize regulators.
 */

struct pca9450_board {
	struct regulator_init_data *init_data[PCA9450_REGULATOR_CNT];
	int	gpio_intr;
	int	irq_base;
};

/*
 * @brief pca9450 sub-driver chip access routines
 */

struct pca9450 {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct mutex io_mutex;
	unsigned int id;

	/* IRQ Handling */
	int	chip_irq;
	struct regmap_irq_chip_data *irq_data;

	/* Client devices */
	struct pca9450_pmic *pmic;
	struct pca9450_power *power;

	struct pca9450_board *of_plat_data;
};

static inline int pca9450_chip_id(struct pca9450 *pca9450)
{
	return pca9450->id;
}

/*
 * @brief pca9450_reg_read
 * read single register's value of pca9450
 * @param pca9450 device to read
 * @param reg register address
 * @return register value if success
 *         error number if fail
 */
static inline int pca9450_reg_read(struct pca9450 *pca9450, u8 reg)
{
	int r, val;

	r = regmap_read(pca9450->regmap, reg, &val);
	if (r < 0)
		return r;

	return val;
}

/*
 * @brief pca9450_reg_write
 * write single register of pca9450
 * @param pca9450 device to write
 * @param reg register address
 * @param val value to write
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int pca9450_reg_write(struct pca9450 *pca9450, u8 reg,
		unsigned int val)
{
	return regmap_write(pca9450->regmap, reg, val);
}

/*
 * @brief pca9450_set_bits
 * set bits in one register of pca9450
 * @param pca9450 device to read
 * @param reg register address
 * @param mask mask bits
 * @retval 0 if success
 * @retval negative error number if fail
 */
static inline int pca9450_set_bits(struct pca9450 *pca9450, u8 reg, u8 mask)
{
	return regmap_update_bits(pca9450->regmap, reg, mask, mask);
}

/*
 * @brief pca9450_clear_bits
 * clear bits in one register of pca9450
 * @param pca9450 device to read
 * @param reg register address
 * @param mask mask bits
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int pca9450_clear_bits(struct pca9450 *pca9450, u8 reg,
		u8 mask)
{
	return regmap_update_bits(pca9450->regmap, reg, mask, 0);
}

/*
 * @brief pca9450_update_bits
 * update bits in one register of pca9450
 * @param pca9450 device to read
 * @param reg register address
 * @param mask mask bits
 * @param val value to update
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int pca9450_update_bits(struct pca9450 *pca9450, u8 reg,
					   u8 mask, u8 val)
{
	return regmap_update_bits(pca9450->regmap, reg, mask, val);
}

/*
 * @brief pca9450 platform data type
 */
struct pca9450_gpo_plat_data {
	u32 drv;
	int gpio_base;
};

u8 ext_pca9450_reg_read8(u8 reg);
int ext_pca9450_reg_write8(int reg, u8 val);
#endif /* __LINUX_MFD_PCA9450_H */
