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
#include <linux/io.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <linux/version.h>      /* for KERNEL_VERSION MACRO     */

#include "stfm1000.h"

#define stfm1000_i2c_debug(p, fmt, arg...) \
	do { \
		if ((p)->dbgflg & STFM1000_DBGFLG_I2C) \
			printk(KERN_INFO "stfm1000: " fmt, ##arg); \
	} while (0)

static const char *reg_names[STFM1000_NUM_REGS] = {
#undef REGNAME
#define REGNAME(x) \
	[STFM1000_ ## x / 4] = #x ""

	REGNAME(TUNE1),
	REGNAME(SDNOMINAL),
	REGNAME(PILOTTRACKING),
	REGNAME(INITIALIZATION1),
	REGNAME(INITIALIZATION2),
	REGNAME(INITIALIZATION3),
	REGNAME(INITIALIZATION4),
	REGNAME(INITIALIZATION5),
	REGNAME(INITIALIZATION6),
	REGNAME(REF),
	REGNAME(LNA),
	REGNAME(MIXFILT),
	REGNAME(CLK1),
	REGNAME(CLK2),
	REGNAME(ADC),
	REGNAME(AGC_CONTROL1),
	REGNAME(AGC_CONTROL2),
	REGNAME(DATAPATH),
	REGNAME(RMS),
	REGNAME(AGC_STAT),
	REGNAME(SIGNALQUALITY),
	REGNAME(DCEST),
	REGNAME(RSSI_TONE),
	REGNAME(PILOTCORRECTION),
	REGNAME(ATTENTION),
	REGNAME(CLK3),
	REGNAME(CHIPID),
#undef REGNAME
};

static const int stfm1000_rw_regs[] = {
	STFM1000_TUNE1,
	STFM1000_SDNOMINAL,
	STFM1000_PILOTTRACKING,
	STFM1000_INITIALIZATION1,
	STFM1000_INITIALIZATION2,
	STFM1000_INITIALIZATION3,
	STFM1000_INITIALIZATION4,
	STFM1000_INITIALIZATION5,
	STFM1000_INITIALIZATION6,
	STFM1000_REF,
	STFM1000_LNA,
	STFM1000_MIXFILT,
	STFM1000_CLK1,
	STFM1000_CLK2,
	STFM1000_ADC,
	STFM1000_AGC_CONTROL1,
	STFM1000_AGC_CONTROL2,
	STFM1000_DATAPATH,
	STFM1000_ATTENTION,	/* it's both WR/RD */
};

static const int stfm1000_ra_regs[] = {
	STFM1000_RMS,
	STFM1000_AGC_STAT,
	STFM1000_SIGNALQUALITY,
	STFM1000_DCEST,
	STFM1000_RSSI_TONE,
	STFM1000_PILOTCORRECTION,
	STFM1000_ATTENTION,	/* it's both WR/RD - always read */
	STFM1000_CLK3,
	STFM1000_CHIPID
};

static int verify_writes;

void stfm1000_setup_reg_set(struct stfm1000 *stfm1000)
{
	int i, reg;

	/* set up register sets (read/write) */
	for (i = 0; i < ARRAY_SIZE(stfm1000_rw_regs); i++) {
		reg = stfm1000_rw_regs[i] / 4;
		stfm1000->reg_rw_set[reg / 32] |= 1U << (reg & 31);
		/* printk(KERN_INFO "STFM1000: rw <= %d\n", reg); */
	}

	/* for (i = 0; i < ARRAY_SIZE(stfm1000->reg_rw_set); i++)
		printk("RW[%d] = 0x%08x\n", i, stfm1000->reg_rw_set[i]); */

	/* set up register sets (read only) */
	for (i = 0; i < ARRAY_SIZE(stfm1000_ra_regs); i++) {
		reg = stfm1000_ra_regs[i] / 4;
		stfm1000->reg_ra_set[reg / 32] |= 1U << (reg & 31);
		/* printk(KERN_INFO "STFM1000: rw <= %d\n", reg); */
	}
	/* for (i = 0; i < ARRAY_SIZE(stfm1000->reg_ra_set); i++)
		printk("RO[%d] = 0x%08x\n", i, stfm1000->reg_ra_set[i]); */

	/* clear dirty */
	memset(stfm1000->reg_dirty_set, 0, sizeof(stfm1000->reg_dirty_set));
}

static int stfm1000_reg_is_rw(struct stfm1000 *stfm1000, int reg)
{
	reg >>= 2;
	return !!(stfm1000->reg_rw_set[reg / 32] & (1 << (reg & 31)));
}

static int stfm1000_reg_is_ra(struct stfm1000 *stfm1000, int reg)
{
	reg >>= 2;
	return !!(stfm1000->reg_ra_set[reg / 32] & (1 << (reg & 31)));
}

static int stfm1000_reg_is_dirty(struct stfm1000 *stfm1000, int reg)
{
	reg >>= 2;
	return !!(stfm1000->reg_dirty_set[reg / 32] & (1 << (reg & 31)));
}

static void stfm1000_reg_set_dirty(struct stfm1000 *stfm1000, int reg)
{
	reg >>= 2;
	stfm1000->reg_dirty_set[reg / 32] |= 1 << (reg & 31);
}

static inline int stfm1000_reg_is_writeable(struct stfm1000 *stfm1000, int reg)
{
	return stfm1000_reg_is_rw(stfm1000, reg);
}

static inline int stfm1000_reg_is_readable(struct stfm1000 *stfm1000, int reg)
{
	return stfm1000_reg_is_rw(stfm1000, reg) ||
	       stfm1000_reg_is_ra(stfm1000, reg);
}

/********************************************************/

static int write_reg_internal(struct stfm1000 *stfm1000, int reg, u32 value)
{
	u8 values[5];
	int ret;

	stfm1000_i2c_debug(stfm1000, "%s(%s - 0x%02x, 0x%08x)\n", __func__,
		reg_names[reg / 4], reg, value);

	values[0] = (u8)reg;
	values[1] = (u8)value;
	values[2] = (u8)(value >> 8);
	values[3] = (u8)(value >> 16);
	values[4] = (u8)(value >> 24);
	ret = i2c_master_send(stfm1000->client, values, 5);
	if (ret < 0)
		return ret;
	return 0;
}

static int read_reg_internal(struct stfm1000 *stfm1000, int reg, u32 *value)
{
	u8 regb = reg;
	u8 values[4];
	int ret;

	ret = i2c_master_send(stfm1000->client, &regb, 1);
	if (ret < 0)
		goto out;
	ret = i2c_master_recv(stfm1000->client, values, 4);
	if (ret < 0)
		goto out;
	*value = (u32)values[0] | ((u32)values[1] << 8) |
		 ((u32)values[2] << 16) | ((u32)values[3] << 24);
	ret = 0;

	stfm1000_i2c_debug(stfm1000, "%s(%s - 0x%02x, 0x%08x)\n", __func__,
		reg_names[reg / 4], reg, *value);
out:
	return ret;
}

int stfm1000_raw_write(struct stfm1000 *stfm1000, int reg, u32 value)
{
	int ret;

	mutex_lock(&stfm1000->xfer_lock);
	ret = write_reg_internal(stfm1000, reg, value);
	mutex_unlock(&stfm1000->xfer_lock);

	if (ret < 0)
		dev_err(&stfm1000->client->dev, "%s: failed", __func__);

	return ret;
}

int stfm1000_raw_read(struct stfm1000 *stfm1000, int reg, u32 *value)
{
	int ret;

	mutex_lock(&stfm1000->xfer_lock);
	ret = read_reg_internal(stfm1000, reg, value);
	mutex_unlock(&stfm1000->xfer_lock);

	if (ret < 0)
		dev_err(&stfm1000->client->dev, "%s: failed", __func__);

	return ret;
}

static inline void stfm1000_set_shadow_reg(struct stfm1000 *stfm1000,
	int reg, u32 val)
{
	stfm1000->shadow_regs[reg / 4] = val;
}

static inline u32 stfm1000_get_shadow_reg(struct stfm1000 *stfm1000, int reg)
{
	return stfm1000->shadow_regs[reg / 4];
}

int stfm1000_write(struct stfm1000 *stfm1000, int reg, u32 value)
{
	int ret;

	if (!stfm1000_reg_is_writeable(stfm1000, reg)) {
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&stfm1000->xfer_lock);

	/* same value as last one written? */
	if (stfm1000_reg_is_dirty(stfm1000, reg) &&
		stfm1000_get_shadow_reg(stfm1000, reg) == value) {
		ret = 0;

		stfm1000_i2c_debug(stfm1000, "%s - HIT "
			"(%s - 0x%02x, 0x%08x)\n", __func__,
			reg_names[reg / 4], reg, value);

		goto out_unlock;
	}

	/* actually write the register */
	ret = write_reg_internal(stfm1000, reg, value);
	if (ret < 0)
		goto out_unlock;

	/* update shadow register & mark it as dirty */
	/* only if register is not read always */
	if (!stfm1000_reg_is_ra(stfm1000, reg)) {
		stfm1000_set_shadow_reg(stfm1000, reg, value);
		stfm1000_reg_set_dirty(stfm1000, reg);
	}

out_unlock:
	mutex_unlock(&stfm1000->xfer_lock);

out:
	if (ret < 0)
		dev_err(&stfm1000->client->dev, "%s: failed", __func__);

	if (verify_writes) {
		u32 value2 = ~0;

		stfm1000_raw_read(stfm1000, reg, &value2);

		stfm1000_i2c_debug(stfm1000, "%s - VER "
			"(%s - 0x%02x, W=0x%08x V=0x%08x) %s\n", __func__,
			reg_names[reg / 4], reg, value, value2,
			value == value2 ? "OK" : "** differs **");
	}

	return ret;
}

int stfm1000_read(struct stfm1000 *stfm1000, int reg, u32 *value)
{
	int ret = 0;

	if (!stfm1000_reg_is_readable(stfm1000, reg)) {
		ret = -EINVAL;
		printk(KERN_INFO "%s: !readable(%d)\n", __func__, reg);
		goto out;
	}

	mutex_lock(&stfm1000->xfer_lock);

	/* if the register can be written & is dirty, use the shadow */
	if (stfm1000_reg_is_writeable(stfm1000, reg) &&
		stfm1000_reg_is_dirty(stfm1000, reg)) {

		*value = stfm1000_get_shadow_reg(stfm1000, reg);
		ret = 0;

		stfm1000_i2c_debug(stfm1000, "%s - HIT "
			"(%s - 0x%02x, 0x%08x)\n", __func__,
			reg_names[reg / 4], reg, *value);

		goto out_unlock;
	}

	/* register must be read */
	ret = read_reg_internal(stfm1000, reg, value);
	if (ret < 0)
		goto out;

	/* if the register is writeable, update shadow */
	if (stfm1000_reg_is_writeable(stfm1000, reg)) {
		stfm1000_set_shadow_reg(stfm1000, reg, *value);
		stfm1000_reg_set_dirty(stfm1000, reg);
	}

out_unlock:
	mutex_unlock(&stfm1000->xfer_lock);

out:
	if (ret < 0)
		dev_err(&stfm1000->client->dev, "%s: failed", __func__);

	return ret;
}

int stfm1000_write_masked(struct stfm1000 *stfm1000, int reg, u32 value,
	u32 mask)
{
	int ret = 0;
	u32 old_value;

	if (!stfm1000_reg_is_writeable(stfm1000, reg)) {
		ret = -EINVAL;
		printk(KERN_ERR "%s: !writeable(%d)\n", __func__, reg);
		goto out;
	}

	mutex_lock(&stfm1000->xfer_lock);

	/* if the register wasn't written before, read it */
	if (!stfm1000_reg_is_dirty(stfm1000, reg)) {
		ret = read_reg_internal(stfm1000, reg, &old_value);
		if (ret != 0)
			goto out_unlock;
	} else	/* register was written, use the last value */
		old_value = stfm1000_get_shadow_reg(stfm1000, reg);

	/* perform masking */
	value = (old_value & ~mask) | (value & mask);

	/* if we write the same value, don't bother */
	if (stfm1000_reg_is_dirty(stfm1000, reg) && value == old_value) {
		ret = 0;

		stfm1000_i2c_debug(stfm1000, "%s - HIT "
			"(%s - 0x%02x, 0x%08x)\n", __func__,
			reg_names[reg / 4], reg, value);

		goto out_unlock;
	}

	/* actually write the register to the chip */
	ret = write_reg_internal(stfm1000, reg, value);
	if (ret < 0)
		goto out_unlock;

	/* if no error, update the shadow register and mark it as dirty */
	stfm1000_set_shadow_reg(stfm1000, reg, value);
	stfm1000_reg_set_dirty(stfm1000, reg);

out_unlock:
	mutex_unlock(&stfm1000->xfer_lock);

out:
	if (ret < 0)
		dev_err(&stfm1000->client->dev, "%s: failed", __func__);

	if (verify_writes) {
		u32 value2 = ~0;

		stfm1000_raw_read(stfm1000, reg, &value2);

		stfm1000_i2c_debug(stfm1000, "%s - VER "
			"(%s - 0x%02x, W=0x%08x V=0x%08x) %s\n", __func__,
			reg_names[reg / 4], reg, value, value2,
			value == value2 ? "OK" : "** differs **");
	}

	return ret;
}

int stfm1000_set_bits(struct stfm1000 *stfm1000, int reg, u32 value)
{
	return stfm1000_write_masked(stfm1000, reg, value, value);
}

int stfm1000_clear_bits(struct stfm1000 *stfm1000, int reg, u32 value)
{
	return stfm1000_write_masked(stfm1000, reg, ~value, value);
}

int stfm1000_write_regs(struct stfm1000 *stfm1000,
	const struct stfm1000_reg *reg)
{
	int ret;

	for (; reg && reg->regno != STFM1000_REG_END; reg++) {

		if (reg->regno == STFM1000_REG_DELAY) {
			msleep(reg->value);
			continue;
		}

		if (reg->regno & STFM1000_REG_SET_BITS_MASK)
			ret = stfm1000_set_bits(stfm1000, reg->regno & 0xff,
				reg->value);
		else if (reg->regno & STFM1000_REG_CLEAR_BITS_MASK)
			ret = stfm1000_clear_bits(stfm1000, reg->regno & 0xff,
				reg->value);
		else
			ret = stfm1000_write(stfm1000, reg->regno, reg->value);

		if (ret != 0) {
			printk(KERN_ERR "%s: failed to write reg 0x%x "
				"with 0x%08x\n",
				__func__, reg->regno, reg->value);
			return ret;
		}
	}
	return 0;
}
