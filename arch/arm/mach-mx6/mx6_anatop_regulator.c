/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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

/*
 * mx6_anatop_regulator.c  --  i.MX6 Driver for Anatop regulators
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>

#include <mach/clock.h>
#include <mach/system.h>

#include "crm_regs.h"
#include "regs-anadig.h"

#define GPC_PGC_GPU_PGCR_OFFSET	0x260
#define GPC_CNTR_OFFSET		0x0

#define LDO_RAMP_UP_UNIT_IN_CYCLES	64 /* 64 cycles per step */
#define LDO_RAMP_UP_FREQ_IN_MHZ		24 /* time base on 24M OSC */

extern struct platform_device sgtl5000_vdda_reg_devices;
extern struct platform_device sgtl5000_vddio_reg_devices;
extern struct platform_device sgtl5000_vddd_reg_devices;
extern void __iomem *gpc_base;
/* we use the below flag to keep PU regulator state, because enable/disable
of PU regulator share with the same register as  voltage set of PU regulator.
PU voltage set by cpufreq driver if the flag is set, and enable/disable by
GPU/VPU driver*/
static unsigned int pu_is_enabled;
static unsigned int get_clk;
static struct clk *gpu3d_clk, *gpu3d_shade_clk, *gpu2d_clk, *gpu2d_axi_clk;
static struct clk *openvg_axi_clk, *vpu_clk;
extern int external_pureg;
extern struct regulator *pu_regulator;
extern u32 enable_ldo_mode;


static int get_voltage(struct anatop_regulator *sreg)
{
	int uv;
	struct anatop_regulator_data *rdata = sreg->rdata;

	if (sreg->rdata->control_reg) {
		u32 val = (__raw_readl(rdata->control_reg) >>
			   rdata->vol_bit_shift) & rdata->vol_bit_mask;
		uv = rdata->min_voltage + (val - rdata->min_bit_val) * 25000;
		pr_debug("vddio = %d, val=%u\n", uv, val);
		return uv;
	} else {
		pr_debug("Regulator not supported.\n");
		return -ENOTSUPP;
	}
}

static int set_voltage(struct anatop_regulator *sreg, int uv)
{
	u32 val, reg;
	u32 delay, steps, old_val;

	pr_debug("%s: uv %d, min %d, max %d\n", __func__,
		uv, sreg->rdata->min_voltage, sreg->rdata->max_voltage);

	if (uv < sreg->rdata->min_voltage || uv > sreg->rdata->max_voltage)
		return -EINVAL;

	if (sreg->rdata->control_reg) {
		val = sreg->rdata->min_bit_val +
		      (uv - sreg->rdata->min_voltage) / 25000;

		reg = (__raw_readl(sreg->rdata->control_reg) &
			~(sreg->rdata->vol_bit_mask <<
			sreg->rdata->vol_bit_shift));
		pr_debug("%s: calculated val %d\n", __func__, val);

		old_val = (__raw_readl(sreg->rdata->control_reg) >>
			sreg->rdata->vol_bit_shift) & sreg->rdata->vol_bit_mask;

		__raw_writel((val << sreg->rdata->vol_bit_shift) | reg,
			     sreg->rdata->control_reg);

		if (sreg->rdata->control_reg == (unsigned int)(MXC_PLL_BASE +
			HW_ANADIG_REG_CORE)) {
			/* calculate how many steps to ramp up */
			steps = (val > old_val) ? val - old_val : 0;
			if (steps) {
				switch (sreg->rdata->vol_bit_shift) {
				case BP_ANADIG_REG_CORE_REG0_TRG:
				reg = (__raw_readl(MXC_PLL_BASE +
					HW_ANADIG_ANA_MISC2) &
					BM_ANADIG_ANA_MISC2_REG0_STEP_TIME) >>
					BP_ANADIG_ANA_MISC2_REG0_STEP_TIME;
					break;
				case BP_ANADIG_REG_CORE_REG1_TRG:
				reg = (__raw_readl(MXC_PLL_BASE +
					HW_ANADIG_ANA_MISC2) &
					BM_ANADIG_ANA_MISC2_REG1_STEP_TIME) >>
					BP_ANADIG_ANA_MISC2_REG1_STEP_TIME;
					break;
				case BP_ANADIG_REG_CORE_REG2_TRG:
				reg = (__raw_readl(MXC_PLL_BASE +
					HW_ANADIG_ANA_MISC2) &
					BM_ANADIG_ANA_MISC2_REG2_STEP_TIME) >>
					BP_ANADIG_ANA_MISC2_REG2_STEP_TIME;
					break;
				default:
					break;
				}

				/*
				 * the delay time for LDO ramp up time is
				 * based on the register setting, we need
				 * to calculate how many steps LDO need to
				 * ramp up, and how much delay needs. (us)
				 */
				delay = steps * ((LDO_RAMP_UP_UNIT_IN_CYCLES <<
					reg) / LDO_RAMP_UP_FREQ_IN_MHZ + 1);
				udelay(delay);
				pr_debug("%s: %s: delay %d, steps %d, uv %d\n",
					__func__, sreg->rdata->name, delay,
					steps, uv);
			}
		}

		return 0;
	} else {
		pr_debug("Regulator not supported.\n");
		return -ENOTSUPP;
	}
}

static int pu_enable(struct anatop_regulator *sreg)
{
	unsigned int reg, vddsoc;
	int ret = 0;
	/*get PU related clk to finish PU regulator power up*/
	if (!get_clk) {
		if (!cpu_is_mx6sl()) {
			gpu3d_clk = clk_get(NULL, "gpu3d_clk");
			if (IS_ERR(gpu3d_clk))
				printk(KERN_ERR "%s: failed to get gpu3d_clk!\n"
					, __func__);
			gpu3d_shade_clk = clk_get(NULL, "gpu3d_shader_clk");
			if (IS_ERR(gpu3d_shade_clk))
				printk(KERN_ERR "%s: failed to get shade_clk!\n"
					, __func__);
			if (IS_ERR(vpu_clk))
				printk(KERN_ERR "%s: failed to get vpu_clk!\n",
					__func__);
		}
		gpu2d_clk = clk_get(NULL, "gpu2d_clk");
		if (IS_ERR(gpu2d_clk))
			printk(KERN_ERR "%s: failed to get gpu2d_clk!\n",
				__func__);
		gpu2d_axi_clk = clk_get(NULL, "gpu2d_axi_clk");
		if (IS_ERR(gpu2d_axi_clk))
			printk(KERN_ERR "%s: failed to get gpu2d_axi_clk!\n",
				__func__);
		openvg_axi_clk = clk_get(NULL, "openvg_axi_clk");
		if (IS_ERR(openvg_axi_clk))
			printk(KERN_ERR "%s: failed to get openvg_axi_clk!\n",
				__func__);
		get_clk = 1;

	}
	if (external_pureg) {
		/*enable extern PU regulator*/
		ret = regulator_enable(pu_regulator);
		if (ret < 0)
			printk(KERN_ERR "%s: enable pu error!\n", __func__);
	} else {
		/*Track the voltage of VDDPU with VDDSOC if use internal PU
		*regulator.
		*/
		reg = __raw_readl(ANADIG_REG_CORE);
		vddsoc  = reg & (ANADIG_REG_TARGET_MASK <<
				ANADIG_REG2_SOC_TARGET_OFFSET);
		reg &= ~(ANADIG_REG_TARGET_MASK <<
				ANADIG_REG1_PU_TARGET_OFFSET);
		reg |= vddsoc >> (ANADIG_REG2_SOC_TARGET_OFFSET
				-ANADIG_REG1_PU_TARGET_OFFSET);
		__raw_writel(reg, ANADIG_REG_CORE);
	}

	/* Need to wait for the regulator to come back up */
	/*
	 * Delay time is based on the number of 24MHz clock cycles
	 * programmed in the ANA_MISC2_BASE_ADDR for each
	 * 25mV step.
	 */
	udelay(150);
	/*enable gpu clock to powerup GPU right.*/
	if (get_clk) {
		if (!cpu_is_mx6sl()) {
			clk_enable(gpu3d_clk);
			clk_enable(gpu3d_shade_clk);
			clk_enable(vpu_clk);
		}
		clk_enable(gpu2d_clk);
		clk_enable(gpu2d_axi_clk);
		clk_enable(openvg_axi_clk);
	}
	/* enable power up request */
	reg = __raw_readl(gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
	__raw_writel(reg | 0x1, gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
	/* power up request */
	reg = __raw_readl(gpc_base + GPC_CNTR_OFFSET);
	__raw_writel(reg | 0x2, gpc_base + GPC_CNTR_OFFSET);
	/* Wait for the power up bit to clear */
	while (__raw_readl(gpc_base + GPC_CNTR_OFFSET) & 0x2)
		;
	/* Enable the Brown Out detection. */
	reg = __raw_readl(ANA_MISC2_BASE_ADDR);
	reg |= ANADIG_ANA_MISC2_REG1_BO_EN;
	__raw_writel(reg, ANA_MISC2_BASE_ADDR);
	if (enable_ldo_mode != LDO_MODE_BYPASSED) {
		/* Unmask the ANATOP brown out interrupt in the GPC. */
		reg = __raw_readl(gpc_base + 0x14);
		reg &= ~0x80000000;
		__raw_writel(reg, gpc_base + 0x14);
	}
	pu_is_enabled = 1;
	if (get_clk) {
		if (!cpu_is_mx6sl()) {
			clk_disable(gpu3d_clk);
			clk_disable(gpu3d_shade_clk);
			clk_disable(vpu_clk);
		}
		clk_disable(gpu2d_clk);
		clk_disable(gpu2d_axi_clk);
		clk_disable(openvg_axi_clk);
	}
	return 0;
}

static int pu_disable(struct anatop_regulator *sreg)
{
	unsigned int reg;
	int ret = 0;

	/* Disable the brown out detection since we are going to be
	  * disabling the LDO.
	  */
	reg = __raw_readl(ANA_MISC2_BASE_ADDR);
	reg &= ~ANADIG_ANA_MISC2_REG1_BO_EN;
	__raw_writel(reg, ANA_MISC2_BASE_ADDR);

	/* Power gate the PU LDO. */
	/* Power gate the PU domain first. */
	/* enable power down request */
	reg = __raw_readl(gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
	__raw_writel(reg | 0x1, gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
	/* power down request */
	reg = __raw_readl(gpc_base + GPC_CNTR_OFFSET);
	__raw_writel(reg | 0x1, gpc_base + GPC_CNTR_OFFSET);
	/* Wait for power down to complete. */
	while (__raw_readl(gpc_base + GPC_CNTR_OFFSET) & 0x1)
			;
	if (enable_ldo_mode != LDO_MODE_BYPASSED) {
		/* Mask the ANATOP brown out interrupt in the GPC. */
		reg = __raw_readl(gpc_base + 0x14);
		reg |= 0x80000000;
		__raw_writel(reg, gpc_base + 0x14);
	}

	if (external_pureg) {
		/*disable extern PU regulator*/
		ret = regulator_disable(pu_regulator);
		if (ret < 0)
			printk(KERN_ERR "%s: disable pu error!\n", __func__);
	} else {
		/* PU power gating. */
		reg = __raw_readl(ANADIG_REG_CORE);
		reg &= ~(ANADIG_REG_TARGET_MASK <<
			ANADIG_REG1_PU_TARGET_OFFSET);
		__raw_writel(reg, ANADIG_REG_CORE);
	}
	pu_is_enabled = 0;
	/* Clear the BO interrupt in the ANATOP. */
	reg = __raw_readl(ANADIG_MISC1_REG);
	reg |= 0x80000000;
	__raw_writel(reg, ANADIG_MISC1_REG);
	return 0;
}
static int is_pu_enabled(struct anatop_regulator *sreg)
{
	return pu_is_enabled;
}
static int enable(struct anatop_regulator *sreg)
{
	return 0;
}

static int disable(struct anatop_regulator *sreg)
{
	return 0;
}

static int is_enabled(struct anatop_regulator *sreg)
{
	return 1;
}
static int vdd3p0_enable(struct anatop_regulator *sreg)
{
	__raw_writel(BM_ANADIG_REG_3P0_ENABLE_LINREG,
					sreg->rdata->control_reg+4);
	return 0;
}

static int vdd3p0_disable(struct anatop_regulator *sreg)
{
	__raw_writel(BM_ANADIG_REG_3P0_ENABLE_LINREG,
					sreg->rdata->control_reg+8);
	return 0;
}

static int vdd3p0_is_enabled(struct anatop_regulator *sreg)
{
	return !!(__raw_readl(sreg->rdata->control_reg) & BM_ANADIG_REG_3P0_ENABLE_LINREG);
}

static struct anatop_regulator_data vddpu_data = {
	.name		= "vddpu",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= pu_enable,
	.disable	= pu_disable,
	.is_enabled	= is_pu_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_CORE),
	.vol_bit_shift	= 9,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 1,
	.min_voltage	= 725000,
	.max_voltage	= 1300000,
};

static struct anatop_regulator_data vddcore_data = {
	.name		= "vddcore",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_CORE),
	.vol_bit_shift	= 0,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 1,
	.min_voltage	= 725000,
	.max_voltage	= 1300000,
};

static struct anatop_regulator_data vddsoc_data = {
	.name		= "vddsoc",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_CORE),
	.vol_bit_shift	= 18,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 1,
	.min_voltage	= 725000,
	.max_voltage	= 1300000,
};

static struct anatop_regulator_data vdd2p5_data = {
	.name		= "vdd2p5",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_2P5),
	.vol_bit_shift	= 8,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 0,
	.min_voltage	= 2000000,
	.max_voltage	= 2775000,
};

static struct anatop_regulator_data vdd1p1_data = {
	.name		= "vdd1p1",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_1P1),
	.vol_bit_shift	= 8,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 4,
	.min_voltage	= 800000,
	.max_voltage	= 1400000,
};

static struct anatop_regulator_data vdd3p0_data = {
	.name		= "vdd3p0",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= vdd3p0_enable,
	.disable		= vdd3p0_disable,
	.is_enabled	= vdd3p0_is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_3P0),
	.vol_bit_shift	= 8,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 0,
	.min_voltage	= 2625000,
	.max_voltage	= 3400000,
};

/* CPU */
static struct regulator_consumer_supply vddcore_consumers[] = {
	{
		.supply = "cpu_vddgp",
	}
};
/* PU */
static struct regulator_consumer_supply vddpu_consumers[] = {
	{
		.supply = "cpu_vddvpu",
	},
	{
		.supply = "cpu_vddgpu",
	}
};
/* SOC */
static struct regulator_consumer_supply vddsoc_consumers[] = {
	{
		.supply = "cpu_vddsoc",
	},
};

/* USB phy 3P0 */
static struct regulator_consumer_supply vdd3p0_consumers[] = {
	{
		.supply = "cpu_vdd3p0",
	},
};

static struct regulator_init_data vddpu_init = {
	.constraints = {
		.name			= "vddpu",
		.min_uV			= 725000,
		.max_uV			= 1300000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE,
	},
	.num_consumer_supplies = ARRAY_SIZE(vddpu_consumers),
	.consumer_supplies = vddpu_consumers,
};

static struct regulator_init_data vddcore_init = {
	.constraints = {
		.name			= "vddcore",
		.min_uV			= 725000,
		.max_uV			= 1300000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(vddcore_consumers),
	.consumer_supplies = &vddcore_consumers[0],
};

static struct regulator_init_data vddsoc_init = {
	.constraints = {
		.name			= "vddsoc",
		.min_uV			= 725000,
		.max_uV			= 1300000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(vddsoc_consumers),
	.consumer_supplies = &vddsoc_consumers[0],
};

static struct regulator_init_data vdd2p5_init = {
	.constraints = {
		.name			= "vdd2p5",
		.min_uV			= 2000000,
		.max_uV			= 2775000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};


static struct regulator_init_data vdd1p1_init = {
	.constraints = {
		.name			= "vdd1p1",
		.min_uV			= 800000,
		.max_uV			= 1400000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.input_uV		= 5000000,
		.always_on		= 1,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};


static struct regulator_init_data vdd3p0_init = {
	.constraints = {
		.name			= "vdd3p0",
		.min_uV			= 2625000,
		.max_uV			= 3400000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE |
					  REGULATOR_CHANGE_STATUS,
		.always_on		= 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(vdd3p0_consumers),
	.consumer_supplies = &vdd3p0_consumers[0],
};

static struct anatop_regulator vddpu_reg = {
		.rdata = &vddpu_data,
};

static struct anatop_regulator vddcore_reg = {
		.rdata = &vddcore_data,
};

static struct anatop_regulator vddsoc_reg = {
		.rdata = &vddsoc_data,
};

static struct anatop_regulator vdd2p5_reg = {
		.rdata = &vdd2p5_data,
};

static struct anatop_regulator vdd1p1_reg = {
		.rdata = &vdd1p1_data,
};

static struct anatop_regulator vdd3p0_reg = {
		.rdata = &vdd3p0_data,
};

static int __init regulators_init(void)
{
	unsigned int reg;

	anatop_register_regulator(&vddpu_reg, ANATOP_VDDPU, &vddpu_init);
	anatop_register_regulator(&vddcore_reg, ANATOP_VDDCORE, &vddcore_init);
	anatop_register_regulator(&vddsoc_reg, ANATOP_VDDSOC, &vddsoc_init);
	anatop_register_regulator(&vdd2p5_reg, ANATOP_VDD2P5, &vdd2p5_init);
	anatop_register_regulator(&vdd1p1_reg, ANATOP_VDD1P1, &vdd1p1_init);
	anatop_register_regulator(&vdd3p0_reg, ANATOP_VDD3P0, &vdd3p0_init);

	/* Set the REGx step time back to reset value,
	 * as ROM may modify it according to fuse setting,
	 * so we need to set it back, otherwise, the delay
	 * time in cpu freq change will be impacted, the reset
	 * value is 0'b00, 64 cycles of 24M clock.
	 */
	reg = __raw_readl(ANADIG_MISC2_REG);
	reg &= ~ANADIG_ANA_MISC2_REG0_STEP_TIME_MASK;
	reg &= ~ANADIG_ANA_MISC2_REG1_STEP_TIME_MASK;
	reg &= ~ANADIG_ANA_MISC2_REG2_STEP_TIME_MASK;
	__raw_writel(reg, ANADIG_MISC2_REG);

	/* clear flag in boot */
	pu_is_enabled = 0;
	get_clk = 0;
	return 0;
}
postcore_initcall(regulators_init);
