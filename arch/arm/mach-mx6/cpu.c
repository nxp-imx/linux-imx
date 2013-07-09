/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/iram_alloc.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/system.h>
#include <asm/io.h>
#include <asm/mach/map.h>

#include "crm_regs.h"
#include "cpu_op-mx6.h"

struct cpu_op *(*get_cpu_op)(int *op);
bool enable_wait_mode = true;
u32 enable_ldo_mode = LDO_MODE_DEFAULT;
u32 arm_max_freq = CPU_AT_1_2GHz;
bool mem_clk_on_in_wait;
bool enet_to_gpio_6;
int chip_rev;

void __iomem *gpc_base;
void __iomem *ccm_base;

extern unsigned int num_cpu_idle_lock;

static int cpu_silicon_rev = -1;
#define MX6_USB_ANALOG_DIGPROG  0x260
#define MX6SL_USB_ANALOG_DIGPROG  0x280

static int mx6_get_srev(void)
{
	void __iomem *anatop = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	u32 rev;
	if (cpu_is_mx6sl())
		rev = __raw_readl(anatop + MX6SL_USB_ANALOG_DIGPROG);
	else
		rev = __raw_readl(anatop + MX6_USB_ANALOG_DIGPROG);

	rev &= 0xff;

	if (rev == 0)
		return IMX_CHIP_REVISION_1_0;
	else if (rev == 1)
		return IMX_CHIP_REVISION_1_1;
	else if (rev == 2)
		return IMX_CHIP_REVISION_1_2;

	return IMX_CHIP_REVISION_UNKNOWN;
}

/*
 * Returns:
 *	the silicon revision of the cpu
 */
int mx6q_revision(void)
{
	if (!cpu_is_mx6q())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = mx6_get_srev();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx6q_revision);

/*
 * Returns:
 *	the silicon revision of the cpu
 */
int mx6dl_revision(void)
{
	if (!cpu_is_mx6dl())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = mx6_get_srev();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx6dl_revision);

/*
 * Returns:
 *	the silicon revision of the cpu
 */
int mx6sl_revision(void)
{
	if (!cpu_is_mx6sl())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = mx6_get_srev();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx6sl_revision);

static int __init post_cpu_init(void)
{
	unsigned int reg;
	void __iomem *base;
	u32	iram_size;

	if (cpu_is_mx6q())
		iram_size = MX6Q_IRAM_SIZE;
	else
		iram_size = MX6DL_MX6SL_IRAM_SIZE;

	iram_init(MX6Q_IRAM_BASE_ADDR, iram_size);

	base = ioremap(AIPS1_ON_BASE_ADDR, PAGE_SIZE);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);
	iounmap(base);

	base = ioremap(AIPS2_ON_BASE_ADDR, PAGE_SIZE);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);
	iounmap(base);

	/* Allow SCU_CLK to be disabled when all cores are in WFI*/
	base = IO_ADDRESS(SCU_BASE_ADDR);
	reg = __raw_readl(base);
	reg |= 0x20;
	__raw_writel(reg, base);

	/* Disable SRC warm reset to work aound system reboot issue */
	base = IO_ADDRESS(SRC_BASE_ADDR);
	reg = __raw_readl(base);
	reg &= ~0x1;
	__raw_writel(reg, base);

	gpc_base = MX6_IO_ADDRESS(GPC_BASE_ADDR);
	ccm_base = MX6_IO_ADDRESS(CCM_BASE_ADDR);

	/* enable AXI cache for VDOA/VPU/IPU
	 * set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7
	 * clear OCRAM_CTL bits to disable pipeline control
	 */
	reg = __raw_readl(IOMUXC_GPR3);
	reg &= ~IOMUXC_GPR3_OCRAM_CTL_EN;
	__raw_writel(reg, IOMUXC_GPR3);
	reg = __raw_readl(IOMUXC_GPR4);
	reg |= IOMUXC_GPR4_VDOA_CACHE_EN | IOMUXC_GPR4_VPU_CACHE_EN |
		IOMUXC_GPR4_IPU_CACHE_EN;
	__raw_writel(reg, IOMUXC_GPR4);
	__raw_writel(IOMUXC_GPR6_IPU1_QOS, IOMUXC_GPR6);
	__raw_writel(IOMUXC_GPR7_IPU2_QOS, IOMUXC_GPR7);

	num_cpu_idle_lock = 0x0;
	if (cpu_is_mx6dl())
		num_cpu_idle_lock = 0xffff0000;

#ifdef CONFIG_SMP
	switch (setup_max_cpus) {
	case 3:
		num_cpu_idle_lock = 0xff000000;
		break;
	case 2:
		num_cpu_idle_lock = 0xffff0000;
		break;
	case 1:
	case 0:
		num_cpu_idle_lock = 0xffffff00;
		break;
	}
#endif
	/*
	  * The option to keep ARM memory clocks enabled during WAIT
	  * is only available on MX6SL, MX6DQ TO1.2  (or later) and
	  * MX6DL TO1.1 (or later)
	  * So if the user specifies "mem_clk_on" on any other chip,
	  * ensure that it is disabled.
	  */
	if (!cpu_is_mx6sl() && (mx6q_revision() < IMX_CHIP_REVISION_1_2) &&
		(mx6dl_revision() < IMX_CHIP_REVISION_1_1))
		mem_clk_on_in_wait = false;

	if (cpu_is_mx6q())
		chip_rev = mx6q_revision();
	else if (cpu_is_mx6dl())
		chip_rev = mx6dl_revision();
	else
		chip_rev = mx6sl_revision();

	/* mx6sl doesn't have pcie. save power, disable PCIe PHY */
	if (!cpu_is_mx6sl()) {
		reg = __raw_readl(IOMUXC_GPR1);
		reg = reg & (~IOMUXC_GPR1_PCIE_REF_CLK_EN);
		reg |= IOMUXC_GPR1_TEST_POWERDOWN;
		__raw_writel(reg, IOMUXC_GPR1);
	}
	return 0;
}
postcore_initcall(post_cpu_init);

static int __init enable_wait(char *p)
{
	if (memcmp(p, "on", 2) == 0) {
		enable_wait_mode = true;
		p += 2;
	} else if (memcmp(p, "off", 3) == 0) {
		enable_wait_mode = false;
		p += 3;
	}
	return 0;
}
early_param("enable_wait_mode", enable_wait);

static int __init arm_core_max(char *p)
{
	if (memcmp(p, "1200", 4) == 0) {
		arm_max_freq = CPU_AT_1_2GHz;
		p += 4;
	} else if (memcmp(p, "1000", 4) == 0) {
		arm_max_freq = CPU_AT_1GHz;
		p += 4;
	} else if (memcmp(p, "800", 3) == 0) {
		arm_max_freq = CPU_AT_800MHz;
		p += 3;
	}
	return 0;
}

early_param("arm_freq", arm_core_max);

static int __init enable_ldo(char *p)
{
	if (memcmp(p, "on", 2) == 0) {
		enable_ldo_mode = LDO_MODE_ENABLED;
		p += 2;
	} else if (memcmp(p, "off", 3) == 0) {
		enable_ldo_mode = LDO_MODE_BYPASSED;
		p += 3;
	}
	return 0;
}
early_param("ldo_active", enable_ldo);

static int __init enable_mem_clk_in_wait(char *p)
{
	mem_clk_on_in_wait = true;

	return 0;
}

early_param("mem_clk_on", enable_mem_clk_in_wait);

static int __init set_enet_irq_to_gpio(char *p)
{
	enet_to_gpio_6 = true;
	return 0;
}

early_param("enet_gpio_6", set_enet_irq_to_gpio);
