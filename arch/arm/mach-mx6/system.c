/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/clockchips.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#include <asm/hardware/gic.h>
#include "crm_regs.h"
#include "regs-anadig.h"

#define SCU_CTRL					0x00
#define SCU_CONFIG					0x04
#define SCU_CPU_STATUS				0x08
#define SCU_INVALIDATE				0x0c
#define SCU_FPGA_REVISION			0x10
#define GPC_CNTR_OFFSET				0x0
#define GPC_PGC_GPU_PGCR_OFFSET		0x260
#define GPC_PGC_CPU_PDN_OFFSET		0x2a0
#define GPC_PGC_CPU_PUPSCR_OFFSET	0x2a4
#define GPC_PGC_CPU_PDNSCR_OFFSET	0x2a8

#define MODULE_CLKGATE		(1 << 30)
#define MODULE_SFTRST		(1 << 31)

extern unsigned int gpc_wake_irq[4];

static void __iomem *gpc_base = IO_ADDRESS(GPC_BASE_ADDR);

int wait_mode_arm_podf;
volatile unsigned int num_cpu_idle;
volatile unsigned int num_cpu_idle_lock = 0x0;
int wait_mode_arm_podf;
int cur_arm_podf;
void arch_idle_with_workaround(int cpu);

extern void *mx6sl_wfi_iram_base;
extern void (*mx6sl_wfi_iram)(int arm_podf, unsigned long wfi_iram_addr);
extern void mx6_wait(void *num_cpu_idle_lock, void *num_cpu_idle, \
				int wait_arm_podf, int cur_arm_podf);
extern bool enable_wait_mode;
extern int low_bus_freq_mode;
extern int audio_bus_freq_mode;
extern bool mem_clk_on_in_wait;
extern int chip_rev;

void gpc_set_wakeup(unsigned int irq[4])
{
	/* Mask all wake up source */
	__raw_writel(~irq[0], gpc_base + 0x8);
	__raw_writel(~irq[1], gpc_base + 0xc);
	__raw_writel(~irq[2], gpc_base + 0x10);
	__raw_writel(~irq[3], gpc_base + 0x14);

	return;
}

/* set cpu low power mode before WFI instruction */
void mxc_cpu_lp_set(enum mxc_cpu_pwr_mode mode)
{

	int stop_mode = 0;
	void __iomem *anatop_base = IO_ADDRESS(ANATOP_BASE_ADDR);
	u32 ccm_clpcr, anatop_val, reg;

	ccm_clpcr = __raw_readl(MXC_CCM_CLPCR) & ~(MXC_CCM_CLPCR_LPM_MASK);

	switch (mode) {
	case WAIT_CLOCKED:
		break;
	case WAIT_UNCLOCKED:
		ccm_clpcr |= 0x1 << MXC_CCM_CLPCR_LPM_OFFSET;
		break;
	case WAIT_UNCLOCKED_POWER_OFF:
	case STOP_POWER_OFF:
	case ARM_POWER_OFF:
		if (mode == WAIT_UNCLOCKED_POWER_OFF) {
			ccm_clpcr &= ~MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr &= ~MXC_CCM_CLPCR_SBYOS;
			ccm_clpcr |= 0x1 << MXC_CCM_CLPCR_LPM_OFFSET;
			if (cpu_is_mx6sl()) {
				ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH0_LPM_HS;
				ccm_clpcr |= MXC_CCM_CLPCR_BYPASS_PMIC_VFUNC_READY;
			} else
				ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH1_LPM_HS;
			stop_mode = 0;
		} else if (mode == STOP_POWER_OFF) {
			ccm_clpcr |= 0x2 << MXC_CCM_CLPCR_LPM_OFFSET;
			ccm_clpcr |= 0x3 << MXC_CCM_CLPCR_STBY_COUNT_OFFSET;
			ccm_clpcr |= MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr |= MXC_CCM_CLPCR_SBYOS;
			if (cpu_is_mx6sl()) {
				ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH0_LPM_HS;
				ccm_clpcr |= MXC_CCM_CLPCR_BYPASS_PMIC_VFUNC_READY;
			} else
				ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH1_LPM_HS;
			stop_mode = 1;
		} else {
			ccm_clpcr |= 0x2 << MXC_CCM_CLPCR_LPM_OFFSET;
			ccm_clpcr |= 0x3 << MXC_CCM_CLPCR_STBY_COUNT_OFFSET;
			ccm_clpcr |= MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr |= MXC_CCM_CLPCR_SBYOS;
			if (cpu_is_mx6sl()) {
				ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH0_LPM_HS;
				ccm_clpcr |= MXC_CCM_CLPCR_BYPASS_PMIC_VFUNC_READY;
			} else
				ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH1_LPM_HS;
			stop_mode = 2;
		}
		break;
	case STOP_POWER_ON:
		ccm_clpcr |= 0x2 << MXC_CCM_CLPCR_LPM_OFFSET;

		break;
	default:
		printk(KERN_WARNING "UNKNOWN cpu power mode: %d\n", mode);
		return;
	}

	if (stop_mode > 0) {
		gpc_set_wakeup(gpc_wake_irq);
		/* Power down and power up sequence */
		__raw_writel(0xFFFFFFFF, gpc_base + GPC_PGC_CPU_PUPSCR_OFFSET);
		__raw_writel(0xFFFFFFFF, gpc_base + GPC_PGC_CPU_PDNSCR_OFFSET);
		if (stop_mode == 2) {
			/* dormant mode, need to power off the arm core */
			__raw_writel(0x1, gpc_base + GPC_PGC_CPU_PDN_OFFSET);
			if (cpu_is_mx6q() || cpu_is_mx6dl()) {
				/* If stop_mode_config is clear, then 2P5 will be off,
				need to enable weak 2P5, as DDR IO need 2P5 as
				pre-driver */
				if ((__raw_readl(anatop_base + HW_ANADIG_ANA_MISC0)
					& BM_ANADIG_ANA_MISC0_STOP_MODE_CONFIG) == 0) {
					/* Enable weak 2P5 linear regulator */
					anatop_val = __raw_readl(anatop_base +
						HW_ANADIG_REG_2P5);
					anatop_val |= BM_ANADIG_REG_2P5_ENABLE_WEAK_LINREG;
					__raw_writel(anatop_val, anatop_base +
						HW_ANADIG_REG_2P5);
				}
				if (mx6q_revision() != IMX_CHIP_REVISION_1_0) {
					/* Enable fet_odrive */
					anatop_val = __raw_readl(anatop_base +
						HW_ANADIG_REG_CORE);
					anatop_val |= BM_ANADIG_REG_CORE_FET_ODRIVE;
					__raw_writel(anatop_val, anatop_base +
						HW_ANADIG_REG_CORE);
				}
			} else {
				/* Disable VDDHIGH_IN to VDDSNVS_IN power path,
				 * only used when VDDSNVS_IN is powered by dedicated
				 * power rail */
				anatop_val = __raw_readl(anatop_base +
					HW_ANADIG_ANA_MISC0);
				anatop_val |= BM_ANADIG_ANA_MISC0_RTC_RINGOSC_EN;
				__raw_writel(anatop_val, anatop_base +
					HW_ANADIG_ANA_MISC0);
				/* We need to allow the memories to be clock gated
				 * in STOP mode, else the power consumption will
				 * be very high. */
				reg = __raw_readl(MXC_CCM_CGPR);
				reg |= MXC_CCM_CGPR_MEM_IPG_STOP_MASK;
				__raw_writel(reg, MXC_CCM_CGPR);
			}
			/* DL's TO1.0 can't support DSM mode due to ipg glitch */
			if (mx6dl_revision() != IMX_CHIP_REVISION_1_0)
				__raw_writel(__raw_readl(MXC_CCM_CCR) |
					MXC_CCM_CCR_RBC_EN, MXC_CCM_CCR);

			/* Make sure we clear WB_COUNT and re-config it */
			__raw_writel(__raw_readl(MXC_CCM_CCR) &
				(~MXC_CCM_CCR_WB_COUNT_MASK) &
				(~MXC_CCM_CCR_REG_BYPASS_CNT_MASK), MXC_CCM_CCR);
			udelay(60);
			/* Reconfigurate WB and RBC counter */
			__raw_writel(__raw_readl(MXC_CCM_CCR) |
				(0x1 << MXC_CCM_CCR_WB_COUNT_OFFSET) |
				(0x20 << MXC_CCM_CCR_REG_BYPASS_CNT_OFFSET), MXC_CCM_CCR);

			/* Set WB_PER enable */
			ccm_clpcr |= MXC_CCM_CLPCR_WB_PER_AT_LPM;
		}
		if (cpu_is_mx6sl() ||
			(mx6q_revision() > IMX_CHIP_REVISION_1_1) ||
			(mx6dl_revision() > IMX_CHIP_REVISION_1_0)) {
			u32 reg;
			/* We need to allow the memories to be clock gated
			  * in STOP mode, else the power consumption will
			  * be very high.
			  */
			reg = __raw_readl(MXC_CCM_CGPR);
			reg |= MXC_CCM_CGPR_MEM_IPG_STOP_MASK;
			if (!cpu_is_mx6sl()) {
				/*
				  * For MX6QTO1.2 or later and MX6DLTO1.1 or later,
				  * ensure that the CCM_CGPR bit 17 is cleared before
				  * dormant mode is entered.
				  */
				reg &= ~MXC_CCM_CGPR_WAIT_MODE_FIX;
			}
			__raw_writel(reg, MXC_CCM_CGPR);
		}
	}
	__raw_writel(ccm_clpcr, MXC_CCM_CLPCR);
}

extern int tick_broadcast_oneshot_active(void);

void ca9_do_idle(void)
{
	do {
		cpu_do_idle();
	} while (__raw_readl(gic_cpu_base_addr + GIC_CPU_HIGHPRI) == 1023);
}

void arch_idle_single_core(void)
{
	u32 reg;

	if (cpu_is_mx6dl() && chip_rev > IMX_CHIP_REVISION_1_0) {
		/*
		  * MX6DLS TO1.1 has the HW fix for the WAIT mode issue.
		  * Ensure that the CGPR bit 17 is set to enable the fix.
		  */
		reg = __raw_readl(MXC_CCM_CGPR);
		reg |= MXC_CCM_CGPR_WAIT_MODE_FIX;
		__raw_writel(reg, MXC_CCM_CGPR);

		ca9_do_idle();
	} else {
		if (low_bus_freq_mode && cpu_is_mx6sl()) {
			u32 org_arm_podf = __raw_readl(MXC_CCM_CACRR);

			/* Need to run WFI code from IRAM so that
			  * we can lower DDR freq.
			  */
			mx6sl_wfi_iram(org_arm_podf,
				(unsigned long)mx6sl_wfi_iram_base);

			/* Clear the chicken bit to allow memories
			  * to be powered down
			  */
		} else {
			/*
			  * Implement the 12:5 ARM:IPG_CLK ratio
			  * workaround for the WAIT mode issue.
			  * We can directly use the divider to drop the ARM
			  * core freq in a single core environment.
			  *  Set the ARM_PODF to get the max freq possible
			  * to avoid the WAIT mode issue when IPG is at 66MHz.
			  */
			__raw_writel(wait_mode_arm_podf, MXC_CCM_CACRR);
			while (__raw_readl(MXC_CCM_CDHIPR))
				;
			ca9_do_idle();

			__raw_writel(cur_arm_podf - 1, MXC_CCM_CACRR);
		}
	}
}

void arch_idle_with_workaround(cpu)
{
	u32 podf = wait_mode_arm_podf;

	*((char *)(&num_cpu_idle_lock) + (char)cpu) = 0x0;

	if (low_bus_freq_mode || audio_bus_freq_mode)
		/* In case when IPG is at 12MHz, we need to ensure that
		  * ARM is at 24MHz, as the max freq ARM can run at is
		  *~28.8MHz.
		  */
		podf = 0;

	mx6_wait((void *)&num_cpu_idle_lock,
		(void *)&num_cpu_idle,
		podf, cur_arm_podf - 1);

}

void arch_idle_multi_core(void)
{
	u32 reg;
	int cpu = smp_processor_id();

#ifdef CONFIG_LOCAL_TIMERS
	if (!tick_broadcast_oneshot_active()
		|| !tick_oneshot_mode_active())
		return;

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu);
#endif
	/* iMX6Q and iMX6DL */
	if ((cpu_is_mx6q() && chip_rev >= IMX_CHIP_REVISION_1_2) ||
		(cpu_is_mx6dl() && chip_rev >= IMX_CHIP_REVISION_1_1)) {
		/*
		  * This code should only be executed on MX6QTO1.2 or later
		  * and MX6DL TO1.1 or later.
		  * These chips have the HW fix for the WAIT mode issue.
		  * Ensure that the CGPR bit 17 is set to enable the fix.
		  */

		reg = __raw_readl(MXC_CCM_CGPR);
		reg |= MXC_CCM_CGPR_WAIT_MODE_FIX;
		__raw_writel(reg, MXC_CCM_CGPR);

		ca9_do_idle();
	} else
		arch_idle_with_workaround(cpu);
#ifdef CONFIG_LOCAL_TIMERS
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu);
#endif

}

void arch_idle(void)
{
	if (enable_wait_mode) {
		mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
		if (mem_clk_on_in_wait) {
			u32 reg;
			/*
			  * MX6SL, MX6Q (TO1.2 or later) and
			  * MX6DL (TO1.1 or later) have a bit in CCM_CGPR that
			  * when cleared keeps the clocks to memories ON
			  * when ARM is in WFI. This mode can be used when
			  * IPG clock is very low (12MHz) and the ARM:IPG ratio
			  * perhaps cannot be maintained.
			  */
			reg = __raw_readl(MXC_CCM_CGPR);
			reg &= ~MXC_CCM_CGPR_MEM_IPG_STOP_MASK;
			__raw_writel(reg, MXC_CCM_CGPR);

			ca9_do_idle();
		} else if (num_possible_cpus() == 1)
			/* iMX6SL or iMX6DLS */
			arch_idle_single_core();
		else
			arch_idle_multi_core();
	} else {
		mxc_cpu_lp_set(WAIT_CLOCKED);
		ca9_do_idle();
	}
}

static int __mxs_reset_block(void __iomem *hwreg, int just_enable)
{
	u32 c;
	int timeout;

	/* the process of software reset of IP block is done
	   in several steps:

	   - clear SFTRST and wait for block is enabled;
	   - clear clock gating (CLKGATE bit);
	   - set the SFTRST again and wait for block is in reset;
	   - clear SFTRST and wait for reset completion.
	 */
	c = __raw_readl(hwreg);
	c &= ~MODULE_SFTRST;	/* clear SFTRST */
	__raw_writel(c, hwreg);
	for (timeout = 1000000; timeout > 0; timeout--)
		/* still in SFTRST state ? */
		if ((__raw_readl(hwreg) & MODULE_SFTRST) == 0)
			break;
	if (timeout <= 0) {
		printk(KERN_ERR "%s(%p): timeout when enabling\n",
		       __func__, hwreg);
		return -ETIME;
	}

	c = __raw_readl(hwreg);
	c &= ~MODULE_CLKGATE;	/* clear CLKGATE */
	__raw_writel(c, hwreg);

	if (!just_enable) {
		c = __raw_readl(hwreg);
		c |= MODULE_SFTRST;	/* now again set SFTRST */
		__raw_writel(c, hwreg);
		for (timeout = 1000000; timeout > 0; timeout--)
			/* poll until CLKGATE set */
			if (__raw_readl(hwreg) & MODULE_CLKGATE)
				break;
		if (timeout <= 0) {
			printk(KERN_ERR "%s(%p): timeout when resetting\n",
			       __func__, hwreg);
			return -ETIME;
		}

		c = __raw_readl(hwreg);
		c &= ~MODULE_SFTRST;	/* clear SFTRST */
		__raw_writel(c, hwreg);
		for (timeout = 1000000; timeout > 0; timeout--)
			/* still in SFTRST state ? */
			if ((__raw_readl(hwreg) & MODULE_SFTRST) == 0)
				break;
		if (timeout <= 0) {
			printk(KERN_ERR "%s(%p): timeout when enabling "
			       "after reset\n", __func__, hwreg);
			return -ETIME;
		}

		c = __raw_readl(hwreg);
		c &= ~MODULE_CLKGATE;	/* clear CLKGATE */
		__raw_writel(c, hwreg);
	}
	for (timeout = 1000000; timeout > 0; timeout--)
		/* still in SFTRST state ? */
		if ((__raw_readl(hwreg) & MODULE_CLKGATE) == 0)
			break;

	if (timeout <= 0) {
		printk(KERN_ERR "%s(%p): timeout when unclockgating\n",
		       __func__, hwreg);
		return -ETIME;
	}

	return 0;
}

static int _mxs_reset_block(void __iomem *hwreg, int just_enable)
{
	int try = 10;
	int r;

	while (try--) {
		r = __mxs_reset_block(hwreg, just_enable);
		if (!r)
			break;
		pr_debug("%s: try %d failed\n", __func__, 10 - try);
	}
	return r;
}


#define BOOT_MODE_SERIAL_ROM			(0x00000030)
#define PERSIST_WATCHDOG_RESET_BOOT		(0x10000000)
/*BOOT_CFG1[7..4] = 0x3 Boot from Serial ROM (I2C/SPI)*/

#ifdef CONFIG_MXC_REBOOT_MFGMODE
void do_switch_mfgmode(void)
{
	u32 reg;

	/*
	 * During reset, if GPR10[28] is 1, ROM will copy GPR9[25:0]
	 * to SBMR1, which will determine what is the boot device.
	 * Here SERIAL_ROM mode is selected
	 */
	reg = __raw_readl(SRC_BASE_ADDR + SRC_GPR9);
	reg |= BOOT_MODE_SERIAL_ROM;
	__raw_writel(reg, SRC_BASE_ADDR + SRC_GPR9);

	reg = __raw_readl(SRC_BASE_ADDR + SRC_GPR10);
	reg |= PERSIST_WATCHDOG_RESET_BOOT;
	__raw_writel(reg, SRC_BASE_ADDR + SRC_GPR10);

}

void mxc_clear_mfgmode(void)
{
	u32 reg;
	reg = __raw_readl(SRC_BASE_ADDR + SRC_GPR9);

	reg &= ~BOOT_MODE_SERIAL_ROM;
	__raw_writel(reg, SRC_BASE_ADDR + SRC_GPR9);

	reg = __raw_readl(SRC_BASE_ADDR + SRC_GPR10);
	reg &= ~PERSIST_WATCHDOG_RESET_BOOT;
	__raw_writel(reg, SRC_BASE_ADDR + SRC_GPR10);
}
#endif

#ifdef CONFIG_MXC_REBOOT_ANDROID_CMD
/* This function will set a bit on SRC_GPR10[7-8] bits to enter
 * special boot mode.  These bits will not clear by watchdog reset, so
 * it can be checked by bootloader to choose enter different mode.*/

#define ANDROID_RECOVERY_BOOT  (1 << 7)
#define ANDROID_FASTBOOT_BOOT  (1 << 8)

void do_switch_recovery(void)
{
	u32 reg;

	reg = __raw_readl(SRC_BASE_ADDR + SRC_GPR10);
	reg |= ANDROID_RECOVERY_BOOT;
	__raw_writel(reg, SRC_BASE_ADDR + SRC_GPR10);
}

void do_switch_fastboot(void)
{
	u32 reg;

	reg = __raw_readl(SRC_BASE_ADDR + SRC_GPR10);
	reg |= ANDROID_FASTBOOT_BOOT;
	__raw_writel(reg, SRC_BASE_ADDR + SRC_GPR10);
}
#endif

int mxs_reset_block(void __iomem *hwreg)
{
	return _mxs_reset_block(hwreg, false);
}
EXPORT_SYMBOL(mxs_reset_block);
