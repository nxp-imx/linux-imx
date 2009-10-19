/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file bus_freq.c
 *
 * @brief A common API for the Freescale Semiconductor i.MXC CPUfreq module
 * and DVFS CORE module.
 *
 * The APIs are for setting bus frequency to low or high.
 *
 * @ingroup PM
 */

#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <linux/regulator/consumer.h>
#include <mach/mxc_dvfs.h>

#include "iomux.h"
#include "crm_regs.h"

#define GP_LPM_VOLTAGE 850000
#define LP_LPM_VOLTAGE 1050000
#define LP_LOWFREQ_VOLTAGE 1050000
#define LP_NORMAL_VOLTAGE 1200000

DEFINE_SPINLOCK(bus_freq_lock);

struct clk *main_bus_clk;
struct clk *pll2;
struct clk *pll1;
struct clk *axi_a_clk;
struct clk *axi_b_clk;
struct clk *axi_c_clk;
struct clk *emi_core_clk;
struct clk *emi_intr_clk;
struct clk *nfc_clk;
struct clk *ahb_clk;
struct clk *vpu_clk;
struct clk *vpu_core_clk;
struct clk *arm_axi_clk;
struct clk *ddr_clk;
struct clk *ipu_clk;
struct clk *periph_apm_clk;
struct clk *lp_apm;
struct clk *cpu_clk;
struct clk *osc;
struct clk *uart_clk;
struct regulator *lp_regulator;
int low_bus_freq_mode;
int high_bus_freq_mode;
char *gp_reg_id = "SW1";
char *lp_reg_id = "SW2";
static struct cpu_wp *cpu_wp_tbl;

struct dvfs_wp dvfs_core_setpoint[] = {{33, 7, 33, 10, 10, 0x10},
					    {22, 0, 33, 10, 10, 0x10},};

int set_low_bus_freq(void)
{
	int ret = 0;
	unsigned long lp_lpm_clk;
	unsigned long flags;

	struct clk *p_clk;
	struct clk *amode_parent_clk;

	if (low_bus_freq_mode)
		return ret;

	if (clk_get_rate(cpu_clk) != 200000000)
		return;

	clk_disable(uart_clk);

	lp_lpm_clk = clk_get_rate(lp_apm);
	amode_parent_clk = lp_apm;
	p_clk = clk_get_parent(periph_apm_clk);
	spin_lock_irqsave(&bus_freq_lock, flags);
	/* Make sure osc_clk is the parent of lp_apm. */
	if (clk_get_parent(amode_parent_clk) != osc)
		clk_set_parent(amode_parent_clk, osc);

	/* Set the parent of periph_apm_clk to be lp_apm */
	clk_set_parent(periph_apm_clk, amode_parent_clk);
	amode_parent_clk = periph_apm_clk;

	p_clk = clk_get_parent(main_bus_clk);
	/* Set the parent of main_bus_clk to be periph_apm_clk */
	clk_set_parent(main_bus_clk, amode_parent_clk);

	clk_set_rate(ahb_clk, lp_lpm_clk);
	/* Set the emi_internal clock to 24MHz */
	clk_set_rate(emi_intr_clk, lp_lpm_clk);
	if (clk_get_parent(emi_core_clk) != ahb_clk)
		clk_set_rate(emi_core_clk, lp_lpm_clk);

	if (clk_get_usecount(axi_a_clk) != 0)
		clk_set_rate(axi_a_clk, lp_lpm_clk);

	if (clk_get_usecount(axi_b_clk) != 0)
		clk_set_rate(axi_b_clk, lp_lpm_clk);

	if (clk_get_usecount(axi_c_clk) != 0)
		clk_set_rate(axi_c_clk, lp_lpm_clk);

	amode_parent_clk = emi_core_clk;

	p_clk = clk_get_parent(arm_axi_clk);
	if (p_clk != amode_parent_clk)
		clk_set_parent(arm_axi_clk, amode_parent_clk);

	p_clk = clk_get_parent(vpu_clk);
	if (p_clk != amode_parent_clk)
		clk_set_parent(vpu_clk, amode_parent_clk);

	p_clk = clk_get_parent(vpu_core_clk);
	if (p_clk != amode_parent_clk)
		clk_set_parent(vpu_core_clk, amode_parent_clk);
	spin_unlock_irqrestore(&bus_freq_lock, flags);

	/* Set the voltage to 1.05V for the LP domain. */
	ret = regulator_set_voltage(lp_regulator, 1050000, 1050000);
	udelay(100);
	if (ret < 0) {
		printk(KERN_ERR "COULD NOT SET LP VOLTAGE!!!!!!\n");
		return ret;
	}

	low_bus_freq_mode = 1;
	high_bus_freq_mode = 0;
	return ret;
}

int set_high_bus_freq(int high_bus_freq)
{
	struct clk *p_clk;
	struct clk *rmode_parent_clk;
	int ret = 0;
	unsigned long flags;

	if (!low_bus_freq_mode)
		return ret;

	low_bus_freq_mode = 0;

	/* Set the voltage to 1.25V for the LP domain. */
	ret = regulator_set_voltage(lp_regulator, 1250000, 1250000);
	udelay(100);
	if (ret < 0) {
		printk(KERN_ERR "COULD NOT SET LP VOLTAGE!!!!!!\n");
		return ret;
	}

	rmode_parent_clk = pll2;
	spin_lock_irqsave(&bus_freq_lock, flags);

	/* Set the dividers before setting the parent clock. */
	if (clk_get_usecount(axi_a_clk) != 0)
		clk_set_rate(axi_a_clk, 4800000);
	if (clk_get_usecount(axi_b_clk) != 0)
		clk_set_rate(axi_b_clk, 4000000);
	if (clk_get_usecount(axi_c_clk) != 0)
		clk_set_rate(axi_c_clk, 6000000);
	if (clk_get_parent(emi_core_clk) != ahb_clk)
		clk_set_rate(emi_core_clk, 4800000);

	clk_set_rate(ahb_clk, 4800000);
	/* Set emi_intr clock back to divide by 2. */
	clk_set_rate(emi_intr_clk, 2400000);
	/* Set the parent of main_bus_clk to be pll2 */
	clk_set_parent(main_bus_clk, rmode_parent_clk);
	spin_unlock_irqrestore(&bus_freq_lock, flags);

	clk_enable(uart_clk);
	high_bus_freq_mode = 1;
	return ret;
}

int low_freq_bus_used(void)
{
	if ((clk_get_usecount(ipu_clk) == 0)
	    && (clk_get_usecount(vpu_clk) == 0))
		return 1;
	else
		return 0;
}

void setup_pll(void)
{
	u32 reg;
	u32 hfsm;
	struct cpu_wp *p;

	/* Setup the DPLL registers */
	hfsm = __raw_readl(MXC_DPLL1_BASE + MXC_PLL_DP_CTL) &
	       MXC_PLL_DP_CTL_HFSM;
	reg = __raw_readl(MXC_DPLL1_BASE + MXC_PLL_DP_CONFIG);
	reg &= ~MXC_PLL_DP_CONFIG_AREN;
	__raw_writel(reg, MXC_DPLL1_BASE + MXC_PLL_DP_CONFIG);

	if (hfsm) {
		/* Running at lower frequency, need to bump up. */
		p = &cpu_wp_tbl[0];
		/* PDF and MFI */
		reg = p->pdf | p->mfi << MXC_PLL_DP_OP_MFI_OFFSET;
		__raw_writel(reg, MXC_DPLL1_BASE + MXC_PLL_DP_OP);

		/* MFD */
		__raw_writel(p->mfd, MXC_DPLL1_BASE + MXC_PLL_DP_MFD);

		/* MFI */
		__raw_writel(p->mfn, MXC_DPLL1_BASE + MXC_PLL_DP_MFN);
	} else {
		/* Running at high frequency, need to lower it. */
		p = &cpu_wp_tbl[1];
		/* PDF and MFI */
		reg = p->pdf | p->mfi << MXC_PLL_DP_OP_MFI_OFFSET;
		__raw_writel(reg, MXC_DPLL1_BASE + MXC_PLL_DP_HFS_OP);

		/* MFD */
		__raw_writel(p->mfd, MXC_DPLL1_BASE + MXC_PLL_DP_HFS_MFD);

		/* MFN */
		__raw_writel(p->mfn, MXC_DPLL1_BASE + MXC_PLL_DP_HFS_MFN);
	}

	/* Set PLL2_PODF to be 3 */
	reg = __raw_readl(MXC_CCM_CCSR);
	reg |= 2 << MXC_CCM_CCSR_PLL2_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CCSR);
	/* Set the parent of STEP_CLK to be PLL2 */
	reg = __raw_readl(MXC_CCM_CCSR);
	reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
	    (2 << MXC_CCM_CCSR_STEP_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CCSR);
}

/*!
 * This is the probe routine for the bus frequency driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */
static int __devinit busfreq_probe(struct platform_device *pdev)
{
	int cpu_wp_nr;

	main_bus_clk = clk_get(NULL, "main_bus_clk");
	if (IS_ERR(main_bus_clk)) {
		printk(KERN_DEBUG "%s: failed to get main_bus_clk\n", __func__);
		return PTR_ERR(main_bus_clk);
	}

	pll2 = clk_get(NULL, "pll2");
	if (IS_ERR(pll2)) {
		printk(KERN_DEBUG "%s: failed to get pll2\n", __func__);
		return PTR_ERR(pll2);
	}

	axi_a_clk = clk_get(NULL, "axi_a_clk");
	if (IS_ERR(axi_a_clk)) {
		printk(KERN_DEBUG "%s: failed to get axi_a_clk\n", __func__);
		return PTR_ERR(axi_a_clk);
	}

	axi_b_clk = clk_get(NULL, "axi_b_clk");
	if (IS_ERR(axi_b_clk)) {
		printk(KERN_DEBUG "%s: failed to get axi_b_clk\n", __func__);
		return PTR_ERR(axi_b_clk);
	}

	axi_c_clk = clk_get(NULL, "axi_c_clk");
	if (IS_ERR(axi_c_clk)) {
		printk(KERN_DEBUG "%s: failed to get axi_c_clk\n", __func__);
		return PTR_ERR(axi_c_clk);
	}

	emi_core_clk = clk_get(NULL, "emi_core_clk");
	if (IS_ERR(emi_core_clk)) {
		printk(KERN_DEBUG "%s: failed to get emi_core_clk\n", __func__);
		return PTR_ERR(emi_core_clk);
	}

	emi_intr_clk = clk_get(NULL, "emi_intr_clk");
	if (IS_ERR(emi_intr_clk)) {
		printk(KERN_DEBUG "%s: failed to get emi_intr_clk\n", __func__);
		return PTR_ERR(emi_intr_clk);
	}

	nfc_clk = clk_get(NULL, "nfc_clk");
	if (IS_ERR(nfc_clk)) {
		printk(KERN_DEBUG "%s: failed to get nfc_clk\n", __func__);
		return PTR_ERR(nfc_clk);
	}

	ahb_clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(ahb_clk)) {
		printk(KERN_DEBUG "%s: failed to get ahb_clk\n", __func__);
		return PTR_ERR(ahb_clk);
	}

	vpu_core_clk = clk_get(NULL, "vpu_core_clk");
	if (IS_ERR(vpu_core_clk)) {
		printk(KERN_DEBUG "%s: failed to get vpu_core_clk\n", __func__);
		return PTR_ERR(vpu_core_clk);
	}

	arm_axi_clk = clk_get(NULL, "arm_axi_clk");
	if (IS_ERR(arm_axi_clk)) {
		printk(KERN_DEBUG "%s: failed to get arm_axi_clk\n", __func__);
		return PTR_ERR(arm_axi_clk);
	}

	ddr_clk = clk_get(NULL, "ddr_clk");
	if (IS_ERR(ddr_clk)) {
		printk(KERN_DEBUG "%s: failed to get ddr_clk\n", __func__);
		return PTR_ERR(ddr_clk);
	}

	ipu_clk = clk_get(NULL, "ipu_clk");
	if (IS_ERR(ipu_clk)) {
		printk(KERN_DEBUG "%s: failed to get ipu_clk\n", __func__);
		return PTR_ERR(ipu_clk);
	}

	vpu_clk = clk_get(NULL, "vpu_clk");
	if (IS_ERR(vpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get vpu_clk\n", __func__);
		return PTR_ERR(vpu_clk);
	}

	periph_apm_clk = clk_get(NULL, "periph_apm_clk");
	if (IS_ERR(periph_apm_clk)) {
		printk(KERN_DEBUG "%s: failed to get periph_apm_clk\n",
		       __func__);
		return PTR_ERR(periph_apm_clk);
	}

	lp_apm = clk_get(NULL, "lp_apm");
	if (IS_ERR(lp_apm)) {
		printk(KERN_DEBUG "%s: failed to get lp_apm\n", __func__);
		return PTR_ERR(lp_apm);
	}

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n", __func__);
		return PTR_ERR(cpu_clk);
	}

	osc = clk_get(NULL, "osc");
	if (IS_ERR(osc)) {
		printk(KERN_DEBUG "%s: failed to get osc\n", __func__);
		return PTR_ERR(osc);
	}

	uart_clk = clk_get(NULL, "uart_clk.0");
	if (IS_ERR(uart_clk)) {
		printk(KERN_DEBUG "%s: failed to get uart_clk-0\n", __func__);
		return PTR_ERR(uart_clk);
	}

	pll1 = clk_get(NULL, "pll1_sw_clk");
	if (IS_ERR(pll1)) {
		printk(KERN_DEBUG "%s: failed to get pll1_sw_clk\n", __func__);
		return PTR_ERR(pll1);
	}


	lp_regulator = regulator_get(NULL, lp_reg_id);
	if (IS_ERR(lp_regulator)) {
		clk_put(ahb_clk);
		printk(KERN_DEBUG "%s: failed to get lp regulator\n", __func__);
		return PTR_ERR(lp_regulator);
	}

	low_bus_freq_mode = 0;
	high_bus_freq_mode = 0;

	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);

	return 0;
}

static struct platform_driver busfreq_driver = {
	.driver = {
		   .name = "busfreq",
		   },
	.probe = busfreq_probe,
};

/*!
 * Initialise the busfreq_driver.
 *
 * @return  The function always returns 0.
 */

static int __init busfreq_init(void)
{
	if (platform_driver_register(&busfreq_driver) != 0) {
		printk(KERN_ERR "busfreq_driver register failed\n");
		return -ENODEV;
	}

	printk(KERN_INFO "Bus freq driver module loaded\n");
	return 0;
}

static void __exit busfreq_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&busfreq_driver);

	clk_put(main_bus_clk);
	clk_put(pll2);
	clk_put(axi_a_clk);
	clk_put(axi_b_clk);
	clk_put(axi_c_clk);
	clk_put(emi_core_clk);
	clk_put(emi_intr_clk);
	clk_put(nfc_clk);
	clk_put(ahb_clk);
	clk_put(vpu_core_clk);
	clk_put(arm_axi_clk);
	clk_put(ddr_clk);
	clk_put(ipu_clk);
	clk_put(periph_apm_clk);
	clk_put(lp_apm);
	clk_put(osc);
	clk_put(pll1);
	clk_put(pll2);
	regulator_put(lp_regulator);

}

module_init(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("BusFreq driver");
MODULE_LICENSE("GPL");
