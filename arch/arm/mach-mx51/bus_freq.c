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
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include <mach/sdram_autogating.h>
#include "crm_regs.h"

#define LP_NORMAL_CLK			133000000
#define LP_MED_CLK			83125000
#define LP_APM_CLK   			24000000
#define NAND_LP_APM_CLK			12000000
#define DDR_LOW_FREQ_CLK		133000000
#define DDR_NORMAL_CLK			200000000
#define AXI_A_NORMAL_CLK		166250000
#define AXI_A_CLK_NORMAL_DIV		4
#define AXI_B_CLK_NORMAL_DIV		5
#define AHB_CLK_NORMAL_DIV		AXI_B_CLK_NORMAL_DIV
#define EMI_SLOW_CLK_NORMAL_DIV		AXI_B_CLK_NORMAL_DIV
#define NFC_CLK_NORMAL_DIV      	4

static struct clk *ddr_clk;
static struct clk *pll2;
static struct clk *main_bus_clk;
static struct clk *axi_a_clk;
static struct clk *axi_b_clk;
static struct clk *cpu_clk;
static struct clk *ddr_hf_clk;
static struct clk *nfc_clk;
static struct clk *ahb_clk;
static struct clk *vpu_clk;
static struct clk *vpu_core_clk;
static struct clk *emi_slow_clk;
static struct clk *ddr_clk;
static struct clk *ipu_clk;
static struct clk *periph_apm_clk;
static struct clk *lp_apm;
static struct clk *osc;
static struct clk *gpc_dvfs_clk;
static struct clk *mipi_hsp_clk;
struct regulator *lp_regulator;
int low_bus_freq_mode;
int high_bus_freq_mode;
int bus_freq_scaling_initialized;
char *gp_reg_id = "SW1";
char *lp_reg_id = "SW2";

static struct cpu_wp *cpu_wp_tbl;
static struct device *busfreq_dev;
static int busfreq_suspended;
/* True if bus_frequency is scaled not using DVFS-PER */
int bus_freq_scaling_is_active;

extern int lp_high_freq;
extern int lp_med_freq;
extern int dvfs_core_is_active;
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern int cpu_wp_nr;

struct dvfs_wp dvfs_core_setpoint[] = {
						{33, 8, 33, 10, 10, 0x08},
						{26, 0, 33, 20, 10, 0x08},
						{28, 8, 33, 20, 30, 0x08},
						{29, 0, 33, 20, 10, 0x08},};

#define DISABLE_PLL1

int set_low_bus_freq(void)
{
	struct clk *tclk;
	u32 reg;

	if (busfreq_suspended)
		return 0;

	if (bus_freq_scaling_initialized) {
		if (clk_get_rate(cpu_clk) != cpu_wp_tbl[cpu_wp_nr - 1].cpu_rate)
			return 0;

		stop_dvfs_per();

		stop_sdram_autogating();
#ifdef DISABLE_PLL1
		tclk = clk_get(NULL, "ddr_clk");
		clk_set_parent(tclk, clk_get(NULL, "axi_a_clk"));

		/* Set CPU clock to be derived from PLL2 instead of PLL1 */
		tclk = clk_get(NULL, "pll1_sw_clk");
		clk_set_parent(tclk, clk_get(NULL, "pll2"));
		clk_enable(tclk);

		tclk = clk_get(NULL, "ddr_clk");
		clk_set_parent(tclk, clk_get(NULL, "ddr_hf_clk"));
#endif

		/*Change the DDR freq to 133Mhz. */
		clk_set_rate(ddr_hf_clk,
			     clk_round_rate(ddr_hf_clk, DDR_LOW_FREQ_CLK));

		clk_enable(gpc_dvfs_clk);

		/* Setup the GPC. */
		reg = __raw_readl(MXC_GPC_VCR);
		reg &= ~(MXC_GPCVCR_VINC_MASK | MXC_GPCVCR_VCNTU_MASK |
			 MXC_GPCVCR_VCNT_MASK);

		reg |= (1 << MXC_GPCVCR_VCNTU_OFFSET) |
			(0 << MXC_GPCVCR_VCNT_OFFSET);
		__raw_writel(reg, MXC_GPC_VCR);

		reg = __raw_readl(MXC_GPC_CNTR);
		reg &= ~(MXC_GPCCNTR_ADU_MASK | MXC_GPCCNTR_FUPD_MASK);
		reg |= MXC_GPCCNTR_FUPD;
		__raw_writel(reg, MXC_GPC_CNTR);

		/* Enable DVFS-PER */
		reg = __raw_readl(MXC_DVFSPER_PMCR0);
		reg &= ~(MXC_DVFSPER_PMCR0_UDCS_MASK |
				MXC_DVFSPER_PMCR0_ENABLE_MASK);
		reg |= MXC_DVFSPER_PMCR0_ENABLE;
		__raw_writel(reg, MXC_DVFSPER_PMCR0);

		/* Set the dvfs-podf to divide by 4. */
		reg = __raw_readl(MXC_CCM_CDCR);
		reg &= ~MXC_CCM_CDCR_PERIPH_CLK_DVFS_PODF_MASK;
		reg |= 3 << MXC_CCM_CDCR_PERIPH_CLK_DVFS_PODF_OFFSET;
		__raw_writel(reg, MXC_CCM_CDCR);

		/* Setup the GPC */
		reg = __raw_readl(MXC_GPC_VCR) & ~MXC_GPCVCR_VINC_MASK;
		/* Set VINC to 0. */
		reg |= 0 << MXC_GPCVCR_VINC_OFFSET;
		__raw_writel(reg, MXC_GPC_VCR);

		reg = __raw_readl(MXC_GPC_CNTR);
		reg |= MXC_GPCCNTR_STRT;
		__raw_writel(reg, MXC_GPC_CNTR);
		while (__raw_readl(MXC_GPC_CNTR) & MXC_GPCCNTR_STRT)
			udelay(10);

		/* Disable DVFS-PER */
		reg = __raw_readl(MXC_DVFSPER_PMCR0);
		reg &= ~MXC_DVFSPER_PMCR0_ENABLE;
		__raw_writel(reg, MXC_DVFSPER_PMCR0);

		/* Set the dividers to be  close to 24Mhz from 166.25MHz*/
		reg = __raw_readl(MXC_CCM_CBCDR);
		reg &= ~(MXC_CCM_CBCDR_AXI_A_PODF_MASK
				| MXC_CCM_CBCDR_AXI_B_PODF_MASK
				| MXC_CCM_CBCDR_AHB_PODF_MASK
				| MXC_CCM_CBCDR_EMI_PODF_MASK
				| MXC_CCM_CBCDR_NFC_PODF_OFFSET);
		reg |= (6 << MXC_CCM_CBCDR_AXI_A_PODF_OFFSET
				| 6 << MXC_CCM_CBCDR_AXI_B_PODF_OFFSET
				| 6 << MXC_CCM_CBCDR_AHB_PODF_OFFSET
				| 6 << MXC_CCM_CBCDR_EMI_PODF_OFFSET
				| 3 << MXC_CCM_CBCDR_NFC_PODF_OFFSET);
		__raw_writel(reg, MXC_CCM_CBCDR);

		while (__raw_readl(MXC_CCM_CDHIPR) & 0x1F)
			udelay(10);

		low_bus_freq_mode = 1;
		high_bus_freq_mode = 0;
		clk_set_parent(main_bus_clk, pll2);
	}
	return 0;
}

int set_high_bus_freq(int high_bus_freq)
{
	u32 reg;
	struct clk *tclk;

	if (bus_freq_scaling_initialized) {
		stop_sdram_autogating();

		if (low_bus_freq_mode) {
			reg = __raw_readl(MXC_CCM_CBCDR);
			reg &= ~(MXC_CCM_CBCDR_AXI_A_PODF_MASK
					| MXC_CCM_CBCDR_AXI_B_PODF_MASK
					| MXC_CCM_CBCDR_AHB_PODF_MASK
					| MXC_CCM_CBCDR_EMI_PODF_MASK
					| MXC_CCM_CBCDR_NFC_PODF_OFFSET);
			reg |= (3 << MXC_CCM_CBCDR_AXI_A_PODF_OFFSET
					| 4 << MXC_CCM_CBCDR_AXI_B_PODF_OFFSET
					| 4 << MXC_CCM_CBCDR_AHB_PODF_OFFSET
					| 4 << MXC_CCM_CBCDR_EMI_PODF_OFFSET
					| 3 << MXC_CCM_CBCDR_NFC_PODF_OFFSET);
			__raw_writel(reg, MXC_CCM_CBCDR);
			while (__raw_readl(MXC_CCM_CDHIPR) & 0x1F)
				udelay(10);

			/* Setup the GPC. */
			reg = __raw_readl(MXC_GPC_VCR);
			reg &= ~(MXC_GPCVCR_VINC_MASK | MXC_GPCVCR_VCNTU_MASK |
				 MXC_GPCVCR_VCNT_MASK);

			reg |= (1 << MXC_GPCVCR_VCNTU_OFFSET) |
				(0 << MXC_GPCVCR_VCNT_OFFSET);
			__raw_writel(reg, MXC_GPC_VCR);

			reg = __raw_readl(MXC_GPC_CNTR);
			reg &= ~(MXC_GPCCNTR_ADU_MASK | MXC_GPCCNTR_FUPD_MASK);
			reg |= MXC_GPCCNTR_FUPD;
			__raw_writel(reg, MXC_GPC_CNTR);

			/* Enable DVFS-PER */
			reg = __raw_readl(MXC_DVFSPER_PMCR0);
			reg |= MXC_DVFSPER_PMCR0_UDCS;
			reg |= MXC_DVFSPER_PMCR0_ENABLE;
			__raw_writel(reg, MXC_DVFSPER_PMCR0);

			/* Set the dvfs-podf to divide by 1. */
			reg = __raw_readl(MXC_CCM_CDCR);
			reg &= ~MXC_CCM_CDCR_PERIPH_CLK_DVFS_PODF_MASK;
			reg |= 1 << MXC_CCM_CDCR_PERIPH_CLK_DVFS_PODF_OFFSET;
			__raw_writel(reg, MXC_CCM_CDCR);

			/* Setup the GPC */
			reg = __raw_readl(MXC_GPC_VCR) & ~MXC_GPCVCR_VINC_MASK;
			/* Set VINC to 1. */
			reg |= 1 << MXC_GPCVCR_VINC_OFFSET;
			__raw_writel(reg, MXC_GPC_VCR);

			reg = __raw_readl(MXC_GPC_CNTR);
			reg |= MXC_GPCCNTR_STRT;
			__raw_writel(reg, MXC_GPC_CNTR);
			while (__raw_readl(MXC_GPC_CNTR) & MXC_GPCCNTR_STRT)
				udelay(10);

			/* Disable DVFS-PER */
			reg = __raw_readl(MXC_DVFSPER_PMCR0);
			reg &= ~MXC_DVFSPER_PMCR0_ENABLE;
			__raw_writel(reg, MXC_DVFSPER_PMCR0);

			low_bus_freq_mode = 0;
			clk_set_parent(main_bus_clk, pll2);
			clk_disable(gpc_dvfs_clk);
#ifdef DISABLE_PLL1
			tclk = clk_get(NULL, "ddr_clk");
			clk_set_parent(tclk, clk_get(NULL, "axi_a_clk"));

			/* Set CPU clock to be derived from PLL1 instead of PLL2 */
			tclk = clk_get(NULL, "pll1_sw_clk");
			clk_set_parent(tclk, clk_get(NULL, "pll1_main_clk"));
			clk_disable(tclk);

			tclk = clk_get(NULL, "ddr_clk");
			clk_set_parent(tclk, clk_get(NULL, "ddr_hf_clk"));
#endif
			/*Change the DDR freq to 200MHz*/
			clk_set_rate(ddr_hf_clk,
				    clk_round_rate(ddr_hf_clk, DDR_NORMAL_CLK));

			start_dvfs_per();
		}
	if (bus_freq_scaling_is_active) {
		/*
		 * If the CPU freq is 800MHz, set the bus to the high setpoint
		 * (133MHz) and DDR to 200MHz.
		 */
		if (clk_get_rate(cpu_clk) != cpu_wp_tbl[cpu_wp_nr - 1].cpu_rate)
			high_bus_freq = 1;

		if (((clk_get_rate(ahb_clk) == LP_MED_CLK) && lp_high_freq) ||
							high_bus_freq) {
			/* Set to the high setpoint. */
			high_bus_freq_mode = 1;
			clk_set_rate(ahb_clk,
				clk_round_rate(ahb_clk, LP_NORMAL_CLK));
			clk_set_rate(ddr_hf_clk,
				clk_round_rate(ddr_hf_clk, DDR_NORMAL_CLK));
		}
		if (!lp_high_freq && !high_bus_freq) {
			/* Set to the medium setpoint. */
			high_bus_freq_mode = 0;
			low_bus_freq_mode = 0;
			clk_set_rate(ddr_hf_clk,
				clk_round_rate(ddr_hf_clk, DDR_LOW_FREQ_CLK));
			clk_set_rate(ahb_clk,
				     clk_round_rate(ahb_clk, LP_MED_CLK));
		}
	}
	start_sdram_autogating();
	}
	return 0;
}

int low_freq_bus_used(void)
{
	if ((clk_get_usecount(ipu_clk) == 0)
	    && (clk_get_usecount(vpu_clk) == 0)
	    && (lp_high_freq == 0)
	    && (lp_med_freq == 0))
		return 1;
	else
		return 0;
}

void setup_pll(void)
{
}

static ssize_t bus_freq_scaling_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (bus_freq_scaling_is_active)
		return sprintf(buf, "Bus frequency scaling is enabled\n");
	else
		return sprintf(buf, "Bus frequency scaling is disabled\n");
}

static ssize_t bus_freq_scaling_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	u32 reg;


	if (strstr(buf, "1") != NULL) {
		if (dvfs_per_active()) {
			printk(KERN_INFO "bus frequency scaling cannot be\
				 enabled when DVFS-PER is active\n");
			return size;
		}

		/* Initialize DVFS-PODF to 0. */
		reg = __raw_readl(MXC_CCM_CDCR);
		reg &= ~MXC_CCM_CDCR_PERIPH_CLK_DVFS_PODF_MASK;
		__raw_writel(reg, MXC_CCM_CDCR);
		clk_set_parent(main_bus_clk, pll2);

		bus_freq_scaling_is_active = 1;
	}
	else if (strstr(buf, "0") != NULL) {
		if (bus_freq_scaling_is_active)
			set_high_bus_freq(1);
		bus_freq_scaling_is_active = 0;
	}
	return size;
}

static int busfreq_suspend(struct platform_device *pdev, pm_message_t message)
{
	if (low_bus_freq_mode)
		set_high_bus_freq(1);
	busfreq_suspended = 1;
	return 0;
}

static int busfreq_resume(struct platform_device *pdev)
{
	busfreq_suspended = 0;
	return  0;
}

static DEVICE_ATTR(enable, 0644, bus_freq_scaling_enable_show,
			bus_freq_scaling_enable_store);

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
	int err = 0;

	busfreq_dev = &pdev->dev;

	main_bus_clk = clk_get(NULL, "main_bus_clk");
	if (IS_ERR(main_bus_clk)) {
		printk(KERN_DEBUG "%s: failed to get main_bus_clk\n",
		       __func__);
		return PTR_ERR(main_bus_clk);
	}

	pll2 = clk_get(NULL, "pll2");
	if (IS_ERR(pll2)) {
		printk(KERN_DEBUG "%s: failed to get pll2\n", __func__);
		return PTR_ERR(pll2);
	}

	axi_a_clk = clk_get(NULL, "axi_a_clk");
	if (IS_ERR(axi_a_clk)) {
		printk(KERN_DEBUG "%s: failed to get axi_a_clk\n",
		       __func__);
		return PTR_ERR(axi_a_clk);
	}

	axi_b_clk = clk_get(NULL, "axi_b_clk");
	if (IS_ERR(axi_b_clk)) {
		printk(KERN_DEBUG "%s: failed to get axi_b_clk\n",
		       __func__);
		return PTR_ERR(axi_b_clk);
	}

	ddr_hf_clk = clk_get(NULL, "ddr_hf_clk");
	if (IS_ERR(ddr_hf_clk)) {
		printk(KERN_DEBUG "%s: failed to get ddr_hf_clk\n",
		       __func__);
		return PTR_ERR(ddr_hf_clk);
	}

	emi_slow_clk = clk_get(NULL, "emi_slow_clk");
	if (IS_ERR(emi_slow_clk)) {
		printk(KERN_DEBUG "%s: failed to get emi_slow_clk\n",
		       __func__);
		return PTR_ERR(emi_slow_clk);
	}

	nfc_clk = clk_get(NULL, "nfc_clk");
	if (IS_ERR(nfc_clk)) {
		printk(KERN_DEBUG "%s: failed to get nfc_clk\n",
		       __func__);
		return PTR_ERR(nfc_clk);
	}

	ahb_clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(ahb_clk)) {
		printk(KERN_DEBUG "%s: failed to get ahb_clk\n",
		       __func__);
		return PTR_ERR(ahb_clk);
	}

	vpu_core_clk = clk_get(NULL, "vpu_core_clk");
	if (IS_ERR(vpu_core_clk)) {
		printk(KERN_DEBUG "%s: failed to get vpu_core_clk\n",
		       __func__);
		return PTR_ERR(vpu_core_clk);
	}

	ddr_clk = clk_get(NULL, "ddr_clk");
	if (IS_ERR(ddr_clk)) {
		printk(KERN_DEBUG "%s: failed to get ddr_clk\n",
		       __func__);
		return PTR_ERR(ddr_clk);
	}

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n",
		       __func__);
		return PTR_ERR(cpu_clk);
	}

	ipu_clk = clk_get(NULL, "ipu_clk");
	if (IS_ERR(ipu_clk)) {
		printk(KERN_DEBUG "%s: failed to get ipu_clk\n",
		       __func__);
		return PTR_ERR(ipu_clk);
	}

	mipi_hsp_clk = clk_get(NULL, "mipi_hsp_clk");
	if (IS_ERR(mipi_hsp_clk)) {
		printk(KERN_DEBUG "%s: failed to get mipi_hsp_clk\n",
		       __func__);
		return PTR_ERR(mipi_hsp_clk);
	}

	vpu_clk = clk_get(NULL, "vpu_clk");
	if (IS_ERR(vpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get vpu_clk\n",
		       __func__);
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
		printk(KERN_DEBUG "%s: failed to get lp_apm\n",
		       __func__);
		return PTR_ERR(lp_apm);
	}

	osc = clk_get(NULL, "osc");
	if (IS_ERR(osc)) {
		printk(KERN_DEBUG "%s: failed to get osc\n", __func__);
		return PTR_ERR(osc);
	}

	gpc_dvfs_clk = clk_get(NULL, "gpc_dvfs_clk");
	if (IS_ERR(gpc_dvfs_clk)) {
		printk(KERN_DEBUG "%s: failed to get gpc_dvfs_clk\n", __func__);
		return PTR_ERR(gpc_dvfs_clk);
	}

	err = sysfs_create_file(&busfreq_dev->kobj, &dev_attr_enable.attr);
	if (err) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for BUSFREQ");
		return err;
	}

	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	low_bus_freq_mode = 0;
	high_bus_freq_mode = 1;
	bus_freq_scaling_is_active = 0;
	bus_freq_scaling_initialized = 1;

	return 0;
}

static struct platform_driver busfreq_driver = {
	.driver = {
		   .name = "busfreq",
		},
	.probe = busfreq_probe,
	.suspend = busfreq_suspend,
	.resume = busfreq_resume,
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
	sysfs_remove_file(&busfreq_dev->kobj, &dev_attr_enable.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&busfreq_driver);
	bus_freq_scaling_initialized = 0;
}

module_init(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("BusFreq driver");
MODULE_LICENSE("GPL");
