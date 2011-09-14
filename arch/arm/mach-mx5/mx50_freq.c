/*
 * Copyright (C) 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx50_ddr.c
 *
 * @brief MX50 DDR specific information file.
 *
 *
 *
 * @ingroup PM
 */
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/iram_alloc.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>

#define LP_APM_CLK				24000000
#define HW_QOS_DISABLE			0x70
#define HW_QOS_DISABLE_SET	0x74
#define HW_QOS_DISABLE_CLR	0x78

static struct clk *epdc_clk;

/* DDR settings */
unsigned long (*iram_ddr_settings)[2];
unsigned long (*normal_databahn_settings)[2];
unsigned int mx50_ddr_type;
void *ddr_freq_change_iram_base;
void __iomem *databahn_base;

void (*change_ddr_freq)(void *ccm_addr, void *databahn_addr,
			u32 freq, void *iram_ddr_settings) = NULL;
void *wait_in_iram_base;
void (*wait_in_iram)(void *ccm_addr, void *databahn_addr, u32 sys_clk_count);

extern void mx50_wait(u32 ccm_base, u32 databahn_addr, u32 sys_clk_count);
extern int ddr_med_rate;
extern void __iomem *ccm_base;
extern void __iomem *databahn_base;
extern void mx50_ddr_freq_change(u32 ccm_base,
					u32 databahn_addr, u32 freq);

static void __iomem *qosc_base;
static int ddr_settings_size;

unsigned long lpddr2_databhan_regs_offsets[][2] = {
	{0x8, 0x0},
	{0xc, 0x0},
	{0x10, 0x0},
	{0x14, 0x0},
	{0x18, 0x0},
	{0x1c, 0x0},
	{0x20, 0x0},
	{0x24, 0x0},
	{0x28, 0x0},
	{0x2c, 0x0},
	{0x34, 0x0},
	{0x38, 0x0},
	{0x3c, 0x0},
	{0x40, 0x0},
	{0x48, 0x0},
	{0x6c, 0x0},
	{0x78, 0x0},
	{0x80, 0x0},
	{0x84, 0x0},
	{0x88, 0x0},
	{0x8c, 0x0},
	{0xcc, 0x0},
	{0xd4, 0x0},
	{0xd8, 0x0},
	{0x104, 0x0},
	{0x108, 0x0},
	{0x10c, 0x0},
	{0x110, 0x0},
	{0x114, 0x0},
	{0x200, 0x0},
	{0x204, 0x0},
	{0x208, 0x0},
	{0x20c, 0x0},
	{0x210, 0x0},
	{0x214, 0x0},
	{0x218, 0x0},
	{0x21c, 0x0},
	{0x220, 0x0},
	{0x224, 0x0},
	{0x228, 0x0},
	{0x22c, 0x0},
	{0x234, 0x0},
	{0x238, 0x0},
	{0x23c, 0x0},
	{0x240, 0x0},
	{0x244, 0x0},
	{0x248, 0x0},
	{0x24c, 0x0},
	{0x250, 0x0},
	{0x254, 0x0},
	{0x258, 0x0},
	{0x25c, 0x0} };

unsigned long lpddr2_24[][2] = {
		{0x08, 0x00000003},
		{0x0c, 0x000012c0},
		{0x10, 0x00000018},
		{0x14, 0x000000f0},
		{0x18, 0x02030b0c},
		{0x1c, 0x02020104},
		{0x20, 0x05010102},
		{0x24, 0x00068005},
		{0x28, 0x01000103},
		{0x2c, 0x04030101},
		{0x34, 0x00000202},
		{0x38, 0x00000001},
		{0x3c, 0x00000401},
		{0x40, 0x00030050},
		{0x48, 0x00040004},
		{0x6c, 0x00040022},
		{0x78, 0x00040022},
		{0x80, 0x00180000},
		{0x84, 0x00000009},
		{0x88, 0x02400003},
		{0x8c, 0x01000200},
		{0xcc, 0x00000000},
		{0xd4, 0x01010301},
		{0xd8, 0x00000101},
		{0x104, 0x02000602},
		{0x108, 0x00560000},
		{0x10c, 0x00560056},
		{0x110, 0x00560056},
		{0x114, 0x03060056},
		{0x200, 0x00000000},
		{0x204, 0x00000000},
		{0x208, 0xf3003a27},
		{0x20c, 0x074002c1},
		{0x210, 0xf3003a27},
		{0x214, 0x074002c1},
		{0x218, 0xf3003a27},
		{0x21c, 0x074002c1},
		{0x220, 0xf3003a27},
		{0x224, 0x074002c1},
		{0x228, 0xf3003a27},
		{0x22c, 0x074002c1},
		{0x234, 0x00810004},
		{0x238, 0x30219fd3},
		{0x23c, 0x00219fc1},
		{0x240, 0x30219fd3},
		{0x244, 0x00219fc1},
		{0x248, 0x30219fd3},
		{0x24c, 0x00219fc1},
		{0x250, 0x30219fd3},
		{0x254, 0x00219fc1},
		{0x258, 0x30219fd3},
		{0x25c, 0x00219fc1} };

unsigned long mddr_databhan_regs_offsets[][2] = {
	{0x08, 0x0},
	{0x14, 0x0},
	{0x18, 0x0},
	{0x1c, 0x0},
	{0x20, 0x0},
	{0x24, 0x0},
	{0x28, 0x0},
	{0x2c, 0x0},
	{0x34, 0x0},
	{0x38, 0x0},
	{0x3c, 0x0},
	{0x40, 0x0},
	{0x48, 0x0},
	{0x6c, 0x0},
	{0xd4, 0x0},
	{0x108, 0x0},
	{0x10c, 0x0},
	{0x110, 0x0},
	{0x114, 0x0},
	{0x200, 0x0},
	{0x204, 0x0},
	{0x208, 0x0},
	{0x20c, 0x0},
	{0x210, 0x0},
	{0x214, 0x0},
	{0x218, 0x0},
	{0x21c, 0x0},
	{0x220, 0x0},
	{0x224, 0x0},
	{0x228, 0x0},
	{0x22c, 0x0},
	{0x234, 0x0},
	{0x238, 0x0},
	{0x23c, 0x0},
	{0x240, 0x0},
	{0x244, 0x0},
	{0x248, 0x0},
	{0x24c, 0x0},
	{0x250, 0x0},
	{0x254, 0x0},
	{0x258, 0x0},
	{0x25c, 0x0} };


unsigned long mddr_24[][2] = {
		{0x08, 0x000012c0},
		{0x14, 0x02000000},
		{0x18, 0x01010506},
		{0x1c, 0x01020101},
		{0x20, 0x02000103},
		{0x24, 0x01069002},
		{0x28, 0x01000101},
		{0x2c, 0x02010101},
		{0x34, 0x00000602},
		{0x38, 0x00000001},
		{0x3c, 0x00000301},
		{0x40, 0x000500b0},
		{0x48, 0x00030003},
		{0x6c, 0x00000000},
		{0xd4, 0x00000200},
		{0x108, 0x00b30000},
		{0x10c, 0x00b300b3},
		{0x110, 0x00b300b3},
		{0x114, 0x010300b3},
		{0x200, 0x00000100},
		{0x204, 0x00000000},
		{0x208, 0xf4003a27},
		{0x20c, 0x074002c0},
		{0x210, 0xf4003a27},
		{0x214, 0x074002c0},
		{0x218, 0xf4003a27},
		{0x21c, 0x074002c0},
		{0x220, 0xf4003a27},
		{0x224, 0x074002c0},
		{0x228, 0xf4003a27},
		{0x22c, 0x074002c0},
		{0x234, 0x00800005},
		{0x238, 0x30319f14},
		{0x23c, 0x00319f01},
		{0x240, 0x30319f14},
		{0x244, 0x00319f01},
		{0x248, 0x30319f14},
		{0x24c, 0x00319f01},
		{0x250, 0x30319f14},
		{0x254, 0x00319f01},
		{0x258, 0x30319f14},
		{0x25c, 0x00319f01} };

int can_change_ddr_freq(void)
{
	if (clk_get_usecount(epdc_clk) == 0)
		return 1;
	return 0;
}

int update_ddr_freq(int ddr_rate)
{
	int i;
	unsigned int reg;

	if (!can_change_ddr_freq())
		return -1;

	local_flush_tlb_all();
	flush_cache_all();

	iram_ddr_settings[0][0] = ddr_settings_size;
	if (ddr_rate == LP_APM_CLK) {
		if (mx50_ddr_type == MX50_LPDDR2) {
			for (i = 0; i < iram_ddr_settings[0][0]; i++) {
				iram_ddr_settings[i + 1][0] =
								lpddr2_24[i][0];
				iram_ddr_settings[i + 1][1] =
								lpddr2_24[i][1];
			}
		} else {
			for (i = 0; i < iram_ddr_settings[0][0]; i++) {
				iram_ddr_settings[i + 1][0]
								= mddr_24[i][0];
				iram_ddr_settings[i + 1][1]
								= mddr_24[i][1];
			}
		}
	} else {
		for (i = 0; i < iram_ddr_settings[0][0]; i++) {
			iram_ddr_settings[i + 1][0] =
					normal_databahn_settings[i][0];
			iram_ddr_settings[i + 1][1] =
					normal_databahn_settings[i][1];
		}
		if (ddr_rate == ddr_med_rate) {
			/*Change the tref setting */
			for (i = 0; i < iram_ddr_settings[0][0]; i++) {
				if (iram_ddr_settings[i + 1][0] == 0x40) {
					if (mx50_ddr_type == MX50_LPDDR2)
						/* LPDDR2 133MHz. */
						iram_ddr_settings[i + 1][1] =
								0x00050180;
					else
						/* mDDR 133MHz. */
						iram_ddr_settings[i + 1][1] =
								0x00050208;
					break;
				}
			}
		}
	}
	/* Disable all masters from accessing the DDR. */
	reg = __raw_readl(qosc_base + HW_QOS_DISABLE);
	reg |= 0xFFE;
	__raw_writel(reg, qosc_base + HW_QOS_DISABLE_SET);
	udelay(100);

	/* Set the DDR to default freq. */
	change_ddr_freq(ccm_base, databahn_base, ddr_rate,
					iram_ddr_settings);

	/* Enable all masters to access the DDR. */
	__raw_writel(reg, qosc_base + HW_QOS_DISABLE_CLR);

	return 0;
}

void init_ddr_settings(void)
{
	unsigned long iram_paddr;
	unsigned int reg;
	int i;
	struct clk *ddr_clk = clk_get(NULL, "ddr_clk");

	databahn_base = ioremap(MX50_DATABAHN_BASE_ADDR, SZ_16K);

	/* Find the memory type, LPDDR2 or mddr. */
	mx50_ddr_type = __raw_readl(databahn_base) & 0xF00;
	if (mx50_ddr_type == MX50_LPDDR2) {
		normal_databahn_settings = lpddr2_databhan_regs_offsets;
		ddr_settings_size = ARRAY_SIZE(lpddr2_databhan_regs_offsets);
		}
	else if (mx50_ddr_type == MX50_MDDR) {
		normal_databahn_settings = mddr_databhan_regs_offsets;
		ddr_settings_size = ARRAY_SIZE(mddr_databhan_regs_offsets);
	} else {
		printk(KERN_DEBUG
		"%s: Unsupported memory type\n", __func__);
		return;
	}

	/* Copy the databhan settings into the iram location. */
	for (i = 0; i < ddr_settings_size; i++) {
			normal_databahn_settings[i][1] =
				__raw_readl(databahn_base
				+ normal_databahn_settings[i][0]);
		}
	/* Store the size of the array in iRAM also,
	 * increase the size by 8 bytes.
	 */
	iram_ddr_settings = iram_alloc(ddr_settings_size + 8, &iram_paddr);
	if (iram_ddr_settings == NULL) {
			printk(KERN_DEBUG
			"%s: failed to allocate iRAM memory for ddr settings\n",
			__func__);
			return;
	}

	/* Allocate IRAM for the DDR freq change code. */
	iram_alloc(SZ_8K, &iram_paddr);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	ddr_freq_change_iram_base = __arm_ioremap(iram_paddr,
						SZ_8K, MT_MEMORY);
	memcpy(ddr_freq_change_iram_base, mx50_ddr_freq_change, SZ_8K);
	change_ddr_freq = (void *)ddr_freq_change_iram_base;

	qosc_base = ioremap(MX50_QOSC_BASE_ADDR, SZ_4K);
	/* Enable the QoSC */
	reg = __raw_readl(qosc_base);
	reg &= ~0xC0000000;
	__raw_writel(reg, qosc_base);

	/* Allocate IRAM to run the WFI code from iram, since
	 * we can turn off the DDR clocks when ARM is in WFI.
	 */
	iram_alloc(SZ_4K, &iram_paddr);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	wait_in_iram_base = __arm_ioremap(iram_paddr,
						SZ_4K, MT_MEMORY);
	memcpy(wait_in_iram_base, mx50_wait, SZ_4K);
	wait_in_iram = (void *)wait_in_iram_base;

	clk_enable(ddr_clk);

	/* Set the DDR to enter automatic self-refresh. */
	/* Set the DDR to automatically enter lower power mode 4. */
	reg = __raw_readl(databahn_base + DATABAHN_CTL_REG22);
	reg &= ~LOWPOWER_AUTOENABLE_MASK;
	reg |= 1 << 1;
	__raw_writel(reg, databahn_base + DATABAHN_CTL_REG22);

	/* set the counter for entering mode 4. */
	reg = __raw_readl(databahn_base + DATABAHN_CTL_REG21);
	reg &= ~LOWPOWER_EXTERNAL_CNT_MASK;
	reg = 128 << LOWPOWER_EXTERNAL_CNT_OFFSET;
	__raw_writel(reg, databahn_base + DATABAHN_CTL_REG21);

	/* Enable low power mode 4 */
	reg = __raw_readl(databahn_base + DATABAHN_CTL_REG20);
	reg &= ~LOWPOWER_CONTROL_MASK;
	reg |= 1 << 1;
	__raw_writel(reg, databahn_base + DATABAHN_CTL_REG20);
	clk_disable(ddr_clk);

	epdc_clk = clk_get(NULL, "epdc_axi");
	if (IS_ERR(epdc_clk)) {
		printk(KERN_DEBUG "%s: failed to get epdc_axi_clk\n",
			__func__);
		return;
	}
}
