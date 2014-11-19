/*
 * Copyright 2011-2014 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mcc_imx6sx.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/suspend.h>
#include <linux/genalloc.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <asm/cacheflush.h>
#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/tlb.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/map.h>

#include "common.h"
#include "hardware.h"

#define CCR				0x0
#define BM_CCR_WB_COUNT			(0x7 << 16)
#define BM_CCR_RBC_BYPASS_COUNT		(0x3f << 21)
#define BM_CCR_RBC_EN			(0x1 << 27)

#define CLPCR				0x54
#define BP_CLPCR_LPM			0
#define BM_CLPCR_LPM			(0x3 << 0)
#define BM_CLPCR_BYPASS_PMIC_READY	(0x1 << 2)
#define BM_CLPCR_ARM_CLK_DIS_ON_LPM	(0x1 << 5)
#define BM_CLPCR_SBYOS			(0x1 << 6)
#define BM_CLPCR_DIS_REF_OSC		(0x1 << 7)
#define BM_CLPCR_VSTBY			(0x1 << 8)
#define BP_CLPCR_STBY_COUNT		9
#define BM_CLPCR_STBY_COUNT		(0x3 << 9)
#define BM_CLPCR_COSC_PWRDOWN		(0x1 << 11)
#define BM_CLPCR_WB_PER_AT_LPM		(0x1 << 16)
#define BM_CLPCR_WB_CORE_AT_LPM		(0x1 << 17)
#define BM_CLPCR_BYP_MMDC_CH0_LPM_HS	(0x1 << 19)
#define BM_CLPCR_BYP_MMDC_CH1_LPM_HS	(0x1 << 21)
#define BM_CLPCR_MASK_CORE0_WFI		(0x1 << 22)
#define BM_CLPCR_MASK_CORE1_WFI		(0x1 << 23)
#define BM_CLPCR_MASK_CORE2_WFI		(0x1 << 24)
#define BM_CLPCR_MASK_CORE3_WFI		(0x1 << 25)
#define BM_CLPCR_MASK_SCU_IDLE		(0x1 << 26)
#define BM_CLPCR_MASK_L2CC_IDLE		(0x1 << 27)

#define CGPR				0x64
#define BM_CGPR_INT_MEM_CLK_LPM		(0x1 << 17)

#define MX6_INT_IOMUXC			32

#define ROMC_ROMPATCH0D			0xf0
#define ROMC_ROMPATCHCNTL		0xf4
#define ROMC_ROMPATCHENL		0xfc
#define ROMC_ROMPATCH0A			0x100
#define BM_ROMPATCHCNTL_0D		(0x1 << 0)
#define BM_ROMPATCHCNTL_DIS		(0x1 << 29)
#define BM_ROMPATCHENL_0D		(0x1 << 0)
#define ROM_ADDR_FOR_INTERNAL_RAM_BASE	0x10d7c

#define UART_UCR1	0x80
#define UART_UCR2	0x84
#define UART_UCR3	0x88
#define UART_UCR4	0x8c
#define UART_UFCR	0x90
#define UART_UESC	0x9c
#define UART_UTIM	0xa0
#define UART_UBIR	0xa4
#define UART_UBMR	0xa8
#define UART_UBRC	0xac
#define UART_UTS	0xb4

/* QSPI register layout */
#define QSPI_MCR			0x00
#define QSPI_IPCR			0x08
#define QSPI_BUF0CR			0x10
#define QSPI_BUF1CR			0x14
#define QSPI_BUF2CR			0x18
#define QSPI_BUF3CR			0x1c
#define QSPI_BFGENCR			0x20
#define QSPI_BUF0IND			0x30
#define QSPI_BUF1IND			0x34
#define QSPI_BUF2IND			0x38
#define QSPI_SFAR			0x100
#define QSPI_SMPR			0x108
#define QSPI_RBSR			0x10c
#define QSPI_RBCT			0x110
#define QSPI_TBSR			0x150
#define QSPI_TBDR			0x154
#define QSPI_SFA1AD			0x180
#define QSPI_SFA2AD			0x184
#define QSPI_SFB1AD			0x188
#define QSPI_SFB2AD			0x18c
#define QSPI_RBDR_BASE			0x200
#define QSPI_LUTKEY			0x300
#define QSPI_LCKCR			0x304
#define QSPI_LUT_BASE			0x310

#define QSPI_RBDR_(x)		(QSPI_RBDR_BASE + (x) * 4)
#define QSPI_LUT(x)		(QSPI_LUT_BASE + (x) * 4)

#define QSPI_LUTKEY_VALUE	0x5AF05AF0
#define QSPI_LCKER_LOCK		0x1
#define QSPI_LCKER_UNLOCK	0x2

enum qspi_regs_valuetype {
	QSPI_PREDEFINED,
	QSPI_RETRIEVED,
};

struct qspi_regs {
	int offset;
	unsigned int value;
	enum qspi_regs_valuetype valuetype;
};

struct qspi_regs qspi_regs_imx6sx[] = {
	{QSPI_IPCR, 0, QSPI_RETRIEVED},
	{QSPI_BUF0CR, 0, QSPI_RETRIEVED},
	{QSPI_BUF1CR, 0, QSPI_RETRIEVED},
	{QSPI_BUF2CR, 0, QSPI_RETRIEVED},
	{QSPI_BUF3CR, 0, QSPI_RETRIEVED},
	{QSPI_BFGENCR, 0, QSPI_RETRIEVED},
	{QSPI_BUF0IND, 0, QSPI_RETRIEVED},
	{QSPI_BUF1IND, 0, QSPI_RETRIEVED},
	{QSPI_BUF2IND, 0, QSPI_RETRIEVED},
	{QSPI_SFAR, 0, QSPI_RETRIEVED},
	{QSPI_SMPR, 0, QSPI_RETRIEVED},
	{QSPI_RBSR, 0, QSPI_RETRIEVED},
	{QSPI_RBCT, 0, QSPI_RETRIEVED},
	{QSPI_TBSR, 0, QSPI_RETRIEVED},
	{QSPI_TBDR, 0, QSPI_RETRIEVED},
	{QSPI_SFA1AD, 0, QSPI_RETRIEVED},
	{QSPI_SFA2AD, 0, QSPI_RETRIEVED},
	{QSPI_SFB1AD, 0, QSPI_RETRIEVED},
	{QSPI_SFB2AD, 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(0), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(1), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(2), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(3), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(4), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(5), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(6), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(7), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(8), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(9), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(10), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(11), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(12), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(13), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(14), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(15), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(16), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(17), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(18), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(19), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(20), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(21), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(22), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(23), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(24), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(25), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(26), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(27), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(28), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(29), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(30), 0, QSPI_RETRIEVED},
	{QSPI_RBDR_(31), 0, QSPI_RETRIEVED},
	{QSPI_LUTKEY, QSPI_LUTKEY_VALUE, QSPI_PREDEFINED},
	{QSPI_LCKCR, QSPI_LCKER_UNLOCK, QSPI_PREDEFINED},
	{QSPI_LUT(0), 0, QSPI_RETRIEVED},
	{QSPI_LUT(1), 0, QSPI_RETRIEVED},
	{QSPI_LUT(2), 0, QSPI_RETRIEVED},
	{QSPI_LUT(3), 0, QSPI_RETRIEVED},
	{QSPI_LUT(4), 0, QSPI_RETRIEVED},
	{QSPI_LUT(5), 0, QSPI_RETRIEVED},
	{QSPI_LUT(6), 0, QSPI_RETRIEVED},
	{QSPI_LUT(7), 0, QSPI_RETRIEVED},
	{QSPI_LUT(8), 0, QSPI_RETRIEVED},
	{QSPI_LUT(9), 0, QSPI_RETRIEVED},
	{QSPI_LUT(10), 0, QSPI_RETRIEVED},
	{QSPI_LUT(11), 0, QSPI_RETRIEVED},
	{QSPI_LUT(12), 0, QSPI_RETRIEVED},
	{QSPI_LUT(13), 0, QSPI_RETRIEVED},
	{QSPI_LUT(14), 0, QSPI_RETRIEVED},
	{QSPI_LUT(15), 0, QSPI_RETRIEVED},
	{QSPI_LUT(16), 0, QSPI_RETRIEVED},
	{QSPI_LUT(17), 0, QSPI_RETRIEVED},
	{QSPI_LUT(18), 0, QSPI_RETRIEVED},
	{QSPI_LUT(19), 0, QSPI_RETRIEVED},
	{QSPI_LUT(20), 0, QSPI_RETRIEVED},
	{QSPI_LUT(21), 0, QSPI_RETRIEVED},
	{QSPI_LUT(22), 0, QSPI_RETRIEVED},
	{QSPI_LUT(23), 0, QSPI_RETRIEVED},
	{QSPI_LUT(24), 0, QSPI_RETRIEVED},
	{QSPI_LUT(25), 0, QSPI_RETRIEVED},
	{QSPI_LUT(26), 0, QSPI_RETRIEVED},
	{QSPI_LUT(27), 0, QSPI_RETRIEVED},
	{QSPI_LUT(28), 0, QSPI_RETRIEVED},
	{QSPI_LUT(29), 0, QSPI_RETRIEVED},
	{QSPI_LUT(30), 0, QSPI_RETRIEVED},
	{QSPI_LUT(31), 0, QSPI_RETRIEVED},
	{QSPI_LUT(32), 0, QSPI_RETRIEVED},
	{QSPI_LUT(33), 0, QSPI_RETRIEVED},
	{QSPI_LUT(34), 0, QSPI_RETRIEVED},
	{QSPI_LUT(35), 0, QSPI_RETRIEVED},
	{QSPI_LUT(36), 0, QSPI_RETRIEVED},
	{QSPI_LUT(37), 0, QSPI_RETRIEVED},
	{QSPI_LUT(38), 0, QSPI_RETRIEVED},
	{QSPI_LUT(39), 0, QSPI_RETRIEVED},
	{QSPI_LUT(40), 0, QSPI_RETRIEVED},
	{QSPI_LUT(41), 0, QSPI_RETRIEVED},
	{QSPI_LUT(42), 0, QSPI_RETRIEVED},
	{QSPI_LUT(43), 0, QSPI_RETRIEVED},
	{QSPI_LUT(44), 0, QSPI_RETRIEVED},
	{QSPI_LUT(45), 0, QSPI_RETRIEVED},
	{QSPI_LUT(46), 0, QSPI_RETRIEVED},
	{QSPI_LUT(47), 0, QSPI_RETRIEVED},
	{QSPI_LUT(48), 0, QSPI_RETRIEVED},
	{QSPI_LUT(49), 0, QSPI_RETRIEVED},
	{QSPI_LUT(50), 0, QSPI_RETRIEVED},
	{QSPI_LUT(51), 0, QSPI_RETRIEVED},
	{QSPI_LUT(52), 0, QSPI_RETRIEVED},
	{QSPI_LUT(53), 0, QSPI_RETRIEVED},
	{QSPI_LUT(54), 0, QSPI_RETRIEVED},
	{QSPI_LUT(55), 0, QSPI_RETRIEVED},
	{QSPI_LUT(56), 0, QSPI_RETRIEVED},
	{QSPI_LUT(57), 0, QSPI_RETRIEVED},
	{QSPI_LUT(58), 0, QSPI_RETRIEVED},
	{QSPI_LUT(59), 0, QSPI_RETRIEVED},
	{QSPI_LUT(60), 0, QSPI_RETRIEVED},
	{QSPI_LUT(61), 0, QSPI_RETRIEVED},
	{QSPI_LUT(62), 0, QSPI_RETRIEVED},
	{QSPI_LUT(63), 0, QSPI_RETRIEVED},
	{QSPI_LUTKEY, QSPI_LUTKEY_VALUE, QSPI_PREDEFINED},
	{QSPI_LCKCR, QSPI_LCKER_LOCK, QSPI_PREDEFINED},
	{QSPI_MCR, 0, QSPI_RETRIEVED},
};

unsigned long iram_tlb_base_addr;
unsigned long iram_tlb_phys_addr;

static unsigned int *ocram_saved_in_ddr;
static void *suspend_iram_base;
static unsigned long iram_paddr;
static int (*suspend_in_iram_fn)(void *iram_vbase,
	unsigned long iram_pbase, unsigned int cpu_type);
static unsigned int cpu_type;
static void __iomem *ccm_base;
static void __iomem *ocram_base;
static void __iomem *console_base;
static void __iomem *qspi_base;
static unsigned int ocram_size;
struct regmap *romcp;

unsigned long total_suspend_size;
static unsigned long dcr;
static unsigned long pcr;
extern unsigned long imx6_suspend_start asm("imx6_suspend_start");
extern unsigned long imx6_suspend_end asm("imx6_suspend_end");
extern bool m4_freq_low;

unsigned long save_ttbr1(void)
{
	unsigned long lttbr1;
	asm volatile(
		".align 4\n"
		"mrc p15, 0, %0, c2, c0, 1\n"
	: "=r" (lttbr1)
	);
	return lttbr1;
}

void restore_ttbr1(unsigned long ttbr1)
{
	asm volatile(
		".align 4\n"
		"mcr p15, 0, %0, c2, c0, 1\n"
	: : "r" (ttbr1)
	);
}

void imx6_set_cache_lpm_in_wait(bool enable)
{
	if ((cpu_is_imx6q() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_1) ||
		(cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0) || cpu_is_imx6sx()) {
		u32 val;

		val = readl_relaxed(ccm_base + CGPR);
		if (enable)
			val |= BM_CGPR_INT_MEM_CLK_LPM;
		else
			val &= ~BM_CGPR_INT_MEM_CLK_LPM;
		writel_relaxed(val, ccm_base + CGPR);
	}
}

static void imx6_save_cpu_arch_regs(void)
{
	/* Save the Diagnostic Control Register. */
	asm volatile(
		"mrc p15, 0, %0, c15, c0, 1\n"
	: "=r" (dcr)
	);
	/* Save the Power Control Register. */
	asm volatile(
		"mrc p15, 0, %0, c15, c0, 0\n"
	: "=r" (pcr)
	);
}

static void imx6_restore_cpu_arch_regs(void)
{
	/* Restore the diagnostic Control Register. */
	asm volatile(
		"mcr p15, 0, %0, c15, c0, 1\n"
	: : "r" (dcr)
	);
	/* Restore the Power Control Register. */
	asm volatile(
		"mcr p15, 0, %0, c15, c0, 0\n"
	: : "r" (pcr)
	);
}

void imx6_enable_rbc(bool enable)
{
	u32 val;

	/*
	 * need to mask all interrupts in GPC before
	 * operating RBC configurations
	 */
	imx_gpc_mask_all();

	/* configure RBC enable bit */
	val = readl_relaxed(ccm_base + CCR);
	val &= ~BM_CCR_RBC_EN;
	val |= enable ? BM_CCR_RBC_EN : 0;
	writel_relaxed(val, ccm_base + CCR);

	/* configure RBC count */
	val = readl_relaxed(ccm_base + CCR);
	val &= ~BM_CCR_RBC_BYPASS_COUNT;
	val |= enable ? BM_CCR_RBC_BYPASS_COUNT : 0;
	writel(val, ccm_base + CCR);

	/*
	 * need to delay at least 2 cycles of CKIL(32K)
	 * due to hardware design requirement, which is
	 * ~61us, here we use 65us for safe
	 */
	udelay(65);

	/* restore GPC interrupt mask settings */
	imx_gpc_restore_all();
}

static void imx6_enable_wb(bool enable)
{
	u32 val;

	/* configure well bias enable bit */
	val = readl_relaxed(ccm_base + CLPCR);
	val &= ~BM_CLPCR_WB_PER_AT_LPM;
	val |= enable ? BM_CLPCR_WB_PER_AT_LPM : 0;
	writel_relaxed(val, ccm_base + CLPCR);

	/* configure well bias count */
	val = readl_relaxed(ccm_base + CCR);
	val &= ~BM_CCR_WB_COUNT;
	val |= enable ? BM_CCR_WB_COUNT : 0;
	writel_relaxed(val, ccm_base + CCR);
}

int imx6_set_lpm(enum mxc_cpu_pwr_mode mode)
{
	u32 val = readl_relaxed(ccm_base + CLPCR);
	struct irq_desc *desc = irq_to_desc(MX6_INT_IOMUXC);

	/*
	 * CCM state machine has restriction, before enabling
	 * LPM mode, need to make sure last LPM mode is waked up
	 * by dsm_wakeup_signal, which means the wakeup source
	 * must be seen by GPC, then CCM will clean its state machine
	 * and re-sample necessary signal to decide whether it can
	 * enter LPM mode. We force irq #32 to be always pending,
	 * unmask it before we enable LPM mode and mask it after LPM
	 * is enabled, this flow will make sure CCM state machine in
	 * reliable status before entering LPM mode. Otherwise, CCM
	 * may enter LPM mode by mistake which will cause system bus
	 * locked by CPU access not finished, as when CCM enter
	 * LPM mode, CPU will stop running.
	 */
	imx_gpc_irq_unmask(&desc->irq_data);

	val &= ~BM_CLPCR_LPM;
	switch (mode) {
	case WAIT_CLOCKED:
		break;
	case WAIT_UNCLOCKED:
		val |= 0x1 << BP_CLPCR_LPM;
		val |= BM_CLPCR_ARM_CLK_DIS_ON_LPM;
		val &= ~BM_CLPCR_VSTBY;
		val &= ~BM_CLPCR_SBYOS;
		if (cpu_is_imx6sl() || cpu_is_imx6sx())
			val |= BM_CLPCR_BYP_MMDC_CH0_LPM_HS;
		else
			val |= BM_CLPCR_BYP_MMDC_CH1_LPM_HS;
		break;
	case STOP_POWER_ON:
		val |= 0x2 << BP_CLPCR_LPM;
		val &= ~BM_CLPCR_VSTBY;
		val &= ~BM_CLPCR_SBYOS;
		if (cpu_is_imx6sl())
			val |= BM_CLPCR_BYPASS_PMIC_READY;
		if (cpu_is_imx6sl() || cpu_is_imx6sx()) {
			val |= BM_CLPCR_BYP_MMDC_CH0_LPM_HS;
		} else {
			val |= BM_CLPCR_BYP_MMDC_CH1_LPM_HS;
		}
		break;
	case WAIT_UNCLOCKED_POWER_OFF:
		val |= 0x1 << BP_CLPCR_LPM;
		val &= ~BM_CLPCR_VSTBY;
		val &= ~BM_CLPCR_SBYOS;
		break;
	case STOP_POWER_OFF:
		val |= 0x2 << BP_CLPCR_LPM;
		val |= 0x3 << BP_CLPCR_STBY_COUNT;
		val |= BM_CLPCR_VSTBY;
		val |= BM_CLPCR_SBYOS;
		if (cpu_is_imx6sl())
			val |= BM_CLPCR_BYPASS_PMIC_READY;
		if (cpu_is_imx6sl() || cpu_is_imx6sx()) {
			val |= BM_CLPCR_BYP_MMDC_CH0_LPM_HS;
		} else {
			val |= BM_CLPCR_BYP_MMDC_CH1_LPM_HS;
		}
		break;
	default:
		imx_gpc_irq_mask(&desc->irq_data);
		return -EINVAL;
	}

	writel_relaxed(val, ccm_base + CLPCR);
	imx_gpc_irq_mask(&desc->irq_data);

	return 0;
}

static int imx6_suspend_finish(unsigned long val)
{
	/*
	 * call low level suspend function in iram,
	 * as we need to float DDR IO.
	 */
	u32 ttbr1;

	ttbr1 = save_ttbr1();
	suspend_in_iram_fn(suspend_iram_base, iram_paddr, cpu_type);
	restore_ttbr1(ttbr1);
	return 0;
}

static void imx6_console_save(unsigned int *regs)
{
	if (!console_base)
		return;

	regs[0] = readl_relaxed(console_base + UART_UCR1);
	regs[1] = readl_relaxed(console_base + UART_UCR2);
	regs[2] = readl_relaxed(console_base + UART_UCR3);
	regs[3] = readl_relaxed(console_base + UART_UCR4);
	regs[4] = readl_relaxed(console_base + UART_UFCR);
	regs[5] = readl_relaxed(console_base + UART_UESC);
	regs[6] = readl_relaxed(console_base + UART_UTIM);
	regs[7] = readl_relaxed(console_base + UART_UBIR);
	regs[8] = readl_relaxed(console_base + UART_UBMR);
	regs[9] = readl_relaxed(console_base + UART_UBRC);
	regs[10] = readl_relaxed(console_base + UART_UTS);
}

static void imx6_console_restore(unsigned int *regs)
{
	if (!console_base)
		return;

	writel_relaxed(regs[4], console_base + UART_UFCR);
	writel_relaxed(regs[5], console_base + UART_UESC);
	writel_relaxed(regs[6], console_base + UART_UTIM);
	writel_relaxed(regs[7], console_base + UART_UBIR);
	writel_relaxed(regs[8], console_base + UART_UBMR);
	writel_relaxed(regs[9], console_base + UART_UBRC);
	writel_relaxed(regs[10], console_base + UART_UTS);
	writel_relaxed(regs[0], console_base + UART_UCR1);
	writel_relaxed(regs[1] | 0x1, console_base + UART_UCR2);
	writel_relaxed(regs[2], console_base + UART_UCR3);
	writel_relaxed(regs[3], console_base + UART_UCR4);
}

static void imx6_qspi_save(struct qspi_regs *pregs, int reg_num)
{
	int i;

	if (!qspi_base)
		return;

	for (i = 0; i < reg_num; i++) {
		if (QSPI_RETRIEVED == pregs[i].valuetype)
			pregs[i].value = readl_relaxed(qspi_base + pregs[i].offset);
	}
}

static void imx6_qspi_restore(struct qspi_regs *pregs, int reg_num)
{
	int i;

	if (!qspi_base)
		return;

	for (i = 0; i < reg_num; i++)
		writel_relaxed(pregs[i].value, qspi_base + pregs[i].offset);
}

static int imx6_pm_enter(suspend_state_t state)
{
	struct regmap *g;
	unsigned int console_saved_reg[11] = {0};

	if (imx_src_is_m4_enabled()) {
		if (imx_gpc_is_m4_sleeping() && m4_freq_low) {
			imx_gpc_hold_m4_in_sleep();
			mcc_enable_m4_irqs_in_gic(true);
		} else {
			pr_info("M4 is busy, enter WAIT mode instead of STOP!\n");
			imx6_set_lpm(WAIT_UNCLOCKED);
			imx6_set_cache_lpm_in_wait(true);
			imx_gpc_pre_suspend(false);
			/* Zzz ... */
			cpu_do_idle();
			imx_gpc_post_resume();
			imx6_set_lpm(WAIT_CLOCKED);
			return 0;
		}
	}

	if (!iram_tlb_base_addr) {
		pr_warn("No IRAM/OCRAM memory allocated for suspend/resume code. \
			Please ensure device tree has an entry for fsl,lpm-sram.\n");
			return -EINVAL;
	}

	/*
	 * L2 can exit by 'reset' or Inband beacon (from remote EP)
	 * toggling phy_powerdown has same effect as 'inband beacon'
	 * So, toggle bit18 of GPR1, used as a workaround of errata
	 * "PCIe PCIe does not support L2 Power Down"
	 */
	if (IS_ENABLED(CONFIG_PCI_IMX6) && !cpu_is_imx6sx()) {
		g = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (IS_ERR(g)) {
			pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
			return PTR_ERR(g);
		}
		regmap_update_bits(g, IOMUXC_GPR1, IMX6Q_GPR1_PCIE_TEST_PD,
				IMX6Q_GPR1_PCIE_TEST_PD);
	}

	switch (state) {
	case PM_SUSPEND_STANDBY:
		imx6_set_lpm(STOP_POWER_ON);
		imx6_set_cache_lpm_in_wait(true);
		imx_gpc_pre_suspend(false);
		if (cpu_is_imx6sl())
			imx6sl_set_wait_clk(true);
		/* Zzz ... */
		cpu_do_idle();
		if (cpu_is_imx6sl())
			imx6sl_set_wait_clk(false);
		imx_gpc_post_resume();
		imx6_set_lpm(WAIT_CLOCKED);
		break;
	case PM_SUSPEND_MEM:
		imx6_enable_wb(true);
		imx6_set_cache_lpm_in_wait(false);
		imx6_set_lpm(STOP_POWER_OFF);
		imx_gpc_pre_suspend(true);
		imx_anatop_pre_suspend();
		imx_set_cpu_jump(0, v7_cpu_resume);

		imx6_save_cpu_arch_regs();
		if (cpu_is_imx6sx() && imx_gpc_is_mf_mix_off()) {
			memcpy(ocram_saved_in_ddr, ocram_base, ocram_size);
			imx6_console_save(console_saved_reg);
			if (imx_src_is_m4_enabled())
				imx6_qspi_save(qspi_regs_imx6sx, sizeof(qspi_regs_imx6sx)/sizeof(struct qspi_regs));
		}
		/* Zzz ... */
		cpu_suspend(0, imx6_suspend_finish);
		if (cpu_is_imx6sx() && imx_gpc_is_mf_mix_off()) {
			memcpy(ocram_base, ocram_saved_in_ddr, ocram_size);
			imx6_console_restore(console_saved_reg);
			if (imx_src_is_m4_enabled())
				imx6_qspi_restore(qspi_regs_imx6sx, sizeof(qspi_regs_imx6sx)/sizeof(struct qspi_regs));
		}
		imx6_restore_cpu_arch_regs();

		if (!cpu_is_imx6sl() && !cpu_is_imx6sx())
			imx_smp_prepare();
		imx_anatop_post_resume();
		imx_gpc_post_resume();
		imx6_enable_rbc(false);
		imx6_enable_wb(false);
		imx6_set_cache_lpm_in_wait(true);
		imx6_set_lpm(WAIT_CLOCKED);
		break;
	default:
		return -EINVAL;
	}

	/*
	 * L2 can exit by 'reset' or Inband beacon (from remote EP)
	 * toggling phy_powerdown has same effect as 'inband beacon'
	 * So, toggle bit18 of GPR1, used as a workaround of errata
	 * "PCIe PCIe does not support L2 Power Down"
	 */
	if (IS_ENABLED(CONFIG_PCI_IMX6) && !cpu_is_imx6sx()) {
		g = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (IS_ERR(g)) {
			pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
			return PTR_ERR(g);
		}
		regmap_update_bits(g, IOMUXC_GPR1, IMX6Q_GPR1_PCIE_TEST_PD,
				!IMX6Q_GPR1_PCIE_TEST_PD);
	}

	if (imx_src_is_m4_enabled()) {
		mcc_enable_m4_irqs_in_gic(false);
		imx_gpc_release_m4_in_sleep();
	}

	return 0;
}

static struct map_desc imx6_pm_io_desc[] __initdata = {
	imx_map_entry(MX6Q, MMDC_P0, MT_DEVICE),
	imx_map_entry(MX6Q, MMDC_P1, MT_DEVICE),
	imx_map_entry(MX6Q, SRC, MT_DEVICE),
	imx_map_entry(MX6Q, IOMUXC, MT_DEVICE),
	imx_map_entry(MX6Q, CCM, MT_DEVICE),
	imx_map_entry(MX6Q, ANATOP, MT_DEVICE),
	imx_map_entry(MX6Q, GPC, MT_DEVICE),
	imx_map_entry(MX6Q, L2, MT_DEVICE),
	imx_map_entry(MX6Q, SEMA4, MT_DEVICE),
};

static struct map_desc iram_tlb_io_desc __initdata = {
	/* .virtual and .pfn are run-time assigned */
	.length		= SZ_1M,
	.type		= MT_MEMORY_NONCACHED,
};

const static char *low_power_ocram_match[] __initconst = {
	"fsl,lpm-sram",
	NULL
};

static int __init imx6_dt_find_lpsram(unsigned long node,
		const char *uname, int depth, void *data)
{
	unsigned long lpram_addr;
	__be32 *prop;

	if (of_flat_dt_match(node, low_power_ocram_match)) {
		prop = of_get_flat_dt_prop(node, "reg", NULL);
		if (!prop)
			return -EINVAL;

		lpram_addr = be32_to_cpup(prop);

		/* We need to create a 1M page table entry. */
		iram_tlb_io_desc.virtual = IMX_IO_P2V(lpram_addr & 0xFFF00000);
		iram_tlb_io_desc.pfn = __phys_to_pfn(lpram_addr & 0xFFF00000);
		iram_tlb_phys_addr = lpram_addr;
		iram_tlb_base_addr = IMX_IO_P2V(lpram_addr);

		iotable_init(&iram_tlb_io_desc, 1);
	}
	return 0;

}

void __init imx6_pm_map_io(void)
{
	iotable_init(imx6_pm_io_desc, ARRAY_SIZE(imx6_pm_io_desc));

	/*
	 * Get the address of IRAM or OCRAM to be used by the low
	 * power code from the device tree.
	 */
	WARN_ON(of_scan_flat_dt(imx6_dt_find_lpsram, NULL));

	/* Return if no IRAM space is allocated for suspend/resume code. */
	if (!iram_tlb_base_addr)
		return;
}

static int imx6_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY || state == PM_SUSPEND_MEM);
}

static const struct platform_suspend_ops imx6_pm_ops = {
	.enter = imx6_pm_enter,
	.valid = imx6_pm_valid,
};

void imx6_pm_set_ccm_base(void __iomem *base)
{
	if (!base)
		pr_warn("ccm base is NULL!\n");
	ccm_base = base;
}

void __init imx6_pm_init(void)
{
	unsigned long suspend_code_size;
	struct device_node *np;
	struct resource res;
	unsigned long i;

	if (!iram_tlb_base_addr) {
		pr_warn("No IRAM/OCRAM memory allocated for suspend/resume code. \
Please ensure device tree has an entry fsl,lpm-sram\n");
			return;
	}

	/* Set all IRAM page table entries to 0. */
	memset((void *)iram_tlb_base_addr, 0, MX6Q_IRAM_TLB_SIZE);
	/*
	 * Make sure the IRAM virtual address has a mapping
	 * in the IRAM page table.
	 * Only use the top 11 bits [31-20] when storing the
	 * physical address in the page table as only these
	 * bits are required for 1M mapping.
	 */
	i = ((iram_tlb_base_addr >> 20) << 2) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		(iram_tlb_phys_addr & 0xFFF00000) | TT_ATTRIB_NON_CACHEABLE_1M;
	/*
	 * Make sure the AIPS1 virtual address has a mapping
	 * in the IRAM page table.
	 */
	i = ((IMX_IO_P2V(MX6Q_AIPS1_BASE_ADDR) >> 20) << 2) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		(MX6Q_AIPS1_BASE_ADDR  & 0xFFF00000) | TT_ATTRIB_NON_CACHEABLE_1M;
	/*
	 * Make sure the AIPS2 virtual address has a mapping
	 * in the IRAM page table.
	 */
	i = ((IMX_IO_P2V(MX6Q_AIPS2_BASE_ADDR) >> 20) << 2) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		(MX6Q_AIPS2_BASE_ADDR  & 0xFFF00000) | TT_ATTRIB_NON_CACHEABLE_1M;
	/*
	 * Make sure the AIPS3 virtual address has a mapping
	 * in the IRAM page table.
	 */
	i = ((IMX_IO_P2V(MX6Q_AIPS3_BASE_ADDR) >> 20) << 2) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		(MX6Q_AIPS3_BASE_ADDR & 0xFFF00000) |
		TT_ATTRIB_NON_CACHEABLE_1M;
	/*
	 * Make sure the AIPS2 virtual address has a mapping
	 * in the IRAM page table.
	 */
	i = ((IMX_IO_P2V(MX6Q_L2_BASE_ADDR) >> 20) << 2) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		(MX6Q_L2_BASE_ADDR  & 0xFFF00000) | TT_ATTRIB_NON_CACHEABLE_1M;

	iram_paddr = iram_tlb_phys_addr + MX6_SUSPEND_IRAM_ADDR_OFFSET;

	/* Make sure iram_paddr is 8 byte aligned. */
	if ((uintptr_t)(iram_paddr) & (FNCPY_ALIGN - 1))
		iram_paddr += FNCPY_ALIGN - iram_paddr % (FNCPY_ALIGN);

	/* Get the virtual address of the suspend code. */
	suspend_iram_base = (void *)IMX_IO_P2V(iram_paddr);

	suspend_code_size = (&imx6_suspend_end -&imx6_suspend_start) *4;
	suspend_in_iram_fn = (void *)fncpy(suspend_iram_base,
		&imx6_suspend, suspend_code_size);

	/* Now add the space used for storing various registers and IO in suspend. */
	total_suspend_size = suspend_code_size + MX6_SUSPEND_IRAM_DATA_SIZE;

	suspend_set_ops(&imx6_pm_ops);

	/* Set cpu_type for DSM */
	if (cpu_is_imx6q())
		cpu_type = MXC_CPU_IMX6Q;
	else if (cpu_is_imx6dl())
		cpu_type = MXC_CPU_IMX6DL;
	else if (cpu_is_imx6sl())
		cpu_type = MXC_CPU_IMX6SL;
	else {
		cpu_type = MXC_CPU_IMX6SX;
		if (imx_get_soc_revision() < IMX_CHIP_REVISION_1_2) {
			/*
			 * As there is a 16K OCRAM(start from 0x8f8000)
			 * dedicated for low power function on i.MX6SX,
			 * but ROM did NOT do the ocram address change
			 * accordingly, so we need to add a data patch
			 * to workaround this issue, otherwise, system
			 * will fail to resume from DSM mode. TO1.2 fixes
			 * this issue.
			 */
			romcp = syscon_regmap_lookup_by_compatible(
				"fsl,imx6sx-romcp");
			if (IS_ERR(romcp)) {
				pr_err("failed to find fsl,imx6sx-romcp regmap\n");
				return;
			}
			regmap_write(romcp, ROMC_ROMPATCH0D, iram_paddr);
			regmap_update_bits(romcp, ROMC_ROMPATCHCNTL,
				BM_ROMPATCHCNTL_0D, BM_ROMPATCHCNTL_0D);
			regmap_update_bits(romcp, ROMC_ROMPATCHENL,
				BM_ROMPATCHENL_0D, BM_ROMPATCHENL_0D);
			regmap_write(romcp, ROMC_ROMPATCH0A,
				ROM_ADDR_FOR_INTERNAL_RAM_BASE);
			regmap_update_bits(romcp, ROMC_ROMPATCHCNTL,
				BM_ROMPATCHCNTL_DIS, ~BM_ROMPATCHCNTL_DIS);
		}
		np = of_find_compatible_node(NULL, NULL, "fsl,mega-fast-sram");
		ocram_base = of_iomap(np, 0);
		WARN_ON(!ocram_base);
		WARN_ON(of_address_to_resource(np, 0, &res));
		ocram_size = resource_size(&res);
		ocram_saved_in_ddr = kzalloc(ocram_size, GFP_KERNEL);
		WARN_ON(!ocram_saved_in_ddr);

		np = of_find_node_by_path("/soc/aips-bus@02000000/spba-bus@02000000/serial@02020000");
		if (np)
			console_base = of_iomap(np, 0);
		if (imx_src_is_m4_enabled()) {
			np = of_find_compatible_node(NULL, NULL,
					"fsl,imx6sx-qspi-m4-restore");
			if (np)
				qspi_base = of_iomap(np, 0);
			WARN_ON(!qspi_base);
		}
	}
}
