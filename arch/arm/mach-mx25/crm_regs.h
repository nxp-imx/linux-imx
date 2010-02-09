/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ARCH_ARM_MACH_MX25_CRM_REGS_H__
#define __ARCH_ARM_MACH_MX25_CRM_REGS_H__

#include <mach/hardware.h>

#define MXC_CCM_BASE			((char *)IO_ADDRESS(CCM_BASE_ADDR))

/* Register offsets */
#define MXC_CCM_MPCTL               (MXC_CCM_BASE + 0x00)
#define MXC_CCM_UPCTL               (MXC_CCM_BASE + 0x04)
#define MXC_CCM_CCTL                (MXC_CCM_BASE + 0x08)
#define MXC_CCM_CGCR0               (MXC_CCM_BASE + 0x0C)
#define MXC_CCM_CGCR1               (MXC_CCM_BASE + 0x10)
#define MXC_CCM_CGCR2               (MXC_CCM_BASE + 0x14)
#define MXC_CCM_PCDR0               (MXC_CCM_BASE + 0x18)
#define MXC_CCM_PCDR1               (MXC_CCM_BASE + 0x1C)
#define MXC_CCM_PCDR2               (MXC_CCM_BASE + 0x20)
#define MXC_CCM_PCDR3               (MXC_CCM_BASE + 0x24)
#define MXC_CCM_RCSR                (MXC_CCM_BASE + 0x28)
#define MXC_CCM_CRDR                (MXC_CCM_BASE + 0x2C)
#define MXC_CCM_DCVR0               (MXC_CCM_BASE + 0x30)
#define MXC_CCM_DCVR1               (MXC_CCM_BASE + 0x34)
#define MXC_CCM_DCVR2               (MXC_CCM_BASE + 0x38)
#define MXC_CCM_DCVR3               (MXC_CCM_BASE + 0x3C)
#define MXC_CCM_LTR0                (MXC_CCM_BASE + 0x40)
#define MXC_CCM_LTR1                (MXC_CCM_BASE + 0x44)
#define MXC_CCM_LTR2                (MXC_CCM_BASE + 0x48)
#define MXC_CCM_LTR3                (MXC_CCM_BASE + 0x4C)
#define MXC_CCM_LTBR0               (MXC_CCM_BASE + 0x50)
#define MXC_CCM_LTBR1               (MXC_CCM_BASE + 0x54)
#define MXC_CCM_PMCR0               (MXC_CCM_BASE + 0x58)
#define MXC_CCM_PMCR1               (MXC_CCM_BASE + 0x5C)
#define MXC_CCM_PMCR2               (MXC_CCM_BASE + 0x60)
#define MXC_CCM_MCR                 (MXC_CCM_BASE + 0x64)
#define MXC_CCM_LPIMR0              (MXC_CCM_BASE + 0x68)
#define MXC_CCM_LPIMR1              (MXC_CCM_BASE + 0x6C)

#define MXC_CCM_MPCTL_BRMO          (1 << 31)
#define MXC_CCM_MPCTL_PD_OFFSET     26
#define MXC_CCM_MPCTL_PD_MASK       (0xf << 26)
#define MXC_CCM_MPCTL_MFD_OFFSET    16
#define MXC_CCM_MPCTL_MFD_MASK      (0x3ff << 16)
#define MXC_CCM_MPCTL_MFI_OFFSET    10
#define MXC_CCM_MPCTL_MFI_MASK      (0xf << 10)
#define MXC_CCM_MPCTL_MFN_OFFSET    0
#define MXC_CCM_MPCTL_MFN_MASK      0x3ff
#define MXC_CCM_MPCTL_LF            (1 << 15)

#define MXC_CCM_UPCTL_BRMO          (1 << 31)
#define MXC_CCM_UPCTL_PD_OFFSET     26
#define MXC_CCM_UPCTL_PD_MASK       (0xf << 26)
#define MXC_CCM_UPCTL_MFD_OFFSET    16
#define MXC_CCM_UPCTL_MFD_MASK      (0x3ff << 16)
#define MXC_CCM_UPCTL_MFI_OFFSET    10
#define MXC_CCM_UPCTL_MFI_MASK      (0xf << 10)
#define MXC_CCM_UPCTL_MFN_OFFSET    0
#define MXC_CCM_UPCTL_MFN_MASK      0x3ff
#define MXC_CCM_UPCTL_LF            (1 << 15)

#define MXC_CCM_CCTL_ARM_OFFSET     30
#define MXC_CCM_CCTL_ARM_MASK       (0x3 << 30)
#define MXC_CCM_CCTL_AHB_OFFSET     28
#define MXC_CCM_CCTL_AHB_MASK       (0x3 << 28)
#define MXC_CCM_CCTL_MPLL_RST       (1 << 27)
#define MXC_CCM_CCTL_UPLL_RST       (1 << 26)
#define MXC_CCM_CCTL_LP_CTL_OFFSET  24
#define MXC_CCM_CCTL_LP_CTL_MASK    (0x3 << 24)
#define MXC_CCM_CCTL_LP_MODE_RUN    (0x0 << 24)
#define MXC_CCM_CCTL_LP_MODE_WAIT   (0x1 << 24)
#define MXC_CCM_CCTL_LP_MODE_DOZE   (0x2 << 24)
#define MXC_CCM_CCTL_LP_MODE_STOP   (0x3 << 24)
#define MXC_CCM_CCTL_UPLL_DISABLE   (1 << 23)
#define MXC_CCM_CCTL_MPLL_BYPASS    (1 << 22)
#define MXC_CCM_CCTL_USB_DIV_OFFSET 16
#define MXC_CCM_CCTL_USB_DIV_MASK   (0x3 << 16)
#define MXC_CCM_CCTL_CG_CTRL        (1 << 15)
#define MXC_CCM_CCTL_ARM_SRC        (1 << 14)
#define MXC_CCM_CCTL_ARM_SRC_OFFSET	14

#define MXC_CCM_CGCR0_HCLK_ATA_OFFSET    16
#define MXC_CCM_CGCR0_HCLK_BROM_OFFSET   17
#define MXC_CCM_CGCR0_HCLK_CSI_OFFSET    18
#define MXC_CCM_CGCR0_HCLK_EMI_OFFSET    19
#define MXC_CCM_CGCR0_HCLK_ESAI_OFFSET   20
#define MXC_CCM_CGCR0_HCLK_ESDHC1_OFFSET 21
#define MXC_CCM_CGCR0_HCLK_ESDHC2_OFFSET 22
#define MXC_CCM_CGCR0_HCLK_FEC_OFFSET    23
#define MXC_CCM_CGCR0_HCLK_LCDC_OFFSET   24
#define MXC_CCM_CGCR0_HCLK_RTIC_OFFSET   25
#define MXC_CCM_CGCR0_HCLK_SDMA_OFFSET   26
#define MXC_CCM_CGCR0_HCLK_SLCDC_OFFSET  27
#define MXC_CCM_CGCR0_HCLK_USBOTG_OFFSET 28

#define MXC_CCM_CGCR0_PER_CSI_OFFSET     0
#define MXC_CCM_CGCR0_PER_EPIT_OFFSET    1
#define MXC_CCM_CGCR0_PER_ESAI_OFFSET    2
#define MXC_CCM_CGCR0_PER_ESDHC1_OFFSET  3
#define MXC_CCM_CGCR0_PER_ESDHC2_OFFSET  4
#define MXC_CCM_CGCR0_PER_GPT_OFFSET     5
#define MXC_CCM_CGCR0_PER_I2C_OFFSET     6
#define MXC_CCM_CGCR0_PER_LCDC_OFFSET    7
#define MXC_CCM_CGCR0_PER_NFC_OFFSET     8
#define MXC_CCM_CGCR0_PER_OWIRE_OFFSET   9
#define MXC_CCM_CGCR0_PER_PWM_OFFSET     10
#define MXC_CCM_CGCR0_PER_SIM1_OFFSET    11
#define MXC_CCM_CGCR0_PER_SIM2_OFFSET    12
#define MXC_CCM_CGCR0_PER_SSI1_OFFSET    13
#define MXC_CCM_CGCR0_PER_SSI2_OFFSET    14
#define MXC_CCM_CGCR0_PER_UART_OFFSET    15

#define MXC_CCM_CGCR1_AUDMUX_OFFSET      0
#define MXC_CCM_CGCR1_ATA_OFFSET         1
#define MXC_CCM_CGCR1_CAN1_OFFSET        2
#define MXC_CCM_CGCR1_CAN2_OFFSET        3
#define MXC_CCM_CGCR1_CSI_OFFSET         4
#define MXC_CCM_CGCR1_CSPI1_OFFSET       5
#define MXC_CCM_CGCR1_CSPI2_OFFSET       6
#define MXC_CCM_CGCR1_CSPI3_OFFSET       7
#define MXC_CCM_CGCR1_DRYICE_OFFSET      8
#define MXC_CCM_CGCR1_ECT_OFFSET         9
#define MXC_CCM_CGCR1_EPIT1_OFFSET       10
#define MXC_CCM_CGCR1_EPIT2_OFFSET       11
#define MXC_CCM_CGCR1_ESAI_OFFSET        12
#define MXC_CCM_CGCR1_ESDHC1_OFFSET      13
#define MXC_CCM_CGCR1_ESDHC2_OFFSET      14
#define MXC_CCM_CGCR1_FEC_OFFSET         15
#define MXC_CCM_CGCR1_GPIO1_OFFSET       16
#define MXC_CCM_CGCR1_GPIO2_OFFSET       17
#define MXC_CCM_CGCR1_GPIO3_OFFSET       18
#define MXC_CCM_CGCR1_GPT1_OFFSET        19
#define MXC_CCM_CGCR1_GPT2_OFFSET        20
#define MXC_CCM_CGCR1_GPT3_OFFSET        21
#define MXC_CCM_CGCR1_GPT4_OFFSET        22
#define MXC_CCM_CGCR1_I2C1_OFFSET        23
#define MXC_CCM_CGCR1_I2C2_OFFSET        24
#define MXC_CCM_CGCR1_I2C3_OFFSET        25
#define MXC_CCM_CGCR1_IIM_OFFSET         26
#define MXC_CCM_CGCR1_IOMUXC_OFFSET      27
#define MXC_CCM_CGCR1_KPP_OFFSET         28
#define MXC_CCM_CGCR1_LCDC_OFFSET        29
#define MXC_CCM_CGCR1_OWIRE_OFFSET       30
#define MXC_CCM_CGCR1_PWM1_OFFSET        31

#define MXC_CCM_CGCR2_PWM2_OFFSET        (32-32)
#define MXC_CCM_CGCR2_PWM3_OFFSET        (33-32)
#define MXC_CCM_CGCR2_PWM4_OFFSET        (34-32)
#define MXC_CCM_CGCR2_RNGB_OFFSET        (35-32)
#define MXC_CCM_CGCR2_RTIC_OFFSET        (36-32)
#define MXC_CCM_CGCR2_SCC_OFFSET         (37-32)
#define MXC_CCM_CGCR2_SDMA_OFFSET        (38-32)
#define MXC_CCM_CGCR2_SIM1_OFFSET        (39-32)
#define MXC_CCM_CGCR2_SIM2_OFFSET        (40-32)
#define MXC_CCM_CGCR2_SLCDC_OFFSET       (41-32)
#define MXC_CCM_CGCR2_SPBA_OFFSET        (42-32)
#define MXC_CCM_CGCR2_SSI1_OFFSET        (43-32)
#define MXC_CCM_CGCR2_SSI2_OFFSET        (44-32)
#define MXC_CCM_CGCR2_TCHSCRN_OFFSET     (45-32)
#define MXC_CCM_CGCR2_UART1_OFFSET       (46-32)
#define MXC_CCM_CGCR2_UART2_OFFSET       (47-32)
#define MXC_CCM_CGCR2_UART3_OFFSET       (48-32)
#define MXC_CCM_CGCR2_UART4_OFFSET       (49-32)
#define MXC_CCM_CGCR2_UART5_OFFSET       (50-32)
#define MXC_CCM_CGCR2_WDOG_OFFSET        (51-32)

#define MXC_CCM_CGCR0_STOP_MODE_MASK	\
			((1 << MXC_CCM_CGCR0_HCLK_SLCDC_OFFSET) | \
			 (1 << MXC_CCM_CGCR0_HCLK_RTIC_OFFSET) | \
			 (1 << MXC_CCM_CGCR0_HCLK_EMI_OFFSET) | \
			 (1 << MXC_CCM_CGCR0_HCLK_BROM_OFFSET))

#define MXC_CCM_CGCR1_STOP_MODE_MASK	((1 << MXC_CCM_CGCR1_IIM_OFFSET) | \
					 (1 << MXC_CCM_CGCR1_CAN2_OFFSET) | \
					 (1 << MXC_CCM_CGCR1_CAN1_OFFSET))

#define MXC_CCM_CGCR2_STOP_MODE_MASK	((1 << MXC_CCM_CGCR2_SPBA_OFFSET) | \
					 (1 << MXC_CCM_CGCR2_SDMA_OFFSET) | \
					 (1 << MXC_CCM_CGCR2_RTIC_OFFSET))

#define MXC_CCM_PCDR1_PERDIV1_MASK       0x3f

#define MXC_CCM_RCSR_NF16B               (1 << 14)

#define MXC_CCM_PMCR1_CPEN_EMI		(1 << 29)
#define MXC_CCM_PMCR1_CSPAEM_P_OFFSET	26
#define MXC_CCM_PMCR1_CSPAEM_N_OFFSET	24
#define MXC_CCM_PMCR1_CSPAEM_MASK	(0xf << 24)
#define MXC_CCM_PMCR1_WBCN_OFFSET	16
#define MXC_CCM_PMCR1_CPEN		(1 << 13)
#define MXC_CCM_PMCR1_CSPA_P_OFFSET	11
#define MXC_CCM_PMCR1_CSPA_N_OFFSET	9
#define MXC_CCM_PMCR1_CSPA_MASK		(0xf << 9)

#define MXC_CCM_PMCR1_WBCN_MASK		(0xff << 16)
#define MXC_CCM_PMCR1_WBCN_DEFAULT	0xa0
#define MXC_CCM_PMCR1_WBB_INCR		0
#define MXC_CCM_PMCR1_WBB_MODE		1
#define MXC_CCM_PMCR1_WBB_DECR		2
#define MXC_CCM_PMCR1_WBB_MINI		3

#define MXC_CCM_PMCR2_VSTBY		(1 << 17)
#define MXC_CCM_PMCR2_OSC24M_DOWN	(1 << 16)

#define MXC_CCM_PMCR1_AWB_EN		(MXC_CCM_PMCR1_CPEN_EMI | \
					 MXC_CCM_PMCR1_CPEN | \
					 (MXC_CCM_PMCR1_WBCN_DEFAULT << \
					 MXC_CCM_PMCR1_WBCN_OFFSET))

#define MXC_CCM_PMCR1_WBB_DEFAULT	((MXC_CCM_PMCR1_WBB_DECR << \
					 MXC_CCM_PMCR1_CSPAEM_P_OFFSET) | \
					 (MXC_CCM_PMCR1_WBB_DECR << \
					 MXC_CCM_PMCR1_CSPAEM_N_OFFSET) | \
					 (MXC_CCM_PMCR1_WBB_DECR << \
					 MXC_CCM_PMCR1_CSPA_P_OFFSET) | \
					 (MXC_CCM_PMCR1_WBB_DECR << \
					 MXC_CCM_PMCR1_CSPA_N_OFFSET))

#define MXC_CCM_PMCR1_AWB_DEFAULT	(MXC_CCM_PMCR1_AWB_EN | \
					 MXC_CCM_PMCR1_WBB_DEFAULT)

#define MXC_CCM_MCR_USB_XTAL_MUX_OFFSET  31
#define MXC_CCM_MCR_CLKO_EN_OFFSET       30
#define MXC_CCM_MCR_CLKO_DIV_OFFSET      24
#define MXC_CCM_MCR_CLKO_DIV_MASK        (0x3F << 24)
#define MXC_CCM_MCR_CLKO_SEL_OFFSET      20
#define MXC_CCM_MCR_CLKO_SEL_MASK        (0xF << 20)
#define MXC_CCM_MCR_ESAI_CLK_MUX_OFFSET  19
#define MXC_CCM_MCR_SSI2_CLK_MUX_OFFSET  18
#define MXC_CCM_MCR_SSI1_CLK_MUX_OFFSET  17
#define MXC_CCM_MCR_USB_CLK_MUX_OFFSET   16

#define MXC_CCM_MCR_PER_CLK_MUX_MASK     (0xFFFF << 0)

#define MXC_CCM_LPIMR0_MASK		0xFFFFFFFF
#define MXC_CCM_LPIMR1_MASK		0xFFFFFFFF

#endif				/* __ARCH_ARM_MACH_MX25_CRM_REGS_H__ */
