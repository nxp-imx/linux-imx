/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ARCH_ARM_MACH_MX35_CRM_REGS_H__
#define __ARCH_ARM_MACH_MX35_CRM_REGS_H__

#define CKIH_CLK_FREQ           24000000
#define CKIE_CLK_FREQ		24576000
#define CKIL_CLK_FREQ           32000
#define CKIL_EXT_FREQ		32768

#define MXC_CCM_BASE		((char *)IO_ADDRESS(CCM_BASE_ADDR))

/* Register addresses */
#define MXC_CCM_CCMR		(MXC_CCM_BASE + 0x00)
#define MXC_CCM_PDR0		(MXC_CCM_BASE + 0x04)
#define MXC_CCM_PDR1		(MXC_CCM_BASE + 0x08)
#define MXC_CCM_PDR2		(MXC_CCM_BASE + 0x0C)
#define MXC_CCM_PDR3		(MXC_CCM_BASE + 0x10)
#define MXC_CCM_PDR4		(MXC_CCM_BASE + 0x14)
#define MXC_CCM_RCSR		(MXC_CCM_BASE + 0x18)
#define MXC_CCM_MPCTL		(MXC_CCM_BASE + 0x1C)
#define MXC_CCM_PPCTL		(MXC_CCM_BASE + 0x20)
#define MXC_CCM_ACMR		(MXC_CCM_BASE + 0x24)
#define MXC_CCM_COSR		(MXC_CCM_BASE + 0x28)
#define MXC_CCM_CGR0		(MXC_CCM_BASE + 0x2C)
#define MXC_CCM_CGR1		(MXC_CCM_BASE + 0x30)
#define MXC_CCM_CGR2		(MXC_CCM_BASE + 0x34)
#define MXC_CCM_CGR3		(MXC_CCM_BASE + 0x38)
#define MXC_CCM_RESV		(MXC_CCM_BASE + 0x3C)
#define MXC_CCM_DCVR0		(MXC_CCM_BASE + 0x40)
#define MXC_CCM_DCVR1		(MXC_CCM_BASE + 0x44)
#define MXC_CCM_DCVR2		(MXC_CCM_BASE + 0x48)
#define MXC_CCM_DCVR3		(MXC_CCM_BASE + 0x4C)
#define MXC_CCM_LTR0		(MXC_CCM_BASE + 0x50)
#define MXC_CCM_LTR1		(MXC_CCM_BASE + 0x54)
#define MXC_CCM_LTR2		(MXC_CCM_BASE + 0x58)
#define MXC_CCM_LTR3		(MXC_CCM_BASE + 0x5C)
#define MXC_CCM_LTBR0		(MXC_CCM_BASE + 0x60)
#define MXC_CCM_LTBR1		(MXC_CCM_BASE + 0x64)
#define MXC_CCM_PMCR0		(MXC_CCM_BASE + 0x68)
#define MXC_CCM_PMCR1		(MXC_CCM_BASE + 0x6C)
#define MXC_CCM_PMCR2		(MXC_CCM_BASE + 0x70)

/* Register bit definitions */
#define MXC_CCM_CCMR_WFI                        (1 << 30)
#define MXC_CCM_CCMR_STBY_EXIT_SRC              (1 << 29)
#define MXC_CCM_CCMR_VSTBY                      (1 << 28)
#define MXC_CCM_CCMR_WBEN                       (1 << 27)
#define MXC_CCM_CCMR_VOL_RDY_CNT_OFFSET        20
#define MXC_CCM_CCMR_VOL_RDY_CNT_MASK          (0xF << 20)
#define MXC_CCM_CCMR_ROMW_OFFSET               18
#define MXC_CCM_CCMR_ROMW_MASK                 (0x3 << 18)
#define MXC_CCM_CCMR_RAMW_OFFSET               21
#define MXC_CCM_CCMR_RAMW_MASK                 (0x3 << 21)
#define MXC_CCM_CCMR_LPM_OFFSET                 14
#define MXC_CCM_CCMR_LPM_MASK                   (0x3 << 14)
#define MXC_CCM_CCMR_UPE                        (1 << 9)
#define MXC_CCM_CCMR_MPE                        (1 << 3)

#define MXC_CCM_PDR0_PER_SEL			(1 << 26)
#define MXC_CCM_PDR0_IPU_HND_BYP                (1 << 23)
#define MXC_CCM_PDR0_HSP_PODF_OFFSET            20
#define MXC_CCM_PDR0_HSP_PODF_MASK              (0x3 << 20)
#define MXC_CCM_PDR0_CON_MUX_DIV_OFFSET       	16
#define MXC_CCM_PDR0_CON_MUX_DIV_MASK           (0xF << 16)
#define MXC_CCM_PDR0_CKIL_SEL			(1 << 15)
#define MXC_CCM_PDR0_PER_PODF_OFFSET            12
#define MXC_CCM_PDR0_PER_PODF_MASK              (0xF << 12)
#define MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET        9
#define MXC_CCM_PDR0_AUTO_MUX_DIV_MASK          (0x7 << 9)
#define MXC_CCM_PDR0_AUTO_CON	                0x1

#define MXC_CCM_PDR1_MSHC_PRDF_OFFSET           28
#define MXC_CCM_PDR1_MSHC_PRDF_MASK             (0x7 << 28)
#define MXC_CCM_PDR1_MSHC_PODF_OFFSET           22
#define MXC_CCM_PDR1_MSHC_PODF_MASK             (0x3F << 22)
#define MXC_CCM_PDR1_MSHC_M_U 			(1 << 7)

#define MXC_CCM_PDR2_SSI2_PRDF_OFFSET           27
#define MXC_CCM_PDR2_SSI2_PRDF_MASK             (0x7 << 27)
#define MXC_CCM_PDR2_SSI1_PRDF_OFFSET           24
#define MXC_CCM_PDR2_SSI1_PRDF_MASK             (0x7 << 24)
#define MXC_CCM_PDR2_CSI_PRDF_OFFSET            19
#define MXC_CCM_PDR2_CSI_PRDF_MASK              (0x7 << 19)
#define MXC_CCM_PDR2_CSI_PODF_OFFSET            16
#define MXC_CCM_PDR2_CSI_PODF_MASK              (0x7 << 16)
#define MXC_CCM_PDR2_SSI2_PODF_OFFSET           8
#define MXC_CCM_PDR2_SSI2_PODF_MASK             (0x3F << 8)
#define MXC_CCM_PDR2_CSI_M_U			(1 << 7)
#define MXC_CCM_PDR2_SSI_M_U			(1 << 6)
#define MXC_CCM_PDR2_SSI1_PODF_OFFSET           0
#define MXC_CCM_PDR2_SSI1_PODF_MASK             (0x3F)

/* Extra definitions for Chip Version 2*/
#define MXC_CCM_PDR2_CSI_PODF_MASK_V2              (0x3F << 16)

#define MXC_CCM_PDR3_SPDIF_PRDF_OFFSET          29
#define MXC_CCM_PDR3_SPDIF_PRDF_MASK            (0x7 << 29)
#define MXC_CCM_PDR3_SPDIF_PODF_OFFSET          23
#define MXC_CCM_PDR3_SPDIF_PODF_MASK            (0x3F << 23)
#define MXC_CCM_PDR3_SPDIF_M_U			(1 << 22)
#define MXC_CCM_PDR3_ESDHC3_PRDF_OFFSET         19
#define MXC_CCM_PDR3_ESDHC3_PRDF_MASK           (0x7 << 19)
#define MXC_CCM_PDR3_ESDHC3_PODF_OFFSET         16
#define MXC_CCM_PDR3_ESDHC3_PODF_MASK           (0x7 << 16)
#define MXC_CCM_PDR3_UART_M_U			(1 << 15)
#define MXC_CCM_PDR3_ESDHC2_PRDF_OFFSET         11
#define MXC_CCM_PDR3_ESDHC2_PRDF_MASK           (0x7 << 11)
#define MXC_CCM_PDR3_ESDHC2_PODF_OFFSET         8
#define MXC_CCM_PDR3_ESDHC2_PODF_MASK           (0x7 << 8)
#define MXC_CCM_PDR3_ESDHC_M_U			(1 << 6)
#define MXC_CCM_PDR3_ESDHC1_PRDF_OFFSET         3
#define MXC_CCM_PDR3_ESDHC1_PRDF_MASK           (0x7 << 3)
#define MXC_CCM_PDR3_ESDHC1_PODF_OFFSET         0
#define MXC_CCM_PDR3_ESDHC1_PODF_MASK           (0x7)

/* Extra definitions for Chip Version 2 */
#define MXC_CCM_PDR3_ESDHC3_PODF_MASK_V2	(0x3F << 16)
#define MXC_CCM_PDR3_ESDHC2_PODF_MASK_V2        (0x3F << 8)
#define MXC_CCM_PDR3_ESDHC1_PODF_MASK_V2        0x3F

#define MXC_CCM_PDR4_NFC_PODF_OFFSET           	28
#define MXC_CCM_PDR4_NFC_PODF_MASK            	(0xF << 28)
#define MXC_CCM_PDR4_USB_PRDF_OFFSET          	25
#define MXC_CCM_PDR4_USB_PRDF_MASK            	(0x7 << 25)
#define MXC_CCM_PDR4_USB_PODF_OFFSET          	22
#define MXC_CCM_PDR4_USB_PODF_MASK            	(0x7 << 22)
#define MXC_CCM_PDR4_PER0_PRDF_OFFSET          	19
#define MXC_CCM_PDR4_PER0_PRDF_MASK            	(0x7 << 19)
#define MXC_CCM_PDR4_PER0_PODF_OFFSET          	16
#define MXC_CCM_PDR4_PER0_PODF_MASK            	(0x7 << 16)
#define MXC_CCM_PDR4_UART_PRDF_OFFSET          	13
#define MXC_CCM_PDR4_UART_PRDF_MASK            	(0x7 << 13)
#define MXC_CCM_PDR4_UART_PODF_OFFSET          	10
#define MXC_CCM_PDR4_UART_PODF_MASK            	(0x7 << 10)
#define MXC_CCM_PDR4_USB_M_U			(1 << 9)

/* Extra definitions for Chip Version 2 */
#define MXC_CCM_PDR4_USB_PODF_MASK_V2		(0x3F << 22)
#define MXC_CCM_PDR4_PER0_PODF_MASK_V2		(0x3F << 16)
#define MXC_CCM_PDR4_UART_PODF_MASK_V2          (0x3F << 10)

/* Bit definitions for RCSR */
#define MXC_CCM_RCSR_BUS_WIDTH			(1 << 29)
#define MXC_CCM_RCSR_BUS_16BIT			(1 << 29)
#define MXC_CCM_RCSR_PAGE_SIZE			(3 << 27)
#define MXC_CCM_RCSR_PAGE_512			(0 << 27)
#define MXC_CCM_RCSR_PAGE_2K			(1 << 27)
#define MXC_CCM_RCSR_PAGE_4K1			(2 << 27)
#define MXC_CCM_RCSR_PAGE_4K2			(3 << 27)
#define MXC_CCM_RCSR_SOFT_RESET			(1 << 15)
#define MXC_CCM_RCSR_NF16B			(1 << 14)
#define MXC_CCM_RCSR_NFC_4K			(1 << 9)
#define MXC_CCM_RCSR_NFC_FMS			(1 << 8)

/* Bit definitions for both MCU, PERIPHERAL PLL control registers */
#define MXC_CCM_PCTL_BRM                        0x80000000
#define MXC_CCM_PCTL_PD_OFFSET                  26
#define MXC_CCM_PCTL_PD_MASK                    (0xF << 26)
#define MXC_CCM_PCTL_MFD_OFFSET                 16
#define MXC_CCM_PCTL_MFD_MASK                   (0x3FF << 16)
#define MXC_CCM_PCTL_MFI_OFFSET                 10
#define MXC_CCM_PCTL_MFI_MASK                   (0xF << 10)
#define MXC_CCM_PCTL_MFN_OFFSET                 0
#define MXC_CCM_PCTL_MFN_MASK                   0x3FF

/* Bit definitions for Audio clock mux register*/
#define MXC_CCM_ACMR_ESAI_CLK_SEL_OFFSET	12
#define MXC_CCM_ACMR_ESAI_CLK_SEL_MASK		(0xF << 12)
#define MXC_CCM_ACMR_SPDIF_CLK_SEL_OFFSET	8
#define MXC_CCM_ACMR_SPDIF_CLK_SEL_MASK		(0xF << 8)
#define MXC_CCM_ACMR_SSI1_CLK_SEL_OFFSET	4
#define MXC_CCM_ACMR_SSI1_CLK_SEL_MASK		(0xF << 4)
#define MXC_CCM_ACMR_SSI2_CLK_SEL_OFFSET	0
#define MXC_CCM_ACMR_SSI2_CLK_SEL_MASK		(0xF << 0)

/* Extra definitions for Version 2 */
#define MXC_CCM_ACMR_CKILH_PODF0_OFFSET		16
#define MXC_CCM_ACMR_CKILH_PODF1_OFFSET		19
#define MXC_CCM_ACMR_CKILH_PODF2_OFFSET		22
#define MXC_CCM_ACMR_CKILH_PODF3_OFFSET         25
#define MXC_CCM_ACMR_CKILH_PODF_MASK		0x7

/* Bit definitions for Clock gating Register*/
#define MXC_CCM_CGR0_ASRC_OFFSET             	0
#define MXC_CCM_CGR0_ASRC_MASK               	(0x3 << 0)
#define MXC_CCM_CGR0_ATA_OFFSET             	2
#define MXC_CCM_CGR0_ATA_MASK               	(0x3 << 2)
#define MXC_CCM_CGR0_CAN1_OFFSET                6
#define MXC_CCM_CGR0_CAN1_MASK                  (0x3 << 6)
#define MXC_CCM_CGR0_CAN2_OFFSET                8
#define MXC_CCM_CGR0_CAN2_MASK                  (0x3 << 8)
#define MXC_CCM_CGR0_CSPI1_OFFSET               10
#define MXC_CCM_CGR0_CSPI1_MASK                 (0x3 << 10)
#define MXC_CCM_CGR0_CSPI2_OFFSET               12
#define MXC_CCM_CGR0_CSPI2_MASK                 (0x3 << 12)
#define MXC_CCM_CGR0_ECT_OFFSET               	14
#define MXC_CCM_CGR0_ECT_MASK                 	(0x3 << 14)
#define MXC_CCM_CGR0_EMI_OFFSET               	18
#define MXC_CCM_CGR0_EMI_MASK                 	(0x3 << 18)
#define MXC_CCM_CGR0_EPIT1_OFFSET               20
#define MXC_CCM_CGR0_EPIT1_MASK                 (0x3 << 20)
#define MXC_CCM_CGR0_EPIT2_OFFSET               22
#define MXC_CCM_CGR0_EPIT2_MASK                 (0x3 << 22)
#define MXC_CCM_CGR0_ESAI_OFFSET                24
#define MXC_CCM_CGR0_ESAI_MASK                  (0x3 << 24)
#define MXC_CCM_CGR0_ESDHC1_OFFSET              26
#define MXC_CCM_CGR0_ESDHC1_MASK                (0x3 << 26)
#define MXC_CCM_CGR0_ESDHC2_OFFSET              28
#define MXC_CCM_CGR0_ESDHC2_MASK                (0x3 << 28)
#define MXC_CCM_CGR0_ESDHC3_OFFSET              30
#define MXC_CCM_CGR0_ESDHC3_MASK                (0x3 << 30)

#define MXC_CCM_CGR1_FEC_OFFSET              	0
#define MXC_CCM_CGR1_FEC_MASK                	(0x3 << 0)
#define MXC_CCM_CGR1_GPIO1_OFFSET           	2
#define MXC_CCM_CGR1_GPIO1_MASK             	(0x3 << 2)
#define MXC_CCM_CGR1_GPIO2_OFFSET           	4
#define MXC_CCM_CGR1_GPIO2_MASK             	(0x3 << 4)
#define MXC_CCM_CGR1_GPIO3_OFFSET               6
#define MXC_CCM_CGR1_GPIO3_MASK                 (0x3 << 6)
#define MXC_CCM_CGR1_GPT_OFFSET                 8
#define MXC_CCM_CGR1_GPT_MASK                   (0x3 << 8)
#define MXC_CCM_CGR1_I2C1_OFFSET                10
#define MXC_CCM_CGR1_I2C1_MASK                  (0x3 << 10)
#define MXC_CCM_CGR1_I2C2_OFFSET                12
#define MXC_CCM_CGR1_I2C2_MASK                  (0x3 << 12)
#define MXC_CCM_CGR1_I2C3_OFFSET                14
#define MXC_CCM_CGR1_I2C3_MASK                  (0x3 << 14)
#define MXC_CCM_CGR1_IOMUXC_OFFSET              16
#define MXC_CCM_CGR1_IOMUXC_MASK                (0x3 << 16)
#define MXC_CCM_CGR1_IPU_OFFSET              	18
#define MXC_CCM_CGR1_IPU_MASK                	(0x3 << 18)
#define MXC_CCM_CGR1_KPP_OFFSET                 20
#define MXC_CCM_CGR1_KPP_MASK                   (0x3 << 20)
#define MXC_CCM_CGR1_MLB_OFFSET                 22
#define MXC_CCM_CGR1_MLB_MASK                   (0x3 << 22)
#define MXC_CCM_CGR1_MSHC_OFFSET               	24
#define MXC_CCM_CGR1_MSHC_MASK                 	(0x3 << 24)
#define MXC_CCM_CGR1_OWIRE_OFFSET              	26
#define MXC_CCM_CGR1_OWIRE_MASK                 (0x3 << 26)
#define MXC_CCM_CGR1_PWM_OFFSET               	28
#define MXC_CCM_CGR1_PWM_MASK                 	(0x3 << 28)
#define MXC_CCM_CGR1_RNGC_OFFSET               	30
#define MXC_CCM_CGR1_RNGC_MASK                 	(0x3 << 30)

#define MXC_CCM_CGR2_RTC_OFFSET                	0
#define MXC_CCM_CGR2_RTC_MASK                  	(0x3 << 0)
#define MXC_CCM_CGR2_RTIC_OFFSET               	2
#define MXC_CCM_CGR2_RTIC_MASK                 	(0x3 << 2)
#define MXC_CCM_CGR2_SCC_OFFSET               	4
#define MXC_CCM_CGR2_SCC_MASK                 	(0x3 << 4)
#define MXC_CCM_CGR2_SDMA_OFFSET                6
#define MXC_CCM_CGR2_SDMA_MASK                  (0x3 << 6)
#define MXC_CCM_CGR2_SPBA_OFFSET                8
#define MXC_CCM_CGR2_SPBA_MASK                  (0x3 << 8)
#define MXC_CCM_CGR2_SPDIF_OFFSET               10
#define MXC_CCM_CGR2_SPDIF_MASK                 (0x3 << 10)
#define MXC_CCM_CGR2_SSI1_OFFSET                12
#define MXC_CCM_CGR2_SSI1_MASK                  (0x3 << 12)
#define MXC_CCM_CGR2_SSI2_OFFSET              	14
#define MXC_CCM_CGR2_SSI2_MASK                	(0x3 << 14)
#define MXC_CCM_CGR2_UART1_OFFSET              	16
#define MXC_CCM_CGR2_UART1_MASK                	(0x3 << 16)
#define MXC_CCM_CGR2_UART2_OFFSET              	18
#define MXC_CCM_CGR2_UART2_MASK                	(0x3 << 18)
#define MXC_CCM_CGR2_UART3_OFFSET              	20
#define MXC_CCM_CGR2_UART3_MASK                	(0x3 << 20)
#define MXC_CCM_CGR2_USBOTG_OFFSET             	22
#define MXC_CCM_CGR2_USBOTG_MASK                (0x3 << 22)
#define MXC_CCM_CGR2_WDOG_OFFSET              	24
#define MXC_CCM_CGR2_WDOG_MASK                	(0x3 << 24)
#define MXC_CCM_CGR2_MAX_OFFSET              	26
#define MXC_CCM_CGR2_MAX_MASK                	(0x3 << 26)
#define MXC_CCM_CGR2_MAX_ENABLE                	(0x2 << 26)
#define MXC_CCM_CGR2_AUDMUX_OFFSET              30
#define MXC_CCM_CGR2_AUDMUX_MASK                (0x3 << 30)

#define MXC_CCM_CGR3_CSI_OFFSET              	0
#define MXC_CCM_CGR3_CSI_MASK                	(0x3 << 0)
#define MXC_CCM_CGR3_IIM_OFFSET              	2
#define MXC_CCM_CGR3_IIM_MASK                	(0x3 << 2)
#define MXC_CCM_CGR3_GPU2D_OFFSET              	4
#define MXC_CCM_CGR3_GPU2D_MASK                	(0x3 << 4)
/*
 * LTR0 register offsets
 */
#define MXC_CCM_LTR0_DNTHR_OFFSET               16
#define MXC_CCM_LTR0_DNTHR_MASK                 (0x3F << 16)
#define MXC_CCM_LTR0_UPTHR_OFFSET               22
#define MXC_CCM_LTR0_UPTHR_MASK                 (0x3F << 22)
#define MXC_CCM_LTR0_DIV3CK_OFFSET              1
#define MXC_CCM_LTR0_DIV3CK_MASK                (0x3 << 1)

/*
 * LTR1 register offsets
 */
#define MXC_CCM_LTR1_PNCTHR_OFFSET              0
#define MXC_CCM_LTR1_PNCTHR_MASK                0x3F
#define MXC_CCM_LTR1_UPCNT_OFFSET               6
#define MXC_CCM_LTR1_UPCNT_MASK                 (0xFF << 6)
#define MXC_CCM_LTR1_DNCNT_OFFSET               14
#define MXC_CCM_LTR1_DNCNT_MASK                 (0xFF << 14)
#define MXC_CCM_LTR1_LTBRSR_MASK                0x400000
#define MXC_CCM_LTR1_LTBRSR_OFFSET              22
#define MXC_CCM_LTR1_LTBRSR                     0x400000
#define MXC_CCM_LTR1_LTBRSH                     0x800000

/*
 * LTR2 bit definitions. x ranges from 0 for WSW9 to 6 for WSW15
 */
#define MXC_CCM_LTR2_WSW_OFFSET(x)	(11 + (x) * 3)
#define MXC_CCM_LTR2_WSW_MASK(x)	(0x7 << MXC_CCM_LTR2_WSW_OFFSET((x)))
#define MXC_CCM_LTR2_EMAC_OFFSET        0
#define MXC_CCM_LTR2_EMAC_MASK          0x1FF

/*
 * LTR3 bit definitions. x ranges from 0 for WSW0 to 8 for WSW8
 */
#define MXC_CCM_LTR3_WSW_OFFSET(x)     	(5 + (x) * 3)
#define MXC_CCM_LTR3_WSW_MASK(x)       	(0x7 << MXC_CCM_LTR3_WSW_OFFSET((x)))

#define DVSUP_TURBO				0
#define DVSUP_HIGH				1
#define DVSUP_MEDIUM				2
#define DVSUP_LOW				3
#define MXC_CCM_PMCR0_DVSUP_TURBO               (DVSUP_TURBO << 28)
#define MXC_CCM_PMCR0_DVSUP_HIGH                (DVSUP_HIGH << 28)
#define MXC_CCM_PMCR0_DVSUP_MEDIUM              (DVSUP_MEDIUM << 28)
#define MXC_CCM_PMCR0_DVSUP_LOW                 (DVSUP_LOW << 28)
#define MXC_CCM_PMCR0_DVSUP_OFFSET              28
#define MXC_CCM_PMCR0_DVSUP_MASK                (0x3 << 28)
#define MXC_CCM_PMCR0_DVFS_UPDATE_FINISH        0x01000000
#define MXC_CCM_PMCR0_DVFEV                     0x00800000
#define MXC_CCM_PMCR0_DVFIS                     0x00400000
#define MXC_CCM_PMCR0_LBMI                      0x00200000
#define MXC_CCM_PMCR0_LBFL                      0x00100000
#define MXC_CCM_PMCR0_LBCF_4                    (0x0 << 18)
#define MXC_CCM_PMCR0_LBCF_8                    (0x1 << 18)
#define MXC_CCM_PMCR0_LBCF_12                   (0x2 << 18)
#define MXC_CCM_PMCR0_LBCF_16                   (0x3 << 18)
#define MXC_CCM_PMCR0_LBCF_OFFSET               18
#define MXC_CCM_PMCR0_LBCF_MASK                 (0x3 << 18)
#define MXC_CCM_PMCR0_PTVIS                     0x00020000
#define MXC_CCM_PMCR0_DVFS_START                0x00010000
#define MXC_CCM_PMCR0_DVFS_START_MASK           0x1 << 16)
#define MXC_CCM_PMCR0_FSVAIM                    0x00008000
#define MXC_CCM_PMCR0_FSVAI_OFFSET              13
#define MXC_CCM_PMCR0_FSVAI_MASK                (0x3 << 13)
#define MXC_CCM_PMCR0_DPVCR                     0x00001000
#define MXC_CCM_PMCR0_DPVV                      0x00000800
#define MXC_CCM_PMCR0_WFIM                      0x00000400
#define MXC_CCM_PMCR0_DRCE3                     0x00000200
#define MXC_CCM_PMCR0_DRCE2                     0x00000100
#define MXC_CCM_PMCR0_DRCE1                     0x00000080
#define MXC_CCM_PMCR0_DRCE0                     0x00000040
#define MXC_CCM_PMCR0_DCR                       0x00000020
#define MXC_CCM_PMCR0_DVFEN                     0x00000010
#define MXC_CCM_PMCR0_PTVAIM                    0x00000008
#define MXC_CCM_PMCR0_PTVAI_OFFSET              1
#define MXC_CCM_PMCR0_PTVAI_MASK                (0x3 << 1)
#define MXC_CCM_PMCR0_DPTEN                     0x00000001

#define MXC_CCM_PMCR1_DVGP_OFFSET               0
#define MXC_CCM_PMCR1_DVGP_MASK                 (0xF)

#define MXC_CCM_PMCR1_PLLRDIS                      (0x1 << 7)
#define MXC_CCM_PMCR1_EMIRQ_EN                      (0x1 << 8)

#define MXC_CCM_DCVR_ULV_MASK                   (0x3FF << 22)
#define MXC_CCM_DCVR_ULV_OFFSET                 22
#define MXC_CCM_DCVR_LLV_MASK                   (0x3FF << 12)
#define MXC_CCM_DCVR_LLV_OFFSET                 12
#define MXC_CCM_DCVR_ELV_MASK                   (0x3FF << 2)
#define MXC_CCM_DCVR_ELV_OFFSET                 2

#define MXC_CCM_PDR2_MST2_PDF_MASK              (0x3F << 7)
#define MXC_CCM_PDR2_MST2_PDF_OFFSET            7
#define MXC_CCM_PDR2_MST1_PDF_MASK              0x3F
#define MXC_CCM_PDR2_MST1_PDF_OFFSET            0

#define MXC_CCM_COSR_CLKOSEL_MASK               0x1F
#define MXC_CCM_COSR_CLKOSEL_OFFSET             0
#define MXC_CCM_COSR_CLKOEN                     (1 << 5)
#define MXC_CCM_COSR_CLKOUTDIV_1             	(1 << 6)
#define MXC_CCM_COSR_CLKOUT_PREDIV_MASK         (0x7 << 13)
#define MXC_CCM_COSR_CLKOUT_PREDIV_OFFSET       13
#define MXC_CCM_COSR_CLKOUT_PRODIV_MASK         (0x7 << 10)
#define MXC_CCM_COSR_CLKOUT_PRODIV_OFFSET       10
#define MXC_CCM_COSR_SSI1_RX_SRC_SEL_MASK       (0x3 << 16)
#define MXC_CCM_COSR_SSI1_RX_SRC_SEL_OFFSET     16
#define MXC_CCM_COSR_SSI1_TX_SRC_SEL_MASK       (0x3 << 18)
#define MXC_CCM_COSR_SSI1_TX_SRC_SEL_OFFSET     18
#define MXC_CCM_COSR_SSI2_RX_SRC_SEL_MASK       (0x3 << 20)
#define MXC_CCM_COSR_SSI2_RX_SRC_SEL_OFFSET     20
#define MXC_CCM_COSR_SSI2_TX_SRC_SEL_MASK       (0x3 << 22)
#define MXC_CCM_COSR_SSI2_TX_SRC_SEL_OFFSET     22
#define MXC_CCM_COSR_ASRC_AUDIO_EN              (1 << 24)
#define MXC_CCM_COSR_ASRC_AUDIO_PODF_MASK       (0x3F << 26)
#define MXC_CCM_COSR_ASRC_AUDIO_PODF_OFFSET     26

/* extra definitions for Version 2 */
#define MXC_CCM_COSR_CKIL_CKIH_MASK		(1 << 7)
#define MXC_CCM_COSR_CKIL_CKIH_OFFSET		7
#define MXC_CCM_COSR_CLKOUT_PRODIV_MASK_V2	(0x3F << 10)

/*
 * PMCR0 register offsets
 */
#define MXC_CCM_PMCR0_LBFL_OFFSET   20
#define MXC_CCM_PMCR0_DFSUP0_OFFSET 30
#define MXC_CCM_PMCR0_DFSUP1_OFFSET 31

/*
 * PMCR2 register definitions
 */
#define MXC_CCM_PMCR2_OSC24M_DOWN	(1 << 16)
#define MXC_CCM_PMCR2_OSC_AUDIO_DOWN	(1 << 17)

#endif				/* __ARCH_ARM_MACH_MX3_CRM_REGS_H__ */
