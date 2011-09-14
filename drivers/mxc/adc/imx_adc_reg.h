/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

#ifndef __IMX_ADC_H__
#define __IMX_ADC_H__

/* TSC General Config Register */
#define TGCR                  0x000
#define TGCR_IPG_CLK_EN       (1 << 0)
#define TGCR_TSC_RST          (1 << 1)
#define TGCR_FUNC_RST         (1 << 2)
#define TGCR_SLPC             (1 << 4)
#define TGCR_STLC             (1 << 5)
#define TGCR_HSYNC_EN         (1 << 6)
#define TGCR_HSYNC_POL        (1 << 7)
#define TGCR_POWERMODE_SHIFT  8
#define TGCR_POWER_OFF        (0x0 << TGCR_POWERMODE_SHIFT)
#define TGCR_POWER_SAVE       (0x1 << TGCR_POWERMODE_SHIFT)
#define TGCR_POWER_ON         (0x3 << TGCR_POWERMODE_SHIFT)
#define TGCR_POWER_MASK       (0x3 << TGCR_POWERMODE_SHIFT)
#define TGCR_INTREFEN         (1 << 10)
#define TGCR_ADCCLKCFG_SHIFT  16
#define TGCR_PD_EN            (1 << 23)
#define TGCR_PDB_EN           (1 << 24)
#define TGCR_PDBTIME_SHIFT    25
#define TGCR_PDBTIME128       (0x3f << TGCR_PDBTIME_SHIFT)
#define TGCR_PDBTIME_MASK     (0x7f << TGCR_PDBTIME_SHIFT)

/* TSC General Status Register */
#define TGSR                  0x004
#define TCQ_INT               (1 << 0)
#define GCQ_INT               (1 << 1)
#define SLP_INT               (1 << 2)
#define TCQ_DMA               (1 << 16)
#define GCQ_DMA               (1 << 17)

/* TSC IDLE Config Register */
#define TICR                  0x008

/* TouchScreen Convert Queue FIFO Register */
#define TCQFIFO               0x400
/* TouchScreen Convert Queue Control Register */
#define TCQCR                 0x404
#define CQCR_QSM_SHIFT        0
#define CQCR_QSM_STOP         (0x0 << CQCR_QSM_SHIFT)
#define CQCR_QSM_PEN          (0x1 << CQCR_QSM_SHIFT)
#define CQCR_QSM_FQS          (0x2 << CQCR_QSM_SHIFT)
#define CQCR_QSM_FQS_PEN      (0x3 << CQCR_QSM_SHIFT)
#define CQCR_QSM_MASK         (0x3 << CQCR_QSM_SHIFT)
#define CQCR_FQS              (1 << 2)
#define CQCR_RPT              (1 << 3)
#define CQCR_LAST_ITEM_ID_SHIFT   4
#define CQCR_LAST_ITEM_ID_MASK    (0xf << CQCR_LAST_ITEM_ID_SHIFT)
#define CQCR_FIFOWATERMARK_SHIFT  8
#define CQCR_FIFOWATERMARK_MASK   (0xf << CQCR_FIFOWATERMARK_SHIFT)
#define CQCR_REPEATWAIT_SHIFT     12
#define CQCR_REPEATWAIT_MASK      (0xf << CQCR_REPEATWAIT_SHIFT)
#define CQCR_QRST             (1 << 16)
#define CQCR_FRST             (1 << 17)
#define CQCR_PD_MSK           (1 << 18)
#define CQCR_PD_CFG           (1 << 19)

/* TouchScreen Convert Queue Status Register */
#define TCQSR                 0x408
#define CQSR_PD               (1 << 0)
#define CQSR_EOQ              (1 << 1)
#define CQSR_FOR              (1 << 4)
#define CQSR_FUR              (1 << 5)
#define CQSR_FER              (1 << 6)
#define CQSR_EMPT             (1 << 13)
#define CQSR_FULL             (1 << 14)
#define CQSR_FDRY             (1 << 15)

/* TouchScreen Convert Queue Mask Register */
#define TCQMR                 0x40c
#define TCQMR_PD_IRQ_MSK      (1 << 0)
#define TCQMR_EOQ_IRQ_MSK     (1 << 1)
#define TCQMR_FOR_IRQ_MSK     (1 << 4)
#define TCQMR_FUR_IRQ_MSK     (1 << 5)
#define TCQMR_FER_IRQ_MSK     (1 << 6)
#define TCQMR_PD_DMA_MSK      (1 << 16)
#define TCQMR_EOQ_DMA_MSK     (1 << 17)
#define TCQMR_FOR_DMA_MSK     (1 << 20)
#define TCQMR_FUR_DMA_MSK     (1 << 21)
#define TCQMR_FER_DMA_MSK     (1 << 22)
#define TCQMR_FDRY_DMA_MSK    (1 << 31)

/* TouchScreen Convert Queue ITEM 7~0 */
#define TCQ_ITEM_7_0          0x420

/* TouchScreen Convert Queue ITEM 15~8 */
#define TCQ_ITEM_15_8         0x424

#define TCQ_ITEM7_SHIFT       28
#define TCQ_ITEM6_SHIFT       24
#define TCQ_ITEM5_SHIFT       20
#define TCQ_ITEM4_SHIFT       16
#define TCQ_ITEM3_SHIFT       12
#define TCQ_ITEM2_SHIFT       8
#define TCQ_ITEM1_SHIFT       4
#define TCQ_ITEM0_SHIFT       0

#define TCQ_ITEM_TCC0         0x0
#define TCQ_ITEM_TCC1         0x1
#define TCQ_ITEM_TCC2         0x2
#define TCQ_ITEM_TCC3         0x3
#define TCQ_ITEM_TCC4         0x4
#define TCQ_ITEM_TCC5         0x5
#define TCQ_ITEM_TCC6         0x6
#define TCQ_ITEM_TCC7         0x7
#define TCQ_ITEM_GCC7         0x8
#define TCQ_ITEM_GCC6         0x9
#define TCQ_ITEM_GCC5         0xa
#define TCQ_ITEM_GCC4         0xb
#define TCQ_ITEM_GCC3         0xc
#define TCQ_ITEM_GCC2         0xd
#define TCQ_ITEM_GCC1         0xe
#define TCQ_ITEM_GCC0         0xf

/* TouchScreen Convert Config 0-7 */
#define TCC0                  0x440
#define TCC1                  0x444
#define TCC2                  0x448
#define TCC3                  0x44c
#define TCC4                  0x450
#define TCC5                  0x454
#define TCC6                  0x458
#define TCC7                  0x45c
#define CC_PEN_IACK           (1 << 1)
#define CC_SEL_REFN_SHIFT     2
#define CC_SEL_REFN_YNLR      (0x1 << CC_SEL_REFN_SHIFT)
#define CC_SEL_REFN_AGND      (0x2 << CC_SEL_REFN_SHIFT)
#define CC_SEL_REFN_MASK      (0x3 << CC_SEL_REFN_SHIFT)
#define CC_SELIN_SHIFT        4
#define CC_SELIN_XPUL         (0x0 << CC_SELIN_SHIFT)
#define CC_SELIN_YPLL         (0x1 << CC_SELIN_SHIFT)
#define CC_SELIN_XNUR         (0x2 << CC_SELIN_SHIFT)
#define CC_SELIN_YNLR         (0x3 << CC_SELIN_SHIFT)
#define CC_SELIN_WIPER        (0x4 << CC_SELIN_SHIFT)
#define CC_SELIN_INAUX0       (0x5 << CC_SELIN_SHIFT)
#define CC_SELIN_INAUX1       (0x6 << CC_SELIN_SHIFT)
#define CC_SELIN_INAUX2       (0x7 << CC_SELIN_SHIFT)
#define CC_SELIN_MASK         (0x7 << CC_SELIN_SHIFT)
#define CC_SELREFP_SHIFT      7
#define CC_SELREFP_YPLL       (0x0 << CC_SELREFP_SHIFT)
#define CC_SELREFP_XPUL       (0x1 << CC_SELREFP_SHIFT)
#define CC_SELREFP_EXT        (0x2 << CC_SELREFP_SHIFT)
#define CC_SELREFP_INT        (0x3 << CC_SELREFP_SHIFT)
#define CC_SELREFP_MASK       (0x3 << CC_SELREFP_SHIFT)
#define CC_XPULSW             (1 << 9)
#define CC_XNURSW_SHIFT       10
#define CC_XNURSW_HIGH        (0x0 << CC_XNURSW_SHIFT)
#define CC_XNURSW_OFF         (0x1 << CC_XNURSW_SHIFT)
#define CC_XNURSW_LOW         (0x3 << CC_XNURSW_SHIFT)
#define CC_XNURSW_MASK        (0x3 << CC_XNURSW_SHIFT)
#define CC_YPLLSW_SHIFT       12
#define CC_YPLLSW_MASK        (0x3 << CC_YPLLSW_SHIFT)
#define CC_YNLRSW             (1 << 14)
#define CC_WIPERSW            (1 << 15)
#define CC_NOS_SHIFT          16
#define CC_YPLLSW_HIGH        (0x0 << CC_NOS_SHIFT)
#define CC_YPLLSW_OFF         (0x1 << CC_NOS_SHIFT)
#define CC_YPLLSW_LOW         (0x3 << CC_NOS_SHIFT)
#define CC_NOS_MASK           (0xf << CC_NOS_SHIFT)
#define CC_IGS                (1 << 20)
#define CC_SETTLING_TIME_SHIFT 24
#define CC_SETTLING_TIME_MASK (0xff << CC_SETTLING_TIME_SHIFT)

#define TSC_4WIRE_PRECHARGE    0x158c
#define TSC_4WIRE_TOUCH_DETECT 0x578e

#define TSC_4WIRE_X_MEASUMENT  0x1c90
#define TSC_4WIRE_Y_MEASUMENT  0x4604

#define TSC_GENERAL_ADC_GCC0   0x17dc
#define TSC_GENERAL_ADC_GCC1   0x17ec
#define TSC_GENERAL_ADC_GCC2   0x17fc

/* GeneralADC Convert Queue FIFO Register */
#define GCQFIFO                0x800
#define GCQFIFO_ADCOUT_SHIFT   4
#define GCQFIFO_ADCOUT_MASK    (0xfff << GCQFIFO_ADCOUT_SHIFT)
/* GeneralADC Convert Queue Control Register */
#define GCQCR                  0x804
/* GeneralADC Convert Queue Status Register */
#define GCQSR                  0x808
/* GeneralADC Convert Queue Mask Register */
#define GCQMR                  0x80c

/* GeneralADC Convert Queue ITEM 7~0 */
#define GCQ_ITEM_7_0           0x820
/* GeneralADC Convert Queue ITEM 15~8 */
#define GCQ_ITEM_15_8          0x824

#define GCQ_ITEM7_SHIFT        28
#define GCQ_ITEM6_SHIFT        24
#define GCQ_ITEM5_SHIFT        20
#define GCQ_ITEM4_SHIFT        16
#define GCQ_ITEM3_SHIFT        12
#define GCQ_ITEM2_SHIFT        8
#define GCQ_ITEM1_SHIFT        4
#define GCQ_ITEM0_SHIFT        0

#define GCQ_ITEM_GCC0          0x0
#define GCQ_ITEM_GCC1          0x1
#define GCQ_ITEM_GCC2          0x2
#define GCQ_ITEM_GCC3          0x3

/* GeneralADC Convert Config 0-7 */
#define GCC0                   0x840
#define GCC1                   0x844
#define GCC2                   0x848
#define GCC3                   0x84c
#define GCC4                   0x850
#define GCC5                   0x854
#define GCC6                   0x858
#define GCC7                   0x85c

/* TSC Test Register R/W */
#define TTR                    0xc00
/* TSC Monitor Register 1, 2 */
#define MNT1                   0xc04
#define MNT2                   0xc04

#define DETECT_ITEM_ID_1       1
#define DETECT_ITEM_ID_2       5
#define TS_X_ITEM_ID           2
#define TS_Y_ITEM_ID           3
#define TSI_DATA               1
#define FQS_DATA               0

#endif				/* __IMX_ADC_H__ */
