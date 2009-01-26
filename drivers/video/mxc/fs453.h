/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
  * @file fs453.h
  * @brief Driver for FS453/4 TV encoder
  *
  * @ingroup FS453
  */

#ifndef __FS453_H__
#define __FS453_H__

/* I2C address of the FS453 chip */

#define I2C1_BUS	0
#define FS453_I2C_ADDR	0x6A

/*!
 *
 * FS453 register file
 *
 */
#define FS453_IHO		0x00	/*! Input Horizontal Offset */
#define FS453_IVO		0x02	/*! Input Vertical Offset */
#define FS453_IHW		0x04	/*! Input Horizontal Width */
#define FS453_VSC		0x06	/*! Vertical Scaling Coefficient */
#define FS453_HSC		0x08	/*! Horizontal Scaling Coefficient */
#define FS453_BYPASS		0x0A	/*! BYPASS */
#define FS453_CR		0x0C	/*! Command Register */
#define FS453_MISC		0x0E	/*! Miscellaneous Bits Register */
#define FS453_NCON		0x10	/*! Numerator of NCO Word */
#define FS453_NCOD		0x14	/*! Denominator of NCO Word */
#define FS453_PLL_M_PUMP	0x18	/*! PLL M and Pump Control */
#define FS453_PLL_N		0x1A	/*! PLL N */
#define FS453_PLL_PDIV		0x1C	/*! PLL Post-Divider */
#define FS453_SHP		0x24	/*! Sharpness Filter */
#define FS453_FLK		0x26	/*! Filcker Filter Coefficient */
#define FS453_GPIO		0x28	/*! General Purpose I/O, Output Enab */
#define FS453_ID		0x32	/*! Part Identification Number */
#define FS453_STATUS		0x34	/*! Status Port */
#define FS453_FIFO_SP		0x36	/*! FIFO Status Port Fill/Underrun */
#define FS453_FIFO_LAT		0x38	/*! FIFO Latency */
#define FS453_CHR_FREQ		0x40	/*! Chroma Subcarrier Frequency */
#define FS453_CHR_PHASE		0x44	/*! Chroma Phase */
#define FS453_MISC_45		0x45	/*! Miscellaneous Bits Register 45 */
#define FS453_MISC_46		0x46	/*! Miscellaneous Bits Register 46 */
#define FS453_MISC_47		0x47	/*! Miscellaneous Bits Register 47 */
#define FS453_HSYNC_WID		0x48	/*! HSync Width */
#define FS453_BURST_WID		0x49	/*! Burst Width */
#define FS453_BPORCH		0x4A	/*! Back Porch Width */
#define FS453_CB_BURST		0x4B	/*! Cb Burst Amplitude */
#define FS453_CR_BURST		0x4C	/*! Cr Burst Amplitude */
#define FS453_MISC_4D		0x4D	/*! Miscellaneous Bits Register 4D */
#define FS453_BLACK_LVL		0x4E	/*! Black Level */
#define FS453_BLANK_LVL		0x50	/*! Blank Level */
#define FS453_NUM_LINES		0x57	/*! Number of Lines */
#define FS453_WHITE_LVL		0x5E	/*! White Level */
#define FS453_CB_GAIN		0x60	/*! Cb Color Saturation */
#define FS453_CR_GAIN		0x62	/*! Cr Color Saturation */
#define FS453_TINT		0x65	/*! Tint */
#define FS453_BR_WAY		0x69	/*! Width of Breezeway */
#define FS453_FR_PORCH		0x6C	/*! Front Porch */
#define FS453_NUM_PIXELS	0x71	/*! Total num. of luma/chroma Pixels */
#define FS453_1ST_LINE		0x73	/*! First Video Line */
#define FS453_MISC_74		0x74	/*! Miscellaneous Bits Register 74 */
#define FS453_SYNC_LVL		0x75	/*! Sync Level */
#define FS453_VBI_BL_LVL	0x7C	/*! VBI Blank Level */
#define FS453_SOFT_RST		0x7E	/*! Encoder Soft Reset */
#define FS453_ENC_VER		0x7F	/*! Encoder Version */
#define FS453_WSS_CONFIG	0x80	/*! WSS Configuration Register */
#define FS453_WSS_CLK		0x81	/*! WSS Clock */
#define FS453_WSS_DATAF1	0x83	/*! WSS Data Field 1 */
#define FS453_WSS_DATAF0	0x86	/*! WSS Data Field 0 */
#define FS453_WSS_LNF1		0x89	/*! WSS Line Number Field 1 */
#define FS453_WSS_LNF0		0x8A	/*! WSS Line Number Field 0 */
#define FS453_WSS_LVL		0x8B	/*! WSS Level */
#define FS453_MISC_8D		0x8D	/*! Miscellaneous Bits Register 8D */
#define FS453_VID_CNTL0		0x92	/*! Video Control 0 */
#define FS453_HD_FP_SYNC	0x94	/*! Horiz. Front Porch & HSync Width */
#define FS453_HD_YOFF_BP	0x96	/*! HDTV Lum. Offset & Back Porch */
#define FS453_SYNC_DL		0x98	/*! Sync Delay Value */
#define FS453_LD_DET		0x9C	/*! DAC Load Detect */
#define FS453_DAC_CNTL		0x9E	/*! DAC Control */
#define FS453_PWR_MGNT		0xA0	/*! Power Management */
#define FS453_RED_MTX		0xA2	/*! RGB to YCrCb Matrix Red Coeff. */
#define FS453_GRN_MTX		0xA4	/*! RGB to YCrCb Matrix Green Coeff. */
#define FS453_BLU_MTX		0xA6	/*! RGB to YCrCb Matrix Blue Coeff. */
#define FS453_RED_SCL		0xA8	/*! RGB to YCrCb Scaling Red Coeff. */
#define FS453_GRN_SCL		0xAA	/*! RGB to YCrCb Scaling Green Coeff. */
#define FS453_BLU_SCL		0xAC	/*! RGB to YCrCb Scaling Blue Coeff. */
#define FS453_CC_FIELD_1	0xAE	/*! Closed Caption Field 1 Data */
#define FS453_CC_FIELD_2	0xB0	/*! Closed Caption Field 2 Data */
#define FS453_CC_CONTROL	0xB2	/*! Closed Caption Control */
#define FS453_CC_BLANK_VALUE	0xB4	/*! Closed Caption Blanking Value */
#define FS453_CC_BLANK_SAMPLE	0xB6	/*! Closed Caption Blanking Sample */
#define FS453_HACT_ST		0xB8	/*! HDTV Horizontal Active Start */
#define FS453_HACT_WD		0xBA	/*! HDTV Horizontal Active Width */
#define FS453_VACT_ST		0xBC	/*! HDTV Veritical Active Width */
#define FS453_VACT_HT		0xBE	/*! HDTV Veritical Active Height */
#define FS453_PR_PB_SCALING	0xC0	/*! Pr and Pb Relative Scaling */
#define FS453_LUMA_BANDWIDTH	0xC2	/*! Luminance Frequency Response */
#define FS453_QPR		0xC4	/*! Quick Program Register */

/*! Command register bits */

#define CR_GCC_CK_LVL		0x2000	/*! Graphics Controller switching lev */
#define CR_P656_LVL		0x1000	/*! Pixel Port Output switching level */
#define CR_P656_IN		0x0800	/*! Pixel Port In */
#define CR_P656_OUT		0x0400	/*! Pixel Port Out */
#define CR_CBAR_480P		0x0200	/*! 480P Color Bars */
#define CR_PAL_NTSCIN		0x0100	/*! PAL or NTSC input */
#define CR_SYNC_MS		0x0080	/*! Sync Master or Slave */
#define CR_FIFO_CLR		0x0040	/*! FIFO Clear */
#define CR_CACQ_CLR		0x0020	/*! CACQ Clear */
#define CR_CDEC_BP		0x0010	/*! Chroma Decimator Bypass */
#define CR_NCO_EN		0x0002	/*! Enable NCO Latch */
#define CR_SRESET		0x0001	/*! Soft Reset */

/*! Chip ID register bits */

#define FS453_CHIP_ID		0xFE05	/*! Chip ID register expected value */

#endif				/* __FS453_H__ */
