/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MXC_I2C_HS_REG_H__
#define __MXC_I2C_HS_REG_H__

#define	HISADR        0x00

#define	HIMADR        0x04
#define	HIMADR_LSB_ADR(x) ((x) << 1)
#define	HIMADR_MSB_ADR(x) (((x) & 0x7) << 8)

#define	HICR          0x08
#define	HICR_HIEN	0x1
#define	HICR_DMA_EN_RX 0x2
#define	HICR_DMA_EN_TR 0x4
#define	HICR_RSTA 0x8
#define	HICR_TXAK 0x10
#define	HICR_MTX 0x20
#define	HICR_MSTA 0x40
#define	HICR_HIIEN 0x80
#define	HICR_ADDR_MODE 0x100
#define	HICR_MST_CODE(x) (((x)&0x7) << 9)
#define	HICR_HSM_EN 0x1000
#define	HICR_SAMC(x) (((x)&0x3) << 13)
#define 	SAMC_7_10 	0
#define	SMAC_7		1
#define	SMAC_10		2

#define  HISR          0x0c
#define 	HISR_RDF		0x1
#define 	HISR_TDE		0x2
#define 	HISR_HIAAS	0x4
#define 	HISR_HIAL	0x8
#define 	HISR_BTD		0x10
#define 	HISR_RDC_ZERO	0x20
#define 	HISR_TDC_ZERO	0x40
#define 	HISR_RXAK		0x80
#define 	HISR_HIBB		0x100
#define 	HISR_SRW		0x200
#define 	HISR_SADDR_MODE	0x400
#define 	HISR_SHS_MODE	0x800

#define  HIIMR         0x10
#define  	HIIMR_RDF	0x1
#define  	HIIMR_TDE	0x2
#define  	HIIMR_AAS	0x4
#define  	HIIMR_AL	0x8
#define  	HIIMR_BTD	0x10
#define  	HIIMR_RDC	0x20
#define  	HIIMR_TDC	0x40
#define  	HIIMR_RXAK	0x80

#define  HITDR         0x14

#define  HIRDR         0x18

#define  HIFSFDR       0x1c

#define  HIHSFDR       0x20

#define  HITFR         0x24
#define	HITFR_TFEN		0x1
#define	HITFR_TFLSH	0x2
#define	HITFR_TFWM(x) (((x) & 0x7) << 2)
#define	HITFR_TFC(x)	(((x) >> 8) & 0xF)
#define	HITFR_MAX_COUNT 8

#define  HIRFR         0x28
#define  	HIRFR_RFEN			0x1
#define  	HIRFR_RFLSH		0x2
#define  	HIRFR_RFWM(x)	 	(((x) & 0x7) << 2)
#define  	HIRFR_RFC(x) 		(((x) >> 8) & 0xF)
#define	HIRFR_MAX_COUNT 8

#define  HITDCR        0x2c
#define	HITDCR_TDC(x)		((x) & 0xFF)
#define	HITDCR_TDC_EN	0x100
#define	HITDCR_TDC_RSTA	0x200
#define	HITDCR_MAX_COUNT 0xFF

#define  HIRDCR        0x30
#define	HIRDCR_RDC(x)		((x) & 0xFF)
#define	HIRDCR_RDC_EN	0x100
#define	HIRDCR_RDC_RSTA	0x200
#define	HIRDCR_MAX_COUNT 0xFF

#endif
