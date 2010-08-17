/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MXC_I2C_REG_H__
#define __MXC_I2C_REG_H__

/* Address offsets of the I2C registers */
#define MXC_IADR                0x00	/* Address Register */
#define MXC_IFDR                0x04	/* Freq div register */
#define MXC_I2CR                0x08	/* Control regsiter */
#define MXC_I2SR                0x0C	/* Status register */
#define MXC_I2DR                0x10	/* Data I/O register */

/* Bit definitions of I2CR */
#define MXC_I2CR_IEN            0x0080
#define MXC_I2CR_IIEN           0x0040
#define MXC_I2CR_MSTA           0x0020
#define MXC_I2CR_MTX            0x0010
#define MXC_I2CR_TXAK           0x0008
#define MXC_I2CR_RSTA           0x0004

/* Bit definitions of I2SR */
#define MXC_I2SR_ICF            0x0080
#define MXC_I2SR_IAAS           0x0040
#define MXC_I2SR_IBB            0x0020
#define MXC_I2SR_IAL            0x0010
#define MXC_I2SR_SRW            0x0004
#define MXC_I2SR_IIF            0x0002
#define MXC_I2SR_RXAK           0x0001

#endif				/* __MXC_I2C_REG_H__ */
