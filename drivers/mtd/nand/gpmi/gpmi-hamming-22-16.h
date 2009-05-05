/*
 * Freescale STMP37XX/STMP378X GPMI (General-Purpose-Media-Interface)
 *
 * NCB software ECC Hamming code
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __LINUX_GPMI_H
#define __LINUX_GPMI_H

#define NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES	(512)
#define NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES	\
	((((NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES*8)/16)*6)/8)
#define NAND_HC_ECC_OFFSET_FIRST_DATA_COPY	    (0)
#define NAND_HC_ECC_OFFSET_SECOND_DATA_COPY	   	\
	(NAND_HC_ECC_OFFSET_FIRST_DATA_COPY + 		\
	 NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES)
#define NAND_HC_ECC_OFFSET_THIRD_DATA_COPY		\
	(NAND_HC_ECC_OFFSET_SECOND_DATA_COPY + 		\
	 NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES)
#define NAND_HC_ECC_OFFSET_FIRST_PARITY_COPY		\
	(NAND_HC_ECC_OFFSET_THIRD_DATA_COPY + 		\
	 NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES)
#define NAND_HC_ECC_OFFSET_SECOND_PARITY_COPY		\
	(NAND_HC_ECC_OFFSET_FIRST_PARITY_COPY + 	\
	 NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES)
#define NAND_HC_ECC_OFFSET_THIRD_PARITY_COPY		\
	(NAND_HC_ECC_OFFSET_SECOND_PARITY_COPY + 	\
	 NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES)

#endif
