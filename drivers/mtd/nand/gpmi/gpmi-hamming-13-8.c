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
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <mach/dma.h>
#include "gpmi.h"

#define BIT_VAL(v, n) (((v) >> (n)) & 0x1)
#define B(n) (BIT_VAL(d, n))

static u8 calculate_parity(u8 d)
{
	u8 p = 0;

	if (d == 0 || d == 0xFF)
		return 0;	/* optimization :) */

	p |= (B(6) ^ B(5) ^ B(3) ^ B(2)) 	<< 0;
	p |= (B(7) ^ B(5) ^ B(4) ^ B(2) ^ B(1)) << 1;
	p |= (B(7) ^ B(6) ^ B(5) ^ B(1) ^ B(0)) << 2;
	p |= (B(7) ^ B(4) ^ B(3) ^ B(0)) 	<< 3;
	p |= (B(6) ^ B(4) ^ B(3) ^ B(2) ^ B(1) ^ B(0)) << 4;
	return p;
}

static inline int even_number_of_1s(u8 byte)
{
	int even = 1;

	while (byte > 0) {
		even ^= (byte & 0x1);
		byte >>= 1;
	}
	return even;
}

static int lookup_single_error(u8 syndrome)
{
	int i;
	u8 syndrome_table[] = {
		0x1C, 0x16, 0x13, 0x19,
		0x1A, 0x07, 0x15, 0x0E,
		0x01, 0x02, 0x04, 0x08,
		0x10,
	};

	for (i = 0; i < ARRAY_SIZE(syndrome_table); i++)
		if (syndrome_table[i] == syndrome)
			return i;
	return -ENOENT;
}

int gpmi_verify_hamming_13_8(void *data, u8 *parity, size_t size)
{
	int i;
	u8 *pdata = data;
	int bit_to_flip;
	u8 np, syndrome;
	int errors = 0;

	for (i = 0; i < size; i ++, pdata++) {
		np = calculate_parity(*pdata);
		syndrome = np ^ parity[i];
		if (syndrome == 0) /* cool */ {
			continue;
		}

		if (even_number_of_1s(syndrome))
			return -i;	/* can't recover */

		bit_to_flip = lookup_single_error(syndrome);
		if (bit_to_flip < 0)
			return -i;	/* can't fix the error */

		if (bit_to_flip < 8) {
			*pdata ^= (1 << bit_to_flip);
			errors++;
		}
	}
	return errors;
}

void gpmi_encode_hamming_13_8(void *source_block, size_t src_size,
			void *source_ecc, size_t ecc_size)
{
	int i;
	u8 *src = source_block;
	u8 *ecc = source_ecc;

	for (i = 0; i < src_size && i < ecc_size; i++)
		ecc[i] = calculate_parity(src[i]);
}

void gpmi_encode_hamming_ncb_13_8(void *source_block, size_t source_size,
		void *target_block, size_t target_size)
{
	if (target_size < 12 + 2 * NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES)
		return;
	memset(target_block, 0xFF, target_size);
	memcpy((u8 *)target_block + 12, source_block, source_size);
	gpmi_encode_hamming_13_8(source_block,
	   NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES,
	   (u8 *)target_block + 12 + NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES,
	   NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES);
}

unsigned gpmi_hamming_ecc_size_13_8(int block_size)
{
	return block_size;
}
