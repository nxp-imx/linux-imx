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
#include "gpmi-hamming-22-16.h"

#define BIT_VAL(v, n) (((v) >> (n)) & 0x1)
#define B(n) (BIT_VAL(d, n))
#define BSEQ(a1, a2, a3, a4, a5, a6, a7, a8) \
	(B(a1) ^ B(a2) ^ B(a3) ^ B(a4) ^ B(a5) ^ B(a6) ^ B(a7) ^ B(a8))
static u8 calculate_parity(u16 d)
{
	u8 p = 0;

	if (d == 0 || d == 0xFFFF)
		return 0;	/* optimization :) */

	p |= BSEQ(15, 12, 11,  8,  5,  4,  3,  2) << 0;
	p |= BSEQ(13, 12, 11, 10,  9,  7,  3,  1) << 1;
	p |= BSEQ(15, 14, 13, 11, 10,  9,  6,  5) << 2;
	p |= BSEQ(15, 14, 13,  8,  7,  6,  4,  0) << 3;
	p |= BSEQ(12,  9,  8,  7,  6,  2,  1,  0) << 4;
	p |= BSEQ(14, 10,  5,  4,  3,  2,  1,  0) << 5;
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
		0x38, 0x32, 0x31, 0x23, 0x29, 0x25, 0x1C, 0x1A,
		0x19, 0x16, 0x26, 0x07, 0x13, 0x0E, 0x2C, 0x0D,
		0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
	};

	for (i = 0; i < ARRAY_SIZE(syndrome_table); i++)
		if (syndrome_table[i] == syndrome)
			return i;
	return -ENOENT;
}

int gpmi_verify_hamming_22_16(void *data, u8 *parity, size_t size)
{
	int i, j;
	int bit_index, bit_to_flip;
	u16 *pdata = data;
	u8 p = 0, np, syndrome;
	int errors = 0;

	for (bit_index = i = j = 0;
	     i < size / sizeof(u16);
	     i ++, pdata++) {

		switch (bit_index) {

		case 0:
			p = parity[j] & 0x3F;
			break;
		case 2:
			p = (parity[j++] & 0xC0) >> 6;
			p |= (parity[j] & 0x0F) << 2;
			break;
		case 4:
			p = (parity[j++] & 0xF0) >> 4;
			p |= (parity[j] & 0x03) << 4;
			break;
		case 6:
			p = (parity[j++] & 0xFC) >> 2;
			break;
		default:
			BUG();	/* how did you get this ?! */
			break;
		}
		bit_index = (bit_index + 2) % 8;

		np = calculate_parity(*pdata);
		syndrome = np ^ p;
		if (syndrome == 0) /* cool */ {
			continue;
		}

		if (even_number_of_1s(syndrome))
			return -i;	/* can't recover */

		bit_to_flip = lookup_single_error(syndrome);
		if (bit_to_flip < 0)
			return -i;	/* can't fix the error */

		if (bit_to_flip < 16) {
			*pdata ^= (1 << bit_to_flip);
			errors++;
		}
	}
	return errors;
}

void gpmi_encode_hamming_22_16(void *source_block, size_t src_size,
			void *source_ecc, size_t ecc_size)
{
	int i, j, bit_index;
	u16 *src = source_block;
	u8 *ecc = source_ecc;
	u8   np;

	for (bit_index = j = i = 0;
	     j < src_size/sizeof(u16) && i < ecc_size;
	     j++) {

		np = calculate_parity(src[j]);

		switch (bit_index) {

		case 0:
			ecc[i] = np & 0x3F;
			break;
		case 2:
			ecc[i++] |= (np & 0x03) << 6;
			ecc[i]    = (np & 0x3C) >> 2;
			break;
		case 4:
			ecc[i++] |= (np & 0x0F) << 4;
			ecc[i]    = (np & 0x30) >> 4;
			break;
		case 6:
			ecc[i++] |= (np & 0x3F) << 2;
			break;
		}
		bit_index = (bit_index + 2) % 8;
	}

}

void gpmi_encode_hamming_ncb_22_16(void *source_block, size_t source_size,
		void *target_block, size_t target_size)
{
	u8  *dst = target_block;
	u8   ecc[NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES];

	gpmi_encode_hamming_22_16(source_block,
			NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES,
			ecc,
			NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES);
	/* create THREE copies of source block 	*/
	memcpy(dst + NAND_HC_ECC_OFFSET_FIRST_DATA_COPY,
		source_block, NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES);
	memcpy(dst + NAND_HC_ECC_OFFSET_SECOND_DATA_COPY,
		source_block, NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES);
	memcpy(dst + NAND_HC_ECC_OFFSET_THIRD_DATA_COPY,
		source_block, NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES);
	/* ..and three copies of ECC block 	*/
	memcpy(dst + NAND_HC_ECC_OFFSET_FIRST_PARITY_COPY,
		ecc, NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES);
	memcpy(dst + NAND_HC_ECC_OFFSET_SECOND_PARITY_COPY,
		ecc, NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES);
	memcpy(dst + NAND_HC_ECC_OFFSET_THIRD_PARITY_COPY,
		ecc, NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES);
}

unsigned gpmi_hamming_ecc_size_22_16(int block_size)
{
	return (((block_size * 8) / 16) * 6) / 8;
}
