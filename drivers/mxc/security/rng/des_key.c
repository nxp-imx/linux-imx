/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file des_key.c
 *
 * This file implements the function #fsl_shw_permute1_bytes().
 *
 * The code was lifted from crypto++ v5.5.2, which is public domain code.  The
 * code to handle words instead of bytes was extensively modified from the byte
 * version and then converted to handle one to three keys at once.
 *
 */

#include "shw_driver.h"
#ifdef DIAG_SECURITY_FUNC
#include "apihelp.h"
#endif

#ifndef __KERNEL__
#include <asm/types.h>
#include <linux/byteorder/little_endian.h>	/* or whichever is proper for target arch */
#endif

#ifdef DEBUG
#undef DEBUG			/* TEMPORARY */
#endif

#if defined(DEBUG) || defined(SELF_TEST)
static void DUMP_BYTES(const char *label, const uint8_t * data, int len)
{
	int i;

	printf("%s: ", label);
	for (i = 0; i < len; i++) {
		printf("%02X", data[i]);
		if ((i % 8 == 0) && (i != 0)) {
			printf("_");	/* key separator */
		}
	}
	printf("\n");
}

static void DUMP_WORDS(const char *label, const uint32_t * data, int len)
{
	int i, j;

	printf("%s: ", label);
	/* Dump the words in reverse order, so that they are intelligible */
	for (i = len - 1; i >= 0; i--) {
		for (j = 3; j >= 0; j--) {
			uint32_t word = data[i];
			printf("%02X", (word >> ((j * 8)) & 0xff));
			if ((i != 0) && ((((i) * 4 + 5 + j) % 7) == 5))
				printf("_");	/* key separator */
		}
		printf("|");	/* word separator */
	}
	printf("\n");
}
#else
#define DUMP_BYTES(label, data,len)
#define DUMP_WORDS(label, data,len)
#endif

/*!
 * permuted choice table (key)
 *
 * Note that this table has had one subtracted from each element so that the
 * code doesn't have to do it.
 */
static const uint8_t pc1[] = {
	56, 48, 40, 32, 24, 16, 8,
	0, 57, 49, 41, 33, 25, 17,
	9, 1, 58, 50, 42, 34, 26,
	18, 10, 2, 59, 51, 43, 35,
	62, 54, 46, 38, 30, 22, 14,
	6, 61, 53, 45, 37, 29, 21,
	13, 5, 60, 52, 44, 36, 28,
	20, 12, 4, 27, 19, 11, 3,
};

/*! bit 0 is left-most in byte */
static const int bytebit[] = {
	0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01
};

/*!
 * Convert a 3-key 3DES key into the first-permutation 168-bit version.
 *
 * This is the format of the input key:
 *
 * @verbatim
   BIT:  |191                                 128|127                                  64|63                                    0|
   BYTE: |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 |
   KEY:  |                   0                   |                   1                   |                   2                   |
  @endverbatim
 *
 * This is the format of the output key:
 *
 * @verbatim
   BIT:  |167                            112|111                             56|55                               0|
   BYTE: |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 |
   KEY:  |                 1                |                 2                |                 3                |
  @endverbatim
 * 
 * @param[in]   key          bytes of 3DES key
 * @param[out] permuted_key  21 bytes of permuted key
 * @param[in]  key_count     How many DES keys (2 or 3)
 */
void fsl_shw_permute1_bytes(const uint8_t * key, uint8_t * permuted_key,
			    int key_count)
{
	int i;
	int j;
	int l;
	int m;

	DUMP_BYTES("Input key", key, 8 * key_count);

	/* For each individual sub-key */
	for (i = 0; i < 3; i++) {
		DUMP_BYTES("(key)", key, 8);
		memset(permuted_key, 0, 7);
		/* For each bit of key */
		for (j = 0; j < 56; j++) {	/* convert pc1 to bits of key */
			l = pc1[j];	/* integer bit location  */
			m = l & 07;	/* find bit              */
			permuted_key[j >> 3] |= (((key[l >> 3] &	/* find which key byte l is in */
						   bytebit[m])	/* and which bit of that byte */
						  ? 0x80 : 0) >> (j % 8));	/* and store 1-bit result */
		}
		switch (i) {
		case 0:
			if (key_count != 1)
				key += 8;	/* move on to second key */
			break;
		case 1:
			if (key_count == 2)
				key -= 8;	/* go back to first key */
			else if (key_count == 3)
				key += 8;	/* move on to third key */
			break;
		default:
			break;
		}
		permuted_key += 7;
	}
	DUMP_BYTES("Output key (bytes)", permuted_key - 21, 21);
}

#ifdef SELF_TEST
const uint8_t key1_in[] = {
	/* FE01FE01FE01FE01_01FE01FE01FE01FE_FEFE0101FEFE0101 */
	0xFE, 0x01, 0xFE, 0x01, 0xFE, 0x01, 0xFE, 0x01,
	0x01, 0xFE, 0x01, 0xFE, 0x01, 0xFE, 0x01, 0xFE,
	0xFE, 0xFE, 0x01, 0x01, 0xFE, 0xFE, 0x01, 0x01
};

const uint32_t key1_word_in[] = {
	0xFE01FE01, 0xFE01FE01,
	0x01FE01FE, 0x01FE01FE,
	0xFEFE0101, 0xFEFE0101
};

uint8_t exp_key1_out[] = {
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33
};

uint32_t exp_word_key1_out[] = {
	0x33333333, 0xAA333333, 0xAAAAAAAA, 0x5555AAAA,
	0x55555555, 0x00000055,
};

const uint8_t key2_in[] = {
	0xEF, 0x10, 0xBB, 0xA4, 0x23, 0x49, 0x42, 0x58,
	0x01, 0x28, 0x01, 0x4A, 0x10, 0xE4, 0x03, 0x59,
	0xFE, 0x84, 0x30, 0x29, 0x8E, 0xF1, 0x10, 0x5A
};

const uint32_t key2_word_in[] = {
	0xEF10BBA4, 0x23494258,
	0x0128014A, 0x10E40359,
	0xFE843029, 0x8EF1105A
};

uint8_t exp_key2_out[] = {
	0x0D, 0xE1, 0x1D, 0x85, 0x50, 0x9A, 0x56, 0x20,
	0xA8, 0x22, 0x94, 0x82, 0x08, 0xA0, 0x33, 0xA1,
	0x2D, 0xE9, 0x11, 0x39, 0x95
};

uint32_t exp_word_key2_out[] = {
	0xE9113995, 0xA033A12D, 0x22948208, 0x9A5620A8,
	0xE11D8550, 0x0000000D
};

const uint8_t key3_in[] = {
	0x3F, 0xE9, 0x49, 0x4B, 0x67, 0x57, 0x07, 0x3C,
	0x89, 0x77, 0x73, 0x0C, 0xA0, 0x05, 0x41, 0x69,
	0xB3, 0x7C, 0x98, 0xD8, 0xC9, 0x35, 0x57, 0x19
};

const uint32_t key3_word_in[] = {
	0xEF10BBA4, 0x23494258,
	0x0128014A, 0x10E40359,
	0xFE843029, 0x8EF1105A
};

uint8_t exp_key3_out[] = {
	0x02, 0x3E, 0x93, 0xA7, 0x9F, 0x18, 0xF1, 0x11,
	0xC6, 0x96, 0x00, 0x62, 0xA8, 0x96, 0x02, 0x3E,
	0x93, 0xA7, 0x9F, 0x18, 0xF1
};

uint32_t exp_word_key3_out[] = {
	0xE9113995, 0xA033A12D, 0x22948208, 0x9A5620A8,
	0xE11D8550, 0x0000000D
};

const uint8_t key4_in[] = {
	0x3F, 0xE9, 0x49, 0x4B, 0x67, 0x57, 0x07, 0x3C,
	0x89, 0x77, 0x73, 0x0C, 0xA0, 0x05, 0x41, 0x69,
};

const uint32_t key4_word_in[] = {
	0xEF10BBA4, 0x23494258,
	0x0128014A, 0x10E40359,
	0xFE843029, 0x8EF1105A
};

const uint8_t key5_in[] = {
	0x3F, 0xE9, 0x49, 0x4B, 0x67, 0x57, 0x07, 0x3C,
	0x89, 0x77, 0x73, 0x0C, 0xA0, 0x05, 0x41, 0x69,
	0x3F, 0xE9, 0x49, 0x4B, 0x67, 0x57, 0x07, 0x3C,
};

uint8_t exp_key4_out[] = {
	0x0D, 0xE1, 0x1D, 0x85, 0x50, 0x9A, 0x56, 0x20,
	0xA8, 0x22, 0x94, 0x82, 0x08, 0xA0, 0x33, 0xA1,
	0x2D, 0xE9, 0x11, 0x39, 0x95
};

uint32_t exp_word_key4_out[] = {
	0xE9113995, 0xA033A12D, 0x22948208, 0x9A5620A8,
	0xE11D8550, 0x0000000D
};

const uint8_t key6_in[] = {
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
};

uint8_t exp_key6_out[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00
};

uint32_t exp_word_key6_out[] = {
	0x00000000, 0x0000000, 0x0000000, 0x00000000,
	0x00000000, 0x0000000
};

const uint8_t key7_in[] = {
	/* 01FE01FE01FE01FE_FE01FE01FE01FE01_0101FEFE0101FEFE */
	/*  0101FEFE0101FEFE_FE01FE01FE01FE01_01FE01FE01FE01FE */
	0x01, 0x01, 0xFE, 0xFE, 0x01, 0x01, 0xFE, 0xFE,
	0xFE, 0x01, 0xFE, 0x01, 0xFE, 0x01, 0xFE, 0x01,
	0x01, 0xFE, 0x01, 0xFE, 0x01, 0xFE, 0x01, 0xFE,
};

uint8_t exp_key7_out[] = {
	0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x55,
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa
};

uint32_t exp_word_key7_out[] = {
	0xcccccccc, 0x55cccccc, 0x55555555, 0xaaaa5555,
	0xaaaaaaaa, 0x000000aa
};

int run_test(const uint8_t * key_in,
	     const int key_count,
	     const uint32_t * key_word_in,
	     const uint8_t * exp_bytes_key_out,
	     const uint32_t * exp_word_key_out)
{
	uint8_t key_out[22];
	uint32_t word_key_out[6];
	int failed = 0;

	memset(key_out, 0x42, 22);
	fsl_shw_permute1_bytes(key_in, key_out, key_count);
	if (memcmp(key_out, exp_bytes_key_out, 21) != 0) {
		printf("bytes_to_bytes: ERROR: \n");
		DUMP_BYTES("key_in", key_in, 8 * key_count);
		DUMP_BYTES("key_out", key_out, 21);
		DUMP_BYTES("exp_out", exp_bytes_key_out, 21);
		failed |= 1;
	} else if (key_out[21] != 0x42) {
		printf("bytes_to_bytes: ERROR: Buffer overflow 0x%02x\n",
		       (int)key_out[21]);
	} else {
		printf("bytes_to_bytes: OK\n");
	}
#if 0
	memset(word_key_out, 0x42, 21);
	fsl_shw_permute1_bytes_to_words(key_in, word_key_out, key_count);
	if (memcmp(word_key_out, exp_word_key_out, 21) != 0) {
		printf("bytes_to_words: ERROR: \n");
		DUMP_BYTES("key_in", key_in, 8 * key_count);
		DUMP_WORDS("key_out", word_key_out, 6);
		DUMP_WORDS("exp_out", exp_word_key_out, 6);
		failed |= 1;
	} else {
		printf("bytes_to_words: OK\n");
	}

	if (key_word_in != NULL) {
		memset(word_key_out, 0x42, 21);
		fsl_shw_permute1_words_to_words(key_word_in, word_key_out);
		if (memcmp(word_key_out, exp_word_key_out, 21) != 0) {
			printf("words_to_words: ERROR: \n");
			DUMP_BYTES("key_in", key_in, 24);
			DUMP_WORDS("key_out", word_key_out, 6);
			DUMP_WORDS("exp_out", exp_word_key_out, 6);
			failed |= 1;
		} else {
			printf("words_to_words: OK\n");
		}
	}
#endif

	return failed;
}				/* end fn run_test */

int main()
{
	int failed = 0;

	printf("key1\n");
	failed |=
	    run_test(key1_in, 3, key1_word_in, exp_key1_out, exp_word_key1_out);
	printf("\nkey2\n");
	failed |=
	    run_test(key2_in, 3, key2_word_in, exp_key2_out, exp_word_key2_out);
	printf("\nkey3\n");
	failed |= run_test(key3_in, 3, NULL, exp_key3_out, exp_word_key3_out);
	printf("\nkey4\n");
	failed |= run_test(key4_in, 2, NULL, exp_key4_out, exp_word_key4_out);
	printf("\nkey5\n");
	failed |= run_test(key5_in, 3, NULL, exp_key4_out, exp_word_key4_out);
	printf("\nkey6 - 3\n");
	failed |= run_test(key6_in, 3, NULL, exp_key6_out, exp_word_key6_out);
	printf("\nkey6 - 2\n");
	failed |= run_test(key6_in, 2, NULL, exp_key6_out, exp_word_key6_out);
	printf("\nkey6 - 1\n");
	failed |= run_test(key6_in, 1, NULL, exp_key6_out, exp_word_key6_out);
	printf("\nkey7\n");
	failed |= run_test(key7_in, 3, NULL, exp_key7_out, exp_word_key7_out);
	printf("\n");

	if (failed != 0) {
		printf("TEST FAILED\n");
	}
	return failed;
}

#endif				/* SELF_TEST */
