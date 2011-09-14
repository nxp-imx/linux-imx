/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file shw_hash.c
 *
 * This file contains implementations for use of the (internal) SHW hash
 * software computation.  It defines the usual three steps:
 *
 * - #shw_hash_init()
 * - #shw_hash_update()
 * - #shw_hash_final()
 *
 * In support of the above functions, it also contains these functions:
 * - #sha256_init()
 * - #sha256_process_block()
 *
 *
 * These functions depend upon the Linux Endian functions __be32_to_cpu(),
 * __cpu_to_be32() to convert a 4-byte big-endian array to an integer and
 * vice-versa.  For those without Linux, it should be pretty obvious what they
 * do.
 *
 * The #shw_hash_update() and #shw_hash_final() functions are generic enough to
 * support SHA-1/SHA-224/SHA-256, as needed.  Some extra tweaking would be
 * necessary to get them to support SHA-384/SHA-512.
 *
 */

#include "shw_driver.h"
#include "shw_hash.h"

#ifndef __KERNEL__
#include <asm/types.h>
#include <linux/byteorder/little_endian.h>	/* or whichever is proper for target arch */
#define printk printf
#endif

/*!
 * Rotate a value right by a number of bits.
 *
 * @param x   Word of data which needs rotating
 * @param y   Number of bits to rotate
 *
 * @return The new value
 */
inline uint32_t rotr32fixed(uint32_t x, unsigned int y)
{
	return (uint32_t) ((x >> y) | (x << (32 - y)));
}

#define blk0(i) (W[i] = data[i])
// Referencing parameters so many times is really poor practice.  Do not imitate these macros
#define blk2(i) (W[i & 15] += s1(W[(i - 2) & 15]) + W[(i - 7) & 15] + s0(W[(i - 15) & 15]))

#define Ch(x,y,z) (z ^ (x & (y ^ z)))
#define Maj(x,y,z) ((x & y) | (z & (x | y)))

#define a(i) T[(0 - i) & 7]
#define b(i) T[(1 - i) & 7]
#define c(i) T[(2 - i) & 7]
#define d(i) T[(3 - i) & 7]
#define e(i) T[(4 - i) & 7]
#define f(i) T[(5 - i) & 7]
#define g(i) T[(6 - i) & 7]
#define h(i) T[(7 - i) & 7]

// This is a bad way to write a multi-statement macro... and referencing 'i' so many
// times is really poor practice.  Do not imitate.
#define R(i) h(i) += S1( e(i)) + Ch(e(i), f(i), g(i)) + K[i + j] +(j ? blk2(i) : blk0(i));\
        d(i) += h(i);h(i) += S0(a(i)) + Maj(a(i), b(i), c(i))

// for SHA256
#define S0(x) (rotr32fixed(x, 2) ^ rotr32fixed(x, 13) ^ rotr32fixed(x, 22))
#define S1(x) (rotr32fixed(x, 6) ^ rotr32fixed(x, 11) ^ rotr32fixed(x, 25))
#define s0(x) (rotr32fixed(x, 7) ^ rotr32fixed(x, 18) ^ (x >> 3))
#define s1(x) (rotr32fixed(x, 17) ^ rotr32fixed(x, 19) ^ (x >> 10))

/*!
 * Initialize the Hash State
 *
 *  Constructs the SHA256 hash engine.
 *  Specification:
 *      State Size  = 32 bytes
 *      Block Size  = 64 bytes
 *      Digest Size = 32 bytes
 *
 * @param state     Address of hash state structure
 *
 */
void sha256_init(shw_hash_state_t * state)
{
	state->bit_count = 0;
	state->partial_count_bytes = 0;

	state->state[0] = 0x6a09e667;
	state->state[1] = 0xbb67ae85;
	state->state[2] = 0x3c6ef372;
	state->state[3] = 0xa54ff53a;
	state->state[4] = 0x510e527f;
	state->state[5] = 0x9b05688c;
	state->state[6] = 0x1f83d9ab;
	state->state[7] = 0x5be0cd19;
}

const uint32_t K[64] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
	0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
	0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
	0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
	0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
	0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
	0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
	0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
	0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

/*!
 * Hash a block of data into the SHA-256 hash state.
 *
 * This function hash the block of data in the @c partial_block
 * element of the state structure into the state variables of the
 * state structure.
 *
 * @param state     Address of hash state structure
 *
 */
static void sha256_process_block(shw_hash_state_t * state)
{
	uint32_t W[16];
	uint32_t T[8];
	uint32_t stack_buffer[SHW_HASH_BLOCK_WORD_SIZE];
	uint32_t *data = &stack_buffer[0];
	uint8_t *input = state->partial_block;
	unsigned int i;
	unsigned int j;

	/* Copy byte-oriented input block into word-oriented registers */
	for (i = 0; i < SHW_HASH_BLOCK_LEN / sizeof(uint32_t);
	     i++, input += sizeof(uint32_t)) {
		stack_buffer[i] = __be32_to_cpu(*(uint32_t *) input);
	}

	/* Copy context->state[] to working vars */
	memcpy(T, state->state, sizeof(T));

	/* 64 operations, partially loop unrolled */
	for (j = 0; j < SHW_HASH_BLOCK_LEN; j += 16) {
		R(0);
		R(1);
		R(2);
		R(3);
		R(4);
		R(5);
		R(6);
		R(7);
		R(8);
		R(9);
		R(10);
		R(11);
		R(12);
		R(13);
		R(14);
		R(15);
	}
	/* Add the working vars back into context.state[] */
	state->state[0] += a(0);
	state->state[1] += b(0);
	state->state[2] += c(0);
	state->state[3] += d(0);
	state->state[4] += e(0);
	state->state[5] += f(0);
	state->state[6] += g(0);
	state->state[7] += h(0);

	/* Wipe variables */
	memset(W, 0, sizeof(W));
	memset(T, 0, sizeof(T));
}

/*!
 * Initialize the hash state structure
 *
 * @param state     Address of hash state structure.
 * @param alg Which hash algorithm to use (must be FSL_HASH_ALG_SHA1)
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hash_init(shw_hash_state_t * state, fsl_shw_hash_alg_t alg)
{
	if (alg != FSL_HASH_ALG_SHA256) {
		return FSL_RETURN_BAD_ALGORITHM_S;
	}

	sha256_init(state);

	return FSL_RETURN_OK_S;
}

/*!
 * Add input bytes to the hash
 *
 * The bytes are added to the partial_block element of the hash state, and as
 * the partial block is filled, it is processed by sha1_process_block().  This
 * function also updates the bit_count element of the hash state.
 *
 * @param state     Address of hash state structure
 * @param input     Address of bytes to add to the hash
 * @param input_len Numbef of bytes at @c input
 *
 */
fsl_shw_return_t shw_hash_update(shw_hash_state_t * state,
				 const uint8_t * input, unsigned int input_len)
{
	unsigned int bytes_needed;	/* Needed to fill a block */
	unsigned int bytes_to_copy;	/* to copy into the block */

	/* Account for new data */
	state->bit_count += 8 * input_len;

	/*
	 * Process input bytes into the ongoing block; process the block when it
	 * gets full.
	 */
	while (input_len > 0) {
		bytes_needed = SHW_HASH_BLOCK_LEN - state->partial_count_bytes;
		bytes_to_copy = ((input_len < bytes_needed) ?
				 input_len : bytes_needed);

		/* Add in the bytes and do the accounting */
		memcpy(state->partial_block + state->partial_count_bytes,
		       input, bytes_to_copy);
		input += bytes_to_copy;
		input_len -= bytes_to_copy;
		state->partial_count_bytes += bytes_to_copy;

		/* Run a full block through the transform */
		if (state->partial_count_bytes == SHW_HASH_BLOCK_LEN) {
			sha256_process_block(state);
			state->partial_count_bytes = 0;
		}
	}

	return FSL_RETURN_OK_S;
}				/* end fn shw_hash_update */

/*!
 * Finalize the hash
 *
 * Performs the finalize operation on the previous input data & returns the
 * resulting digest.  The finalize operation performs the appropriate padding
 * up to the block size.
 *
 * @param state     Address of hash state structure
 * @param result    Location to store the hash result
 * @param result_len Number of bytes of @c result to be stored.
 *
 * @return FSL_RETURN_OK_S if all went well, FSL_RETURN_BAD_DATA_LENGTH_S if
 * hash_len is too long, otherwise an error code.
 */
fsl_shw_return_t shw_hash_final(shw_hash_state_t * state, uint8_t * result,
				unsigned int result_len)
{
	static const uint8_t pad[SHW_HASH_BLOCK_LEN * 2] = {
		0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	};

	uint8_t data[sizeof(state->bit_count)];
	uint32_t pad_length;
	uint64_t bit_count = state->bit_count;
	uint8_t hash[SHW_HASH_LEN];
	int i;

	if (result_len > SHW_HASH_LEN) {
		return FSL_RETURN_BAD_DATA_LENGTH_S;
	}

	/* Save the length before padding. */
	for (i = sizeof(state->bit_count) - 1; i >= 0; i--) {
		data[i] = bit_count & 0xFF;
		bit_count >>= 8;
	}
	pad_length = ((state->partial_count_bytes < 56) ?
		      (56 - state->partial_count_bytes) :
		      (120 - state->partial_count_bytes));

	/* Pad to 56 bytes mod 64 (BLOCK_SIZE). */
	shw_hash_update(state, pad, pad_length);

	/*
	 * Append the length.  This should trigger transform of the final block.
	 */
	shw_hash_update(state, data, sizeof(state->bit_count));

	/* Copy the result into a byte array */
	for (i = 0; i < SHW_HASH_STATE_WORDS; i++) {
		*(uint32_t *) (hash + 4 * i) = __cpu_to_be32(state->state[i]);
	}

	/* And copy the result out to caller */
	memcpy(result, hash, result_len);

	return FSL_RETURN_OK_S;
}				/* end fn shw_hash_final */
