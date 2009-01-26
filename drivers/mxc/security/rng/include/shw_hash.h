/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @file shw_hash.h
 *
 * This file contains definitions for use of the (internal) SHW hash
 * software computation.  It defines the usual three steps:
 *
 * - #shw_hash_init()
 * - #shw_hash_update()
 * - #shw_hash_final()
 *
 * The only other item of note to callers is #SHW_HASH_LEN, which is the number
 * of bytes calculated for the hash.
 */

#ifndef SHW_HASH_H
#define SHW_HASH_H

/*! Define which gives the number of bytes available in an hash result */
#define SHW_HASH_LEN 32

/* Define which matches block length in bytes of the underlying hash */
#define SHW_HASH_BLOCK_LEN 64

/* "Internal" define which matches SHA-256 state size (32-bit words) */
#define SHW_HASH_STATE_WORDS 8

/* "Internal" define which matches word length in blocks of the underlying
   hash. */
#define SHW_HASH_BLOCK_WORD_SIZE 16

#define SHW_HASH_STATE_SIZE 32

/*!
 * State for a SHA-1/SHA-2 Hash
 *
 * (Note to maintainers: state needs to be updated to uint64_t to handle
 * SHA-384/SHA-512)... And bit_count to uint128_t (heh).
 */
typedef struct shw_hash_state {
	unsigned int partial_count_bytes;	/*!< Number of bytes of message sitting
						 * in @c partial_block */
	uint8_t partial_block[SHW_HASH_BLOCK_LEN];	/*!< Data waiting to be processed as a block  */
	uint32_t state[SHW_HASH_STATE_WORDS];	/*!< Current hash state variables */
	uint64_t bit_count;	/*!< Number of bits sent through the update function */
} shw_hash_state_t;

/*!
 * Initialize the hash state structure
 *
 * @param state     Address of hash state structure.
 * @param algorithm Which hash algorithm to use (must be FSL_HASH_ALG_SHA256)
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hash_init(shw_hash_state_t * state,
			       fsl_shw_hash_alg_t algorithm);

/*!
 * Put data into the hash calculation
 *
 * @param state     Address of hash state structure.
 * @param msg       Address of the message data for the hash.
 * @param msg_len   Number of bytes of @c msg.
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hash_update(shw_hash_state_t * state,
				 const uint8_t * msg, unsigned int msg_len);

/*!
 * Calculate the final hash value
 *
 * @param state     Address of hash state structure.
 * @param hash      Address of location to store the hash.
 * @param hash_len  Number of bytes of @c hash to be stored.
 *
 * @return FSL_RETURN_OK_S if all went well, FSL_RETURN_BAD_DATA_LENGTH_S if
 * hash_len is too long, otherwise an error code.
 */
fsl_shw_return_t shw_hash_final(shw_hash_state_t * state,
				uint8_t * hash, unsigned int hash_len);

#endif				/* SHW_HASH_H */
