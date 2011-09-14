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
 * @file shw_hmac.c
 *
 * This file contains implementations for use of the (internal) SHW HMAC
 * software computation.  It defines the usual three steps:
 *
 * - #shw_hmac_init()
 * - #shw_hmac_update()
 * - #shw_hmac_final()
 *
 *
 */

#include "shw_driver.h"
#include "shw_hmac.h"

#ifndef __KERNEL__
#include <asm/types.h>
#include <linux/byteorder/little_endian.h>	/* or whichever is proper for target arch */
#define printk printf
#endif

/*! XOR value for HMAC inner key */
#define INNER_HASH_CONSTANT 0x36

/*! XOR value for HMAC outer key */
#define OUTER_HASH_CONSTANT 0x5C

/*!
 * Initialize the HMAC state structure with the HMAC key
 *
 * @param state     Address of HMAC state structure
 * @param key       Address of the key to be used for the HMAC.
 * @param key_len   Number of bytes of @c key.
 *
 * Convert the key into its equivalent inner and outer hash state objects.
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hmac_init(shw_hmac_state_t * state,
			       const uint8_t * key, unsigned int key_len)
{
	fsl_shw_return_t code = FSL_RETURN_ERROR_S;
	uint8_t first_block[SHW_HASH_BLOCK_LEN];
	unsigned int i;

	/* Don't bother handling the pre-hash. */
	if (key_len > SHW_HASH_BLOCK_LEN) {
		code = FSL_RETURN_BAD_KEY_LENGTH_S;
		goto out;
	}

	/* Prepare inner hash */
	for (i = 0; i < SHW_HASH_BLOCK_LEN; i++) {
		if (i < key_len) {
			first_block[i] = key[i] ^ INNER_HASH_CONSTANT;
		} else {
			first_block[i] = INNER_HASH_CONSTANT;
		}
	}
	code = shw_hash_init(&state->inner_hash, FSL_HASH_ALG_SHA256);
	if (code != FSL_RETURN_OK_S) {
		goto out;
	}
	shw_hash_update(&state->inner_hash, first_block, SHW_HASH_BLOCK_LEN);

	/* Prepare outer hash */
	for (i = 0; i < SHW_HASH_BLOCK_LEN; i++) {
		if (i < key_len) {
			first_block[i] = key[i] ^ OUTER_HASH_CONSTANT;
		} else {
			first_block[i] = OUTER_HASH_CONSTANT;
		}
	}
	code = shw_hash_init(&state->outer_hash, FSL_HASH_ALG_SHA256);
	if (code != FSL_RETURN_OK_S) {
		goto out;
	}
	shw_hash_update(&state->outer_hash, first_block, SHW_HASH_BLOCK_LEN);

	/* Wipe evidence of key */
	memset(first_block, 0, SHW_HASH_BLOCK_LEN);

      out:
	return code;
}

/*!
 * Put data into the HMAC calculation
 *
 * Send the msg data inner inner hash's update function.
 *
 * @param state     Address of HMAC state structure.
 * @param msg       Address of the message data for the HMAC.
 * @param msg_len   Number of bytes of @c msg.
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hmac_update(shw_hmac_state_t * state,
				 const uint8_t * msg, unsigned int msg_len)
{
	shw_hash_update(&state->inner_hash, msg, msg_len);

	return FSL_RETURN_OK_S;
}

/*!
 * Calculate the final HMAC
 *
 * @param state     Address of HMAC state structure.
 * @param hmac      Address of location to store the HMAC.
 * @param hmac_len  Number of bytes of @c mac to be stored.  Probably best if
 *                  this value is no greater than #SHW_HASH_LEN.
 *
 * This function finalizes the internal hash, and uses that result as
 * data for the outer hash.  As many bytes of that result are passed
 * to the user as desired.
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hmac_final(shw_hmac_state_t * state,
				uint8_t * hmac, unsigned int hmac_len)
{
	uint8_t hash_result[SHW_HASH_LEN];

	shw_hash_final(&state->inner_hash, hash_result, sizeof(hash_result));
	shw_hash_update(&state->outer_hash, hash_result, SHW_HASH_LEN);

	shw_hash_final(&state->outer_hash, hmac, hmac_len);

	return FSL_RETURN_OK_S;
}
