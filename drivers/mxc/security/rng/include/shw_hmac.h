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
 * @file shw_hmac.h
 *
 * This file contains definitions for use of the (internal) SHW HMAC
 * software computation.  It defines the usual three steps:
 *
 * - #shw_hmac_init()
 * - #shw_hmac_update()
 * - #shw_hmac_final()
 *
 * The only other item of note to callers is #SHW_HASH_LEN, which is the number
 * of bytes calculated for the HMAC.
 */

#ifndef SHW_HMAC_H
#define SHW_HMAC_H

#include "shw_hash.h"

/*!
 * State for an HMAC
 *
 * Note to callers: This structure contains key material and should be kept in
 * a secure location, such as internal RAM.
 */
typedef struct shw_hmac_state {
	shw_hash_state_t inner_hash;	/*!< Current state of inner hash */
	shw_hash_state_t outer_hash;	/*!< Current state of outer hash */
} shw_hmac_state_t;

/*!
 * Initialize the HMAC state structure with the HMAC key
 *
 * @param state     Address of HMAC state structure.
 * @param key       Address of the key to be used for the HMAC.
 * @param key_len   Number of bytes of @c key.  This must not be greater than
 *                  the block size of the underlying hash (#SHW_HASH_BLOCK_LEN).
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hmac_init(shw_hmac_state_t * state,
			       const uint8_t * key, unsigned int key_len);

/*!
 * Put data into the HMAC calculation
 *
 * @param state     Address of HMAC state structure.
 * @param msg       Address of the message data for the HMAC.
 * @param msg_len   Number of bytes of @c msg.
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hmac_update(shw_hmac_state_t * state,
				 const uint8_t * msg, unsigned int msg_len);

/*!
 * Calculate the final HMAC
 *
 * @param state     Address of HMAC state structure.
 * @param hmac      Address of location to store the HMAC.
 * @param hmac_len  Number of bytes of @c mac to be stored.  Probably best if
 *                  this value is no greater than #SHW_HASH_LEN.
 *
 * @return  FSL_RETURN_OK_S if all went well, otherwise an error code.
 */
fsl_shw_return_t shw_hmac_final(shw_hmac_state_t * state,
				uint8_t * hmac, unsigned int hmac_len);

#endif				/* SHW_HMAC_H */
