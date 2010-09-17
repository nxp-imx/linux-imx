/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file adaptor.h
*
* @brief The Adaptor component provides an interface to the device
* driver.
*
* Intended to be used by the FSL SHW API, this can also be called directly
*/

#ifndef ADAPTOR_H
#define ADAPTOR_H

#include <sahara.h>

/*!
 * Structure passed during user ioctl() call to submit request.
 */
typedef struct sah_dar {
	sah_Desc *desc_addr;	/*!< head of descriptor chain */
	uint32_t uco_flags;	/*!< copy of fsl_shw_uco flags field */
	uint32_t uco_user_ref;	/*!< copy of fsl_shw_uco user_ref */
	uint32_t result;	/*!< result of descriptor chain request */
	struct sah_dar *next;	/*!< for driver use */
} sah_dar_t;

/*!
 * Structure passed during user ioctl() call to Register a user
 */
typedef struct sah_register {
	uint32_t pool_size;	/*!< max number of outstanding requests possible */
	uint32_t result;	/*!< result of registration request */
} sah_register_t;

/*!
 * Structure passed during ioctl() call to request SCC operation
 */
typedef struct scc_data {
	uint32_t length;	/*!< length of data */
	uint8_t *in;		/*!< input data */
	uint8_t *out;		/*!< output data */
	unsigned direction;	/*!< encrypt or decrypt */
	fsl_shw_sym_mode_t crypto_mode;	/*!< CBC or EBC */
	uint8_t *init_vector;	/*!< initialization vector or NULL */
} scc_data_t;

/*!
 * Structure passed during user ioctl() calls to manage stored keys and
 * stored-key slots.
 */
typedef struct scc_slot_t {
	uint64_t ownerid;	/*!< Owner's id to check/set permissions */
	uint32_t key_length;	/*!< Length of key */
	uint32_t slot;		/*!< Slot to operation on, or returned slot
				   number. */
	uint8_t *key;		/*!< User-memory pointer to key value */
	fsl_shw_return_t code;	/*!< API return code from operation */
} scc_slot_t;

/*
 * Structure passed during user ioctl() calls to manage data stored in secure
 * partitions.
 */
typedef struct scc_region_t {
    uint32_t partition_base;	/*!< User virtual address of the
					   partition base. */
	uint32_t offset;	/*!< Offset from the start of the
				   partition where the cleartext data
				   is located. */
	uint32_t length;	/*!< Length of the region to be
				   operated on */
	uint8_t *black_data;	/*!< User virtual address of any black
				   (encrypted) data. */
	fsl_shw_cypher_mode_t cypher_mode;	/*!< Cypher mode to use in an encryt/
						   decrypt operation. */
	uint32_t IV[4];		/*!< Intialization vector to use in an
				   encrypt/decrypt operation. */
	fsl_shw_return_t code;	/*!< API return code from operation */
} scc_region_t;

/*
 * Structure passed during user ioctl() calls to manage secure partitions.
 */
typedef struct scc_partition_info_t {
    uint32_t user_base;            /**< Userspace pointer to base of partition */
    uint32_t permissions;          /**< Permissions to give the partition (only
                                        used in call to _DROP_PERMS) */
	fsl_shw_partition_status_t status;	/*!< Status of the partition */
} scc_partition_info_t;

fsl_shw_return_t adaptor_Exec_Descriptor_Chain(sah_Head_Desc * dar,
					       fsl_shw_uco_t * uco);
fsl_shw_return_t sah_get_results(sah_results * arg, fsl_shw_uco_t * uco);
fsl_shw_return_t sah_register(fsl_shw_uco_t * user_ctx);
fsl_shw_return_t sah_deregister(fsl_shw_uco_t * user_ctx);
fsl_shw_return_t get_capabilities(fsl_shw_uco_t * user_ctx,
							fsl_shw_pco_t *capabilities);

#endif				/* ADAPTOR_H */

/* End of adaptor.h */
