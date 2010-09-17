/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/**
 * @file fsl_shw_keystore.c
 *
 * File which implements a default keystore policy, for use as the system
 * keystore.
 */
#include "fsl_platform.h"
#include "fsl_shw.h"
#include "fsl_shw_keystore.h"

#if defined(DIAG_DRV_IF)
#include <diagnostic.h>
#endif

#if !defined(FSL_HAVE_SCC2) && defined(__KERNEL__)
#include <linux/mxc_scc_driver.h>
#endif

/* Define a semaphore to protect the keystore data */
#ifdef __KERNEL__
#define LOCK_INCLUDES os_lock_context_t context
#define ACQUIRE_LOCK os_lock_save_context(keystore->lock, context)
#define RELEASE_LOCK os_unlock_restore_context(keystore->lock, context);
#else
#define LOCK_INCLUDES
#define ACQUIRE_LOCK
#define RELEASE_LOCK
#endif	/* __KERNEL__ */

/*!
 * Calculates the byte offset into a word
 *  @param   bp  The byte (char*) pointer
 *  @return      The offset (0, 1, 2, or 3)
 */
#define SCC_BYTE_OFFSET(bp) ((uint32_t)(bp) % sizeof(uint32_t))

/*!
 * Converts (by rounding down) a byte pointer into a word pointer
 *  @param  bp  The byte (char*) pointer
 *  @return     The word (uint32_t) as though it were an aligned (uint32_t*)
 */
#define SCC_WORD_PTR(bp) (((uint32_t)(bp)) & ~(sizeof(uint32_t)-1))

/* Depending on the architecture, these functions should be defined
 * differently.  On Platforms with SCC2, the functions use the secure
 * partition interface and should be available in both user and kernel space.
 * On platforms with SCC, they use the SCC keystore interface.  This is only
 * available in kernel mode, so they should be stubbed out in user mode.
 */
#if defined(FSL_HAVE_SCC2) || (defined(FSL_HAVE_SCC) && defined(__KERNEL__))
EXPORT_SYMBOL(fsl_shw_init_keystore);
void fsl_shw_init_keystore(
			    fsl_shw_kso_t *keystore,
			    fsl_shw_return_t(*data_init) (fsl_shw_uco_t *user_ctx,
							   void **user_data),
			    void (*data_cleanup) (fsl_shw_uco_t *user_ctx,
						   void **user_data),
			    fsl_shw_return_t(*slot_alloc) (void *user_data,
							    uint32_t size,
							    uint64_t owner_id,
							    uint32_t *slot),
			    fsl_shw_return_t(*slot_dealloc) (void *user_data,
							      uint64_t
							      owner_id,
							      uint32_t slot),
			    fsl_shw_return_t(*slot_verify_access) (void
								    *user_data,
								    uint64_t
								    owner_id,
								    uint32_t
								    slot),
			    void *(*slot_get_address) (void *user_data,
							uint32_t handle),
			    uint32_t(*slot_get_base) (void *user_data,
						       uint32_t handle),
			    uint32_t(*slot_get_offset) (void *user_data,
							 uint32_t handle),
			    uint32_t(*slot_get_slot_size) (void *user_data,
							    uint32_t handle))
{
	keystore->data_init = data_init;
	keystore->data_cleanup = data_cleanup;
	keystore->slot_alloc = slot_alloc;
	keystore->slot_dealloc = slot_dealloc;
	keystore->slot_verify_access = slot_verify_access;
	keystore->slot_get_address = slot_get_address;
	keystore->slot_get_base = slot_get_base;
	keystore->slot_get_offset = slot_get_offset;
	keystore->slot_get_slot_size = slot_get_slot_size;
}

EXPORT_SYMBOL(fsl_shw_init_keystore_default);
void fsl_shw_init_keystore_default(fsl_shw_kso_t *keystore)
{
	keystore->data_init = shw_kso_init_data;
	keystore->data_cleanup = shw_kso_cleanup_data;
	keystore->slot_alloc = shw_slot_alloc;
	keystore->slot_dealloc = shw_slot_dealloc;
	keystore->slot_verify_access = shw_slot_verify_access;
	keystore->slot_get_address = shw_slot_get_address;
	keystore->slot_get_base = shw_slot_get_base;
	keystore->slot_get_offset = shw_slot_get_offset;
	keystore->slot_get_slot_size = shw_slot_get_slot_size;
}

/*!
 * Do any keystore specific initializations
 */
EXPORT_SYMBOL(fsl_shw_establish_keystore);
fsl_shw_return_t fsl_shw_establish_keystore(fsl_shw_uco_t *user_ctx,
						fsl_shw_kso_t *keystore)
{
	if (keystore->data_init == NULL) {
		return FSL_RETURN_ERROR_S;
	}

    /* Call the data_init function for any user setup */
    return keystore->data_init(user_ctx, &(keystore->user_data));
}

EXPORT_SYMBOL(fsl_shw_release_keystore);
void fsl_shw_release_keystore(fsl_shw_uco_t *user_ctx,
				 fsl_shw_kso_t *keystore)
{

    /* Call the data_cleanup function for any keystore cleanup.
	  * NOTE: The keystore doesn't have any way of telling which keys are using
	  * it, so it is up to the user program to manage their key objects
	  * correctly.
	  */
	 if ((keystore != NULL) && (keystore->data_cleanup != NULL)) {
		keystore->data_cleanup(user_ctx, &(keystore->user_data));
	}
	return;
}

fsl_shw_return_t keystore_slot_alloc(fsl_shw_kso_t *keystore, uint32_t size,
				       uint64_t owner_id, uint32_t *slot)
{
	LOCK_INCLUDES;
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;

#ifdef DIAG_DRV_IF
	    LOG_DIAG("In keystore_slot_alloc.");

#endif
	ACQUIRE_LOCK;
	if ((keystore->slot_alloc == NULL) || (keystore->user_data == NULL)) {
		goto out;
	}

#ifdef DIAG_DRV_IF
	    LOG_DIAG_ARGS("key length: %i, handle: %i\n", size, *slot);

#endif
retval = keystore->slot_alloc(keystore->user_data, size, owner_id, slot);
out:RELEASE_LOCK;
	return retval;
}

fsl_shw_return_t keystore_slot_dealloc(fsl_shw_kso_t *keystore,
					 uint64_t owner_id, uint32_t slot)
{
	LOCK_INCLUDES;
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;
	ACQUIRE_LOCK;
	if ((keystore->slot_alloc == NULL) || (keystore->user_data == NULL)) {
		goto out;
	}
	retval =
	    keystore->slot_dealloc(keystore->user_data, owner_id, slot);
out:RELEASE_LOCK;
	return retval;
}

fsl_shw_return_t
keystore_slot_load(fsl_shw_kso_t * keystore, uint64_t owner_id, uint32_t slot,
		   const uint8_t * key_data, uint32_t key_length)
{

#ifdef FSL_HAVE_SCC2
	    LOCK_INCLUDES;
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;
	uint32_t slot_size;
	uint32_t i;
	uint8_t * slot_location;
	ACQUIRE_LOCK;
	if ((keystore->slot_verify_access == NULL) ||
	      (keystore->user_data == NULL))
		goto out;
	if (keystore->
	      slot_verify_access(keystore->user_data, owner_id,
				 slot) !=FSL_RETURN_OK_S) {
		retval = FSL_RETURN_AUTH_FAILED_S;
		goto out;
	}
	slot_size = keystore->slot_get_slot_size(keystore->user_data, slot);
	if (key_length > slot_size) {
		retval = FSL_RETURN_BAD_DATA_LENGTH_S;
		goto out;
	}
	slot_location = keystore->slot_get_address(keystore->user_data, slot);
	for (i = 0; i < key_length; i++) {
		slot_location[i] = key_data[i];
	}
	retval = FSL_RETURN_OK_S;
out:RELEASE_LOCK;
	return retval;

#else	/* FSL_HAVE_SCC2 */
	fsl_shw_return_t retval;
	scc_return_t scc_ret;
	scc_ret =
	    scc_load_slot(owner_id, slot, (uint8_t *) key_data, key_length);
	switch (scc_ret) {
	case SCC_RET_OK:
		retval = FSL_RETURN_OK_S;
		break;
	case SCC_RET_VERIFICATION_FAILED:
		retval = FSL_RETURN_AUTH_FAILED_S;
		break;
	case SCC_RET_INSUFFICIENT_SPACE:
		retval = FSL_RETURN_BAD_DATA_LENGTH_S;
		break;
	default:
		retval = FSL_RETURN_ERROR_S;
	}
	return retval;

#endif	/* FSL_HAVE_SCC2 */
}

fsl_shw_return_t
keystore_slot_read(fsl_shw_kso_t * keystore, uint64_t owner_id, uint32_t slot,
		   uint32_t key_length, uint8_t * key_data)
{
#ifdef FSL_HAVE_SCC2
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;
	uint8_t *slot_addr;
	uint32_t slot_size;

	slot_addr = keystore->slot_get_address(keystore->user_data, slot);
	slot_size = keystore->slot_get_slot_size(keystore->user_data, slot);

	if (key_length > slot_size) {
		retval = FSL_RETURN_BAD_KEY_LENGTH_S;
		goto out;
	}

	memcpy(key_data, slot_addr, key_length);
	retval = FSL_RETURN_OK_S;

      out:
	return retval;

#else				/* Have SCC2 */
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;
	scc_return_t scc_ret;
	printk("keystore SCC \n");

	scc_ret =
	    scc_read_slot(owner_id, slot, key_length, (uint8_t *) key_data);
	printk("keystore SCC Ret value: %d \n", scc_ret);
	switch (scc_ret) {
	case SCC_RET_OK:
		retval = FSL_RETURN_OK_S;
		break;
	case SCC_RET_VERIFICATION_FAILED:
		retval = FSL_RETURN_AUTH_FAILED_S;
		break;
	case SCC_RET_INSUFFICIENT_SPACE:
		retval = FSL_RETURN_BAD_DATA_LENGTH_S;
		break;
	default:
		retval = FSL_RETURN_ERROR_S;
	}

	return retval;

#endif				/* FSL_HAVE_SCC2 */
}/* end fn keystore_slot_read */

fsl_shw_return_t
keystore_slot_encrypt(fsl_shw_uco_t *user_ctx, fsl_shw_kso_t *keystore,
		      uint64_t owner_id, uint32_t slot, uint32_t length,
		      uint8_t *destination)
{

#ifdef FSL_HAVE_SCC2
	LOCK_INCLUDES;
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;
	uint32_t slot_length;
	uint32_t IV[4];
	uint32_t * iv_ptr = (uint32_t *) & (owner_id);

	/* Build the IV */
	IV[0] = iv_ptr[0];
	IV[1] = iv_ptr[1];
	IV[2] = 0;
	IV[3] = 0;
	ACQUIRE_LOCK;

	/* Ensure that the data will fit in the key slot */
	slot_length =
	    keystore->slot_get_slot_size(keystore->user_data, slot);
	if (length > slot_length) {
		goto out;
	}

	  /* Call scc encrypt function to encrypt the data. */
	    retval = do_scc_encrypt_region(user_ctx,
					   (void *)keystore->
					   slot_get_base(keystore->user_data,
							 slot),
					   keystore->slot_get_offset(keystore->
								      user_data,
								      slot),
					   length, destination, IV,
					   FSL_SHW_CYPHER_MODE_CBC);
	goto out;
out:RELEASE_LOCK;
	return retval;

#else
	scc_return_t retval;
	retval = scc_encrypt_slot(owner_id, slot, length, destination);
	if (retval == SCC_RET_OK)
		return FSL_RETURN_OK_S;
	return FSL_RETURN_ERROR_S;

#endif	/* FSL_HAVE_SCC2 */
}

fsl_shw_return_t
keystore_slot_decrypt(fsl_shw_uco_t *user_ctx, fsl_shw_kso_t *keystore,
		      uint64_t owner_id, uint32_t slot, uint32_t length,
		      const uint8_t *source)
{

#ifdef FSL_HAVE_SCC2
	LOCK_INCLUDES;
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;
	uint32_t slot_length;
	uint32_t IV[4];
	uint32_t *iv_ptr = (uint32_t *) & (owner_id);

    /* Build the IV */
    IV[0] = iv_ptr[0];
	IV[1] = iv_ptr[1];
	IV[2] = 0;
	IV[3] = 0;
	ACQUIRE_LOCK;

	/* Call scc decrypt function to decrypt the data. */

	/* Ensure that the data will fit in the key slot */
	    slot_length =
	    keystore->slot_get_slot_size(keystore->user_data, slot);
	if (length > slot_length)
		goto out;

	/* Call scc decrypt function to encrypt the data. */
	    retval = do_scc_decrypt_region(user_ctx,
					   (void *)keystore->
					   slot_get_base(keystore->user_data,
							 slot),
					   keystore->slot_get_offset(keystore->
								      user_data,
								      slot),
					   length, source, IV,
					   FSL_SHW_CYPHER_MODE_CBC);
	goto out;
out:RELEASE_LOCK;
	return retval;

#else
	scc_return_t retval;
	retval = scc_decrypt_slot(owner_id, slot, length, source);
	if (retval == SCC_RET_OK)
		return FSL_RETURN_OK_S;
	return FSL_RETURN_ERROR_S;

#endif	/* FSL_HAVE_SCC2 */
}

#else	/* SCC in userspace */
void fsl_shw_init_keystore(
			    fsl_shw_kso_t *keystore,
			    fsl_shw_return_t(*data_init) (fsl_shw_uco_t *user_ctx,
							   void **user_data),
			    void (*data_cleanup) (fsl_shw_uco_t *user_ctx,
						   void **user_data),
			    fsl_shw_return_t(*slot_alloc) (void *user_data,
							    uint32_t size,
							    uint64_t owner_id,
							    uint32_t *slot),
			    fsl_shw_return_t(*slot_dealloc) (void *user_data,
							      uint64_t
							      owner_id,
							      uint32_t slot),
			    fsl_shw_return_t(*slot_verify_access) (void
								    *user_data,
								    uint64_t
								    owner_id,
								    uint32_t
								    slot),
			    void *(*slot_get_address) (void *user_data,
							uint32_t handle),
			    uint32_t(*slot_get_base) (void *user_data,
						       uint32_t handle),
			    uint32_t(*slot_get_offset) (void *user_data,
							 uint32_t handle),
			    uint32_t(*slot_get_slot_size) (void *user_data,
							    uint32_t handle))
{
	(void)keystore;
	(void)data_init;
	(void)data_cleanup;
	(void)slot_alloc;
	(void)slot_dealloc;
	(void)slot_verify_access;
	(void)slot_get_address;
	(void)slot_get_base;
	(void)slot_get_offset;
	(void)slot_get_slot_size;
}

void fsl_shw_init_keystore_default(fsl_shw_kso_t * keystore)
{
	(void)keystore;
}
fsl_shw_return_t fsl_shw_establish_keystore(fsl_shw_uco_t *user_ctx,
						 fsl_shw_kso_t *keystore)
{
	(void)user_ctx;
	(void)keystore;
	return FSL_RETURN_NO_RESOURCE_S;
}
void fsl_shw_release_keystore(fsl_shw_uco_t *user_ctx,
				fsl_shw_kso_t *keystore)
{
	(void)user_ctx;
	(void)keystore;
	return;
}

fsl_shw_return_t keystore_slot_alloc(fsl_shw_kso_t *keystore, uint32_t size,
					uint64_t owner_id, uint32_t *slot)
{
	(void)keystore;
	(void)size;
	(void)owner_id;
	(void)slot;
	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t keystore_slot_dealloc(fsl_shw_kso_t *keystore,
					 uint64_t owner_id, uint32_t slot)
{
	(void)keystore;
	(void)owner_id;
	(void)slot;
	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t
keystore_slot_load(fsl_shw_kso_t *keystore, uint64_t owner_id, uint32_t slot,
		   const uint8_t *key_data, uint32_t key_length)
{
	(void)keystore;
	(void)owner_id;
	(void)slot;
	(void)key_data;
	(void)key_length;
	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t
keystore_slot_read(fsl_shw_kso_t * keystore, uint64_t owner_id, uint32_t slot,
		   uint32_t key_length, uint8_t * key_data)
{
	(void)keystore;
	(void)owner_id;
	(void)slot;
	(void)key_length;
	(void)key_data;

	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t
keystore_slot_decrypt(fsl_shw_uco_t *user_ctx, fsl_shw_kso_t *keystore,
		      uint64_t owner_id, uint32_t slot, uint32_t length,
		      const uint8_t *source)
{
	(void)user_ctx;
	(void)keystore;
	(void)owner_id;
	(void)slot;
	(void)length;
	(void)source;
	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t
keystore_slot_encrypt(fsl_shw_uco_t *user_ctx, fsl_shw_kso_t *keystore,
		      uint64_t owner_id, uint32_t slot, uint32_t length,
		      uint8_t *destination)
{
	(void)user_ctx;
	(void)keystore;
	(void)owner_id;
	(void)slot;
	(void)length;
	(void)destination;
	return FSL_RETURN_NO_RESOURCE_S;
}


#endif	/* FSL_HAVE_SCC2 */

/***** Default keystore implementation **************************************/

#ifdef FSL_HAVE_SCC2
    fsl_shw_return_t shw_kso_init_data(fsl_shw_uco_t *user_ctx,
					void **user_data)
{
	int retval = FSL_RETURN_ERROR_S;
	keystore_data_t *keystore_data = NULL;
	fsl_shw_pco_t *capabilities = fsl_shw_get_capabilities(user_ctx);
	uint32_t partition_size;
	uint32_t slot_count;
	uint32_t keystore_data_size;
	uint8_t UMID[16] = {
	0x42, 0, 0, 0, 0x43, 0, 0, 0, 0x19, 0, 0, 0, 0x59, 0, 0, 0};
	uint32_t permissions =
	    FSL_PERM_TH_R | FSL_PERM_TH_W | FSL_PERM_HD_R | FSL_PERM_HD_W |
	    FSL_PERM_HD_X;

	/* Look up the size of a partition to see how big to make the keystore */
	partition_size = fsl_shw_pco_get_spo_size_bytes(capabilities);

	/* Calculate the required size of the keystore data structure, based on the
	  * number of keys that can fit in the partition.
	  */
	slot_count = partition_size / KEYSTORE_SLOT_SIZE;
	keystore_data_size =
	    sizeof(keystore_data_t) +
	    slot_count * sizeof(keystore_data_slot_info_t);

#ifdef __KERNEL__
	keystore_data = os_alloc_memory(keystore_data_size, GFP_KERNEL);

#else
	keystore_data = malloc(keystore_data_size);

#endif
	if (keystore_data == NULL) {
		retval = FSL_RETURN_NO_RESOURCE_S;
		goto out;
	}

    /* Clear the memory (effectively clear all key assignments) */
    memset(keystore_data, 0, keystore_data_size);

   /* Place the slot information structure directly after the keystore data
	* structure.
	*/
	    keystore_data->slot =
	    (keystore_data_slot_info_t *) (keystore_data + 1);
	keystore_data->slot_count = slot_count;

	/* Retrieve a secure partition to put the keystore in. */
	keystore_data->base_address =
	    fsl_shw_smalloc(user_ctx, partition_size, UMID, permissions);
	if (keystore_data->base_address == NULL) {
		retval = FSL_RETURN_NO_RESOURCE_S;
		goto out;
	}
	*user_data = keystore_data;
	retval = FSL_RETURN_OK_S;
out:if (retval != FSL_RETURN_OK_S) {
		if (keystore_data != NULL) {
			if (keystore_data->base_address != NULL)
				fsl_shw_sfree(NULL,
					       keystore_data->base_address);

#ifdef __KERNEL__
			    os_free_memory(keystore_data);

#else
			    free(keystore_data);

#endif
		}
	}
	return retval;
}
void shw_kso_cleanup_data(fsl_shw_uco_t *user_ctx, void **user_data)
{
	if (user_data != NULL) {
		keystore_data_t * keystore_data =
		    (keystore_data_t *) (*user_data);
		fsl_shw_sfree(user_ctx, keystore_data->base_address);

#ifdef __KERNEL__
		    os_free_memory(*user_data);

#else
		    free(*user_data);

#endif
	}
	return;
}

fsl_shw_return_t shw_slot_verify_access(void *user_data, uint64_t owner_id,
					  uint32_t slot)
{
	keystore_data_t * data = user_data;
	if (data->slot[slot].owner == owner_id) {
		return FSL_RETURN_OK_S;
	} else {

#ifdef DIAG_DRV_IF
		    LOG_DIAG_ARGS("Access to slot %i fails.\n", slot);

#endif
		    return FSL_RETURN_AUTH_FAILED_S;
	}
}

fsl_shw_return_t shw_slot_alloc(void *user_data, uint32_t size,
				   uint64_t owner_id, uint32_t *slot)
{
	keystore_data_t *data = user_data;
	uint32_t i;
	if (size > KEYSTORE_SLOT_SIZE)
		return FSL_RETURN_BAD_KEY_LENGTH_S;
	for (i = 0; i < data->slot_count; i++) {
		if (data->slot[i].allocated == 0) {
			data->slot[i].allocated = 1;
			data->slot[i].owner = owner_id;
			(*slot) = i;

#ifdef DIAG_DRV_IF
			    LOG_DIAG_ARGS("Keystore: allocated slot %i. Slot "
					  "address: %p\n",
					  (*slot),
					  data->base_address +
					  (*slot) * KEYSTORE_SLOT_SIZE);

#endif
			    return FSL_RETURN_OK_S;
		}
	}
	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t shw_slot_dealloc(void *user_data, uint64_t owner_id,
				    uint32_t slot)
{
	keystore_data_t * data = user_data;
	(void)owner_id;
	(void)slot;
	if (slot >= data->slot_count)
		return FSL_RETURN_ERROR_S;
	if (data->slot[slot].allocated == 1) {
		/* Forcibly remove the data from the keystore */
		memset(shw_slot_get_address(user_data, slot), 0,
		       KEYSTORE_SLOT_SIZE);
		data->slot[slot].allocated = 0;
		return FSL_RETURN_OK_S;
	}
	return FSL_RETURN_ERROR_S;
}

void *shw_slot_get_address(void *user_data, uint32_t slot)
{
	keystore_data_t * data = user_data;
	if (slot >= data->slot_count)
		return NULL;
	return data->base_address + slot * KEYSTORE_SLOT_SIZE;
}

uint32_t shw_slot_get_base(void *user_data, uint32_t slot)
{
	keystore_data_t * data = user_data;

	/* There could potentially be more than one secure partition object
	  * associated with this keystore.  For now, there is just one.
	  */
	(void)slot;
	return (uint32_t) (data->base_address);
}

uint32_t shw_slot_get_offset(void *user_data, uint32_t slot)
{
	keystore_data_t *data = user_data;
	if (slot >= data->slot_count)
		return FSL_RETURN_ERROR_S;
	return (slot * KEYSTORE_SLOT_SIZE);
}

uint32_t shw_slot_get_slot_size(void *user_data, uint32_t slot)
{
	(void)user_data;
	(void)slot;

	/* All slots are the same size in the default implementation */
	return KEYSTORE_SLOT_SIZE;
}

#else	/* FSL_HAVE_SCC2 */

#ifdef __KERNEL__
    fsl_shw_return_t shw_kso_init_data(fsl_shw_uco_t *user_ctx,
					void **user_data)
{

   /* The SCC does its own initialization.  All that needs to be done here is
	* make sure an SCC exists.
	*/
	*user_data = (void *)0xFEEDFEED;
	return FSL_RETURN_OK_S;
}
void shw_kso_cleanup_data(fsl_shw_uco_t *user_ctx, void **user_data)
{

    /* The SCC does its own cleanup. */
	*user_data = NULL;
	return;
}

fsl_shw_return_t shw_slot_verify_access(void *user_data, uint64_t owner_id,
					  uint32_t slot)
{

	/* Zero is used for the size because the newer interface does bounds
	  * checking later.
	  */
	scc_return_t retval;
	retval = scc_verify_slot_access(owner_id, slot, 0);
	if (retval == SCC_RET_OK) {
		return FSL_RETURN_OK_S;
	}
	return FSL_RETURN_AUTH_FAILED_S;
}

fsl_shw_return_t shw_slot_alloc(void *user_data, uint32_t size,
				   uint64_t owner_id, uint32_t *slot)
{
	scc_return_t retval;

#ifdef DIAG_DRV_IF
	    LOG_DIAG_ARGS("key length: %i, handle: %i\n", size, *slot);

#endif
	retval = scc_alloc_slot(size, owner_id, slot);
	if (retval == SCC_RET_OK)
		return FSL_RETURN_OK_S;

	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t shw_slot_dealloc(void *user_data, uint64_t owner_id,
				    uint32_t slot)
{
	scc_return_t retval;
	retval = scc_dealloc_slot(owner_id, slot);
	if (retval == SCC_RET_OK)
		return FSL_RETURN_OK_S;

	return FSL_RETURN_ERROR_S;
}
void *shw_slot_get_address(void *user_data, uint32_t slot)
{
	uint64_t owner_id = *((uint64_t *) user_data);
	uint32_t address;
	uint32_t value_size_bytes;
	uint32_t slot_size_bytes;
	scc_return_t scc_ret;
	scc_ret =
	    scc_get_slot_info(owner_id, slot, &address, &value_size_bytes,
			      &slot_size_bytes);
	if (scc_ret == SCC_RET_OK) {
		return (void *)address;
	}
	return NULL;
}

uint32_t shw_slot_get_base(void *user_data, uint32_t slot)
{
	return 0;
}

uint32_t shw_slot_get_offset(void *user_data, uint32_t slot)
{
	return 0;
}


/* Return the size of the key slot, in octets */
uint32_t shw_slot_get_slot_size(void *user_data, uint32_t slot)
{
	uint64_t owner_id = *((uint64_t *) user_data);
	uint32_t address;
	uint32_t value_size_bytes;
	uint32_t slot_size_bytes;
	scc_return_t scc_ret;
	scc_ret =
	    scc_get_slot_info(owner_id, slot, &address, &value_size_bytes,
			      &slot_size_bytes);
	if (scc_ret == SCC_RET_OK)
		return slot_size_bytes;
	return 0;
}


#endif	/* __KERNEL__ */

#endif	/* FSL_HAVE_SCC2 */

/*****************************************************************************/
