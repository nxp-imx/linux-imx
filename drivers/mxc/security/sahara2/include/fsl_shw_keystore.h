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


#ifndef FSL_SHW_KEYSTORE_H
#define FSL_SHW_KEYSTORE_H

/*!
 * @file fsl_shw_keystore.h
 *
 * @brief Definition of the User Keystore API.
 *
 */

/*! \page user_keystore User Keystore API
 *
 * Definition of the User Keystore API.
 *
 * On platforms with multiple partitions of Secure Memory, the Keystore Object
 * (#fsl_shw_kso_t) is provided to allow users to manage a private keystore for
 * use in software cryptographic routines.  The user can define a custom set of
 * methods for managing their keystore, or use a default keystore handler.  The
 * keystore is established by #fsl_shw_establish_keystore(), and released by
 * #fsl_shw_release_keystore().  The intent of this design is to make the
 * keystore implementation as flexible as possible.
 *
 * See @ref keystore_api for the generic keystore API, and @ref
 * default_keystore for the default keystore implementation.
 *
 */

/*!
 * @defgroup keystore_api User Keystore API
 *
 * Keystore API
 *
 * These functions define the generic keystore API, which can be used in
 * conjunction with a keystore implementation backend to support a user
 * keystore.
 */

/*!
 * @defgroup default_keystore Default Keystore Implementation
 *
 * Default Keystore Implementation
 *
 * These functions define the default keystore implementation, which is used
 * for the system keystore and for user keystores initialized by
 * #fsl_shw_init_keystore_default().  They can be used as-is or as a reference
 * for creating a custom keystore handler.  It uses an entire Secure Memory
 * partition, divided in to equal slots of length #KEYSTORE_SLOT_SIZE.  These
 * functions are not intended to be used directly-  all user interaction with
 * the keystore should be through the @ref keystore_api and the Wrapped Key
 * interface.
 *
 * The current implementation is designed to work with both SCC and SCC2.
 * Differences between the two versions are noted below.
 */

/*! @addtogroup keystore_api
    @{ */

#ifndef KEYSTORE_SLOT_SIZE
/*! Size of each key slot, in octets.  This sets an upper bound on the size
 * of a key that can placed in the keystore.
 */
#define KEYSTORE_SLOT_SIZE 32
#endif

/*!
 * Initialize a Keystore Object.
 *
 * This function must be called before performing any other operation with the
 * Object. It allows the user to associate a custom keystore interface by
 * specifying the correct set of functions that will be used to perform actions
 * on the keystore object.  To use the default keystore handler, the function
 * #fsl_shw_init_keystore_default() can be used instead.
 *
 * @param keystore      The Keystore object to operate on.
 * @param data_init     Keystore initialization function.  This function is
 *                      responsible for initializing the keystore.  A
 *                      user-defined object can be assigned to the user_data
 *                      pointer, and will be passed to any function acting on
 *                      that keystore.  It is called during
 *                      #fsl_shw_establish_keystore().
 * @param data_cleanup  Keystore cleanup function.  This function cleans up
 *                      any data structures associated with the keyboard.  It
 *                      is called by #fsl_shw_release_keystore().
 * @param slot_alloc    Slot allocation function.  This function allocates a
 *                      key slot, potentially based on size and owner id.  It
 *                      is called by #fsl_shw_establish_key().
 * @param slot_dealloc  Slot deallocation function.
 * @param slot_verify_access Function to verify that a given Owner ID
 *                           credential matches the given slot.
 * @param slot_get_address   For SCC2: Get the virtual address (kernel or
 *                           userspace) of the data stored in the slot.
 *                           For SCC: Get the physical address of the data
 *                           stored in the slot.
 * @param slot_get_base      For SCC2: Get the (virtual) base address of the
 *                           partition that the slot is located on.
 *                           For SCC: Not implemented.
 * @param slot_get_offset    For SCC2: Get the offset from the start of the
 *                           partition that the slot data is located at (in
 *                           octets)
 *                           For SCC: Not implemented.
 * @param slot_get_slot_size Get the size of the key slot, in octets.
 */
extern void fsl_shw_init_keystore(fsl_shw_kso_t * keystore,
				  fsl_shw_return_t(*data_init) (fsl_shw_uco_t *
								user_ctx,
								void
								**user_data),
				  void (*data_cleanup) (fsl_shw_uco_t *
							user_ctx,
							void **user_data),
				  fsl_shw_return_t(*slot_alloc) (void
								 *user_data,
								 uint32_t size,
								 uint64_t
								 owner_id,
								 uint32_t *
								 slot),
				  fsl_shw_return_t(*slot_dealloc) (void
								   *user_data,
								   uint64_t
								   owner_id,
								   uint32_t
								   slot),
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
				  uint32_t(*slot_get_slot_size) (void
								 *user_data,
								 uint32_t
								 handle));

/*!
 * Initialize a Keystore Object.
 *
 * This function must be called before performing any other operation with the
 * Object. It sets the user keystore object up to use the default keystore
 * handler.  If a custom keystore handler is desired, the function
 * #fsl_shw_init_keystore() can be used instead.
 *
 * @param keystore      The Keystore object to operate on.
 */
extern void fsl_shw_init_keystore_default(fsl_shw_kso_t * keystore);

/*!
 * Establish a Keystore Object.
 *
 * This function establishes a keystore object that has been set up by a call
 * to #fsl_shw_init_keystore().  It is a wrapper for the user-defined
 * data_init() function, which is specified during keystore initialization.
 *
 * @param user_ctx      The user context that this keystore should be attached
 *                      to
 * @param keystore      The Keystore object to operate on.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_establish_keystore(fsl_shw_uco_t * user_ctx,
						   fsl_shw_kso_t * keystore);

/*!
 * Release a Keystore Object.
 *
 * This function releases an established keystore object.  It is a wrapper for
 * the user-defined data_cleanup() function, which is specified during keystore
 * initialization.
 *
 * @param user_ctx      The user context that this keystore should be attached
 *                      to.
 * @param keystore      The Keystore object to operate on.
 */
extern void fsl_shw_release_keystore(fsl_shw_uco_t * user_ctx,
				     fsl_shw_kso_t * keystore);

/*!
 * Allocate a slot in the Keystore.
 *
 * This function attempts to allocate a slot to hold a key in the keystore.  It
 * is called by #fsl_shw_establish_key() when establishing a Secure Key Object,
 * if the key has been flagged to be stored in a user keystore by the
 * #fsl_shw_sko_set_keystore() function.  It is a wrapper for the
 * implementation-specific function slot_alloc().
 *
 * @param keystore      The Keystore object to operate on.
 * @param[in] size      Size of the key to be stored (octets).
 * @param[in] owner_id  ID of the key owner.
 * @param[out] slot     If successful, assigned slot ID
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t keystore_slot_alloc(fsl_shw_kso_t * keystore,
					    uint32_t size,
					    uint64_t owner_id, uint32_t * slot);

/*!
 * Deallocate a slot in the Keystore.
 *
 * This function attempts to allocate a slot to hold a key in the keystore.
 * It is called by #fsl_shw_extract_key() and #fsl_shw_release_key() when the
 * key that it contains is to be released.  It is a wrapper for the
 * implmentation-specific function slot_dealloc().

 * @param keystore      The Keystore object to operate on.
 * @param[in] owner_id  ID of the key owner.
 * @param[in] slot     If successful, assigned slot ID.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t keystore_slot_dealloc(fsl_shw_kso_t * keystore,
					      uint64_t owner_id, uint32_t slot);

/*!
 * Load cleartext key data into a key slot
 *
 * This function loads a key slot with cleartext data.
 *
 * @param keystore      The Keystore object to operate on.
 * @param[in] owner_id  ID of the key owner.
 * @param[in] slot      If successful, assigned slot ID.
 * @param[in] key_data  Pointer to the location of the cleartext key data.
 * @param[in] key_length Length of the key data (octets).
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t
keystore_slot_load(fsl_shw_kso_t * keystore, uint64_t owner_id, uint32_t slot,
		   const uint8_t * key_data, uint32_t key_length);

/*!
 * Read cleartext key data from a key slot
 *
 * This function returns the key in a key slot.
 *
 * @param keystore      The Keystore object to operate on.
 * @param[in]  owner_id   ID of the key owner.
 * @param[in]  slot       ID of slot where key resides.
 * @param[in]  key_length Length of the key data (octets).
 * @param[out] key_data   Pointer to the location of the cleartext key data.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t
keystore_slot_read(fsl_shw_kso_t * keystore, uint64_t owner_id, uint32_t slot,
		   uint32_t key_length, uint8_t * key_data);

/*!
 * Encrypt a keyslot
 *
 * This function encrypts a key using the hardware secret key.
 *
 * @param user_ctx      User context
 * @param keystore      The Keystore object to operate on.
 * @param[in] owner_id  ID of the key owner.
 * @param[in] slot      Slot ID of the key to encrypt.
 * @param[in] length    Length of the key
 * @param[out] destination  Pointer to the location where the encrypted data
 *                          is to be stored.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t
keystore_slot_encrypt(fsl_shw_uco_t * user_ctx,
		      fsl_shw_kso_t * keystore, uint64_t owner_id,
		      uint32_t slot, uint32_t length, uint8_t * destination);

/*!
 * Decrypt a keyslot
 *
 * This function decrypts a key using the hardware secret key.
 *
 * @param user_ctx      User context
 * @param keystore      The Keystore object to operate on.
 * @param[in] owner_id  ID of the key owner.
 * @param[in] slot      Slot ID of the key to encrypt.
 * @param[in] length    Length of the key
 * @param[in] source    Pointer to the location where the encrypted data
 *                      is stored.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t
keystore_slot_decrypt(fsl_shw_uco_t * user_ctx,
		      fsl_shw_kso_t * keystore, uint64_t owner_id,
		      uint32_t slot, uint32_t length, const uint8_t * source);

/* @} */

/*! @addtogroup default_keystore
    @{ */

/*!
 * Data structure to hold per-slot information
 */
typedef struct keystore_data_slot_info_t {
	uint8_t allocated;	/*!< Track slot assignments */
	uint64_t owner;		/*!< Owner IDs */
	uint32_t key_length;	/*!< Size of the key */
} keystore_data_slot_info_t;

/*!
 * Data structure to hold keystore information.
 */
typedef struct keystore_data_t {
	void *base_address;	/*!< Base of the Secure Partition */
	uint32_t slot_count;	/*!< Number of slots in the keystore */
	struct keystore_data_slot_info_t *slot;	/*!< Per-slot information */
} keystore_data_t;

/*!
 * Default keystore initialization routine.
 *
 * This function acquires a Secure Partition Object to store the keystore,
 * divides it into slots of length #KEYSTORE_SLOT_SIZE, and builds a data
 * structure to hold key information.
 *
 * @param user_ctx       User context
 * @param[out] user_data Pointer to the location where the keystore data
 *                       structure is to be stored.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t shw_kso_init_data(fsl_shw_uco_t * user_ctx, void **user_data);

/*!
 * Default keystore cleanup routine.
 *
 * This function releases the Secure Partition Object and the memory holding
 * the keystore data structure, that obtained by the shw_kso_init_data
 * function.
 *
 * @param user_ctx          User context
 * @param[in,out] user_data Pointer to the location where the keystore data
 *                          structure is stored.
 */
void shw_kso_cleanup_data(fsl_shw_uco_t * user_ctx, void **user_data);

/*!
 * Default keystore slot access verification
 *
 * This function compares the supplied Owner ID to the registered owner of
 * the key slot, to see if the supplied ID is correct.
 *
 * @param[in] user_data Pointer to the location where the keystore data
 *                      structure stored.
 * @param[in] owner_id  Owner ID supplied as a credential.
 * @param[in] slot      Requested slot
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t shw_slot_verify_access(void *user_data, uint64_t owner_id,
					uint32_t slot);

/*!
 * Default keystore slot allocation
 *
 * This function first checks that the requested size is equal to or less than
 * the maximum keystore slot size.  If so, it searches the keystore for a free
 * key slot, and if found, marks it as used and returns a slot reference to the
 * user.
 *
 * @param[in] user_data Pointer to the location where the keystore data
 *                      structure stored.
 * @param[in] size      Size of the key data that will be stored in this slot
 *                      (octets)
 * @param[in] owner_id  Owner ID supplied as a credential.
 * @param[out] slot     Requested slot
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t shw_slot_alloc(void *user_data, uint32_t size,
				uint64_t owner_id, uint32_t * slot);

/*!
 * Default keystore slot deallocation
 *
 * This function releases the given key slot in the keystore, making it
 * available to store a new key.
 *
 * @param[in] user_data Pointer to the location where the keystore data
 *                      structure stored.
 * @param[in] owner_id  Owner ID supplied as a credential.
 * @param[in] slot      Requested slot
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t shw_slot_dealloc(void *user_data,
				  uint64_t owner_id, uint32_t slot);

/*!
 * Default keystore slot address lookup
 *
 * This function calculates the address where the key data is stored.
 *
 * @param[in] user_data Pointer to the location where the keystore data
 *                      structure stored.
 * @param[in] slot      Requested slot
 *
 * @return    SCC2: Virtual address (kernel or userspace) of the key data.
 *            SCC: Physical address of the key data.
 */
void *shw_slot_get_address(void *user_data, uint32_t slot);

/*!
 * Default keystore slot base address lookup
 *
 * This function calculates the base address of the Secure Partition on which
 * the key data is located.  For the reference design, only one Secure
 * Partition is used per Keystore, however in general, any number may be used.
 *
 * @param[in] user_data Pointer to the location where the keystore data
 *                      structure stored.
 * @param[in] slot      Requested slot
 *
 * @return    SCC2: Secure Partition virtual (kernel or userspace) base address.
 *            SCC: Secure Partition physical base address.
 */
uint32_t shw_slot_get_base(void *user_data, uint32_t slot);

/*!
 * Default keystore slot offset lookup
 *
 * This function calculates the offset from the base of the Secure Partition
 * where the key data is located.
 *
 * @param[in] user_data Pointer to the location where the keystore data
 *                      structure stored.
 * @param[in] slot      Requested slot
 *
 * @return    SCC2: Key data offset (octets)
 *            SCC: Not implemented
 */
uint32_t shw_slot_get_offset(void *user_data, uint32_t slot);

/*!
 * Default keystore slot offset lookup
 *
 * This function returns the size of the given key slot.  In the reference
 * implementation, all key slots are of the same size, however in general,
 * the keystore slot sizes can be made variable.
 *
 * @param[in] user_data Pointer to the location where the keystore data
 *                      structure stored.
 * @param[in] slot      Requested slot
 *
 * @return    SCC2: Keystore slot size.
 *            SCC: Not implemented
 */
uint32_t shw_slot_get_slot_size(void *user_data, uint32_t slot);

/* @} */

#endif /* FSL_SHW_KEYSTORE_H */
