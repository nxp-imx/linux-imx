/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file km_adaptor.c
*
* @brief The Adaptor component provides an interface to the
*        driver for a kernel user.
*/

#include <adaptor.h>
#include <sf_util.h>
#include <sah_queue_manager.h>
#include <sah_memory_mapper.h>
#include <fsl_shw_keystore.h>
#ifdef FSL_HAVE_SCC
#include <linux/mxc_scc_driver.h>
#elif defined (FSL_HAVE_SCC2)
#include <linux/mxc_scc2_driver.h>
#endif


EXPORT_SYMBOL(adaptor_Exec_Descriptor_Chain);
EXPORT_SYMBOL(sah_register);
EXPORT_SYMBOL(sah_deregister);
EXPORT_SYMBOL(sah_get_results);
EXPORT_SYMBOL(fsl_shw_smalloc);
EXPORT_SYMBOL(fsl_shw_sfree);
EXPORT_SYMBOL(fsl_shw_sstatus);
EXPORT_SYMBOL(fsl_shw_diminish_perms);
EXPORT_SYMBOL(do_scc_encrypt_region);
EXPORT_SYMBOL(do_scc_decrypt_region);
EXPORT_SYMBOL(do_system_keystore_slot_alloc);
EXPORT_SYMBOL(do_system_keystore_slot_dealloc);
EXPORT_SYMBOL(do_system_keystore_slot_load);
EXPORT_SYMBOL(do_system_keystore_slot_read);
EXPORT_SYMBOL(do_system_keystore_slot_encrypt);
EXPORT_SYMBOL(do_system_keystore_slot_decrypt);


#if defined(DIAG_DRV_IF) || defined(DIAG_MEM) || defined(DIAG_ADAPTOR)
#include <diagnostic.h>
#endif

#if defined(DIAG_DRV_IF) || defined(DIAG_MEM) || defined(DIAG_ADAPTOR)
#define MAX_DUMP 16

#define DIAG_MSG_SIZE   300
static char Diag_msg[DIAG_MSG_SIZE];
#endif

/* This is the wait queue to this mode of driver */
DECLARE_WAIT_QUEUE_HEAD(Wait_queue_km);

/*! This matches Sahara2 capabilities... */
fsl_shw_pco_t sahara2_capabilities = {
	1, 3,			/* api version number - major & minor */
	1, 6,			/* driver version number - major & minor */
	{
	 FSL_KEY_ALG_AES,
	 FSL_KEY_ALG_DES,
	 FSL_KEY_ALG_TDES,
	 FSL_KEY_ALG_ARC4},
	{
	 FSL_SYM_MODE_STREAM,
	 FSL_SYM_MODE_ECB,
	 FSL_SYM_MODE_CBC,
	 FSL_SYM_MODE_CTR},
	{
	 FSL_HASH_ALG_MD5,
	 FSL_HASH_ALG_SHA1,
	 FSL_HASH_ALG_SHA224,
	 FSL_HASH_ALG_SHA256},
	/*
	 * The following table must be set to handle all values of key algorithm
	 * and sym mode, and be in the correct order..
	 */
	{			/* Stream, ECB, CBC, CTR */
	 {0, 0, 0, 0},		/* HMAC */
	 {0, 1, 1, 1},		/* AES  */
	 {0, 1, 1, 0},		/* DES */
	 {0, 1, 1, 0},		/* 3DES */
	 {1, 0, 0, 0}		/* ARC4 */
	 },
	0, 0,
	0, 0, 0,
	{{0, 0}}
};

#ifdef DIAG_ADAPTOR
void km_Dump_Chain(const sah_Desc * chain);

void km_Dump_Region(const char *prefix, const unsigned char *data,
		    unsigned length);

static void km_Dump_Link(const char *prefix, const sah_Link * link);

void km_Dump_Words(const char *prefix, const unsigned *data, unsigned length);
#endif

/**** Memory routines ****/

static void *my_malloc(void *ref, size_t n)
{
	register void *mem;

#ifndef DIAG_MEM_ERRORS
	mem = os_alloc_memory(n, GFP_KERNEL);

#else
	{
		uint32_t rand;
		/* are we feeling lucky ? */
		os_get_random_bytes(&rand, sizeof(rand));
		if ((rand % DIAG_MEM_CONST) == 0) {
			mem = 0;
		} else {
			mem = os_alloc_memory(n, GFP_ATOMIC);
		}
	}
#endif				/* DIAG_MEM_ERRORS */

#ifdef DIAG_MEM
	sprintf(Diag_msg, "API kmalloc: %p for %d\n", mem, n);
	LOG_KDIAG(Diag_msg);
#endif
	ref = 0;		/* unused param warning */
	return mem;
}

static sah_Head_Desc *my_alloc_head_desc(void *ref)
{
	register sah_Head_Desc *ptr;

#ifndef DIAG_MEM_ERRORS
	ptr = sah_Alloc_Head_Descriptor();

#else
	{
		uint32_t rand;
		/* are we feeling lucky ? */
		os_get_random_bytes(&rand, sizeof(rand));
		if ((rand % DIAG_MEM_CONST) == 0) {
			ptr = 0;
		} else {
			ptr = sah_Alloc_Head_Descriptor();
		}
	}
#endif
	ref = 0;
	return ptr;
}

static sah_Desc *my_alloc_desc(void *ref)
{
	register sah_Desc *ptr;

#ifndef DIAG_MEM_ERRORS
	ptr = sah_Alloc_Descriptor();

#else
	{
		uint32_t rand;
		/* are we feeling lucky ? */
		os_get_random_bytes(&rand, sizeof(rand));
		if ((rand % DIAG_MEM_CONST) == 0) {
			ptr = 0;
		} else {
			ptr = sah_Alloc_Descriptor();
		}
	}
#endif
	ref = 0;
	return ptr;
}

static sah_Link *my_alloc_link(void *ref)
{
	register sah_Link *ptr;

#ifndef DIAG_MEM_ERRORS
	ptr = sah_Alloc_Link();

#else
	{
		uint32_t rand;
		/* are we feeling lucky ? */
		os_get_random_bytes(&rand, sizeof(rand));
		if ((rand % DIAG_MEM_CONST) == 0) {
			ptr = 0;
		} else {
			ptr = sah_Alloc_Link();
		}
	}
#endif
	ref = 0;
	return ptr;
}

static void my_free(void *ref, void *ptr)
{
	ref = 0;		/* unused param warning */
#ifdef DIAG_MEM
	sprintf(Diag_msg, "API kfree: %p\n", ptr);
	LOG_KDIAG(Diag_msg);
#endif
	os_free_memory(ptr);
}

static void my_free_head_desc(void *ref, sah_Head_Desc * ptr)
{
	sah_Free_Head_Descriptor(ptr);
}

static void my_free_desc(void *ref, sah_Desc * ptr)
{
	sah_Free_Descriptor(ptr);
}

static void my_free_link(void *ref, sah_Link * ptr)
{
	sah_Free_Link(ptr);
}

static void *my_memcpy(void *ref, void *dest, const void *src, size_t n)
{
	ref = 0;		/* unused param warning */
	return memcpy(dest, src, n);
}

static void *my_memset(void *ref, void *ptr, int ch, size_t n)
{
	ref = 0;		/* unused param warning */
	return memset(ptr, ch, n);
}

/*! Standard memory manipulation routines for kernel API. */
static sah_Mem_Util std_kernelmode_mem_util = {
	.mu_ref = 0,
	.mu_malloc = my_malloc,
	.mu_alloc_head_desc = my_alloc_head_desc,
	.mu_alloc_desc = my_alloc_desc,
	.mu_alloc_link = my_alloc_link,
	.mu_free = my_free,
	.mu_free_head_desc = my_free_head_desc,
	.mu_free_desc = my_free_desc,
	.mu_free_link = my_free_link,
	.mu_memcpy = my_memcpy,
	.mu_memset = my_memset
};

fsl_shw_return_t get_capabilities(fsl_shw_uco_t * user_ctx,
				  fsl_shw_pco_t * capabilities)
{
	scc_config_t *scc_capabilities;

	/* Fill in the Sahara2 capabilities. */
	memcpy(capabilities, &sahara2_capabilities, sizeof(fsl_shw_pco_t));

	/* Fill in the SCC portion of the capabilities object */
	scc_capabilities = scc_get_configuration();
	capabilities->scc_driver_major = scc_capabilities->driver_major_version;
	capabilities->scc_driver_minor = scc_capabilities->driver_minor_version;
	capabilities->scm_version = scc_capabilities->scm_version;
	capabilities->smn_version = scc_capabilities->smn_version;
	capabilities->block_size_bytes = scc_capabilities->block_size_bytes;

#ifdef FSL_HAVE_SCC
	capabilities->scc_info.black_ram_size_blocks =
	    scc_capabilities->black_ram_size_blocks;
	capabilities->scc_info.red_ram_size_blocks =
	    scc_capabilities->red_ram_size_blocks;
#elif defined(FSL_HAVE_SCC2)
	capabilities->scc2_info.partition_size_bytes =
	    scc_capabilities->partition_size_bytes;
	capabilities->scc2_info.partition_count =
	    scc_capabilities->partition_count;
#endif

	return FSL_RETURN_OK_S;
}

/*!
 * Sends a request to register this user
 *
 * @brief    Sends a request to register this user
 *
 * @param[in,out] user_ctx  part of the structure contains input parameters and
 *                          part is filled in by the driver
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_register(fsl_shw_uco_t * user_ctx)
{
	fsl_shw_return_t status;

	/* this field is used in user mode to indicate a file open has occured.
	 * it is used here, in kernel mode, to indicate that the uco is registered
	 */
	user_ctx->sahara_openfd = 0;	/* set to 'registered' */
	user_ctx->mem_util = &std_kernelmode_mem_util;

	/* check that uco is valid */
	status = sah_validate_uco(user_ctx);

	/*  If life is good, register this user */
	if (status == FSL_RETURN_OK_S) {
		status = sah_handle_registration(user_ctx);
	}

	if (status != FSL_RETURN_OK_S) {
		user_ctx->sahara_openfd = -1;	/* set to 'not registered' */
	}

	return status;
}

/*!
 * Sends a request to deregister this user
 *
 * @brief    Sends a request to deregister this user
 *
 * @param[in,out]  user_ctx   Info on user being deregistered.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_deregister(fsl_shw_uco_t * user_ctx)
{
	fsl_shw_return_t status = FSL_RETURN_OK_S;

	if (user_ctx->sahara_openfd == 0) {
		status = sah_handle_deregistration(user_ctx);
		user_ctx->sahara_openfd = -1;	/* set to 'no registered */
	}

	return status;
}

/*!
 * Sends a request to get results for this user
 *
 * @brief    Sends a request to get results for this user
 *
 * @param[in,out] arg      Pointer to structure to collect results
 * @param         uco      User's context
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_get_results(sah_results * arg, fsl_shw_uco_t * uco)
{
	fsl_shw_return_t code = sah_get_results_from_pool(uco, arg);

	if ((code == FSL_RETURN_OK_S) && (arg->actual != 0)) {
		sah_Postprocess_Results(uco, arg);
	}

	return code;
}

/*!
 * This function writes the Descriptor Chain to the kernel driver.
 *
 * @brief    Writes the Descriptor Chain to the kernel driver.
 *
 * @param    dar   A pointer to a Descriptor Chain of type sah_Head_Desc
 * @param    uco   The user context object
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t adaptor_Exec_Descriptor_Chain(sah_Head_Desc * dar,
					       fsl_shw_uco_t * uco)
{
	sah_Head_Desc *kernel_space_desc = NULL;
	fsl_shw_return_t code = FSL_RETURN_OK_S;
	int os_error_code = 0;
	unsigned blocking_mode = dar->uco_flags & FSL_UCO_BLOCKING_MODE;

#ifdef DIAG_ADAPTOR
	km_Dump_Chain(&dar->desc);
#endif

	dar->user_info = uco;
	dar->user_desc = dar;

	/* This code has been shamelessly copied from sah_driver_interface.c */
	/* It needs to be moved somewhere common ... */
	kernel_space_desc = sah_Physicalise_Descriptors(dar);

	if (kernel_space_desc == NULL) {
		/* We may have failed due to a -EFAULT as well, but we will return
		 * -ENOMEM since either way it is a memory related failure. */
		code = FSL_RETURN_NO_RESOURCE_S;
#ifdef DIAG_DRV_IF
		LOG_KDIAG("sah_Physicalise_Descriptors() failed\n");
#endif
	} else {
		if (blocking_mode) {
#ifdef SAHARA_POLL_MODE
			os_error_code = sah_Handle_Poll(dar);
#else
			os_error_code = sah_blocking_mode(dar);
#endif
			if (os_error_code != 0) {
				code = FSL_RETURN_ERROR_S;
			} else {	/* status of actual operation */
				code = dar->result;
			}
		} else {
#ifdef SAHARA_POLL_MODE
			sah_Handle_Poll(dar);
#else
			/* just put someting in the DAR */
			sah_Queue_Manager_Append_Entry(dar);
#endif				/* SAHARA_POLL_MODE */
		}
	}

	return code;
}


/* System keystore context, defined in sah_driver_interface.c */
extern fsl_shw_kso_t system_keystore;

fsl_shw_return_t do_system_keystore_slot_alloc(fsl_shw_uco_t * user_ctx,
					       uint32_t key_length,
					       uint64_t ownerid,
					       uint32_t * slot)
{
	(void)user_ctx;
	return keystore_slot_alloc(&system_keystore, key_length, ownerid, slot);
}

fsl_shw_return_t do_system_keystore_slot_dealloc(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot)
{
	(void)user_ctx;
	return keystore_slot_dealloc(&system_keystore, ownerid, slot);
}

fsl_shw_return_t do_system_keystore_slot_load(fsl_shw_uco_t * user_ctx,
					      uint64_t ownerid,
					      uint32_t slot,
					      const uint8_t * key,
					      uint32_t key_length)
{
	(void)user_ctx;
	return keystore_slot_load(&system_keystore, ownerid, slot,
				  (void *)key, key_length);
}

fsl_shw_return_t do_system_keystore_slot_read(fsl_shw_uco_t * user_ctx,
					      uint64_t ownerid,
					      uint32_t slot,
					      uint32_t key_length,
					      const uint8_t * key)
{
	(void)user_ctx;
	return keystore_slot_read(&system_keystore, ownerid, slot,
				  key_length, (void *)key);
}

fsl_shw_return_t do_system_keystore_slot_encrypt(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot,
						 uint32_t key_length,
						 uint8_t * black_data)
{
	(void)user_ctx;
	return keystore_slot_encrypt(NULL, &system_keystore, ownerid,
				     slot, key_length, black_data);
}

fsl_shw_return_t do_system_keystore_slot_decrypt(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot,
						 uint32_t key_length,
						 const uint8_t * black_data)
{
	(void)user_ctx;
	return keystore_slot_decrypt(NULL, &system_keystore, ownerid,
				     slot, key_length, black_data);
}

void *fsl_shw_smalloc(fsl_shw_uco_t * user_ctx,
		      uint32_t size, const uint8_t * UMID, uint32_t permissions)
{
#ifdef FSL_HAVE_SCC2
	int part_no;
	void *part_base;
	uint32_t part_phys;
	scc_config_t *scc_configuration;

	/* Check that the memory size requested is correct */
	scc_configuration = scc_get_configuration();
	if (size != scc_configuration->partition_size_bytes) {
		return NULL;
	}

	/* Attempt to grab a partition. */
	if (scc_allocate_partition(0, &part_no, &part_base, &part_phys)
	    != SCC_RET_OK) {
		return NULL;
	}
	printk(KERN_ALERT "In fsh_shw_smalloc (km): partition_base:%p "
	       "partition_base_phys: %p\n", part_base, (void *)part_phys);

	/* these bits should be in a separate function */
	printk(KERN_ALERT "writing UMID and MAP to secure the partition\n");

	scc_engage_partition(part_base, UMID, permissions);

	(void)user_ctx;		/* unused param warning */

	return part_base;
#else				/* FSL_HAVE_SCC2 */
	(void)user_ctx;
	(void)size;
	(void)UMID;
	(void)permissions;
	return NULL;
#endif				/* FSL_HAVE_SCC2 */

}

fsl_shw_return_t fsl_shw_sfree(fsl_shw_uco_t * user_ctx, void *address)
{
	(void)user_ctx;

#ifdef FSL_HAVE_SCC2
	if (scc_release_partition(address) == SCC_RET_OK) {
		return FSL_RETURN_OK_S;
	}
#endif

	return FSL_RETURN_ERROR_S;
}

fsl_shw_return_t fsl_shw_sstatus(fsl_shw_uco_t * user_ctx,
				 void *address,
				 fsl_shw_partition_status_t * status)
{
	(void)user_ctx;

#ifdef FSL_HAVE_SCC2
	*status = scc_partition_status(address);
	return FSL_RETURN_OK_S;
#endif

	return FSL_RETURN_ERROR_S;
}

/* Diminish permissions on some secure memory */
fsl_shw_return_t fsl_shw_diminish_perms(fsl_shw_uco_t * user_ctx,
					void *address, uint32_t permissions)
{

	(void)user_ctx;		/* unused parameter warning */

#ifdef FSL_HAVE_SCC2
	if (scc_diminish_permissions(address, permissions) == SCC_RET_OK) {
		return FSL_RETURN_OK_S;
	}
#endif
	return FSL_RETURN_ERROR_S;
}

/*
 * partition_base - physical address of the partition
 * offset - offset, in blocks, of the data from the start of the partition
 * length - length, in bytes, of the data to be encrypted (multiple of 4)
 * black_data - virtual address that the encrypted data should be stored at
 * Note that this virtual address must be translatable using the __virt_to_phys
 * macro; ie, it can't be a specially mapped address.  To do encryption with those
 * addresses, use the scc_encrypt_region function directly.  This is to make
 * this function compatible with the user mode declaration, which does not know
 * the physical addresses of the data it is using.
 */
fsl_shw_return_t
do_scc_encrypt_region(fsl_shw_uco_t * user_ctx,
		      void *partition_base, uint32_t offset_bytes,
		      uint32_t byte_count, uint8_t * black_data,
		      uint32_t * IV, fsl_shw_cypher_mode_t cypher_mode)
{
	scc_return_t scc_ret;
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;

#ifdef FSL_HAVE_SCC2

#ifdef DIAG_ADAPTOR
	uint32_t *owner_32 = (uint32_t *) & (owner_id);

	LOG_KDIAG_ARGS
	    ("partition base: %p, offset: %i, count: %i, black data: %p\n",
	     partition_base, offset_bytes, byte_count, (void *)black_data);
#endif
	(void)user_ctx;

	os_cache_flush_range(black_data, byte_count);

	scc_ret =
	    scc_encrypt_region((uint32_t) partition_base, offset_bytes,
			       byte_count, __virt_to_phys(black_data), IV,
			       cypher_mode);

	if (scc_ret == SCC_RET_OK) {
		retval = FSL_RETURN_OK_S;
	} else {
		retval = FSL_RETURN_ERROR_S;
	}

	/* The SCC2 DMA engine should have written to the black ram, so we need to
	 * invalidate that region of memory.  Note that the red ram is not an
	 * because it is mapped with the cache disabled.
	 */
	os_cache_inv_range(black_data, byte_count);

#else
	(void)scc_ret;
#endif				/* FSL_HAVE_SCC2 */

	return retval;
}

/*!
 * Call the proper function to decrypt a region of encrypted secure memory
 *
 * @brief 
 *
 * @param   user_ctx        User context of the partition owner (NULL in kernel)
 * @param   partition_base  Base address (physical) of the partition
 * @param   offset_bytes    Offset from base address that the decrypted data
 *                          shall be placed
 * @param   byte_count      Length of the message (bytes)
 * @param   black_data      Pointer to where the encrypted data is stored
 * @param   owner_id    
 *
 * @return  status
 */

fsl_shw_return_t
do_scc_decrypt_region(fsl_shw_uco_t * user_ctx,
		      void *partition_base, uint32_t offset_bytes,
		      uint32_t byte_count, const uint8_t * black_data,
		      uint32_t * IV, fsl_shw_cypher_mode_t cypher_mode)
{
	scc_return_t scc_ret;
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;

#ifdef FSL_HAVE_SCC2

#ifdef DIAG_ADAPTOR
	uint32_t *owner_32 = (uint32_t *) & (owner_id);

	LOG_KDIAG_ARGS
	    ("partition base: %p, offset: %i, count: %i, black data: %p\n",
	     partition_base, offset_bytes, byte_count, (void *)black_data);
#endif

	(void)user_ctx;

	/* The SCC2 DMA engine will be reading from the black ram, so we need to
	 * make sure that the data is pushed out of the cache.  Note that the red
	 * ram is not an issue because it is mapped with the cache disabled.
	 */
	os_cache_flush_range(black_data, byte_count);

	scc_ret =
	    scc_decrypt_region((uint32_t) partition_base, offset_bytes,
			       byte_count,
			       (uint8_t *) __virt_to_phys(black_data), IV,
			       cypher_mode);

	if (scc_ret == SCC_RET_OK) {
		retval = FSL_RETURN_OK_S;
	} else {
		retval = FSL_RETURN_ERROR_S;
	}

#else
	(void)scc_ret;
#endif				/* FSL_HAVE_SCC2 */

	return retval;
}

#ifdef DIAG_ADAPTOR
/*!
 * Dump chain of descriptors to the log.
 *
 * @brief Dump descriptor chain
 *
 * @param    chain     Kernel virtual address of start of chain of descriptors
 *
 * @return   void
 */
void km_Dump_Chain(const sah_Desc * chain)
{
	while (chain != NULL) {
		km_Dump_Words("Desc", (unsigned *)chain,
			      6 /*sizeof(*chain)/sizeof(unsigned) */ );
		/* place this definition elsewhere */
		if (chain->ptr1) {
			if (chain->header & SAH_HDR_LLO) {
				km_Dump_Region(" Data1", chain->ptr1,
					       chain->len1);
			} else {
				km_Dump_Link(" Link1", chain->ptr1);
			}
		}
		if (chain->ptr2) {
			if (chain->header & SAH_HDR_LLO) {
				km_Dump_Region(" Data2", chain->ptr2,
					       chain->len2);
			} else {
				km_Dump_Link(" Link2", chain->ptr2);
			}
		}

		chain = chain->next;
	}
}

/*!
 * Dump chain of links to the log.
 *
 * @brief Dump chain of links
 *
 * @param    prefix    Text to put in front of dumped data
 * @param    link      Kernel virtual address of start of chain of links
 *
 * @return   void
 */
static void km_Dump_Link(const char *prefix, const sah_Link * link)
{
	while (link != NULL) {
		km_Dump_Words(prefix, (unsigned *)link,
			      3 /* # words in h/w link */ );
		if (link->flags & SAH_STORED_KEY_INFO) {
#ifdef CAN_DUMP_SCC_DATA
			uint32_t len;
#endif

#ifdef CAN_DUMP_SCC_DATA
			{
				char buf[50];

				scc_get_slot_info(link->ownerid, link->slot, (uint32_t *) & link->data,	/* RED key address */
						  &len);	/* key length */
				sprintf(buf, "  SCC slot %d: ", link->slot);
				km_Dump_Words(buf,
					      (void *)IO_ADDRESS((uint32_t)
								 link->data),
					      link->len / 4);
			}
#else
			sprintf(Diag_msg, "  SCC slot %d", link->slot);
			LOG_KDIAG(Diag_msg);
#endif
		} else if (link->data != NULL) {
			km_Dump_Region("  Data", link->data, link->len);
		}

		link = link->next;
	}
}

/*!
 * Dump given region of data to the log.
 *
 * @brief Dump data
 *
 * @param    prefix    Text to put in front of dumped data
 * @param    data      Kernel virtual address of start of region to dump
 * @param    length    Amount of data to dump
 *
 * @return   void
*/
void km_Dump_Region(const char *prefix, const unsigned char *data,
		    unsigned length)
{
	unsigned count;
	char *output;
	unsigned data_len;

	sprintf(Diag_msg, "%s (%08X,%u):", prefix, (uint32_t) data, length);

	/* Restrict amount of data to dump */
	if (length > MAX_DUMP) {
		data_len = MAX_DUMP;
	} else {
		data_len = length;
	}

	/* We've already printed some text in output buffer, skip over it */
	output = Diag_msg + strlen(Diag_msg);

	for (count = 0; count < data_len; count++) {
		if (count % 4 == 0) {
			*output++ = ' ';
		}
		sprintf(output, "%02X", *data++);
		output += 2;
	}

	LOG_KDIAG(Diag_msg);
}

/*!
 * Dump given wors of data to the log.
 *
 * @brief Dump data
 *
 * @param    prefix       Text to put in front of dumped data
 * @param    data         Kernel virtual address of start of region to dump
 * @param    word_count   Amount of data to dump
 *
 * @return   void
*/
void km_Dump_Words(const char *prefix, const unsigned *data,
		   unsigned word_count)
{
	char *output;

	sprintf(Diag_msg, "%s (%08X,%uw): ", prefix, (uint32_t) data,
		word_count);

	/* We've already printed some text in output buffer, skip over it */
	output = Diag_msg + strlen(Diag_msg);

	while (word_count--) {
		sprintf(output, "%08X ", *data++);
		output += 9;
	}

	LOG_KDIAG(Diag_msg);
}
#endif
