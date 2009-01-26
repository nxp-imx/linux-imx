/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sf_util.c
*
* @brief Security Functions component API - Utility functions
*
* These are the 'Sahara api' functions which are used by the higher-level
* FSL SHW API to build and then execute descriptor chains.
*/


#include "sf_util.h"
#include <adaptor.h>

#ifdef DIAG_SECURITY_FUNC
#include <diagnostic.h>
#endif /*DIAG_SECURITY_FUNC*/


#ifdef __KERNEL__
EXPORT_SYMBOL(sah_Append_Desc);
EXPORT_SYMBOL(sah_Append_Link);
EXPORT_SYMBOL(sah_Create_Link);
EXPORT_SYMBOL(sah_Create_Key_Link);
EXPORT_SYMBOL(sah_Destroy_Link);
EXPORT_SYMBOL(sah_Descriptor_Chain_Execute);
EXPORT_SYMBOL(sah_insert_mdha_algorithm);
EXPORT_SYMBOL(sah_insert_skha_algorithm);
EXPORT_SYMBOL(sah_insert_skha_mode);
EXPORT_SYMBOL(sah_insert_skha_modulus);
EXPORT_SYMBOL(sah_Descriptor_Chain_Destroy);
EXPORT_SYMBOL(sah_add_two_in_desc);
EXPORT_SYMBOL(sah_add_in_key_desc);
EXPORT_SYMBOL(sah_add_two_out_desc);
EXPORT_SYMBOL(sah_add_in_out_desc);
EXPORT_SYMBOL(sah_add_key_out_desc);
#endif

#ifdef DEBUG_REWORK
#ifndef __KERNEL__
#include <stdio.h>
#define os_printk printf
#endif
#endif

/**
 * Convert fsl_shw_hash_alg_t to mdha mode bits.
 *
 * Index must be maintained in order of fsl_shw_hash_alg_t enumeration!!!
 */
const uint32_t sah_insert_mdha_algorithm[] =
{
    [FSL_HASH_ALG_MD5] = sah_insert_mdha_algorithm_md5,
    [FSL_HASH_ALG_SHA1] = sah_insert_mdha_algorithm_sha1,
    [FSL_HASH_ALG_SHA224] = sah_insert_mdha_algorithm_sha224,
    [FSL_HASH_ALG_SHA256] = sah_insert_mdha_algorithm_sha256,
};

/**
 * Header bits for Algorithm field of SKHA header
 *
 * Index value must be kept in sync with fsl_shw_key_alg_t
 */
const uint32_t sah_insert_skha_algorithm[] =
{
    [FSL_KEY_ALG_HMAC] = 0x00000040,
    [FSL_KEY_ALG_AES] = sah_insert_skha_algorithm_aes,
    [FSL_KEY_ALG_DES] = sah_insert_skha_algorithm_des,
    [FSL_KEY_ALG_TDES] = sah_insert_skha_algorithm_tdes,
    [FSL_KEY_ALG_ARC4] = sah_insert_skha_algorithm_arc4,
};


/**
 * Header bits for MODE field of SKHA header
 *
 * Index value must be kept in sync with fsl_shw_sym_mod_t
 */
const uint32_t sah_insert_skha_mode[] =
{
    [FSL_SYM_MODE_STREAM] = sah_insert_skha_mode_ecb,
    [FSL_SYM_MODE_ECB] = sah_insert_skha_mode_ecb,
    [FSL_SYM_MODE_CBC] = sah_insert_skha_mode_cbc,
    [FSL_SYM_MODE_CTR] = sah_insert_skha_mode_ctr,
};


/**
 * Header bits to set CTR modulus size.  These have parity
 * included to allow XOR insertion of values.
 *
 * @note Must be kept in sync with fsl_shw_ctr_mod_t
 */
const uint32_t sah_insert_skha_modulus[] =
{
    [FSL_CTR_MOD_8] = 0x00000000, /**< 2**8 */
    [FSL_CTR_MOD_16] = 0x80000200, /**< 2**16 */
    [FSL_CTR_MOD_24] = 0x80000400, /**< 2**24 */
    [FSL_CTR_MOD_32] = 0x00000600, /**< 2**32 */
    [FSL_CTR_MOD_40] = 0x80000800, /**< 2**40 */
    [FSL_CTR_MOD_48] = 0x00000a00, /**< 2**48 */
    [FSL_CTR_MOD_56] = 0x00000c00, /**< 2**56 */
    [FSL_CTR_MOD_64] = 0x80000e00, /**< 2**64 */
    [FSL_CTR_MOD_72] = 0x80001000, /**< 2**72 */
    [FSL_CTR_MOD_80] = 0x00001200, /**< 2**80 */
    [FSL_CTR_MOD_88] = 0x00001400, /**< 2**88 */
    [FSL_CTR_MOD_96] = 0x80001600, /**< 2**96 */
    [FSL_CTR_MOD_104] = 0x00001800, /**< 2**104 */
    [FSL_CTR_MOD_112] = 0x80001a00, /**< 2**112 */
    [FSL_CTR_MOD_120] = 0x80001c00, /**< 2**120 */
    [FSL_CTR_MOD_128] = 0x00001e00 /**< 2**128 */
};


/******************************************************************************
* Internal function declarations
******************************************************************************/
static fsl_shw_return_t sah_Create_Desc(
    const sah_Mem_Util *mu,
    sah_Desc ** desc,
    int head,
    uint32_t header,
    sah_Link * link1,
    sah_Link * link2);


/**
 * Create a descriptor chain using the the header and links passed in as
 * parameters. The newly created descriptor will be added to the end of
 * the descriptor chain passed.
 *
 * If @a desc_head points to a NULL value, then a sah_Head_Desc will be created
 * as the first descriptor.  Otherwise a sah_Desc will be created and appended.
 *
 * @pre
 *
 * - None
 *
 * @post
 *
 * - A descriptor has been created from the header, link1 and link2.
 *
 * - The newly created descriptor has been appended to the end of
 *   desc_head, or its location stored into the location pointed to by
 *   @a desc_head.
 *
 * - On allocation failure, @a link1 and @a link2 will be destroyed., and
 *   @a desc_head will be untouched.
 *
 * @brief   Create and append descriptor chain, inserting header and links
 *          pointing to link1 and link2
 *
 * @param  mu Memory functions
 * @param  header Value of descriptor header to be added
 * @param  desc_head Pointer to head of descriptor chain to append new desc
 * @param  link1 Pointer to sah_Link 1 (or NULL)
 * @param  link2 Pointer to sah_Link 2 (or NULL)
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_Append_Desc(
    const sah_Mem_Util *mu,
    sah_Head_Desc **desc_head,
    const uint32_t header,
    sah_Link *link1,
    sah_Link *link2)
{
    fsl_shw_return_t status;
    sah_Desc         *desc;
    sah_Desc         *desc_ptr;


    status = sah_Create_Desc(mu, (sah_Desc**)&desc, (*desc_head == NULL),
                             header, link1, link2);
    /* append newly created descriptor to end of current chain */
    if (status == FSL_RETURN_OK_S) {
        if (*desc_head == NULL) {
            (*desc_head) = (sah_Head_Desc*)desc;
            (*desc_head)->out1_ptr = NULL;
            (*desc_head)->out2_ptr = NULL;

        } else {
            desc_ptr = (sah_Desc*)*desc_head;
            while (desc_ptr->next != NULL) {
                desc_ptr = desc_ptr->next;
            }
            desc_ptr->next = desc;
        }
    }

    return status;
}


/**
 * Releases the memory allocated by the Security Function library for
 * descriptors, links and any internally allocated memory referenced in the
 * given chain. Note that memory allocated by user applications is not
 * released.
 *
 * @post The @a desc_head pointer will be set to NULL to prevent further use.
 *
 * @brief   Destroy a descriptor chain and free memory of associated links
 *
 * @param  mu Memory functions
 * @param  desc_head Pointer to head of descriptor chain to be freed
 *
 * @return none
 */
void sah_Descriptor_Chain_Destroy (
    const sah_Mem_Util *mu,
    sah_Head_Desc **desc_head)
{
    sah_Desc *desc_ptr = &(*desc_head)->desc;
    sah_Head_Desc *desc_head_ptr = (sah_Head_Desc *)desc_ptr;

    while (desc_ptr != NULL) {
        register sah_Desc *next_desc_ptr;

        if (desc_ptr->ptr1 != NULL) {
            sah_Destroy_Link(mu, desc_ptr->ptr1);
        }
        if (desc_ptr->ptr2 != NULL) {
            sah_Destroy_Link(mu, desc_ptr->ptr2);
        }

        next_desc_ptr = desc_ptr->next;

        /* Be sure to free head descriptor as such */
        if (desc_ptr == (sah_Desc*)desc_head_ptr) {
            mu->mu_free_head_desc(mu->mu_ref, desc_head_ptr);
        } else {
            mu->mu_free_desc(mu->mu_ref, desc_ptr);
        }

        desc_ptr = next_desc_ptr;
    }

    *desc_head = NULL;
}


#ifndef NO_INPUT_WORKAROUND
/**
 * Reworks the link chain
 *
 * @brief   Reworks the link chain
 *
 * @param  mu    Memory functions
 * @param  link  Pointer to head of link chain to be reworked
 *
 * @return none
 */
static fsl_shw_return_t sah_rework_link_chain(
        const sah_Mem_Util *mu,
        sah_Link* link)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;
    int found_potential_problem = 0;
    uint32_t total_data = 0;
#ifdef DEBUG_REWORK
    sah_Link* first_link = link;
#endif

    if ((link->flags & SAH_OUTPUT_LINK)) {
        return status;
    }

    while (link != NULL) {
        total_data += link->len;

    /* Only non-key Input Links are affected by the DMA flush-to-FIFO
     * problem */

        /* If have seen problem and at end of chain... */
        if (found_potential_problem && (link->next == NULL) &&
                                         (total_data > 16)) {
            /* insert new 1-byte link */
            sah_Link* new_tail_link = mu->mu_alloc_link(mu->mu_ref);
            if (new_tail_link == NULL) {
                status = FSL_RETURN_NO_RESOURCE_S;
            } else {
#ifdef DEBUG_REWORK
                sah_Link* dump_link = first_link;
                while (dump_link != NULL) {
                    uint32_t i;
                    unsigned bytes_to_dump = (dump_link->len > 32) ?
                                              32 : dump_link->len;
                    os_printk("(rework)Link %p: %p/%u/%p\n", dump_link,
                              dump_link->data, dump_link->len,
                              dump_link->next);
                    if (!(dump_link->flags & SAH_STORED_KEY_INFO)) {
                        os_printk("(rework)Data %p: ", dump_link->data);
                        for (i = 0; i < bytes_to_dump; i++) {
                            os_printk("%02X ", dump_link->data[i]);
                        }
                        os_printk("\n");
                    }
                    else {
                        os_printk("rework)Data %p: Red key data\n", dump_link);
                    }
                    dump_link = dump_link->next;
                }
#endif
                link->len--;
                link->next = new_tail_link;
                new_tail_link->len = 1;
                new_tail_link->data = link->data+link->len;
                new_tail_link->flags = link->flags & ~(SAH_OWNS_LINK_DATA);
                new_tail_link->next = NULL;
                link = new_tail_link;
#ifdef DEBUG_REWORK
                os_printk("(rework)New link chain:\n");
                dump_link = first_link;
                while (dump_link != NULL) {
                    uint32_t i;
                    unsigned bytes_to_dump = (dump_link->len > 32) ?
                                              32 : dump_link->len;

                    os_printk("Link %p: %p/%u/%p\n", dump_link,
                              dump_link->data, dump_link->len,
                              dump_link->next);
                    if (!(dump_link->flags & SAH_STORED_KEY_INFO)) {
                        os_printk("Data %p: ", dump_link->data);
                        for (i = 0; i < bytes_to_dump; i++) {
                            os_printk("%02X ", dump_link->data[i]);
                        }
                        os_printk("\n");
                    }
                    else {
                        os_printk("Data %p: Red key data\n", dump_link);
                    }
                    dump_link = dump_link->next;
                }
#endif
            }
        } else if ((link->len % 4) || ((uint32_t)link->data % 4)) {
            found_potential_problem = 1;
        }

        link = link->next;
    }

    return status;
}


/**
 * Rework links to avoid H/W bug
 *
 * @param head   Beginning of descriptor chain
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
static fsl_shw_return_t sah_rework_links(
        const sah_Mem_Util *mu,
        sah_Head_Desc *head)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;
    sah_Desc* desc = &head->desc;

    while ((status == FSL_RETURN_OK_S) && (desc != NULL)) {
        if (desc->header & SAH_HDR_LLO) {
            status = FSL_RETURN_ERROR_S;
            break;
        }
        if (desc->ptr1 != NULL) {
            status = sah_rework_link_chain(mu, desc->ptr1);
        }
        if ((status == FSL_RETURN_OK_S) && (desc->ptr2 != NULL)) {
            status = sah_rework_link_chain(mu, desc->ptr2);
        }
        desc = desc->next;
    }

    return status;
}
#endif /* NO_INPUT_WORKAROUND */


/**
 * Send a descriptor chain to the SAHARA driver for processing.
 *
 * Note that SAHARA will read the input data from and write the output data
 * directly to the locations indicated during construction of the chain.
 *
 * @pre
 *
 * - None
 *
 * @post
 *
 * - @a head will have been executed on SAHARA
 * - @a head Will be freed unless a SAVE flag is set
 *
 * @brief   Execute a descriptor chain
 *
 * @param  head      Pointer to head of descriptor chain to be executed
 * @param  user_ctx  The user context on which to execute the descriptor chain.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_Descriptor_Chain_Execute(
    sah_Head_Desc *head,
    fsl_shw_uco_t *user_ctx)
{
    fsl_shw_return_t status;

	/* Check for null pointer or non-multiple-of-four value */
    if ((head == NULL) || ((uint32_t)head & 0x3)) {
        status = FSL_RETURN_ERROR_S;
        goto out;
    }

#ifndef NO_INPUT_WORKAROUND
    status = sah_rework_links(user_ctx->mem_util, head);
    if (status != FSL_RETURN_OK_S) {
        goto out;
    }
#endif

    /* complete the information in the descriptor chain head node */
    head->user_ref = user_ctx->user_ref;
    head->uco_flags = user_ctx->flags;
    head->next = NULL; /* driver will use this to link chain heads */

    status = adaptor_Exec_Descriptor_Chain(head, user_ctx);

#ifdef DIAG_SECURITY_FUNC
	 if (status == FSL_RETURN_OK_S)
		 LOG_DIAG("after exec desc chain: status is ok\n");
	 else
		 LOG_DIAG("after exec desc chain: status is not ok\n");
#endif

 out:
    return status;
}


/**
 * Create Link
 *
 * @brief   Allocate Memory for Link structure and populate using input
 *          parameters
 *
 * @post    On allocation failure, @a p will be freed if #SAH_OWNS_LINK_DATA is
 *          p set in @a flags.

 * @param  mu Memory functions
 * @param  link Pointer to link to be created
 * @param  p Pointer to data to use in link
 * @param  length Length of buffer 'p' in bytes
 * @param  flags Indicates whether memory has been allocated by the calling
 *         function or the security function
 *
 * @return FSL_RETURN_OK_S or FSL_RETURN_NO_RESOURCE_S
 */
fsl_shw_return_t sah_Create_Link(
    const sah_Mem_Util *mu,
    sah_Link **link,
    uint8_t *p,
    const size_t length,
    const sah_Link_Flags flags)
{

#ifdef DIAG_SECURITY_FUNC

    char diag[50];
#endif /*DIAG_SECURITY_FUNC_UGLY*/
    fsl_shw_return_t status = FSL_RETURN_NO_RESOURCE_S;


    *link = mu->mu_alloc_link(mu->mu_ref);

    /* populate link if memory allocation successful */
    if (*link != NULL) {
        (*link)->len = length;
        (*link)->data = p;
        (*link)->next = NULL;
        (*link)->flags = flags;
        status = FSL_RETURN_OK_S;

#ifdef DIAG_SECURITY_FUNC

        LOG_DIAG("Created Link");
        LOG_DIAG("------------");
        sprintf(diag," address       = 0x%x", (int) *link);
        LOG_DIAG(diag);
        sprintf(diag," link->len     = %d",(*link)->len);
        LOG_DIAG(diag);
        sprintf(diag," link->data    = 0x%x",(int) (*link)->data);
        LOG_DIAG(diag);
        sprintf(diag," link->flags   = 0x%x",(*link)->flags);
        LOG_DIAG(diag);
        LOG_DIAG(" link->next    = NULL");
#endif /*DIAG_SECURITY_FUNC_UGLY*/

    } else {
#ifdef DIAG_SECURITY_FUNC
        LOG_DIAG("Allocation of memory for sah_Link failed!\n");
#endif /*DIAG_SECURITY_FUNC*/

        /* Free memory previously allocated by the security function layer for
        link data. Note that the memory being pointed to will be zeroed before
        being freed, for security reasons. */
        if (flags & SAH_OWNS_LINK_DATA) {
            mu->mu_memset(mu->mu_ref, p, 0x00, length);
            mu->mu_free(mu->mu_ref, p);
        }
    }

    return status;
}


/**
 * Create Key Link
 *
 * @brief   Allocate Memory for Link structure and populate using key info
 *          object
 *
 * @param  mu Memory functions
 * @param  link Pointer to store address of link to be created
 * @param  key_info Pointer to Key Info object to be referenced
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_Create_Key_Link(
    const sah_Mem_Util *mu,
    sah_Link **link,
    fsl_shw_sko_t *key_info)
{
#ifdef DIAG_SECURITY_FUNC_UGLY
    char diag[50];
#endif /*DIAG_SECURITY_FUNC_UGLY*/
    fsl_shw_return_t status = FSL_RETURN_NO_RESOURCE_S;
    sah_Link_Flags flags = 0;


    *link = mu->mu_alloc_link(mu->mu_ref);

    /* populate link if memory allocation successful */
    if (*link != NULL) {
        (*link)->len = key_info->key_length;

        if (key_info->flags & FSL_SKO_KEY_PRESENT) {
            (*link)->data = key_info->key;
            status = FSL_RETURN_OK_S;
        } else {
            if (key_info->flags & FSL_SKO_KEY_ESTABLISHED) {
				
				if (key_info->keystore == NULL) {
					/* System Keystore */
                	(*link)->slot = key_info->handle;
                	(*link)->ownerid = key_info->userid;
                	(*link)->data = 0;
                	flags |= SAH_STORED_KEY_INFO;
                	status = FSL_RETURN_OK_S;
				} else {
#ifdef FSL_HAVE_SCC2
					/* User Keystore */
					fsl_shw_kso_t *keystore = key_info->keystore;
					/* Note: the key data is stored here, but the address has to
					 * be converted to a partition and offset in the kernel.
					 * This will be calculated in kernel space, based on the
					 * list of partitions held by the users context.
					 */
					(*link)->data =	
						keystore->slot_get_address(keystore->user_data,
												 key_info->handle);
				
					flags |= SAH_IN_USER_KEYSTORE;
					status = FSL_RETURN_OK_S;
#else
					/* User keystores only supported in SCC2 */
					status = FSL_RETURN_BAD_FLAG_S;
#endif				/* FSL_HAVE_SCC2 */
				
				}
            } else {
                /* the flag is bad. Should never get here */
                status = FSL_RETURN_BAD_FLAG_S;
            }
        }

        (*link)->next = NULL;
        (*link)->flags = flags;

#ifdef DIAG_SECURITY_FUNC_UGLY
        if (status == FSL_RETURN_OK_S) {
            LOG_DIAG("Created Link");
            LOG_DIAG("------------");
            sprintf(diag," address       = 0x%x", (int) *link);
            LOG_DIAG(diag);
            sprintf(diag," link->len     = %d", (*link)->len);
            LOG_DIAG(diag);
            if (key_info->flags & FSL_SKO_KEY_ESTABLISHED) {
                sprintf(diag," link->slot    = 0x%x", (*link)->slot);
                LOG_DIAG(diag);
            } else {
                sprintf(diag," link->data    = 0x%x", (int)(*link)->data);
                LOG_DIAG(diag);
            }
            sprintf(diag," link->flags   = 0x%x", (*link)->flags);
            LOG_DIAG(diag);
            LOG_DIAG(" link->next    = NULL");
        }
#endif /*DIAG_SECURITY_FUNC_UGLY*/

        if (status == FSL_RETURN_BAD_FLAG_S) {
            mu->mu_free_link(mu->mu_ref, *link);
            *link = NULL;
#ifdef DIAG_SECURITY_FUNC
            LOG_DIAG("Creation of sah_Key_Link failed due to bad key flag!\n");
#endif /*DIAG_SECURITY_FUNC*/
        }

    } else {
#ifdef DIAG_SECURITY_FUNC
        LOG_DIAG("Allocation of memory for sah_Key_Link failed!\n");
#endif /*DIAG_SECURITY_FUNC*/
    }

    return status;
}


/**
 * Append Link
 *
 * @brief   Allocate Memory for Link structure and append it to the end of
 *          the link chain.
 *
 * @post    On allocation failure, @a p will be freed if #SAH_OWNS_LINK_DATA is
 *          p set in @a flags.
 *
 * @param  mu Memory functions
 * @param  link_head Pointer to (head of) existing link chain
 * @param  p Pointer to data to use in link
 * @param  length Length of buffer 'p' in bytes
 * @param  flags Indicates whether memory has been allocated by the calling
 *         function or the security function
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_Append_Link(
    const sah_Mem_Util *mu,
    sah_Link *link_head,
    uint8_t *p,
    const size_t length,
    const sah_Link_Flags flags)
{
    sah_Link* new_link;
    fsl_shw_return_t status;


    status = sah_Create_Link(mu, &new_link, p, length, flags);

    if (status == FSL_RETURN_OK_S) {
        /* chase for the tail */
        while (link_head->next != NULL) {
            link_head = link_head->next;
        }

        /* and add new tail */
        link_head->next = new_link;
    }

    return status;
}


/**
 * Create and populate a single descriptor
 *
 * The pointer and length fields will be be set based on the chains passed in
 * as @a link1 and @a link2.
 *
 * @param  mu         Memory utility suite
 * @param  desc       Location to store pointer of new descriptor
 * @param  head_desc  Non-zero if this will be first in chain; zero otherwise
 * @param  header     The Sahara header value to store in the descriptor
 * @param  link1      A value (or NULL) for the first ptr
 * @param  link2      A value (or NULL) for the second ptr
 *
 * @post  If allocation succeeded, the @a link1 and @link2 will be linked into
 *        the descriptor.  If allocation failed, those link structues will be
 *        freed, and the @a desc will be unchanged.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
static fsl_shw_return_t sah_Create_Desc(
    const sah_Mem_Util *mu,
    sah_Desc ** desc,
    int head_desc,
    uint32_t header,
    sah_Link * link1,
    sah_Link * link2)
{
    fsl_shw_return_t status = FSL_RETURN_NO_RESOURCE_S;
#ifdef DIAG_SECURITY_FUNC_UGLY
    char diag[50];
#endif /*DIAG_SECURITY_FUNC_UGLY*/


    if (head_desc != 0) {
        *desc = (sah_Desc *)mu->mu_alloc_head_desc(mu->mu_ref);
    } else {
        *desc = mu->mu_alloc_desc(mu->mu_ref);
    }

    /* populate descriptor if memory allocation successful */
    if ((*desc) != NULL) {
        sah_Link* temp_link;

        status = FSL_RETURN_OK_S;
        (*desc)->header = header;

        temp_link = (*desc)->ptr1 = link1;
        (*desc)->len1 = 0;
        while (temp_link != NULL) {
            (*desc)->len1 += temp_link->len;
            temp_link = temp_link->next;
        }

        temp_link = (*desc)->ptr2 = link2;
        (*desc)->len2 = 0;
        while (temp_link != NULL) {
            (*desc)->len2 += temp_link->len;
            temp_link = temp_link->next;
        }

        (*desc)->next = NULL;

#ifdef DIAG_SECURITY_FUNC_UGLY
        LOG_DIAG("Created Desc");
        LOG_DIAG("------------");
        sprintf(diag," address       = 0x%x",(int) *desc);
        LOG_DIAG(diag);
        sprintf(diag," desc->header  = 0x%x",(*desc)->header);
        LOG_DIAG(diag);
        sprintf(diag," desc->len1    = %d",(*desc)->len1);
        LOG_DIAG(diag);
        sprintf(diag," desc->ptr1    = 0x%x",(int) ((*desc)->ptr1));
        LOG_DIAG(diag);
        sprintf(diag," desc->len2    = %d",(*desc)->len2);
        LOG_DIAG(diag);
        sprintf(diag," desc->ptr2    = 0x%x",(int) ((*desc)->ptr2));
        LOG_DIAG(diag);
        sprintf(diag," desc->next    = 0x%x",(int) ((*desc)->next));
        LOG_DIAG(diag);
#endif /*DIAG_SECURITY_FUNC_UGLY*/
    } else {
#ifdef DIAG_SECURITY_FUNC
        LOG_DIAG("Allocation of memory for sah_Desc failed!\n");
#endif /*DIAG_SECURITY_FUNC*/

        /* Destroy the links, otherwise the memory allocated by the Security
        Function layer for the links (and possibly the data within the links)
        will be lost */
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    }

    return status;
}


/**
 * Destroy a link (chain) and free memory
 *
 * @param mu   memory utility suite
 * @param link The Link to destroy
 *
 * @return none
 */
void sah_Destroy_Link(
    const sah_Mem_Util *mu,
    sah_Link * link)
{

    while (link != NULL) {
        sah_Link * next_link = link->next;

        if (link->flags & SAH_OWNS_LINK_DATA) {
            /* zero data for security purposes */
            mu->mu_memset(mu->mu_ref, link->data, 0x00, link->len);
            mu->mu_free(mu->mu_ref, link->data);
        }

        link->data = NULL;
        link->next = NULL;
        mu->mu_free_link(mu->mu_ref, link);

        link = next_link;
    }
}


/**
 * Add descriptor where both links are inputs.
 *
 * @param         header     The Sahara header value for the descriptor.
 * @param         in1        The first input buffer (or NULL)
 * @param         in1_length Size of @a in1
 * @param[out]    in2        The second input buffer (or NULL)
 * @param         in2_length Size of @a in2
 * @param         mu         Memory functions
 * @param[in,out] desc_chain Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_two_in_desc(uint32_t header,
                                 const uint8_t* in1,
                                 uint32_t in1_length,
                                 const uint8_t* in2,
                                 uint32_t in2_length,
                                 const sah_Mem_Util* mu,
                                 sah_Head_Desc** desc_chain)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;
    sah_Link* link1 = NULL;
    sah_Link* link2 = NULL;

	printk("Entering sah_add_two_in_desc \n");

    if (in1 != NULL) {
        status = sah_Create_Link(mu, &link1,
                                 (sah_Oct_Str) in1, in1_length,
                                 SAH_USES_LINK_DATA);
    }

    if ( (in2 != NULL) && (status == FSL_RETURN_OK_S) ) {
        status = sah_Create_Link(mu, &link2,
                                 (sah_Oct_Str) in2, in2_length,
                                 SAH_USES_LINK_DATA);
    }

    if (status != FSL_RETURN_OK_S) {
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    } else {
        status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
    }

    return status;
}

/*!
 * Add descriptor where neither link needs sync
 *
 * @param         header     The Sahara header value for the descriptor.
 * @param         in1        The first input buffer (or NULL)
 * @param         in1_length Size of @a in1
 * @param[out]    in2        The second input buffer (or NULL)
 * @param         in2_length Size of @a in2
 * @param         mu         Memory functions
 * @param[in,out] desc_chain Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_two_d_desc(uint32_t header,
				    const uint8_t * in1,
				    uint32_t in1_length,
				    const uint8_t * in2,
				    uint32_t in2_length,
				    const sah_Mem_Util * mu,
				    sah_Head_Desc ** desc_chain)
{
	fsl_shw_return_t status = FSL_RETURN_OK_S;
	sah_Link *link1 = NULL;
	sah_Link *link2 = NULL;

	printk("Entering sah_add_two_d_desc \n");

	if (in1 != NULL) {
		status = sah_Create_Link(mu, &link1,
					 (sah_Oct_Str) in1, in1_length,
					 SAH_USES_LINK_DATA);
	}

	if ((in2 != NULL) && (status == FSL_RETURN_OK_S)) {
		status = sah_Create_Link(mu, &link2,
					 (sah_Oct_Str) in2, in2_length,
					 SAH_USES_LINK_DATA);
	}

	if (status != FSL_RETURN_OK_S) {
		if (link1 != NULL) {
			sah_Destroy_Link(mu, link1);
		}
		if (link2 != NULL) {
			sah_Destroy_Link(mu, link2);
		}
	} else {
		status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
	}

	return status;
}				/* sah_add_two_d_desc() */

/**
 * Add descriptor where both links are inputs, the second one being a key.
 *
 * @param         header     The Sahara header value for the descriptor.
 * @param         in1        The first input buffer (or NULL)
 * @param         in1_length Size of @a in1
 * @param[out]    in2        The second input buffer (or NULL)
 * @param         in2_length Size of @a in2
 * @param         mu         Memory functions
 * @param[in,out] desc_chain Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_in_key_desc(uint32_t header,
                                 const uint8_t* in1,
                                 uint32_t in1_length,
                                 fsl_shw_sko_t* key_info,
                                 const sah_Mem_Util* mu,
                                 sah_Head_Desc** desc_chain)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;
    sah_Link         *link1 = NULL;
    sah_Link         *link2 = NULL;

	if (in1 != NULL) {
        status = sah_Create_Link(mu, &link1,
                                 (sah_Oct_Str) in1, in1_length,
                                 SAH_USES_LINK_DATA);
    }

	if (status != FSL_RETURN_OK_S) {
		goto out;
	}

    status = sah_Create_Key_Link(mu, &link2, key_info);

	
	if (status != FSL_RETURN_OK_S) {
		goto out;
	}

	status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
	
out:
    if (status != FSL_RETURN_OK_S) {
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    }

    return status;
}


/**
 * Create two links using keys allocated in the scc
 *
 * @param         header     The Sahara header value for the descriptor.
 * @param         in1        The first input buffer (or NULL)
 * @param         in1_length Size of @a in1
 * @param[out]    in2        The second input buffer (or NULL)
 * @param         in2_length Size of @a in2
 * @param         mu         Memory functions
 * @param[in,out] desc_chain Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_key_key_desc(uint32_t header,
                                  fsl_shw_sko_t *key_info1,
                                  fsl_shw_sko_t *key_info2,
                                  const sah_Mem_Util *mu,
                                  sah_Head_Desc **desc_chain)
{
    fsl_shw_return_t status;
    sah_Link         *link1 = NULL;
    sah_Link         *link2 = NULL;


    status = sah_Create_Key_Link(mu, &link1, key_info1);

    if (status == FSL_RETURN_OK_S) {
        status = sah_Create_Key_Link(mu, &link2, key_info2);
    }

    if (status != FSL_RETURN_OK_S) {
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    } else {
        status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
    }

    return status;
}


/**
 * Add descriptor where first link is input, the second is a changing key
 *
 * @param         header     The Sahara header value for the descriptor.
 * @param         in1        The first input buffer (or NULL)
 * @param         in1_length Size of @a in1
 * @param[out]    in2        The key for output
 * @param         mu         Memory functions
 * @param[in,out] desc_chain Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_in_keyout_desc(uint32_t header,
                                 const uint8_t* in1,
                                 uint32_t in1_length,
                                 fsl_shw_sko_t* key_info,
                                 const sah_Mem_Util* mu,
                                 sah_Head_Desc** desc_chain)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;
    sah_Link         *link1 = NULL;
    sah_Link         *link2 = NULL;

	if (in1 != NULL) {
        status = sah_Create_Link(mu, &link1,
                                 (sah_Oct_Str) in1, in1_length,
                                 SAH_USES_LINK_DATA);
    }

    if (status != FSL_RETURN_OK_S) {
		goto out;
    }	

	status = sah_Create_Key_Link(mu, &link2, key_info);
    
	if (status != FSL_RETURN_OK_S) {
		goto out;
	}   

link2->flags |= SAH_OUTPUT_LINK;	/* mark key for output */
status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
	
out:

    if (status != FSL_RETURN_OK_S) {
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    } 

    return status;
}

/**
 * Add descriptor where both links are outputs.
 *
 * @param         header      The Sahara header value for the descriptor.
 * @param         out1        The first output buffer (or NULL)
 * @param         out1_length Size of @a out1
 * @param[out]    out2        The second output buffer (or NULL)
 * @param         out2_length Size of @a out2
 * @param         mu          Memory functions
 * @param[in,out] desc_chain  Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_two_out_desc(uint32_t header,
                                      uint8_t* out1,
                                      uint32_t out1_length,
                                      uint8_t* out2,
                                      uint32_t out2_length,
                                      const sah_Mem_Util* mu,
                                      sah_Head_Desc** desc_chain)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;
    sah_Link         *link1 = NULL;
    sah_Link         *link2 = NULL;


	printk("Entering sah_add_two_out_desc \n");

    if (out1 != NULL) {
        status = sah_Create_Link(mu, &link1,
                                 (sah_Oct_Str) out1, out1_length,
                                 SAH_OUTPUT_LINK |
                                 SAH_USES_LINK_DATA);
    }

    if ( (out2 != NULL) && (status == FSL_RETURN_OK_S) ) {
        status = sah_Create_Link(mu, &link2,
                                 (sah_Oct_Str) out2, out2_length,
                                 SAH_OUTPUT_LINK |
                                 SAH_USES_LINK_DATA);
    }

    if (status != FSL_RETURN_OK_S) {
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    } else {
        status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
    }

    return status;
}


/**
 * Add descriptor where first link is output, second is output
 *
 * @param         header      The Sahara header value for the descriptor.
 * @param         out1        The first output buffer (or NULL)
 * @param         out1_length Size of @a out1
 * @param[out]    in2         The input buffer (or NULL)
 * @param         in2_length  Size of @a in2
 * @param         mu          Memory functions
 * @param[in,out] desc_chain  Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_out_in_desc(uint32_t header,
                                     uint8_t* out1,
                                     uint32_t out1_length,
                                     const uint8_t* in2,
                                     uint32_t in2_length,
                                     const sah_Mem_Util* mu,
                                     sah_Head_Desc** desc_chain)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;
    sah_Link         *link1 = NULL;
    sah_Link         *link2 = NULL;


    if (out1 != NULL) {
        status = sah_Create_Link(mu, &link1,
                                 (sah_Oct_Str) out1, out1_length,
                                 SAH_OUTPUT_LINK |
                                 SAH_USES_LINK_DATA);
    }

    if ( (in2 != NULL) && (status == FSL_RETURN_OK_S) ) {
        status = sah_Create_Link(mu, &link2,
                                 (sah_Oct_Str) in2, in2_length,
                                 SAH_USES_LINK_DATA);
    }

    if (status != FSL_RETURN_OK_S) {
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    } else {
        status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
    }

    return status;
}


/**
 * Add descriptor where link1 is input buffer, link2 is output buffer.
 *
 * @param         header     The Sahara header value for the descriptor.
 * @param         in         The input buffer
 * @param         in_length  Size of the input buffer
 * @param[out]    out        The output buffer
 * @param         out_length Size of the output buffer
 * @param         mu         Memory functions
 * @param[in,out] desc_chain Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_in_out_desc(uint32_t header,
                                 const uint8_t* in, uint32_t in_length,
                                 uint8_t* out, uint32_t out_length,
                                 const sah_Mem_Util* mu,
                                 sah_Head_Desc** desc_chain)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;
    sah_Link         *link1 = NULL;
    sah_Link         *link2 = NULL;

	if (in != NULL) {
        status = sah_Create_Link(mu, &link1,
                                 (sah_Oct_Str) in, in_length,
                                 SAH_USES_LINK_DATA);
    }

    if ((status == FSL_RETURN_OK_S) && (out != NULL))  {
        status = sah_Create_Link(mu, &link2,
                                 (sah_Oct_Str) out, out_length,
                                 SAH_OUTPUT_LINK |
                                 SAH_USES_LINK_DATA);
    }

    if (status != FSL_RETURN_OK_S) {
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    } else {
        status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
    }

    return status;
}


/**
 * Add descriptor where link1 is a key, link2 is output buffer.
 *
 * @param         header     The Sahara header value for the descriptor.
 * @param         key_info   Key information
 * @param[out]    out        The output buffer
 * @param         out_length Size of the output buffer
 * @param         mu         Memory functions
 * @param[in,out] desc_chain Chain to start or append to
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_add_key_out_desc(uint32_t header,
								  const fsl_shw_sko_t *key_info,
                                  uint8_t* out, uint32_t out_length,
                                  const sah_Mem_Util *mu,
                                  sah_Head_Desc **desc_chain)
{
    fsl_shw_return_t status;
    sah_Link         *link1 = NULL;
    sah_Link         *link2 = NULL;

	status = sah_Create_Key_Link(mu, &link1, (fsl_shw_sko_t *) key_info);
	if (status != FSL_RETURN_OK_S) {
		goto out;
	}
    

    if (out != NULL)  {
        status = sah_Create_Link(mu, &link2,
                                 (sah_Oct_Str) out, out_length,
                                 SAH_OUTPUT_LINK |
                                 SAH_USES_LINK_DATA);
		if (status != FSL_RETURN_OK_S) {
			goto out;
	   }
    }
status = sah_Append_Desc(mu, desc_chain, header, link1, link2);
	
out:
    if (status != FSL_RETURN_OK_S) {
        if (link1 != NULL) {
            sah_Destroy_Link(mu, link1);
        }
        if (link2 != NULL) {
            sah_Destroy_Link(mu, link2);
        }
    }

    return status;
}


/**
 * Sanity checks the user context object fields to ensure that they make some
 * sense before passing the uco as a parameter
 *
 * @brief Verify the user context object
 *
 * @param  uco  user context object
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_validate_uco(fsl_shw_uco_t *uco)
{
    fsl_shw_return_t status = FSL_RETURN_OK_S;


    /* check if file is opened */
    if (uco->sahara_openfd < 0) {
        status = FSL_RETURN_NO_RESOURCE_S;
    } else {
        /* check flag combination: the only invalid setting of the
         * blocking and callback flags is blocking with callback. So check
         * for that
         */
        if ((uco->flags & (FSL_UCO_BLOCKING_MODE | FSL_UCO_CALLBACK_MODE)) ==
                         (FSL_UCO_BLOCKING_MODE | FSL_UCO_CALLBACK_MODE)) {
            status = FSL_RETURN_BAD_FLAG_S;
        } else {
            /* check that memory utilities have been attached */
            if (uco->mem_util == NULL) {
                status = FSL_RETURN_MEMORY_ERROR_S;
            } else {
                /* must have pool of at least 1, even for blocking mode */
                if (uco->pool_size == 0) {
                    status = FSL_RETURN_ERROR_S;
                } else {
                    /* if callback flag is set, it better have a callback
                     * routine */
                    if (uco->flags & FSL_UCO_CALLBACK_MODE) {
                        if (uco->callback == NULL) {
                            status = FSL_RETURN_INTERNAL_ERROR_S;
                        }
                    }
                }
            }
        }
    }

    return status;
}


/**
 * Perform any post-processing on non-blocking results.
 *
 * For instance, free descriptor chains, compare authentication codes, ...
 *
 * @param  user_ctx     User context object
 * @param  result_info  A set of results
 */
void sah_Postprocess_Results(fsl_shw_uco_t* user_ctx, sah_results* result_info)
{
    unsigned i;

    /* for each result returned */
    for (i = 0; i < *result_info->actual; i++) {
        sah_Head_Desc* desc = result_info->results[i].user_desc;
        uint8_t* out1 = desc->out1_ptr;
        uint8_t* out2 = desc->out2_ptr;
        uint32_t len = desc->out_len;

        /*
         * For now, tne only post-processing besides freeing the
         * chain is the need to check the auth code for fsl_shw_auth_decrypt().
         *
         * If other uses are required in the future, this code will probably
         * need a flag in the sah_Head_Desc (or a function pointer!) to
         * determine what needs to be done.
         */
        if ((out1 != NULL) && (out2 != NULL)) {
            unsigned j;
            for (j = 0; j < len; j++) {
                if (out1[j] != out2[j]) {
                    /* Problem detected.  Change result. */
                    result_info->results[i].code = FSL_RETURN_AUTH_FAILED_S;
                    break;
                }
            }
            /* free allocated 'calced_auth' */
            user_ctx->mem_util->
                mu_free(user_ctx->mem_util->mu_ref, out1);
        }

        /* Free the API-created chain, unless tagged as not-from-API */
        if (! (desc->uco_flags & FSL_UCO_SAVE_DESC_CHAIN)) {
            sah_Descriptor_Chain_Destroy(user_ctx->mem_util, &desc);
        }
    }
}


/* End of sf_util.c */
