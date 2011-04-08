/*
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file fsl_shw_wrap.c
 *
 * This file implements Key-Wrap (Black Key) functions of the FSL SHW API for
 * Sahara.
 *
 * - Ownerid is an 8-byte, user-supplied, value to keep KEY confidential.
 * - KEY is a 1-32 byte value which starts in SCC RED RAM before
 *     wrapping, and ends up there on unwrap.  Length is limited because of
 *    size of SCC1 RAM.
 * - KEY' is the encrypted KEY
 * - LEN is a 1-byte (for now) byte-length of KEY
 * - ALG is a 1-byte value for the algorithm which which the key is
 *    associated.  Values are defined by the FSL SHW API
 * - Ownerid, LEN, and ALG come from the user's "key_info" object, as does the
 *    slot number where KEY already is/will be.
 * - T is a Nonce
 * - T' is the encrypted T
 * - KEK is a Key-Encryption Key for the user's Key
 * - ICV is the "Integrity Check Value" for the wrapped key
 * - Black Key is the string of bytes returned as the wrapped key
<table>
<tr><TD align="right">BLACK_KEY <TD width="3">=<TD>ICV | T' | LEN | ALG |
     KEY'</td></tr>
<tr><td>&nbsp;</td></tr>

<tr><th>To Wrap</th></tr>
<tr><TD align="right">T</td> <TD width="3">=</td> <TD>RND()<sub>16</sub>
    </td></tr>
<tr><TD align="right">KEK</td><TD width="3">=</td><TD>HASH<sub>sha256</sub>(T |
     Ownerid)<sub>16</sub></td></tr>
<tr><TD align="right">KEY'<TD width="3">=</td><TD>
     AES<sub>ctr-enc</sub>(Key=KEK, CTR=0, Data=KEY)</td></tr>
<tr><TD align="right">ICV</td><TD width="3">=</td><td>HMAC<sub>sha256</sub>
     (Key=T, Data=Ownerid | LEN | ALG | KEY')<sub>16</sub></td></tr>
<tr><TD align="right">T'</td><TD width="3">=</td><TD>TDES<sub>cbc-enc</sub>
     (Key=SLID, IV=Ownerid, Data=T)</td></tr>

<tr><td>&nbsp;</td></tr>

<tr><th>To Unwrap</th></tr>
<tr><TD align="right">T</td><TD width="3">=</td><TD>TDES<sub>ecb-dec</sub>
    (Key=SLID, IV=Ownerid, Data=T')</sub></td></tr>
<tr><TD align="right">ICV</td><TD width="3">=</td><td>HMAC<sub>sha256</sub>
    (Key=T, Data=Ownerid | LEN | ALG | KEY')<sub>16</sub></td></tr>
<tr><TD align="right">KEK</td><TD width="3">=</td><td>HASH<sub>sha256</sub>
    (T | Ownerid)<sub>16</sub></td></tr>
<tr><TD align="right">KEY<TD width="3">=</td><TD>AES<sub>ctr-dec</sub>
    (Key=KEK, CTR=0, Data=KEY')</td></tr>
</table>

 */

#include <linux/mxc_sahara.h>
#include "fsl_platform.h"
#include "fsl_shw_keystore.h"

#include "sf_util.h"
#include "adaptor.h"

#if defined(DIAG_SECURITY_FUNC)
#include <diagnostic.h>
#endif

#if defined(NEED_CTR_WORKAROUND)
/* CTR mode needs block-multiple data in/out */
#define LENGTH_PATCH  16
#define LENGTH_PATCH_MASK  0xF
#else
#define LENGTH_PATCH  4
#define LENGTH_PATCH_MASK  3
#endif

#if LENGTH_PATCH
#define ROUND_LENGTH(len)                                                  \
({                                                                         \
    uint32_t orig_len = len;                                               \
    uint32_t new_len;                                                      \
                                                                           \
   if ((orig_len & LENGTH_PATCH_MASK) != 0) {                              \
       new_len = (orig_len + LENGTH_PATCH                                  \
            - (orig_len & LENGTH_PATCH_MASK));                             \
   }                                                                       \
   else {                                                                  \
       new_len = orig_len;                                                 \
   }                                                                       \
       new_len;                                                            \
})
#else
#define ROUND_LENGTH(len) (len)
#endif

#ifdef __KERNEL__
EXPORT_SYMBOL(fsl_shw_establish_key);
EXPORT_SYMBOL(fsl_shw_extract_key);
EXPORT_SYMBOL(fsl_shw_release_key);
EXPORT_SYMBOL(fsl_shw_read_key);
#endif

#define ICV_LENGTH 16
#define T_LENGTH 16
#define KEK_LENGTH 16
#define LENGTH_LENGTH 1
#define ALGORITHM_LENGTH 1
#define FLAGS_LENGTH 1

/* ICV | T' | LEN | ALG | KEY' */
#define ICV_OFFSET       0
#define T_PRIME_OFFSET   (ICV_OFFSET + ICV_LENGTH)
#define LENGTH_OFFSET    (T_PRIME_OFFSET + T_LENGTH)
#define ALGORITHM_OFFSET (LENGTH_OFFSET + LENGTH_LENGTH)
#define FLAGS_OFFSET     (ALGORITHM_OFFSET + ALGORITHM_LENGTH)
#define KEY_PRIME_OFFSET (FLAGS_OFFSET + FLAGS_LENGTH)
#define FLAGS_SW_KEY     0x01

/*
 * For testing of the algorithm implementation,, the DO_REPEATABLE_WRAP flag
 * causes the T_block to go into the T field during a wrap operation.  This
 * will make the black key value repeatable (for a given SCC secret key, or
 * always if the default key is in use).
 *
 * Normally, a random sequence is used.
 */
#ifdef DO_REPEATABLE_WRAP
/*!
 * Block of zeroes which is maximum Symmetric block size, used for
 * initializing context register, etc.
 */
static uint8_t T_block_[16] = {
	0x42, 0, 0, 0x42, 0x42, 0, 0, 0x42,
	0x42, 0, 0, 0x42, 0x42, 0, 0, 0x42
};
#endif

/*!
 * Insert descriptors to calculate ICV = HMAC(key=T, data=LEN|ALG|KEY')
 *
 * @param  user_ctx      User's context for this operation
 * @param  desc_chain    Descriptor chain to append to
 * @param  t_key_info    T's key object
 * @param  black_key     Beginning of Black Key region
 * @param  key_length    Number of bytes of key' there are in @c black_key
 * @param[out] hmac      Location to store ICV.  Will be tagged "USES" so
 *                       sf routines will not try to free it.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
static inline fsl_shw_return_t create_icv_calc(fsl_shw_uco_t * user_ctx,
					       sah_Head_Desc ** desc_chain,
					       fsl_shw_sko_t * t_key_info,
					       const uint8_t * black_key,
					       uint32_t key_length,
					       uint8_t * hmac)
{
	fsl_shw_return_t sah_code;
	uint32_t header;
	sah_Link *link1 = NULL;
	sah_Link *link2 = NULL;

	/* Load up T as key for the HMAC */
	header = (SAH_HDR_MDHA_SET_MODE_MD_KEY	/* #6 */
		  ^ sah_insert_mdha_algorithm_sha256
		  ^ sah_insert_mdha_init ^ sah_insert_mdha_hmac ^
		  sah_insert_mdha_pdata ^ sah_insert_mdha_mac_full);
	sah_code = sah_add_in_key_desc(header, NULL, 0, t_key_info,	/* Reference T in RED */
				       user_ctx->mem_util, desc_chain);
	if (sah_code != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Previous step loaded key; Now set up to hash the data */
	header = SAH_HDR_MDHA_HASH;	/* #10 */

	/* Input - start with ownerid */
	sah_code = sah_Create_Link(user_ctx->mem_util, &link1,
				   (void *)&t_key_info->userid,
				   sizeof(t_key_info->userid),
				   SAH_USES_LINK_DATA);
	if (sah_code != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Still input  - Append black-key fields len, alg, key' */
	sah_code = sah_Append_Link(user_ctx->mem_util, link1,
				   (void *)black_key + LENGTH_OFFSET,
				   (LENGTH_LENGTH
				    + ALGORITHM_LENGTH
				    + key_length), SAH_USES_LINK_DATA);

	if (sah_code != FSL_RETURN_OK_S) {
		goto out;
	}
	/* Output - computed ICV/HMAC */
	sah_code = sah_Create_Link(user_ctx->mem_util, &link2,
				   hmac, ICV_LENGTH,
				   SAH_USES_LINK_DATA | SAH_OUTPUT_LINK);
	if (sah_code != FSL_RETURN_OK_S) {
		goto out;
	}

	sah_code = sah_Append_Desc(user_ctx->mem_util, desc_chain,
				   header, link1, link2);

      out:
	if (sah_code != FSL_RETURN_OK_S) {
		(void)sah_Destroy_Link(user_ctx->mem_util, link1);
		(void)sah_Destroy_Link(user_ctx->mem_util, link2);
	}

	return sah_code;
}				/* create_icv_calc */

/*!
 * Perform unwrapping of a black key into a RED slot
 *
 * @param         user_ctx      A user context from #fsl_shw_register_user().
 * @param[in,out] key_info      The information about the key to be which will
 *                              be unwrapped... key length, slot info, etc.
 * @param         black_key     Encrypted key
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
static fsl_shw_return_t unwrap(fsl_shw_uco_t * user_ctx,
			       fsl_shw_sko_t * key_info,
			       const uint8_t * black_key)
{
	SAH_SF_DCLS;
	uint8_t *hmac = NULL;
	fsl_shw_sko_t t_key_info;
	sah_Link *link1 = NULL;
	sah_Link *link2 = NULL;
	unsigned i;
	unsigned rounded_key_length;
	unsigned original_key_length = key_info->key_length;

	hmac = DESC_TEMP_ALLOC(ICV_LENGTH);

	/* Set up key_info for "T" - use same slot as eventual key */
	fsl_shw_sko_init(&t_key_info, FSL_KEY_ALG_AES);
	t_key_info.userid = key_info->userid;
	t_key_info.handle = key_info->handle;
	t_key_info.flags = key_info->flags;
	t_key_info.key_length = T_LENGTH;
	t_key_info.keystore = key_info->keystore;

	/* Validate SW flags to prevent misuse */
	if ((key_info->flags & FSL_SKO_KEY_SW_KEY)
		&& !(black_key[FLAGS_OFFSET] & FLAGS_SW_KEY)) {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	/* Compute T = SLID_decrypt(T'); leave in RED slot */
	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		ret = do_system_keystore_slot_decrypt(user_ctx,
					key_info->userid,
				    t_key_info.handle,
				    T_LENGTH,
				    black_key + T_PRIME_OFFSET);

	} else {
			/* Key goes in user keystore */
			ret = keystore_slot_decrypt(user_ctx,
							key_info->keystore,
							key_info->userid,
							t_key_info.handle,
							T_LENGTH,
						    black_key + T_PRIME_OFFSET);
	}
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Compute ICV = HMAC(T, ownerid | len | alg | key' */
	ret = create_icv_calc(user_ctx, &desc_chain, &t_key_info,
			      black_key, original_key_length, hmac);
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Creation of sah_Key_Link failed due to bad key"
			 " flag!\n");
#endif				/*DIAG_SECURITY_FUNC */
		goto out;
	}
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG("Validating MAC of wrapped key");
#endif
	SAH_SF_EXECUTE();
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}
	SAH_SF_DESC_CLEAN();

	/* Check computed ICV against value in Black Key */
	for (i = 0; i < ICV_LENGTH; i++) {
		if (black_key[ICV_OFFSET + i] != hmac[i]) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG_ARGS("computed ICV fails at offset %i\n", i);

			{
				char buff[300];
				int a;
				for (a = 0; a < ICV_LENGTH; a++)
					sprintf(&(buff[a * 2]), "%02x",
						black_key[ICV_OFFSET + a]);
				buff[a * 2 + 1] = 0;
				LOG_DIAG_ARGS("black key: %s", buff);

				for (a = 0; a < ICV_LENGTH; a++)
					sprintf(&(buff[a * 2]), "%02x",
						hmac[a]);
				buff[a * 2 + 1] = 0;
				LOG_DIAG_ARGS("hmac:      %s", buff);
			}
#endif
			ret = FSL_RETURN_AUTH_FAILED_S;
			goto out;
		}
	}

	/* This is no longer needed. */
	DESC_TEMP_FREE(hmac);

	/* Compute KEK = SHA256(T | ownerid).  Rewrite slot with value */
	header = (SAH_HDR_MDHA_SET_MODE_HASH	/* #8 */
		  ^ sah_insert_mdha_init
		  ^ sah_insert_mdha_algorithm_sha256 ^ sah_insert_mdha_pdata);

	/* Input - Start with T */
	ret = sah_Create_Key_Link(user_ctx->mem_util, &link1, &t_key_info);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Still input - append ownerid */
	ret = sah_Append_Link(user_ctx->mem_util, link1,
			      (void *)&key_info->userid,
			      sizeof(key_info->userid), SAH_USES_LINK_DATA);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Output - KEK goes into RED slot */
	ret = sah_Create_Key_Link(user_ctx->mem_util, &link2, &t_key_info);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Put the Hash calculation into the chain. */
	ret = sah_Append_Desc(user_ctx->mem_util, &desc_chain,
			      header, link1, link2);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Compute KEY = AES-decrypt(KEK, KEY') */
	header = (SAH_HDR_SKHA_SET_MODE_IV_KEY	/* #1 */
		  ^ sah_insert_skha_mode_ctr
		  ^ sah_insert_skha_algorithm_aes
		  ^ sah_insert_skha_modulus_128);
	/* Load KEK in as the key to use */
	DESC_IN_KEY(header, 0, NULL, &t_key_info);

	rounded_key_length = ROUND_LENGTH(original_key_length);
	key_info->key_length = rounded_key_length;

	/* Now set up for computation.  Result in RED */
	header = SAH_HDR_SKHA_ENC_DEC;	/* #4 */
	DESC_IN_KEY(header, rounded_key_length, black_key + KEY_PRIME_OFFSET,
		    key_info);

	/* Perform the operation */
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG("Decrypting key with KEK");
#endif
	SAH_SF_EXECUTE();

      out:
	key_info->key_length = original_key_length;
	SAH_SF_DESC_CLEAN();

	DESC_TEMP_FREE(hmac);

	/* Erase tracks */
	t_key_info.userid = 0xdeadbeef;
	t_key_info.handle = 0xdeadbeef;

	return ret;
}				/* unwrap */

/*!
 * Perform wrapping of a black key from a RED slot
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param[in,out] key_info      The information about the key to be which will
 *                              be wrapped... key length, slot info, etc.
 * @param      black_key        Place to store encrypted key
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
static fsl_shw_return_t wrap(fsl_shw_uco_t * user_ctx,
			     fsl_shw_sko_t * key_info, uint8_t * black_key)
{
	SAH_SF_DCLS;
	unsigned slots_allocated = 0;	/* boolean */
	fsl_shw_sko_t T_key_info;	/* for holding T */
	fsl_shw_sko_t KEK_key_info;	/* for holding KEK */
	unsigned original_key_length = key_info->key_length;
	unsigned rounded_key_length;
	sah_Link *link1;
	sah_Link *link2;

	black_key[LENGTH_OFFSET] = key_info->key_length;
	black_key[ALGORITHM_OFFSET] = key_info->algorithm;

	memcpy(&T_key_info, key_info, sizeof(T_key_info));
	fsl_shw_sko_set_key_length(&T_key_info, T_LENGTH);
	T_key_info.algorithm = FSL_KEY_ALG_HMAC;

	memcpy(&KEK_key_info, &T_key_info, sizeof(KEK_key_info));
	KEK_key_info.algorithm = FSL_KEY_ALG_AES;

	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		ret = do_system_keystore_slot_alloc(user_ctx,
				T_LENGTH, key_info->userid,
			    &T_key_info.handle);

	} else {
		/* Key goes in user keystore */
		ret = keystore_slot_alloc(key_info->keystore,
				  T_LENGTH,
				  key_info->userid, &T_key_info.handle);
	}
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		ret = do_system_keystore_slot_alloc(user_ctx,
				KEK_LENGTH, key_info->userid,
				&KEK_key_info.handle);

	} else {
		/* Key goes in user keystore */
		ret = keystore_slot_alloc(key_info->keystore,
			  KEK_LENGTH,  key_info->userid,
			  &KEK_key_info.handle);
	}

	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("do_scc_slot_alloc() failed");
#endif
		if (key_info->keystore == NULL) {
			/* Key goes in system keystore */
			(void)do_system_keystore_slot_dealloc(user_ctx,
				key_info->userid, T_key_info.handle);

		} else {
			/* Key goes in user keystore */
			(void)keystore_slot_dealloc(key_info->keystore,
					key_info->userid, T_key_info.handle);
		}
	} else {
		slots_allocated = 1;
	}

	/* Set up to compute everything except T' ... */
#ifndef DO_REPEATABLE_WRAP
	/* Compute T = RND() */
	header = SAH_HDR_RNG_GENERATE;	/* Desc. #18 */
	DESC_KEY_OUT(header, &T_key_info, 0, NULL);
#else
	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		ret = do_system_keystore_slot_load(user_ctx,
			   T_key_info.userid,
			   T_key_info.handle, T_block,
			   T_key_info.key_length);
	} else {
		/* Key goes in user keystore */
		ret = keystore_slot_load(key_info->keystore,
				 T_key_info.userid,
				 T_key_info.handle,
				 T_block, T_key_info.key_length);
	}

	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}
#endif

	/* Compute KEK = SHA256(T | Ownerid) */
	header = (SAH_HDR_MDHA_SET_MODE_HASH	/* #8 */
		  ^ sah_insert_mdha_init
		  ^ sah_insert_mdha_algorithm[FSL_HASH_ALG_SHA256]
		  ^ sah_insert_mdha_pdata);
	/* Input - Start with T */
	ret = sah_Create_Key_Link(user_ctx->mem_util, &link1, &T_key_info);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}
	/* Still input - append ownerid */
	ret = sah_Append_Link(user_ctx->mem_util, link1,
			      (void *)&key_info->userid,
			      sizeof(key_info->userid), SAH_USES_LINK_DATA);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}
	/* Output - KEK goes into RED slot */
	ret = sah_Create_Key_Link(user_ctx->mem_util, &link2, &KEK_key_info);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}
	/* Put the Hash calculation into the chain. */
	ret = sah_Append_Desc(user_ctx->mem_util, &desc_chain,
			      header, link1, link2);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}
#if defined(NEED_CTR_WORKAROUND)
	rounded_key_length = ROUND_LENGTH(original_key_length);
	key_info->key_length = rounded_key_length;
#else
	rounded_key_length = original_key_length;
#endif
	/* Compute KEY' = AES-encrypt(KEK, KEY) */
	header = (SAH_HDR_SKHA_SET_MODE_IV_KEY	/* #1 */
		  ^ sah_insert_skha_mode[FSL_SYM_MODE_CTR]
		  ^ sah_insert_skha_algorithm[FSL_KEY_ALG_AES]
		  ^ sah_insert_skha_modulus[FSL_CTR_MOD_128]);
	/* Set up KEK as key to use */
	DESC_IN_KEY(header, 0, NULL, &KEK_key_info);
	header = SAH_HDR_SKHA_ENC_DEC;
	DESC_KEY_OUT(header, key_info,
		     key_info->key_length, black_key + KEY_PRIME_OFFSET);

	/* Set up flags info */
	black_key[FLAGS_OFFSET] = 0;
	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
		black_key[FLAGS_OFFSET] |= FLAGS_SW_KEY;
	}

	/* Compute and store ICV into Black Key */
	ret = create_icv_calc(user_ctx, &desc_chain, &T_key_info,
			      black_key, original_key_length,
			      black_key + ICV_OFFSET);
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Creation of sah_Key_Link failed due to bad key"
			 " flag!\n");
#endif				/*DIAG_SECURITY_FUNC */
		goto out;
	}

	/* Now get Sahara to do the work. */
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG("Encrypting key with KEK");
#endif
	SAH_SF_EXECUTE();
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("sah_Descriptor_Chain_Execute() failed");
#endif
		goto out;
	}

	/* Compute T' = SLID_encrypt(T); Result goes to Black Key */
	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		ret = do_system_keystore_slot_encrypt(user_ctx,
					T_key_info.userid,  T_key_info.handle,
					T_LENGTH, black_key + T_PRIME_OFFSET);
	} else {
		/* Key goes in user keystore */
		ret = keystore_slot_encrypt(user_ctx,
					    key_info->keystore,
					    T_key_info.userid,
					    T_key_info.handle,
					    T_LENGTH,
					    black_key + T_PRIME_OFFSET);
	}

	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("do_scc_slot_encrypt() failed");
#endif
		goto out;
	}

      out:
	key_info->key_length = original_key_length;

	SAH_SF_DESC_CLEAN();
	if (slots_allocated) {
		if (key_info->keystore == NULL) {
			/* Key goes in system keystore */
			(void)do_system_keystore_slot_dealloc(user_ctx,
							      key_info->userid,
							      T_key_info.
							      handle);
			(void)do_system_keystore_slot_dealloc(user_ctx,
							      key_info->userid,
							      KEK_key_info.
							      handle);
		} else {
			/* Key goes in user keystore */
			(void)keystore_slot_dealloc(key_info->keystore,
						    key_info->userid,
				    T_key_info.handle);
			(void)keystore_slot_dealloc(key_info->keystore,
				    key_info->userid,
				    KEK_key_info.handle);
	    }
	}

	return ret;
}				/* wrap */

/*!
 * Place a key into a protected location for use only by cryptographic
 * algorithms.
 *
 * This only needs to be used to a) unwrap a key, or b) set up a key which
 * could be wrapped with a later call to #fsl_shw_extract_key().  Normal
 * cleartext keys can simply be placed into #fsl_shw_sko_t key objects with
 * #fsl_shw_sko_set_key() and used directly.
 *
 * The maximum key size supported for wrapped/unwrapped keys is 32 octets.
 * (This is the maximum reasonable key length on Sahara - 32 octets for an HMAC
 * key based on SHA-256.)  The key size is determined by the @a key_info.  The
 * expected length of @a key can be determined by
 * #fsl_shw_sko_calculate_wrapped_size()
 *
 * The protected key will not be available for use until this operation
 * successfully completes.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param[in,out] key_info      The information about the key to be which will
 *                              be established.  In the create case, the key
 *                              length must be set.
 * @param      establish_type   How @a key will be interpreted to establish a
 *                              key for use.
 * @param key                   If @a establish_type is #FSL_KEY_WRAP_UNWRAP,
 *                              this is the location of a wrapped key.  If
 *                              @a establish_type is #FSL_KEY_WRAP_CREATE, this
 *                              parameter can be @a NULL.  If @a establish_type
 *                              is #FSL_KEY_WRAP_ACCEPT, this is the location
 *                              of a plaintext key.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_establish_key(fsl_shw_uco_t * user_ctx,
				       fsl_shw_sko_t * key_info,
				       fsl_shw_key_wrap_t establish_type,
				       const uint8_t * key)
{
	SAH_SF_DCLS;
	unsigned original_key_length = key_info->key_length;
	unsigned rounded_key_length;
	unsigned slot_allocated = 0;
	uint32_t old_flags;

	header = SAH_HDR_RNG_GENERATE;	/* Desc. #18 for rand */

	/* TODO: THIS STILL NEEDS TO BE REFACTORED */

	/* Write operations into SCC memory require word-multiple number of
	 * bytes.  For ACCEPT and CREATE functions, the key length may need
	 * to be rounded up.  Calculate. */
	if (LENGTH_PATCH && (original_key_length & LENGTH_PATCH_MASK) != 0) {
		rounded_key_length = original_key_length + LENGTH_PATCH
		    - (original_key_length & LENGTH_PATCH_MASK);
	} else {
		rounded_key_length = original_key_length;
	}

	SAH_SF_USER_CHECK();

	if (key_info->flags & FSL_SKO_KEY_ESTABLISHED) {
#ifdef DIAG_SECURITY_FUNC
			ret = FSL_RETURN_BAD_FLAG_S;
			LOG_DIAG("Key already established\n");
#endif
	}


	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		ret = do_system_keystore_slot_alloc(user_ctx,
						    key_info->key_length,
						    key_info->userid,
						    &(key_info->handle));
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS
		    ("key length: %i, handle: %i, rounded key length: %i",
		     key_info->key_length, key_info->handle,
		     rounded_key_length);
#endif

	} else {
		/* Key goes in user keystore */
		ret = keystore_slot_alloc(key_info->keystore,
					  key_info->key_length,
					  key_info->userid,
					  &(key_info->handle));
	}
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Slot allocation failed\n");
#endif
		goto out;
	}
	slot_allocated = 1;

	key_info->flags |= FSL_SKO_KEY_ESTABLISHED;
	switch (establish_type) {
	case FSL_KEY_WRAP_CREATE:
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Creating random key\n");
#endif
		/* Use safe version of key length */
		key_info->key_length = rounded_key_length;
		/* Generate descriptor to put random value into */
		DESC_KEY_OUT(header, key_info, 0, NULL);
		/* Restore actual, desired key length */
		key_info->key_length = original_key_length;

		old_flags = user_ctx->flags;
		/* Now put random value into key */
		SAH_SF_EXECUTE();
		/* Restore user's old flag value */
		user_ctx->flags = old_flags;
#ifdef DIAG_SECURITY_FUNC
		if (ret == FSL_RETURN_OK_S) {
			LOG_DIAG("ret is ok");
		} else {
			LOG_DIAG("ret is not ok");
		}
#endif
		break;

	case FSL_KEY_WRAP_ACCEPT:
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Accepting plaintext key\n");
#endif
		if (key == NULL) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG("ACCEPT:  Red Key is NULL");
#endif
			ret = FSL_RETURN_ERROR_S;
			goto out;
		}
		/* Copy in safe number of bytes of Red key */
		if (key_info->keystore == NULL) {
			/* Key goes in system keystore */
			ret = do_system_keystore_slot_load(user_ctx,
						   key_info->userid,
						   key_info->handle, key,
						   rounded_key_length);
		} else {
			/* Key goes in user keystore */
			ret = keystore_slot_load(key_info->keystore,
								 key_info->userid,
								 key_info->handle, key,
								 key_info->key_length);
		}
		break;

	case FSL_KEY_WRAP_UNWRAP:
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Unwrapping wrapped key\n");
#endif
		/* For now, disallow non-blocking calls. */
		if (!(user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {
			ret = FSL_RETURN_BAD_FLAG_S;
		} else if (key == NULL) {
			ret = FSL_RETURN_ERROR_S;
		} else {
			ret = unwrap(user_ctx, key_info, key);
		}
		break;

	default:
		ret = FSL_RETURN_BAD_FLAG_S;
		break;
	}			/* switch */

      out:
	if (slot_allocated && (ret != FSL_RETURN_OK_S)) {
		fsl_shw_return_t scc_err;

		if (key_info->keystore == NULL) {
			/* Key goes in system keystore */
			scc_err = do_system_keystore_slot_dealloc(user_ctx,
						  key_info->userid,
						  key_info->handle);
		} else {
			/* Key goes in user keystore */
			scc_err = keystore_slot_dealloc(key_info->keystore,
						key_info->userid,  key_info->handle);
		}

		key_info->flags &= ~FSL_SKO_KEY_ESTABLISHED;
	}

	SAH_SF_DESC_CLEAN();

	return ret;
}				/* fsl_shw_establish_key() */

/*!
 * Wrap a key and retrieve the wrapped value.
 *
 * A wrapped key is a key that has been cryptographically obscured.  It is
 * only able to be used with #fsl_shw_establish_key().
 *
 * This function will also release the key (see #fsl_shw_release_key()) so
 * that it must be re-established before reuse.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      key_info         The information about the key to be deleted.
 * @param[out] covered_key      The location to store the 48-octet wrapped key.
 *                              (This size is based upon the maximum key size
 *                              of 32 octets).
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_extract_key(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info,
				     uint8_t * covered_key)
{
	SAH_SF_DCLS;

	SAH_SF_USER_CHECK();

	/* For now, only blocking mode calls are supported */
	if (user_ctx->flags & FSL_UCO_BLOCKING_MODE) {
		if (key_info->flags & FSL_SKO_KEY_ESTABLISHED) {
			ret = wrap(user_ctx, key_info, covered_key);
			if (ret != FSL_RETURN_OK_S) {
				goto out;
			}

			/* Verify that a SW key info really belongs to a SW key */
			if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
			/*  ret = FSL_RETURN_BAD_FLAG_S;
			  goto out;*/
			}

			/* Need to deallocate on successful extraction */
			if (key_info->keystore == NULL) {
				/* Key goes in system keystore */
				ret = do_system_keystore_slot_dealloc(user_ctx,
						key_info->userid, key_info->handle);
			} else {
				/* Key goes in user keystore */
				ret = keystore_slot_dealloc(key_info->keystore,
						key_info->userid, key_info->handle);
			}
			/* Mark key not available in the flags */
			key_info->flags &=
			    ~(FSL_SKO_KEY_ESTABLISHED | FSL_SKO_KEY_PRESENT);
		}
	}

out:
	SAH_SF_DESC_CLEAN();

	return ret;
}

/*!
 * De-establish a key so that it can no longer be accessed.
 *
 * The key will need to be re-established before it can again be used.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      key_info         The information about the key to be deleted.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_release_key(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info)
{
	SAH_SF_DCLS;

	SAH_SF_USER_CHECK();

	if (key_info->flags & FSL_SKO_KEY_ESTABLISHED) {
		if (key_info->keystore == NULL) {
			/* Key goes in system keystore */
			do_system_keystore_slot_dealloc(user_ctx,
							key_info->userid,
							key_info->handle);
		} else {
			/* Key goes in user keystore */
			keystore_slot_dealloc(key_info->keystore,
						key_info->userid,
						key_info->handle);
		}
		key_info->flags &= ~(FSL_SKO_KEY_ESTABLISHED |
				     FSL_SKO_KEY_PRESENT);
	}

out:
	SAH_SF_DESC_CLEAN();

	return ret;
}

fsl_shw_return_t fsl_shw_read_key(fsl_shw_uco_t * user_ctx,
				  fsl_shw_sko_t * key_info, uint8_t * key)
{
	SAH_SF_DCLS;

	SAH_SF_USER_CHECK();

	if (!(key_info->flags & FSL_SKO_KEY_ESTABLISHED)
	    || !(key_info->flags & FSL_SKO_KEY_SW_KEY)) {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	if (key_info->keystore == NULL) {
		/* Key lives in system keystore */
		ret = do_system_keystore_slot_read(user_ctx,
						   key_info->userid,
						   key_info->handle,
						   key_info->key_length, key);
	} else {
		/* Key lives in user keystore */
		ret = keystore_slot_read(key_info->keystore,
					 key_info->userid,
					 key_info->handle,
					 key_info->key_length, key);
	}

      out:
	SAH_SF_DESC_CLEAN();

	return ret;
}
