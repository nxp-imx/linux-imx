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
 * @file fsl_shw_sym.c
 *
 * This file implements Symmetric Cipher functions of the FSL SHW API for
 * Sahara.  This does not include CCM.
 */

#include <linux/mxc_sahara.h>
#include "fsl_platform.h"

#include "sf_util.h"
#include "adaptor.h"

#ifdef LINUX_KERNEL
EXPORT_SYMBOL(fsl_shw_symmetric_encrypt);
EXPORT_SYMBOL(fsl_shw_symmetric_decrypt);
#endif

#if defined(NEED_CTR_WORKAROUND)
/* CTR mode needs block-multiple data in/out */
#define LENGTH_PATCH  (sym_ctx->block_size_bytes)
#define LENGTH_PATCH_MASK  (sym_ctx->block_size_bytes-1)
#else
#define LENGTH_PATCH  0
#define LENGTH_PATCH_MASK  0	/* du not use! */
#endif

/*!
 * Block of zeroes which is maximum Symmetric block size, used for
 * initializing context register, etc.
 */
static uint32_t block_zeros[4] = {
	0, 0, 0, 0
};

typedef enum cipher_direction {
	SYM_DECRYPT,
	SYM_ENCRYPT
} cipher_direction_t;

/*!
 * Create and run the chain for a symmetric-key operation.
 *
 * @param user_ctx    Who the user is
 * @param key_info    What key is to be used
 * @param sym_ctx     Info details about algorithm
 * @param encrypt     0 = decrypt, non-zero = encrypt
 * @param length      Number of octets at @a in and @a out
 * @param in          Pointer to input data
 * @param out         Location to store output data
 *
 * @return         The status of handing chain to driver,
 *                 or an earlier argument/flag or allocation
 *                 error.
 */
static fsl_shw_return_t do_symmetric(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info,
				     fsl_shw_scco_t * sym_ctx,
				     cipher_direction_t encrypt,
				     uint32_t length,
				     const uint8_t * in, uint8_t * out)
{
	SAH_SF_DCLS;
	uint8_t *sink = NULL;
	sah_Link *link1 = NULL;
	sah_Link *link2 = NULL;
	sah_Oct_Str ptr1;
	uint32_t size1 = sym_ctx->block_size_bytes;

	SAH_SF_USER_CHECK();

	/* Two different sets of chains, depending on algorithm */
	if (key_info->algorithm == FSL_KEY_ALG_ARC4) {
		if (sym_ctx->flags & FSL_SYM_CTX_INIT) {
			/* Desc. #35 w/ARC4 - start from key */
			header = SAH_HDR_ARC4_SET_MODE_KEY
			    ^ sah_insert_skha_algorithm_arc4;

			DESC_IN_KEY(header, 0, NULL, key_info);
		} else {	/* load SBox */
			/* Desc. #33 w/ARC4 and NO PERMUTE */
			header = SAH_HDR_ARC4_SET_MODE_SBOX
			    ^ sah_insert_skha_no_permute
			    ^ sah_insert_skha_algorithm_arc4;
			DESC_IN_IN(header, 256, sym_ctx->context,
				   3, sym_ctx->context + 256);
		}		/* load SBox */

		/* Add in-out data descriptor to process the data */
		if (length != 0) {
			DESC_IN_OUT(SAH_HDR_SKHA_ENC_DEC, length, in, length,
				    out);
		}

		/* Operation is done ... save what came out? */
		if (sym_ctx->flags & FSL_SYM_CTX_SAVE) {
			/* Desc. #34 - Read SBox, pointers */
			header = SAH_HDR_ARC4_READ_SBOX;
			DESC_OUT_OUT(header, 256, sym_ctx->context,
				     3, sym_ctx->context + 256);
		}
	} else {		/* not ARC4 */
		/* Doing 1- or 2- descriptor chain. */
		/* Desc. #1 and algorithm and mode */
		header = SAH_HDR_SKHA_SET_MODE_IV_KEY
		    ^ sah_insert_skha_mode[sym_ctx->mode]
		    ^ sah_insert_skha_algorithm[key_info->algorithm];

		/* Honor 'no key parity checking' for DES and TDES */
		if ((key_info->flags & FSL_SKO_KEY_IGNORE_PARITY) &&
		    ((key_info->algorithm == FSL_KEY_ALG_DES) ||
		     (key_info->algorithm == FSL_KEY_ALG_TDES))) {
			header ^= sah_insert_skha_no_key_parity;
		}

		/* Header by default is decrypting, so... */
		if (encrypt == SYM_ENCRYPT) {
			header ^= sah_insert_skha_encrypt;
		}

		if (sym_ctx->mode == FSL_SYM_MODE_CTR) {
			header ^= sah_insert_skha_modulus[sym_ctx->modulus_exp];
		}

		if (sym_ctx->mode == FSL_SYM_MODE_ECB) {
			ptr1 = NULL;
			size1 = 0;
		} else if (sym_ctx->flags & FSL_SYM_CTX_INIT) {
			ptr1 = (uint8_t *) block_zeros;
		} else {
			ptr1 = sym_ctx->context;
		}

		DESC_IN_KEY(header, sym_ctx->block_size_bytes, ptr1, key_info);

		/* Add in-out data descriptor */
		if (length != 0) {
			header = SAH_HDR_SKHA_ENC_DEC;
			if (LENGTH_PATCH && (sym_ctx->mode == FSL_SYM_MODE_CTR)
			    && ((length & LENGTH_PATCH_MASK) != 0)) {
				sink = DESC_TEMP_ALLOC(LENGTH_PATCH);
				ret =
				    sah_Create_Link(user_ctx->mem_util, &link1,
						    (uint8_t *) in, length,
						    SAH_USES_LINK_DATA);
				ret =
				    sah_Append_Link(user_ctx->mem_util, link1,
						    (uint8_t *) sink,
						    LENGTH_PATCH -
						    (length &
						     LENGTH_PATCH_MASK),
						    SAH_USES_LINK_DATA);
				if (ret != FSL_RETURN_OK_S) {
					goto out;
				}
				ret =
				    sah_Create_Link(user_ctx->mem_util, &link2,
						    out, length,
						    SAH_USES_LINK_DATA |
						    SAH_OUTPUT_LINK);
				if (ret != FSL_RETURN_OK_S) {
					goto out;
				}
				ret = sah_Append_Link(user_ctx->mem_util, link2,
						      sink,
						      LENGTH_PATCH -
						      (length &
						       LENGTH_PATCH_MASK),
						      SAH_USES_LINK_DATA);
				if (ret != FSL_RETURN_OK_S) {
					goto out;
				}
				ret =
				    sah_Append_Desc(user_ctx->mem_util,
						    &desc_chain, header, link1,
						    link2);
				if (ret != FSL_RETURN_OK_S) {
					goto out;
				}
				link1 = link2 = NULL;
			} else {
				DESC_IN_OUT(header, length, in, length, out);
			}
		}

		/* Unload any desired context */
		if (sym_ctx->flags & FSL_SYM_CTX_SAVE) {
			DESC_OUT_OUT(SAH_HDR_SKHA_READ_CONTEXT_IV, 0, NULL,
				     sym_ctx->block_size_bytes,
				     sym_ctx->context);
		}

	}			/* not ARC4 */

	SAH_SF_EXECUTE();

      out:
	SAH_SF_DESC_CLEAN();
	DESC_TEMP_FREE(sink);
	if (LENGTH_PATCH) {
		sah_Destroy_Link(user_ctx->mem_util, link1);
		sah_Destroy_Link(user_ctx->mem_util, link2);
	}

	return ret;
}

/* REQ-S2LRD-PINTFC-API-BASIC-SYM-002 */
/* PINTFC-API-BASIC-SYM-ARC4-001 */
/* PINTFC-API-BASIC-SYM-ARC4-002 */

/*!
 * Compute symmetric encryption
 *
 *
 * @param    user_ctx
 * @param    key_info
 * @param    sym_ctx
 * @param    length
 * @param    pt
 * @param    ct
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_symmetric_encrypt(fsl_shw_uco_t * user_ctx,
					   fsl_shw_sko_t * key_info,
					   fsl_shw_scco_t * sym_ctx,
					   uint32_t length,
					   const uint8_t * pt, uint8_t * ct)
{
	fsl_shw_return_t ret;

	ret = do_symmetric(user_ctx, key_info, sym_ctx, SYM_ENCRYPT,
			   length, pt, ct);

	return ret;
}

/* PINTFC-API-BASIC-SYM-002 */
/* PINTFC-API-BASIC-SYM-ARC4-001 */
/* PINTFC-API-BASIC-SYM-ARC4-002 */

/*!
 * Compute symmetric decryption
 *
 *
 * @param    user_ctx
 * @param    key_info
 * @param    sym_ctx
 * @param    length
 * @param    pt
 * @param    ct
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_symmetric_decrypt(fsl_shw_uco_t * user_ctx,
					   fsl_shw_sko_t * key_info,
					   fsl_shw_scco_t * sym_ctx,
					   uint32_t length,
					   const uint8_t * ct, uint8_t * pt)
{
	fsl_shw_return_t ret;

	ret = do_symmetric(user_ctx, key_info, sym_ctx, SYM_DECRYPT,
			   length, ct, pt);

	return ret;
}
