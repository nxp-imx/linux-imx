/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file fsl_shw_hash.c
 *
 * This file implements Cryptographic Hashing functions of the FSL SHW API
 * for Sahara.  This does not include HMAC.
 */

#include "sahara.h"
#include "sf_util.h"

#ifdef LINUX_KERNEL
EXPORT_SYMBOL(fsl_shw_hash);
#endif

/* REQ-S2LRD-PINTFC-API-BASIC-HASH-005 */
/*!
 * Hash a stream of data with a cryptographic hash algorithm.
 *
 * The flags in the @a hash_ctx control the operation of this function.
 *
 * Hashing functions work on 64 octets of message at a time.  Therefore, when
 * any partial hashing of a long message is performed, the message @a length of
 * each segment must be a multiple of 64.  When ready to
 * #FSL_HASH_FLAGS_FINALIZE the hash, the @a length may be any value.
 *
 * With the #FSL_HASH_FLAGS_INIT and #FSL_HASH_FLAGS_FINALIZE flags on, a
 * one-shot complete hash, including padding, will be performed.  The @a length
 * may be any value.
 *
 * The first octets of a data stream can be hashed by setting the
 * #FSL_HASH_FLAGS_INIT and #FSL_HASH_FLAGS_SAVE flags.  The @a length must be
 * a multiple of 64.
 *
 * The flag #FSL_HASH_FLAGS_LOAD is used to load a context previously saved by
 * #FSL_HASH_FLAGS_SAVE.  The two in combination will allow a (multiple-of-64
 * octets) 'middle sequence' of the data stream to be hashed with the
 * beginning.  The @a length must again be a multiple of 64.
 *
 * Since the flag #FSL_HASH_FLAGS_LOAD is used to load a context previously
 * saved by #FSL_HASH_FLAGS_SAVE, the #FSL_HASH_FLAGS_LOAD and
 * #FSL_HASH_FLAGS_FINALIZE flags, used together, can be used to finish the
 * stream.  The @a length may be any value.
 *
 * If the user program wants to do the padding for the hash, it can leave off
 * the #FSL_HASH_FLAGS_FINALIZE flag.  The @a length must then be a multiple of
 * 64 octets.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param[in,out] hash_ctx Hashing algorithm and state of the cipher.
 * @param      msg       Pointer to the data to be hashed.
 * @param      length    Length, in octets, of the @a msg.
 * @param[out] result    If not null, pointer to where to store the hash
 *                       digest.
 * @param      result_len Number of octets to store in @a result.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_hash(fsl_shw_uco_t * user_ctx,
			      fsl_shw_hco_t * hash_ctx,
			      const uint8_t * msg,
			      uint32_t length,
			      uint8_t * result, uint32_t result_len)
{
	SAH_SF_DCLS;
	unsigned ctx_flags = (hash_ctx->flags & (FSL_HASH_FLAGS_INIT
						 | FSL_HASH_FLAGS_LOAD
						 | FSL_HASH_FLAGS_SAVE
						 | FSL_HASH_FLAGS_FINALIZE));

	SAH_SF_USER_CHECK();

	/* Reset expectations if user gets overly zealous. */
	if (result_len > hash_ctx->digest_length) {
		result_len = hash_ctx->digest_length;
	}

	/* Validate hash ctx flags.
	 * Need INIT or LOAD but not both.
	 * Need SAVE or digest ptr (both is ok).
	 */
	if (((ctx_flags & (FSL_HASH_FLAGS_INIT | FSL_HASH_FLAGS_LOAD))
	     == (FSL_HASH_FLAGS_INIT | FSL_HASH_FLAGS_LOAD))
	    || ((ctx_flags & (FSL_HASH_FLAGS_INIT | FSL_HASH_FLAGS_LOAD)) == 0)
	    || (!(ctx_flags & FSL_HASH_FLAGS_SAVE) && (result == NULL))) {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	if (ctx_flags & FSL_HASH_FLAGS_INIT) {
		sah_Oct_Str out_ptr;
		unsigned out_len;

		/* Create desc to perform the initial hashing operation */
		/* Desc. #8 w/INIT and algorithm */
		header = SAH_HDR_MDHA_SET_MODE_HASH
		    ^ sah_insert_mdha_init
		    ^ sah_insert_mdha_algorithm[hash_ctx->algorithm];

		/* If user wants one-shot, set padding operation. */
		if (ctx_flags & FSL_HASH_FLAGS_FINALIZE) {
			header ^= sah_insert_mdha_pdata;
		}

		/* Determine where Digest will go - hash_ctx or result */
		if (ctx_flags & FSL_HASH_FLAGS_SAVE) {
			out_ptr = (sah_Oct_Str) hash_ctx->context;
			out_len = hash_ctx->context_register_length;
		} else {
			out_ptr = result;
			out_len = (result_len > hash_ctx->digest_length)
			    ? hash_ctx->digest_length : result_len;
		}

		DESC_IN_OUT(header, length, (sah_Oct_Str) msg, out_len,
			    out_ptr);
	} else {		/* not doing hash INIT */
		void *out_ptr;
		unsigned out_len;

		/*
		 * Build two descriptors -- one to load in context/set mode, the
		 * other to compute & retrieve hash/context value.
		 *
		 * First up - Desc. #6 to load context.
		 */
		/* Desc. #8 w/algorithm */
		header = SAH_HDR_MDHA_SET_MODE_MD_KEY
		    ^ sah_insert_mdha_algorithm[hash_ctx->algorithm];

		if (ctx_flags & FSL_HASH_FLAGS_FINALIZE) {
			header ^= sah_insert_mdha_pdata;
		}

		/* Message Digest (in) */
		DESC_IN_IN(header,
			   hash_ctx->context_register_length,
			   (sah_Oct_Str) hash_ctx->context, 0, NULL);

		if (ctx_flags & FSL_HASH_FLAGS_SAVE) {
			out_ptr = hash_ctx->context;
			out_len = hash_ctx->context_register_length;
		} else {
			out_ptr = result;
			out_len = result_len;
		}

		/* Second -- run data through and retrieve ctx regs */
		/* Desc. #10 - no mode register with this. */
		header = SAH_HDR_MDHA_HASH;
		DESC_IN_OUT(header, length, (sah_Oct_Str) msg, out_len,
			    out_ptr);
	}			/* else not INIT */

	/* Now that execution is rejoined, we can append another descriptor
	   to extract the digest/context a second time, into the result. */
	if ((ctx_flags & FSL_HASH_FLAGS_SAVE)
	    && (result != NULL) && (result_len != 0)) {

		header = SAH_HDR_MDHA_STORE_DIGEST;

		/* Message Digest (out) */
		DESC_IN_OUT(header, 0, NULL,
			    (result_len > hash_ctx->digest_length)
			    ? hash_ctx->digest_length : result_len, result);
	}

	SAH_SF_EXECUTE();

      out:
	SAH_SF_DESC_CLEAN();

	return ret;
}				/* fsl_shw_hash() */
