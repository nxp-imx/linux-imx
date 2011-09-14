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
 * @file fsl_shw_sym.c
 *
 * This file implements the Symmetric Cipher functions of the FSL SHW API. Its
 * features are limited to what can be done with the combination of SCC and
 * DryIce.
 */
#include "fsl_platform.h"
#include "shw_driver.h"

#if defined(__KERNEL__) && defined(FSL_HAVE_DRYICE)

#include "../dryice.h"
#include <linux/mxc_scc_driver.h>
#ifdef DIAG_SECURITY_FUNC
#include "apihelp.h"
#endif

#include <diagnostic.h>

#define SYM_DECRYPT 0
#define SYM_ENCRYPT 1

extern fsl_shw_return_t shw_convert_pf_key(fsl_shw_pf_key_t shw_pf_key,
					   di_key_t * di_keyp);

/*! 'Initial' IV for presence of FSL_SYM_CTX_LOAD flag */
static uint8_t zeros[8] = {
	0, 0, 0, 0, 0, 0, 0, 0
};

/*!
 * Common function for encryption and decryption
 *
 * This is for a device with DryIce.
 *
 * A key must either refer to a 'pure' HW key, or, if PRG or PRG_IIM,
 * established, then that key will be programmed.  Then, the HW_key in the
 * object will be selected.  After this setup, the ciphering will be performed
 * by calling the SCC driver..
 *
 * The function 'releases' the reservations before it completes.
 */
fsl_shw_return_t do_symmetric(fsl_shw_uco_t * user_ctx,
			      fsl_shw_sko_t * key_info,
			      fsl_shw_scco_t * sym_ctx,
			      int encrypt,
			      uint32_t length,
			      const uint8_t * in, uint8_t * out)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;
	int key_selected = 0;
	uint8_t *iv = NULL;
	unsigned long count_out = length;
	di_key_t di_key = DI_KEY_PK;	/* default for user key */
	di_key_t di_key_orig;	/* currently selected key */
	di_key_t selected_key = -1;
	di_return_t di_code;
	scc_return_t scc_code;

	/* For now, only blocking mode calls are supported */
	if (!(user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	/* No software keys allowed */
	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
		ret = FSL_RETURN_BAD_FLAG_S;
	}

	/* The only algorithm the SCC supports */
	if (key_info->algorithm != FSL_KEY_ALG_TDES) {
		ret = FSL_RETURN_BAD_ALGORITHM_S;
		goto out;
	}

	/* Validate key length */
	if ((key_info->key_length != 16)
	    && (key_info->key_length != 21)
	    && (key_info->key_length != 24)) {
		ret = FSL_RETURN_BAD_KEY_LENGTH_S;
		goto out;
	}

	/* Validate data is multiple of DES/TDES block */
	if ((length & 7) != 0) {
		ret = FSL_RETURN_BAD_DATA_LENGTH_S;
		goto out;
	}

	/* Do some setup according to where the key lives */
	if (key_info->flags & FSL_SKO_KEY_ESTABLISHED) {
		if ((key_info->pf_key != FSL_SHW_PF_KEY_PRG)
		    && (key_info->pf_key != FSL_SHW_PF_KEY_IIM_PRG)) {
			ret = FSL_RETURN_ERROR_S;
		}
	} else if (key_info->flags & FSL_SKO_KEY_PRESENT) {
		ret = FSL_RETURN_BAD_FLAG_S;
	} else if (key_info->flags & FSL_SKO_KEY_SELECT_PF_KEY) {
		/*
		 * No key present or established, just refer to HW
		 * as programmed.
		 */
	} else {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	/* Now make proper selection */
	ret = shw_convert_pf_key(key_info->pf_key, &di_key);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Determine the current DI key selection */
	di_code = dryice_check_key(&di_key_orig);
	if (di_code != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("Could not save current DI key state: %s\n",
					  di_error_string(di_code));
#endif
		ret = FSL_RETURN_ERROR_S;
		goto out;
	}

	/* If the requested DI key is already selected, don't re-select it. */
	if (di_key != di_key_orig) {
	   di_code = dryice_select_key(di_key, 0);
	   if (di_code != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
		  LOG_DIAG_ARGS("Error from select_key: %s\n",
			      di_error_string(di_code));
#endif
		  ret = FSL_RETURN_INTERNAL_ERROR_S;
		  goto out;
	   }
	}
	key_selected = 1;

	/* Verify that we are using the key we want */
	di_code = dryice_check_key(&selected_key);
	if (di_code != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("Error from check_key: %s\n",
			      di_error_string(di_code));
#endif
		ret = FSL_RETURN_INTERNAL_ERROR_S;
		goto out;
	}

	if (di_key != selected_key) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("Wrong key in use: %d instead of %d\n\n",
			      selected_key, di_key);
#endif
		ret = FSL_RETURN_ERROR_S;
		goto out;
	}

	if (sym_ctx->mode == FSL_SYM_MODE_CBC) {
		if ((sym_ctx->flags & FSL_SYM_CTX_LOAD)
		    && !(sym_ctx->flags & FSL_SYM_CTX_INIT)) {
			iv = sym_ctx->context;
		} else if ((sym_ctx->flags & FSL_SYM_CTX_INIT)
			   && !(sym_ctx->flags & FSL_SYM_CTX_LOAD)) {
			iv = zeros;
		} else {
			/* Exactly one must be set! */
			ret = FSL_RETURN_BAD_FLAG_S;
			goto out;
		}
	}

	/* Now run the data through the SCC */
	scc_code = scc_crypt(length, in, iv,
			     encrypt ? SCC_ENCRYPT : SCC_DECRYPT,
			     (sym_ctx->mode == FSL_SYM_MODE_ECB)
			     ? SCC_ECB_MODE : SCC_CBC_MODE,
			     SCC_VERIFY_MODE_NONE, out, &count_out);
	if (scc_code != SCC_RET_OK) {
		ret = FSL_RETURN_INTERNAL_ERROR_S;
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("scc_code from scc_crypt() is %d\n", scc_code);
#endif
		goto out;
	}

	if ((sym_ctx->mode == FSL_SYM_MODE_CBC)
	    && (sym_ctx->flags & FSL_SYM_CTX_SAVE)) {
		/* Save the context for the caller */
		if (encrypt) {
			/* Last ciphertext block ... */
			memcpy(sym_ctx->context, out + length - 8, 8);
		} else {
			/* Last ciphertext block ... */
			memcpy(sym_ctx->context, in + length - 8, 8);
		}
	}

	ret = FSL_RETURN_OK_S;

      out:
	if (key_selected) {
		(void)dryice_release_key_selection();
	}

	return ret;
}

EXPORT_SYMBOL(fsl_shw_symmetric_encrypt);
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

EXPORT_SYMBOL(fsl_shw_symmetric_decrypt);
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

#else				/* __KERNEL__ && DRYICE */

fsl_shw_return_t fsl_shw_symmetric_encrypt(fsl_shw_uco_t * user_ctx,
					   fsl_shw_sko_t * key_info,
					   fsl_shw_scco_t * sym_ctx,
					   uint32_t length,
					   const uint8_t * pt, uint8_t * ct)
{
	/* Unused */
	(void)user_ctx;
	(void)key_info;
	(void)sym_ctx;
	(void)length;
	(void)pt;
	(void)ct;

	return FSL_RETURN_ERROR_S;
}

fsl_shw_return_t fsl_shw_symmetric_decrypt(fsl_shw_uco_t * user_ctx,
					   fsl_shw_sko_t * key_info,
					   fsl_shw_scco_t * sym_ctx,
					   uint32_t length,
					   const uint8_t * ct, uint8_t * pt)
{
	/* Unused */
	(void)user_ctx;
	(void)key_info;
	(void)sym_ctx;
	(void)length;
	(void)ct;
	(void)pt;

	return FSL_RETURN_ERROR_S;
}

#endif				/* __KERNEL__ and DRYICE */
