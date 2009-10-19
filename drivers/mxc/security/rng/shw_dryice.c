/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include "shw_driver.h"
#include "../dryice.h"

#include <diagnostic.h>

#ifdef FSL_HAVE_DRYICE

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_gen_random_pf_key);
#endif
/*!
 * Cause the hardware to create a new random key for secure memory use.
 *
 * Have the hardware use the secure hardware random number generator to load a
 * new secret key into the hardware random key register.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_gen_random_pf_key(fsl_shw_uco_t * user_ctx)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;
	di_return_t di_ret;

	/* For now, only blocking mode calls are supported */
	if (!(user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	di_ret = dryice_set_random_key(0);
	if (di_ret != DI_SUCCESS) {
		printk("dryice_set_random_key returned %d\n", di_ret);
		goto out;
	}

	ret = FSL_RETURN_OK_S;

      out:
	return ret;
}

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_read_tamper_event);
#endif
fsl_shw_return_t fsl_shw_read_tamper_event(fsl_shw_uco_t * user_ctx,
					   fsl_shw_tamper_t * tamperp,
					   uint64_t * timestampp)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;
	di_return_t di_ret;
	uint32_t di_events = 0;
	uint32_t di_time_stamp;

	/* Only blocking mode calls are supported */
	if (!(user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	di_ret = dryice_get_tamper_event(&di_events, &di_time_stamp, 0);
	if ((di_ret != DI_SUCCESS) && (di_ret != DI_ERR_STATE)) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("dryice_get_tamper_event returned %s\n",
			      di_error_string(di_ret));
#endif
		goto out;
	}

	/* Pass time back to caller */
	*timestampp = (uint64_t) di_time_stamp;

	if (di_events & DI_TAMPER_EVENT_WTD) {
		*tamperp = FSL_SHW_TAMPER_WTD;
	} else if (di_events & DI_TAMPER_EVENT_ETBD) {
		*tamperp = FSL_SHW_TAMPER_ETBD;
	} else if (di_events & DI_TAMPER_EVENT_ETAD) {
		*tamperp = FSL_SHW_TAMPER_ETAD;
	} else if (di_events & DI_TAMPER_EVENT_EBD) {
		*tamperp = FSL_SHW_TAMPER_EBD;
	} else if (di_events & DI_TAMPER_EVENT_SAD) {
		*tamperp = FSL_SHW_TAMPER_SAD;
	} else if (di_events & DI_TAMPER_EVENT_TTD) {
		*tamperp = FSL_SHW_TAMPER_TTD;
	} else if (di_events & DI_TAMPER_EVENT_CTD) {
		*tamperp = FSL_SHW_TAMPER_CTD;
	} else if (di_events & DI_TAMPER_EVENT_VTD) {
		*tamperp = FSL_SHW_TAMPER_VTD;
	} else if (di_events & DI_TAMPER_EVENT_MCO) {
		*tamperp = FSL_SHW_TAMPER_MCO;
	} else if (di_events & DI_TAMPER_EVENT_TCO) {
		*tamperp = FSL_SHW_TAMPER_TCO;
	} else if (di_events != 0) {
		/* Apparentliy a tamper type not known to this driver was detected */
		goto out;
	} else {
		*tamperp = FSL_SHW_TAMPER_NONE;
	}

	ret = FSL_RETURN_OK_S;

      out:
	return ret;
}				/* end fn fsl_shw_read_tamper_event */
#endif
/*!
 * Convert an SHW HW key reference into a DI driver key reference
 *
 * @param shw_pf_key   An SHW HW key value
 * @param di_keyp      Location to store the equivalent DI driver key
 *
 * @return FSL_RETURN_OK_S, or error if key is unknown or cannot translate.
 */
fsl_shw_return_t shw_convert_pf_key(fsl_shw_pf_key_t shw_pf_key,
				    di_key_t * di_keyp)
{
	fsl_shw_return_t ret = FSL_RETURN_BAD_FLAG_S;

	switch (shw_pf_key) {
	case FSL_SHW_PF_KEY_IIM:
		*di_keyp = DI_KEY_FK;
		break;
	case FSL_SHW_PF_KEY_RND:
		*di_keyp = DI_KEY_RK;
		break;
	case FSL_SHW_PF_KEY_IIM_RND:
		*di_keyp = DI_KEY_FRK;
		break;
	case FSL_SHW_PF_KEY_PRG:
		*di_keyp = DI_KEY_PK;
		break;
	case FSL_SHW_PF_KEY_IIM_PRG:
		*di_keyp = DI_KEY_FPK;
		break;
	default:
		goto out;
	}

	ret = FSL_RETURN_OK_S;

      out:
	return ret;
}

#ifdef DIAG_SECURITY_FUNC
const char *di_error_string(int code)
{
	char *str = "unknown";

	switch (code) {
	case DI_SUCCESS:
		str = "operation was successful";
		break;
	case DI_ERR_BUSY:
		str = "device or resource busy";
		break;
	case DI_ERR_STATE:
		str = "dryice is in incompatible state";
		break;
	case DI_ERR_INUSE:
		str = "resource is already in use";
		break;
	case DI_ERR_UNSET:
		str = "resource has not been initialized";
		break;
	case DI_ERR_WRITE:
		str = "error occurred during register write";
		break;
	case DI_ERR_INVAL:
		str = "invalid argument";
		break;
	case DI_ERR_FAIL:
		str = "operation failed";
		break;
	case DI_ERR_HLOCK:
		str = "resource is hard locked";
		break;
	case DI_ERR_SLOCK:
		str = "resource is soft locked";
		break;
	case DI_ERR_NOMEM:
		str = "out of memory";
		break;
	default:
		break;
	}

	return str;
}
#endif				/* HAVE DRYICE */
