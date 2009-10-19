/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file fsl_shw_hmac.c
 *
 * This file implements Hashed Message Authentication Code functions of the FSL
 * SHW API.
 */

#include "shw_driver.h"

/* REQ-S2LRD-PINTFC-API-BASIC-HMAC-001 */
/*!
 * Get the precompute information
 *
 *
 * @param   user_ctx
 * @param   key_info
 * @param   hmac_ctx
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_hmac_precompute(fsl_shw_uco_t * user_ctx,
					 fsl_shw_sko_t * key_info,
					 fsl_shw_hmco_t * hmac_ctx)
{
	fsl_shw_return_t status = FSL_RETURN_ERROR_S;

	/* Unused */
	(void)user_ctx;
	(void)key_info;
	(void)hmac_ctx;

	return status;
}

/* REQ-S2LRD-PINTFC-API-BASIC-HMAC-002 */
/*!
 * Get the hmac
 *
 *
 * @param         user_ctx    Info for acquiring memory
 * @param         key_info
 * @param         hmac_ctx
 * @param         msg
 * @param         length
 * @param         result
 * @param         result_len
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_hmac(fsl_shw_uco_t * user_ctx,
			      fsl_shw_sko_t * key_info,
			      fsl_shw_hmco_t * hmac_ctx,
			      const uint8_t * msg,
			      uint32_t length,
			      uint8_t * result, uint32_t result_len)
{
	fsl_shw_return_t status = FSL_RETURN_ERROR_S;

	/* Unused */
	(void)user_ctx;
	(void)key_info;
	(void)hmac_ctx;
	(void)msg;
	(void)length;
	(void)result;
	(void)result_len;

	return status;
}
