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
 * @file fsl_shw_rand.c
 *
 * This file implements Random Number Generation functions of the FSL SHW API
 * in USER MODE for talking to a standalone RNGA/RNGC device driver.
 *
 * It contains the fsl_shw_get_random() and fsl_shw_add_entropy() functions.
 *
 * These routines will build a request block and pass it to the SHW driver.
 */

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>

#ifdef FSL_DEBUG
#include <stdio.h>
#include <errno.h>
#include <string.h>
#endif				/* FSL_DEBUG */

#include "shw_driver.h"

extern fsl_shw_return_t validate_uco(fsl_shw_uco_t * uco);

#if defined(FSL_HAVE_RNGA) || defined(FSL_HAVE_RNGB) || defined(FSL_HAVE_RNGC)

/* REQ-S2LRD-PINTFC-API-BASIC-RNG-002 */
fsl_shw_return_t fsl_shw_get_random(fsl_shw_uco_t * user_ctx,
				    uint32_t length, uint8_t * data)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

	/* perform a sanity check / update uco */
	ret = validate_uco(user_ctx);
	if (ret == FSL_RETURN_OK_S) {
		struct get_random_req *req = malloc(sizeof(*req));

		if (req == NULL) {
			ret = FSL_RETURN_NO_RESOURCE_S;
		} else {

			init_req(&req->hdr, user_ctx);
			req->size = length;
			req->random = data;

			ret =
			    send_req(SHW_USER_REQ_GET_RANDOM, &req->hdr,
				     user_ctx);
		}
	}

	return ret;
}

fsl_shw_return_t fsl_shw_add_entropy(fsl_shw_uco_t * user_ctx,
				     uint32_t length, uint8_t * data)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

	/* perform a sanity check on the uco */
	ret = validate_uco(user_ctx);
	if (ret == FSL_RETURN_OK_S) {
		struct add_entropy_req *req = malloc(sizeof(*req));

		if (req == NULL) {
			ret = FSL_RETURN_NO_RESOURCE_S;
		} else {
			init_req(&req->hdr, user_ctx);
			req->size = length;
			req->entropy = data;

			ret =
			    send_req(SHW_USER_REQ_ADD_ENTROPY, &req->hdr,
				     user_ctx);
		}
	}

	return ret;
}

#else				/* no H/W RNG block */

fsl_shw_return_t fsl_shw_get_random(fsl_shw_uco_t * user_ctx,
				    uint32_t length, uint8_t * data)
{

	(void)user_ctx;
	(void)length;
	(void)data;

	return FSL_RETURN_ERROR_S;
}

fsl_shw_return_t fsl_shw_add_entropy(fsl_shw_uco_t * user_ctx,
				     uint32_t length, uint8_t * data)
{

	(void)user_ctx;
	(void)length;
	(void)data;

	return FSL_RETURN_ERROR_S;
}
#endif
