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
 * @file fsl_shw_rand.c
 *
 * This file implements Random Number Generation functions of the FSL SHW API
 * for Sahara.
 */

#include "sahara.h"
#include "sf_util.h"

#ifdef __KERNEL__
EXPORT_SYMBOL(fsl_shw_get_random);
#endif

/* REQ-S2LRD-PINTFC-API-BASIC-RNG-002 */
/*!
 * Get a random number
 *
 *
 * @param    user_ctx
 * @param    length
 * @param    data
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_get_random(fsl_shw_uco_t * user_ctx,
				    uint32_t length, uint8_t * data)
{
	SAH_SF_DCLS;

	/* perform a sanity check on the uco */
	ret = sah_validate_uco(user_ctx);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	header = SAH_HDR_RNG_GENERATE;	/* Desc. #18 */
	DESC_OUT_OUT(header, length, data, 0, NULL);

	SAH_SF_EXECUTE();

      out:
	SAH_SF_DESC_CLEAN();

	return ret;
}

#ifdef __KERNEL__
EXPORT_SYMBOL(fsl_shw_add_entropy);
#endif

/*!
 * Add entropy to a random number generator

 * @param    user_ctx
 * @param    length
 * @param    data
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_add_entropy(fsl_shw_uco_t * user_ctx,
				     uint32_t length, uint8_t * data)
{
	SAH_SF_DCLS;

	/* perform a sanity check on the uco */
	ret = sah_validate_uco(user_ctx);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	header = SAH_HDR_RNG_GENERATE;	/* Desc. #18 */

	/* create descriptor #18. Generate random data */
	DESC_IN_IN(header, 0, NULL, length, data)

	    SAH_SF_EXECUTE();

      out:
	SAH_SF_DESC_CLEAN();

	return ret;
}
