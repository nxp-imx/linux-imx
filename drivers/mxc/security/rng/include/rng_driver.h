/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

#ifndef RNG_DRIVER_H
#define RNG_DRIVER_H

#include "shw_driver.h"

/* This is a Linux flag meaning 'compiling kernel code'...  */
#ifndef __KERNEL__
#include <inttypes.h>
#include <stdlib.h>
#include <memory.h>
#else
#include "../../sahara2/include/portable_os.h"
#endif

#include "../../sahara2/include/fsl_platform.h"

/*! @file rng_driver.h
 *
 * @brief Header file to use the RNG driver.
 *
 *  @ingroup RNG
 */

#if defined(FSL_HAVE_RNGA)

#include "rng_rnga.h"

#elif defined(FSL_HAVE_RNGB) || defined(FSL_HAVE_RNGC)

#include "rng_rngc.h"

#else				/* neither RNGA,  RNGB,  nor  RNGC */

#error NO_RNG_TYPE_IDENTIFIED

#endif

/*****************************************************************************
 * Enumerations
 *****************************************************************************/

/*! Values from Version ID register  */
enum rng_type {
	/*! Type RNGA. */
	RNG_TYPE_RNGA = 0,
	/*! Type RNGB. */
	RNG_TYPE_RNGB = 1,
	/*! Type RNGC */
	RNG_TYPE_RNGC = 2
};

/*!
 * Return values (error codes) for kernel register interface functions
 */
typedef enum rng_return {
	RNG_RET_OK = 0,		/*!< Function succeeded  */
	RNG_RET_FAIL		/*!< Non-specific failure */
} rng_return_t;

/*****************************************************************************
 * Data Structures
 *****************************************************************************/
/*!
 * An entry in the RNG Work Queue.  Based upon standard SHW queue entry.
 *
 * This entry also gets saved (for non-blocking requests) in the user's result
 * pool.  When the user picks up the request, the final processing (copy from
 * data_local to data_user) will get made if status was good.
 */
typedef struct rng_work_entry {
	struct shw_queue_entry_t hdr;	/*!< Standards SHW queue info.  */
	uint32_t length;	/*!< Number of bytes still needed to satisfy request. */
	uint32_t *data_local;	/*!< Where data from RNG FIFO gets placed. */
	uint8_t *data_user;	/*!< Ultimate target of data.  */
	unsigned completed;	/*!< Non-zero if job is done.  */
} rng_work_entry_t;

/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/

#ifdef RNG_REGISTER_PEEK_POKE
/*!
 * Read value from an RNG register.
 * The offset will be checked for validity as well as whether it is
 * accessible at the time of the call.
 *
 * This routine cannot be used to read the RNG's Output FIFO if the RNG is in
 * High Assurance mode.
 *
 * @param[in]   register_offset  The (byte) offset within the RNG block
 *                               of the register to be queried.  See
 *                               RNG(A, C) registers for meanings.
 * @param[out]  value            Pointer to where value from the register
 *                               should be placed.
 *
 * @return                   See #rng_return_t.
 */
/* REQ-FSLSHW-PINTFC-API-LLF-001 */
extern rng_return_t rng_read_register(uint32_t register_offset,
				      uint32_t * value);

/*!
 * Write a new value into an RNG register.
 *
 * The offset will be checked for validity as well as whether it is
 * accessible at the time of the call.
 *
 * @param[in]  register_offset  The (byte) offset within the RNG block
 *                              of the register to be modified.  See
 *                              RNG(A, C) registers for meanings.
 * @param[in]  value            The value to store into the register.
 *
 * @return                   See #rng_return_t.
 */
/* REQ-FSLSHW-PINTFC-API-LLF-002 */
extern rng_return_t rng_write_register(uint32_t register_offset,
				       uint32_t value);
#endif				/* RNG_REGISTER_PEEK_POKE */

#endif				/* RNG_DRIVER_H */
