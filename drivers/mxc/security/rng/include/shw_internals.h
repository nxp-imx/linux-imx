/*
 * Copyright (C) 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef SHW_INTERNALS_H
#define SHW_INTERNALS_H

/*! @file shw_internals.h
 *
 *  This file contains definitions which are internal to the SHW driver.
 *
 *  This header file should only ever be included by shw_driver.c
 *
 *  Compile-time flags minimally needed:
 *
 *  @li Some sort of platform flag.
 *
 */

#include "portable_os.h"
#include "shw_driver.h"

/*! @defgroup shwcompileflags SHW Compile Flags
 *
 * These are flags which are used to configure the SHW driver at compilation
 * time.
 *
 * The terms 'defined' and 'undefined' refer to whether a @c \#define (or -D on
 * a compile command) has defined a given preprocessor symbol.  If a given
 * symbol is defined, then @c \#ifdef \<symbol\> will succeed.  Some symbols
 * described below default to not having a definition, i.e. they are undefined.
 *
 */

/*! @addtogroup shwcompileflags */
/*! @{ */
#ifndef SHW_MAJOR_NODE
/*!
 * This should be configured in a Makefile/compile command line.  It is the
 * value the driver will use to register itself as a device driver for a
 * /dev/node file.  Zero means allow (Linux) to assign a value.  Any positive
 * number will be attempted as the registration value, to allow for
 * coordination with the creation/existence of a /dev/fsl_shw (for instance)
 * file in the filesystem.
 */
#define SHW_MAJOR_NODE 0
#endif

/* Temporarily define compile-time flags to make Doxygen happy and allow them
   to get into the documentation. */
#ifdef DOXYGEN_HACK

/*!
 * Turn on compilation of run-time operational, debug, and error messages.
 *
 * This flag is undefined by default.
 */
/* REQ-FSLSHW-DEBUG-001 */
#define SHW_DEBUG
#undef SHW_DEBUG

/*! @} */
#endif				/* end DOXYGEN_HACK */

#ifndef SHW_DRIVER_NAME
/*! @addtogroup shwcompileflags */
/*! @{ */
/*! Name the driver will use to register itself to the kernel as the driver for
 *  the #shw_major_node and interrupt handling. */
#define SHW_DRIVER_NAME "fsl_shw"
/*! @} */
#endif
/*#define SHW_DEBUG*/

/*!
 * Add a user context onto the list of registered users.
 *
 * Place it at the head of the #user_list queue.
 *
 * @param ctx  A pointer to a user context
 *
 * @return void
 */
inline static void SHW_ADD_USER(fsl_shw_uco_t * ctx)
{
	os_lock_context_t lock_context;

	os_lock_save_context(shw_queue_lock, lock_context);
	ctx->next = user_list;
	user_list = ctx;
	os_unlock_restore_context(shw_queue_lock, lock_context);

}

/*!
 * Remove a user context from the list of registered users.
 *
 * @param ctx  A pointer to a user context
 *
 * @return void
 *
 */
inline static void SHW_REMOVE_USER(fsl_shw_uco_t * ctx)
{
	fsl_shw_uco_t *prev_ctx = user_list;
	os_lock_context_t lock_context;

	os_lock_save_context(shw_queue_lock, lock_context);

	if (prev_ctx == ctx) {
		/* Found at head, so just set new head */
		user_list = ctx->next;
	} else {
		for (; (prev_ctx != NULL); prev_ctx = prev_ctx->next) {
			if (prev_ctx->next == ctx) {
				prev_ctx->next = ctx->next;
				break;
			}
		}
	}
	os_unlock_restore_context(shw_queue_lock, lock_context);
}

static void shw_user_callback(fsl_shw_uco_t * uco);

/* internal functions */
static os_error_code shw_setup_user_driver_interaction(void);
static void shw_cleanup(void);

static os_error_code init_uco(fsl_shw_uco_t * user_ctx, void *user_mode_uco);
static os_error_code get_capabilities(fsl_shw_uco_t * user_ctx,
				      void *user_mode_pco_request);
static os_error_code get_results(fsl_shw_uco_t * user_ctx,
				 void *user_mode_result_req);
static os_error_code get_random(fsl_shw_uco_t * user_ctx,
				void *user_mode_get_random_req);
static os_error_code add_entropy(fsl_shw_uco_t * user_ctx,
				 void *user_mode_add_entropy_req);

void* wire_user_memory(void* address, uint32_t length, void** page_ctx);
void unwire_user_memory(void** page_ctx);
os_error_code map_user_memory(struct vm_area_struct* vma,
                              uint32_t physical_addr, uint32_t size);
os_error_code unmap_user_memory(uint32_t user_addr, uint32_t size);

#if defined(LINUX_VERSION_CODE)

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Device Driver for FSL SHW API");

#endif				/* LINUX_VERSION_CODE */

#endif				/* SHW_INTERNALS_H */
