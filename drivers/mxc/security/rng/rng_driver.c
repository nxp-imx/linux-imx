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

/*! @file rng_driver.c
 *
 * This is the driver code for the hardware Random Number Generator (RNG).
 *
 * It provides the following functions to callers:
 *   fsl_shw_return_t fsl_shw_get_random(fsl_shw_uco_t* user_ctx,
 *                                       uint32_t length,
 *                                       uint8_t* data);
 *
 *   fsl_shw_return_t fsl_shw_add_entropy(fsl_shw_uco_t* user_ctx,
 *                                        uint32_t length,
 *                                        uint8_t* data);
 *
 * The life of the driver starts at boot (or module load) time, with a call by
 * the kernel to #rng_init().  As part of initialization, a background task
 * running #rng_entropy_task() will be created.
 *
 * The life of the driver ends when the kernel is shutting down (or the driver
 * is being unloaded).  At this time, #rng_shutdown() is called.  No function
 * will ever be called after that point.  In the case that the driver is
 * reloaded, a new copy of the driver, with fresh global values, etc., is
 * loaded, and there will be a new call to #rng_init().
 *
 * A call to fsl_shw_get_random() gets converted into a work entry which is
 * queued and handed off to a background task for fulfilment.  This provides
 * for a single thread of control for reading the RNG's FIFO register, which
 * might otherwise underflow if not carefully managed.
 *
 * A call to fsl_shw_add_entropy() will cause the additional entropy to
 * be passed directly into the hardware.
 *
 * In a debug configuration, it provides the following kernel functions:
 * rng_return_t rng_read_register(uint32_t byte_offset, uint32_t* valuep);
 * rng_return_t rng_write_register(uint32_t byte_offset, uint32_t value);
 *  @ingroup RNG
 */

#include "portable_os.h"
#include "fsl_shw.h"
#include "rng_internals.h"

#ifdef FSL_HAVE_SCC2
#include <linux/mxc_scc2_driver.h>
#else
#include <linux/mxc_scc_driver.h>
#endif

#if defined(RNG_DEBUG) || defined(RNG_ENTROPY_DEBUG) ||                     \
    defined(RNG_REGISTER_DEBUG)
    
#include <diagnostic.h>

#else

#define LOG_KDIAG_ARGS(fmt, ...)
#define LOG_KDIAG(diag)

#endif

/* These are often handy */
#ifndef FALSE
/*! Non-true value for arguments, return values. */
#define FALSE 0
#endif
#ifndef TRUE
/*! True value for arguments, return values. */
#define TRUE 1
#endif

/******************************************************************************
 *
 *  Global / Static Variables
 *
 *****************************************************************************/

/*!
 * This is type void* so that a) it cannot directly be dereferenced, and b)
 * pointer arithmetic on it will function for the byte offsets in rng_rnga.h
 * and rng_rngc.h
 *
 * rng_base is the location in the iomap where the RNG's registers
 * (and memory) start.
 *
 * The referenced data is declared volatile so that the compiler will
 * not make any assumptions about the value of registers in the RNG,
 * and thus will always reload the register into CPU memory before
 * using it (i.e. wherever it is referenced in the driver).
 *
 * This value should only be referenced by the #RNG_READ_REGISTER and
 * #RNG_WRITE_REGISTER macros and their ilk.  All dereferences must be
 * 32 bits wide.
 */
static volatile void *rng_base;

/*!
 * Flag to say whether interrupt handler has been registered for RNG
 * interrupt */
static int rng_irq_set = FALSE;

/*!
 * Size of the RNG's OUTPUT_FIFO, in words.  Retrieved with
 * #RNG_GET_FIFO_SIZE() during driver initialization.
 */
static int rng_output_fifo_size;

/*! Major number for device driver. */
static int rng_major;

/*! Registration handle for registering driver with OS. */
os_driver_reg_t rng_reg_handle;

/*!
 * Internal flag to know whether RNG is in Failed state (and thus many
 * registers are unavailable).  If the value ever goes to #RNG_STATUS_FAILED,
 * it will never change.
 */
static volatile rng_status_t rng_availability = RNG_STATUS_INITIAL;

/*!
 * Global lock for the RNG driver.  Mainly used for entries on the RNG work
 * queue.
 */
static os_lock_t rng_queue_lock = NULL;

/*!
 * Queue for the RNG task to process.
 */
static shw_queue_t rng_work_queue;

/*!
 * Flag to say whether task initialization succeeded.
 */
static unsigned task_started = FALSE;
/*!
 * Waiting queue for RNG SELF TESTING 
 */
static DECLARE_COMPLETION(rng_self_testing);
static DECLARE_COMPLETION(rng_seed_done);
/*!
 *  Object for blocking-mode callers of RNG driver to sleep.
 */
OS_WAIT_OBJECT(rng_wait_queue);

/******************************************************************************
 *
 *  Function Implementations - Externally Accessible
 *
 *****************************************************************************/

/*****************************************************************************/
/* fn rng_init()                                                             */
/*****************************************************************************/
/*!
 * Initialize the driver.
 *
 * Set up the driver to have access to RNG device registers and verify that
 * it appears to be a proper working device.
 *
 * Set up interrupt handling.  Assure RNG is ready to go and (possibly) set it
 * into High Assurance mode.  Create a background task to run
 * #rng_entropy_task().  Set up up a callback with the SCC driver should the
 * security alarm go off.  Tell the kernel that the driver is here.
 *
 * This routine is called during kernel init or module load (insmod).
 *
 * The function will fail in one of two ways: Returning OK to the caller so
 * that kernel startup / driver initialization completes, or returning an
 * error.  In the success case, the function could set the rng_avaailability to
 * RNG_STATUS_FAILED so that only minimal support (e.g. register peek / poke)
 * is available in the driver.
 *
 * @return a call to os_dev_init_return()
 */
OS_DEV_INIT(rng_init)
{
	struct clk *clk;
	os_error_code return_code = OS_ERROR_FAIL_S;
	rng_availability = RNG_STATUS_CHECKING;

#if !defined(FSL_HAVE_RNGA)
	INIT_COMPLETION(rng_self_testing);
	INIT_COMPLETION(rng_seed_done);
#endif
	rng_work_queue.head = NULL;
	rng_work_queue.tail = NULL;

	clk = clk_get(NULL, "rng_clk");

	// Check that the clock was found
	if (IS_ERR(clk)) {
		LOG_KDIAG("RNG: Failed to find rng_clock.");
		return_code = OS_ERROR_FAIL_S;
		goto check_err;
	}

	clk_enable(clk);

	os_printk(KERN_INFO "RNG Driver: Loading\n");

	return_code = rng_map_RNG_memory();
	if (return_code != OS_ERROR_OK_S) {
		rng_availability = RNG_STATUS_UNIMPLEMENTED;
		LOG_KDIAG_ARGS("RNG: Driver failed to map RNG registers. %d",
			       return_code);
		goto check_err;
	}
	LOG_KDIAG_ARGS("RNG Driver: rng_base is 0x%08x", (uint32_t) rng_base);
	/*Check SCC keys are fused */
	if (RNG_HAS_ERROR()) {
		if (RNG_HAS_BAD_KEY()) {
#ifdef RNG_DEBUG
#if !defined(FSL_HAVE_RNGA)
			LOG_KDIAG("ERROR: BAD KEYS SELECTED");
			{
				uint32_t rngc_status =
				    RNG_READ_REGISTER(RNGC_STATUS);
				uint32_t rngc_error =
				    RNG_READ_REGISTER(RNGC_ERROR);
				LOG_KDIAG_ARGS
				    ("status register: %08x, error status: %08x",
				     rngc_status, rngc_error);
			}
#endif
#endif
			rng_availability = RNG_STATUS_FAILED;
			return_code = OS_ERROR_FAIL_S;
			goto check_err;
		}
	}

	/* Check RNG configuration and status */
	return_code = rng_grab_config_values();
	if (return_code != OS_ERROR_OK_S) {
		rng_availability = RNG_STATUS_UNIMPLEMENTED;
		goto check_err;
	}

	/* Masking All Interrupts */
	/* They are unmasked later in rng_setup_interrupt_handling() */
	RNG_MASK_ALL_INTERRUPTS();

	RNG_WAKE();

	/* Determine status of RNG */
	if (RNG_OSCILLATOR_FAILED()) {
		LOG_KDIAG("RNG Driver: RNG Oscillator is dead");
		rng_availability = RNG_STATUS_FAILED;
		goto check_err;
	}

	/* Oscillator not dead.  Setup interrupt code and start the RNG. */
	if ((return_code = rng_setup_interrupt_handling()) == OS_ERROR_OK_S) {
#if defined(FSL_HAVE_RNGA)
		scc_return_t scc_code;
#endif

		RNG_GO();

		/* Self Testing For RNG */
		do {
			RNG_CLEAR_ERR();

			/* wait for Clearing Erring finished */
			msleep(1);

			RNG_UNMASK_ALL_INTERRUPTS();
			RNG_SELF_TEST();
#if !defined(FSL_HAVE_RNGA)
			wait_for_completion(&rng_self_testing);
#endif
		} while (RNG_CHECK_SELF_ERR());

		RNG_CLEAR_ALL_STATUS();
		/* checking for RNG SEED done */
		do {
			RNG_CLEAR_ERR();
			RNG_SEED_GEN();
#if !defined(FSL_HAVE_RNGA)
			wait_for_completion(&rng_seed_done);
#endif
		} while (RNG_CHECK_SEED_ERR());
#ifndef RNG_NO_FORCE_HIGH_ASSURANCE
		RNG_SET_HIGH_ASSURANCE();
#endif
		if (RNG_GET_HIGH_ASSURANCE()) {
			LOG_KDIAG("RNG Driver: RNG is in High Assurance mode");
		} else {
#ifndef RNG_NO_FORCE_HIGH_ASSURANCE
			LOG_KDIAG
			    ("RNG Driver: RNG could not be put in High Assurance mode");
			rng_availability = RNG_STATUS_FAILED;
			goto check_err;
#endif				/* RNG_NO_FORCE_HIGH_ASSURANCE */
		}

		/* Check that RNG is OK */
		if (!RNG_WORKING()) {
			LOG_KDIAG_ARGS
			    ("RNG determined to be inoperable.  Status %08x",
			     RNG_GET_STATUS());
			/* Couldn't wake it up or other problem */
			rng_availability = RNG_STATUS_FAILED;
			goto check_err;
		}

		rng_queue_lock = os_lock_alloc_init();
		if (rng_queue_lock == NULL) {
			LOG_KDIAG("RNG: lock initialization failed");
			rng_availability = RNG_STATUS_FAILED;
			goto check_err;
		}

		return_code = os_create_task(rng_entropy_task);
		if (return_code != OS_ERROR_OK_S) {
			LOG_KDIAG("RNG: task initialization failed");
			rng_availability = RNG_STATUS_FAILED;
			goto check_err;
		} else {
			task_started = TRUE;
		}
#ifdef FSL_HAVE_RNGA
		scc_code = scc_monitor_security_failure(rng_sec_failure);
		if (scc_code != SCC_RET_OK) {
			LOG_KDIAG_ARGS("Failed to register SCC callback: %d",
				       scc_code);
#ifndef RNG_NO_FORCE_HIGH_ASSURANCE
			return_code = OS_ERROR_FAIL_S;
			goto check_err;
#endif
		}
#endif				/* FSL_HAVE_RNGA */
		return_code = os_driver_init_registration(rng_reg_handle);
		if (return_code != OS_ERROR_OK_S) {
			goto check_err;
		}
		/* add power suspend here */
		/* add power resume here */
		return_code =
		    os_driver_complete_registration(rng_reg_handle,
						    rng_major, RNG_DRIVER_NAME);
	}
	/* RNG is working */

      check_err:

	/* If FIFO underflow or other error occurred during drain, this will fail,
	 * as system will have been put into fail mode by SCC. */
	if ((return_code == OS_ERROR_OK_S)
	    && (rng_availability == RNG_STATUS_CHECKING)) {
		RNG_PUT_RNG_TO_SLEEP();
		rng_availability = RNG_STATUS_OK;	/* RNG & driver are ready */
	} else if (return_code != OS_ERROR_OK_S) {
		os_printk(KERN_ALERT "Driver initialization failed. %d",
			  return_code);
		rng_cleanup();
	}

	os_dev_init_return(return_code);

}				/* rng_init */

/*****************************************************************************/
/* fn rng_shutdown()                                                         */
/*****************************************************************************/
/*!
 * Prepare driver for exit.
 *
 * This is called during @c rmmod when the driver is unloading.
 * Try to undo whatever was done during #rng_init(), to make the machine be
 * in the same state, if possible.
 *
 * Calls rng_cleanup() to do all work, and then unmap device's register space.
 */
OS_DEV_SHUTDOWN(rng_shutdown)
{
	LOG_KDIAG("shutdown called");

	rng_cleanup();

	os_driver_remove_registration(rng_reg_handle);
	if (rng_base != NULL) {
		/* release the virtual memory map to the RNG */
		os_unmap_device((void *)rng_base, RNG_ADDRESS_RANGE);
		rng_base = NULL;
	}

	os_dev_shutdown_return(OS_ERROR_OK_S);
}				/* rng_shutdown */

/*****************************************************************************/
/* fn rng_cleanup()                                                          */
/*****************************************************************************/
/*!
 * Undo everything done by rng_init() and place driver in fail mode.
 *
 * Deregister from SCC, stop tasklet, shutdown the RNG.  Leave the register
 * map in place in case other drivers call rng_read/write_register()
 *
 * @return void
 */
static void rng_cleanup(void)
{
	struct clk *clk;

#ifdef FSL_HAVE_RNGA
	scc_stop_monitoring_security_failure(rng_sec_failure);
#endif

	clk = clk_get(NULL, "rng_clk");
	clk_disable(clk);
	if (task_started) {
		os_dev_stop_task(rng_entropy_task);
	}

	if (rng_base != NULL) {
		/* mask off RNG interrupts */
		RNG_MASK_ALL_INTERRUPTS();
		RNG_SLEEP();

		if (rng_irq_set) {
			/* unmap the interrupts from the IRQ lines */
			os_deregister_interrupt(INT_RNG);
			rng_irq_set = FALSE;
		}
		LOG_KDIAG("Leaving rng driver status as failed");
		rng_availability = RNG_STATUS_FAILED;
	} else {
		LOG_KDIAG("Leaving rng driver status as unimplemented");
		rng_availability = RNG_STATUS_UNIMPLEMENTED;
	}
	LOG_KDIAG("Cleaned up");
}				/* rng_cleanup */

/*!
 * Post-process routine for fsl_shw_get_random().
 *
 * This function will copy the random data generated by the background task
 * into the user's buffer and then free the local buffer.
 *
 * @param gen_entry   The work request.
 *
 * @return 0 = meaning work completed, pass back result.
 */
static uint32_t finish_random(shw_queue_entry_t * gen_entry)
{
	rng_work_entry_t *work = (rng_work_entry_t *) gen_entry;

	if (work->hdr.flags & FSL_UCO_USERMODE_USER) {
		os_copy_to_user(work->data_user, work->data_local,
				work->length);
	} else {
		memcpy(work->data_user, work->data_local, work->length);
	}

	os_free_memory(work->data_local);
	work->data_local = NULL;

	return 0;		/* means completed. */
}

/* REQ-FSLSHW-PINTFC-API-BASIC-RNG-002 */
/*****************************************************************************/
/* fn fsl_shw_get_random()                                                   */
/*****************************************************************************/
/*!
 * Get random data.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param      length    The number of octets of @a data being requested.
 * @param data      A pointer to a location of @a length octets to where
 *                       random data will be returned.
 *
 * @return     FSL_RETURN_NO_RESOURCE_S  A return code of type #fsl_shw_return_t.
 *             FSL_RETURN_OK_S 
 */
fsl_shw_return_t fsl_shw_get_random(fsl_shw_uco_t * user_ctx, uint32_t length,
				    uint8_t * data)
{
	fsl_shw_return_t return_code = FSL_RETURN_NO_RESOURCE_S;
	/* Boost up length to cover any 'missing' bytes at end of a word */
	uint32_t *buf = os_alloc_memory(length + 3, 0);
	volatile rng_work_entry_t *work = os_alloc_memory(sizeof(*work), 0);

	if ((rng_availability != RNG_STATUS_OK) || (buf == NULL)
	    || (work == NULL)) {
		if (rng_availability != RNG_STATUS_OK) {
			LOG_KDIAG_ARGS("rng not available: %d\n",
				       rng_availability);
		} else {
			LOG_KDIAG_ARGS
			    ("Resource allocation failure: %d or %d bytes",
			     length, sizeof(*work));
		}
		/* Cannot perform function.  Clean up and clear out. */
		if (buf != NULL) {
			os_free_memory(buf);
		}
		if (work != NULL) {
			os_free_memory((void *)work);
		}
	} else {
		unsigned blocking = user_ctx->flags & FSL_UCO_BLOCKING_MODE;

		work->hdr.user_ctx = user_ctx;
		work->hdr.flags = user_ctx->flags;
		work->hdr.callback = user_ctx->callback;
		work->hdr.user_ref = user_ctx->user_ref;
		work->hdr.postprocess = finish_random;
		work->length = length;
		work->data_local = buf;
		work->data_user = data;

		RNG_ADD_WORK_ENTRY((rng_work_entry_t *) work);

		if (blocking) {
			os_sleep(rng_wait_queue, work->completed != FALSE,
				 FALSE);
			finish_random((shw_queue_entry_t *) work);
			return_code = work->hdr.code;
			os_free_memory((void *)work);
		} else {
			return_code = FSL_RETURN_OK_S;
		}
	}

	return return_code;
}				/* fsl_shw_get_entropy */

/*****************************************************************************/
/* fn fsl_shw_add_entropy()                                                  */
/*****************************************************************************/
/*!
 * Add entropy to random number generator.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param      length    Number of bytes at @a data.
 * @param      data      Entropy to add to random number generator.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_add_entropy(fsl_shw_uco_t * user_ctx, uint32_t length,
				     uint8_t * data)
{
	fsl_shw_return_t return_code = FSL_RETURN_NO_RESOURCE_S;
#if defined(FSL_HAVE_RNGC)
	/* No Entropy Register in RNGC */
	return_code = FSL_RETURN_OK_S;
#else
	uint32_t *local_data = NULL;
	if (rng_availability == RNG_STATUS_OK) {
		/* make 32-bit aligned place to hold data */
		local_data = os_alloc_memory(length + 3, 0);
		if (local_data == NULL) {
			return_code = FSL_RETURN_NO_RESOURCE_S;
		} else {
			memcpy(local_data, data, length);

			/* Copy one word at a time to hardware */
			while (TRUE) {
				register uint32_t *ptr = local_data;

				RNG_ADD_ENTROPY(*ptr++);
				if (length <= 4) {
					break;
				}
				length -= 4;
			}
			return_code = FSL_RETURN_OK_S;
			os_free_memory(local_data);
		}		/* else local_data not NULL */

	}
#endif
	/* rng_availability is OK */
	return return_code;
}				/* fsl_shw_add_entropy */

#ifdef RNG_REGISTER_PEEK_POKE
/*****************************************************************************/
/* fn rng_read_register()                                                   */
/*****************************************************************************/
/*
 * Debug routines to allow reading of RNG registers.
 *
 * This routine is only for accesses by other than this driver.
 *
 * @param register_offset  The byte offset of the register to be read.
 * @param value            Pointer to store the value of the register.
 *
 * @return RNG_RET_OK or an error return.
 */
rng_return_t rng_read_register(uint32_t register_offset, uint32_t * value)
{
	rng_return_t return_code = RNG_RET_FAIL;

	if ((rng_availability == RNG_STATUS_OK)
	    || (rng_availability == RNG_STATUS_FAILED)) {
		if ((rng_check_register_offset(register_offset)
		     && rng_check_register_accessible(register_offset,
						      RNG_CHECK_READ))) {
			/* The guards let the check through */
			*value = RNG_READ_REGISTER(register_offset);
			return_code = RNG_RET_OK;
		}
	}

	return return_code;
}				/* rng_read_register */

/*****************************************************************************/
/* fn rng_write_register()                                                  */
/*****************************************************************************/
/*
 * Debug routines to allow writing of RNG registers.
 *
 * This routine is only for accesses by other than this driver.
 *
 * @param register_offset  The byte offset of the register to be written.
 * @param value            Value to store in the register.
 *
 * @return RNG_RET_OK or an error return.
 */
rng_return_t rng_write_register(uint32_t register_offset, uint32_t value)
{
	rng_return_t return_code = RNG_RET_FAIL;

	if ((rng_availability == RNG_STATUS_OK)
	    || (rng_availability == RNG_STATUS_FAILED)) {
		if ((rng_check_register_offset(register_offset)
		     && rng_check_register_accessible(register_offset,
						      RNG_CHECK_WRITE))) {
			RNG_WRITE_REGISTER(register_offset, value);
			return_code = RNG_RET_OK;
		}
	}

	return return_code;
}				/* rng_write_register */
#endif				/*  RNG_REGISTER_PEEK_POKE */

/******************************************************************************
 *
 *  Function Implementations - Internal
 *
 *****************************************************************************/

#ifdef RNG_REGISTER_PEEK_POKE
/*****************************************************************************/
/* fn check_register_offset()                                                */
/*****************************************************************************/
/*!
 * Verify that the @c offset is appropriate for the RNG's register set.
 *
 * @param[in]  offset  The (byte) offset within the RNG block
 *                     of the register to be accessed.  See
 *                     RNG(A, C) register definitions for meanings.
 *
 * This routine is only for checking accesses by other than this driver.
 *
 * @return  0 if register offset out of bounds, 1 if ok to use
 */
inline int rng_check_register_offset(uint32_t offset)
{
	int return_code = FALSE;	/* invalid */

	/* Make sure offset isn't too high and also that it is aligned to
	 * aa 32-bit offset (multiple of four).
	 */
	if ((offset < RNG_ADDRESS_RANGE) && (offset % sizeof(uint32_t) == 0)) {
		return_code = TRUE;	/* OK */
	} else {
		pr_debug("RNG: Denied access to offset %8x\n", offset);
	}

	return return_code;

}				/* rng_check_register */

/*****************************************************************************/
/* fn check_register_accessible()                                            */
/*****************************************************************************/
/*!
 * Make sure that register access is legal.
 *
 * Verify that, if in secure mode, only safe registers are used.
 * For any register access, make sure that read-only registers are not written
 * and that write-only registers are not read.  This check also disallows any
 * access to the RNG's Output FIFO, to prevent other drivers from draining the
 * FIFO and causing an underflow condition.
 *
 * This routine is only for checking accesses by other than this driver.
 *
 * @param  offset   The (byte) offset within the RNG block
 *                      of the register to be accessed.  See
 *                      @ref rngregs for meanings.
 * @param access_write  0 for read, anything else for write
 *
 * @return 0 if invalid, 1 if OK.
 */
static int rng_check_register_accessible(uint32_t offset, int access_write)
{
	int return_code = FALSE;	/* invalid */
	uint32_t secure = RNG_GET_HIGH_ASSURANCE();

	/* First check for RNG in Secure Mode -- most registers inaccessible.
	 * Also disallowing access to RNG_OUTPUT_FIFO except by the driver.
	 */
	if (!
#ifdef FSL_HAVE_RNGA
	    (secure &&
	     ((offset == RNGA_OUTPUT_FIFO) ||
	      (offset == RNGA_MODE) ||
	      (offset == RNGA_VERIFICATION_CONTROL) ||
	      (offset == RNGA_OSCILLATOR_CONTROL_COUNTER) ||
	      (offset == RNGA_OSCILLATOR1_COUNTER) ||
	      (offset == RNGA_OSCILLATOR2_COUNTER) ||
	      (offset == RNGA_OSCILLATOR_COUNTER_STATUS)))
#else				/* RNGB or RNGC */
	    (secure &&
	     ((offset == RNGC_FIFO) ||
	      (offset == RNGC_VERIFICATION_CONTROL) ||
	      (offset == RNGC_OSC_COUNTER_CONTROL) ||
	      (offset == RNGC_OSC_COUNTER) ||
	      (offset == RNGC_OSC_COUNTER_STATUS)))
#endif
	    ) {

		/*  Passed that test.  Either not in high assurance, and/or are
		   checking register that is always available.  Now check
		   R/W permissions. */
		if (access_write == RNG_CHECK_READ) {	/* read request */
			/* Only the entropy register is write-only */
#ifdef FSL_HAVE_RNGC
			/* No registers are write-only */
			return_code = TRUE;
#else				/* else RNGA or RNGB */
#ifdef FSL_HAVE_RNGA
			if (1) {
#else
			if (!(offset == RNGB_ENTROPY)) {
#endif
				return_code = TRUE;	/* Let all others be read */
			} else {
				pr_debug
				    ("RNG: Offset %04x denied read access\n",
				     offset);
			}
#endif				/* RNGA or RNGB */
		} /* read */
		else {		/* access_write means write */
			/* Check against list of non-writable registers */
			if (!
#ifdef FSL_HAVE_RNGA
			    ((offset == RNGA_STATUS) ||
			     (offset == RNGA_OUTPUT_FIFO) ||
			     (offset == RNGA_OSCILLATOR1_COUNTER) ||
			     (offset == RNGA_OSCILLATOR2_COUNTER) ||
			     (offset == RNGA_OSCILLATOR_COUNTER_STATUS))
#else				/* FSL_HAVE_RNGB or FSL_HAVE_RNGC */
			    ((offset == RNGC_STATUS) ||
			     (offset == RNGC_FIFO) ||
			     (offset == RNGC_OSC_COUNTER) ||
			     (offset == RNGC_OSC_COUNTER_STATUS))
#endif
			    ) {
				return_code = TRUE;	/* can be written */
			} else {
				LOG_KDIAG_ARGS
				    ("Offset %04x denied write access", offset);
			}
		}		/* write */
	} /* not high assurance and inaccessible register... */
	else {
		LOG_KDIAG_ARGS("Offset %04x denied high-assurance access",
			       offset);
	}

	return return_code;
}				/* rng_check_register_accessible */
#endif				/*  RNG_REGISTER_PEEK_POKE */

/*****************************************************************************/
/* fn rng_irq()                                                             */
/*****************************************************************************/
/*!
 * This is the interrupt handler for the RNG.  It is only ever invoked if the
 * RNG detects a FIFO Underflow error.
 *
 * If the error is a Security Violation, this routine will
 * set the #rng_availability to #RNG_STATUS_FAILED, as the entropy pool may
 * have been corrupted.  The RNG will also be placed into low power mode.  The
 * SCC will have noticed the problem as well.
 *
 * The other possibility, if the RNG is not in High Assurance mode, would be
 * simply a FIFO Underflow.  No special action, other than to
 * clear the interrupt, is taken.
 */
OS_DEV_ISR(rng_irq)
{
	int handled = FALSE;	/* assume interrupt isn't from RNG */

	LOG_KDIAG("rng irq!");

	if (RNG_SEED_DONE()) {
		complete(&rng_seed_done);
		RNG_CLEAR_ALL_STATUS();
		handled = TRUE;
	}

	if (RNG_SELF_TEST_DONE()) {
		complete(&rng_self_testing);
		RNG_CLEAR_ALL_STATUS();
		handled = TRUE;
	}
	/* Look to see whether RNG needs attention */
	if (RNG_HAS_ERROR()) {
		if (RNG_GET_HIGH_ASSURANCE()) {
			RNG_SLEEP();
			rng_availability = RNG_STATUS_FAILED;
			RNG_MASK_ALL_INTERRUPTS();
		}
		handled = TRUE;
		/* Clear the interrupt */
		RNG_CLEAR_ALL_STATUS();

	}
	os_dev_isr_return(handled);
}				/* rng_irq */

/*****************************************************************************/
/* fn map_RNG_memory()                                                      */
/*****************************************************************************/
/*!
 * Place the RNG's memory into kernel virtual space.
 *
 * @return OS_ERROR_OK_S on success, os_error_code on failure
 */
static os_error_code rng_map_RNG_memory(void)
{
	os_error_code error_code = OS_ERROR_FAIL_S;

	rng_base = os_map_device(RNG_BASE_ADDR, RNG_ADDRESS_RANGE);
	if (rng_base == NULL) {
		/* failure ! */
		LOG_KDIAG("RNG Driver: ioremap failed.");
	} else {
		error_code = OS_ERROR_OK_S;
	}

	return error_code;
}				/* rng_map_RNG_memory */

/*****************************************************************************/
/* fn rng_setup_interrupt_handling()                                        */
/*****************************************************************************/
/*!
 * Register #rng_irq() as the interrupt handler for #INT_RNG.
 *
 * @return OS_ERROR_OK_S on success, os_error_code on failure
 */
static os_error_code rng_setup_interrupt_handling(void)
{
	os_error_code error_code;

	/*
	 * Install interrupt service routine for the RNG. Ignore the
	 * assigned IRQ number.
	 */
	error_code = os_register_interrupt(RNG_DRIVER_NAME, INT_RNG,
					   OS_DEV_ISR_REF(rng_irq));
	if (error_code != OS_ERROR_OK_S) {
		LOG_KDIAG("RNG Driver: Error installing Interrupt Handler");
	} else {
		rng_irq_set = TRUE;
		RNG_UNMASK_ALL_INTERRUPTS();
	}

	return error_code;
}				/* rng_setup_interrupt_handling */

/*****************************************************************************/
/* fn rng_grab_config_values()                                               */
/*****************************************************************************/
/*!
 * Read configuration information from the RNG.
 *
 * Sets #rng_output_fifo_size.
 *
 * @return  A error code indicating whether the part is the expected one.
 */
static os_error_code rng_grab_config_values(void)
{
	enum rng_type type;
	os_error_code ret = OS_ERROR_FAIL_S;

	/* Go for type, versions... */
	type = RNG_GET_RNG_TYPE();

	/* Make sure type is the one this code has been compiled for. */
	if (RNG_VERIFY_TYPE(type)) {
		rng_output_fifo_size = RNG_GET_FIFO_SIZE();
		if (rng_output_fifo_size != 0) {
			ret = OS_ERROR_OK_S;
		}
	}
	if (ret != OS_ERROR_OK_S) {
		LOG_KDIAG_ARGS
		    ("Unknown or unexpected RNG type %d (FIFO size %d)."
		     "  Failing driver initialization", type,
		     rng_output_fifo_size);
	}

	return ret;
}

 /* rng_grab_config_values */

/*****************************************************************************/
/* fn rng_drain_fifo()                                                       */
/*****************************************************************************/
/*!
 * This function copies words from the RNG FIFO into the caller's buffer.
 *
 *
 * @param random_p    Location to copy random data
 * @param count_words Number of words to copy
 *
 * @return An error code.
 */
static fsl_shw_return_t rng_drain_fifo(uint32_t * random_p, int count_words)
{

	int words_in_rng;	/* Number of words available now in RNG */
	fsl_shw_return_t code = FSL_RETURN_ERROR_S;
	int sequential_count = 0;	/* times through big while w/empty FIFO */
	int fifo_empty_count = 0;	/* number of times FIFO was empty */
	int max_sequential = 0;	/* max times 0 seen in a row */
#if !defined(FSL_HAVE_RNGA)
	int count_for_reseed = 0;
	INIT_COMPLETION(rng_seed_done);
#endif
#if !defined(FSL_HAVE_RNGA)
	if (RNG_RESEED()) {
		do {
			LOG_KDIAG("Reseeding RNG");

			RNG_CLEAR_ERR();
			RNG_SEED_GEN();
			wait_for_completion(&rng_seed_done);
			if (count_for_reseed == 3) {
				os_printk(KERN_ALERT
					  "Device was not able to enter RESEED Mode\n");
				code = FSL_RETURN_INTERNAL_ERROR_S;
			}
			count_for_reseed++;
		} while (RNG_CHECK_SEED_ERR());
	}
#endif
	/* Copy all of them in.  Stop if pool fills. */
	while ((rng_availability == RNG_STATUS_OK) && (count_words > 0)) {
		/* Ask RNG how many words currently in FIFO */
		words_in_rng = RNG_GET_WORDS_IN_FIFO();
		if (words_in_rng == 0) {
			++sequential_count;
			fifo_empty_count++;
			if (sequential_count > max_sequential) {
				max_sequential = sequential_count;
			}
			if (sequential_count >= RNG_MAX_TRIES) {
				LOG_KDIAG_ARGS("FIFO staying empty (%d)",
					       words_in_rng);
				code = FSL_RETURN_NO_RESOURCE_S;
				break;
			}
		} else {
			/* Found at least one word */
			sequential_count = 0;
			/* Now adjust: words_in_rng = MAX(count_words, words_in_rng) */
			words_in_rng = (count_words < words_in_rng)
			    ? count_words : words_in_rng;
		}		/* else found words */

#ifdef RNG_FORCE_FIFO_UNDERFLOW
		/*
		 * For unit test, force occasional extraction of more words than
		 * available.  This should cause FIFO Underflow, and IRQ invocation.
		 */
		words_in_rng = count_words;
#endif

		/* Copy out all available & neeeded data */
		while (words_in_rng-- > 0) {
			*random_p++ = RNG_READ_FIFO();
			count_words--;
		}
	}			/* while words still needed */

	if (count_words == 0) {
		code = FSL_RETURN_OK_S;
	}
	if (fifo_empty_count != 0) {
		LOG_KDIAG_ARGS("FIFO empty %d times, max loop count %d",
			       fifo_empty_count, max_sequential);
	}

	return code;
}				/* rng_drain_fifo */

/*****************************************************************************/
/* fn rng_entropy_task()                                                     */
/*****************************************************************************/
/*!
 * This is the background task of the driver.  It is scheduled by
 * RNG_ADD_WORK_ENTRY().
 *
 * This will process each entry on the #rng_work_queue.  Blocking requests will
 * cause sleepers to be awoken.  Non-blocking requests will be placed on the
 * results queue, and if appropriate, the callback function will be invoked.
 */
OS_DEV_TASK(rng_entropy_task)
{
	rng_work_entry_t *work;

	os_dev_task_begin();

#ifdef RNG_ENTROPY_DEBUG
	LOG_KDIAG("entropy task starting");
#endif

	while ((work = RNG_GET_WORK_ENTRY()) != NULL) {
#ifdef RNG_ENTROPY_DEBUG
		LOG_KDIAG_ARGS("found %d bytes of work at %p (%p)",
			       work->length, work, work->data_local);
#endif
		work->hdr.code = rng_drain_fifo(work->data_local,
						BYTES_TO_WORDS(work->length));
		work->completed = TRUE;

		if (work->hdr.flags & FSL_UCO_BLOCKING_MODE) {
#ifdef RNG_ENTROPY_DEBUG
			LOG_KDIAG("Waking queued processes");
#endif
			os_wake_sleepers(rng_wait_queue);
		} else {
			os_lock_context_t lock_context;

			os_lock_save_context(rng_queue_lock, lock_context);
			RNG_ADD_QUEUE_ENTRY(&work->hdr.user_ctx->result_pool,
					    work);
			os_unlock_restore_context(rng_queue_lock, lock_context);

			if (work->hdr.flags & FSL_UCO_CALLBACK_MODE) {
				if (work->hdr.callback != NULL) {
					work->hdr.callback(work->hdr.user_ctx);
				} else {
#ifdef RNG_ENTROPY_DEBUG
					LOG_KDIAG_ARGS
					    ("Callback ptr for %p is NULL",
					     work);
#endif
				}
			}
		}
	}			/* while */

#ifdef RNG_ENTROPY_DEBUG
	LOG_KDIAG("entropy task ending");
#endif

	os_dev_task_return(OS_ERROR_OK_S);
}				/* rng_entropy_task */

#ifdef FSL_HAVE_RNGA
/*****************************************************************************/
/* fn rng_sec_failure()                                                      */
/*****************************************************************************/
/*!
 * Function to handle "Security Alarm" indication from SCC.
 *
 * This function is registered with the Security Monitor ans the callback
 * function for the RNG driver.  Upon alarm, it will shut down the driver so
 * that no more random data can be retrieved.
 *
 * @return void
 */
static void rng_sec_failure(void)
{
	os_printk(KERN_ALERT "RNG Driver: Security Failure Alarm received.\n");

	rng_cleanup();

	return;
}
#endif

#ifdef RNG_REGISTER_DEBUG
/*****************************************************************************/
/* fn dbg_rng_read_register()                                                */
/*****************************************************************************/
/*!
 * Noisily read a 32-bit value to an RNG register.
 * @param offset        The address of the register to read.
 *
 * @return  The register value
 * */
static uint32_t dbg_rng_read_register(uint32_t offset)
{
	uint32_t value;

	value = os_read32(rng_base + offset);
#ifndef RNG_ENTROPY_DEBUG
	if (offset != RNG_OUTPUT_FIFO) {
#endif
		pr_debug("RNG RD: 0x%4x : 0x%08x\n", offset, value);
#ifndef RNG_ENTROPY_DEBUG
	}
#endif
	return value;
}

/*****************************************************************************/
/* fn dbg_rng_write_register()                                               */
/*****************************************************************************/
/*!
 * Noisily write a 32-bit value to an RNG register.
 * @param offset        The address of the register to written.
 *
 * @param value         The new register value
 */
static void dbg_rng_write_register(uint32_t offset, uint32_t value)
{
	LOG_KDIAG_ARGS("WR: 0x%4x : 0x%08x", offset, value);
	os_write32(value, rng_base + offset);
	return;
}

#endif				/* RNG_REGISTER_DEBUG */
