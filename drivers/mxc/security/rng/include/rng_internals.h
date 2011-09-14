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

#ifndef RNG_INTERNALS_H
#define RNG_INTERNALS_H

/*! @file rng_internals.h
 *
 *  This file contains definitions which are internal to the RNG driver.
 *
 *  This header file should only ever be needed by rng_driver.c
 *
 *  Compile-time flags minimally needed:
 *
 *  @li Some sort of platform flag.  (FSL_HAVE_RNGA or FSL_HAVE_RNGC)
 *
 *  @ingroup RNG
 */

#include "portable_os.h"
#include "shw_driver.h"
#include "rng_driver.h"

/*! @defgroup rngcompileflags RNG Compile Flags
 *
 * These are flags which are used to configure the RNG driver at compilation
 * time.
 *
 * Most of them default to good values for normal operation, but some
 * (#INT_RNG and #RNG_BASE_ADDR) need to be provided.
 *
 * The terms 'defined' and 'undefined' refer to whether a @c \#define (or -D on
 * a compile command) has defined a given preprocessor symbol.  If a given
 * symbol is defined, then @c \#ifdef \<symbol\> will succeed.  Some symbols
 * described below default to not having a definition, i.e. they are undefined.
 *
 */

/*! @addtogroup rngcompileflags */
/*! @{ */

/*!
 * This is the maximum number of times the driver will loop waiting for the
 * RNG hardware to say that it has generated random data.  It prevents the
 * driver from stalling forever should there be a hardware problem.
 *
 * Default value is 100.  It should be revisited as CPU clocks speed up.
 */
#ifndef RNG_MAX_TRIES
#define RNG_MAX_TRIES 100
#endif

/* Temporarily define compile-time flags to make Doxygen happy and allow them
   to get into the documentation. */
#ifdef DOXYGEN_HACK

/*!
 * This symbol is the base address of the RNG in the CPU memory map.  It may
 * come from some included header file, or it may come from the compile command
 * line.  This symbol has no default, and the driver will not compile without
 * it.
 */
#define RNG_BASE_ADDR
#undef RNG_BASE_ADDR

/*!
 * This symbol is the Interrupt Number of the RNG in the CPU.  It may come
 * from some included header file, or it may come from the compile command
 * line.  This symbol has no default, and the driver will not compile without
 * it.
 */
#define INT_RNG
#undef INT_RNG

/*!
 * Defining this symbol will allow other kernel programs to call the
 * #rng_read_register() and #rng_write_register() functions.  If this symbol is
 * not defined, those functions will not be present in the driver.
 */
#define RNG_REGISTER_PEEK_POKE
#undef RNG_REGISTER_PEEK_POKE

/*!
 * Turn on compilation of run-time operational, debug, and error messages.
 *
 * This flag is undefined by default.
 */
/* REQ-FSLSHW-DEBUG-001 */

/*!
 * Turn on compilation of run-time logging of access to the RNG registers,
 * except for the RNG's Output FIFO register.  See #RNG_ENTROPY_DEBUG.
 *
 *  This flag is undefined by default
 */
#define RNG_REGISTER_DEBUG
#undef  RNG_REGISTER_DEBUG

/*!
 * Turn on compilation of run-time logging of reading of the RNG's Output FIFO
 * register.  This flag does nothing if #RNG_REGISTER_DEBUG is not defined.
 *
 *  This flag is undefined by default
 */
#define RNG_ENTROPY_DEBUG
#undef  RNG_ENTROPY_DEBUG

/*!
 * If this flag is defined, the driver will not attempt to put the RNG into
 * High Assurance mode.

 * If it is undefined, the driver will attempt to put the RNG into High
 * Assurance mode.  If RNG fails to go into High Assurance mode, the driver
 * will fail to initialize.

 * In either case, if the RNG is already in this mode, the driver will operate
 * normally.
 *
 *  This flag is undefined by default.
 */
#define RNG_NO_FORCE_HIGH_ASSURANCE
#undef RNG_NO_FORCE_HIGH_ASSURANCE

/*!
 * If this flag is defined, the driver will put the RNG into low power mode
 * every opportunity.
 *
 *  This flag is undefined by default.
 */
#define RNG_USE_LOW_POWER_MODE
#undef RNG_USE_LOW_POWER_MODE

/*! @} */
#endif				/* end DOXYGEN_HACK */

/*!
 * If this flag is defined, the driver will not attempt to put the RNG into
 * High Assurance mode.

 * If it is undefined, the driver will attempt to put the RNG into High
 * Assurance mode.  If RNG fails to go into High Assurance mode, the driver
 * will fail to initialize.

 * In either case, if the RNG is already in this mode, the driver will operate
 * normally.
 *
 */
#define RNG_NO_FORCE_HIGH_ASSURANCE

/*!
 * Read a 32-bit value from an RNG register.  This macro depends upon
 * #rng_base.  The os_read32() macro operates on 32-bit quantities, as do
 * all RNG register reads.
 *
 * @param     offset  Register byte offset within RNG.
 *
 * @return    The value from the RNG's register.
 */
#ifndef RNG_REGISTER_DEBUG
#define RNG_READ_REGISTER(offset) os_read32(rng_base+(offset))
#else
#define RNG_READ_REGISTER(offset) dbg_rng_read_register(offset)
#endif

/*!
 * Write a 32-bit value to an RNG register.  This macro depends upon
 * #rng_base.  The os_write32() macro operates on 32-bit quantities, as do
 * all RNG register writes.
 *
 * @param   offset  Register byte offset within RNG.
 * @param   value   32-bit value to store into the register
 *
 * @return   (void)
 */
#ifndef RNG_REGISTER_DEBUG
#define RNG_WRITE_REGISTER(offset,value)                                    \
    (void)os_write32(rng_base+(offset), value)
#else
#define RNG_WRITE_REGISTER(offset,value) dbg_rng_write_register(offset,value)
#endif

#ifndef RNG_DRIVER_NAME
/*! @addtogroup rngcompileflags */
/*! @{ */
/*! Name the driver will use to register itself to the kernel as the driver. */
#define RNG_DRIVER_NAME "rng"
/*! @} */
#endif

/*!
 * Calculate number of words needed to hold the given number of bytes.
 *
 * @param byte_count    Number of bytes
 *
 * @return              Number of words
 */
#define BYTES_TO_WORDS(byte_count)                                          \
    (((byte_count)+sizeof(uint32_t)-1)/sizeof(uint32_t))

/*! Gives high-level view of state of the RNG */
typedef enum rng_status {
	RNG_STATUS_INITIAL,	/*!< Driver status before ever starting. */
	RNG_STATUS_CHECKING,	/*!< During driver initialization. */
	RNG_STATUS_UNIMPLEMENTED,	/*!< Hardware is non-existent / unreachable. */
	RNG_STATUS_OK,		/*!< Hardware is In Secure or Default state. */
	RNG_STATUS_FAILED	/*!< Hardware is In Failed state / other fatal
				   problem.  Driver is still able to read/write
				   some registers, but cannot get Random
				   data. */
} rng_status_t;

static shw_queue_t rng_work_queue;

/*****************************************************************************
 *
 *  Function Declarations
 *
 *****************************************************************************/

/* kernel interface functions */
OS_DEV_INIT_DCL(rng_init);
OS_DEV_TASK_DCL(rng_entropy_task);
OS_DEV_SHUTDOWN_DCL(rng_shutdown);
OS_DEV_ISR_DCL(rng_irq);

#define RNG_ADD_QUEUE_ENTRY(pool, entry)                                    \
      SHW_ADD_QUEUE_ENTRY(pool, (shw_queue_entry_t*)entry)

#define RNG_REMOVE_QUEUE_ENTRY(pool, entry)                                 \
      SHW_REMOVE_QUEUE_ENTRY(pool, (shw_queue_entry_t*)entry)
#define  RNG_GET_WORK_ENTRY()                                               \
      (rng_work_entry_t*)SHW_POP_FIRST_ENTRY(&rng_work_queue)

/*!
 * Add an work item to a work list.  Item will be marked incomplete.
 *
 * @param work Work entry to place at tail of list.
 *
 * @return none
 */
inline static void RNG_ADD_WORK_ENTRY(rng_work_entry_t * work)
{
	work->completed = FALSE;

	SHW_ADD_QUEUE_ENTRY(&rng_work_queue, (shw_queue_entry_t *) work);

	os_dev_schedule_task(rng_entropy_task);
}

/*!
 * For #rng_check_register_accessible(), check read permission on given
 * register.
 */
#define RNG_CHECK_READ 0

/*!
 * For #rng_check_register_accessible(), check write permission on given
 * register.
 */
#define RNG_CHECK_WRITE 1

/* Define different helper symbols based on RNG type */
#ifdef FSL_HAVE_RNGA

/******************************************************************************
 *
 *  RNGA support
 *
 *****************************************************************************/

/*! Interrupt number for driver. */
#if defined(MXC_INT_RNG)
/* Most modern definition */
#define INT_RNG MXC_INT_RNG
#elif defined(MXC_INT_RNGA)
#define INT_RNG MXC_INT_RNGA
#else
#define INT_RNG INT_RNGA
#endif

/*! Base (bus?) address of RNG component. */
#define RNG_BASE_ADDR RNGA_BASE_ADDR

/*! Read and return the status register. */
#define RNG_GET_STATUS()                                                 \
    RNG_READ_REGISTER(RNGA_STATUS)
/*! Configure RNG for Auto seeding */
#define RNG_AUTO_SEED()
/* Put RNG for Seed Generation */
#define RNG_SEED_GEN()
/*!
 * Return RNG Type value.  Should be RNG_TYPE_RNGA, RNG_TYPE_RNGB,
 * or RNG_TYPE_RNGC.
 */
#define RNG_GET_RNG_TYPE()                                                \
    ((RNG_READ_REGISTER(RNGA_CONTROL) & RNGA_CONTROL_RNG_TYPE_MASK)       \
     >> RNGA_CONTROL_RNG_TYPE_SHIFT)

/*!
 * Verify Type value of RNG.
 *
 * Returns true of OK, false if not.
 */
#define RNG_VERIFY_TYPE(type)                                             \
    ((type) == RNG_TYPE_RNGA)

/*! Returns non-zero if RNG device is reporting an error. */
#define RNG_HAS_ERROR()                                                   \
    (RNG_READ_REGISTER(RNGA_STATUS) & RNGA_STATUS_ERROR_INTERRUPT)
/*! Returns non-zero if Bad Key is selected */
#define RNG_HAS_BAD_KEY()     0
/*! Return non-zero if Self Test Done */
#define RNG_SELF_TEST_DONE()  0
/*! Returns non-zero if RNG ring oscillators have failed. */
#define RNG_OSCILLATOR_FAILED()                                           \
    (RNG_READ_REGISTER(RNGA_STATUS) & RNGA_STATUS_OSCILLATOR_DEAD)

/*! Returns maximum number of 32-bit words in the RNG's output fifo. */
#define RNG_GET_FIFO_SIZE()                                               \
    ((RNG_READ_REGISTER(RNGA_STATUS) & RNGA_STATUS_OUTPUT_FIFO_SIZE_MASK) \
     >> RNGA_STATUS_OUTPUT_FIFO_SIZE_SHIFT)

/*! Returns number of 32-bit words currently in the RNG's output fifo. */
#define RNG_GET_WORDS_IN_FIFO()                                            \
    ((RNG_READ_REGISTER(RNGA_STATUS) & RNGA_STATUS_OUTPUT_FIFO_LEVEL_MASK) \
     >> RNGA_STATUS_OUTPUT_FIFO_LEVEL_SHIFT)
/* Configuring RNG for Self Test */
#define RNG_SELF_TEST()
/*! Get a random value from the RNG's output FIFO. */
#define RNG_READ_FIFO()                                                  \
    RNG_READ_REGISTER(RNGA_OUTPUT_FIFO)

/*! Put entropy into the RNG's algorithm.
 *  @param value  32-bit value to add to RNG's entropy.
 **/
#define RNG_ADD_ENTROPY(value)                                           \
    RNG_WRITE_REGISTER(RNGA_ENTROPY, (value))
/*! Return non-zero in case of Error during Self Test */
#define RNG_CHECK_SELF_ERR() 0
/*! Return non-zero in case of Error during Seed Generation */
#define RNG_CHECK_SEED_ERR() 0
/*! Get the RNG started at generating output. */
#define RNG_GO()                                                         \
{                                                                        \
    register uint32_t control = RNG_READ_REGISTER(RNGA_CONTROL);         \
    RNG_WRITE_REGISTER(RNGA_CONTROL, control | RNGA_CONTROL_GO);         \
}
/*! To clear all Error Bits in Error Status Register */
#define RNG_CLEAR_ERR()
/*! Put RNG into High Assurance mode */
#define RNG_SET_HIGH_ASSURANCE()                                              \
{                                                                             \
    register uint32_t control = RNG_READ_REGISTER(RNGA_CONTROL);              \
    RNG_WRITE_REGISTER(RNGA_CONTROL, control | RNGA_CONTROL_HIGH_ASSURANCE);  \
}

/*! Return non-zero if the RNG is in High Assurance mode. */
#define RNG_GET_HIGH_ASSURANCE()                                              \
    (RNG_READ_REGISTER(RNGA_CONTROL) & RNGA_CONTROL_HIGH_ASSURANCE)

/*! Clear all status, error and otherwise. */
#define RNG_CLEAR_ALL_STATUS()                                                \
{                                                                             \
    register uint32_t control = RNG_READ_REGISTER(RNGA_CONTROL);              \
    RNG_WRITE_REGISTER(RNGA_CONTROL, control | RNGA_CONTROL_CLEAR_INTERRUPT); \
}
/* Return non-zero if RESEED Required */
#define RNG_RESEED() 1

/*! Return non-zero if Seeding is done */
#define RNG_SEED_DONE()  1

/*! Return non-zero if everything seems OK with the RNG. */
#define RNG_WORKING()                                                    \
    ((RNG_READ_REGISTER(RNGA_STATUS)                                     \
      & (RNGA_STATUS_SLEEP | RNGA_STATUS_SECURITY_VIOLATION              \
         | RNGA_STATUS_ERROR_INTERRUPT | RNGA_STATUS_FIFO_UNDERFLOW      \
         | RNGA_STATUS_LAST_READ_STATUS )) == 0)

/*! Put the RNG into sleep (low-power) mode. */
#define RNG_SLEEP()                                                      \
{                                                                        \
    register uint32_t control = RNG_READ_REGISTER(RNGA_CONTROL);         \
    RNG_WRITE_REGISTER(RNGA_CONTROL, control | RNGA_CONTROL_SLEEP);      \
}

/*! Wake the RNG from sleep (low-power) mode. */
#define RNG_WAKE()                                                       \
{                                                                        \
    uint32_t control = RNG_READ_REGISTER(RNGA_CONTROL);                  \
     RNG_WRITE_REGISTER(RNGA_CONTROL, control & ~RNGA_CONTROL_SLEEP);    \
}

/*! Mask interrupts so that the driver/OS will not see them. */
#define RNG_MASK_ALL_INTERRUPTS()                                             \
{                                                                             \
    register uint32_t control = RNG_READ_REGISTER(RNGA_CONTROL);              \
    RNG_WRITE_REGISTER(RNGA_CONTROL, control | RNGA_CONTROL_MASK_INTERRUPTS); \
}

/*! Unmask interrupts so that the driver/OS will see them. */
#define RNG_UNMASK_ALL_INTERRUPTS()                                           \
{                                                                             \
    register uint32_t control = RNG_READ_REGISTER(RNGA_CONTROL);              \
    RNG_WRITE_REGISTER(RNGA_CONTROL, control & ~RNGA_CONTROL_MASK_INTERRUPTS);\
}

/*!
 * @def RNG_PUT_RNG_TO_SLEEP()
 *
 * If compiled with #RNG_USE_LOW_POWER_MODE, this routine will put the RNG
 * to sleep (low power mode).
 *
 * @return none
 */
/*!
 * @def RNG_WAKE_RNG_FROM_SLEEP()
 *
 * If compiled with #RNG_USE_LOW_POWER_MODE, this routine will wake the RNG
 * from sleep (low power mode).
 *
 * @return none
 */
#ifdef RNG_USE_LOW_POWER_MODE

#define RNG_PUT_RNG_TO_SLEEP()                                                \
    RNG_SLEEP()

#define RNG_WAKE_FROM_SLEEP()                                                 \
    RNG_WAKE()  1

#else				/* not low power mode */

#define RNG_PUT_RNG_TO_SLEEP()

#define RNG_WAKE_FROM_SLEEP()

#endif				/* Use low-power mode */

#else				/* FSL_HAVE_RNGB or FSL_HAVE_RNGC */

/******************************************************************************
 *
 *  RNGB and RNGC support
 *
 *****************************************************************************/
/*
 * The operational interfaces for RNGB and RNGC are almost identical, so
 * the defines for RNGC work fine for both.  There are minor differences
 * which will be treated within this conditional block.
 */

/*! Interrupt number for driver. */
#if defined(MXC_INT_RNG)
/* Most modern definition */
#define INT_RNG MXC_INT_RNG
#elif defined(MXC_INT_RNGC)
#define INT_RNG MXC_INT_RNGC
#elif defined(MXC_INT_RNGB)
#define INT_RNG MXC_INT_RNGB
#elif defined(INT_RNGC)
#define INT_RNG INT_RNGC
#else
#error NO_INTERRUPT_DEFINED
#endif

/*! Base address of RNG component. */
#ifdef FSL_HAVE_RNGB
#define RNG_BASE_ADDR RNGB_BASE_ADDR
#else
#define RNG_BASE_ADDR RNGC_BASE_ADDR
#endif

/*! Read and return the status register. */
#define RNG_GET_STATUS()                                                 \
    RNG_READ_REGISTER(RNGC_ERROR)

/*!
 * Return RNG Type value.  Should be RNG_TYPE_RNGA or RNG_TYPE_RNGC.
 */
#define RNG_GET_RNG_TYPE()                                                \
    ((RNG_READ_REGISTER(RNGC_VERSION_ID) & RNGC_VERID_RNG_TYPE_MASK)      \
     >> RNGC_VERID_RNG_TYPE_SHIFT)

/*!
 * Verify Type value of RNG.
 *
 * Returns true of OK, false if not.
 */
#ifdef FSL_HAVE_RNGB
#define RNG_VERIFY_TYPE(type)                                             \
    ((type) == RNG_TYPE_RNGB)
#else				/* RNGC */
#define RNG_VERIFY_TYPE(type)                                             \
    ((type) == RNG_TYPE_RNGC)
#endif

/*! Returns non-zero if RNG device is reporting an error. */
#define RNG_HAS_ERROR()                                                   \
    (RNG_READ_REGISTER(RNGC_STATUS) & RNGC_STATUS_ERROR)
/*! Returns non-zero if Bad Key is selected */
#define RNG_HAS_BAD_KEY()                                                 \
     (RNG_READ_REGISTER(RNGC_ERROR) & RNGC_ERROR_STATUS_BAD_KEY)
/*! Returns non-zero if RNG ring oscillators have failed. */
#define RNG_OSCILLATOR_FAILED()                                           \
    (RNG_READ_REGISTER(RNGC_ERROR) & RNGC_ERROR_STATUS_OSC_ERR)

/*! Returns maximum number of 32-bit words in the RNG's output fifo. */
#define RNG_GET_FIFO_SIZE()                                               \
    ((RNG_READ_REGISTER(RNGC_STATUS) & RNGC_STATUS_FIFO_SIZE_MASK)        \
     >> RNGC_STATUS_FIFO_SIZE_SHIFT)

/*! Returns number of 32-bit words currently in the RNG's output fifo. */
#define RNG_GET_WORDS_IN_FIFO()                                           \
    ((RNG_READ_REGISTER(RNGC_STATUS) & RNGC_STATUS_FIFO_LEVEL_MASK)       \
     >> RNGC_STATUS_FIFO_LEVEL_SHIFT)

/*! Get a random value from the RNG's output FIFO. */
#define RNG_READ_FIFO()                                                   \
    RNG_READ_REGISTER(RNGC_FIFO)

/*! Put entropy into the RNG's algorithm.
 *  @param value  32-bit value to add to RNG's entropy.
 **/
#ifdef FSL_HAVE_RNGB
#define RNG_ADD_ENTROPY(value)                                             \
    RNG_WRITE_REGISTER(RNGB_ENTROPY, value)
#else				/* RNGC does not have Entropy register */
#define RNG_ADD_ENTROPY(value)
#endif
/*! Wake the RNG from sleep (low-power) mode. */
#define RNG_WAKE()  1
/*! Get the RNG started at generating output. */
#define RNG_GO()
/*! Put RNG into High Assurance mode. */
#define RNG_SET_HIGH_ASSURANCE()
/*! Returns non-zero in case of Error during Self Test       */
#define RNG_CHECK_SELF_ERR()                                                 \
        (RNG_READ_REGISTER(RNGC_ERROR) & RNGC_ERROR_STATUS_ST_ERR)
/*! Return non-zero in case of Error during Seed Generation */
#define RNG_CHECK_SEED_ERR()                                                 \
        (RNG_READ_REGISTER(RNGC_ERROR) & RNGC_ERROR_STATUS_STAT_ERR)

/*! Configure RNG for Self Test */
#define RNG_SELF_TEST()                                                       \
{                                                                             \
    register uint32_t command = RNG_READ_REGISTER(RNGC_COMMAND);              \
    RNG_WRITE_REGISTER(RNGC_COMMAND, command                                  \
                                    | RNGC_COMMAND_SELF_TEST);                \
}
/*! Clearing the Error bits in Error Status Register */
#define RNG_CLEAR_ERR()                                                       \
{                                                                             \
    register uint32_t command = RNG_READ_REGISTER(RNGC_COMMAND);              \
    RNG_WRITE_REGISTER(RNGC_COMMAND, command                                  \
                                    | RNGC_COMMAND_CLEAR_ERROR);              \
}

/*! Return non-zero if Self Test Done */
#define RNG_SELF_TEST_DONE()                                                  \
         (RNG_READ_REGISTER(RNGC_STATUS) & RNGC_STATUS_ST_DONE)
/* Put RNG for SEED Generation */
#define RNG_SEED_GEN()                                                        \
{                                                                             \
    register uint32_t command = RNG_READ_REGISTER(RNGC_COMMAND);              \
    RNG_WRITE_REGISTER(RNGC_COMMAND, command                                  \
                                    | RNGC_COMMAND_SEED);                     \
}
/* Return non-zero if RESEED Required */
#define RNG_RESEED()                                                          \
    (RNG_READ_REGISTER(RNGC_STATUS) & RNGC_STATUS_RESEED)

/*! Return non-zero if the RNG is in High Assurance mode. */
#define RNG_GET_HIGH_ASSURANCE() (RNG_READ_REGISTER(RNGC_STATUS) &            \
                                  RNGC_STATUS_SEC_STATE)

/*! Clear all status, error and otherwise. */
#define RNG_CLEAR_ALL_STATUS()                                                \
    RNG_WRITE_REGISTER(RNGC_COMMAND,                                          \
                       RNGC_COMMAND_CLEAR_INTERRUPT                           \
                       | RNGC_COMMAND_CLEAR_ERROR)

/*! Return non-zero if everything seems OK with the RNG. */
#define RNG_WORKING()                                                         \
    ((RNG_READ_REGISTER(RNGC_ERROR)                                           \
      & (RNGC_ERROR_STATUS_STAT_ERR | RNGC_ERROR_STATUS_RAND_ERR              \
       | RNGC_ERROR_STATUS_FIFO_ERR | RNGC_ERROR_STATUS_ST_ERR |              \
         RNGC_ERROR_STATUS_OSC_ERR  | RNGC_ERROR_STATUS_LFSR_ERR )) == 0)
/*! Return Non zero if SEEDING is DONE */
#define RNG_SEED_DONE()                                                       \
     ((RNG_READ_REGISTER(RNGC_STATUS) & RNGC_STATUS_SEED_DONE) != 0)

/*! Put the RNG into sleep (low-power) mode. */
#define RNG_SLEEP()

/*! Wake the RNG from sleep (low-power) mode. */

/*! Mask interrupts so that the driver/OS will not see them. */
#define RNG_MASK_ALL_INTERRUPTS()                                             \
{                                                                             \
    register uint32_t control = RNG_READ_REGISTER(RNGC_CONTROL);              \
    RNG_WRITE_REGISTER(RNGC_CONTROL, control                                  \
                                    | RNGC_CONTROL_MASK_DONE                  \
                                    | RNGC_CONTROL_MASK_ERROR);               \
}
/*! Configuring RNGC for self Test. */

#define RNG_AUTO_SEED()                                                       \
{                                                                             \
    register uint32_t control = RNG_READ_REGISTER(RNGC_CONTROL);              \
    RNG_WRITE_REGISTER(RNGC_CONTROL, control                                  \
                                    | RNGC_CONTROL_AUTO_SEED);                \
}

/*! Unmask interrupts so that the driver/OS will see them. */
#define RNG_UNMASK_ALL_INTERRUPTS()                                           \
{                                                                             \
    register uint32_t control = RNG_READ_REGISTER(RNGC_CONTROL);              \
    RNG_WRITE_REGISTER(RNGC_CONTROL,                                          \
                control & ~(RNGC_CONTROL_MASK_DONE|RNGC_CONTROL_MASK_ERROR)); \
}

/*! Put RNG to sleep if appropriate. */
#define RNG_PUT_RNG_TO_SLEEP()

/*! Wake RNG from sleep if necessary. */
#define RNG_WAKE_FROM_SLEEP()

#endif				/* RNG TYPE */

/* internal functions */
static os_error_code rng_map_RNG_memory(void);
static os_error_code rng_setup_interrupt_handling(void);
#ifdef RNG_REGISTER_PEEK_POKE
inline static int rng_check_register_offset(uint32_t offset);
inline static int rng_check_register_accessible(uint32_t offset,
						int access_write);
#endif				/*  DEBUG_RNG_REGISTERS */
static fsl_shw_return_t rng_drain_fifo(uint32_t * random_p, int count_words);
static os_error_code rng_grab_config_values(void);
static void rng_cleanup(void);

#ifdef FSL_HAVE_RNGA
static void rng_sec_failure(void);
#endif

#ifdef RNG_REGISTER_DEBUG
static uint32_t dbg_rng_read_register(uint32_t offset);
static void dbg_rng_write_register(uint32_t offset, uint32_t value);
#endif

#if defined(LINUX_VERSION_CODE)

EXPORT_SYMBOL(fsl_shw_add_entropy);
EXPORT_SYMBOL(fsl_shw_get_random);

#ifdef RNG_REGISTER_PEEK_POKE
/* For Linux kernel, export the API functions to other kernel modules */
EXPORT_SYMBOL(rng_read_register);
EXPORT_SYMBOL(rng_write_register);
#endif				/* DEBUG_RNG_REGISTERS */



MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Device Driver for RNG");

#endif				/* LINUX_VERSION_CODE */

#endif				/* RNG_INTERNALS_H */
