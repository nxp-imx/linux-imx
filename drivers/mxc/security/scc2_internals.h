/*
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef SCC_INTERNALS_H
#define SCC_INTERNALS_H

/** @file scc2_internals.h
 *
 * @brief This is intended to be the file which contains most or all of the
 * code or changes need to port the driver.  It also includes other definitions
 * needed by the driver.
 *
 *  This header file should only ever be included by scc2_driver.c
 *
 *  Compile-time flags minimally needed:
 *
 *  @li Some sort of platform flag.  Currently TAHITI and MXC are understood.
 *  @li Some start-of-SCC consideration, such as SCC_BASE_ADDR
 *
 *  Some changes which could be made when porting this driver:
 *  #SCC_SPIN_COUNT
 *
 */

#include <linux/version.h>          /* Current version Linux kernel */
#include <linux/module.h>           /* Basic support for loadable modules,
                                       printk */
#include <linux/init.h>             /* module_init, module_exit */
#include <linux/kernel.h>           /* General kernel system calls */
#include <linux/sched.h>            /* for interrupt.h */
#include <linux/spinlock.h>

#include <linux/io.h>               /* ioremap() */
#include <linux/interrupt.h>        /* IRQ / interrupt definitions */


#include <linux/mxc_scc2_driver.h>

/**
 * Define the number of Stored Keys which the SCC driver will make available.
 * Value shall be from 0 to 20.  Default is zero (0).
 */
/*#define SCC_KEY_SLOTS   20*/


/* Temporarily define compile-time flags to make Doxygen happy. */
#ifdef DOXYGEN_HACK
/** @addtogroup scccompileflags */
/** @{ */


/** @def NO_SMN_INTERRUPT
 * The SMN interrupt is not wired to the CPU at all.
 */
#define NO_SMN_INTERRUPT


/**
 * Register an interrupt handler for the SMN as well as
 * the SCM.  In some implementations, the SMN is not connected at all (see
 * #NO_SMN_INTERRUPT), and in others, it is on the same interrupt line as the
 * SCM.  When defining this flag, the SMN interrupt should be on a separate
 * line from the SCM interrupt.
 */

#define USE_SMN_INTERRUPT


/**
 * Turn on generation of run-time operational, debug, and error messages
 */
#define SCC_DEBUG


/**
 * Turn on generation of run-time logging of access to the SCM and SMN
 * registers.
 */
#define SCC_REGISTER_DEBUG


/**
 * Turn on generation of run-time logging of access to the SCM Red and
 * Black memories.  Will only work if #SCC_REGISTER_DEBUG is also defined.
 */
#define SCC_RAM_DEBUG


/**
 *  If the driver finds the SCC in HEALTH_CHECK state, go ahead and
 *  run a quick ASC to bring it to SECURE state.
 */
#define SCC_BRINGUP


/**
 * Expected to come from platform header files or compile command line.
 * This symbol must be the address of the SCC
 */
#define SCC_BASE

/**
 *  This must be the interrupt line number of the SCM interrupt.
 */
#define INT_SCM

/**
 *  if #USE_SMN_INTERRUPT is defined, this must be the interrupt line number of
 *  the SMN interrupt.
 */
#define INT_SMN

/**
 * Define the number of Stored Keys which the SCC driver will make available.
 * Value shall be from 0 to 20.  Default is zero (0).
 */
#define SCC_KEY_SLOTS

/**
 * Make sure that this flag is defined if compiling for a Little-Endian
 * platform.  Linux Kernel builds provide this flag.
 */
#define __LITTLE_ENDIAN

/**
 * Make sure that this flag is defined if compiling for a Big-Endian platform.
 * Linux Kernel builds provide this flag.
 */
#define __BIG_ENDIAN

/**
 * Read a 32-bit register value from a 'peripheral'.  Standard Linux/Unix
 * macro.
 *
 * @param offset   Bus address of register to be read
 *
 * @return The value of the register
 */
#define readl(offset)


/**
 * Write a 32-bit value to a register in a 'peripheral'.  Standard Linux/Unix
 * macro.
 *
 * @param value    The 32-bit value to store
 * @param offset   Bus address of register to be written
 *
 * return (none)
 */
#define writel(value,offset)


/** @} */  /* end group scccompileflags */

#endif  /* DOXYGEN_HACK */


#ifndef SCC_KEY_SLOTS
#define SCC_KEY_SLOTS 0

#else

#if (SCC_KEY_SLOTS < 0) || (SCC_KEY_SLOTS > 20)
#error Bad value for SCC_KEY_SLOTS
#endif

#endif


/**
 * Maximum length of key/secret value which can be stored in SCC.
 */
#define SCC_MAX_KEY_SIZE 256


/**
 * This is the size, in bytes, of each key slot, and therefore the maximum size
 * of the wrapped key.
 */
#define SCC_KEY_SLOT_SIZE 32


/* These come for free with Linux, but may need to be set in a port. */
#ifndef __BIG_ENDIAN
#ifndef __LITTLE_ENDIAN
#error One of __LITTLE_ENDIAN or __BIG_ENDIAN must be #defined
#endif
#else
#ifdef __LITTLE_ENDIAN
#error Exactly one of __LITTLE_ENDIAN or __BIG_ENDIAN must be #defined
#endif
#endif


#ifndef SCC_CALLBACK_SIZE
/** The number of function pointers which can be stored in #scc_callbacks.
 *  Defaults to 4, can be overridden with compile-line argument.
 */
#define SCC_CALLBACK_SIZE 4
#endif


/** Initial CRC value for CCITT-CRC calculation. */
#define CRC_CCITT_START 0xFFFF


#ifdef TAHITI

/**
 * The SCC_BASE has to be SMN_BASE_ADDR on TAHITI, as the banks of
 * registers are swapped in place.
 */
#define SCC_BASE       SMN_BASE_ADDR


/** The interrupt number for the SCC (SCM only!) on Tahiti */
#define INT_SCC_SCM       62


/** Tahiti does not have the SMN interrupt wired to the CPU.  */
#define NO_SMN_INTERRUPT


#endif /* TAHITI */


/** Number of times to spin between polling of SCC while waiting for cipher
 *  or zeroizing function to complete. See also #SCC_CIPHER_MAX_POLL_COUNT. */
#define SCC_SPIN_COUNT 1000


/** Number of times to polling SCC while waiting for cipher
 *  or zeroizing function to complete.  See also #SCC_SPIN_COUNT.  */
#define SCC_CIPHER_MAX_POLL_COUNT 100


/**
 * @def SCC_READ_REGISTER
 * Read a 32-bit value from an SCC register.  Macro which depends upon
 * #scc_base.  Linux readl()/writel() macros operate on 32-bit quantities, as
 * do SCC register reads/writes.
 *
 * @param     offset  Register offset within SCC.
 *
 * @return    The value from the SCC's register.
 */
#ifndef SCC_REGISTER_DEBUG
#define SCC_READ_REGISTER(offset) __raw_readl(scc_base+(offset))
#else
#define SCC_READ_REGISTER(offset) dbg_scc_read_register(offset)
#endif


/**
 * Write a 32-bit value to an SCC register.  Macro depends upon #scc_base.
 * Linux readl()/writel() macros operate on 32-bit quantities, as do SCC
 * register reads/writes.
 *
 * @param   offset  Register offset within SCC.
 * @param   value   32-bit value to store into the register
 *
 * @return   (void)
 */
#ifndef SCC_REGISTER_DEBUG
#define SCC_WRITE_REGISTER(offset,value) \
	(void)__raw_writel(value, scc_base+(offset))
#else
#define SCC_WRITE_REGISTER(offset,value) \
	dbg_scc_write_register(offset, value)
#endif

/**
 * Calculate the kernel virtual address of a partition from the partition number.
 */
#define SCM_PART_ADDRESS(part)                                          \
 (scm_ram_base + (part*scc_configuration.partition_size_bytes))

/**
 * Calculate the partition number from the kernel virtual address.
 */
#define SCM_PART_NUMBER(address)                                        \
 ((address - (uint32_t)scm_ram_base)/scc_configuration.partition_size_bytes)

/**
 * Calculates the byte offset into a word
 *  @param   bp  The byte (char*) pointer
 *  @return      The offset (0, 1, 2, or 3)
 */
#define SCC_BYTE_OFFSET(bp) ((uint32_t)(bp) % sizeof(uint32_t))


/**
 * Converts (by rounding down) a byte pointer into a word pointer
 *  @param  bp  The byte (char*) pointer
 *  @return     The word (uint32_t) as though it were an aligned (uint32_t*)
 */
#define SCC_WORD_PTR(bp) (((uint32_t)(bp)) & ~(sizeof(uint32_t)-1))


/**
 * Determine number of bytes in an SCC block
 *
 * @return Bytes / block
 */
#define SCC_BLOCK_SIZE_BYTES() scc_configuration.block_size_bytes


/**
 * Maximum number of additional bytes which may be added in CRC+padding mode.
 */
#define PADDING_BUFFER_MAX_BYTES (CRC_SIZE_BYTES + sizeof(scc_block_padding))

/**
 *  Shorthand (clearer, anyway) for number of bytes in a CRC.
 */
#define CRC_SIZE_BYTES (sizeof(crc_t))

/**
 * The polynomial used in CCITT-CRC calculation
 */
#define CRC_POLYNOMIAL 0x1021

/**
 * Calculate CRC on one byte of data
 *
 * @param[in,out] running_crc  A value of type crc_t where CRC is kept. This
 *                             must be an rvalue and an lvalue.
 * @param[in]     byte_value   The byte (uint8_t, char) to be put in the CRC
 *
 * @return      none
 */
#define CALC_CRC(byte_value,running_crc)  {                                  \
    uint8_t data;                                                            \
    data = (0xff&(byte_value)) ^ (running_crc >> 8);                         \
    running_crc = scc_crc_lookup_table[data] ^ (running_crc << 8);           \
}

/** Value of 'beginning of padding' marker in driver-provided padding */
#define SCC_DRIVER_PAD_CHAR  0x80


/** Name of the driver.  Used (on Linux, anyway) when registering interrupts */
#define SCC_DRIVER_NAME "mxc_scc"


/* Port -- these symbols are defined in Linux 2.6 and later.  They are defined
 * here for backwards compatibility because this started life as a 2.4
 * driver, and as a guide to portation to other platforms.
 */

#if !defined(LINUX_VERSION_CODE) || LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)

#define irqreturn_t void        /* Return type of an interrupt handler */

#define IRQ_HANDLED /* Would be '1' for handled -- as in return IRQ_HANDLED; */

#define IRQ_NONE   /* would be '0' for not handled -- as in return IRQ_NONE; */

#define IRQ_RETVAL(x) /* Return x==0 (not handled) or non-zero (handled) */

#endif /* LINUX earlier than 2.5 */


/* These are nice to have around */
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif


/** Provide a typedef for the CRC which can be used in encrypt/decrypt */
typedef uint16_t crc_t;


/** Gives high-level view of state of the SCC */
enum scc_status {
    SCC_STATUS_INITIAL,         /**< State of driver before ever checking */
    SCC_STATUS_CHECKING,        /**< Transient state while driver loading  */
    SCC_STATUS_UNIMPLEMENTED,   /**< SCC is non-existent or unuseable  */
    SCC_STATUS_OK,              /**< SCC is in Secure or Default state  */
    SCC_STATUS_FAILED           /**< In Failed state */
};

/**
 * Information about a key slot.
 */
struct scc_key_slot
{
    uint64_t   owner_id;        /**< Access control value. */
    uint32_t   length;          /**< Length of value in slot. */
    uint32_t   offset;          /**< Offset of value from start of RAM. */
    uint32_t   status;          /**< 0 = unassigned, 1 = assigned. */
    uint32_t   part_ctl;        /**< for the CCMD register  */
};

/* Forward-declare a number routines which are not part of user api */
static int scc_init(void);
static void scc_cleanup(void);
static int __init scc_driver_init(void);
static void __exit scc_driver_exit(void);

/* Forward defines of internal functions */
OS_DEV_ISR(scc_irq);
/*static irqreturn_t scc_irq(int irq, void *dev_id);*/
/** Perform callbacks registered by #scc_monitor_security_failure().
 *
 *  Make sure callbacks only happen once...  Since there may be some reason why
 *  the interrupt isn't generated, this routine could be called from base(task)
 *  level.
 *
 *  One at a time, go through #scc_callbacks[] and call any non-null pointers.
 */
static void scc_perform_callbacks(void);
/*static uint32_t copy_to_scc(const uint8_t* from, uint32_t to, unsigned long count_bytes, uint16_t* crc);
static uint32_t copy_from_scc(const uint32_t from, uint8_t* to,unsigned long count_bytes, uint16_t* crc);
static scc_return_t scc_strip_padding(uint8_t* from,unsigned* count_bytes_stripped);*/
static uint32_t scc_update_state(void);
static void scc_init_ccitt_crc(void);
static uint32_t scc_grab_config_values(void);
static int setup_interrupt_handling(void);
/**
 * Perform an encryption on the input.  If @c verify_crc is true, a CRC must be
 * calculated on the plaintext, and appended, with padding, before computing
 * the ciphertext.
 *
 * @param[in]     count_in_bytes  Count of bytes of plaintext
 * @param[in]     data_in         Pointer to the plaintext
 * @param[in]     scm_control     Bit values for the SCM_CONTROL register
 * @param[in,out] data_out        Pointer for storing ciphertext
 * @param[in]     add_crc         Flag for computing CRC - 0 no, else yes
 * @param[in,out] count_out_bytes Number of bytes available at @c data_out
 */
/*static scc_return_t scc_encrypt(uint32_t count_in_bytes, uint8_t* data_in, uint32_t scm_control, uint8_t* data_out,int add_crc, unsigned long* count_out_bytes);*/
/**
 * Perform a decryption on the input.  If @c verify_crc is true, the last block
 * (maybe the two last blocks) is special - it should contain a CRC and
 * padding.  These must be stripped and verified.
 *
 * @param[in]     count_in_bytes  Count of bytes of ciphertext
 * @param[in]     data_in         Pointer to the ciphertext
 * @param[in]     scm_control     Bit values for the SCM_CONTROL register
 * @param[in,out] data_out        Pointer for storing plaintext
 * @param[in]     verify_crc      Flag for running CRC - 0 no, else yes
 * @param[in,out] count_out_bytes Number of bytes available at @c data_out

 */
/*static scc_return_t scc_decrypt(uint32_t count_in_bytes, uint8_t* data_in, uint32_t scm_control, uint8_t* data_out, int verify_crc, unsigned long* count_out_bytes);*/
static uint32_t host_owns_partition(uint32_t part_no);
static uint32_t partition_engaged(uint32_t part_no);

static scc_return_t scc_wait_completion(uint32_t* scm_status);
static int is_cipher_done(uint32_t* scm_status);
static scc_return_t check_register_accessible (uint32_t offset,
                                               uint32_t smn_status,
                                               uint32_t scm_status);
static scc_return_t check_register_offset(uint32_t offset);
/*uint8_t make_vpu_partition(void);*/

#ifdef SCC_REGISTER_DEBUG
static uint32_t dbg_scc_read_register(uint32_t offset);
static void dbg_scc_write_register(uint32_t offset, uint32_t value);
#endif


/* For Linux kernel, export the API functions to other kernel modules */
EXPORT_SYMBOL(scc_get_configuration);
EXPORT_SYMBOL(scc_zeroize_memories);
/*EXPORT_SYMBOL(scc_crypt);*/
EXPORT_SYMBOL(scc_set_sw_alarm);
EXPORT_SYMBOL(scc_monitor_security_failure);
EXPORT_SYMBOL(scc_stop_monitoring_security_failure);
EXPORT_SYMBOL(scc_read_register);
EXPORT_SYMBOL(scc_write_register);
EXPORT_SYMBOL(scc_allocate_partition);
EXPORT_SYMBOL(scc_engage_partition);
EXPORT_SYMBOL(scc_release_partition);
EXPORT_SYMBOL(scc_diminish_permissions);
EXPORT_SYMBOL(scc_encrypt_region);
EXPORT_SYMBOL(scc_decrypt_region);
EXPORT_SYMBOL(scc_partition_status);
EXPORT_SYMBOL(scc_virt_to_phys);
/*EXPORT_SYMBOL(make_vpu_partition);*/


/* Tell Linux this is not GPL code */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Device Driver for SCC (SMN/SCM)");


#endif /* SCC_INTERNALS_H */
