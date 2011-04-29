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

/*! @file scc2_driver.c
 *
 * This is the driver code for the Security Controller version 2 (SCC2).  It's
 * interaction with the Linux kernel is from calls to #scc_init() when the
 * driver is loaded, and #scc_cleanup() should the driver be unloaded.  The
 * driver uses locking and (task-sleep/task-wakeup) functions from the kernel.
 * It also registers itself to handle the interrupt line(s) from the SCC.  New
 * to this version of the driver is an interface providing access to the secure
 * partitions.  This is in turn exposed to the API user through the
 * fsl_shw_smalloc() series of functions.  Other drivers in the kernel may use
 * the remaining API functions to get at the services of the SCC.  The main
 * service provided is the Secure Memory, which allows encoding and decoding of
 * secrets with a per-chip secret key.
 *
 * The SCC is single-threaded, and so is this module.  When the scc_crypt()
 * routine is called, it will lock out other accesses to the function.  If
 * another task is already in the module, the subsequent caller will spin on a
 * lock waiting for the other access to finish.
 *
 * Note that long crypto operations could cause a task to spin for a while,
 * preventing other kernel work (other than interrupt processing) to get done.
 *
 * The external (kernel module) interface is through the following functions:
 * @li scc_get_configuration() @li scc_crypt() @li scc_zeroize_memories() @li
 * scc_monitor_security_failure() @li scc_stop_monitoring_security_failure()
 * @li scc_set_sw_alarm() @li scc_read_register() @li scc_write_register() @li
 * scc_allocate_partition() @li scc_initialize_partition @li
 * scc_release_partition() @li scc_diminish_permissions @li
 * scc_encrypt_region() @li scc_decrypt_region() @li scc_virt_to_phys
 *
 * All other functions are internal to the driver.
 */

#include "sahara2/include/portable_os.h"
#include "scc2_internals.h"
#include <linux/delay.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))

#include <linux/device.h>
#include <mach/clock.h>
#include <linux/device.h>

#else

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>

#endif

#include <linux/dmapool.h>

/**
 * This is the set of errors which signal that access to the SCM RAM has
 * failed or will fail.
 */
#define SCM_ACCESS_ERRORS                                                  \
       (SCM_ERRSTAT_ILM | SCM_ERRSTAT_SUP | SCM_ERRSTAT_ERC_MASK)

/******************************************************************************
 *
 *  Global / Static Variables
 *
 *****************************************************************************/

#ifdef SCC_REGISTER_DEBUG

#define REG_PRINT_BUFFER_SIZE 200

static char reg_print_buffer[REG_PRINT_BUFFER_SIZE];

typedef char *(*reg_print_routine_t) (uint32_t value, char *print_buffer,
				      int buf_size);

#endif

/**
 * This is type void* so that a) it cannot directly be dereferenced,
 * and b) pointer arithmetic on it will function in a 'normal way' for
 * the offsets in scc_defines.h
 *
 * scc_base is the location in the iomap where the SCC's registers
 * (and memory) start.
 *
 * The referenced data is declared volatile so that the compiler will
 * not make any assumptions about the value of registers in the SCC,
 * and thus will always reload the register into CPU memory before
 * using it (i.e. wherever it is referenced in the driver).
 *
 * This value should only be referenced by the #SCC_READ_REGISTER and
 * #SCC_WRITE_REGISTER macros and their ilk.  All dereferences must be
 * 32 bits wide.
 */
static volatile void *scc_base;
uint32_t scc_phys_base;

/** Array to hold function pointers registered by
    #scc_monitor_security_failure() and processed by
    #scc_perform_callbacks() */
static void (*scc_callbacks[SCC_CALLBACK_SIZE]) (void);
/*SCC need IRAM's base address but use only the partitions allocated for it.*/
uint32_t scm_ram_phys_base;

void *scm_ram_base = NULL;

/** Calculated once for quick reference to size of the unreserved space in
 *  RAM in SCM.
 */
uint32_t scm_memory_size_bytes;

/** Structure returned by #scc_get_configuration() */
static scc_config_t scc_configuration = {
	.driver_major_version = SCC_DRIVER_MAJOR_VERSION,
	.driver_minor_version = SCC_DRIVER_MINOR_VERSION_2,
	.scm_version = -1,
	.smn_version = -1,
	.block_size_bytes = -1,
	.partition_size_bytes = -1,
	.partition_count = -1,
};

/** Internal flag to know whether SCC is in Failed state (and thus many
 *  registers are unavailable).  Once it goes failed, it never leaves it. */
static volatile enum scc_status scc_availability = SCC_STATUS_INITIAL;

/* Variables to hold irq numbers */
static int irq_smn;
static int irq_scm;

/** Flag to say whether interrupt handler has been registered for
 * SMN interrupt */
static int smn_irq_set = 0;

/** Flag to say whether interrupt handler has been registered for
 * SCM interrupt */
static int scm_irq_set = 0;

/** This lock protects the #scc_callbacks list as well as the @c
 * callbacks_performed flag in #scc_perform_callbacks.  Since the data this
 * protects may be read or written from either interrupt or base level, all
 * operations should use the irqsave/irqrestore or similar to make sure that
 * interrupts are inhibited when locking from base level.
 */
static os_lock_t scc_callbacks_lock = NULL;

/**
 * Ownership of this lock prevents conflicts on the crypto operation in the
 * SCC.
 */
static os_lock_t scc_crypto_lock = NULL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
/** Pointer to SCC's clock information.  Initialized during scc_init(). */
static struct clk *scc_clk = NULL;
#endif

/** The lookup table for an 8-bit value.  Calculated once
 * by #scc_init_ccitt_crc().
 */
static uint16_t scc_crc_lookup_table[256];

/******************************************************************************
 *
 *  Function Implementations - Externally Accessible
 *
 *****************************************************************************/

/**
 * Allocate a partition of secure memory
 *
 * @param       smid_value  Value to use for the SMID register.  Must be 0 for
 *                          kernel mode access.
 * @param[out]  part_no     (If successful) Assigned partition number.
 * @param[out]  part_base   Kernel virtual address of the partition.
 * @param[out]  part_phys   Physical address of the partition.
 *
 * @return
 */
scc_return_t scc_allocate_partition(uint32_t smid_value,
				    int *part_no,
				    void **part_base, uint32_t *part_phys)
{
	uint32_t i;
	os_lock_context_t irq_flags = 0;	/* for IRQ save/restore */
	int local_part;
	scc_return_t retval = SCC_RET_FAIL;
	void *base_addr = NULL;
	uint32_t reg_value;

	local_part = -1;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}
	if (scc_availability == SCC_STATUS_UNIMPLEMENTED) {
		goto out;
	}

	/* ACQUIRE LOCK to prevent others from using crypto or acquiring a
	 * partition.  Note that crypto operations could take a long time, so the
	 * calling process could potentially spin for some time.
	 */
	os_lock_save_context(scc_crypto_lock, irq_flags);

	do {
		/* Find current state of partition ownership */
		reg_value = SCC_READ_REGISTER(SCM_PART_OWNERS_REG);

		/* Search for a free one */
		for (i = 0; i < scc_configuration.partition_count; i++) {
			if (((reg_value >> (SCM_POWN_SHIFT * i))
			     & SCM_POWN_MASK) == SCM_POWN_PART_FREE) {
				break;	/* found a free one */
			}
		}
		if (i == local_part) {
			/* found this one last time, and failed to allocated it */
			pr_debug(KERN_ERR "Partition %d cannot be allocated\n",
				 i);
			goto out;
		}
		if (i >= scc_configuration.partition_count) {
			retval = SCC_RET_INSUFFICIENT_SPACE;	/* all used up */
			goto out;
		}

		pr_debug
		    ("SCC2: Attempting to allocate partition %i, owners:%08x\n",
		     i, SCC_READ_REGISTER(SCM_PART_OWNERS_REG));

		local_part = i;
		/* Store SMID to grab a partition */
		SCC_WRITE_REGISTER(SCM_SMID0_REG +
				   SCM_SMID_WIDTH * (local_part), smid_value);
		mdelay(2);

		/* Now make sure it is ours... ? */
		reg_value = SCC_READ_REGISTER(SCM_PART_OWNERS_REG);

		if (((reg_value >> (SCM_POWN_SHIFT * (local_part)))
		     & SCM_POWN_MASK) != SCM_POWN_PART_OWNED) {
			continue;	/* try for another */
		}
		base_addr = scm_ram_base +
		    (local_part * scc_configuration.partition_size_bytes);
		break;
	} while (1);

out:

	/* Free the lock */
	os_unlock_restore_context(scc_callbacks_lock, irq_flags);

	/* If the base address was assigned, then a partition was successfully
	 * acquired.
	 */
	if (base_addr != NULL) {
		pr_debug("SCC2 Part owners: %08x, engaged: %08x\n",
			 reg_value, SCC_READ_REGISTER(SCM_PART_ENGAGED_REG));
		pr_debug("SCC2 MAP for part %d: %08x\n",
			 local_part,
			 SCC_READ_REGISTER(SCM_ACC0_REG + 8 * local_part));

		/* Copy the partition information to the data structures passed by the
		 * user.
		 */
		*part_no = local_part;
		*part_base = base_addr;
		*part_phys = (uint32_t) scm_ram_phys_base
		    + (local_part * scc_configuration.partition_size_bytes);
		retval = SCC_RET_OK;

		pr_debug
		    ("SCC2 partition engaged.  Kernel address: %p.  Physical "
		     "address: %p, pfn: %08x\n", *part_base, (void *)*part_phys,
		     __phys_to_pfn(*part_phys));
	}

	return retval;
}				/* allocate_partition() */

/**
 * Release a partition of secure memory
 *
 * @param   part_base   Kernel virtual address of the partition to be released.
 *
 * @return  SCC_RET_OK if successful.
 */
scc_return_t scc_release_partition(void *part_base)
{
	uint32_t partition_no;

	if (part_base == NULL) {
		return SCC_RET_FAIL;
	}

	/* Ensure that this is a proper partition location */
	partition_no = SCM_PART_NUMBER((uint32_t) part_base);

	pr_debug("SCC2: Attempting to release partition %i, owners:%08x\n",
		 partition_no, SCC_READ_REGISTER(SCM_PART_OWNERS_REG));

	/* check that the partition is ours to de-establish */
	if (!host_owns_partition(partition_no)) {
		return SCC_RET_FAIL;
	}

	/* TODO: The state of the zeroize engine (SRS field in the Command Status
	 * Register) should be examined before issuing the zeroize command here.
	 * To make the driver thread-safe, a lock should be taken out before
	 * issuing the check and released after the zeroize command has been
	 * issued.
	 */

	/* Zero the partition to release it */
	scc_write_register(SCM_ZCMD_REG,
			   (partition_no << SCM_ZCMD_PART_SHIFT) |
			   (ZCMD_DEALLOC_PART << SCM_ZCMD_CCMD_SHIFT));
	mdelay(2);

	pr_debug("SCC2: done releasing partition %i, owners:%08x\n",
		 partition_no, SCC_READ_REGISTER(SCM_PART_OWNERS_REG));

	/* Check that the de-assignment went correctly */
	if (host_owns_partition(partition_no)) {
		return SCC_RET_FAIL;
	}

	return SCC_RET_OK;
}

/**
 * Diminish the permissions on a partition of secure memory
 *
 * @param   part_base     Kernel virtual address of the partition.
 * @param   permissions   ORed values of the type SCM_PERM_* which will be used as
 *                        initial partition permissions.  SHW API users should use
 *                        the FSL_PERM_* definitions instead.
 *
 * @return  SCC_RET_OK if successful.
 */
scc_return_t scc_diminish_permissions(void *part_base, uint32_t permissions)
{
	uint32_t partition_no;
	uint32_t permissions_requested;
	permissions_requested = permissions;

	/* ensure that this is a proper partition location */
	partition_no = SCM_PART_NUMBER((uint32_t) part_base);

	/* invert the permissions, masking out unused bits */
	permissions = (~permissions) & SCM_PERM_MASK;

	/* attempt to diminish the permissions */
	scc_write_register(SCM_ACC0_REG + 8 * partition_no, permissions);
	mdelay(2);

	/* Reading it back puts it into the original form */
	permissions = SCC_READ_REGISTER(SCM_ACC0_REG + 8 * partition_no);
	if (permissions == permissions_requested) {
		pr_debug("scc_partition_diminish_perms: successful\n");
		pr_debug("scc_partition_diminish_perms: successful\n");
		return SCC_RET_OK;
	}
	pr_debug("scc_partition_diminish_perms: not successful\n");

	return SCC_RET_FAIL;
}

scc_partition_status_t scc_partition_status(void *part_base)
{
	uint32_t part_no;
	uint32_t part_owner;

	/* Determine the partition number from the address */
	part_no = SCM_PART_NUMBER((uint32_t) part_base);

	/* Check if the partition is implemented */
	if (part_no >= scc_configuration.partition_count) {
		return SCC_PART_S_UNUSABLE;
	}

	/* Determine the value of the partition owners register */
	part_owner = (SCC_READ_REGISTER(SCM_PART_OWNERS_REG)
		      >> (part_no * SCM_POWN_SHIFT)) & SCM_POWN_MASK;

	switch (part_owner) {
	case SCM_POWN_PART_OTHER:
		return SCC_PART_S_UNAVAILABLE;
		break;
	case SCM_POWN_PART_FREE:
		return SCC_PART_S_AVAILABLE;
		break;
	case SCM_POWN_PART_OWNED:
		/* could be allocated or engaged*/
		if (partition_engaged(part_no)) {
			return SCC_PART_S_ENGAGED;
		} else {
			return SCC_PART_S_ALLOCATED;
		}
		break;
	case SCM_POWN_PART_UNUSABLE:
	default:
		return SCC_PART_S_UNUSABLE;
		break;
	}
}

/**
 * Calculate the physical address from the kernel virtual address.
 *
 * @param   address Kernel virtual address of data in an Secure Partition.
 * @return  Physical address of said data.
 */
uint32_t scc_virt_to_phys(void *address)
{
	return (uint32_t) address - (uint32_t) scm_ram_base
	    + (uint32_t) scm_ram_phys_base;
}

/**
 * Engage partition of secure memory
 *
 * @param part_base (kernel) Virtual
 * @param UMID NULL, or 16-byte UMID for partition security
 * @param permissions ORed values from fsl_shw_permission_t which
 * will be used as initial partiition permissions.
 *
 * @return SCC_RET_OK if successful.
 */

scc_return_t
scc_engage_partition(void *part_base,
		     const uint8_t *UMID, uint32_t permissions)
{
	uint32_t partition_no;
	uint8_t *UMID_base = part_base + 0x10;
	uint32_t *MAP_base = part_base;
	uint8_t i;

	partition_no = SCM_PART_NUMBER((uint32_t) part_base);

	if (!host_owns_partition(partition_no) ||
	    partition_engaged(partition_no) ||
	    !(SCC_READ_REGISTER(SCM_SMID0_REG + (partition_no * 8)) == 0)) {

		return SCC_RET_FAIL;
	}

	if (UMID != NULL) {
		for (i = 0; i < 16; i++) {
			UMID_base[i] = UMID[i];
		}
	}

	MAP_base[0] = permissions;

	udelay(20);

	/* Check that the partition was engaged correctly, and that it has the
	 * proper permissions.
	 */

	if ((!partition_engaged(partition_no)) ||
	    (permissions !=
	     SCC_READ_REGISTER(SCM_ACC0_REG + 8 * partition_no))) {
		return SCC_RET_FAIL;
	}

	return SCC_RET_OK;
}

/*****************************************************************************/
/* fn scc_init()                                                             */
/*****************************************************************************/
/**
 *  Initialize the driver at boot time or module load time.
 *
 *  Register with the kernel as the interrupt handler for the SCC interrupt
 *  line(s).
 *
 *  Map the SCC's register space into the driver's memory space.
 *
 *  Query the SCC for its configuration and status.  Save the configuration in
 *  #scc_configuration and save the status in #scc_availability.  Called by the
 *  kernel.
 *
 *  Do any locking/wait queue initialization which may be necessary.
 *
 *  The availability fuse may be checked, depending on platform.
 */
static int scc_init(void)
{
	uint32_t smn_status;
	int i;
	int return_value = -EIO;	/* assume error */

	if (scc_availability == SCC_STATUS_INITIAL) {

		/* Set this until we get an initial reading */
		scc_availability = SCC_STATUS_CHECKING;

		/* Initialize the constant for the CRC function */
		scc_init_ccitt_crc();

		/* initialize the callback table */
		for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
			scc_callbacks[i] = 0;
		}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
		mxc_clks_enable(SCC_CLK);
#else
		if (IS_ERR(scc_clk)) {
			return_value = PTR_ERR(scc_clk);
			goto out;
		} else
			clk_enable(scc_clk);
#endif

		/* Set up the hardware access locks */
		scc_callbacks_lock = os_lock_alloc_init();
		scc_crypto_lock = os_lock_alloc_init();
		if (scc_callbacks_lock == NULL || scc_crypto_lock == NULL) {
			os_printk(KERN_ERR
				  "SCC2: Failed to allocate context locks.  Exiting.\n");
			goto out;
		}

		/* Map the SCC (SCM and SMN) memory on the internal bus into
		   kernel address space */
		scc_base = (void *)ioremap(scc_phys_base, SZ_4K);
		if (scc_base == NULL) {
			os_printk(KERN_ERR
				  "SCC2: Register mapping failed.  Exiting.\n");
			goto out;
		}

		/* If that worked, we can try to use the SCC */
		/* Get SCM into 'clean' condition w/interrupts cleared &
		   disabled */
		SCC_WRITE_REGISTER(SCM_INT_CTL_REG, 0);

		/* Clear error status register */
		(void)SCC_READ_REGISTER(SCM_ERR_STATUS_REG);

		/*
		 * There is an SCC.  Determine its current state.  Side effect
		 * is to populate scc_config and scc_availability
		 */
		smn_status = scc_grab_config_values();

		/* Try to set up interrupt handler(s) */
		if (scc_availability != SCC_STATUS_OK) {
			goto out;
		}

		scm_ram_base = (void *)ioremap_nocache(scm_ram_phys_base,
						       scc_configuration.
						       partition_count *
						       scc_configuration.
						       partition_size_bytes);
		if (scm_ram_base == NULL) {
			os_printk(KERN_ERR
				  "SCC2: RAM failed to remap: %p for %d bytes\n",
				  (void *)scm_ram_phys_base,
				  scc_configuration.partition_count *
				  scc_configuration.partition_size_bytes);
			goto out;
		}
		pr_debug("SCC2: RAM at Physical %p / Virtual %p\n",
			 (void *)scm_ram_phys_base, scm_ram_base);

		pr_debug("Secure Partition Table: Found %i partitions\n",
			 scc_configuration.partition_count);

		if (setup_interrupt_handling() != 0) {
			unsigned err_cond;
	    /**
		* The error could be only that the SCM interrupt was
		* not set up.  This interrupt is always masked, so
		* that is not an issue.
		* The SMN's interrupt may be shared on that line, it
		* may be separate, or it may not be wired.  Do what
		* is necessary to check its status.
		* Although the driver is coded for possibility of not
		* having SMN interrupt, the fact that there is one
		* means it should be available and used.
		*/
#ifdef USE_SMN_INTERRUPT
			err_cond = !smn_irq_set;	/* Separate. Check SMN binding */
#elif !defined(NO_SMN_INTERRUPT)
			err_cond = !scm_irq_set;	/* Shared. Check SCM binding */
#else
			err_cond = FALSE;	/*  SMN not wired at all.  Ignore. */
#endif
			if (err_cond) {
				/* setup was not able to set up SMN interrupt */
				scc_availability = SCC_STATUS_UNIMPLEMENTED;
				goto out;
			}
		}

		/* interrupt handling returned non-zero */
		/* Get SMN into 'clean' condition w/interrupts cleared &
		   enabled */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT
				   | SMN_COMMAND_ENABLE_INTERRUPT);

	      out:
		/*
		 * If status is SCC_STATUS_UNIMPLEMENTED or is still
		 * SCC_STATUS_CHECKING, could be leaving here with the driver partially
		 * initialized.  In either case, cleanup (which will mark the SCC as
		 * UNIMPLEMENTED).
		 */
		if (scc_availability == SCC_STATUS_CHECKING ||
		    scc_availability == SCC_STATUS_UNIMPLEMENTED) {
			scc_cleanup();
		} else {
			return_value = 0;	/* All is well */
		}
	}
	/* ! STATUS_INITIAL */
	os_printk(KERN_ALERT "SCC2: Driver Status is %s\n",
		  (scc_availability == SCC_STATUS_INITIAL) ? "INITIAL" :
		  (scc_availability == SCC_STATUS_CHECKING) ? "CHECKING" :
		  (scc_availability ==
		   SCC_STATUS_UNIMPLEMENTED) ? "UNIMPLEMENTED"
		  : (scc_availability ==
		     SCC_STATUS_OK) ? "OK" : (scc_availability ==
					      SCC_STATUS_FAILED) ? "FAILED" :
		  "UNKNOWN");

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
			mxc_clks_disable(SCC_CLK);
#else
			if (!IS_ERR(scc_clk))
				clk_disable(scc_clk);
#endif

	return return_value;
}				/* scc_init */

/*****************************************************************************/
/* fn scc_cleanup()                                                          */
/*****************************************************************************/
/**
 * Perform cleanup before driver/module is unloaded by setting the machine
 * state close to what it was when the driver was loaded.  This function is
 * called when the kernel is shutting down or when this driver is being
 * unloaded.
 *
 * A driver like this should probably never be unloaded, especially if there
 * are other module relying upon the callback feature for monitoring the SCC
 * status.
 *
 * In any case, cleanup the callback table (by clearing out all of the
 * pointers).  Deregister the interrupt handler(s).  Unmap SCC registers.
 *
 * Note that this will not release any partitions that have been allocated.
 *
 */
static void scc_cleanup(void)
{
	int i;

    /******************************************************/

	/* Mark the driver / SCC as unusable. */
	scc_availability = SCC_STATUS_UNIMPLEMENTED;

	/* Clear out callback table */
	for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
		scc_callbacks[i] = 0;
	}

	/* If SCC has been mapped in, clean it up and unmap it */
	if (scc_base) {
		/* For the SCM, disable interrupts. */
		SCC_WRITE_REGISTER(SCM_INT_CTL_REG, 0);

		/* For the SMN, clear and disable interrupts */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT);
	}

	/* Now that interrupts cannot occur, disassociate driver from the interrupt
	 * lines.
	 */

	/* Deregister SCM interrupt handler */
	if (scm_irq_set) {
		os_deregister_interrupt(irq_scm);
	}

	/* Deregister SMN interrupt handler */
	if (smn_irq_set) {
#ifdef USE_SMN_INTERRUPT
		os_deregister_interrupt(irq_smn);
#endif
	}

	/* Finally, release the mapped memory */
	iounmap(scm_ram_base);

	if (scc_callbacks_lock != NULL)
		os_lock_deallocate(scc_callbacks_lock);

	if (scc_crypto_lock != NULL)
		os_lock_deallocate(scc_crypto_lock);

    /*Disabling SCC Clock*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
			mxc_clks_disable(SCC_CLK);
#else
			if (!IS_ERR(scc_clk))
				clk_disable(scc_clk);
			clk_put(scc_clk);
#endif
	pr_debug("SCC2 driver cleaned up.\n");

}				/* scc_cleanup */

/*****************************************************************************/
/* fn scc_get_configuration()                                                */
/*****************************************************************************/
scc_config_t *scc_get_configuration(void)
{
	/*
	 * If some other driver calls scc before the kernel does, make sure that
	 * this driver's initialization is performed.
	 */
	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

    /**
     * If there is no SCC, yet the driver exists, the value -1 will be in
     * the #scc_config_t fields for other than the driver versions.
     */
	return &scc_configuration;
}				/* scc_get_configuration */

/*****************************************************************************/
/* fn scc_zeroize_memories()                                                 */
/*****************************************************************************/
scc_return_t scc_zeroize_memories(void)
{
	scc_return_t return_status = SCC_RET_FAIL;

	return return_status;
}				/* scc_zeroize_memories */

/*****************************************************************************/
/* fn scc_set_sw_alarm()                                                     */
/*****************************************************************************/
void scc_set_sw_alarm(void)
{

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* Update scc_availability based on current SMN status.  This might
	 * perform callbacks.
	 */
	(void)scc_update_state();

	/* if everything is OK, make it fail */
	if (scc_availability == SCC_STATUS_OK) {

		/* sound the alarm (and disable SMN interrupts */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_SET_SOFTWARE_ALARM);

		scc_availability = SCC_STATUS_FAILED;	/* Remember what we've done */

		/* In case SMN interrupt is not available, tell the world */
		scc_perform_callbacks();
	}

	return;
}				/* scc_set_sw_alarm */

/*****************************************************************************/
/* fn scc_monitor_security_failure()                                         */
/*****************************************************************************/
scc_return_t scc_monitor_security_failure(void callback_func(void))
{
	int i;
	os_lock_context_t irq_flags;	/* for IRQ save/restore */
	scc_return_t return_status = SCC_RET_TOO_MANY_FUNCTIONS;
	int function_stored = FALSE;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* Acquire lock of callbacks table.  Could be spin_lock_irq() if this
	 * routine were just called from base (not interrupt) level
	 */
	os_lock_save_context(scc_callbacks_lock, irq_flags);

	/* Search through table looking for empty slot */
	for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
		if (scc_callbacks[i] == callback_func) {
			if (function_stored) {
				/* Saved duplicate earlier.  Clear this later one. */
				scc_callbacks[i] = NULL;
			}
			/* Exactly one copy is now stored */
			return_status = SCC_RET_OK;
			break;
		} else if (scc_callbacks[i] == NULL && !function_stored) {
			/* Found open slot.  Save it and remember */
			scc_callbacks[i] = callback_func;
			return_status = SCC_RET_OK;
			function_stored = TRUE;
		}
	}

	/* Free the lock */
	os_unlock_restore_context(scc_callbacks_lock, irq_flags);

	return return_status;
}				/* scc_monitor_security_failure */

/*****************************************************************************/
/* fn scc_stop_monitoring_security_failure()                                 */
/*****************************************************************************/
void scc_stop_monitoring_security_failure(void callback_func(void))
{
	os_lock_context_t irq_flags;	/* for IRQ save/restore */
	int i;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* Acquire lock of callbacks table.  Could be spin_lock_irq() if this
	 * routine were just called from base (not interrupt) level
	 */
	os_lock_save_context(scc_callbacks_lock, irq_flags);

	/* Search every entry of the table for this function */
	for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
		if (scc_callbacks[i] == callback_func) {
			scc_callbacks[i] = NULL;	/* found instance - clear it out */
			break;
		}
	}

	/* Free the lock */
	os_unlock_restore_context(scc_callbacks_lock, irq_flags);

	return;
}				/* scc_stop_monitoring_security_failure */

/*****************************************************************************/
/* fn scc_read_register()                                                    */
/*****************************************************************************/
scc_return_t scc_read_register(int register_offset, uint32_t * value)
{
	scc_return_t return_status = SCC_RET_FAIL;
	uint32_t smn_status;
	uint32_t scm_status;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* First layer of protection -- completely unaccessible SCC */
	if (scc_availability != SCC_STATUS_UNIMPLEMENTED) {

		/* Second layer -- that offset is valid */
		if (register_offset != SMN_BB_DEC_REG &&	/* write only! */
		    check_register_offset(register_offset) == SCC_RET_OK) {

			/* Get current status / update local state */
			smn_status = scc_update_state();
			scm_status = SCC_READ_REGISTER(SCM_STATUS_REG);

			/*
			 * Third layer - verify that the register being requested is
			 * available in the current state of the SCC.
			 */
			if ((return_status =
			     check_register_accessible(register_offset,
						       smn_status,
						       scm_status)) ==
			    SCC_RET_OK) {
				*value = SCC_READ_REGISTER(register_offset);
			}
		}
	}

	return return_status;
}				/* scc_read_register */

/*****************************************************************************/
/* fn scc_write_register()                                                   */
/*****************************************************************************/
scc_return_t scc_write_register(int register_offset, uint32_t value)
{
	scc_return_t return_status = SCC_RET_FAIL;
	uint32_t smn_status;
	uint32_t scm_status;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* First layer of protection -- completely unaccessible SCC */
	if (scc_availability != SCC_STATUS_UNIMPLEMENTED) {

		/* Second layer -- that offset is valid */
		if (!((register_offset == SCM_STATUS_REG) ||	/* These registers are */
		      (register_offset == SCM_VERSION_REG) ||	/*  Read Only */
		      (register_offset == SMN_BB_CNT_REG) ||
		      (register_offset == SMN_TIMER_REG)) &&
		    check_register_offset(register_offset) == SCC_RET_OK) {

			/* Get current status / update local state */
			smn_status = scc_update_state();
			scm_status = SCC_READ_REGISTER(SCM_STATUS_REG);

			/*
			 * Third layer - verify that the register being requested is
			 * available in the current state of the SCC.
			 */
			if (check_register_accessible
			    (register_offset, smn_status, scm_status) == 0) {
				SCC_WRITE_REGISTER(register_offset, value);
				return_status = SCC_RET_OK;
			}
		}
	}

	return return_status;
}				/* scc_write_register() */

/******************************************************************************
 *
 *  Function Implementations - Internal
 *
 *****************************************************************************/

/*****************************************************************************/
/* fn scc_irq()                                                              */
/*****************************************************************************/
/**
 * This is the interrupt handler for the SCC.
 *
 * This function checks the SMN Status register to see whether it
 * generated the interrupt, then it checks the SCM Status register to
 * see whether it needs attention.
 *
 * If an SMN Interrupt is active, then the SCC state set to failure, and
 * #scc_perform_callbacks() is invoked to notify any interested parties.
 *
 * The SCM Interrupt should be masked, as this driver uses polling to determine
 * when the SCM has completed a crypto or zeroing operation.  Therefore, if the
 * interrupt is active, the driver will just clear the interrupt and (re)mask.
 */
OS_DEV_ISR(scc_irq)
{
	uint32_t smn_status;
	uint32_t scm_status;
	int handled = 0;	/* assume interrupt isn't from SMN */
#if defined(USE_SMN_INTERRUPT)
	int smn_irq = irq_smn;	/* SMN interrupt is on a line by itself */
#elif defined (NO_SMN_INTERRUPT)
	int smn_irq = -1;	/* not wired to CPU at all */
#else
	int smn_irq = irq_scm;	/* SMN interrupt shares a line with SCM */
#endif

	/* Update current state... This will perform callbacks... */
	smn_status = scc_update_state();

	/* SMN is on its own interrupt line.  Verify the IRQ was triggered
	 * before clearing the interrupt and marking it handled.  */
	if ((os_dev_get_irq() == smn_irq) &&
	    (smn_status & SMN_STATUS_SMN_STATUS_IRQ)) {
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT);
		handled++;	/* tell kernel that interrupt was handled */
	}

	/* Check on the health of the SCM */
	scm_status = SCC_READ_REGISTER(SCM_STATUS_REG);

	/* The driver masks interrupts, so this should never happen. */
	if (os_dev_get_irq() == irq_scm) {
		/* but if it does, try to prevent it in the future */
		SCC_WRITE_REGISTER(SCM_INT_CTL_REG, 0);
		handled++;
	}

	/* Any non-zero value of handled lets kernel know we got something */
	os_dev_isr_return(handled);
}

/*****************************************************************************/
/* fn scc_perform_callbacks()                                                */
/*****************************************************************************/
/** Perform callbacks registered by #scc_monitor_security_failure().
 *
 *  Make sure callbacks only happen once...  Since there may be some reason why
 *  the interrupt isn't generated, this routine could be called from base(task)
 *  level.
 *
 *  One at a time, go through #scc_callbacks[] and call any non-null pointers.
 */
static void scc_perform_callbacks(void)
{
	static int callbacks_performed = 0;
	unsigned long irq_flags;	/* for IRQ save/restore */
	int i;

	/* Acquire lock of callbacks table and callbacks_performed flag */
	os_lock_save_context(scc_callbacks_lock, irq_flags);

	if (!callbacks_performed) {
		callbacks_performed = 1;

		/* Loop over all of the entries in the table */
		for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
			/* If not null, ... */
			if (scc_callbacks[i]) {
				scc_callbacks[i] ();	/* invoke the callback routine */
			}
		}
	}

	os_unlock_restore_context(scc_callbacks_lock, irq_flags);

	return;
}

/*****************************************************************************/
/* fn scc_update_state()                                                     */
/*****************************************************************************/
/**
 * Make certain SCC is still running.
 *
 * Side effect is to update #scc_availability and, if the state goes to failed,
 * run #scc_perform_callbacks().
 *
 * (If #SCC_BRINGUP is defined, bring SCC to secure state if it is found to be
 * in health check state)
 *
 * @return Current value of #SMN_STATUS_REG register.
 */
static uint32_t scc_update_state(void)
{
	uint32_t smn_status_register = SMN_STATE_FAIL;
	int smn_state;

	/* if FAIL or UNIMPLEMENTED, don't bother */
	if (scc_availability == SCC_STATUS_CHECKING ||
	    scc_availability == SCC_STATUS_OK) {

		smn_status_register = SCC_READ_REGISTER(SMN_STATUS_REG);
		smn_state = smn_status_register & SMN_STATUS_STATE_MASK;

#ifdef SCC_BRINGUP
		/* If in Health Check while booting, try to 'bringup' to Secure mode */
		if (scc_availability == SCC_STATUS_CHECKING &&
		    smn_state == SMN_STATE_HEALTH_CHECK) {
			/* Code up a simple algorithm for the ASC */
			SCC_WRITE_REGISTER(SMN_SEQ_START_REG, 0xaaaa);
			SCC_WRITE_REGISTER(SMN_SEQ_END_REG, 0x5555);
			SCC_WRITE_REGISTER(SMN_SEQ_CHECK_REG, 0x5555);
			/* State should be SECURE now */
			smn_status_register = SCC_READ_REGISTER(SMN_STATUS);
			smn_state = smn_status_register & SMN_STATUS_STATE_MASK;
		}
#endif

		/*
		 * State should be SECURE or NON_SECURE for operation of the part.  If
		 * FAIL, mark failed (i.e. limited access to registers).  Any other
		 * state, mark unimplemented, as the SCC is unuseable.
		 */
		if (smn_state == SMN_STATE_SECURE
		    || smn_state == SMN_STATE_NON_SECURE) {
			/* Healthy */
			scc_availability = SCC_STATUS_OK;
		} else if (smn_state == SMN_STATE_FAIL) {
			scc_availability = SCC_STATUS_FAILED;	/* uh oh - unhealthy */
			scc_perform_callbacks();
			os_printk(KERN_ERR "SCC2: SCC went into FAILED mode\n");
		} else {
			/* START, ZEROIZE RAM, HEALTH CHECK, or unknown */
			scc_availability = SCC_STATUS_UNIMPLEMENTED;	/* unuseable */
			os_printk(KERN_ERR
				  "SCC2: SCC declared UNIMPLEMENTED\n");
		}
	}
	/* if availability is initial or ok */
	return smn_status_register;
}

/*****************************************************************************/
/* fn scc_init_ccitt_crc()                                                   */
/*****************************************************************************/
/**
 * Populate the partial CRC lookup table.
 *
 * @return   none
 *
 */
static void scc_init_ccitt_crc(void)
{
	int dividend;		/* index for lookup table */
	uint16_t remainder;	/* partial value for a given dividend */
	int bit;		/* index into bits of a byte */

	/*
	 * Compute the remainder of each possible dividend.
	 */
	for (dividend = 0; dividend < 256; ++dividend) {
		/*
		 * Start with the dividend followed by zeros.
		 */
		remainder = dividend << (8);

		/*
		 * Perform modulo-2 division, a bit at a time.
		 */
		for (bit = 8; bit > 0; --bit) {
			/*
			 * Try to divide the current data bit.
			 */
			if (remainder & 0x8000) {
				remainder = (remainder << 1) ^ CRC_POLYNOMIAL;
			} else {
				remainder = (remainder << 1);
			}
		}

		/*
		 * Store the result into the table.
		 */
		scc_crc_lookup_table[dividend] = remainder;
	}

}				/* scc_init_ccitt_crc() */

/*****************************************************************************/
/* fn grab_config_values()                                                   */
/*****************************************************************************/
/**
 * grab_config_values() will read the SCM Configuration and SMN Status
 * registers and store away version and size information for later use.
 *
 * @return The current value of the SMN Status register.
 */
static uint32_t scc_grab_config_values(void)
{
	uint32_t scm_version_register;
	uint32_t smn_status_register = SMN_STATE_FAIL;

	if (scc_availability != SCC_STATUS_CHECKING) {
		goto out;
	}
	scm_version_register = SCC_READ_REGISTER(SCM_VERSION_REG);
	pr_debug("SCC2 Driver: SCM version is 0x%08x\n", scm_version_register);

	/* Get SMN status and update scc_availability */
	smn_status_register = scc_update_state();
	pr_debug("SCC2 Driver: SMN status is 0x%08x\n", smn_status_register);

	/* save sizes and versions information for later use */
	scc_configuration.block_size_bytes = 16;	/* BPCP ? */
	scc_configuration.partition_count =
	    1 + ((scm_version_register & SCM_VER_NP_MASK) >> SCM_VER_NP_SHIFT);
	scc_configuration.partition_size_bytes =
	    1 << ((scm_version_register & SCM_VER_BPP_MASK) >>
		  SCM_VER_BPP_SHIFT);
	scc_configuration.scm_version =
	    (scm_version_register & SCM_VER_MAJ_MASK) >> SCM_VER_MAJ_SHIFT;
	scc_configuration.smn_version =
	    (smn_status_register & SMN_STATUS_VERSION_ID_MASK)
	    >> SMN_STATUS_VERSION_ID_SHIFT;
	if (scc_configuration.scm_version != SCM_MAJOR_VERSION_2) {
		scc_availability = SCC_STATUS_UNIMPLEMENTED;	/* Unknown version */
	}

      out:
	return smn_status_register;
}				/* grab_config_values */

/*****************************************************************************/
/* fn setup_interrupt_handling()                                             */
/*****************************************************************************/
/**
 * Register the SCM and SMN interrupt handlers.
 *
 * Called from #scc_init()
 *
 * @return 0 on success
 */
static int setup_interrupt_handling(void)
{
	int smn_error_code = -1;
	int scm_error_code = -1;

	/* Disnable SCM interrupts */
	SCC_WRITE_REGISTER(SCM_INT_CTL_REG, 0);

#ifdef USE_SMN_INTERRUPT
	/* Install interrupt service routine for SMN. */
	smn_error_code = os_register_interrupt(SCC_DRIVER_NAME,
					       irq_smn, scc_irq);
	if (smn_error_code != 0) {
		os_printk(KERN_ERR
			  "SCC2 Driver: Error installing SMN Interrupt Handler: %d\n",
			  smn_error_code);
	} else {
		smn_irq_set = 1;	/* remember this for cleanup */
		/* Enable SMN interrupts */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT |
				   SMN_COMMAND_ENABLE_INTERRUPT);
	}
#else
	smn_error_code = 0;	/* no problems... will handle later */
#endif

	/*
	 * Install interrupt service routine for SCM (or both together).
	 */
	scm_error_code = os_register_interrupt(SCC_DRIVER_NAME,
					       irq_scm, scc_irq);
	if (scm_error_code != 0) {
#ifndef MXC
		os_printk(KERN_ERR
			  "SCC2 Driver: Error installing SCM Interrupt Handler: %d\n",
			  scm_error_code);
#else
		os_printk(KERN_ERR
			  "SCC2 Driver: Error installing SCC Interrupt Handler: %d\n",
			  scm_error_code);
#endif
	} else {
		scm_irq_set = 1;	/* remember this for cleanup */
#if defined(USE_SMN_INTERRUPT) && !defined(NO_SMN_INTERRUPT)
		/* Enable SMN interrupts */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT |
				   SMN_COMMAND_ENABLE_INTERRUPT);
#endif
	}

	/* Return an error if one was encountered */
	return scm_error_code ? scm_error_code : smn_error_code;
}				/* setup_interrupt_handling */

/*****************************************************************************/
/* fn scc_do_crypto()                                                        */
/*****************************************************************************/
/** Have the SCM perform the crypto function.
 *
 * Set up length register, and the store @c scm_control into control register
 * to kick off the operation.  Wait for completion, gather status, clear
 * interrupt / status.
 *
 * @param byte_count  number of bytes to perform in this operation
 * @param scm_command Bit values to be set in @c SCM_CCMD_REG register
 *
 * @return 0 on success, value of #SCM_ERR_STATUS_REG on failure
 */
static uint32_t scc_do_crypto(int byte_count, uint32_t scm_command)
{
	int block_count = byte_count / SCC_BLOCK_SIZE_BYTES();
	uint32_t crypto_status;
	scc_return_t ret;

	/* This seems to be necessary in order to allow subsequent cipher
	 * operations to succeed when a partition is deallocated/reallocated!
	 */
	(void)SCC_READ_REGISTER(SCM_STATUS_REG);

	/* In length register, 0 means 1, etc. */
	scm_command |= (block_count - 1) << SCM_CCMD_LENGTH_SHIFT;

	/* set modes and kick off the operation */
	SCC_WRITE_REGISTER(SCM_CCMD_REG, scm_command);

	ret = scc_wait_completion(&crypto_status);

	/* Only done bit should be on */
	if (crypto_status & SCM_STATUS_ERR) {
		/* Replace with error status instead */
		crypto_status = SCC_READ_REGISTER(SCM_ERR_STATUS_REG);
		pr_debug("SCM Failure: 0x%x\n", crypto_status);
		if (crypto_status == 0) {
			/* That came up 0.  Turn on arbitrary bit to signal error. */
			crypto_status = SCM_ERRSTAT_ILM;
		}
	} else {
		crypto_status = 0;
	}
	pr_debug("SCC2: Done waiting.\n");

	return crypto_status;
}

/**
 * Encrypt a region of secure memory.
 *
 * @param   part_base    Kernel virtual address of the partition.
 * @param   offset_bytes Offset from the start of the partition to the plaintext
 *                       data.
 * @param   byte_count   Length of the region (octets).
 * @param   black_data   Physical location to store the encrypted data.
 * @param   IV           Value to use for the IV.
 * @param   cypher_mode  Cyphering mode to use, specified by type
 *                       #scc_cypher_mode_t
 *
 * @return  SCC_RET_OK if successful.
 */
scc_return_t
scc_encrypt_region(uint32_t part_base, uint32_t offset_bytes,
		   uint32_t byte_count, uint8_t *black_data,
		   uint32_t *IV, scc_cypher_mode_t cypher_mode)
{
	os_lock_context_t irq_flags;	/* for IRQ save/restore */
	scc_return_t status = SCC_RET_OK;
	uint32_t crypto_status;
	uint32_t scm_command;
	int offset_blocks = offset_bytes / SCC_BLOCK_SIZE_BYTES();

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
				mxc_clks_enable(SCC_CLK);
#else
		if (IS_ERR(scc_clk)) {
			status = SCC_RET_FAIL;
			goto out;
		} else
			clk_enable(scc_clk);
#endif

	scm_command = ((offset_blocks << SCM_CCMD_OFFSET_SHIFT) |
		       (SCM_PART_NUMBER(part_base) << SCM_CCMD_PART_SHIFT));

	switch (cypher_mode) {
	case SCC_CYPHER_MODE_CBC:
		scm_command |= SCM_CCMD_AES_ENC_CBC;
		break;
	case SCC_CYPHER_MODE_ECB:
		scm_command |= SCM_CCMD_AES_ENC_ECB;
		break;
	default:
		status = SCC_RET_FAIL;
		break;
	}

	pr_debug("Received encrypt request.  SCM_C_BLACK_ST_REG: %p, "
		 "scm_Command: %08x, length: %i (part_base: %08x, "
		 "offset: %i)\n",
		 black_data, scm_command, byte_count, part_base, offset_blocks);

	if (status != SCC_RET_OK)
		goto out;

	/* ACQUIRE LOCK to prevent others from using crypto or releasing slot */
	os_lock_save_context(scc_crypto_lock, irq_flags);

	if (status == SCC_RET_OK) {
		SCC_WRITE_REGISTER(SCM_C_BLACK_ST_REG, (uint32_t) black_data);

		/* Only write the IV if it will actually be used */
		if (cypher_mode == SCC_CYPHER_MODE_CBC) {
			/* Write the IV register */
			SCC_WRITE_REGISTER(SCM_AES_CBC_IV0_REG, *(IV));
			SCC_WRITE_REGISTER(SCM_AES_CBC_IV1_REG, *(IV + 1));
			SCC_WRITE_REGISTER(SCM_AES_CBC_IV2_REG, *(IV + 2));
			SCC_WRITE_REGISTER(SCM_AES_CBC_IV3_REG, *(IV + 3));
		}

		/* Set modes and kick off the encryption */
		crypto_status = scc_do_crypto(byte_count, scm_command);

		if (crypto_status != 0) {
			pr_debug("SCM encrypt red crypto failure: 0x%x\n",
				 crypto_status);
		} else {
			status = SCC_RET_OK;
			pr_debug("SCC2: Encrypted %d bytes\n", byte_count);
		}
	}

	os_unlock_restore_context(scc_crypto_lock, irq_flags);

out:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
					mxc_clks_disable(SCC_CLK);
#else
					if (!IS_ERR(scc_clk))
						clk_disable(scc_clk);
#endif

	return status;
}

/* Decrypt a region into secure memory
 *
 * @param   part_base    Kernel virtual address of the partition.
 * @param   offset_bytes Offset from the start of the partition to store the
 *                       plaintext data.
 * @param   byte_counts  Length of the region (octets).
 * @param   black_data   Physical location of the encrypted data.
 * @param   IV           Value to use for the IV.
 * @param   cypher_mode  Cyphering mode to use, specified by type
 *                       #scc_cypher_mode_t
 *
 * @return  SCC_RET_OK if successful.
 */
scc_return_t
scc_decrypt_region(uint32_t part_base, uint32_t offset_bytes,
		   uint32_t byte_count, uint8_t *black_data,
		   uint32_t *IV, scc_cypher_mode_t cypher_mode)
{
	os_lock_context_t irq_flags;	/* for IRQ save/restore */
	scc_return_t status = SCC_RET_OK;
	uint32_t crypto_status;
	uint32_t scm_command;
	int offset_blocks = offset_bytes / SCC_BLOCK_SIZE_BYTES();

    /*Enabling SCC clock.*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
			mxc_clks_enable(SCC_CLK);
#else
		if (IS_ERR(scc_clk)) {
			status = SCC_RET_FAIL;
			goto out;
		} else
			clk_enable(scc_clk);
#endif
	scm_command = ((offset_blocks << SCM_CCMD_OFFSET_SHIFT) |
		       (SCM_PART_NUMBER(part_base) << SCM_CCMD_PART_SHIFT));

	switch (cypher_mode) {
	case SCC_CYPHER_MODE_CBC:
		scm_command |= SCM_CCMD_AES_DEC_CBC;
		break;
	case SCC_CYPHER_MODE_ECB:
		scm_command |= SCM_CCMD_AES_DEC_ECB;
		break;
	default:
		status = SCC_RET_FAIL;
		break;
	}

	pr_debug("Received decrypt request.  SCM_C_BLACK_ST_REG: %p, "
		 "scm_Command: %08x, length: %i (part_base: %08x, "
		 "offset: %i)\n",
		 black_data, scm_command, byte_count, part_base, offset_blocks);

	if (status != SCC_RET_OK)
		goto out;

	/* ACQUIRE LOCK to prevent others from using crypto or releasing slot */
	os_lock_save_context(scc_crypto_lock, irq_flags);

	if (status == SCC_RET_OK) {
		status = SCC_RET_FAIL;	/* reset expectations */
		SCC_WRITE_REGISTER(SCM_C_BLACK_ST_REG, (uint32_t) black_data);

		/* Write the IV register */
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV0_REG, *(IV));
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV1_REG, *(IV + 1));
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV2_REG, *(IV + 2));
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV3_REG, *(IV + 3));

		/* Set modes and kick off the decryption */
		crypto_status = scc_do_crypto(byte_count, scm_command);

		if (crypto_status != 0) {
			pr_debug("SCM decrypt black crypto failure: 0x%x\n",
				 crypto_status);
		} else {
			status = SCC_RET_OK;
			pr_debug("SCC2: Decrypted %d bytes\n", byte_count);
		}
	}

	os_unlock_restore_context(scc_crypto_lock, irq_flags);
out:
    /*Disabling the Clock when the driver is not in use.*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
			mxc_clks_disable(SCC_CLK);
#else
			if (!IS_ERR(scc_clk))
				clk_disable(scc_clk);
#endif
	return status;
}

/*****************************************************************************/
/* fn host_owns_partition()                                                  */
/*****************************************************************************/
/**
 * Determine if the host owns a given partition.
 *
 * @internal
 *
 * @param part_no       Partition number to query
 *
 * @return TRUE if the host owns the partition, FALSE otherwise.
 */

static uint32_t host_owns_partition(uint32_t part_no)
{
	uint32_t value;

	if (part_no < scc_configuration.partition_count) {

		/* Check the partition owners register */
		value = SCC_READ_REGISTER(SCM_PART_OWNERS_REG);
		if (((value >> (part_no * SCM_POWN_SHIFT)) & SCM_POWN_MASK)
		    == SCM_POWN_PART_OWNED)
			return TRUE;
	}
	return FALSE;
}

/*****************************************************************************/
/* fn partition_engaged()                                                    */
/*****************************************************************************/
/**
 * Determine if the given partition is engaged.
 *
 * @internal
 *
 * @param part_no       Partition number to query
 *
 * @return TRUE if the partition is engaged, FALSE otherwise.
 */

static uint32_t partition_engaged(uint32_t part_no)
{
	uint32_t value;

	if (part_no < scc_configuration.partition_count) {

		/* Check the partition engaged register */
		value = SCC_READ_REGISTER(SCM_PART_ENGAGED_REG);
		if (((value >> (part_no * SCM_PENG_SHIFT)) & 0x1)
		    == SCM_PENG_ENGAGED)
			return TRUE;
	}
	return FALSE;
}

/*****************************************************************************/
/* fn scc_wait_completion()                                                  */
/*****************************************************************************/
/**
 * Poll looking for end-of-cipher indication. Only used
 * if @c SCC_SCM_SLEEP is not defined.
 *
 * @internal
 *
 * On a Tahiti, crypto under 230 or so bytes is done after the first loop, all
 * the way up to five sets of spins for 1024 bytes.  (8- and 16-byte functions
 * are done when we first look.  Zeroizing takes one pass around.
 *
 * @param   scm_status      Address of the SCM_STATUS register
 *
 * @return  A return code of type #scc_return_t
 */
static scc_return_t scc_wait_completion(uint32_t * scm_status)
{
	scc_return_t ret;
	int done;
	int i = 0;

	/* check for completion by polling */
	do {
		done = is_cipher_done(scm_status);
		if (done)
			break;
		/* TODO: shorten this delay */
		udelay(1000);
	} while (i++ < SCC_CIPHER_MAX_POLL_COUNT);

	pr_debug("SCC2: Polled DONE %d times\n", i);
	if (!done) {
		ret = SCC_RET_FAIL;
	}

	return ret;
}				/* scc_wait_completion() */

/*****************************************************************************/
/* fn is_cipher_done()                                                       */
/*****************************************************************************/
/**
 * This function returns non-zero if SCM Status register indicates
 * that a cipher has terminated or some other interrupt-generating
 * condition has occurred.
 *
 * @param scm_status    Address of the SCM STATUS register
 *
 * @return  0 if cipher operations are finished
 */
static int is_cipher_done(uint32_t * scm_status)
{
	register unsigned status;
	register int cipher_done;

	*scm_status = SCC_READ_REGISTER(SCM_STATUS_REG);
	status = (*scm_status & SCM_STATUS_SRS_MASK) >> SCM_STATUS_SRS_SHIFT;

	/*
	 * Done when SCM is not in 'currently performing a function' states.
	 */
	cipher_done = ((status != SCM_STATUS_SRS_ZBUSY)
		       && (status != SCM_STATUS_SRS_CBUSY)
		       && (status != SCM_STATUS_SRS_ABUSY));

	return cipher_done;
}				/* is_cipher_done() */

/*****************************************************************************/
/* fn offset_within_smn()                                                    */
/*****************************************************************************/
/*!
 *  Check that the offset is with the bounds of the SMN register set.
 *
 *  @param[in]  register_offset    register offset of SMN.
 *
 *  @return   1 if true, 0 if false (not within SMN)
 */
static inline int offset_within_smn(uint32_t register_offset)
{
	return ((register_offset >= SMN_STATUS_REG)
		&& (register_offset <= SMN_HAC_REG));
}

/*****************************************************************************/
/* fn offset_within_scm()                                                    */
/*****************************************************************************/
/*!
 *  Check that the offset is with the bounds of the SCM register set.
 *
 *  @param[in]  register_offset    Register offset of SCM
 *
 *  @return   1 if true, 0 if false (not within SCM)
 */
static inline int offset_within_scm(uint32_t register_offset)
{
	return 1;		/* (register_offset >= SCM_RED_START)
				   && (register_offset < scm_highest_memory_address); */
/* Although this would cause trouble for zeroize testing, this change would
 * close a security hole which currently allows any kernel program to access
 * any location in RED RAM.  Perhaps enforce in non-SCC_DEBUG compiles?
 && (register_offset <= SCM_INIT_VECTOR_1); */
}

/*****************************************************************************/
/* fn check_register_accessible()                                            */
/*****************************************************************************/
/**
 *  Given the current SCM and SMN status, verify that access to the requested
 *  register should be OK.
 *
 *  @param[in]   register_offset  register offset within SCC
 *  @param[in]   smn_status  recent value from #SMN_STATUS_REG
 *  @param[in]   scm_status  recent value from #SCM_STATUS_REG
 *
 *  @return   #SCC_RET_OK if ok, #SCC_RET_FAIL if not
 */
static scc_return_t
check_register_accessible(uint32_t register_offset, uint32_t smn_status,
			  uint32_t scm_status)
{
	int error_code = SCC_RET_FAIL;

	/* Verify that the register offset passed in is not among the verboten set
	 * if the SMN is in Fail mode.
	 */
	if (offset_within_smn(register_offset)) {
		if ((smn_status & SMN_STATUS_STATE_MASK) == SMN_STATE_FAIL) {
			if (!((register_offset == SMN_STATUS_REG) ||
			      (register_offset == SMN_COMMAND_REG) ||
			      (register_offset == SMN_SEC_VIO_REG))) {
				pr_debug
				    ("SCC2 Driver: Note: Security State is in FAIL state.\n");
			} /* register not a safe one */
			else {
				/* SMN is in  FAIL, but register is a safe one */
				error_code = SCC_RET_OK;
			}
		} /* State is FAIL */
		else {
			/* State is not fail.  All registers accessible. */
			error_code = SCC_RET_OK;
		}
	}
	/* offset within SMN */
	/*  Not SCM register.  Check for SCM busy. */
	else if (offset_within_scm(register_offset)) {
		/* This is the 'cannot access' condition in the SCM */
		if (0		/* (scm_status & SCM_STATUS_BUSY) */
		    /* these are always available  - rest fail on busy */
		    && !((register_offset == SCM_STATUS_REG) ||
			 (register_offset == SCM_ERR_STATUS_REG) ||
			 (register_offset == SCM_INT_CTL_REG) ||
			 (register_offset == SCM_VERSION_REG))) {
			pr_debug
			    ("SCC2 Driver: Note: Secure Memory is in BUSY state.\n");
		} /* status is busy & register inaccessible */
		else {
			error_code = SCC_RET_OK;
		}
	}
	/* offset within SCM */
	return error_code;

}				/* check_register_accessible() */

/*****************************************************************************/
/* fn check_register_offset()                                                */
/*****************************************************************************/
/**
 *  Check that the offset is with the bounds of the SCC register set.
 *
 *  @param[in]  register_offset    register offset of SMN.
 *
 * #SCC_RET_OK if ok, #SCC_RET_FAIL if not
 */
static scc_return_t check_register_offset(uint32_t register_offset)
{
	int return_value = SCC_RET_FAIL;

	/* Is it valid word offset ? */
	if (SCC_BYTE_OFFSET(register_offset) == 0) {
		/* Yes. Is register within SCM? */
		if (offset_within_scm(register_offset)) {
			return_value = SCC_RET_OK;	/* yes, all ok */
		}
		/* Not in SCM.  Now look within the SMN */
		else if (offset_within_smn(register_offset)) {
			return_value = SCC_RET_OK;	/* yes, all ok */
		}
	}

	return return_value;
}

#ifdef SCC_REGISTER_DEBUG

/**
 * Names of the SCC Registers, indexed by register number
 */
static char *scc_regnames[] = {
	"SCM_VERSION_REG",
	"0x04",
	"SCM_INT_CTL_REG",
	"SCM_STATUS_REG",
	"SCM_ERR_STATUS_REG",
	"SCM_FAULT_ADR_REG",
	"SCM_PART_OWNERS_REG",
	"SCM_PART_ENGAGED_REG",
	"SCM_UNIQUE_ID0_REG",
	"SCM_UNIQUE_ID1_REG",
	"SCM_UNIQUE_ID2_REG",
	"SCM_UNIQUE_ID3_REG",
	"0x30",
	"0x34",
	"0x38",
	"0x3C",
	"0x40",
	"0x44",
	"0x48",
	"0x4C",
	"SCM_ZCMD_REG",
	"SCM_CCMD_REG",
	"SCM_C_BLACK_ST_REG",
	"SCM_DBG_STATUS_REG",
	"SCM_AES_CBC_IV0_REG",
	"SCM_AES_CBC_IV1_REG",
	"SCM_AES_CBC_IV2_REG",
	"SCM_AES_CBC_IV3_REG",
	"0x70",
	"0x74",
	"0x78",
	"0x7C",
	"SCM_SMID0_REG",
	"SCM_ACC0_REG",
	"SCM_SMID1_REG",
	"SCM_ACC1_REG",
	"SCM_SMID2_REG",
	"SCM_ACC2_REG",
	"SCM_SMID3_REG",
	"SCM_ACC3_REG",
	"SCM_SMID4_REG",
	"SCM_ACC4_REG",
	"SCM_SMID5_REG",
	"SCM_ACC5_REG",
	"SCM_SMID6_REG",
	"SCM_ACC6_REG",
	"SCM_SMID7_REG",
	"SCM_ACC7_REG",
	"SCM_SMID8_REG",
	"SCM_ACC8_REG",
	"SCM_SMID9_REG",
	"SCM_ACC9_REG",
	"SCM_SMID10_REG",
	"SCM_ACC10_REG",
	"SCM_SMID11_REG",
	"SCM_ACC11_REG",
	"SCM_SMID12_REG",
	"SCM_ACC12_REG",
	"SCM_SMID13_REG",
	"SCM_ACC13_REG",
	"SCM_SMID14_REG",
	"SCM_ACC14_REG",
	"SCM_SMID15_REG",
	"SCM_ACC15_REG",
	"SMN_STATUS_REG",
	"SMN_COMMAND_REG",
	"SMN_SEQ_START_REG",
	"SMN_SEQ_END_REG",
	"SMN_SEQ_CHECK_REG",
	"SMN_BB_CNT_REG",
	"SMN_BB_INC_REG",
	"SMN_BB_DEC_REG",
	"SMN_COMPARE_REG",
	"SMN_PT_CHK_REG",
	"SMN_CT_CHK_REG",
	"SMN_TIMER_IV_REG",
	"SMN_TIMER_CTL_REG",
	"SMN_SEC_VIO_REG",
	"SMN_TIMER_REG",
	"SMN_HAC_REG"
};

/**
 * Names of the Secure RAM States
 */
static char *srs_names[] = {
	"SRS_Reset",
	"SRS_All_Ready",
	"SRS_ZeroizeBusy",
	"SRS_CipherBusy",
	"SRS_AllBusy",
	"SRS_ZeroizeDoneCipherReady",
	"SRS_CipherDoneZeroizeReady",
	"SRS_ZeroizeDoneCipherBusy",
	"SRS_CipherDoneZeroizeBusy",
	"SRS_UNKNOWN_STATE_9",
	"SRS_TransitionalA",
	"SRS_TransitionalB",
	"SRS_TransitionalC",
	"SRS_TransitionalD",
	"SRS_AllDone",
	"SRS_UNKNOWN_STATE_E",
	"SRS_FAIL"
};

/**
 * Create a text interpretation of the SCM Version Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_version_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size,
		 "Bpp: %u, Bpcb: %u, np: %u, maj: %u, min: %u",
		 (value & SCM_VER_BPP_MASK) >> SCM_VER_BPP_SHIFT,
		 ((value & SCM_VER_BPCB_MASK) >> SCM_VER_BPCB_SHIFT) + 1,
		 ((value & SCM_VER_NP_MASK) >> SCM_VER_NP_SHIFT) + 1,
		 (value & SCM_VER_MAJ_MASK) >> SCM_VER_MAJ_SHIFT,
		 (value & SCM_VER_MIN_MASK) >> SCM_VER_MIN_SHIFT);

	return print_buffer;
}

/**
 * Create a text interpretation of the SCM Status Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_status_reg(uint32_t value, char *print_buffer, int buf_size)
{

	snprintf(print_buffer, buf_size, "%s%s%s%s%s%s%s%s%s%s%s%s%s",
		 (value & SCM_STATUS_KST_DEFAULT_KEY) ? "KST_DefaultKey " : "",
		 /* reserved */
		 (value & SCM_STATUS_KST_WRONG_KEY) ? "KST_WrongKey " : "",
		 (value & SCM_STATUS_KST_BAD_KEY) ? "KST_BadKey " : "",
		 (value & SCM_STATUS_ERR) ? "Error " : "",
		 (value & SCM_STATUS_MSS_FAIL) ? "MSS_FailState " : "",
		 (value & SCM_STATUS_MSS_SEC) ? "MSS_SecureState " : "",
		 (value & SCM_STATUS_RSS_FAIL) ? "RSS_FailState " : "",
		 (value & SCM_STATUS_RSS_SEC) ? "RSS_SecureState " : "",
		 (value & SCM_STATUS_RSS_INIT) ? "RSS_Initializing " : "",
		 (value & SCM_STATUS_UNV) ? "UID_Invalid " : "",
		 (value & SCM_STATUS_BIG) ? "BigEndian " : "",
		 (value & SCM_STATUS_USK) ? "SecretKey " : "",
		 srs_names[(value & SCM_STATUS_SRS_MASK) >>
			   SCM_STATUS_SRS_SHIFT]);

	return print_buffer;
}

/**
 * Names of the SCM Error Codes
 */
static
char *scm_err_code[] = {
	"Unknown_0",
	"UnknownAddress",
	"UnknownCommand",
	"ReadPermErr",
	"WritePermErr",
	"DMAErr",
	"EncBlockLenOvfl",
	"KeyNotEngaged",
	"ZeroizeCmdQOvfl",
	"CipherCmdQOvfl",
	"ProcessIntr",
	"WrongKey",
	"DeviceBusy",
	"DMAUnalignedAddr",
	"Unknown_E",
	"Unknown_F",
};

/**
 * Names of the SMN States
 */
static char *smn_state_name[] = {
	"Start",
	"Invalid_01",
	"Invalid_02",
	"Invalid_03",
	"Zeroizing_04",
	"Zeroizing",
	"HealthCheck",
	"HealthCheck_07",
	"Invalid_08",
	"Fail",
	"Secure",
	"Invalid_0B",
	"NonSecure",
	"Invalid_0D",
	"Invalid_0E",
	"Invalid_0F",
	"Invalid_10",
	"Invalid_11",
	"Invalid_12",
	"Invalid_13",
	"Invalid_14",
	"Invalid_15",
	"Invalid_16",
	"Invalid_17",
	"Invalid_18",
	"FailHard",
	"Invalid_1A",
	"Invalid_1B",
	"Invalid_1C",
	"Invalid_1D",
	"Invalid_1E",
	"Invalid_1F"
};

/**
 * Create a text interpretation of the SCM Error Status Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_err_status_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size,
		 "MID: 0x%x, %s%s ErrorCode: %s, SMSState: %s, SCMState: %s",
		 (value & SCM_ERRSTAT_MID_MASK) >> SCM_ERRSTAT_MID_SHIFT,
		 (value & SCM_ERRSTAT_ILM) ? "ILM, " : "",
		 (value & SCM_ERRSTAT_SUP) ? "SUP, " : "",
		 scm_err_code[(value & SCM_ERRSTAT_ERC_MASK) >>
			      SCM_ERRSTAT_ERC_SHIFT],
		 smn_state_name[(value & SCM_ERRSTAT_SMS_MASK) >>
				SCM_ERRSTAT_SMS_SHIFT],
		 srs_names[(value & SCM_ERRSTAT_SRS_MASK) >>
			   SCM_ERRSTAT_SRS_SHIFT]);
	return print_buffer;
}

/**
 * Create a text interpretation of the SCM Zeroize Command Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_zcmd_reg(uint32_t value, char *print_buffer, int buf_size)
{
	unsigned cmd = (value & SCM_ZCMD_CCMD_MASK) >> SCM_CCMD_CCMD_SHIFT;

	snprintf(print_buffer, buf_size, "%s %u",
		 (cmd ==
		  ZCMD_DEALLOC_PART) ? "DeallocPartition" :
		 "(unknown function)",
		 (value & SCM_ZCMD_PART_MASK) >> SCM_ZCMD_PART_SHIFT);

	return print_buffer;
}

/**
 * Create a text interpretation of the SCM Cipher Command Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_ccmd_reg(uint32_t value, char *print_buffer, int buf_size)
{
	unsigned cmd = (value & SCM_CCMD_CCMD_MASK) >> SCM_CCMD_CCMD_SHIFT;

	snprintf(print_buffer, buf_size,
		 "%s %u bytes, %s offset 0x%x, in partition %u",
		 (cmd == SCM_CCMD_AES_DEC_ECB) ? "ECB Decrypt" : (cmd ==
								  SCM_CCMD_AES_ENC_ECB)
		 ? "ECB Encrypt" : (cmd ==
				    SCM_CCMD_AES_DEC_CBC) ? "CBC Decrypt" : (cmd
									     ==
									     SCM_CCMD_AES_ENC_CBC)
		 ? "CBC Encrypt" : "(unknown function)",
		 16 +
		 16 * ((value & SCM_CCMD_LENGTH_MASK) >> SCM_CCMD_LENGTH_SHIFT),
		 ((cmd == SCM_CCMD_AES_ENC_CBC)
		  || (cmd == SCM_CCMD_AES_ENC_ECB)) ? "at" : "to",
		 16 * ((value & SCM_CCMD_OFFSET_MASK) >> SCM_CCMD_OFFSET_SHIFT),
		 (value & SCM_CCMD_PART_MASK) >> SCM_CCMD_PART_SHIFT);

	return print_buffer;
}

/**
 * Create a text interpretation of an SCM Access Permissions Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_acc_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size, "%s%s%s%s%s%s%s%s%s%s",
		 (value & SCM_PERM_NO_ZEROIZE) ? "NO_ZERO " : "",
		 (value & SCM_PERM_HD_SUP_DISABLE) ? "SUP_DIS " : "",
		 (value & SCM_PERM_HD_READ) ? "HD_RD " : "",
		 (value & SCM_PERM_HD_WRITE) ? "HD_WR " : "",
		 (value & SCM_PERM_HD_EXECUTE) ? "HD_EX " : "",
		 (value & SCM_PERM_TH_READ) ? "TH_RD " : "",
		 (value & SCM_PERM_TH_WRITE) ? "TH_WR " : "",
		 (value & SCM_PERM_OT_READ) ? "OT_RD " : "",
		 (value & SCM_PERM_OT_WRITE) ? "OT_WR " : "",
		 (value & SCM_PERM_OT_EXECUTE) ? "OT_EX" : "");

	return print_buffer;
}

/**
 * Create a text interpretation of the SCM Partitions Engaged Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_part_eng_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
		 (value & 0x8000) ? "15 " : "",
		 (value & 0x4000) ? "14 " : "",
		 (value & 0x2000) ? "13 " : "",
		 (value & 0x1000) ? "12 " : "",
		 (value & 0x0800) ? "11 " : "",
		 (value & 0x0400) ? "10 " : "",
		 (value & 0x0200) ? "9 " : "",
		 (value & 0x0100) ? "8 " : "",
		 (value & 0x0080) ? "7 " : "",
		 (value & 0x0040) ? "6 " : "",
		 (value & 0x0020) ? "5 " : "",
		 (value & 0x0010) ? "4 " : "",
		 (value & 0x0008) ? "3 " : "",
		 (value & 0x0004) ? "2 " : "",
		 (value & 0x0002) ? "1 " : "", (value & 0x0001) ? "0" : "");

	return print_buffer;
}

/**
 * Create a text interpretation of the SMN Status Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *smn_print_status_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size,
		 "Version %d %s%s%s%s%s%s%s%s%s%s%s%s%s",
		 (value & SMN_STATUS_VERSION_ID_MASK) >>
		 SMN_STATUS_VERSION_ID_SHIFT,
		 (value & SMN_STATUS_ILLEGAL_MASTER) ? "IllMaster " : "",
		 (value & SMN_STATUS_SCAN_EXIT) ? "ScanExit " : "",
		 (value & SMN_STATUS_PERIP_INIT) ? "PeripInit " : "",
		 (value & SMN_STATUS_SMN_ERROR) ? "SMNError " : "",
		 (value & SMN_STATUS_SOFTWARE_ALARM) ? "SWAlarm " : "",
		 (value & SMN_STATUS_TIMER_ERROR) ? "TimerErr " : "",
		 (value & SMN_STATUS_PC_ERROR) ? "PTCTErr " : "",
		 (value & SMN_STATUS_BITBANK_ERROR) ? "BitbankErr " : "",
		 (value & SMN_STATUS_ASC_ERROR) ? "ASCErr " : "",
		 (value & SMN_STATUS_SECURITY_POLICY_ERROR) ? "SecPlcyErr " :
		 "",
		 (value & SMN_STATUS_SEC_VIO_ACTIVE_ERROR) ? "SecVioAct " : "",
		 (value & SMN_STATUS_INTERNAL_BOOT) ? "IntBoot " : "",
		 smn_state_name[(value & SMN_STATUS_STATE_MASK) >>
				SMN_STATUS_STATE_SHIFT]);

	return print_buffer;
}

/**
 * The array, indexed by register number (byte-offset / 4), of print routines
 * for the SCC (SCM and SMN) registers.
 */
static reg_print_routine_t reg_printers[] = {
	scm_print_version_reg,
	NULL,			/* 0x04 */
	NULL,			/* SCM_INT_CTL_REG */
	scm_print_status_reg,
	scm_print_err_status_reg,
	NULL,			/* SCM_FAULT_ADR_REG */
	NULL,			/* SCM_PART_OWNERS_REG */
	scm_print_part_eng_reg,
	NULL,			/* SCM_UNIQUE_ID0_REG */
	NULL,			/* SCM_UNIQUE_ID1_REG */
	NULL,			/* SCM_UNIQUE_ID2_REG */
	NULL,			/* SCM_UNIQUE_ID3_REG */
	NULL,			/* 0x30 */
	NULL,			/* 0x34 */
	NULL,			/* 0x38 */
	NULL,			/* 0x3C */
	NULL,			/* 0x40 */
	NULL,			/* 0x44 */
	NULL,			/* 0x48 */
	NULL,			/* 0x4C */
	scm_print_zcmd_reg,
	scm_print_ccmd_reg,
	NULL,			/* SCM_C_BLACK_ST_REG */
	NULL,			/* SCM_DBG_STATUS_REG */
	NULL,			/* SCM_AES_CBC_IV0_REG */
	NULL,			/* SCM_AES_CBC_IV1_REG */
	NULL,			/* SCM_AES_CBC_IV2_REG */
	NULL,			/* SCM_AES_CBC_IV3_REG */
	NULL,			/* 0x70 */
	NULL,			/* 0x74 */
	NULL,			/* 0x78 */
	NULL,			/* 0x7C */
	NULL,			/* SCM_SMID0_REG */
	scm_print_acc_reg,	/* ACC0 */
	NULL,			/* SCM_SMID1_REG */
	scm_print_acc_reg,	/* ACC1 */
	NULL,			/* SCM_SMID2_REG */
	scm_print_acc_reg,	/* ACC2 */
	NULL,			/* SCM_SMID3_REG */
	scm_print_acc_reg,	/* ACC3 */
	NULL,			/* SCM_SMID4_REG */
	scm_print_acc_reg,	/* ACC4 */
	NULL,			/* SCM_SMID5_REG */
	scm_print_acc_reg,	/* ACC5 */
	NULL,			/* SCM_SMID6_REG */
	scm_print_acc_reg,	/* ACC6 */
	NULL,			/* SCM_SMID7_REG */
	scm_print_acc_reg,	/* ACC7 */
	NULL,			/* SCM_SMID8_REG */
	scm_print_acc_reg,	/* ACC8 */
	NULL,			/* SCM_SMID9_REG */
	scm_print_acc_reg,	/* ACC9 */
	NULL,			/* SCM_SMID10_REG */
	scm_print_acc_reg,	/* ACC10 */
	NULL,			/* SCM_SMID11_REG */
	scm_print_acc_reg,	/* ACC11 */
	NULL,			/* SCM_SMID12_REG */
	scm_print_acc_reg,	/* ACC12 */
	NULL,			/* SCM_SMID13_REG */
	scm_print_acc_reg,	/* ACC13 */
	NULL,			/* SCM_SMID14_REG */
	scm_print_acc_reg,	/* ACC14 */
	NULL,			/* SCM_SMID15_REG */
	scm_print_acc_reg,	/* ACC15 */
	smn_print_status_reg,
	NULL,			/* SMN_COMMAND_REG */
	NULL,			/* SMN_SEQ_START_REG */
	NULL,			/* SMN_SEQ_END_REG */
	NULL,			/* SMN_SEQ_CHECK_REG */
	NULL,			/* SMN_BB_CNT_REG */
	NULL,			/* SMN_BB_INC_REG */
	NULL,			/* SMN_BB_DEC_REG */
	NULL,			/* SMN_COMPARE_REG */
	NULL,			/* SMN_PT_CHK_REG */
	NULL,			/* SMN_CT_CHK_REG */
	NULL,			/* SMN_TIMER_IV_REG */
	NULL,			/* SMN_TIMER_CTL_REG */
	NULL,			/* SMN_SEC_VIO_REG */
	NULL,			/* SMN_TIMER_REG */
	NULL,			/* SMN_HAC_REG */
};

/*****************************************************************************/
/* fn dbg_scc_read_register()                                                */
/*****************************************************************************/
/**
 * Noisily read a 32-bit value to an SCC register.
 * @param offset        The address of the register to read.
 *
 * @return  The register value
 * */
uint32_t dbg_scc_read_register(uint32_t offset)
{
	uint32_t value;
	char *regname = scc_regnames[offset / 4];

	value = __raw_readl(scc_base + offset);
	pr_debug("SCC2 RD: 0x%03x : 0x%08x (%s) %s\n", offset, value, regname,
		 reg_printers[offset / 4]
		 ? reg_printers[offset / 4] (value, reg_print_buffer,
					     REG_PRINT_BUFFER_SIZE)
		 : "");

	return value;
}

/*****************************************************************************/
/* fn dbg_scc_write_register()                                               */
/*****************************************************************************/
/*
 * Noisily read a 32-bit value to an SCC register.
 * @param offset        The address of the register to written.
 *
 * @param value         The new register value
 */
void dbg_scc_write_register(uint32_t offset, uint32_t value)
{
	char *regname = scc_regnames[offset / 4];

	pr_debug("SCC2 WR: 0x%03x : 0x%08x (%s) %s\n", offset, value, regname,
		 reg_printers[offset / 4]
		 ? reg_printers[offset / 4] (value, reg_print_buffer,
					     REG_PRINT_BUFFER_SIZE)
		 : "");
	(void)__raw_writel(value, scc_base + offset);

}

#endif				/* SCC_REGISTER_DEBUG */

static int scc_dev_probe(struct platform_device *pdev)
{
	struct resource *r;
	int ret = 0;

	/* get the scc registers base address */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "can't get IORESOURCE_MEM (0)\n");
		ret = -ENXIO;
		goto exit;
	}

	scc_phys_base = r->start;


	/* get the scc ram base address */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!r) {
		dev_err(&pdev->dev, "can't get IORESOURCE_MEM (1)\n");
		ret = -ENXIO;
		goto exit;
	}

	scm_ram_phys_base = r->start;

	/* get the scc smn irq */
	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r) {
		dev_err(&pdev->dev, "can't get IORESOURCE_IRQ (0)\n");
		ret = -ENXIO;
		goto exit;
	}

	irq_smn = r->start;

	/* get the scc scm irq */
	r = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!r) {
		dev_err(&pdev->dev, "can't get IORESOURCE_IRQ (1)\n");
		ret = -ENXIO;
		goto exit;
	}

	irq_scm = r->start;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
	scc_clk = clk_get(&pdev->dev, "scc_clk");
#endif

	/* now initialize the SCC */
	ret = scc_init();

exit:
	return ret;
}

static int scc_dev_remove(struct platform_device *pdev)
{
	scc_cleanup();
	return 0;
}


#ifdef CONFIG_PM
static int scc_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int scc_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define scc_suspend	NULL
#define	scc_resume	NULL
#endif

/*! Linux Driver definition
 *
 */
static struct platform_driver mxcscc_driver = {
	.driver = {
		   .name = SCC_DRIVER_NAME,
		   },
	.probe = scc_dev_probe,
	.remove = scc_dev_remove,
	.suspend = scc_suspend,
	.resume = scc_resume,
};

static int __init scc_driver_init(void)
{
	return platform_driver_register(&mxcscc_driver);
}

module_init(scc_driver_init);

static void __exit scc_driver_exit(void)
{
	platform_driver_unregister(&mxcscc_driver);
}

module_exit(scc_driver_exit);
