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

/*! @file shw_driver.c
 *
 * This is the user-mode driver code for the FSL Security Hardware (SHW) API.
 * as well as the 'common' FSL SHW API code for kernel API users.
 *
 * Its interaction with the Linux kernel is from calls to shw_init() when the
 * driver is loaded, and shw_shutdown() should the driver be unloaded.
 *
 * The User API (driver interface) is handled by the following functions:
 * @li shw_open()    - handles open() system call on FSL SHW device
 * @li shw_release() - handles close() system call on FSL SHW device
 * @li shw_ioctl()    - handles ioctl() system call on FSL SHW device
 *
 * The driver also provides the following functions for kernel users of the FSL
 * SHW API:
 * @li fsl_shw_register_user()
 * @li fsl_shw_deregister_user()
 * @li fsl_shw_get_capabilities()
 * @li fsl_shw_get_results()
 *
 * All other functions are internal to the driver.
 *
 * The life of the driver starts at boot (or module load) time, with a call by
 * the kernel to shw_init().
 *
 * The life of the driver ends when the kernel is shutting down (or the driver
 * is being unloaded).  At this time, shw_shutdown() is called.  No function
 * will ever be called after that point.
 *
 * In the case that the driver is reloaded, a new copy of the driver, with
 * fresh global values, etc., is loaded, and there will be a new call to
 * shw_init().
 *
 * In user mode, the user's fsl_shw_register_user() call causes an open() event
 * on the driver followed by a ioctl() with the registration information.  Any
 * subsequent API calls by the user are handled through the ioctl() function
 * and shuffled off to the appropriate routine (or driver) for service.  The
 * fsl_shw_deregister_user() call by the user results in a close() function
 * call on the driver.
 *
 * In kernel mode, the driver provides the functions fsl_shw_register_user(),
 * fsl_shw_deregister_user(), fsl_shw_get_capabilities(), and
 * fsl_shw_get_results().  Other parts of the API are provided by other
 * drivers, if available, to support the cryptographic functions.
 */

#include "portable_os.h"
#include "fsl_shw.h"
#include "fsl_shw_keystore.h"

#include "shw_internals.h"

#ifdef FSL_HAVE_SCC2
#include <linux/mxc_scc2_driver.h>
#else
#include <linux/mxc_scc_driver.h>
#endif

#ifdef SHW_DEBUG
#include <diagnostic.h>
#endif

/******************************************************************************
 *
 *  Function Declarations
 *
 *****************************************************************************/

/* kernel interface functions */
OS_DEV_INIT_DCL(shw_init);
OS_DEV_SHUTDOWN_DCL(shw_shutdown);
OS_DEV_IOCTL_DCL(shw_ioctl);
OS_DEV_MMAP_DCL(shw_mmap);

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_smalloc);
EXPORT_SYMBOL(fsl_shw_sfree);
EXPORT_SYMBOL(fsl_shw_sstatus);
EXPORT_SYMBOL(fsl_shw_diminish_perms);
EXPORT_SYMBOL(do_scc_encrypt_region);
EXPORT_SYMBOL(do_scc_decrypt_region);

EXPORT_SYMBOL(do_system_keystore_slot_alloc);
EXPORT_SYMBOL(do_system_keystore_slot_dealloc);
EXPORT_SYMBOL(do_system_keystore_slot_load);
EXPORT_SYMBOL(do_system_keystore_slot_encrypt);
EXPORT_SYMBOL(do_system_keystore_slot_decrypt);
#endif

static os_error_code
shw_handle_scc_sfree(fsl_shw_uco_t * user_ctx, uint32_t info);

static os_error_code
shw_handle_scc_sstatus(fsl_shw_uco_t * user_ctx, uint32_t info);

static os_error_code
shw_handle_scc_drop_perms(fsl_shw_uco_t * user_ctx, uint32_t info);

static os_error_code
shw_handle_scc_encrypt(fsl_shw_uco_t * user_ctx, uint32_t info);

static os_error_code
shw_handle_scc_decrypt(fsl_shw_uco_t * user_ctx, uint32_t info);

#ifdef FSL_HAVE_SCC2
static fsl_shw_return_t register_user_partition(fsl_shw_uco_t * user_ctx,
						uint32_t user_base,
						void *kernel_base);
static fsl_shw_return_t deregister_user_partition(fsl_shw_uco_t * user_ctx,
						  uint32_t user_base);
void *lookup_user_partition(fsl_shw_uco_t * user_ctx, uint32_t user_base);

#endif				/* FSL_HAVE_SCC2 */

/******************************************************************************
 *
 *  Global / Static Variables
 *
 *****************************************************************************/

/*!
 *  Major node (user/device interaction value) of this driver.
 */
static int shw_major_node = SHW_MAJOR_NODE;

/*!
 *  Flag to know whether the driver has been associated with its user device
 *  node (e.g. /dev/shw).
 */
static int shw_device_registered = 0;

/*!
 * OS-dependent handle used for registering user interface of a driver.
 */
static os_driver_reg_t reg_handle;

/*!
 * Linked List of registered users of the API
 */
fsl_shw_uco_t *user_list;

/*!
 * This is the lock for all user request pools.  H/W component drivers may also
 * use it for their own work queues.
 */
os_lock_t shw_queue_lock = NULL;

/* This is the system keystore object */
fsl_shw_kso_t system_keystore;

#ifndef FSL_HAVE_SAHARA
/*! Empty list of supported symmetric algorithms. */
static fsl_shw_key_alg_t pf_syms[] = {
};

/*! Empty list of supported symmetric modes. */
static fsl_shw_sym_mode_t pf_modes[] = {
};

/*! Empty list of supported hash algorithms. */
static fsl_shw_hash_alg_t pf_hashes[] = {
};
#endif				/* no Sahara */

/*! This matches SHW capabilities... */
static fsl_shw_pco_t cap = {
	1, 3,			/* api version number - major & minor */
	2, 3,			/* driver version number - major & minor */
	sizeof(pf_syms) / sizeof(fsl_shw_key_alg_t),	/* key alg count */
	pf_syms,		/* key alg list ptr */
	sizeof(pf_modes) / sizeof(fsl_shw_sym_mode_t),	/* sym mode count */
	pf_modes,		/* modes list ptr */
	sizeof(pf_hashes) / sizeof(fsl_shw_hash_alg_t),	/* hash alg count */
	pf_hashes,		/* hash list ptr */
	/*
	 * The following table must be set to handle all values of key algorithm
	 * and sym mode, and be in the correct order..
	 */
	{			/* Stream, ECB, CBC, CTR */
	 {0, 0, 0, 0}
	 ,			/* HMAC */
	 {0, 0, 0, 0}
	 ,			/* AES  */
	 {0, 0, 0, 0}
	 ,			/* DES */
#ifdef FSL_HAVE_DRYICE
	 {0, 1, 1, 0}
	 ,			/* 3DES - ECB and CBC */
#else
	 {0, 0, 0, 0}
	 ,			/* 3DES */
#endif
	 {0, 0, 0, 0}		/* ARC4 */
	 }
	,
	0, 0,			/* SCC driver version */
	0, 0, 0,		/* SCC version/capabilities */
	{{0, 0}
	 }
	,			/* (filled in during OS_INIT) */
};

/* These are often handy */
#ifndef FALSE
/*! Not true.  Guaranteed to be zero. */
#define FALSE 0
#endif
#ifndef TRUE
/*! True.  Guaranteed to be non-zero. */
#define TRUE 1
#endif

/******************************************************************************
 *
 *  Function Implementations - Externally Accessible
 *
 *****************************************************************************/

/*****************************************************************************/
/* fn shw_init()                                                             */
/*****************************************************************************/
/*!
 * Initialize the driver.
 *
 * This routine is called during kernel init or module load (insmod).
 *
 * @return OS_ERROR_OK_S on success, errno on failure
 */
OS_DEV_INIT(shw_init)
{
	os_error_code error_code = OS_ERROR_NO_MEMORY_S;	/* assume failure */
	scc_config_t *shw_capabilities;

#ifdef SHW_DEBUG
	LOG_KDIAG("SHW Driver: Loading");
#endif

	user_list = NULL;
	shw_queue_lock = os_lock_alloc_init();

	if (shw_queue_lock != NULL) {
		error_code = shw_setup_user_driver_interaction();
		if (error_code != OS_ERROR_OK_S) {
#ifdef SHW_DEBUG
			LOG_KDIAG_ARGS
			    ("SHW Driver: Failed to setup user i/f: %d",
			     error_code);
#endif
		}
	}

	/* queue_lock not NULL */
	/* Fill in the SCC portion of the capabilities object */
	shw_capabilities = scc_get_configuration();
	cap.scc_driver_major = shw_capabilities->driver_major_version;
	cap.scc_driver_minor = shw_capabilities->driver_minor_version;
	cap.scm_version = shw_capabilities->scm_version;
	cap.smn_version = shw_capabilities->smn_version;
	cap.block_size_bytes = shw_capabilities->block_size_bytes;

#ifdef FSL_HAVE_SCC
	cap.u.scc_info.black_ram_size_blocks =
	    shw_capabilities->black_ram_size_blocks;
	cap.u.scc_info.red_ram_size_blocks =
	    shw_capabilities->red_ram_size_blocks;
#elif defined(FSL_HAVE_SCC2)
	cap.u.scc2_info.partition_size_bytes =
	    shw_capabilities->partition_size_bytes;
	cap.u.scc2_info.partition_count = shw_capabilities->partition_count;
#endif

#if defined(FSL_HAVE_SCC2) || defined(FSL_HAVE_DRYICE)
	if (error_code == OS_ERROR_OK_S) {
		/* set up the system keystore, using the default keystore handler */
		fsl_shw_init_keystore_default(&system_keystore);

		if (fsl_shw_establish_keystore(NULL, &system_keystore)
		    == FSL_RETURN_OK_S) {
			error_code = OS_ERROR_OK_S;
		} else {
			error_code = OS_ERROR_FAIL_S;
		}

		if (error_code != OS_ERROR_OK_S) {
#ifdef SHW_DEBUG
			LOG_KDIAG_ARGS
			    ("Registering the system keystore failed with error"
			     " code: %d\n", error_code);
#endif
		}
	}
#endif				/* FSL_HAVE_SCC2 */

	if (error_code != OS_ERROR_OK_S) {
#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS("SHW: Driver initialization failed. %d",
			       error_code);
#endif
		shw_cleanup();
	} else {
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: Driver initialization complete.");
#endif
	}

	os_dev_init_return(error_code);
}				/* shw_init */

/*****************************************************************************/
/* fn shw_shutdown()                                                         */
/*****************************************************************************/
/*!
 * Prepare driver for exit.
 *
 * This is called during @c rmmod when the driver is unloading or when the
 * kernel is shutting down.
 *
 * Calls shw_cleanup() to do all work to undo anything that happened during
 * initialization or while driver was running.
 */
OS_DEV_SHUTDOWN(shw_shutdown)
{

#ifdef SHW_DEBUG
	LOG_KDIAG("SHW: shutdown called");
#endif
	shw_cleanup();

	os_dev_shutdown_return(OS_ERROR_OK_S);
}				/* shw_shutdown */

/*****************************************************************************/
/* fn shw_cleanup()                                                          */
/*****************************************************************************/
/*!
 * Prepare driver for shutdown.
 *
 * Remove the driver registration.
 *
 */
static void shw_cleanup(void)
{
	if (shw_device_registered) {

		/* Turn off the all association with OS */
		os_driver_remove_registration(reg_handle);
		shw_device_registered = 0;
	}

	if (shw_queue_lock != NULL) {
		os_lock_deallocate(shw_queue_lock);
	}
#ifdef SHW_DEBUG
	LOG_KDIAG("SHW Driver: Cleaned up");
#endif
}				/* shw_cleanup */

/*****************************************************************************/
/* fn shw_open()                                                             */
/*****************************************************************************/
/*!
 * Handle @c open() call from user.
 *
 * @return OS_ERROR_OK_S on success (always!)
 */
OS_DEV_OPEN(shw_open)
{
	os_error_code status = OS_ERROR_OK_S;

	os_dev_set_user_private(NULL);	/* Make sure */

	os_dev_open_return(status);
}				/* shw_open */

/*****************************************************************************/
/* fn shw_ioctl()                                                            */
/*****************************************************************************/
/*!
 * Process an ioctl() request from user-mode API.
 *
 * This code determines which of the API requests the user has made and then
 * sends the request off to the appropriate function.
 *
 * @return ioctl_return()
 */
OS_DEV_IOCTL(shw_ioctl)
{
	os_error_code code = OS_ERROR_FAIL_S;

	fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

#ifdef SHW_DEBUG
	LOG_KDIAG_ARGS("SHW: IOCTL %d received", os_dev_get_ioctl_op());
#endif
	switch (os_dev_get_ioctl_op()) {

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_REGISTER_USER:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: register_user ioctl received");
#endif
		{
			fsl_shw_uco_t *user_ctx =
			    os_alloc_memory(sizeof(*user_ctx), 0);

			if (user_ctx == NULL) {
				code = OS_ERROR_NO_MEMORY_S;
			} else {
				code =
				    init_uco(user_ctx,
					     (fsl_shw_uco_t *)
					     os_dev_get_ioctl_arg());
				if (code == OS_ERROR_OK_S) {
					os_dev_set_user_private(user_ctx);
				} else {
					os_free_memory(user_ctx);
				}
			}
		}
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_DEREGISTER_USER:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: deregister_user ioctl received");
#endif
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();
			SHW_REMOVE_USER(user_ctx);
		}
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_GET_RESULTS:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: get_results ioctl received");
#endif
		code = get_results(user_ctx,
				   (struct results_req *)
				   os_dev_get_ioctl_arg());
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_GET_CAPABILITIES:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: get_capabilities ioctl received");
#endif
		code = get_capabilities(user_ctx,
					(fsl_shw_pco_t *)
					os_dev_get_ioctl_arg());
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_GET_RANDOM:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: get_random ioctl received");
#endif
		code = get_random(user_ctx,
				  (struct get_random_req *)
				  os_dev_get_ioctl_arg());
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_ADD_ENTROPY:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: add_entropy ioctl received");
#endif
		code = add_entropy(user_ctx,
				   (struct add_entropy_req *)
				   os_dev_get_ioctl_arg());
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_DROP_PERMS:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: drop permissions ioctl received");
#endif
		code =
		    shw_handle_scc_drop_perms(user_ctx, os_dev_get_ioctl_arg());
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_SSTATUS:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: sstatus ioctl received");
#endif
		code = shw_handle_scc_sstatus(user_ctx, os_dev_get_ioctl_arg());
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_SFREE:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: sfree ioctl received");
#endif
		code = shw_handle_scc_sfree(user_ctx, os_dev_get_ioctl_arg());
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_SCC_ENCRYPT:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: scc encrypt ioctl received");
#endif
		code = shw_handle_scc_encrypt(user_ctx, os_dev_get_ioctl_arg());
		break;

	case SHW_IOCTL_REQUEST + SHW_USER_REQ_SCC_DECRYPT:
#ifdef SHW_DEBUG
		LOG_KDIAG("SHW: scc decrypt ioctl received");
#endif
		code = shw_handle_scc_decrypt(user_ctx, os_dev_get_ioctl_arg());
		break;

	default:
#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS("SHW: Unexpected ioctl %d",
			       os_dev_get_ioctl_op());
#endif
		break;
	}

	os_dev_ioctl_return(code);
}

#ifdef FSL_HAVE_SCC2

/*****************************************************************************/
/* fn get_user_smid()                                                        */
/*****************************************************************************/
uint32_t get_user_smid(void *proc)
{
	/*
	 * A real implementation would have some way to handle signed applications
	 * which wouild be assigned distinct SMIDs.  For the reference
	 * implementation, we show where this would be determined (here), but
	 * always provide a fixed answer, thus not separating users at all.
	 */

	return 0x42eaae42;
}

/* user_base: userspace base address of the partition
 * kernel_base: kernel mode base address of the partition
 */
static fsl_shw_return_t register_user_partition(fsl_shw_uco_t * user_ctx,
						uint32_t user_base,
						void *kernel_base)
{
	fsl_shw_spo_t *partition_info;
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

	if (user_ctx == NULL) {
		goto out;
	}

	partition_info = os_alloc_memory(sizeof(fsl_shw_spo_t), GFP_KERNEL);

	if (partition_info == NULL) {
		goto out;
	}

	/* stuff the partition info, then put it at the front of the chain */
	partition_info->user_base = user_base;
	partition_info->kernel_base = kernel_base;
	partition_info->next = user_ctx->partition;

	user_ctx->partition = (struct fsl_shw_spo_t *)partition_info;

#ifdef SHW_DEBUG
	LOG_KDIAG_ARGS
	    ("partition with user_base=%p, kernel_base=%p registered.",
	     (void *)user_base, kernel_base);
#endif

	ret = FSL_RETURN_OK_S;

      out:

	return ret;
}

/* if the partition is in the users list, remove it */
static fsl_shw_return_t deregister_user_partition(fsl_shw_uco_t * user_ctx,
						  uint32_t user_base)
{
	fsl_shw_spo_t *curr = (fsl_shw_spo_t *) user_ctx->partition;
	fsl_shw_spo_t *last = (fsl_shw_spo_t *) user_ctx->partition;

	while (curr != NULL) {
		if (curr->user_base == user_base) {

#ifdef SHW_DEBUG
			LOG_KDIAG_ARGS
			    ("deregister_user_partition: partition with "
			     "user_base=%p, kernel_base=%p deregistered.\n",
			     (void *)curr->user_base, curr->kernel_base);
#endif

			if (last == curr) {
				user_ctx->partition = curr->next;
				os_free_memory(curr);
				return FSL_RETURN_OK_S;
			} else {
				last->next = curr->next;
				os_free_memory(curr);
				return FSL_RETURN_OK_S;
			}
		}
		last = curr;
		curr = (fsl_shw_spo_t *) curr->next;
	}

	return FSL_RETURN_ERROR_S;
}

/* Find the kernel-mode address of the partition.
 * This can then be passed to the SCC functions.
 */
void *lookup_user_partition(fsl_shw_uco_t * user_ctx, uint32_t user_base)
{
	/* search through the partition chain to find one that matches the user base
	 * address.
	 */
	fsl_shw_spo_t *curr = (fsl_shw_spo_t *) user_ctx->partition;

	while (curr != NULL) {
		if (curr->user_base == user_base) {
			return curr->kernel_base;
		}
		curr = (fsl_shw_spo_t *) curr->next;
	}
	return NULL;
}

#endif				/* FSL_HAVE_SCC2 */

/*!
*******************************************************************************
* This function implements the smalloc() function for userspace programs, by
* making a call to the SCC2 mmap() function that acquires a region of secure
* memory on behalf of the user, and then maps it into the users memory space.
* Currently, the only memory size supported is that of a single SCC2 partition.
* Requests for other sized memory regions will fail.
*/
OS_DEV_MMAP(shw_mmap)
{
	os_error_code status = OS_ERROR_NO_MEMORY_S;

#ifdef FSL_HAVE_SCC2
	{
		scc_return_t scc_ret;
		fsl_shw_return_t fsl_ret;
		uint32_t partition_registered = FALSE;

		uint32_t user_base;
		void *partition_base;
		uint32_t smid;
		scc_config_t *scc_configuration;

		int part_no = -1;
		uint32_t part_phys;

		fsl_shw_uco_t *user_ctx =
		    (fsl_shw_uco_t *) os_dev_get_user_private();

		/* Make sure that the user context is valid */
		if (user_ctx == NULL) {
			user_ctx =
			    os_alloc_memory(sizeof(*user_ctx), GFP_KERNEL);

			if (user_ctx == NULL) {
				status = OS_ERROR_NO_MEMORY_S;
				goto out;
			}
			fsl_shw_register_user(user_ctx);
			os_dev_set_user_private(user_ctx);
		}

		/* Determine the size of a secure partition */
		scc_configuration = scc_get_configuration();

		/* Check that the memory size requested is equal to the partition
		 * size, and that the requested destination is on a page boundary.
		 */
		if (((os_mmap_user_base() % PAGE_SIZE) != 0) ||
		    (os_mmap_memory_size() !=
		     scc_configuration->partition_size_bytes)) {
			status = OS_ERROR_BAD_ARG_S;
			goto out;
		}

		/* Retrieve the SMID associated with the user */
		smid = get_user_smid(user_ctx->process);

		/* Attempt to allocate a secure partition */
		scc_ret =
		    scc_allocate_partition(smid, &part_no, &partition_base,
					   &part_phys);
		if (scc_ret != SCC_RET_OK) {
			pr_debug
			    ("SCC mmap() request failed to allocate partition;"
			     " error %d\n", status);
			status = OS_ERROR_FAIL_S;
			goto out;
		}

		pr_debug("scc_mmap() acquired partition %d at %08x\n",
			 part_no, part_phys);

		/* Record partition info in the user context */
		user_base = os_mmap_user_base();
		fsl_ret =
		    register_user_partition(user_ctx, user_base,
					    partition_base);

		if (fsl_ret != FSL_RETURN_OK_S) {
			pr_debug
			    ("SCC mmap() request failed to register partition with user"
			     " context, error: %d\n", fsl_ret);
			status = OS_ERROR_FAIL_S;
		}

		partition_registered = TRUE;

		status = map_user_memory(os_mmap_memory_ctx(), part_phys,
					 os_mmap_memory_size());

#ifdef SHW_DEBUG
		if (status == OS_ERROR_OK_S) {
			LOG_KDIAG_ARGS
			    ("Partition allocated: user_base=%p, partition_base=%p.",
			     (void *)user_base, partition_base);
		}
#endif

	      out:
		/* If there is an error it has to be handled here */
		if (status != OS_ERROR_OK_S) {
			/* if the partition was registered with the user, unregister it. */
			if (partition_registered == TRUE) {
				deregister_user_partition(user_ctx, user_base);
			}

			/* if the partition was allocated, deallocate it */
			if (partition_base != NULL) {
				scc_release_partition(partition_base);
			}
		}
	}
#endif				/* FSL_HAVE_SCC2 */

	return status;
}

/*****************************************************************************/
/* fn shw_release()                                                         */
/*****************************************************************************/
/*!
 * Handle @c close() call from user.
 * This is a Linux device driver interface routine.
 *
 * @return OS_ERROR_OK_S on success (always!)
 */
OS_DEV_CLOSE(shw_release)
{
	fsl_shw_uco_t *user_ctx = os_dev_get_user_private();
	os_error_code code = OS_ERROR_OK_S;

	if (user_ctx != NULL) {

		fsl_shw_deregister_user(user_ctx);
		os_free_memory(user_ctx);
		os_dev_set_user_private(NULL);

	}

	os_dev_close_return(code);
}				/* shw_release */

/*****************************************************************************/
/* fn shw_user_callback()                                                    */
/*****************************************************************************/
/*!
 * FSL SHW User callback function.
 *
 * This function is set in the kernel version of the user context as the
 * callback function when the user mode user wants a callback.  Its job is to
 * inform the user process that results (may) be available.  It does this by
 * sending a SIGUSR2 signal which is then caught by the user-mode FSL SHW
 * library.
 *
 * @param user_ctx        Kernel version of uco associated with the request.
 *
 * @return void
 */
static void shw_user_callback(fsl_shw_uco_t * user_ctx)
{
#ifdef SHW_DEBUG
	LOG_KDIAG_ARGS("SHW: Signalling callback user process for context %p\n",
		       user_ctx);
#endif
	os_send_signal(user_ctx->process, SIGUSR2);
}

/*****************************************************************************/
/* fn setup_user_driver_interaction()                                        */
/*****************************************************************************/
/*!
 * Register the driver with the kernel as the driver for shw_major_node.  Note
 * that this value may be zero, in which case the major number will be assigned
 * by the OS.  shw_major_node is never modified.
 *
 * The open(), ioctl(), and close() handles for the driver ned to be registered
 * with the kernel.  Upon success, shw_device_registered will be true;
 *
 * @return OS_ERROR_OK_S on success, or an os err code
 */
static os_error_code shw_setup_user_driver_interaction(void)
{
	os_error_code error_code;

	os_driver_init_registration(reg_handle);
	os_driver_add_registration(reg_handle, OS_FN_OPEN,
				   OS_DEV_OPEN_REF(shw_open));
	os_driver_add_registration(reg_handle, OS_FN_IOCTL,
				   OS_DEV_IOCTL_REF(shw_ioctl));
	os_driver_add_registration(reg_handle, OS_FN_CLOSE,
				   OS_DEV_CLOSE_REF(shw_release));
	os_driver_add_registration(reg_handle, OS_FN_MMAP,
				   OS_DEV_MMAP_REF(shw_mmap));
	error_code = os_driver_complete_registration(reg_handle, shw_major_node,
						     SHW_DRIVER_NAME);

	if (error_code != OS_ERROR_OK_S) {
		/* failure ! */
#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS("SHW Driver: register device driver failed: %d",
			       error_code);
#endif
	} else {		/* success */
		shw_device_registered = TRUE;
#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS("SHW Driver:  Major node is %d\n",
			       os_driver_get_major(reg_handle));
#endif
	}

	return error_code;
}				/* shw_setup_user_driver_interaction */

/******************************************************************/
/* User Mode Support                                              */
/******************************************************************/

/*!
 * Initialze kernel User Context Object from User-space version.
 *
 * Copy user UCO into kernel UCO, set flags and fields for operation
 * within kernel space.  Add user to driver's list of users.
 *
 * @param user_ctx        Pointer to kernel space UCO
 * @param user_mode_uco   User pointer to user space version
 *
 * @return os_error_code
 */
static os_error_code init_uco(fsl_shw_uco_t * user_ctx, void *user_mode_uco)
{
	os_error_code code;

	code = os_copy_from_user(user_ctx, user_mode_uco, sizeof(*user_ctx));
	if (code == OS_ERROR_OK_S) {
		user_ctx->flags |= FSL_UCO_USERMODE_USER;
		user_ctx->result_pool.head = NULL;
		user_ctx->result_pool.tail = NULL;
		user_ctx->process = os_get_process_handle();
		user_ctx->callback = shw_user_callback;
		SHW_ADD_USER(user_ctx);
	}
#ifdef SHW_DEBUG
	LOG_KDIAG_ARGS("SHW: init uco returning %d (flags %x)",
		       code, user_ctx->flags);
#endif

	return code;
}

/*!
 * Copy array from kernel to user space.
 *
 * This routine will check bounds before trying to copy, and return failure
 * on bounds violation or error during the copy.
 *
 * @param userloc   Location in userloc to place data.  If NULL, the function
 *                  will do nothing (except return NULL).
 * @param userend   Address beyond allowed copy region at @c userloc.
 * @param data_start Location of data to be copied
 * @param element_size  sizeof() an element
 * @param element_count Number of elements of size element_size to copy.
 * @return New value of userloc, or NULL if there was an error.
 */
inline static void *copy_array(void *userloc, void *userend, void *data_start,
			       unsigned element_size, unsigned element_count)
{
	unsigned byte_count = element_size * element_count;

	if ((userloc == NULL) || (userend == NULL)
	    || ((userloc + byte_count) >= userend) ||
	    (copy_to_user(userloc, data_start, byte_count) != OS_ERROR_OK_S)) {
		userloc = NULL;
	} else {
		userloc += byte_count;
	}

	return userloc;
}

/*!
 * Send an FSL SHW API return code up into the user-space request structure.
 *
 * @param user_header   User address of request block / request header
 * @param result_code   The FSL SHW API code to be placed at header.code
 *
 * @return an os_error_code
 *
 * NOTE: This function assumes that the shw_req_header is at the beginning of
 * each request structure.
 */
inline static os_error_code copy_fsl_code(void *user_header,
					  fsl_shw_return_t result_code)
{
	return os_copy_to_user(user_header +
			       offsetof(struct shw_req_header, code),
			       &result_code, sizeof(result_code));
}

static os_error_code shw_handle_scc_drop_perms(fsl_shw_uco_t * user_ctx,
					       uint32_t info)
{
	os_error_code status = OS_ERROR_NO_MEMORY_S;
#ifdef FSL_HAVE_SCC2
	scc_return_t scc_ret;
	scc_partition_info_t partition_info;
	void *kernel_base;

	status =
	    os_copy_from_user(&partition_info, (void *)info,
			      sizeof(partition_info));

	if (status != OS_ERROR_OK_S) {
		goto out;
	}

	/* validate that the user owns this partition, and look up its handle */
	kernel_base = lookup_user_partition(user_ctx, partition_info.user_base);

	if (kernel_base == NULL) {
		status = OS_ERROR_FAIL_S;
#ifdef SHW_DEBUG
		LOG_KDIAG("_scc_drop_perms(): failed to find partition\n");
#endif
		goto out;
	}

	/* call scc driver to perform the drop */
	scc_ret = scc_diminish_permissions(kernel_base,
					   partition_info.permissions);
	if (scc_ret == SCC_RET_OK) {
		status = OS_ERROR_OK_S;
	} else {
		status = OS_ERROR_FAIL_S;
	}

      out:
#endif				/* FSL_HAVE_SCC2 */
	return status;
}

static os_error_code shw_handle_scc_sstatus(fsl_shw_uco_t * user_ctx,
					    uint32_t info)
{
	os_error_code status = OS_ERROR_NO_MEMORY_S;
#ifdef FSL_HAVE_SCC2
	scc_partition_info_t partition_info;
	void *kernel_base;

	status = os_copy_from_user(&partition_info,
				   (void *)info, sizeof(partition_info));

	if (status != OS_ERROR_OK_S) {
		goto out;
	}

	/* validate that the user owns this partition, and look up its handle */
	kernel_base = lookup_user_partition(user_ctx, partition_info.user_base);

	if (kernel_base == NULL) {
		status = OS_ERROR_FAIL_S;
#ifdef SHW_DEBUG
		LOG_KDIAG("Failed to find partition\n");
#endif
		goto out;
	}

	/* Call the SCC driver to ask about the partition status */
	partition_info.status = scc_partition_status(kernel_base);

	/* and copy the structure out */
	status = os_copy_to_user((void *)info,
				 &partition_info, sizeof(partition_info));

      out:
#endif				/* FSL_HAVE_SCC2 */
	return status;
}

static os_error_code shw_handle_scc_sfree(fsl_shw_uco_t * user_ctx,
					  uint32_t info)
{
	os_error_code status = OS_ERROR_NO_MEMORY_S;
#ifdef FSL_HAVE_SCC2
	{
		scc_partition_info_t partition_info;
		void *kernel_base;
		int ret;

		status = os_copy_from_user(&partition_info,
					   (void *)info,
					   sizeof(partition_info));

		/* check that the copy was successful */
		if (status != OS_ERROR_OK_S) {
			goto out;
		}

		/* validate that the user owns this partition, and look up its handle */
		kernel_base =
		    lookup_user_partition(user_ctx, partition_info.user_base);

		if (kernel_base == NULL) {
			status = OS_ERROR_FAIL_S;
#ifdef SHW_DEBUG
			LOG_KDIAG("failed to find partition\n");
#endif				/*SHW_DEBUG */
			goto out;
		}

		/* Unmap the memory region (see sys_munmap in mmap.c) */
		ret = unmap_user_memory(partition_info.user_base, 8192);

		/* If the memory was successfully released */
		if (ret == OS_ERROR_OK_S) {

			/* release the partition */
			scc_release_partition(kernel_base);

			/* and remove it from the users context */
			deregister_user_partition(user_ctx,
						  partition_info.user_base);

			status = OS_ERROR_OK_S;

		} else {
#ifdef SHW_DEBUG
			LOG_KDIAG("do_munmap not successful!");
#endif
		}

	}
      out:
#endif				/* FSL_HAVE_SCC2 */
	return status;
}

static os_error_code shw_handle_scc_encrypt(fsl_shw_uco_t * user_ctx,
					    uint32_t info)
{
	os_error_code status = OS_ERROR_FAIL_S;
#ifdef FSL_HAVE_SCC2
	{
		fsl_shw_return_t retval;
		scc_region_t region_info;
		void *page_ctx = NULL;
		void *black_addr = NULL;
		void *partition_base = NULL;
		scc_config_t *scc_configuration;

		status =
		    os_copy_from_user(&region_info, (void *)info,
				      sizeof(region_info));

		if (status != OS_ERROR_OK_S) {
			goto out;
		}

		/* validate that the user owns this partition, and look up its handle */
		partition_base = lookup_user_partition(user_ctx,
						       region_info.
						       partition_base);

		if (partition_base == NULL) {
			status = OS_ERROR_FAIL_S;
#ifdef SHW_DEBUG
			LOG_KDIAG("failed to find secure partition\n");
#endif
			goto out;
		}

		/* Check that the memory size requested is correct */
		scc_configuration = scc_get_configuration();
		if (region_info.offset + region_info.length >
		    scc_configuration->partition_size_bytes) {
			status = OS_ERROR_FAIL_S;
			goto out;
		}

		/* wire down black_data */
		black_addr = wire_user_memory(region_info.black_data,
					      region_info.length, &page_ctx);

		if (black_addr == NULL) {
			status = OS_ERROR_FAIL_S;
			goto out;
		}

		retval =
		    do_scc_encrypt_region(NULL, partition_base,
					  region_info.offset,
					  region_info.length, black_addr,
					  region_info.IV,
					  region_info.cypher_mode);

		if (retval == FSL_RETURN_OK_S) {
			status = OS_ERROR_OK_S;
		} else {
			status = OS_ERROR_FAIL_S;
		}

		/* release black data */
		unwire_user_memory(&page_ctx);
	}
      out:

#endif				/* FSL_HAVE_SCC2 */
	return status;
}

static os_error_code shw_handle_scc_decrypt(fsl_shw_uco_t * user_ctx,
					    uint32_t info)
{
	os_error_code status = OS_ERROR_FAIL_S;
#ifdef FSL_HAVE_SCC2
	{
		fsl_shw_return_t retval;
		scc_region_t region_info;
		void *page_ctx = NULL;
		void *black_addr;
		void *partition_base;
		scc_config_t *scc_configuration;

		status =
		    os_copy_from_user(&region_info, (void *)info,
				      sizeof(region_info));

#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS
		    ("partition_base: %p, offset: %i, length: %i, black data: %p",
		     (void *)region_info.partition_base, region_info.offset,
		     region_info.length, (void *)region_info.black_data);
#endif

		if (status != OS_ERROR_OK_S) {
			goto out;
		}

		/* validate that the user owns this partition, and look up its handle */
		partition_base = lookup_user_partition(user_ctx,
						       region_info.
						       partition_base);

		if (partition_base == NULL) {
			status = OS_ERROR_FAIL_S;
#ifdef SHW_DEBUG
			LOG_KDIAG("failed to find partition\n");
#endif
			goto out;
		}

		/* Check that the memory size requested is correct */
		scc_configuration = scc_get_configuration();
		if (region_info.offset + region_info.length >
		    scc_configuration->partition_size_bytes) {
			status = OS_ERROR_FAIL_S;
			goto out;
		}

		/* wire down black_data */
		black_addr = wire_user_memory(region_info.black_data,
					      region_info.length, &page_ctx);

		if (black_addr == NULL) {
			status = OS_ERROR_FAIL_S;
			goto out;
		}

		retval =
		    do_scc_decrypt_region(NULL, partition_base,
					  region_info.offset,
					  region_info.length, black_addr,
					  region_info.IV,
					  region_info.cypher_mode);

		if (retval == FSL_RETURN_OK_S) {
			status = OS_ERROR_OK_S;
		} else {
			status = OS_ERROR_FAIL_S;
		}

		/* release black data */
		unwire_user_memory(&page_ctx);
	}
      out:

#endif				/* FSL_HAVE_SCC2 */
	return status;
}

fsl_shw_return_t do_system_keystore_slot_alloc(fsl_shw_uco_t * user_ctx,
					       uint32_t key_length,
					       uint64_t ownerid,
					       uint32_t * slot)
{
	(void)user_ctx;
	return keystore_slot_alloc(&system_keystore, key_length, ownerid, slot);
}

fsl_shw_return_t do_system_keystore_slot_dealloc(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot)
{
	(void)user_ctx;
	return keystore_slot_dealloc(&system_keystore, ownerid, slot);
}

fsl_shw_return_t do_system_keystore_slot_load(fsl_shw_uco_t * user_ctx,
					      uint64_t ownerid,
					      uint32_t slot,
					      const uint8_t * key,
					      uint32_t key_length)
{
	(void)user_ctx;
	return keystore_slot_load(&system_keystore, ownerid, slot,
				  (void *)key, key_length);
}

fsl_shw_return_t do_system_keystore_slot_encrypt(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot,
						 uint32_t key_length,
						 uint8_t * black_data)
{
	(void)user_ctx;
	return keystore_slot_encrypt(NULL, &system_keystore, ownerid,
				     slot, key_length, black_data);
}

fsl_shw_return_t do_system_keystore_slot_decrypt(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot,
						 uint32_t key_length,
						 const uint8_t * black_data)
{
	(void)user_ctx;
	return keystore_slot_decrypt(NULL, &system_keystore, ownerid,
				     slot, key_length, black_data);
}

fsl_shw_return_t do_system_keystore_slot_read(fsl_shw_uco_t * user_ctx,
					      uint64_t ownerid,
					      uint32_t slot,
					      uint32_t key_length,
					      uint8_t * key_data)
{
	(void)user_ctx;

	return keystore_slot_read(&system_keystore, ownerid,
				  slot, key_length, key_data);
}

/*!
 * Handle user-mode Get Capabilities request
 *
 * Right now, this function can only have a failure if the user has failed to
 * provide a pointer to a location in user space with enough room to hold the
 * fsl_shw_pco_t structure and any associated data.  It will treat this failure
 * as an ioctl failure and return an ioctl error code, instead of treating it
 * as an API failure.
 *
 * @param user_ctx    The kernel version of user's context
 * @param user_mode_pco_request  Pointer to user-space request
 *
 * @return an os_error_code
 */
static os_error_code get_capabilities(fsl_shw_uco_t * user_ctx,
				      void *user_mode_pco_request)
{
	os_error_code code;
	struct capabilities_req req;
	fsl_shw_pco_t local_cap;

	memcpy(&local_cap, &cap, sizeof(cap));
	/* Initialize pointers to out-of-struct arrays */
	local_cap.sym_algorithms = NULL;
	local_cap.sym_modes = NULL;
	local_cap.sym_modes = NULL;

	code = os_copy_from_user(&req, user_mode_pco_request, sizeof(req));
	if (code == OS_ERROR_OK_S) {
		void *endcap;
		void *user_bounds;
#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS("SHE: Received get_cap request: 0x%p/%u/0x%x",
			       req.capabilities, req.size,
			       sizeof(fsl_shw_pco_t));
#endif
		endcap = req.capabilities + 1;	/* point to end of structure */
		user_bounds = (void *)req.capabilities + req.size;	/* end of area */

		/* First verify that request is big enough for the main structure */
		if (endcap >= user_bounds) {
			endcap = NULL;	/* No! */
		}

		/* Copy any Symmetric Algorithm suppport */
		if (cap.sym_algorithm_count != 0) {
			local_cap.sym_algorithms = endcap;
			endcap =
			    copy_array(endcap, user_bounds, cap.sym_algorithms,
				       sizeof(fsl_shw_key_alg_t),
				       cap.sym_algorithm_count);
		}

		/* Copy any Symmetric Modes suppport */
		if (cap.sym_mode_count != 0) {
			local_cap.sym_modes = endcap;
			endcap = copy_array(endcap, user_bounds, cap.sym_modes,
					    sizeof(fsl_shw_sym_mode_t),
					    cap.sym_mode_count);
		}

		/* Copy any Hash Algorithm suppport */
		if (cap.hash_algorithm_count != 0) {
			local_cap.hash_algorithms = endcap;
			endcap =
			    copy_array(endcap, user_bounds, cap.hash_algorithms,
				       sizeof(fsl_shw_hash_alg_t),
				       cap.hash_algorithm_count);
		}

		/* Now copy up the (possibly modified) main structure */
		if (endcap != NULL) {
			code =
			    os_copy_to_user(req.capabilities, &local_cap,
					    sizeof(cap));
		}

		if (endcap == NULL) {
			code = OS_ERROR_BAD_ADDRESS_S;
		}

		/* And return the FSL SHW code in the request structure. */
		if (code == OS_ERROR_OK_S) {
			code =
			    copy_fsl_code(user_mode_pco_request,
					  FSL_RETURN_OK_S);
		}
	}

	/* code may already be set to an error.  This is another error case.  */

#ifdef SHW_DEBUG
	LOG_KDIAG_ARGS("SHW: get capabilities returning %d", code);
#endif

	return code;
}

/*!
 * Handle user-mode Get Results request
 *
 * Get arguments from user space into kernel space, then call
 * fsl_shw_get_results, and then copy its return code and any results from
 * kernel space back to user space.
 *
 * @param user_ctx    The kernel version of user's context
 * @param user_mode_results_req  Pointer to user-space request
 *
 * @return an os_error_code
 */
static os_error_code get_results(fsl_shw_uco_t * user_ctx,
				 void *user_mode_results_req)
{
	os_error_code code;
	struct results_req req;
	fsl_shw_result_t *results = NULL;
	int loop;

	code = os_copy_from_user(&req, user_mode_results_req, sizeof(req));
	loop = 0;

	if (code == OS_ERROR_OK_S) {
		results = os_alloc_memory(req.requested * sizeof(*results), 0);
		if (results == NULL) {
			code = OS_ERROR_NO_MEMORY_S;
		}
	}

	if (code == OS_ERROR_OK_S) {
		fsl_shw_return_t err =
		    fsl_shw_get_results(user_ctx, req.requested,
					results, &req.actual);

		/* Send API return code up to user. */
		code = copy_fsl_code(user_mode_results_req, err);

		if ((code == OS_ERROR_OK_S) && (err == FSL_RETURN_OK_S)) {
			/* Now copy up the result count */
			code = os_copy_to_user(user_mode_results_req
					       + offsetof(struct results_req,
							  actual), &req.actual,
					       sizeof(req.actual));
			if ((code == OS_ERROR_OK_S) && (req.actual != 0)) {
				/* now copy up the results... */
				code = os_copy_to_user(req.results, results,
						       req.actual *
						       sizeof(*results));
			}
		}
	}

	if (results != NULL) {
		os_free_memory(results);
	}

	return code;
}

/*!
 * Process header of user-mode request.
 *
 * Mark header as User Mode request.  Update UCO's flags and reference fields
 * with current versions from the header.
 *
 * @param user_ctx  Pointer to kernel version of UCO.
 * @param hdr       Pointer to common part of user request.
 *
 * @return void
 */
inline static void process_hdr(fsl_shw_uco_t * user_ctx,
			       struct shw_req_header *hdr)
{
	hdr->flags |= FSL_UCO_USERMODE_USER;
	user_ctx->flags = hdr->flags;
	user_ctx->user_ref = hdr->user_ref;

	return;
}

/*!
 * Handle user-mode Get Random request
 *
 * @param user_ctx    The kernel version of user's context
 * @param user_mode_get_random_req  Pointer to user-space request
 *
 * @return an os_error_code
 */
static os_error_code get_random(fsl_shw_uco_t * user_ctx,
				void *user_mode_get_random_req)
{
	os_error_code code;
	struct get_random_req req;

	code = os_copy_from_user(&req, user_mode_get_random_req, sizeof(req));
	if (code == OS_ERROR_OK_S) {
		process_hdr(user_ctx, &req.hdr);
#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS
		    ("SHW: get_random() for %d bytes in %sblocking mode",
		     req.size,
		     (req.hdr.flags & FSL_UCO_BLOCKING_MODE) ? "" : "non-");
#endif
		req.hdr.code =
		    fsl_shw_get_random(user_ctx, req.size, req.random);

#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS("SHW: get_random() returning %d", req.hdr.code);
#endif

		/* Copy FSL function status back to user */
		code = copy_fsl_code(user_mode_get_random_req, req.hdr.code);
	}

	return code;
}

/*!
 * Handle user-mode Add Entropy request
 *
 * @param user_ctx    Pointer to the kernel version of user's context
 * @param user_mode_add_entropy_req  Address of user-space request
 *
 * @return an os_error_code
 */
static os_error_code add_entropy(fsl_shw_uco_t * user_ctx,
				 void *user_mode_add_entropy_req)
{
	os_error_code code;
	struct add_entropy_req req;
	uint8_t *local_buffer = NULL;

	code = os_copy_from_user(&req, user_mode_add_entropy_req, sizeof(req));
	if (code == OS_ERROR_OK_S) {
		local_buffer = os_alloc_memory(req.size, 0);	/* for random */
		if (local_buffer != NULL) {
			code =
			    os_copy_from_user(local_buffer, req.entropy,
					      req.size);
		}
		if (code == OS_ERROR_OK_S) {
			req.hdr.code = fsl_shw_add_entropy(user_ctx, req.size,
							   local_buffer);

			code =
			    copy_fsl_code(user_mode_add_entropy_req,
					  req.hdr.code);
		}
	}

	if (local_buffer != NULL) {
		os_free_memory(local_buffer);
	}

	return code;
}

/******************************************************************/
/* End User Mode Support                                          */
/******************************************************************/

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_register_user);
#endif
/* REQ-S2LRD-PINTFC-API-GEN-004 */
/*
 * Handle user registration.
 *
 * @param  user_ctx   The user context for the registration.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_register_user(fsl_shw_uco_t * user_ctx)
{
	fsl_shw_return_t code = FSL_RETURN_INTERNAL_ERROR_S;

	if ((user_ctx->flags & FSL_UCO_BLOCKING_MODE) &&
	    (user_ctx->flags & FSL_UCO_CALLBACK_MODE)) {
		code = FSL_RETURN_BAD_FLAG_S;
		goto error_exit;
	} else if (user_ctx->pool_size == 0) {
		code = FSL_RETURN_NO_RESOURCE_S;
		goto error_exit;
	} else {
		user_ctx->result_pool.head = NULL;
		user_ctx->result_pool.tail = NULL;
		SHW_ADD_USER(user_ctx);
		code = FSL_RETURN_OK_S;
	}

      error_exit:
	return code;
}

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_deregister_user);
#endif
/* REQ-S2LRD-PINTFC-API-GEN-005 */
/*!
 * Destroy the association between the the user and the provider of the API.
 *
 * @param  user_ctx   The user context which is no longer needed.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_deregister_user(fsl_shw_uco_t * user_ctx)
{
	shw_queue_entry_t *finished_request;
	fsl_shw_return_t ret = FSL_RETURN_OK_S;

	/* Clean up what we find in result pool. */
	do {
		os_lock_context_t lock_context;
		os_lock_save_context(shw_queue_lock, lock_context);
		finished_request = user_ctx->result_pool.head;

		if (finished_request != NULL) {
			SHW_QUEUE_REMOVE_ENTRY(&user_ctx->result_pool,
					       finished_request);
			os_unlock_restore_context(shw_queue_lock, lock_context);
			os_free_memory(finished_request);
		} else {
			os_unlock_restore_context(shw_queue_lock, lock_context);
		}
	} while (finished_request != NULL);

#ifdef FSL_HAVE_SCC2
	{
		fsl_shw_spo_t *partition;
		struct mm_struct *mm = current->mm;

		while ((user_ctx->partition != NULL)
		       && (ret == FSL_RETURN_OK_S)) {

			partition = user_ctx->partition;

#ifdef SHW_DEBUG
			LOG_KDIAG_ARGS
			    ("Found an abandoned secure partition at %p, releasing",
			     partition);
#endif

			/* It appears that current->mm is not valid if this is called from a
			 * close routine (perhaps only if the program raised an exception that
			 * caused it to close?)  If that is the case, then still free the
			 * partition, but do not remove it from the memory space (dangerous?)
			 */

			if (mm == NULL) {
#ifdef SHW_DEBUG
				LOG_KDIAG
				    ("Warning: no mm structure found, not unmapping "
				     "partition from user memory\n");
#endif
			} else {
				/* Unmap the memory region (see sys_munmap in mmap.c) */
				/* Note that this assumes a single memory partition */
				unmap_user_memory(partition->user_base, 8192);
			}

			/* If the memory was successfully released */
			if (ret == OS_ERROR_OK_S) {
				/* release the partition */
				scc_release_partition(partition->kernel_base);

				/* and remove it from the users context */
				deregister_user_partition(user_ctx,
							  partition->user_base);

				ret = FSL_RETURN_OK_S;
			} else {
				ret = FSL_RETURN_ERROR_S;

				goto out;
			}
		}
	}
      out:
#endif				/* FSL_HAVE_SCC2 */

	SHW_REMOVE_USER(user_ctx);

	return ret;
}

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_get_results);
#endif
/* REQ-S2LRD-PINTFC-API-GEN-006 */
fsl_shw_return_t fsl_shw_get_results(fsl_shw_uco_t * user_ctx,
				     unsigned result_size,
				     fsl_shw_result_t results[],
				     unsigned *result_count)
{
	shw_queue_entry_t *finished_request;
	unsigned loop = 0;

	do {
		os_lock_context_t lock_context;

		/* Protect state of user's result pool until we have retrieved and
		 * remove the first entry, or determined that the pool is empty. */
		os_lock_save_context(shw_queue_lock, lock_context);
		finished_request = user_ctx->result_pool.head;

		if (finished_request != NULL) {
			uint32_t code = 0;

			SHW_QUEUE_REMOVE_ENTRY(&user_ctx->result_pool,
					       finished_request);
			os_unlock_restore_context(shw_queue_lock, lock_context);

			results[loop].user_ref = finished_request->user_ref;
			results[loop].code = finished_request->code;
			results[loop].detail1 = 0;
			results[loop].detail2 = 0;
			results[loop].user_req =
			    finished_request->user_mode_req;
			if (finished_request->postprocess != NULL) {
				code =
				    finished_request->
				    postprocess(finished_request);
			}

			results[loop].code = finished_request->code;
			os_free_memory(finished_request);
			if (code == 0) {
				loop++;
			}
		} else {	/* finished_request is NULL */
			/* pool is empty */
			os_unlock_restore_context(shw_queue_lock, lock_context);
		}

	} while ((loop < result_size) && (finished_request != NULL));

	*result_count = loop;

	return FSL_RETURN_OK_S;
}

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_get_capabilities);
#endif
fsl_shw_pco_t *fsl_shw_get_capabilities(fsl_shw_uco_t * user_ctx)
{

	/* Unused */
	(void)user_ctx;

	return &cap;
}

#if !(defined(FSL_HAVE_SAHARA) || defined(FSL_HAVE_RNGA)                    \
      || defined(FSL_HAVE_RNGB) || defined(FSL_HAVE_RNGC))

#if defined(LINUX_VERSION_CODE)
EXPORT_SYMBOL(fsl_shw_get_random);
#endif
fsl_shw_return_t fsl_shw_get_random(fsl_shw_uco_t * user_ctx,
				    uint32_t length, uint8_t * data)
{

	/* Unused */
	(void)user_ctx;
	(void)length;
	(void)data;

	return FSL_RETURN_ERROR_S;
}

#if defined(LINUX_VERSION_CODE)
EXPORT_SYMBOL(fsl_shw_add_entropy);
#endif
fsl_shw_return_t fsl_shw_add_entropy(fsl_shw_uco_t * user_ctx,
				     uint32_t length, uint8_t * data)
{

	/* Unused */
	(void)user_ctx;
	(void)length;
	(void)data;

	return FSL_RETURN_ERROR_S;
}
#endif

#if !defined(FSL_HAVE_DRYICE) && !defined(FSL_HAVE_SAHARA2)
#if 0
#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_symmetric_decrypt);
#endif
fsl_shw_return_t fsl_shw_symmetric_decrypt(fsl_shw_uco_t * user_ctx,
					   fsl_shw_sko_t * key_info,
					   fsl_shw_scco_t * sym_ctx,
					   uint32_t length,
					   const uint8_t * ct, uint8_t * pt)
{

	/* Unused */
	(void)user_ctx;
	(void)key_info;
	(void)sym_ctx;
	(void)length;
	(void)ct;
	(void)pt;

	return FSL_RETURN_ERROR_S;
}

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_symmetric_encrypt);
#endif
fsl_shw_return_t fsl_shw_symmetric_encrypt(fsl_shw_uco_t * user_ctx,
					   fsl_shw_sko_t * key_info,
					   fsl_shw_scco_t * sym_ctx,
					   uint32_t length,
					   const uint8_t * pt, uint8_t * ct)
{

	/* Unused */
	(void)user_ctx;
	(void)key_info;
	(void)sym_ctx;
	(void)length;
	(void)pt;
	(void)ct;

	return FSL_RETURN_ERROR_S;
}

/* DryIce support provided in separate file */

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_establish_key);
#endif
fsl_shw_return_t fsl_shw_establish_key(fsl_shw_uco_t * user_ctx,
				       fsl_shw_sko_t * key_info,
				       fsl_shw_key_wrap_t establish_type,
				       const uint8_t * key)
{

	/* Unused */
	(void)user_ctx;
	(void)key_info;
	(void)establish_type;
	(void)key;

	return FSL_RETURN_ERROR_S;
}

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_extract_key);
#endif
fsl_shw_return_t fsl_shw_extract_key(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info,
				     uint8_t * covered_key)
{

	/* Unused */
	(void)user_ctx;
	(void)key_info;
	(void)covered_key;

	return FSL_RETURN_ERROR_S;
}

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_release_key);
#endif
fsl_shw_return_t fsl_shw_release_key(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info)
{

	/* Unused */
	(void)user_ctx;
	(void)key_info;

	return FSL_RETURN_ERROR_S;
}
#endif
#endif				/* SAHARA or DRYICE */

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_hash);
#endif
#if !defined(FSL_HAVE_SAHARA)
fsl_shw_return_t fsl_shw_hash(fsl_shw_uco_t * user_ctx,
			      fsl_shw_hco_t * hash_ctx,
			      const uint8_t * msg,
			      uint32_t length,
			      uint8_t * result, uint32_t result_len)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

	/* Unused */
	(void)user_ctx;
	(void)hash_ctx;
	(void)msg;
	(void)length;
	(void)result;
	(void)result_len;

	return ret;
}
#endif

#ifndef FSL_HAVE_SAHARA
#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_hmac_precompute);
#endif

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

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_hmac);
#endif

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
#endif

/*!
 * Call the proper function to encrypt a region of encrypted secure memory
 *
 * @brief
 *
 * @param   user_ctx        User context of the partition owner (NULL in kernel)
 * @param   partition_base  Base address (physical) of the partition
 * @param   offset_bytes    Offset from base address of the data to be encrypted
 * @param   byte_count      Length of the message (bytes)
 * @param   black_data      Pointer to where the encrypted data is stored
 * @param   IV              IV to use for encryption
 * @param   cypher_mode     Cyphering mode to use, specified by type
 *                          #fsl_shw_cypher_mode_t
 *
 * @return  status
 */
fsl_shw_return_t
do_scc_encrypt_region(fsl_shw_uco_t * user_ctx,
		      void *partition_base, uint32_t offset_bytes,
		      uint32_t byte_count, uint8_t * black_data,
		      uint32_t * IV, fsl_shw_cypher_mode_t cypher_mode)
{
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;
#ifdef FSL_HAVE_SCC2

	scc_return_t scc_ret;

#ifdef SHW_DEBUG
	uint32_t *owner_32 = (uint32_t *) & (owner_id);

	LOG_KDIAG_ARGS
	    ("partition base: %p, offset: %i, count: %i, black data: %p\n",
	     partition_base, offset_bytes, byte_count, (void *)black_data);

	LOG_KDIAG_ARGS("Owner ID: %08x%08x\n", owner_32[1], owner_32[0]);
#endif				/* SHW_DEBUG */
	(void)user_ctx;

	os_cache_flush_range(black_data, byte_count);

	scc_ret =
	    scc_encrypt_region((uint32_t) partition_base, offset_bytes,
			       byte_count, __virt_to_phys(black_data), IV,
			       cypher_mode);

	if (scc_ret == SCC_RET_OK) {
		retval = FSL_RETURN_OK_S;
	} else {
		retval = FSL_RETURN_ERROR_S;
	}

	/* The SCC2 DMA engine should have written to the black ram, so we need to
	 * invalidate that region of memory.  Note that the red ram is not an
	 * because it is mapped with the cache disabled.
	 */
	os_cache_inv_range(black_data, byte_count);

#endif				/* FSL_HAVE_SCC2 */
	return retval;
}

/*!
 * Call the proper function to decrypt a region of encrypted secure memory
 *
 * @brief
 *
 * @param   user_ctx        User context of the partition owner (NULL in kernel)
 * @param   partition_base  Base address (physical) of the partition
 * @param   offset_bytes    Offset from base address that the decrypted data
 *                          shall be placed
 * @param   byte_count      Length of the message (bytes)
 * @param   black_data      Pointer to where the encrypted data is stored
 * @param   IV              IV to use for decryption
 * @param   cypher_mode     Cyphering mode to use, specified by type
 *                          #fsl_shw_cypher_mode_t
 *
 * @return  status
 */
fsl_shw_return_t
do_scc_decrypt_region(fsl_shw_uco_t * user_ctx,
		      void *partition_base, uint32_t offset_bytes,
		      uint32_t byte_count, const uint8_t * black_data,
		      uint32_t * IV, fsl_shw_cypher_mode_t cypher_mode)
{
	fsl_shw_return_t retval = FSL_RETURN_ERROR_S;

#ifdef FSL_HAVE_SCC2

	scc_return_t scc_ret;

#ifdef SHW_DEBUG
	uint32_t *owner_32 = (uint32_t *) & (owner_id);

	LOG_KDIAG_ARGS
	    ("partition base: %p, offset: %i, count: %i, black data: %p\n",
	     partition_base, offset_bytes, byte_count, (void *)black_data);

	LOG_KDIAG_ARGS("Owner ID: %08x%08x\n", owner_32[1], owner_32[0]);
#endif				/* SHW_DEBUG */

	(void)user_ctx;

	/* The SCC2 DMA engine will be reading from the black ram, so we need to
	 * make sure that the data is pushed out of the cache.  Note that the red
	 * ram is not an issue because it is mapped with the cache disabled.
	 */
	os_cache_flush_range(black_data, byte_count);

	scc_ret =
	    scc_decrypt_region((uint32_t) partition_base, offset_bytes,
			       byte_count,
			       (uint8_t *) __virt_to_phys(black_data), IV,
			       cypher_mode);

	if (scc_ret == SCC_RET_OK) {
		retval = FSL_RETURN_OK_S;
	} else {
		retval = FSL_RETURN_ERROR_S;
	}

#endif				/* FSL_HAVE_SCC2 */

	return retval;
}

void *fsl_shw_smalloc(fsl_shw_uco_t * user_ctx,
		      uint32_t size, const uint8_t * UMID, uint32_t permissions)
{
#ifdef FSL_HAVE_SCC2
	int part_no;
	void *part_base;
	uint32_t part_phys;
	scc_config_t *scc_configuration;

	/* Check that the memory size requested is correct */
	scc_configuration = scc_get_configuration();
	if (size != scc_configuration->partition_size_bytes) {
		return NULL;
	}

	/* attempt to grab a partition. */
	if (scc_allocate_partition(0, &part_no, &part_base, &part_phys)
	    != SCC_RET_OK) {
		return NULL;
	}
#ifdef SHW_DEBUG
	LOG_KDIAG_ARGS("Partition_base: %p, partition_base_phys: %p\n",
		       part_base, (void *)part_phys);
#endif

	if (scc_engage_partition(part_base, UMID, permissions)
	    != SCC_RET_OK) {
		/* Engagement failed, so the partition needs to be de-allocated */

#ifdef SHW_DEBUG
		LOG_KDIAG_ARGS("Failed to engage partition %p, de-allocating",
			       part_base);
#endif
		scc_release_partition(part_base);

		return NULL;
	}

	return part_base;

#else				/* FSL_HAVE_SCC2 */

	(void)user_ctx;
	(void)size;
	(void)UMID;
	(void)permissions;
	return NULL;

#endif				/* FSL_HAVE_SCC2 */
}

/* Release a block of secure memory */
fsl_shw_return_t fsl_shw_sfree(fsl_shw_uco_t * user_ctx, void *address)
{
	(void)user_ctx;

#ifdef FSL_HAVE_SCC2
	if (scc_release_partition(address) == SCC_RET_OK) {
		return FSL_RETURN_OK_S;
	}
#endif

	return FSL_RETURN_ERROR_S;
}

/* Check the status of a block of secure memory */
fsl_shw_return_t fsl_shw_sstatus(fsl_shw_uco_t * user_ctx,
				 void *address,
				 fsl_shw_partition_status_t * part_status)
{
	(void)user_ctx;

#ifdef FSL_HAVE_SCC2
	*part_status = scc_partition_status(address);

	return FSL_RETURN_OK_S;
#endif

	return FSL_RETURN_ERROR_S;
}

/* Diminish permissions on some secure memory */
fsl_shw_return_t fsl_shw_diminish_perms(fsl_shw_uco_t * user_ctx,
					void *address, uint32_t permissions)
{

	(void)user_ctx;		/* unused parameter warning */

#ifdef FSL_HAVE_SCC2
	if (scc_diminish_permissions(address, permissions) == SCC_RET_OK) {
		return FSL_RETURN_OK_S;
	}
#endif
	return FSL_RETURN_ERROR_S;
}

#ifndef FSL_HAVE_SAHARA
#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_gen_encrypt);
#endif

fsl_shw_return_t fsl_shw_gen_encrypt(fsl_shw_uco_t * user_ctx,
				     fsl_shw_acco_t * auth_ctx,
				     fsl_shw_sko_t * cipher_key_info,
				     fsl_shw_sko_t * auth_key_info,
				     uint32_t auth_data_length,
				     const uint8_t * auth_data,
				     uint32_t payload_length,
				     const uint8_t * payload,
				     uint8_t * ct, uint8_t * auth_value)
{
	volatile fsl_shw_return_t status = FSL_RETURN_ERROR_S;

	/* Unused */
	(void)user_ctx;
	(void)auth_ctx;
	(void)cipher_key_info;
	(void)auth_key_info;	/* save compilation warning */
	(void)auth_data_length;
	(void)auth_data;
	(void)payload_length;
	(void)payload;
	(void)ct;
	(void)auth_value;

	return status;
}

#ifdef LINUX_VERSION_CODE
EXPORT_SYMBOL(fsl_shw_auth_decrypt);
#endif
/*!
 * @brief Authenticate and decrypt a (CCM) stream.
 *
 * @param user_ctx         The user's context
 * @param auth_ctx         Info on this Auth operation
 * @param cipher_key_info  Key to encrypt payload
 * @param auth_key_info    (unused - same key in CCM)
 * @param auth_data_length Length in bytes of @a auth_data
 * @param auth_data        Any auth-only data
 * @param payload_length   Length in bytes of @a payload
 * @param ct               The encrypted data
 * @param auth_value       The authentication code to validate
 * @param[out] payload     The location to store decrypted data
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_auth_decrypt(fsl_shw_uco_t * user_ctx,
				      fsl_shw_acco_t * auth_ctx,
				      fsl_shw_sko_t * cipher_key_info,
				      fsl_shw_sko_t * auth_key_info,
				      uint32_t auth_data_length,
				      const uint8_t * auth_data,
				      uint32_t payload_length,
				      const uint8_t * ct,
				      const uint8_t * auth_value,
				      uint8_t * payload)
{
	volatile fsl_shw_return_t status = FSL_RETURN_ERROR_S;

	/* Unused */
	(void)user_ctx;
	(void)auth_ctx;
	(void)cipher_key_info;
	(void)auth_key_info;	/* save compilation warning */
	(void)auth_data_length;
	(void)auth_data;
	(void)payload_length;
	(void)ct;
	(void)auth_value;
	(void)payload;

	return status;
}

#endif				/* no SAHARA */

#ifndef FSL_HAVE_DRYICE

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
	volatile fsl_shw_return_t status = FSL_RETURN_ERROR_S;

	return status;
}

#endif				/* not have DRYICE */

fsl_shw_return_t alloc_slot(fsl_shw_uco_t * user_ctx, fsl_shw_sko_t * key_info)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;

	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		ret = do_system_keystore_slot_alloc(user_ctx,
						    key_info->key_length,
						    key_info->userid,
						    &(key_info->handle));
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("key length: %i, handle: %i",
			      key_info->key_length, key_info->handle);
#endif

	} else {
		/* Key goes in user keystore */
		ret = keystore_slot_alloc(key_info->keystore,
					  key_info->key_length,
					  key_info->userid,
					  &(key_info->handle));
	}

	return ret;
}				/* end fn alloc_slot */

fsl_shw_return_t load_slot(fsl_shw_uco_t * user_ctx,
			   fsl_shw_sko_t * key_info, const uint8_t * key)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;

	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		ret = do_system_keystore_slot_load(user_ctx,
						   key_info->userid,
						   key_info->handle, key,
						   key_info->key_length);
	} else {
		/* Key goes in user keystore */
		ret = keystore_slot_load(key_info->keystore,
					 key_info->userid,
					 key_info->handle, key,
					 key_info->key_length);
	}

	return ret;
}				/* end fn load_slot */

fsl_shw_return_t dealloc_slot(fsl_shw_uco_t * user_ctx,
			      fsl_shw_sko_t * key_info)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;

	if (key_info->keystore == NULL) {
		/* Key goes in system keystore */
		do_system_keystore_slot_dealloc(user_ctx,
						key_info->userid,
						key_info->handle);
	} else {
		/* Key goes in user keystore */
		keystore_slot_dealloc(key_info->keystore,
				      key_info->userid, key_info->handle);
	}

	key_info->flags &= ~(FSL_SKO_KEY_ESTABLISHED | FSL_SKO_KEY_PRESENT);

	return ret;
}				/* end fn slot_dealloc */
