/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sah_driver_interface.c
*
* @brief Provides a Linux Kernel Module interface to the SAHARA h/w device.
*
*/

/* SAHARA Includes */
#include <sah_driver_common.h>
#include <sah_kernel.h>
#include <sah_memory_mapper.h>
#include <sah_queue_manager.h>
#include <sah_status_manager.h>
#include <sah_interrupt_handler.h>
#include <sah_hardware_interface.h>
#include <fsl_shw_keystore.h>
#include <adaptor.h>
#ifdef FSL_HAVE_SCC
#include <linux/mxc_scc_driver.h>
#else
#include <linux/mxc_scc2_driver.h>
#endif

#ifdef DIAG_DRV_IF
#include <diagnostic.h>
#endif

#if defined(CONFIG_DEVFS_FS) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
#include <linux/devfs_fs_kernel.h>
#else
#include <linux/proc_fs.h>
#endif

#ifdef PERF_TEST
#define interruptible_sleep_on(x) sah_Handle_Interrupt()
#endif

#define TEST_MODE_OFF 1
#define TEST_MODE_ON 2

/*! Version register on first deployments */
#define SAHARA_VERSION2  2
/*! Version register on MX27 */
#define SAHARA_VERSION3  3
/*! Version register on MXC92323 */
#define SAHARA_VERSION4  4

/******************************************************************************
* Module function declarations
******************************************************************************/

OS_DEV_INIT_DCL(sah_init);
OS_DEV_SHUTDOWN_DCL(sah_cleanup);
OS_DEV_OPEN_DCL(sah_open);
OS_DEV_CLOSE_DCL(sah_release);
OS_DEV_IOCTL_DCL(sah_ioctl);
OS_DEV_MMAP_DCL(sah_mmap);

static os_error_code sah_handle_get_capabilities(fsl_shw_uco_t* user_ctx,
                                                 uint32_t info);

static void sah_user_callback(fsl_shw_uco_t * user_ctx);
static os_error_code sah_handle_scc_sfree(fsl_shw_uco_t* user_ctx,
                                          uint32_t info);
static os_error_code sah_handle_scc_sstatus(fsl_shw_uco_t* user_ctx,
                                          uint32_t info);
static os_error_code sah_handle_scc_drop_perms(fsl_shw_uco_t* user_ctx,
                                               uint32_t info);
static os_error_code sah_handle_scc_encrypt(fsl_shw_uco_t* user_ctx,
                                            uint32_t info);
static os_error_code sah_handle_scc_decrypt(fsl_shw_uco_t* user_ctx,
                                            uint32_t info);

#ifdef FSL_HAVE_SCC2
static fsl_shw_return_t register_user_partition(fsl_shw_uco_t * user_ctx,
						uint32_t user_base,
						void *kernel_base);
static fsl_shw_return_t deregister_user_partition(fsl_shw_uco_t * user_ctx,
						  uint32_t user_base);
#endif

static os_error_code sah_handle_sk_slot_alloc(uint32_t info);
static os_error_code sah_handle_sk_slot_dealloc(uint32_t info);
static os_error_code sah_handle_sk_slot_load(uint32_t info);
static os_error_code sah_handle_sk_slot_read(uint32_t info);
static os_error_code sah_handle_sk_slot_decrypt(uint32_t info);
static os_error_code sah_handle_sk_slot_encrypt(uint32_t info);

/*! Boolean flag for whether interrupt handler needs to be released on exit */
static unsigned interrupt_registered;

static int handle_sah_ioctl_dar(fsl_shw_uco_t * filp, uint32_t user_space_desc);

#if !defined(CONFIG_DEVFS_FS) || (LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0))
static int sah_read_procfs(char *buf,
			   char **start,
			   off_t offset, int count, int *eof, void *data);

static int sah_write_procfs(struct file *file, const char __user * buffer,
			    unsigned long count, void *data);
#endif

#if defined(CONFIG_DEVFS_FS) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))

/* This is a handle to the sahara DEVFS entry. */
static devfs_handle_t Sahara_devfs_handle;

#else

/* Major number assigned to our device driver */
static int Major;

/* This is a handle to the sahara PROCFS entry */
static struct proc_dir_entry *Sahara_procfs_handle;

#endif

uint32_t sah_hw_version;
extern void *sah_virt_base;

/* This is the wait queue to this driver.  Linux declaration. */
DECLARE_WAIT_QUEUE_HEAD(Wait_queue);

/* This is a global variable that is used to track how many times the device
 * has been opened simultaneously. */
#ifdef DIAG_DRV_IF
static int Device_in_use = 0;
#endif

/* This is the system keystore object */
fsl_shw_kso_t system_keystore;

/*!
 * OS-dependent handle used for registering user interface of a driver.
 */
static os_driver_reg_t reg_handle;

#ifdef DIAG_DRV_IF
/* This is for sprintf() to use when constructing output. */
#define DIAG_MSG_SIZE   1024
static char Diag_msg[DIAG_MSG_SIZE];
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18))
/** Pointer to Sahara clock information.  Initialized during os_dev_init(). */
static struct clk *sah_clk;
#endif

/*!
*******************************************************************************
* This function gets called when the module is inserted (insmod) into  the
* running kernel.
*
* @brief     SAHARA device initialisation function.
*
* @return   0 on success
* @return   -EBUSY if the device or proc file entry cannot be created.
* @return   OS_ERROR_NO_MEMORY_S if kernel memory could not be allocated.
* @return   OS_ERROR_FAIL_S if initialisation of proc entry failed
*/
OS_DEV_INIT(sah_init)
{
	/* Status variable */
	int os_error_code = 0;
	uint32_t sah_phys_base = SAHARA_BASE_ADDR;


	interrupt_registered = 0;

	/* Enable the SAHARA Clocks */
#ifdef DIAG_DRV_IF
	LOG_KDIAG("SAHARA : Enabling the IPG and AHB clocks\n")
#endif				/*DIAG_DRV_IF */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
	    mxc_clks_enable(SAHARA2_CLK);
#else
	{
		sah_clk = clk_get(NULL, "sahara_clk");
		if (sah_clk != ERR_PTR(ENOENT))
			clk_enable(sah_clk);
	}
#endif

	if (cpu_is_mx53())
		sah_phys_base -= 0x20000000;

	sah_virt_base = (void *)ioremap(sah_phys_base, SZ_16K);
	if (sah_virt_base == NULL) {
		os_printk(KERN_ERR
			  "SAHARA: Register mapping failed\n");
		os_error_code = OS_ERROR_FAIL_S;
	}

	if (os_error_code == OS_ERROR_OK_S) {
		sah_hw_version = sah_HW_Read_Version();
		os_printk("Sahara HW Version is 0x%08x\n", sah_hw_version);

		/* verify code and hardware are version compatible */
		if ((sah_hw_version != SAHARA_VERSION2)
		    && (sah_hw_version != SAHARA_VERSION3)) {
			if (((sah_hw_version >> 8) & 0xff) != SAHARA_VERSION4) {
				os_printk
				    ("Sahara HW Version was not expected value.\n");
				os_error_code = OS_ERROR_FAIL_S;
			}
		}
	}

	if (os_error_code == OS_ERROR_OK_S) {
#ifdef DIAG_DRV_IF
		LOG_KDIAG("Calling sah_Init_Mem_Map to initialise "
			  "memory subsystem.");
#endif
		/* Do any memory-routine initialization */
		os_error_code = sah_Init_Mem_Map();
	}

	if (os_error_code == OS_ERROR_OK_S) {
#ifdef DIAG_DRV_IF
		LOG_KDIAG("Calling sah_HW_Reset() to Initialise the Hardware.");
#endif
		/* Initialise the hardware */
		os_error_code = sah_HW_Reset();
		if (os_error_code != OS_ERROR_OK_S) {
			os_printk
			    ("sah_HW_Reset() failed to Initialise the Hardware.\n");
		}

	}

	if (os_error_code == OS_ERROR_OK_S) {
#if defined(CONFIG_DEVFS_FS) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
		/* Register the DEVFS entry */
		Sahara_devfs_handle = devfs_register(NULL,
						     SAHARA_DEVICE_SHORT,
						     DEVFS_FL_AUTO_DEVNUM,
						     0, 0,
						     SAHARA_DEVICE_MODE,
						     &Fops, NULL);
		if (Sahara_devfs_handle == NULL) {
#ifdef DIAG_DRV_IF
			LOG_KDIAG
			    ("Registering the DEVFS character device failed.");
#endif				/* DIAG_DRV_IF */
			os_error_code = -EBUSY;
		}
#else				/* CONFIG_DEVFS_FS */
		/* Create the PROCFS entry. This is used to report the assigned device
		 * major number back to user-space. */
#if 1
		Sahara_procfs_handle = create_proc_entry(SAHARA_DEVICE_SHORT, 0700,	/* default mode */
							 NULL);	/* parent dir */
		if (Sahara_procfs_handle == NULL) {
#ifdef DIAG_DRV_IF
			LOG_KDIAG("Registering the PROCFS interface failed.");
#endif				/* DIAG_DRV_IF */
			os_error_code = OS_ERROR_FAIL_S;
		} else {
			Sahara_procfs_handle->nlink = 1;
			Sahara_procfs_handle->data = 0;
			Sahara_procfs_handle->read_proc = sah_read_procfs;
			Sahara_procfs_handle->write_proc = sah_write_procfs;
		}
#endif				/* #if 1 */
	}

	if (os_error_code == OS_ERROR_OK_S) {
#ifdef DIAG_DRV_IF
		LOG_KDIAG
		    ("Calling sah_Queue_Manager_Init() to Initialise the Queue "
		     "Manager.");
#endif
		/* Initialise the Queue Manager */
		if (sah_Queue_Manager_Init() != FSL_RETURN_OK_S) {
			os_error_code = -ENOMEM;
		}
	}
#ifndef SAHARA_POLL_MODE
	if (os_error_code == OS_ERROR_OK_S) {
#ifdef DIAG_DRV_IF
		LOG_KDIAG("Calling sah_Intr_Init() to Initialise the Interrupt "
			  "Handler.");
#endif
		/* Initialise the Interrupt Handler */
		os_error_code = sah_Intr_Init(&Wait_queue);
		if (os_error_code == OS_ERROR_OK_S) {
			interrupt_registered = 1;
		}
	}
#endif				/* ifndef SAHARA_POLL_MODE */

#ifdef SAHARA_POWER_MANAGEMENT
	if (os_error_code == OS_ERROR_OK_S) {
		/* set up dynamic power management (dmp) */
		os_error_code = sah_dpm_init();
	}
#endif

	if (os_error_code == OS_ERROR_OK_S) {
		os_driver_init_registration(reg_handle);
		os_driver_add_registration(reg_handle, OS_FN_OPEN,
					   OS_DEV_OPEN_REF(sah_open));
		os_driver_add_registration(reg_handle, OS_FN_IOCTL,
					   OS_DEV_IOCTL_REF(sah_ioctl));
		os_driver_add_registration(reg_handle, OS_FN_CLOSE,
					   OS_DEV_CLOSE_REF(sah_release));
		os_driver_add_registration(reg_handle, OS_FN_MMAP,
					   OS_DEV_MMAP_REF(sah_mmap));

		os_error_code =
		    os_driver_complete_registration(reg_handle, Major,
						    "sahara");

		if (os_error_code < OS_ERROR_OK_S) {
#ifdef DIAG_DRV_IF
			snprintf(Diag_msg, DIAG_MSG_SIZE,
				 "Registering the regular "
				 "character device failed with error code: %d\n",
				 os_error_code);
			LOG_KDIAG(Diag_msg);
#endif
		}
	}
#endif				/* CONFIG_DEVFS_FS */

	if (os_error_code == OS_ERROR_OK_S) {
		/* set up the system keystore, using the default keystore handler */
		fsl_shw_init_keystore_default(&system_keystore);

		if (fsl_shw_establish_keystore(NULL, &system_keystore)
		    == FSL_RETURN_OK_S) {
			os_error_code = OS_ERROR_OK_S;
		} else {
			os_error_code = OS_ERROR_FAIL_S;
		}

		if (os_error_code != OS_ERROR_OK_S) {
#ifdef DIAG_DRV_IF
			snprintf(Diag_msg, DIAG_MSG_SIZE,
				 "Registering the system keystore "
				 "failed with error code: %d\n", os_error_code);
			LOG_KDIAG(Diag_msg);
#endif
		}
	}

	if (os_error_code != OS_ERROR_OK_S) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
		cleanup_module();
#else
		sah_cleanup();
#endif
	}
#ifdef DIAG_DRV_IF
	else {
		LOG_KDIAG_ARGS("Sahara major node is %d\n", Major);
	}
#endif

/* Disabling the Clock after the driver has been registered fine.
    This is done to save power when Sahara is not in use.*/
#ifdef DIAG_DRV_IF
	LOG_KDIAG("SAHARA : Disabling the clocks\n")
#endif				/* DIAG_DRV_IF */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
		mxc_clks_disable(SAHARA2_CLK);
#else
	{
		if (sah_clk != ERR_PTR(ENOENT))
			clk_disable(sah_clk);
	}
#endif

	os_dev_init_return(os_error_code);
}

/*!
*******************************************************************************
* This function gets called when the module is removed (rmmod) from the running
* kernel.
*
* @brief     SAHARA device clean-up function.
*
* @return   void
*/
OS_DEV_SHUTDOWN(sah_cleanup)
{
	int ret_val = 0;

	printk(KERN_ALERT "Sahara going into cleanup\n");

	/* clear out the system keystore */
	fsl_shw_release_keystore(NULL, &system_keystore);

	/* Unregister the device */
#if defined(CONFIG_DEVFS_FS) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	devfs_unregister(Sahara_devfs_handle);
#else

	if (Sahara_procfs_handle != NULL) {
		remove_proc_entry(SAHARA_DEVICE_SHORT, NULL);
	}

	if (Major >= 0) {
		ret_val = os_driver_remove_registration(reg_handle);
	}
#ifdef DIAG_DRV_IF
	if (ret_val < 0) {
		snprintf(Diag_msg, DIAG_MSG_SIZE, "Error while attempting to "
			 "unregister the device: %d\n", ret_val);
		LOG_KDIAG(Diag_msg);
	}
#endif

#endif				/* CONFIG_DEVFS_FS */
	sah_Queue_Manager_Close();

#ifndef SAHARA_POLL_MODE
	if (interrupt_registered) {
		sah_Intr_Release();
		interrupt_registered = 0;
	}
#endif
	sah_Stop_Mem_Map();
#ifdef SAHARA_POWER_MANAGEMENT
	sah_dpm_close();
#endif

#ifdef DIAG_DRV_IF
	LOG_KDIAG("SAHARA : Disabling the clocks\n")
#endif				/* DIAG_DRV_IF */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
	    mxc_clks_disable(SAHARA2_CLK);
#else
	{
		if (sah_clk != ERR_PTR(ENOENT))
			clk_disable(sah_clk);
		clk_put(sah_clk);
	}
#endif

	os_dev_shutdown_return(OS_ERROR_OK_S);
}

/*!
*******************************************************************************
* This function simply increments the module usage count.
*
* @brief     SAHARA device open function.
*
* @param    inode Part of the kernel prototype.
* @param    file Part of the kernel prototype.
*
* @return   0 - Always returns 0 since any number of calls to this function are
*               allowed.
*
*/
OS_DEV_OPEN(sah_open)
{

#if defined(LINUX_VERSION) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
	MOD_INC_USE_COUNT;
#endif

#ifdef DIAG_DRV_IF
	Device_in_use++;
	snprintf(Diag_msg, DIAG_MSG_SIZE,
		 "Incrementing module use count to: %d ", Device_in_use);
	LOG_KDIAG(Diag_msg);
#endif

	os_dev_set_user_private(NULL);

	/* Return 0 to indicate success */
	os_dev_open_return(0);
}

/*!
*******************************************************************************
* This function simply decrements the module usage count.
*
* @brief     SAHARA device release function.
*
* @param    inode   Part of the kernel prototype.
* @param    file    Part of the kernel prototype.
*
* @return   0 - Always returns 0 since this function does not fail.
*/
OS_DEV_CLOSE(sah_release)
{
	fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

#if defined(LINUX_VERSION) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
	MOD_DEC_USE_COUNT;
#endif

#ifdef DIAG_DRV_IF
	Device_in_use--;
	snprintf(Diag_msg, DIAG_MSG_SIZE,
		 "Decrementing module use count to: %d ", Device_in_use);
	LOG_KDIAG(Diag_msg);
#endif

	if (user_ctx != NULL) {
		sah_handle_deregistration(user_ctx);
		os_free_memory(user_ctx);
		os_dev_set_user_private(NULL);
	}

	/* Return 0 to indicate success */
	os_dev_close_return(OS_ERROR_OK_S);
}

/*!
*******************************************************************************
* This function provides the IO Controls for the SAHARA driver. Three IO
* Controls are supported:
*
*   SAHARA_HWRESET and
*   SAHARA_SET_HA
*   SAHARA_CHK_TEST_MODE
*
* @brief SAHARA device IO Control function.
*
* @param    inode   Part of the kernel prototype.
* @param    filp    Part of the kernel prototype.
* @param    cmd     Part of the kernel prototype.
* @param    arg     Part of the kernel prototype.
*
* @return   0 on success
* @return   -EBUSY if the HA bit could not be set due to busy hardware.
* @return   -ENOTTY if an unsupported IOCTL was attempted on the device.
* @return   -EFAULT if put_user() fails
*/
OS_DEV_IOCTL(sah_ioctl)
{
	int status = 0;
	int test_mode;

	switch (os_dev_get_ioctl_op()) {
	case SAHARA_HWRESET:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_HWRESET IOCTL.");
#endif
		/* We need to reset the hardware. */
		sah_HW_Reset();

		/* Mark all the entries in the Queue Manager's queue with state
		 * SAH_STATE_RESET.
		 */
		sah_Queue_Manager_Reset_Entries();

		/* Wake up all sleeping write() calls. */
		wake_up_interruptible(&Wait_queue);
		break;
#ifdef SAHARA_HA_ENABLED
	case SAHARA_SET_HA:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SET_HA IOCTL.");
#endif				/* DIAG_DRV_IF */
		if (sah_HW_Set_HA() == ERR_INTERNAL) {
			status = -EBUSY;
		}
		break;
#endif				/* SAHARA_HA_ENABLED */

	case SAHARA_CHK_TEST_MODE:
		/* load test_mode */
		test_mode = TEST_MODE_OFF;
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_CHECK_TEST_MODE IOCTL.");
		test_mode = TEST_MODE_ON;
#endif				/* DIAG_DRV_IF */
#if defined(KERNEL_TEST) || defined(PERF_TEST)
		test_mode = TEST_MODE_ON;
#endif				/* KERNEL_TEST || PERF_TEST */
		/* copy test_mode back to user space.  put_user() is Linux fn */
		/* compiler warning `register': no problem found so ignored */
		status = put_user(test_mode, (int *)os_dev_get_ioctl_arg());
		break;

	case SAHARA_DAR:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_DAR IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			if (user_ctx != NULL) {
				status =
				    handle_sah_ioctl_dar(user_ctx,
							 os_dev_get_ioctl_arg
							 ());
			} else {
				status = OS_ERROR_FAIL_S;
			}

		}
		break;

	case SAHARA_GET_RESULTS:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_GET_RESULTS IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			if (user_ctx != NULL) {
				status =
				    sah_get_results_pointers(user_ctx,
							     os_dev_get_ioctl_arg
							     ());
			} else {
				status = OS_ERROR_FAIL_S;
			}
		}
		break;

	case SAHARA_REGISTER:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_REGISTER IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			if (user_ctx != NULL) {
				status = OS_ERROR_FAIL_S;	/* already registered */
			} else {
				user_ctx =
				    os_alloc_memory(sizeof(fsl_shw_uco_t),
						    GFP_KERNEL);
				if (user_ctx == NULL) {
					status = OS_ERROR_NO_MEMORY_S;
				} else {
					/* Copy UCO from user, but only as big as the common UCO */
					if (os_copy_from_user(user_ctx,
							      (void *)
							      os_dev_get_ioctl_arg
							      (),
							      offsetof
							      (fsl_shw_uco_t,
							       result_pool))) {
						status = OS_ERROR_FAIL_S;
					} else {
						os_dev_set_user_private
						    (user_ctx);
						status =
						    sah_handle_registration
						    (user_ctx);
					}
				}
			}
		}
		break;

		/* This ioctl cmd should disappear in favor of a close() routine.  */
	case SAHARA_DEREGISTER:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_DEREGISTER IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			if (user_ctx == NULL) {
				status = OS_ERROR_FAIL_S;
			} else {
				status = sah_handle_deregistration(user_ctx);
				os_free_memory(user_ctx);
				os_dev_set_user_private(NULL);
			}
		}
		break;
	case SAHARA_SCC_DROP_PERMS:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SCC_DROP_PERMS IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			/* drop permissions on the specified partition */
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			status =
			    sah_handle_scc_drop_perms(user_ctx,
						      os_dev_get_ioctl_arg());
		}
		break;

	case SAHARA_SCC_SFREE:
		/* Unmap the specified partition from the users space, and then
		 * free it for use by someone else.
		 */
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SCC_SFREE IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			status =
			    sah_handle_scc_sfree(user_ctx,
						 os_dev_get_ioctl_arg());
		}
		break;

	case SAHARA_SCC_SSTATUS:
		/* Unmap the specified partition from the users space, and then
		 * free it for use by someone else.
		 */
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SCC_SSTATUS IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			status =
			    sah_handle_scc_sstatus(user_ctx,
						   os_dev_get_ioctl_arg());
		}
		break;

	case SAHARA_SCC_ENCRYPT:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SCC_ENCRYPT IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			status =
			    sah_handle_scc_encrypt(user_ctx,
						   os_dev_get_ioctl_arg());
		}
		break;

	case SAHARA_SCC_DECRYPT:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SCC_DECRYPT IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			status =
			    sah_handle_scc_decrypt(user_ctx,
						   os_dev_get_ioctl_arg());
		}
		break;

	case SAHARA_SK_ALLOC:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SK_ALLOC IOCTL.");
#endif				/* DIAG_DRV_IF */
		status = sah_handle_sk_slot_alloc(os_dev_get_ioctl_arg());
		break;

	case SAHARA_SK_DEALLOC:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SK_DEALLOC IOCTL.");
#endif				/* DIAG_DRV_IF */
		status = sah_handle_sk_slot_dealloc(os_dev_get_ioctl_arg());
		break;

	case SAHARA_SK_LOAD:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SK_LOAD IOCTL.");
#endif				/* DIAG_DRV_IF */
		status = sah_handle_sk_slot_load(os_dev_get_ioctl_arg());
		break;
	case SAHARA_SK_READ:
#ifdef DIAG_DRV_IF
				LOG_KDIAG("SAHARA_SK_READ IOCTL.");
#endif				/* DIAG_DRV_IF */
				status = sah_handle_sk_slot_read(os_dev_get_ioctl_arg());
				break;

	case SAHARA_SK_SLOT_DEC:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SK_SLOT_DECRYPT IOCTL.");
#endif				/* DIAG_DRV_IF */
		status = sah_handle_sk_slot_decrypt(os_dev_get_ioctl_arg());
		break;

	case SAHARA_SK_SLOT_ENC:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_SK_SLOT_ENCRYPT IOCTL.");
#endif				/* DIAG_DRV_IF */
		status = sah_handle_sk_slot_encrypt(os_dev_get_ioctl_arg());
		break;
	case SAHARA_GET_CAPS:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("SAHARA_GET_CAPS IOCTL.");
#endif				/* DIAG_DRV_IF */
		{
			fsl_shw_uco_t *user_ctx = os_dev_get_user_private();

			status =
			    sah_handle_get_capabilities(user_ctx,
							os_dev_get_ioctl_arg());
		}
		break;

	default:
#ifdef DIAG_DRV_IF
		LOG_KDIAG("Unknown SAHARA IOCTL.");
#endif				/* DIAG_DRV_IF */
		status = OS_ERROR_FAIL_S;
	}

	os_dev_ioctl_return(status);
}

/* Fill in the user's capabilities structure */
static os_error_code sah_handle_get_capabilities(fsl_shw_uco_t * user_ctx,
						 uint32_t info)
{
	os_error_code status = OS_ERROR_FAIL_S;
	fsl_shw_pco_t capabilities;

	status = os_copy_from_user(&capabilities, (void *)info,
				   sizeof(fsl_shw_pco_t));

	if (status != OS_ERROR_OK_S) {
		goto out;
	}

	if (get_capabilities(user_ctx, &capabilities) == FSL_RETURN_OK_S) {
		status = os_copy_to_user((void *)info, &capabilities,
					 sizeof(fsl_shw_pco_t));
	}

      out:
	return status;
}

#ifdef FSL_HAVE_SCC2

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

#ifdef DIAG_DRV_IF
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

#ifdef DIAG_DRV_IF
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

#endif				/* FSL_HAVE_SCC2 */

static os_error_code sah_handle_scc_drop_perms(fsl_shw_uco_t * user_ctx,
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
#ifdef DIAG_DRV_IF
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

static os_error_code sah_handle_scc_sfree(fsl_shw_uco_t * user_ctx,
					  uint32_t info)
{
	os_error_code status = OS_ERROR_NO_MEMORY_S;
#ifdef FSL_HAVE_SCC2
	{
		scc_partition_info_t partition_info;
		void *kernel_base;
		int ret;

		status =
		    os_copy_from_user(&partition_info, (void *)info,
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
#ifdef DIAG_DRV_IF
			LOG_KDIAG("failed to find partition\n");
#endif				/*DIAG_DRV_IF */
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
		}
	}
      out:
#endif				/* FSL_HAVE_SCC2 */
	return status;
}

static os_error_code sah_handle_scc_sstatus(fsl_shw_uco_t * user_ctx,
					    uint32_t info)
{
	os_error_code status = OS_ERROR_NO_MEMORY_S;
#ifdef FSL_HAVE_SCC2
	{
		scc_partition_info_t partition_info;
		void *kernel_base;

		status =
		    os_copy_from_user(&partition_info, (void *)info,
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
#ifdef DIAG_DRV_IF
			LOG_KDIAG("failed to find partition\n");
#endif				/*DIAG_DRV_IF */
			goto out;
		}

		partition_info.status = scc_partition_status(kernel_base);

		status =
		    os_copy_to_user((void *)info, &partition_info,
				    sizeof(partition_info));
	}
      out:
#endif				/* FSL_HAVE_SCC2 */
	return status;
}

static os_error_code sah_handle_scc_encrypt(fsl_shw_uco_t * user_ctx,
					    uint32_t info)
{
	os_error_code os_err = OS_ERROR_FAIL_S;
#ifdef FSL_HAVE_SCC2
	{
		fsl_shw_return_t retval;
		scc_region_t region_info;
		void *page_ctx = NULL;
		void *black_addr = NULL;
		void *partition_base = NULL;
		scc_config_t *scc_configuration;

		os_err =
		    os_copy_from_user(&region_info, (void *)info,
				      sizeof(region_info));

		if (os_err != OS_ERROR_OK_S) {
			goto out;
		}
#ifdef DIAG_DRV_IF
		LOG_KDIAG_ARGS
		    ("partition_base: %p, offset: %i, length: %i, black data: %p",
		     (void *)region_info.partition_base, region_info.offset,
		     region_info.length, (void *)region_info.black_data);
#endif

		/* validate that the user owns this partition, and look up its handle */
		partition_base = lookup_user_partition(user_ctx,
						       region_info.
						       partition_base);

		if (partition_base == NULL) {
			retval = FSL_RETURN_ERROR_S;
#ifdef DIAG_DRV_IF
			LOG_KDIAG("failed to find secure partition\n");
#endif
			goto out;
		}

		/* Check that the memory size requested is correct */
		scc_configuration = scc_get_configuration();
		if (region_info.offset + region_info.length >
		    scc_configuration->partition_size_bytes) {
			retval = FSL_RETURN_ERROR_S;
			goto out;
		}

		/* wire down black data */
		black_addr = wire_user_memory(region_info.black_data,
					      region_info.length, &page_ctx);

		if (black_addr == NULL) {
			retval = FSL_RETURN_ERROR_S;
			goto out;
		}

		retval =
		    do_scc_encrypt_region(NULL, partition_base,
					  region_info.offset,
					  region_info.length, black_addr,
					  region_info.IV,
					  region_info.cypher_mode);

		/* release black data */
		unwire_user_memory(&page_ctx);

	      out:
		if (os_err == OS_ERROR_OK_S) {
			/* Return error code */
			region_info.code = retval;
			os_err =
			    os_copy_to_user((void *)info, &region_info,
					    sizeof(region_info));
		}
	}

#endif
	return os_err;
}

static os_error_code sah_handle_scc_decrypt(fsl_shw_uco_t * user_ctx,
					    uint32_t info)
{
	os_error_code os_err = OS_ERROR_FAIL_S;
#ifdef FSL_HAVE_SCC2
	{
		fsl_shw_return_t retval;
		scc_region_t region_info;
		void *page_ctx = NULL;
		void *black_addr;
		void *partition_base;
		scc_config_t *scc_configuration;

		os_err =
		    os_copy_from_user(&region_info, (void *)info,
				      sizeof(region_info));

		if (os_err != OS_ERROR_OK_S) {
			goto out;
		}
#ifdef DIAG_DRV_IF
		LOG_KDIAG_ARGS
		    ("partition_base: %p, offset: %i, length: %i, black data: %p",
		     (void *)region_info.partition_base, region_info.offset,
		     region_info.length, (void *)region_info.black_data);
#endif

		/* validate that the user owns this partition, and look up its handle */
		partition_base = lookup_user_partition(user_ctx,
						       region_info.
						       partition_base);

		if (partition_base == NULL) {
			retval = FSL_RETURN_ERROR_S;
#ifdef DIAG_DRV_IF
			LOG_KDIAG("failed to find partition\n");
#endif
			goto out;
		}

		/* Check that the memory size requested is correct */
		scc_configuration = scc_get_configuration();
		if (region_info.offset + region_info.length >
		    scc_configuration->partition_size_bytes) {
			retval = FSL_RETURN_ERROR_S;
			goto out;
		}

		/* wire down black data */
		black_addr = wire_user_memory(region_info.black_data,
					      region_info.length, &page_ctx);

		if (black_addr == NULL) {
			retval = FSL_RETURN_ERROR_S;
			goto out;
		}

		retval =
		    do_scc_decrypt_region(NULL, partition_base,
					  region_info.offset,
					  region_info.length, black_addr,
					  region_info.IV,
					  region_info.cypher_mode);

		/* release black data */
		unwire_user_memory(&page_ctx);

	      out:
		if (os_err == OS_ERROR_OK_S) {
			/* Return error code */
			region_info.code = retval;
			os_err =
			    os_copy_to_user((void *)info, &region_info,
					    sizeof(region_info));
		}
	}

#endif				/* FSL_HAVE_SCC2 */
	return os_err;
}

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

/*!
*******************************************************************************
* This function implements the smalloc() function for userspace programs, by
* making a call to the SCC2 mmap() function that acquires a region of secure
* memory on behalf of the user, and then maps it into the users memory space.
* Currently, the only memory size supported is that of a single SCC2 partition.
* Requests for other sized memory regions will fail.
*/
OS_DEV_MMAP(sah_mmap)
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

			sah_handle_registration(user_ctx);
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

/* Find the physical address of a key stored in the system keystore */
fsl_shw_return_t
system_keystore_get_slot_info(uint64_t owner_id, uint32_t slot,
			      uint32_t * address, uint32_t * slot_size_bytes)
{
	fsl_shw_return_t retval;
	void *kernel_address;

	/* First verify that the key access is valid */
	retval = system_keystore.slot_verify_access(system_keystore.user_data,
						    owner_id, slot);

	if (retval != FSL_RETURN_OK_S) {
#ifdef DIAG_DRV_IF
		LOG_KDIAG("verification failed");
#endif
		return retval;
	}

	if (address != NULL) {
#ifdef FSL_HAVE_SCC2
		kernel_address =
		    system_keystore.slot_get_address(system_keystore.user_data,
						     slot);
		(*address) = scc_virt_to_phys(kernel_address);
#else
		kernel_address =
		    system_keystore.slot_get_address((void *)&owner_id, slot);
		(*address) = (uint32_t) kernel_address;
#endif
	}

	if (slot_size_bytes != NULL) {
#ifdef FSL_HAVE_SCC2
		*slot_size_bytes =
		    system_keystore.slot_get_slot_size(system_keystore.
						       user_data, slot);
#else
		*slot_size_bytes =
		    system_keystore.slot_get_slot_size((void *)&owner_id, slot);
#endif
	}

	return retval;
}

static os_error_code sah_handle_sk_slot_alloc(uint32_t info)
{
	scc_slot_t slot_info;
	os_error_code os_err;
	scc_return_t scc_ret;

	os_err = os_copy_from_user(&slot_info, (void *)info, sizeof(slot_info));
	if (os_err == OS_ERROR_OK_S) {
		scc_ret = keystore_slot_alloc(&system_keystore,
					      slot_info.key_length,
					      slot_info.ownerid,
					      &slot_info.slot);
		if (scc_ret == SCC_RET_OK) {
			slot_info.code = FSL_RETURN_OK_S;
		} else if (scc_ret == SCC_RET_INSUFFICIENT_SPACE) {
			slot_info.code = FSL_RETURN_NO_RESOURCE_S;
		} else {
			slot_info.code = FSL_RETURN_ERROR_S;
		}

#ifdef DIAG_DRV_IF
		LOG_KDIAG_ARGS("key length: %i, handle: %i\n",
			       slot_info.key_length, slot_info.slot);
#endif

		/* Return error code and slot info */
		os_err =
		    os_copy_to_user((void *)info, &slot_info,
				    sizeof(slot_info));

		if (os_err != OS_ERROR_OK_S) {
			(void)keystore_slot_dealloc(&system_keystore,
						    slot_info.ownerid,
						    slot_info.slot);
		}
	}

	return os_err;
}

static os_error_code sah_handle_sk_slot_dealloc(uint32_t info)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;
	scc_slot_t slot_info;
	os_error_code os_err;
	scc_return_t scc_ret;

	os_err = os_copy_from_user(&slot_info, (void *)info, sizeof(slot_info));

	if (os_err == OS_ERROR_OK_S) {
		scc_ret = keystore_slot_dealloc(&system_keystore,
						slot_info.ownerid,
						slot_info.slot);

		if (scc_ret == SCC_RET_OK) {
			ret = FSL_RETURN_OK_S;
		} else {
			ret = FSL_RETURN_ERROR_S;
		}
		slot_info.code = ret;

		os_err =
		    os_copy_to_user((void *)info, &slot_info,
				    sizeof(slot_info));
	}

	return os_err;
}

static os_error_code sah_handle_sk_slot_load(uint32_t info)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;
	scc_slot_t slot_info;
	os_error_code os_err;
	uint8_t *key = NULL;

	os_err = os_copy_from_user(&slot_info, (void *)info, sizeof(slot_info));

	if (os_err == OS_ERROR_OK_S) {
		/* Allow slop in alloc in case we are rounding up to word multiple */
		key = os_alloc_memory(slot_info.key_length + 3, GFP_KERNEL);
		if (key == NULL) {
			ret = FSL_RETURN_NO_RESOURCE_S;
			os_err = OS_ERROR_NO_MEMORY_S;
		} else {
			os_err = os_copy_from_user(key, slot_info.key,
						   slot_info.key_length);
		}
	}

	if (os_err == OS_ERROR_OK_S) {
		unsigned key_length = slot_info.key_length;

		/* Round up if necessary, as SCC call wants a multiple of 32-bit
		 * values for the full object being loaded. */
		if ((key_length & 3) != 0) {
			key_length += 4 - (key_length & 3);
		}
		ret = keystore_slot_load(&system_keystore,
					 slot_info.ownerid, slot_info.slot, key,
					 key_length);

		slot_info.code = ret;
		os_err =
		    os_copy_to_user((void *)info, &slot_info,
				    sizeof(slot_info));
	}

	if (key != NULL) {
		memset(key, 0, slot_info.key_length);
		os_free_memory(key);
	}

	return os_err;
}

static os_error_code sah_handle_sk_slot_read(uint32_t info)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;
	scc_slot_t slot_info;
	os_error_code os_err;
	uint8_t *key = NULL;

	os_err = os_copy_from_user(&slot_info, (void *)info, sizeof(slot_info));

	if (os_err == OS_ERROR_OK_S) {

		/* This operation is not allowed for user keys */
		slot_info.code = FSL_RETURN_NO_RESOURCE_S;
		os_err =
		    os_copy_to_user((void *)info, &slot_info,
				    sizeof(slot_info));

		return os_err;
	}

	if (os_err == OS_ERROR_OK_S) {
		/* Allow slop in alloc in case we are rounding up to word multiple */
		key = os_alloc_memory(slot_info.key_length + 3, GFP_KERNEL);
		if (key == NULL) {
			ret = FSL_RETURN_NO_RESOURCE_S;
			os_err = OS_ERROR_NO_MEMORY_S;
		}
	}

	if (os_err == OS_ERROR_OK_S) {
		unsigned key_length = slot_info.key_length;

		/* @bug Do some PERMISSIONS checking - make sure this is SW key */

		/* Round up if necessary, as SCC call wants a multiple of 32-bit
		 * values for the full object being loaded. */
		if ((key_length & 3) != 0) {
			key_length += 4 - (key_length & 3);
		}
		ret = keystore_slot_read(&system_keystore,
					 slot_info.ownerid, slot_info.slot,
					 key_length, key);

		/* @bug do some error checking */

		/* Send key back to user */
		os_err = os_copy_to_user(slot_info.key, key,
					 slot_info.key_length);

		slot_info.code = ret;
		os_err =
		    os_copy_to_user((void *)info, &slot_info,
				    sizeof(slot_info));
	}

	if (key != NULL) {
		memset(key, 0, slot_info.key_length);
		os_free_memory(key);
	}

	return os_err;
}

static os_error_code sah_handle_sk_slot_encrypt(uint32_t info)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;
	scc_slot_t slot_info;
	os_error_code os_err;
	scc_return_t scc_ret;
	uint8_t *key = NULL;

	os_err = os_copy_from_user(&slot_info, (void *)info, sizeof(slot_info));

	if (os_err == OS_ERROR_OK_S) {
		key = os_alloc_memory(slot_info.key_length, GFP_KERNEL);
		if (key == NULL) {
			ret = FSL_RETURN_NO_RESOURCE_S;
		}
	}

	if (key != NULL) {

		scc_ret = keystore_slot_encrypt(NULL, &system_keystore,
						slot_info.ownerid,
						slot_info.slot,
						slot_info.key_length, key);

		if (scc_ret != SCC_RET_OK) {
			ret = FSL_RETURN_ERROR_S;
		} else {
			os_err =
			    os_copy_to_user(slot_info.key, key,
					    slot_info.key_length);
			if (os_err != OS_ERROR_OK_S) {
				ret = FSL_RETURN_INTERNAL_ERROR_S;
			} else {
				ret = FSL_RETURN_OK_S;
			}
		}

		slot_info.code = ret;
		os_err =
		    os_copy_to_user((void *)info, &slot_info,
				    sizeof(slot_info));

		memset(key, 0, slot_info.key_length);
		os_free_memory(key);
	}

	return os_err;
}

static os_error_code sah_handle_sk_slot_decrypt(uint32_t info)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;
	scc_slot_t slot_info;	/*!< decrypt request fields */
	os_error_code os_err;
	scc_return_t scc_ret;
	uint8_t *key = NULL;

	os_err = os_copy_from_user(&slot_info, (void *)info, sizeof(slot_info));

	if (os_err == OS_ERROR_OK_S) {
		key = os_alloc_memory(slot_info.key_length, GFP_KERNEL);
		if (key == NULL) {
			ret = FSL_RETURN_NO_RESOURCE_S;
			os_err = OS_ERROR_OK_S;
		} else {
			os_err = os_copy_from_user(key, slot_info.key,
						   slot_info.key_length);
		}
	}

	if (os_err == OS_ERROR_OK_S) {
		scc_ret = keystore_slot_decrypt(NULL, &system_keystore,
						slot_info.ownerid,
						slot_info.slot,
						slot_info.key_length, key);
		if (scc_ret == SCC_RET_OK) {
			ret = FSL_RETURN_OK_S;
		} else {
			ret = FSL_RETURN_ERROR_S;
		}

		slot_info.code = ret;
		os_err =
		    os_copy_to_user((void *)info, &slot_info,
				    sizeof(slot_info));
	}

	if (key != NULL) {
		memset(key, 0, slot_info.key_length);
		os_free_memory(key);
	}

	return os_err;
}

/*!
 * Register a user
 *
 * @brief Register a user
  *
 * @param  user_ctx  information about this user
 *
 * @return   status code
 */
fsl_shw_return_t sah_handle_registration(fsl_shw_uco_t * user_ctx)
{
	/* Initialize the user's result pool (like sah_Queue_Construct() */
	user_ctx->result_pool.head = NULL;
	user_ctx->result_pool.tail = NULL;
	user_ctx->result_pool.count = 0;

	/* initialize the user's partition chain */
	user_ctx->partition = NULL;

	return FSL_RETURN_OK_S;
}

/*!
 * Deregister a user
 *
 * @brief Deregister a user
 *
 * @param  user_ctx  information about this user
 *
 * @return   status code
 */
fsl_shw_return_t sah_handle_deregistration(fsl_shw_uco_t * user_ctx)
{
	/* NOTE:
	 * This will release any secure partitions that are held by the user.
	 * Encryption keys that were placed in the system keystore by the user
	 * should not be removed here, because they might have been shared with
	 * another process.  The user must be careful to release any that are no
	 * longer in use.
	 */
	fsl_shw_return_t ret = FSL_RETURN_OK_S;

#ifdef FSL_HAVE_SCC2
	fsl_shw_spo_t *partition;
	struct mm_struct *mm = current->mm;

	while ((user_ctx->partition != NULL) && (ret == FSL_RETURN_OK_S)) {

		partition = user_ctx->partition;

#ifdef DIAG_DRV_IF
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
#ifdef DIAG_DRV_IF
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
      out:
#endif				/* FSL_HAVE_SCC2 */

	return ret;
}

/*!
 * Sets up memory to extract results from results pool
 *
 * @brief Sets up memory to extract results from results pool
 *
 * @param  user_ctx  information about this user
 * @param[in,out]   arg contains input parameters and fields that the driver
 *                   fills in
 *
 * @return   os error code or 0 on success
 */
int sah_get_results_pointers(fsl_shw_uco_t * user_ctx, uint32_t arg)
{
	sah_results results_arg;	/* kernel mode usable version of 'arg' */
	fsl_shw_result_t *user_results;	/* user mode address of results */
	unsigned *user_actual;	/* user mode address of actual number of results */
	unsigned actual;	/* local memory of actual number of results */
	int ret_val = OS_ERROR_FAIL_S;
	sah_Head_Desc *finished_request;
	unsigned int loop;

	/* copy structure from user to kernel space */
	if (!os_copy_from_user(&results_arg, (void *)arg, sizeof(sah_results))) {
		/* save user space pointers */
		user_actual = results_arg.actual;	/* where count goes */
		user_results = results_arg.results;	/* where results goe */

		/* Set pointer for actual value to temporary kernel memory location */
		results_arg.actual = &actual;

		/* Allocate kernel memory to hold temporary copy of the results */
		results_arg.results =
		    os_alloc_memory(sizeof(fsl_shw_result_t) *
				    results_arg.requested, GFP_KERNEL);

		/* if memory allocated, continue */
		if (results_arg.results == NULL) {
			ret_val = OS_ERROR_NO_MEMORY_S;
		} else {
			fsl_shw_return_t get_status;

			/* get the results */
			get_status =
			    sah_get_results_from_pool(user_ctx, &results_arg);

			/* free the copy of the user space descriptor chain */
			for (loop = 0; loop < actual; ++loop) {
				/* get sah_Head_Desc from results and put user address into
				 * the return structure */
				finished_request =
				    results_arg.results[loop].user_desc;
				results_arg.results[loop].user_desc =
				    finished_request->user_desc;
				/* return the descriptor chain memory to the block free pool */
				sah_Free_Chained_Descriptors(finished_request);
			}

			/* if no errors, copy results and then the actual number of results
			 * back to user space
			 */
			if (get_status == FSL_RETURN_OK_S) {
				if (os_copy_to_user
				    (user_results, results_arg.results,
				     actual * sizeof(fsl_shw_result_t))
				    || os_copy_to_user(user_actual, &actual,
						       sizeof(user_actual))) {
					ret_val = OS_ERROR_FAIL_S;
				} else {
					ret_val = 0;	/* no error */
				}
			}
			/* free the allocated memory */
			os_free_memory(results_arg.results);
		}
	}

	return ret_val;
}

/*!
 * Extracts results from results pool
 *
 * @brief Extract results from results pool
 *
 * @param           user_ctx  information about this user
 * @param[in,out]   arg       contains input parameters and fields that the
 *                            driver fills in
 *
 * @return   status code
 */
fsl_shw_return_t sah_get_results_from_pool(volatile fsl_shw_uco_t * user_ctx,
					   sah_results * arg)
{
	sah_Head_Desc *finished_request;
	unsigned int loop = 0;
	os_lock_context_t int_flags;

	/* Get the number of results requested, up to total number of results
	 * available
	 */
	do {
		/* Protect state of user's result pool until we have retrieved and
		 * remove the first entry, or determined that the pool is empty. */
		os_lock_save_context(desc_queue_lock, int_flags);
		finished_request = user_ctx->result_pool.head;

		if (finished_request != NULL) {
			sah_Queue_Remove_Entry((sah_Queue *) & user_ctx->
					       result_pool);
			os_unlock_restore_context(desc_queue_lock, int_flags);

			/* Prepare to free. */
			(void)sah_DePhysicalise_Descriptors(finished_request);

			arg->results[loop].user_ref =
			    finished_request->user_ref;
			arg->results[loop].code = finished_request->result;
			arg->results[loop].detail1 =
			    finished_request->fault_address;
			arg->results[loop].detail2 = 0;
			arg->results[loop].user_desc = finished_request;

			loop++;
		} else {	/* finished_request is NULL */
			/* pool is empty */
			os_unlock_restore_context(desc_queue_lock, int_flags);
		}

	} while ((loop < arg->requested) && (finished_request != NULL));

	/* record number of results actually obtained */
	*arg->actual = loop;

	return FSL_RETURN_OK_S;
}

/*!
 * Converts descriptor chain to kernel space (from user space) and submits
 * chain to Sahara for processing
 *
 * @brief Submits converted descriptor chain to sahara
 *
 * @param  user_ctx        Pointer to Kernel version of user's ctx
 * @param  user_space_desc user space address of descriptor chain that is
 *                         in user space
 *
 * @return   OS status code
 */
static int handle_sah_ioctl_dar(fsl_shw_uco_t * user_ctx,
				uint32_t user_space_desc)
{
	int os_error_code = OS_ERROR_FAIL_S;
	sah_Head_Desc *desc_chain_head;	/* chain in kernel - virtual address */

	/* This will re-create the linked list so that the SAHARA hardware can
	 * DMA on it.
	 */
	desc_chain_head = sah_Copy_Descriptors(user_ctx,
					       (sah_Head_Desc *)
					       user_space_desc);

	if (desc_chain_head == NULL) {
		/* We may have failed due to a -EFAULT as well, but we will return
		 * OS_ERROR_NO_MEMORY_S since either way it is a memory related
		 * failure.
		 */
		os_error_code = OS_ERROR_NO_MEMORY_S;
	} else {
		fsl_shw_return_t stat;

		desc_chain_head->user_info = user_ctx;
		desc_chain_head->user_desc = (sah_Head_Desc *) user_space_desc;

		if (desc_chain_head->uco_flags & FSL_UCO_BLOCKING_MODE) {
#ifdef SAHARA_POLL_MODE
			sah_Handle_Poll(desc_chain_head);
#else
			sah_blocking_mode(desc_chain_head);
#endif
			stat = desc_chain_head->result;
			/* return the descriptor chain memory to the block free pool */
			sah_Free_Chained_Descriptors(desc_chain_head);
			/* Tell user how the call turned out */

			/* Copy 'result' back up to the result member.
			 *
			 * The dereference of the different member will cause correct the
			 * arithmetic to occur on the user-space address because of the
			 * missing dma/bus locations in the user mode version of the
			 * sah_Desc structure. */
			os_error_code =
			    os_copy_to_user((void *)(user_space_desc
						     + offsetof(sah_Head_Desc,
								uco_flags)),
					    &stat, sizeof(fsl_shw_return_t));

		} else {	/* not blocking mode - queue and forget */

			if (desc_chain_head->uco_flags & FSL_UCO_CALLBACK_MODE) {
				user_ctx->process = os_get_process_handle();
				user_ctx->callback = sah_user_callback;
			}
#ifdef SAHARA_POLL_MODE
			/* will put results in result pool */
			sah_Handle_Poll(desc_chain_head);
#else
			/* just put someting in the DAR */
			sah_Queue_Manager_Append_Entry(desc_chain_head);
#endif
			/* assume all went well */
			os_error_code = OS_ERROR_OK_S;
		}
	}

	return os_error_code;
}

static void sah_user_callback(fsl_shw_uco_t * user_ctx)
{
	os_send_signal(user_ctx->process, SIGUSR2);
}

/*!
 * This function is called when a thread attempts to read from the /proc/sahara
 * file.  Upon read, statistics and information about the state of the driver
 * are returned in nthe supplied buffer.
 *
 * @brief     SAHARA PROCFS read function.
 *
 * @param    buf     Anything written to this buffer will be returned to the
 *                   user-space process that is reading from this proc entry.
 * @param    start   Part of the kernel prototype.
 * @param    offset  Part of the kernel prototype.
 * @param    count   The size of the buf argument.
 * @param    eof     An integer which is set to one to tell the user-space
 *                   process that there is no more data to read.
 * @param    data    Part of the kernel prototype.
 *
 * @return   The number of bytes written to the proc entry.
 */
#if !defined(CONFIG_DEVFS_FS) || (LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0))
static int sah_read_procfs(char *buf,
			   char **start,
			   off_t offset, int count, int *eof, void *data)
{
	int output_bytes = 0;
	int in_queue_count = 0;
	os_lock_context_t lock_context;

	os_lock_save_context(desc_queue_lock, lock_context);
	in_queue_count = sah_Queue_Manager_Count_Entries(TRUE, 0);
	os_unlock_restore_context(desc_queue_lock, lock_context);
	output_bytes += snprintf(buf, count - output_bytes, "queued: %d\n",
				 in_queue_count);
	output_bytes += snprintf(buf + output_bytes, count - output_bytes,
				 "Descriptors: %d, "
				 "Interrupts %d (%d Done1Done2, %d Done1Busy2, "
				 " %d Done1)\n",
				 dar_count, interrupt_count, done1done2_count,
				 done1busy2_count, done1_count);
	output_bytes += snprintf(buf + output_bytes, count - output_bytes,
				 "Control: %08x\n", sah_HW_Read_Control());
#if !defined(FSL_HAVE_SAHARA4) || defined(SAHARA4_NO_USE_SQUIB)
	output_bytes += snprintf(buf + output_bytes, count - output_bytes,
				 "IDAR: %08x; CDAR: %08x\n",
				 sah_HW_Read_IDAR(), sah_HW_Read_CDAR());
#endif
#ifdef DIAG_DRV_STATUS
	output_bytes += snprintf(buf + output_bytes, count - output_bytes,
				 "Status: %08x; Error Status: %08x; Op Status: %08x\n",
				 sah_HW_Read_Status(),
				 sah_HW_Read_Error_Status(),
				 sah_HW_Read_Op_Status());
#endif
#ifdef FSL_HAVE_SAHARA4
	output_bytes += snprintf(buf + output_bytes, count - output_bytes,
				 "MMStat: %08x; Config: %08x\n",
				 sah_HW_Read_MM_Status(), sah_HW_Read_Config());
#endif

	/* Signal the end of the file */
	*eof = 1;

	/* To get rid of the unused parameter warnings */
	(void)start;
	(void)data;
	(void)offset;

	return output_bytes;
}

static int sah_write_procfs(struct file *file, const char __user * buffer,
			    unsigned long count, void *data)
{

	/* Any write to this file will reset all counts. */
	dar_count = interrupt_count = done1done2_count =
	    done1busy2_count = done1_count = 0;

	(void)file;
	(void)buffer;
	(void)data;

	return count;
}

#endif

#ifndef SAHARA_POLL_MODE
/*!
 * Block user call until processing is complete.
 *
 * @param entry    The user's request.
 *
 * @return   An OS error code, or 0 if no error
 */
int sah_blocking_mode(sah_Head_Desc * entry)
{
	int os_error_code = 0;
	sah_Queue_Status status;

	/* queue entry, put something in the DAR, if nothing is there currently */
	sah_Queue_Manager_Append_Entry(entry);

	/* get this descriptor chain's current status */
	status = ((volatile sah_Head_Desc *)entry)->status;

	while (!SAH_DESC_PROCESSED(status)) {
		extern sah_Queue *main_queue;

		DEFINE_WAIT(sahara_wait);	/* create a wait queue entry. Linux */

		/* enter the wait queue entry into the queue */
		prepare_to_wait(&Wait_queue, &sahara_wait, TASK_INTERRUPTIBLE);

		/* check if this entry has been processed */
		status = ((volatile sah_Head_Desc *)entry)->status;

		if (!SAH_DESC_PROCESSED(status)) {
			/* go to sleep - Linux */
			schedule();
		}

		/* un-queue the 'prepare to wait' queue? - Linux */
		finish_wait(&Wait_queue, &sahara_wait);

		/* signal belongs to this thread? */
		if (signal_pending(current)) {	/* Linux */
			os_lock_context_t lock_flags;

			/* don't allow access during this check and operation */
			os_lock_save_context(desc_queue_lock, lock_flags);
			status = ((volatile sah_Head_Desc *)entry)->status;
			if (status == SAH_STATE_PENDING) {
				sah_Queue_Remove_Any_Entry(main_queue, entry);
				entry->result = FSL_RETURN_INTERNAL_ERROR_S;
				((volatile sah_Head_Desc *)entry)->status =
				    SAH_STATE_FAILED;
			}
			os_unlock_restore_context(desc_queue_lock, lock_flags);
		}

		status = ((volatile sah_Head_Desc *)entry)->status;
	}			/* while ... */

	/* Do this so that caller can free */
	(void)sah_DePhysicalise_Descriptors(entry);

	return os_error_code;
}

/*!
 * If interrupt does not return in a reasonable time, time out, trigger
 * interrupt, and continue with process
 *
 * @param    data   ignored
 */
void sahara_timeout_handler(unsigned long data)
{
	/* Sahara has not issuing an interrupt, so timed out */
#ifdef DIAG_DRV_IF
	LOG_KDIAG("Sahara HW did not respond.  Resetting.\n");
#endif
	/* assume hardware needs resetting */
	sah_Handle_Interrupt(SAH_EXEC_FAULT);
	/* wake up sleeping thread to try again */
	wake_up_interruptible(&Wait_queue);
}

#endif				/* ifndef SAHARA_POLL_MODE */

/* End of sah_driver_interface.c */
