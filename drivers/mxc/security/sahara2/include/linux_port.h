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
 * @file linux_port.h
 *
 * OS_PORT ported to Linux (2.6.9+ for now)
 *
 */

 /*!
  * @if USE_MAINPAGE
  * @mainpage ==Linux version of== Generic OS API for STC Drivers
  * @endif
  *
  * @section intro_sec Introduction
  *
  * This API / kernel programming environment blah blah.
  *
  * See @ref dkops "Driver-to-Kernel Operations" as a good place to start.
  */

#ifndef LINUX_PORT_H
#define LINUX_PORT_H

#define PORTABLE_OS_VERSION 101

/* Linux Kernel Includes */
#include <linux/version.h>	/* Current version Linux kernel */

#if defined(CONFIG_MODVERSIONS) && ! defined(MODVERSIONS)
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
#include <linux/modversions.h>
#endif
#define MODVERSIONS
#endif
/*!
 * __NO_VERSION__ defined due to Kernel module possibly spanning multiple
 * files.
 */
#define __NO_VERSION__

#include <linux/module.h>	/* Basic support for loadable modules,
				   printk */
#include <linux/init.h>		/* module_init, module_exit */
#include <linux/kernel.h>	/* General kernel system calls */
#include <linux/sched.h>	/* for interrupt.h */
#include <linux/fs.h>		/* for inode */
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>		/* kmalloc */

#include <stdarg.h>

#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
#include <linux/device.h>  /* used in dynamic power management */
#else
#include <linux/platform_device.h>  /* used in dynamic power management */
#endif

#include <linux/dmapool.h>
#include <linux/dma-mapping.h>

#include <linux/clk.h>         /* clock en/disable for DPM */

#include <linux/dmapool.h>
#include <linux/dma-mapping.h>

#include <asm/uaccess.h>            /* copy_to_user(), copy_from_user() */
#include <asm/io.h>                 /* ioremap() */
#include <asm/irq.h>
#include <asm/cacheflush.h>

#include <mach/hardware.h>

#ifndef TRUE
/*! Useful symbol for unsigned values used as flags. */
#define TRUE 1
#endif

#ifndef FALSE
/*! Useful symbol for unsigned values used as flags. */
#define FALSE 0
#endif

/* These symbols are defined in Linux 2.6 and later.  Include here for minimal
 * support of 2.4 kernel.
 **/
#if !defined(LINUX_VERSION_CODE) || LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
/*!
 * Symbol defined somewhere in 2.5/2.6.  It is the return signature of an ISR.
 */
#define irqreturn_t void
/*! Possible return value of 'modern' ISR routine. */
#define IRQ_HANDLED
/*! Method of generating value of 'modern' ISR routine. */
#define IRQ_RETVAL(x)
#endif

/*!
 * Type used for registering and deregistering interrupts.
 */
typedef int os_interrupt_id_t;

/*!
 * Type used as handle for a process
 *
 * See #os_get_process_handle() and #os_send_signal().
 */
/*
 * The following should be defined this way, but it gets compiler errors
 * on the current tool chain.
 *
 * typedef task_t *os_process_handle_t;
 */

#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
typedef task_t *os_process_handle_t;
#else
typedef struct task_struct *os_process_handle_t;
#endif

/*!
 * Generic return code for functions which need such a thing.
 *
 * No knowledge should be assumed of the value of any of these symbols except
 * that @c OS_ERROR_OK_S is guaranteed to be zero.
 */
typedef enum {
	OS_ERROR_OK_S = 0,	/*!< Success  */
	OS_ERROR_FAIL_S = -EIO,	/*!< Generic driver failure */
	OS_ERROR_NO_MEMORY_S = -ENOMEM,	/*!< Failure to acquire/use memory  */
	OS_ERROR_BAD_ADDRESS_S = -EFAULT,	/*!< Bad address  */
	OS_ERROR_BAD_ARG_S = -EINVAL,	/*!< Bad input argument  */
} os_error_code;

/*!
 * Handle to a lock.
 */
#ifdef CONFIG_PREEMPT_RT
typedef raw_spinlock_t *os_lock_t;
#else
typedef spinlock_t *os_lock_t;
#endif

/*!
 * Context while locking.
 */
typedef unsigned long os_lock_context_t;

/*!
 * Declare a wait object for sleeping/waking processes.
 */
#define OS_WAIT_OBJECT(name)                                            \
    DECLARE_WAIT_QUEUE_HEAD(name##_qh)

/*!
 * Driver registration handle
 *
 * Used with #os_driver_init_registration(), #os_driver_add_registration(),
 * and #os_driver_complete_registration().
 */
typedef struct {
	unsigned reg_complete;	/*!< TRUE if next inits succeeded. */
	dev_t dev;		/*!< dev_t for register_chrdev() */
	struct file_operations fops;	/*!< struct for register_chrdev() */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
	struct class_simple *cs;	/*!< results of class_simple_create() */
#else
	struct class *cs;	/*!< results of class_create() */
#endif
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	struct class_device *cd;	/*!< Result of class_device_create() */
#else
	struct device *cd;	/*!< Result of device_create() */
#endif
	unsigned power_complete;	/*!< TRUE if next inits succeeded */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
	struct device_driver dd;	/*!< struct for register_driver() */
#else
	struct platform_driver dd;	/*!< struct for register_driver() */
#endif
	struct platform_device pd;	/*!< struct for platform_register_device() */
} os_driver_reg_t;

/*
 *  Function types which can be associated with driver entry points.
 *
 *  Note that init and shutdown are absent.
 */
/*! @{ */
/*! Keyword for registering open() operation handler. */
#define OS_FN_OPEN open
/*! Keyword for registering close() operation handler. */
#define OS_FN_CLOSE release
/*! Keyword for registering read() operation handler. */
#define OS_FN_READ read
/*! Keyword for registering write() operation handler. */
#define OS_FN_WRITE write
/*! Keyword for registering ioctl() operation handler. */
#define OS_FN_IOCTL ioctl
/*! Keyword for registering mmap() operation handler. */
#define OS_FN_MMAP mmap
/*! @} */

/*!
 * Function signature for the portable interrupt handler
 *
 * While it would be nice to know which interrupt is being serviced, the
 * Least Common Denominator rule says that no arguments get passed in.
 *
 * @return  Zero if not handled, non-zero if handled.
 */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
typedef int (*os_interrupt_handler_t) (int, void *, struct pt_regs *);
#else
typedef int (*os_interrupt_handler_t) (int, void *);
#endif

/*!
 * @defgroup dkops Driver-to-Kernel Operations
 *
 * These are the operations which drivers should call to get the OS to perform
 * services.
 */

/*! @addtogroup dkops */
/*! @{ */

/*!
 * Register an interrupt handler.
 *
 * @param driver_name    The name of the driver
 * @param interrupt_id   The interrupt line to monitor (type
 *                       #os_interrupt_id_t)
 * @param function       The function to be called to handle an interrupt
 *
 * @return #os_error_code
 */
#define os_register_interrupt(driver_name, interrupt_id, function)            \
     request_irq(interrupt_id, function, 0, driver_name, NULL)

/*!
 * Deregister an interrupt handler.
 *
 * @param interrupt_id   The interrupt line to stop monitoring
 *
 * @return #os_error_code
 */
#define os_deregister_interrupt(interrupt_id)                                 \
     free_irq(interrupt_id, NULL)

/*!
 * INTERNAL implementation of os_driver_init_registration()
 *
 * @return An os error code.
 */
inline static int os_drv_do_init_reg(os_driver_reg_t * handle)
{
	memset(handle, 0, sizeof(*handle));
	handle->fops.owner = THIS_MODULE;
	handle->power_complete = FALSE;
	handle->reg_complete = FALSE;
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
	handle->dd.name = NULL;
#else
	handle->dd.driver.name = NULL;
#endif

	return OS_ERROR_OK_S;
}

/*!
 * Initialize driver registration.
 *
 * If the driver handles open(), close(), ioctl(), read(), write(), or mmap()
 * calls, then it needs to register their location with the kernel so that they
 * get associated with the device.
 *
 * @param handle  The handle object to be used with this registration.  The
 *                object must live (be in memory somewhere) at least until
 *                os_driver_remove_registration() is called.
 *
 * @return A handle for further driver registration, or NULL if failed.
 */
#define os_driver_init_registration(handle)                                   \
    os_drv_do_init_reg(&handle)

/*!
 * Add a function registration to driver registration.
 *
 * @param handle    A handle initialized by #os_driver_init_registration().
 * @param name      Which function is being supported.
 * @param function  The result of a call to a @c _REF version of one of the
 *                  driver function signature macros
 * @return void
 */
#define os_driver_add_registration(handle, name, function)                    \
    do {handle.fops.name = (void*)(function); } while (0)

/*!
 * Record 'power suspend' function for the device.
 *
 * @param handle    A handle initialized by #os_driver_init_registration().
 * @param function  Name of function to call on power suspend request
 *
 * Status: Provisional
 *
 * @return void
 */
#define os_driver_register_power_suspend(handle, function)                    \
   handle.dd.suspend = function

/*!
 * Record 'power resume' function for the device.
 *
 * @param handle    A handle initialized by #os_driver_init_registration().
 * @param function  Name of function to call on power resume request
 *
 * Status: Provisional
 *
 * @return void
 */
#define os_driver_register_resume(handle, function)                          \
   handle.dd.resume = function

/*!
 * INTERNAL function of the Linux port of the OS API.   Implements the
 * os_driver_complete_registration() function.
 *
 * @param handle    The handle used with #os_driver_init_registration().
 * @param major      The major device number to be associated with the driver.
 *                   If this value is zero, a major number may be assigned.
 *                   See #os_driver_get_major() to determine final value.
 *                   #os_driver_remove_registration().
 * @param driver_name The driver name.  Can be used as part of 'device node'
 *                   name on platforms which support such a feature.
 *
 * @return  An error code
 */
inline static int os_drv_do_reg(os_driver_reg_t * handle,
				unsigned major, char *driver_name)
{
	os_error_code code = OS_ERROR_NO_MEMORY_S;
	char *name = kmalloc(strlen(driver_name) + 1, 0);

	if (name != NULL) {
		memcpy(name, driver_name, strlen(driver_name) + 1);
		code = OS_ERROR_OK_S;	/* OK so far */
		/* If any chardev/POSIX routines were added, then do chrdev part */
		if (handle->fops.open || handle->fops.release
		    || handle->fops.read || handle->fops.write
		    || handle->fops.ioctl || handle->fops.mmap) {

			printk("ioctl pointer: %p.  mmap pointer: %p\n",
			       handle->fops.ioctl, handle->fops.mmap);

			/* this method is depricated, see:
			 * http://lwn.net/Articles/126808/
			 */
			code =
			    register_chrdev(major, driver_name, &handle->fops);

			/* instead something like this: */
#if 0
			handle->dev = MKDEV(major, 0);
			code =
			    register_chrdev_region(handle->dev, 1, driver_name);
			if (code < 0) {
				code = OS_ERROR_FAIL_S;
			} else {
				cdev_init(&handle->cdev, &handle->fops);
				code = cdev_add(&handle->cdev, major, 1);
			}
#endif

			if (code < 0) {
				code = OS_ERROR_FAIL_S;
			} else {
				if (code != 0) {
					/* Zero was passed in for major; code is actual value */
					handle->dev = MKDEV(code, 0);
				} else {
					handle->dev = MKDEV(major, 0);
				}
				code = OS_ERROR_OK_S;
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
				handle->cs =
				    class_simple_create(THIS_MODULE,
							driver_name);
				if (IS_ERR(handle->cs)) {
					code = (os_error_code) handle->cs;
					handle->cs = NULL;
				} else {
					handle->cd =
					    class_simple_device_add(handle->cs,
								    handle->dev,
								    NULL,
								    driver_name);
					if (IS_ERR(handle->cd)) {
						class_simple_device_remove
						    (handle->dev);
						unregister_chrdev(MAJOR
								  (handle->dev),
								  driver_name);
						code =
						    (os_error_code) handle->cs;
						handle->cs = NULL;
					} else {
						handle->reg_complete = TRUE;
					}
				}
#else
				handle->cs =
				    class_create(THIS_MODULE, driver_name);
				if (IS_ERR(handle->cs)) {
					code = (os_error_code) handle->cs;
					handle->cs = NULL;
				} else {
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
					handle->cd =
					    class_device_create(handle->cs,
								NULL,
								handle->dev,
								NULL,
								driver_name);
#else
					handle->cd =
					    device_create(handle->cs, NULL,
							  handle->dev, NULL,
							  driver_name);
#endif
					if (IS_ERR(handle->cd)) {
						class_destroy(handle->cs);
						unregister_chrdev(MAJOR
								  (handle->dev),
								  driver_name);
						code =
						    (os_error_code) handle->cs;
						handle->cs = NULL;
					} else {
						handle->reg_complete = TRUE;
					}
				}
#endif
			}
		}
		/* ... fops routine registered */
		/* Handle power management fns through separate interface */
		if ((code == OS_ERROR_OK_S) &&
		    (handle->dd.suspend || handle->dd.resume)) {
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
			handle->dd.name = name;
			handle->dd.bus = &platform_bus_type;
			code = driver_register(&handle->dd);
#else
			handle->dd.driver.name = name;
			handle->dd.driver.bus = &platform_bus_type;
			code = driver_register(&handle->dd.driver);
#endif
			if (code == OS_ERROR_OK_S) {
				handle->pd.name = name;
				handle->pd.id = 0;
				code = platform_device_register(&handle->pd);
				if (code != OS_ERROR_OK_S) {
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
					driver_unregister(&handle->dd);
#else
					driver_unregister(&handle->dd.driver);
#endif
				} else {
					handle->power_complete = TRUE;
				}
			}
		}		/* ... suspend or resume */
	}			/* name != NULL */
	return code;
}

/*!
 * Finalize the driver registration with the kernel.
 *
 * Upon return from this call, the driver may begin receiving calls at the
 * defined entry points.
 *
 * @param handle    The handle used with #os_driver_init_registration().
 * @param major      The major device number to be associated with the driver.
 *                   If this value is zero, a major number may be assigned.
 *                   See #os_driver_get_major() to determine final value.
 *                   #os_driver_remove_registration().
 * @param driver_name The driver name.  Can be used as part of 'device node'
 *                   name on platforms which support such a feature.
 *
 * @return  An error code
 */
#define os_driver_complete_registration(handle, major, driver_name)           \
   os_drv_do_reg(&handle, major, driver_name)

/*!
 * Get driver Major Number from handle after a successful registration.
 *
 * @param   handle  A handle which has completed registration.
 *
 * @return The major number (if any) associated with the handle.
 */
#define os_driver_get_major(handle)                                           \
    (handle.reg_complete ? MAJOR(handle.dev) : -1)

/*!
 * INTERNAL implemention of os_driver_remove_registration.
 *
 * @param handle    A handle initialized by #os_driver_init_registration().
 *
 * @return  An error code.
 */
inline static int os_drv_rmv_reg(os_driver_reg_t * handle)
{
	if (handle->reg_complete) {
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
		if (handle->cd != NULL) {
			class_simple_device_remove(handle->dev);
			handle->cd = NULL;
		}
		if (handle->cs != NULL) {
			class_simple_destroy(handle->cs);
			handle->cs = NULL;
		}
		unregister_chrdev(MAJOR(handle->dev), handle->dd.name);
#else
		if (handle->cd != NULL) {
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
			class_device_destroy(handle->cs, handle->dev);
#else
			device_destroy(handle->cs, handle->dev);
#endif
			handle->cd = NULL;
		}
		if (handle->cs != NULL) {
			class_destroy(handle->cs);
			handle->cs = NULL;
		}
		unregister_chrdev(MAJOR(handle->dev), handle->dd.driver.name);
#endif
		handle->reg_complete = FALSE;
	}
	if (handle->power_complete) {
		platform_device_unregister(&handle->pd);
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
		driver_unregister(&handle->dd);
#else
		driver_unregister(&handle->dd.driver);
#endif
		handle->power_complete = FALSE;
	}
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
	if (handle->dd.name != NULL) {
		kfree(handle->dd.name);
		handle->dd.name = NULL;
	}
#else
	if (handle->dd.driver.name != NULL) {
		kfree(handle->dd.driver.name);
		handle->dd.driver.name = NULL;
	}
#endif
	return OS_ERROR_OK_S;
}

/*!
 * Remove the driver's registration with the kernel.
 *
 * Upon return from this call, the driver not receive any more calls at the
 * defined entry points (other than ISR and shutdown).
 *
 * @param handle    A handle initialized by #os_driver_init_registration().
 *
 * @return  An error code.
 */
#define os_driver_remove_registration(handle)                                 \
   os_drv_rmv_reg(&handle)

/*!
 * Register a driver with the Linux Device Model.
 *
 * @param   driver_information  The device_driver structure information
 *
 * @return  An error code.
 *
 * Status: denigrated in favor of #os_driver_complete_registration()
 */
#define os_register_to_driver(driver_information)                             \
              driver_register(driver_information)

/*!
 * Unregister a driver from the Linux Device Model
 *
 * this routine unregisters from the Linux Device Model
 *
 * @param   driver_information  The device_driver structure information
 *
 * @return  An error code.
 *
 * Status: Denigrated. See #os_register_to_driver().
 */
#define os_unregister_from_driver(driver_information)                         \
                driver_unregister(driver_information)

/*!
 * register a device to a driver
 *
 * this routine registers a drivers devices to the Linux Device Model
 *
 * @param   device_information  The platform_device structure information
 *
 * @return  An error code.
 *
 * Status: denigrated in favor of #os_driver_complete_registration()
 */
#define os_register_a_device(device_information)                              \
    platform_device_register(device_information)

/*!
 * unregister a device from a driver
 *
 * this routine unregisters a drivers devices from the Linux Device Model
 *
 * @param   device_information  The platform_device structure information
 *
 * @return  An error code.
 *
 * Status: Denigrated.  See #os_register_a_device().
 */
#define os_unregister_a_device(device_information)                            \
    platform_device_unregister(device_information)

/*!
 * Print a message to console / into log file.  After the @c msg argument a
 * number of printf-style arguments may be added.  Types should be limited to
 * printf string, char, octal, decimal, and hexadecimal types.  (This excludes
 * pointers, and floating point).
 *
 * @param  msg    The main text of the message to be logged
 * @param  s      The printf-style arguments which go with msg, if any
 *
 * @return (void)
 */
#define os_printk(...)                                                        \
    (void) printk(__VA_ARGS__)

/*!
 * Prepare a task to execute the given function.  This should only be done once
 * per function,, during the driver's initialization routine.
 *
 * @param task_fn   Name of the OS_DEV_TASK() function to be created.
 *
 * @return an OS ERROR code.
 */
#define os_create_task(function_name)                                         \
    OS_ERROR_OK_S

/*!
 * Schedule execution of a task.
 *
 * @param function_name   The function associated with the task.
 *
 * @return (void)
 */
#define os_dev_schedule_task(function_name)                                   \
    tasklet_schedule(&(function_name ## let))

/*!
 * Make sure that task is no longer running and will no longer run.
 *
 * This function will not return until both are true.  This is useful when
 * shutting down a driver.
 */
#define os_dev_stop_task(function_name)                                       \
do {                                                                          \
    tasklet_disable(&(function_name ## let));                                 \
    tasklet_kill(&(function_name ## let));                                    \
} while (0)

/*!
 * Allocate some kernel memory
 *
 * @param amount   Number of 8-bit bytes to allocate
 * @param flags    Some indication of purpose of memory (needs definition)
 *
 * @return  Pointer to allocated memory, or NULL if failed.
 */
#define os_alloc_memory(amount, flags)                                        \
    (void*)kmalloc(amount, flags)

/*!
 * Free some kernel memory
 *
 * @param location  The beginning of the region to be freed.
 *
 * Do some OSes have separate free() functions which should be
 * distinguished by passing in @c flags here, too? Don't some also require the
 * size of the buffer being freed?
 */
#define os_free_memory(location)                                              \
    kfree(location)

/*!
 * Allocate cache-coherent memory
 *
 * @param       amount     Number of bytes to allocate
 * @param[out]  dma_addrp  Location to store physical address of allocated
 *                         memory.
 * @param       flags      Some indication of purpose of memory (needs
 *                         definition).
 *
 * @return (virtual space) pointer to allocated memory, or NULL if failed.
 *
 */
#define os_alloc_coherent(amount, dma_addrp, flags)                           \
    (void*)dma_alloc_coherent(NULL, amount, dma_addrp, flags)

/*!
 * Free cache-coherent memory
 *
 * @param       size       Number of bytes which were allocated.
 * @param       virt_addr  Virtual(kernel) address of memory.to be freed, as
 *                         returned by #os_alloc_coherent().
 * @param       dma_addr   Physical address of memory.to be freed, as returned
 *                         by #os_alloc_coherent().
 *
 * @return void
 *
 */
#define os_free_coherent(size, virt_addr, dma_addr)                           \
    dma_free_coherent(NULL, size, virt_addr, dma_addr

/*!
 * Map an I/O space into kernel memory space
 *
 * @param start       The starting address of the (physical / io space) region
 * @param range_bytes The number of bytes to map
 *
 * @return A pointer to the mapped area, or NULL on failure
 */
#define os_map_device(start, range_bytes)                                     \
    (void*)ioremap_nocache((start), range_bytes)

/*!
 * Unmap an I/O space from kernel memory space
 *
 * @param start       The starting address of the (virtual) region
 * @param range_bytes The number of bytes to unmap
 *
 * @return None
 */
#define os_unmap_device(start, range_bytes)                                   \
    iounmap((void*)(start))

/*!
 * Copy data from Kernel space to User space
 *
 * @param to   The target location in user memory
 * @param from The source location in kernel memory
 * @param size The number of bytes to be copied
 *
 * @return #os_error_code
 */
#define os_copy_to_user(to, from, size)                                       \
    ((copy_to_user(to, from, size) == 0) ? 0 : OS_ERROR_BAD_ADDRESS_S)

/*!
 * Copy data from User space to Kernel space
 *
 * @param to   The target location in kernel memory
 * @param from The source location in user memory
 * @param size The number of bytes to be copied
 *
 * @return #os_error_code
 */
#define os_copy_from_user(to, from, size)                                     \
    ((copy_from_user(to, from, size) == 0) ? 0 : OS_ERROR_BAD_ADDRESS_S)

/*!
 * Read a 8-bit device register
 *
 * @param register_address  The (bus) address of the register to write to
 * @return                  The value in the register
 */
#define os_read8(register_address)                                            \
    __raw_readb(register_address)

/*!
 * Write a 8-bit device register
 *
 * @param register_address  The (bus) address of the register to write to
 * @param value             The value to write into the register
 */
#define os_write8(register_address, value)                                    \
    __raw_writeb(value, register_address)

/*!
 * Read a 16-bit device register
 *
 * @param register_address  The (bus) address of the register to write to
 * @return                  The value in the register
 */
#define os_read16(register_address)                                           \
    __raw_readw(register_address)

/*!
 * Write a 16-bit device register
 *
 * @param register_address  The (bus) address of the register to write to
 * @param value             The value to write into the register
 */
#define os_write16(register_address, value)                                   \
    __raw_writew(value, (uint32_t*)(register_address))

/*!
 * Read a 32-bit device register
 *
 * @param register_address  The (bus) address of the register to write to
 * @return                  The value in the register
 */
#define os_read32(register_address)                                           \
    __raw_readl((uint32_t*)(register_address))

/*!
 * Write a 32-bit device register
 *
 * @param register_address  The (bus) address of the register to write to
 * @param value             The value to write into the register
 */
#define os_write32(register_address, value)                                   \
    __raw_writel(value, register_address)

/*!
 * Read a 64-bit device register
 *
 * @param register_address  The (bus) address of the register to write to
 * @return                  The value in the register
 */
#define os_read64(register_address)                                           \
     ERROR_UNIMPLEMENTED

/*!
 * Write a 64-bit device register
 *
 * @param register_address  The (bus) address of the register to write to
 * @param value             The value to write into the register
 */
#define os_write64(register_address, value)                                   \
    ERROR_UNIMPLEMENTED

/*!
 * Delay some number of microseconds
 *
 * Note that this is a busy-loop, not a suspension of the task/process.
 *
 * @param  msecs   The number of microseconds to delay
 *
 * @return void
 */
#define os_mdelay mdelay

/*!
 * Calculate virtual address from physical address
 *
 * @param pa    Physical address
 *
 * @return virtual address
 *
 * @note this assumes that addresses are 32 bits wide
 */
#define os_va __va

/*!
 * Calculate physical address from virtual address
 *
 *
 * @param va    Virtual address
 *
 * @return physical address
 *
 * @note this assumes that addresses are 32 bits wide
 */
#define os_pa __pa

#ifdef CONFIG_PREEMPT_RT
/*!
 * Allocate and initialize a lock, returning a lock handle.
 *
 * The lock state will be initialized to 'unlocked'.
 *
 * @return A lock handle, or NULL if an error occurred.
 */
inline static os_lock_t os_lock_alloc_init(void)
{
	raw_spinlock_t *lockp;
	lockp = (raw_spinlock_t *) kmalloc(sizeof(raw_spinlock_t), 0);
	if (lockp) {
		_raw_spin_lock_init(lockp);
	} else {
		printk("OS: lock init failed\n");
	}

	return lockp;
}
#else
/*!
 * Allocate and initialize a lock, returning a lock handle.
 *
 * The lock state will be initialized to 'unlocked'.
 *
 * @return A lock handle, or NULL if an error occurred.
 */
inline static os_lock_t os_lock_alloc_init(void)
{
	spinlock_t *lockp;
	lockp = (spinlock_t *) kmalloc(sizeof(spinlock_t), 0);
	if (lockp) {
		spin_lock_init(lockp);
	} else {
		printk("OS: lock init failed\n");
	}

	return lockp;
}
#endif				/* CONFIG_PREEMPT_RT */

/*!
 * Acquire a lock.
 *
 * This function should only be called from an interrupt service routine.
 *
 * @param   lock_handle  A handle to the lock to acquire.
 *
 * @return void
 */
#define os_lock(lock_handle)                                              \
   spin_lock(lock_handle)

/*!
 * Unlock a lock.  Lock must have been acquired by #os_lock().
 *
 * @param   lock_handle  A handle to the lock to unlock.
 *
 * @return void
 */
#define os_unlock(lock_handle)                                            \
   spin_unlock(lock_handle)

/*!
 * Acquire a lock in non-ISR context
 *
 * This function will spin until the lock is available.
 *
 * @param lock_handle  A handle of the lock to acquire.
 * @param context      Place to save the before-lock context
 *
 * @return void
 */
#define os_lock_save_context(lock_handle, context)                        \
    spin_lock_irqsave(lock_handle, context)

/*!
 * Release a lock in non-ISR context
 *
 * @param lock_handle  A handle of the lock to release.
 * @param context      Place where before-lock context was saved.
 *
 * @return void
 */
#define os_unlock_restore_context(lock_handle, context)                  \
    spin_unlock_irqrestore(lock_handle, context)

/*!
 * Deallocate a lock handle.
 *
 * @param lock_handle   An #os_lock_t that has been allocated.
 *
 * @return void
 */
#define os_lock_deallocate(lock_handle)                                   \
    kfree(lock_handle)

/*!
 * Determine process handle
 *
 * The process handle of the current user is returned.
 *
 * @return   A handle on the current process.
 */
#define os_get_process_handle()                                           \
    current

/*!
 * Send a signal to a process
 *
 * @param  proc   A handle to the target process.
 * @param  sig    The POSIX signal to send to that process.
 */
#define os_send_signal(proc, sig)                                         \
    send_sig(sig, proc, 0);

/*!
 * Get some random bytes
 *
 * @param buf    The location to store the random data.
 * @param count  The number of bytes to store.
 *
 * @return  void
 */
#define os_get_random_bytes(buf, count)                                   \
    get_random_bytes(buf, count)

/*!
 * Go to sleep on an object.
 *
 * @param object    The object on which to sleep
 * @param condition An expression to check for sleep completion.  Must be
 *                  coded so that it can be referenced more than once inside
 *                  macro, i.e., no ++ or other modifying expressions.
 * @param atomic    Non-zero if sleep must not return until condition.
 *
 * @return error code -- OK or sleep interrupted??
 */
#define os_sleep(object, condition, atomic)                             \
({                                                                      \
    DEFINE_WAIT(_waitentry_);                                           \
    os_error_code code = OS_ERROR_OK_S;                                 \
                                                                        \
    while (!(condition)) {                                              \
        prepare_to_wait(&(object##_qh), &_waitentry_,                   \
                        atomic ? 0 : TASK_INTERRUPTIBLE);               \
        if (!(condition)) {                                             \
            schedule();                                                 \
        }                                                               \
                                                                        \
        finish_wait(&(object##_qh), &_waitentry_);                      \
                                                                        \
        if (!atomic && signal_pending(current)) {                       \
            code = OS_ERROR_FAIL_S; /* NEED SOMETHING BETTER */         \
            break;                                                      \
        }                                                               \
    };                                                                  \
                                                                        \
    code;                                                               \
})

/*!
 * Wake up whatever is sleeping on sleep object
 *
 * @param object  The object on which things might be sleeping
 *
 * @return none
 */
#define os_wake_sleepers(object)                                        \
    wake_up_interruptible(&(object##_qh));

	  /*! @} *//* dkops */

/******************************************************************************
 *  Function signature-generating macros
 *****************************************************************************/

/*!
 * @defgroup drsigs Driver Signatures
 *
 * These macros will define the entry point signatures for interrupt handlers;
 * driver initialization and shutdown; device open/close; etc.
 *
 * There are two versions of each macro for a given Driver Entry Point.  The
 * first version is used to define a function and its implementation in the
 * driver.c file, e.g. #OS_DEV_INIT().
 *
 * The second form is used whenever a forward declaration (prototype) is
 * needed.  It has the letters @c _DCL appended to the name of the defintion
 * function, and takes only the first two arguments (driver_name and
 * function_name).  These are not otherwise mentioned in this documenation.
 *
 * There is a third form used when a reference to a function is required, for
 * instance when passing the routine as a pointer to a function.  It has the
 * letters @c _REF appended to it, and takes only the first two arguments
 * (driver_name and function_name).  These functions are not otherwise
 * mentioned in this documentation.
 *
 * (Note that these two extra forms are required because of the
 * possibility/likelihood of having a 'wrapper function' which invokes the
 * generic function with expected arguments.  An alternative would be to have a
 * generic function which isn't able to get at any arguments directly, but
 * would be equipped with macros which could get at information passed in.
 *
 * Example:
 *
 * (in a header file)
 * @code
 * OS_DEV_INIT_DCL(widget, widget_init);
 * @endcode
 *
 * (in an implementation file)
 * @code
 * OS_DEV_INIT(widget, widget_init)
 * {
 *     os_dev_init_return(TRUE);
 * }
 * @endcode
 *
 */

/*! @addtogroup drsigs */
/*! @{ */

/*!
 * Define a function which will handle device initialization
 *
 * This is tne driver initialization routine.  This is normally where the
 * part would be initialized; queues, locks, interrupts handlers defined;
 * long-term dynamic memory allocated for driver use; etc.
 *
 * @param function_name   The name of the portable initialization function.
 *
 * @return  A call to #os_dev_init_return()
 *
 */
#define OS_DEV_INIT(function_name)                                            \
module_init(function_name);                                                   \
static int __init function_name (void)

/*! Make declaration for driver init function.
 * @param function_name foo
 */
#define OS_DEV_INIT_DCL(function_name)                                        \
static int __init function_name (void);

/*!
 * Generate a function reference to the driver's init function.
 * @param function_name   Name of the OS_DEV_INIT() function.
 *
 * @return A function pointer.
 */
#define OS_DEV_INIT_REF(function_name)                                        \
function_name

/*!
 * Define a function which will handle device shutdown
 *
 * This is the inverse of the #OS_DEV_INIT() routine.
 *
 * @param function_name   The name of the portable driver shutdown routine.
 *
 * @return  A call to #os_dev_shutdown_return()
 *
 */
#define OS_DEV_SHUTDOWN(function_name)                                        \
module_exit(function_name);                                                   \
static void function_name(void)

/*!
 * Generate a function reference to the driver's shutdown function.
 * @param function_name   Name of the OS_DEV_HUSTDOWN() function.
 *
 * @return A function pointer.
 */
#define OS_DEV_SHUTDOWN_DCL(function_name)                                    \
static void function_name(void);

/*!
 * Generate a reference to driver's shutdown function
 * @param function_name   Name of the OS_DEV_HUSTDOWN() function.
*/

#define OS_DEV_SHUTDOWN_REF(function_name)                                    \
function_name

/*!
 * Define a function which will open the device for a user.
 *
 * @param function_name The name of the driver open() function
 *
 * @return  A call to #os_dev_open_return()
 */
#define OS_DEV_OPEN(function_name)                                            \
static int function_name(struct inode* inode_p_, struct file* file_p_)

/*!
 * Declare prototype for an open() function.
 *
 * @param function_name The name of the OS_DEV_OPEN() function.
 */
#define OS_DEV_OPEN_DCL(function_name)                                        \
OS_DEV_OPEN(function_name);

/*!
 * Generate a function reference to the driver's open() function.
 * @param function_name   Name of the OS_DEV_OPEN() function.
 *
 * @return A function pointer.
 */
#define OS_DEV_OPEN_REF(function_name)                                        \
function_name

/*!
 * Define a function which will handle a user's ioctl() request
 *
 * @param function_name The name of the driver ioctl() function
 *
 * @return A call to #os_dev_ioctl_return()
 */
#define OS_DEV_IOCTL(function_name)                                           \
static int function_name(struct inode* inode_p_, struct file* file_p_,        \
                     unsigned int cmd_, unsigned long data_)

/*! Boo. */
#define OS_DEV_IOCTL_DCL(function_name)                                       \
OS_DEV_IOCTL(function_name);

/*!
 * Generate a function reference to the driver's ioctl() function.
 * @param function_name   Name of the OS_DEV_IOCTL() function.
 *
 * @return A function pointer.
 */
#define OS_DEV_IOCTL_REF(function_name)                                       \
function_name

/*!
 * Define a function which will handle a user's mmap() request
 *
 * @param function_name The name of the driver mmap() function
 *
 * @return A call to #os_dev_ioctl_return()
 */
#define OS_DEV_MMAP(function_name)                                            \
int function_name(struct file* file_p_, struct vm_area_struct* vma_)

#define OS_DEV_MMAP_DCL(function_name)                                        \
OS_DEV_MMAP(function_name);

#define OS_DEV_MMAP_REF(function_name)                                        \
function_name

/* Retrieve the context to the memory structure that is to be MMAPed */
#define os_mmap_memory_ctx() (vma_)

/* Determine the size of the requested MMAP region*/
#define os_mmap_memory_size() (vma_->vm_end - vma_->vm_start)

/* Determine the base address of the requested MMAP region*/
#define os_mmap_user_base() (vma_->vm_start)

/*!
 * Declare prototype for an read() function.
 *
 * @param function_name The name of the driver read function.
 */
#define OS_DEV_READ_DCL(function_name)                                        \
OS_DEV_READ(function_name);

/*!
 * Generate a function reference to the driver's read() routine
 * @param function_name   Name of the OS_DEV_READ() function.
 *
 * @return A function pointer.
 */
#define OS_DEV_READ_REF(function_name)                                        \
function_name

/*!
 * Define a function which will handle a user's write() request
 *
 * @param function_name The name of the driver write() function
 *
 * @return A call to #os_dev_write_return()
 */
#define OS_DEV_WRITE(function_name)                                           \
static ssize_t function_name(struct file* file_p_, char* user_buffer_,        \
                     size_t count_bytes_, loff_t* file_position_)

/*!
 * Declare prototype for an write() function.
 *
 * @param function_name The name of the driver write function.
 */
#define OS_DEV_WRITE_DCL(function_name)                                       \
OS_DEV_WRITE(function_name);

/*!
 * Generate a function reference to the driver's write() routine
 * @param function_name   Name of the OS_DEV_WRITE() function.
 *
 * @return A function pointer.
 */
#define OS_DEV_WRITE_REF(function_name)                                       \
function_name

/*!
 * Define a function which will close the device - opposite of OS_DEV_OPEN()
 *
 * @param function_name The name of the driver close() function
 *
 * @return A call to #os_dev_close_return()
 */
#define OS_DEV_CLOSE(function_name)                                           \
static int function_name(struct inode* inode_p_, struct file* file_p_)

/*!
 * Declare prototype for an close() function
 *
 * @param function_name The name of the driver close() function.
 */
#define OS_DEV_CLOSE_DCL(function_name)                                       \
OS_DEV_CLOSE(function_name);

/*!
 * Generate a function reference to the driver's close function.
 * @param function_name   Name of the OS_DEV_CLOSE() function.
 *
 * @return A function pointer.
 */
#define OS_DEV_CLOSE_REF(function_name)                                       \
function_name

/*!
 * Define a function which will handle an interrupt
 *
 * No arguments are available to the generic function.  It must not invoke any
 * OS functions which are illegal in a ISR.  It gets no parameters, and must
 * have a call to #os_dev_isr_return() instead of any/all return statements.
 *
 * Example:
 * @code
 * OS_DEV_ISR(widget)
 * {
 *     os_dev_isr_return(1);
 * }
 * @endcode
 *
 * @param function_name The name of the driver ISR function
 *
 * @return   A call to #os_dev_isr_return()
 */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#define OS_DEV_ISR(function_name)                                             \
static irqreturn_t function_name(int N1_, void* N2_, struct pt_regs* N3_)
#else
#define OS_DEV_ISR(function_name)                                             \
static irqreturn_t function_name(int N1_, void* N2_)
#endif

/*!
 * Declare prototype for an ISR function.
 *
 * @param function_name The name of the driver ISR function.
 */
#define OS_DEV_ISR_DCL(function_name)                                         \
OS_DEV_ISR(function_name);

/*!
 * Generate a function reference to the driver's interrupt service routine
 * @param function_name   Name of the OS_DEV_ISR() function.
 *
 * @return A function pointer.
 */
#define OS_DEV_ISR_REF(function_name)                                         \
function_name

/*!
 * Define a function which will operate as a background task / bottom half.
 *
 * Tasklet stuff isn't strictly limited to 'Device drivers', but leave it
 * this namespace anyway.
 *
 * @param function_name The name of this background task function
 *
 * @return A call to #os_dev_task_return()
 */
#define OS_DEV_TASK(function_name)                                            \
static void function_name(unsigned long data_)

/*!
 * Declare prototype for a background task / bottom half function
 *
 * @param function_name The name of this background task function
 */
#define OS_DEV_TASK_DCL(function_name)                                        \
OS_DEV_TASK(function_name);                                                   \
DECLARE_TASKLET(function_name ## let, function_name, 0);

/*!
 * Generate a reference to an #OS_DEV_TASK() function
 *
 * @param function_name   The name of the task being referenced.
 */
#define OS_DEV_TASK_REF(function_name)                                        \
    (function_name ## let)

	  /*! @} *//* drsigs */

/*****************************************************************************
 *  Functions/Macros for returning values from Driver Signature routines
 *****************************************************************************/

/*!
 * Return from the #OS_DEV_INIT() function
 *
 * @param code    An error code to report success or failure.
 *
 */
#define os_dev_init_return(code)                                             \
    return code

/*!
 * Return from the #OS_DEV_SHUTDOWN() function
 *
 * @param code    An error code to report success or failure.
 *
 */
#define os_dev_shutdown_return(code)                                         \
    return

/*!
 * Return from the #OS_DEV_ISR() function
 *
 * The function should verify that it really was supposed to be called,
 * and that its device needed attention, in order to properly set the
 * return code.
 *
 * @param code    non-zero if interrupt handled, zero otherwise.
 *
 */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#define os_dev_isr_return(code)                                              \
do {                                                                         \
    /* Unused warnings */                                                    \
    (void)N1_;                                                               \
    (void)N2_;                                                               \
    (void)N3_;                                                               \
                                                                             \
    return IRQ_RETVAL(code);                                                 \
} while (0)
#else
#define os_dev_isr_return(code)                                              \
do {                                                                         \
    /* Unused warnings */                                                    \
    (void)N1_;                                                               \
    (void)N2_;                                                               \
                                                                             \
    return IRQ_RETVAL(code);                                                 \
} while (0)
#endif

/*!
 * Return from the #OS_DEV_OPEN() function
 *
 * @param code    An error code to report success or failure.
 *
 */
#define os_dev_open_return(code)                                             \
do {                                                                         \
    int retcode = code;                                                      \
                                                                             \
    /* get rid of 'unused parameter' warnings */                             \
    (void)inode_p_;                                                          \
    (void)file_p_;                                                           \
                                                                             \
    return retcode;                                                          \
} while (0)

/*!
 * Return from the #OS_DEV_IOCTL() function
 *
 * @param code    An error code to report success or failure.
 *
 */
#define os_dev_ioctl_return(code)                                            \
do {                                                                         \
    int retcode = code;                                                      \
                                                                             \
    /* get rid of 'unused parameter' warnings */                             \
    (void)inode_p_;                                                          \
    (void)file_p_;                                                           \
    (void)cmd_;                                                              \
    (void)data_;                                                             \
                                                                             \
    return retcode;                                                          \
} while (0)

/*!
 * Return from the #OS_DEV_READ() function
 *
 * @param code    Number of bytes read, or an error code to report failure.
 *
 */
#define os_dev_read_return(code)                                             \
do {                                                                         \
    ssize_t retcode = code;                                                  \
                                                                             \
    /* get rid of 'unused parameter' warnings */                             \
    (void)file_p_;                                                           \
    (void)user_buffer_;                                                      \
    (void)count_bytes_;                                                      \
    (void)file_position_;                                                    \
                                                                             \
    return retcode;                                                          \
} while (0)

/*!
 * Return from the #OS_DEV_WRITE() function
 *
 * @param code    Number of bytes written, or an error code to report failure.
 *
 */
#define os_dev_write_return(code)                                            \
do {                                                                         \
    ssize_t retcode = code;                                                  \
                                                                             \
    /* get rid of 'unused parameter' warnings */                             \
    (void)file_p_;                                                           \
    (void)user_buffer_;                                                      \
    (void)count_bytes_;                                                      \
    (void)file_position_;                                                    \
                                                                             \
    return retcode;                                                          \
} while (0)

/*!
 * Return from the #OS_DEV_CLOSE() function
 *
 * @param code    An error code to report success or failure.
 *
 */
#define os_dev_close_return(code)                                            \
do {                                                                         \
    ssize_t retcode = code;                                                  \
                                                                             \
    /* get rid of 'unused parameter' warnings */                             \
    (void)inode_p_;                                                          \
    (void)file_p_;                                                           \
                                                                             \
    return retcode;                                                          \
} while (0)

/*!
 * Start the #OS_DEV_TASK() function
 *
 * In some implementations, this could be turned into a label for
 * the os_dev_task_return() call.
 *
 * @return none
 */
#define os_dev_task_begin()

/*!
 * Return from the #OS_DEV_TASK() function
 *
 * In some implementations, this could be turned into a sleep followed
 * by a jump back to the os_dev_task_begin() call.
 *
 * @param code    An error code to report success or failure.
 *
 */
#define os_dev_task_return(code)                                             \
do {                                                                         \
    /* Unused warnings */                                                    \
    (void)data_;                                                             \
                                                                             \
    return;                                                                  \
} while (0)

/*****************************************************************************
 *  Functions/Macros for accessing arguments from Driver Signature routines
 *****************************************************************************/

/*! @defgroup drsigargs Functions for Getting Arguments in Signature functions
 *
 */
/* @addtogroup @drsigargs */
/*! @{ */
/*!
 * Used in #OS_DEV_OPEN(), #OS_DEV_CLOSE(), #OS_DEV_IOCTL(), #OS_DEV_READ() and
 * #OS_DEV_WRITE() routines to check whether user is requesting read
 * (permission)
 */
#define os_dev_is_flag_read()                                                 \
   (file_p_->f_mode & FMODE_READ)

/*!
 * Used in #OS_DEV_OPEN(), #OS_DEV_CLOSE(), #OS_DEV_IOCTL(), #OS_DEV_READ() and
 * #OS_DEV_WRITE() routines to check whether user is requesting write
 * (permission)
 */
#define os_dev_is_flag_write()                                                \
   (file_p_->f_mode & FMODE_WRITE)

/*!
 * Used in #OS_DEV_OPEN(), #OS_DEV_CLOSE(), #OS_DEV_IOCTL(), #OS_DEV_READ() and
 * #OS_DEV_WRITE() routines to check whether user is requesting non-blocking
 * I/O.
 */
#define os_dev_is_flag_nonblock()                                             \
   (file_p_->f_flags & (O_NONBLOCK | O_NDELAY))

/*!
 * Used in #OS_DEV_OPEN() and #OS_DEV_CLOSE() to determine major device being
 * accessed.
 */
#define os_dev_get_major()                                                    \
    (imajor(inode_p_))

/*!
 * Used in #OS_DEV_OPEN() and #OS_DEV_CLOSE() to determine minor device being
 * accessed.
 */
#define os_dev_get_minor()                                                    \
    (iminor(inode_p_))

/*!
 * Used in #OS_DEV_IOCTL() to determine which operation the user wants
 * performed.
 *
 * @return Value of the operation.
 */
#define os_dev_get_ioctl_op()                                                 \
    (cmd_)

/*!
 * Used in #OS_DEV_IOCTL() to return the associated argument for the desired
 * operation.
 *
 * @return    A value which can be cast to a struct pointer or used as
 *            int/long.
 */
#define os_dev_get_ioctl_arg()                                                \
    (data_)

/*!
 * Used in OS_DEV_READ() and OS_DEV_WRITE() routines to access the requested
 * byte count.
 *
 * @return  (unsigned) a count of bytes
 */
#define os_dev_get_count()                                                    \
    ((unsigned)count_bytes_)

/*!
 * Used in OS_DEV_READ() and OS_DEV_WRITE() routines to return the pointer
 * byte count.
 *
 * @return   char* pointer to user buffer
 */
#define os_dev_get_user_buffer()                                              \
    ((void*)user_buffer_)

/*!
 * Used in OS_DEV_READ(), OS_DEV_WRITE(), and OS_DEV_IOCTL() routines to
 * get the POSIX flags field for the associated open file).
 *
 * @return The flags associated with the file.
 */
#define os_dev_get_file_flags()                                               \
    (file_p_->f_flags)

/*!
 * Set the driver's private structure associated with this file/open.
 *
 * Generally used during #OS_DEV_OPEN().  See #os_dev_get_user_private().
 *
 * @param  struct_p   The driver data structure to associate with this user.
 */
#define os_dev_set_user_private(struct_p)                                     \
    file_p_->private_data = (void*)(struct_p)

/*!
 * Get the driver's private structure associated with this file.
 *
 * May be used during #OS_DEV_OPEN(), #OS_DEV_READ(), #OS_DEV_WRITE(),
 * #OS_DEV_IOCTL(), and #OS_DEV_CLOSE().  See #os_dev_set_user_private().
 *
 * @return   The driver data structure to associate with this user.
 */
#define os_dev_get_user_private()                                             \
    ((void*)file_p_->private_data)

/*!
 * Get the IRQ associated with this call to the #OS_DEV_ISR() function.
 *
 * @return  The IRQ (integer) interrupt number.
 */
#define os_dev_get_irq()                                                      \
    N1_

	   /*! @} *//* drsigargs */

/*!
 * @defgroup cacheops Cache Operations
 *
 * These functions are for synchronizing processor cache with RAM.
 */
/*! @addtogroup cacheops */
/*! @{ */

/*!
 * Flush and invalidate all cache lines.
 */
#if 0
#define os_flush_cache_all()                                              \
    flush_cache_all()
#else
/* Call ARM fn directly, in case L2cache=on3 not set */
#define os_flush_cache_all()                                              \
    v6_flush_kern_cache_all_L2()

/*!
 * ARM-routine to flush all cache.  Defined here, because it exists in no
 * easy-access header file.  ARM-11 with L210 cache only!
 */
extern void v6_flush_kern_cache_all_L2(void);
#endif

/*
 *  These macros are using part of the Linux DMA API.  They rely on the
 *  map function to do nothing more than the equivalent clean/inv/flush
 *  operation at the time of the mapping, and do nothing at an unmapping
 *  call, which the Sahara driver code will never invoke.
 */

/*!
 * Clean a range of addresses from the cache.  That is, write updates back
 * to (RAM, next layer).
 *
 * @param  start    Starting virtual address
 * @param  len      Number of bytes to flush
 *
 * @return void
 */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#define os_cache_clean_range(start,len)                                   \
    dma_map_single(NULL, (void*)start, len, DMA_TO_DEVICE)
#else
#define os_cache_clean_range(start,len)                                   \
{                                                                         \
    void *s = (void*)start;                                               \
    void *e = s + len;                                                    \
    dmac_map_area(s, len, DMA_TO_DEVICE);                   \
    outer_clean_range(__pa(s), __pa(e));                                  \
}
#endif

/*!
 * Invalidate a range of addresses in the cache
 *
 * @param  start    Starting virtual address
 * @param  len      Number of bytes to flush
 *
 * @return void
 */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#define os_cache_inv_range(start,len)                                     \
    dma_map_single(NULL, (void*)start, len, DMA_FROM_DEVICE)
#else
#define os_cache_inv_range(start,len)                                     \
{                                                                         \
    void *s = (void*)start;                                               \
    void *e = s + len;                                                    \
    dmac_unmap_area(s, len, DMA_FROM_DEVICE);            \
    outer_inv_range(__pa(s), __pa(e));                                    \
}
#endif

/*!
 * Flush a range of addresses from the cache.  That is, perform clean
 * and invalidate
 *
 * @param  start    Starting virtual address
 * @param  len      Number of bytes to flush
 *
 * @return void
 */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#define os_cache_flush_range(start,len)                                   \
    dma_map_single(NULL, (void*)start, len, DMA_BIDIRECTIONAL)
#else
#define os_cache_flush_range(start,len)                                   \
{                                                                         \
    void *s = (void*)start;                                               \
    void *e = s + len;                                                    \
    dmac_flush_range(s, e);                                               \
    outer_flush_range(__pa(s), __pa(e));                                  \
}
#endif

	  /*! @} *//* cacheops */

#endif				/* LINUX_PORT_H */
