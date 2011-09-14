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

/*
* @file sah_kernel.h
*
* @brief Provides definitions for items that user-space and kernel-space share.
*/
/******************************************************************************
*
* This file needs to be PORTED to a non-Linux platform
*/

#ifndef SAH_KERNEL_H
#define SAH_KERNEL_H

#if defined(__KERNEL__)

#if defined(CONFIG_ARCH_MXC91321) || defined(CONFIG_ARCH_MXC91231)     \
    || defined(CONFIG_ARCH_MX27)  || defined(CONFIG_ARCH_MXC92323)
#include <mach/hardware.h>
#define SAHA_BASE_ADDR SAHARA_BASE_ADDR
#define SAHARA_IRQ     MXC_INT_SAHARA
#elif defined(CONFIG_ARCH_MX5)
#include <mach/hardware.h>
#else
#include <mach/mx2.h>
#endif

#endif				/* KERNEL */

/* IO Controls */
/* The magic number 'k' is reserved for the SPARC architecture. (See <kernel
 * source root>/Documentation/ioctl-number.txt.
 *
 * Note: Numbers 8-13 were used in a previous version of the API and should
 *       be avoided.
 */
#define SAH_IOC_MAGIC        'k'
#define SAHARA_HWRESET       _IO(SAH_IOC_MAGIC, 0)
#define SAHARA_SET_HA        _IO(SAH_IOC_MAGIC, 1)
#define SAHARA_CHK_TEST_MODE _IOR(SAH_IOC_MAGIC,2, int)
#define SAHARA_DAR           _IO(SAH_IOC_MAGIC, 3)
#define SAHARA_GET_RESULTS   _IO(SAH_IOC_MAGIC, 4)
#define SAHARA_REGISTER      _IO(SAH_IOC_MAGIC, 5)
#define SAHARA_DEREGISTER    _IO(SAH_IOC_MAGIC, 6)
/*                                              7 */
/*                                              8 */
/*                                              9 */
/*                                             10 */
/*                                             11 */
/*                                             12 */
/*                                             13 */

#define SAHARA_SCC_DROP_PERMS _IOWR(SAH_IOC_MAGIC, 14, scc_partition_info_t)
#define SAHARA_SCC_SFREE     _IOWR(SAH_IOC_MAGIC, 15, scc_partition_info_t)

#define SAHARA_SK_ALLOC     _IOWR(SAH_IOC_MAGIC, 16, scc_slot_t)
#define SAHARA_SK_DEALLOC   _IOWR(SAH_IOC_MAGIC, 17, scc_slot_t)
#define SAHARA_SK_LOAD      _IOWR(SAH_IOC_MAGIC, 18, scc_slot_t)
#define SAHARA_SK_UNLOAD    _IOWR(SAH_IOC_MAGIC, 19, scc_slot_t)
#define SAHARA_SK_SLOT_ENC  _IOWR(SAH_IOC_MAGIC, 20, scc_slot_t)
#define SAHARA_SK_SLOT_DEC  _IOWR(SAH_IOC_MAGIC, 21, scc_slot_t)

#define SAHARA_SCC_ENCRYPT  _IOWR(SAH_IOC_MAGIC, 22, scc_region_t)
#define SAHARA_SCC_DECRYPT  _IOWR(SAH_IOC_MAGIC, 23, scc_region_t)
#define SAHARA_GET_CAPS     _IOWR(SAH_IOC_MAGIC, 24, fsl_shw_pco_t)

#define SAHARA_SCC_SSTATUS  _IOWR(SAH_IOC_MAGIC, 25, scc_partition_info_t)

#define SAHARA_SK_READ      _IOWR(SAH_IOC_MAGIC, 29, scc_slot_t)

/*! This is the name that will appear in /proc/interrupts */
#define SAHARA_NAME         "SAHARA"

/*!
 * SAHARA IRQ number. See page 9-239 of TLICS - Motorola Semiconductors H.K.
 * TAHITI-Lite IC Specification, Rev 1.1, Feb 2003.
 *
 * TAHITI has two blocks of 32 interrupts. The SAHARA IRQ is number 27
 * in the second block. This means that the SAHARA IRQ is 27 + 32 = 59.
 */
#ifndef SAHARA_IRQ
#define SAHARA_IRQ          (27+32)
#endif

/*!
 * Device file definition. The #ifndef is here to support the unit test code
 * by allowing it to specify a different test device.
 */
#ifndef SAHARA_DEVICE_SHORT
#define SAHARA_DEVICE_SHORT "sahara"
#endif

#ifndef SAHARA_DEVICE
#define SAHARA_DEVICE       "/dev/"SAHARA_DEVICE_SHORT
#endif

#endif				/* SAH_KERNEL_H */

/* End of sah_kernel.h */
