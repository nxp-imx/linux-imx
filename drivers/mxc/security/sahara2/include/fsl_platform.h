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
 * @file fsl_platform.h
 *
 * Header file to isolate code which might be platform-dependent
 */

#ifndef FSL_PLATFORM_H
#define FSL_PLATFORM_H

#ifdef __KERNEL__
#include "portable_os.h"
#endif

#if defined(FSL_PLATFORM_OTHER)

/* Have Makefile or other method of setting FSL_HAVE_* flags */

#elif defined(CONFIG_ARCH_MX3)	/* i.MX31 */

#define FSL_HAVE_SCC
#define FSL_HAVE_RTIC
#define FSL_HAVE_RNGA

#elif defined(CONFIG_ARCH_MX21)

#define FSL_HAVE_HAC
#define FSL_HAVE_RNGA
#define FSL_HAVE_SCC

#elif defined(CONFIG_ARCH_MX25)

#define FSL_HAVE_SCC
#define FSL_HAVE_RNGB
#define FSL_HAVE_RTIC3
#define FSL_HAVE_DRYICE

#elif defined(CONFIG_ARCH_MX27)

#define FSL_HAVE_SAHARA2
#define SUBMIT_MULTIPLE_DARS
#define FSL_HAVE_RTIC
#define FSL_HAVE_SCC
#define ALLOW_LLO_DESCRIPTORS

#elif defined(CONFIG_ARCH_MX35)

#define FSL_HAVE_SCC
#define FSL_HAVE_RNGC
#define FSL_HAVE_RTIC

#elif defined(CONFIG_ARCH_MX37)

#define FSL_HAVE_SCC2
#define FSL_HAVE_RNGC
#define FSL_HAVE_RTIC2
#define FSL_HAVE_SRTC

#elif defined(CONFIG_ARCH_MX5)

#define FSL_HAVE_SCC2
#define FSL_HAVE_SAHARA4
#define FSL_HAVE_RTIC3
#define FSL_HAVE_SRTC
#define NO_RESEED_WORKAROUND
#define NEED_CTR_WORKAROUND
#define USE_S2_CCM_ENCRYPT_CHAIN
#define USE_S2_CCM_DECRYPT_CHAIN
#define ALLOW_LLO_DESCRIPTORS

#elif defined(CONFIG_ARCH_MXC91131)

#define FSL_HAVE_SCC
#define FSL_HAVE_RNGC
#define FSL_HAVE_HAC

#elif defined(CONFIG_ARCH_MXC91221)

#define FSL_HAVE_SCC
#define FSL_HAVE_RNGC
#define FSL_HAVE_RTIC2

#elif defined(CONFIG_ARCH_MXC91231)

#define FSL_HAVE_SAHARA2
#define FSL_HAVE_RTIC
#define FSL_HAVE_SCC
#define NO_OUTPUT_1K_CROSSING

#elif defined(CONFIG_ARCH_MXC91311)

#define FSL_HAVE_SCC
#define FSL_HAVE_RNGC

#elif defined(CONFIG_ARCH_MXC91314)

#define FSL_HAVE_SCC
#define FSL_HAVE_SAHAR4
#define FSL_HAVE_RTIC3
#define NO_RESEED_WORKAROUND
#define NEED_CTR_WORKAROUND
#define USE_S2_CCM_ENCRYPT_CHAIN
#define USE_S2_CCM_DECRYPT_CHAIN
#define ALLOW_LLO_DESCRIPTORS

#elif defined(CONFIG_ARCH_MXC91321)

#define FSL_HAVE_SAHARA2
#define FSL_HAVE_RTIC
#define FSL_HAVE_SCC
#define SCC_CLOCK_NOT_GATED
#define NO_OUTPUT_1K_CROSSING

#elif defined(CONFIG_ARCH_MXC92323)

#define FSL_HAVE_SCC2
#define FSL_HAVE_SAHARA4
#define FSL_HAVE_PKHA
#define FSL_HAVE_RTIC2
#define NO_1K_CROSSING
#define NO_RESEED_WORKAROUND
#define NEED_CTR_WORKAROUND
#define USE_S2_CCM_ENCRYPT_CHAIN
#define USE_S2_CCM_DECRYPT_CHAIN
#define ALLOW_LLO_DESCRIPTORS


#elif  defined(CONFIG_ARCH_MXC91331)

#define FSL_HAVE_SCC
#define FSL_HAVE_RNGA
#define FSL_HAVE_HAC
#define FSL_HAVE_RTIC

#elif defined(CONFIG_8548)

#define FSL_HAVE_SEC2x

#elif defined(CONFIG_MPC8374)

#define FSL_HAVE_SEC3x

#else

#error UNKNOWN_PLATFORM

#endif				/* platform checks */

#endif				/* FSL_PLATFORM_H */
