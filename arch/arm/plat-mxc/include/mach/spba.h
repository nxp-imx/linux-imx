
/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup SPBA Shared Peripheral Bus Arbiter (SPBA)
 * @ingroup MSL_MX31 MSL_MX35 MSL_MX37 MSL_MX51 MSL_MXC91321
 */

/*!
 * @file arch-mxc/spba.h
 * @brief This file contains the Shared Peripheral Bus Arbiter (spba) API.
 *
 * @ingroup SPBA
 */

#ifndef __ASM_ARCH_MXC_SPBA_H__
#define __ASM_ARCH_MXC_SPBA_H__

#ifdef __KERNEL__

#define MXC_SPBA_RAR_MASK       0x7

/*!
 * Defines three SPBA masters: A - ARM, C - SDMA (no master B for MX31)
 */
enum spba_masters {
	SPBA_MASTER_A = 1,
	SPBA_MASTER_B = 2,
	SPBA_MASTER_C = 4,
};

/*!
 * This function allows the three masters (A, B, C) to take ownership of a
 * shared peripheral.
 *
 * @param  mod          specified module as defined in \b enum \b #spba_module
 * @param  master       one of more (or-ed together) masters as defined in \b enum \b #spba_masters
 *
 * @return 0 if successful; -1 otherwise.
 */
int spba_take_ownership(int mod, int master);

/*!
 * This function releases the ownership for a shared peripheral.
 *
 * @param  mod          specified module as defined in \b enum \b #spba_module
 * @param  master       one of more (or-ed together) masters as defined in \b enum \b #spba_masters
 *
 * @return 0 if successful; -1 otherwise.
 */
int spba_rel_ownership(int mod, int master);

#endif				/* __KERNEL__ */

#endif				/* __ASM_ARCH_MXC_SPBA_H__ */
