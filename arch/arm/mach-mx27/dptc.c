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

/*!
 * @file dptc.c
 *
 * @brief DPTC table for the Freescale Semiconductor MXC DPTC module.
 *
 * @ingroup PM
 */

#include <mach/hardware.h>
#include <mach/mxc_dptc.h>

struct dptc_wp dptc_wp_allfreq[DPTC_WP_SUPPORTED] = {
	/* 532MHz */
	/* dcvr0      dcvr1       dcvr2       dcvr3     voltage */
	/* wp0 */
	{0xffe00000, 0x18e2e85b, 0xffe00000, 0x25c4688a, 1600},
	{0xffe00000, 0x18e2e85b, 0xffe00000, 0x25c4688a, 1575},
	{0xffe00000, 0x1902e85b, 0xffe00000, 0x25e4688a, 1550},
	{0xffe00000, 0x1922e85b, 0xffe00000, 0x25e4688a, 1525},
	{0xffe00000, 0x1942ec5b, 0xffe00000, 0x2604688a, 1500},
	/* wp5 */
	{0xffe00000, 0x1942ec5b, 0xffe00000, 0x26646c8a, 1475},
	{0xffe00000, 0x1962ec5b, 0xffe00000, 0x26c4708b, 1450},
	{0xffe00000, 0x1962ec5b, 0xffe00000, 0x26e4708b, 1425},
	{0xffe00000, 0x1982f05c, 0xffe00000, 0x2704748b, 1400},
	{0xffe00000, 0x19c2f05c, 0xffe00000, 0x2744748b, 1375},
	/* wp10 */
	{0xffe00000, 0x1a02f45c, 0xffe00000, 0x2784788b, 1350},
	{0xffe00000, 0x1a42f45c, 0xffe00000, 0x27c47c8b, 1325},
	{0xffe00000, 0x1a82f85c, 0xffe00000, 0x2824808c, 1300},
	{0xffe00000, 0x1aa2f85c, 0xffe00000, 0x2884848c, 1275},
	{0xffe00000, 0x1ac2fc5c, 0xffe00000, 0x28e4888c, 1250},
	/* wp15 */
	{0xffe00000, 0x1ae2fc5c, 0xffe00000, 0x2924888c, 1225},
	{0xffe00000, 0x1b23005d, 0xffe00000, 0x29648c8c, 1200},
};
