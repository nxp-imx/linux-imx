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
 * @file dptc_gp.c
 *
 * @brief DPTC table for the Freescale Semiconductor MXC DPTC module.
 *
 * @ingroup PM
 */

#include <mach/hardware.h>
#include <mach/mxc_dptc.h>

struct dptc_wp dptc_gp_wp_allfreq[DPTC_GP_WP_SUPPORTED] = {
	/* 532MHz */
	/* dcvr0                  dcvr1                dcvr2
	   dcvr3            voltage */
	/* wp0 */
	{DCVR(107, 108, 112), DCVR(122, 123, 127), DCVR(133, 134, 139),
	 DCVR(115, 116, 121), 1000},
	{DCVR(107, 108, 113), DCVR(122, 123, 127), DCVR(133, 134, 139),
	 DCVR(115, 117, 122), 975},
	{DCVR(107, 109, 113), DCVR(122, 123, 127), DCVR(133, 134, 139),
	 DCVR(115, 117, 122), 950},
	{DCVR(107, 109, 114), DCVR(122, 123, 127), DCVR(133, 135, 140),
	 DCVR(115, 117, 122), 925},
	{DCVR(108, 109, 115), DCVR(122, 123, 127), DCVR(133, 136, 142),
	 DCVR(115, 117, 123), 900},
	{DCVR(108, 110, 115), DCVR(122, 123, 127), DCVR(133, 136, 142),
	 DCVR(115, 117, 123), 875},
	{DCVR(108, 110, 115), DCVR(122, 124, 128), DCVR(133, 136, 143),
	 DCVR(115, 118, 124), 850},
};

struct dptc_wp dptc_lp_wp_allfreq[DPTC_LP_WP_SUPPORTED] = {
	/* 532MHz */
	/* dcvr0                  dcvr1                dcvr2
	   dcvr3            regulator  voltage */
	/* wp0 */
	{DCVR(141, 143, 149), DCVR(155, 157, 162), DCVR(106, 108, 112),
	 DCVR(124, 126, 130), 1200},
	{DCVR(141, 143, 149), DCVR(155, 157, 162), DCVR(106, 108, 113),
	 DCVR(124, 126, 131), 1175},
	{DCVR(141, 144, 150), DCVR(155, 157, 163), DCVR(106, 108, 113),
	 DCVR(124, 126, 131), 1150},
	{DCVR(141, 144, 151), DCVR(155, 157, 163), DCVR(106, 108, 114),
	 DCVR(124, 126, 131), 1125},
	{DCVR(142, 144, 152), DCVR(155, 157, 163), DCVR(107, 109, 114),
	 DCVR(125, 127, 132), 1100},
	{DCVR(142, 145, 153), DCVR(155, 157, 164), DCVR(107, 109, 115),
	 DCVR(125, 127, 133), 1075},
	{DCVR(142, 145, 153), DCVR(155, 158, 164), DCVR(107, 109, 116),
	 DCVR(125, 127, 133), 1050},
	{DCVR(142, 145, 154), DCVR(155, 158, 165), DCVR(107, 110, 117),
	 DCVR(125, 127, 134), 1025},
	{DCVR(142, 146, 156), DCVR(155, 158, 165), DCVR(107, 110, 117),
	 DCVR(125, 128, 135), 1000},
};
