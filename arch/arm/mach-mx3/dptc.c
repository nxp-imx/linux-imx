/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

struct dptc_wp dptc_wp_allfreq_26ckih[DPTC_WP_SUPPORTED] = {
	/* 532MHz */
	/* dcvr0      dcvr1       dcvr2       dcvr3     voltage */
	/* wp0 */
	{0xffc00000, 0x95c00000, 0xffc00000, 0xe5800000, 1625},
	{0xffc00000, 0x95e3e8e4, 0xffc00000, 0xe5b6fda0, 1600},
	{0xffc00000, 0x95e3e8e4, 0xffc00000, 0xe5b6fda0, 1575},
	{0xffc00000, 0x95e3e8e8, 0xffc00000, 0xe5f70da4, 1550},
	{0xffc00000, 0x9623f8e8, 0xffc00000, 0xe6371da8, 1525},
	/* wp5 */
	{0xffc00000, 0x966408f0, 0xffc00000, 0xe6b73db0, 1500},
	{0xffc00000, 0x96e428f4, 0xffc00000, 0xe7776dbc, 1475},
	{0xffc00000, 0x976448fc, 0xffc00000, 0xe8379dc8, 1450},
	{0xffc00000, 0x97e46904, 0xffc00000, 0xe977ddd8, 1425},
	{0xffc00000, 0x98a48910, 0xffc00000, 0xeab81de8, 1400},
	/* wp10 */
	{0xffc00000, 0x9964b918, 0xffc00000, 0xebf86df8, 1375},
	{0xffc00000, 0xffe4e924, 0xffc00000, 0xfff8ae08, 1350},
	{0xffc00000, 0xffe5192c, 0xffc00000, 0xfff8fe1c, 1350},
	{0xffc00000, 0xffe54938, 0xffc00000, 0xfff95e2c, 1350},
	{0xffc00000, 0xffe57944, 0xffc00000, 0xfff9ae44, 1350},
	/* wp15 */
	{0xffc00000, 0xffe5b954, 0xffc00000, 0xfffa0e58, 1350},
	{0xffc00000, 0xffe5e960, 0xffc00000, 0xfffa6e70, 1350},
};

struct dptc_wp dptc_wp_allfreq_26ckih_TO_2_0[DPTC_WP_SUPPORTED] = {
	/* Mx31 TO 2.0  Offset table */
	/* 532MHz  */
	/* dcvr0      dcvr1       dcvr2       dcvr3     voltage */
	/* wp0 */
	{0xffc00000, 0x9E265978, 0xffc00000, 0xE4371D9C, 1625},
	{0xffc00000, 0x9E665978, 0xffc00000, 0xE4772D9C, 1600},
	{0xffc00000, 0x9EA65978, 0xffc00000, 0xE4772DA0, 1575},
	{0xffc00000, 0x9EE66978, 0xffc00000, 0xE4B73DA0, 1550},
	{0xffc00000, 0x9F26697C, 0xffc00000, 0xE4F73DA0, 1525},
	/* wp5 */
	{0xffc00000, 0x9F66797C, 0xffc00000, 0xE5774DA4, 1500},
	{0xffc00000, 0x9FE6797C, 0xffc00000, 0xE5F75DA4, 1475},
	{0xffc00000, 0xA026897C, 0xffc00000, 0xE6776DA4, 1450},
	{0xffc00000, 0xA0A6897C, 0xffc00000, 0xE6F77DA8, 1425},
	{0xffc00000, 0xA0E69980, 0xffc00000, 0xE7B78DAC, 1400},
	/* wp10 */
	{0xffc00000, 0xA1669980, 0xffc00000, 0xE8379DAC, 1375},
	{0xffc00000, 0xA1A6A980, 0xffc00000, 0xE8F7ADB0, 1350},
	{0xffc00000, 0xA226B984, 0xffc00000, 0xE9F7CDB0, 1325},
	{0xffc00000, 0xA2A6C984, 0xffc00000, 0xEAB7DDB4, 1300},
	{0xffc00000, 0xA326C988, 0xffc00000, 0xEBB7FDB8, 1275},
	/* wp15 */
	{0xffc00000, 0xA3A6D988, 0xffc00000, 0xECB80DBC, 1250},
	{0xffc00000, 0xA426E988, 0xffc00000, 0xEDB82DC0, 1225},
};

struct dptc_wp dptc_wp_allfreq_27ckih_TO_2_0[DPTC_WP_SUPPORTED] = {
	/* Mx31 TO 2.0  Offset table */
	/* 532MHz  */
	/* dcvr0      dcvr1       dcvr2       dcvr3     voltage */
	/* wp0 */
	{0xffc00000, 0x9864E920, 0xffc00000, 0xDBB50D1C, 1625},
	{0xffc00000, 0x98A4E920, 0xffc00000, 0xDBF51D1C, 1600},
	{0xffc00000, 0x98E4E920, 0xffc00000, 0xDBF51D20, 1575},
	{0xffc00000, 0x9924F920, 0xffc00000, 0xDC352D20, 1550},
	{0xffc00000, 0x9924F924, 0xffc00000, 0xDC752D20, 1525},
	/* wp5 */
	{0xffc00000, 0x99650924, 0xffc00000, 0xDCF53D24, 1500},
	{0xffc00000, 0x99E50924, 0xffc00000, 0xDD754D24, 1475},
	{0xffc00000, 0x9A251924, 0xffc00000, 0xDDF55D24, 1450},
	{0xffc00000, 0x9AA51924, 0xffc00000, 0xDE756D28, 1425},
	{0xffc00000, 0x9AE52928, 0xffc00000, 0xDF357D2C, 1400},
	/* wp10 */
	{0xffc00000, 0x9B652928, 0xffc00000, 0xDFB58D2C, 1375},
	{0xffc00000, 0x9BA53928, 0xffc00000, 0xE0759D30, 1350},
	{0xffc00000, 0x9C254928, 0xffc00000, 0xE135BD30, 1325},
	{0xffc00000, 0x9CA55928, 0xffc00000, 0xE1F5CD34, 1300},
	{0xffc00000, 0x9D25592C, 0xffc00000, 0xE2F5ED38, 1275},
	/* wp15 */
	{0xffc00000, 0x9DA5692C, 0xffc00000, 0xE3F5FD38, 1250},
	{0xffc00000, 0x9E25792C, 0xffc00000, 0xE4F61D3C, 1225},
};
