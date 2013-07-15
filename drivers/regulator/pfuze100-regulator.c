/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/mfd/pfuze.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/err.h>
#include "pfuze-regulator.h"

static const int pfuze100_sw1[] = {
#ifdef PFUZE100_FIRST_VERSION
	650000, 662500, 675000, 687500, 700000, 712500, 725000, 737500,
	750000, 762500, 775000, 787500, 800000, 812500, 825000, 837500,
	850000, 862500, 875000, 887500, 900000, 912500, 925000, 937500,
	950000, 962500, 975000, 987500, 1000000, 1012500, 1025000, 1037500,
	1050000, 1062500, 1075000, 1087500, 1100000, 1112500, 1125000, 1137500,
	1150000, 1162500, 1175000, 1187500, 1200000, 1212500, 1225000, 1237500,
	1250000, 1262500, 1275000, 1287500, 1300000, 1312500, 1325000, 1337500,
	1350000, 1362500, 1375000, 1387500, 1400000, 1412500, 1425000, 1437500,
#else
	300000, 325000, 350000, 375000, 400000, 425000, 450000, 475000,
	500000, 525000, 550000, 575000, 600000, 625000, 650000, 675000,
	700000, 725000, 750000, 775000, 800000, 825000, 850000, 875000,
	900000, 925000, 950000, 975000, 1000000, 1025000, 1050000, 1075000,
	1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 1250000, 1275000,
	1300000, 1325000, 1350000, 1375000, 1400000, 1425000, 1450000, 1475000,
	1500000, 1525000, 1550000, 1575000, 1600000, 1625000, 1650000, 1675000,
	1700000, 1725000, 1750000, 1775000, 1800000, 1825000, 1850000, 1875000,
#endif
};

#if  PFUZE100_SW2_VOL6
static const int pfuze100_sw2[] = {
	800000, 850000, 900000, 950000, 1000000, 1050000, 1100000, 1150000,
	1200000, 1250000, 1300000, 1350000, 1400000, 1450000, 1500000, 1550000,
	1600000, 1650000, 1700000, 1750000, 1800000, 1850000, 1900000, 1950000,
	2000000, 2050000, 2100000, 2150000, 2200000, 2250000, 2300000, 2350000,
	2400000, 2450000, 2500000, 2550000, 2600000, 2650000, 2700000, 2750000,
	2800000, 2850000, 2900000, 2950000, 3000000, 3050000, 3100000, 3150000,
	3200000, 3250000, 3300000, 3300000, 3300000, 3300000, 3300000, 3300000,
	3300000, 3300000, 3300000, 3300000, 3300000, 3300000, 3300000, 3950000,
};
#else
static const int pfuze100_sw2[] = {
	400000, 425000, 450000, 475000, 500000, 525000, 550000, 575000,
	600000, 625000, 650000, 675000, 700000, 725000, 750000, 775000,
	800000, 825000, 850000, 875000, 900000, 925000, 950000, 975000,
	1000000, 1025000, 1050000, 1075000, 1100000, 1125000, 1150000, 1175000,
	1200000, 1225000, 1250000, 1275000, 1300000, 1325000, 1350000, 1375000,
	1400000, 1425000, 1450000, 1475000, 1500000, 1525000, 1550000, 1575000,
	1600000, 1625000, 1650000, 1675000, 1700000, 1725000, 1750000, 1775000,
	1800000, 1825000, 1850000, 1875000, 1900000, 1925000, 1950000, 1975000,
};
#endif

#if PFUZE100_SW3_VOL6
static const int pfuze100_sw3[] = {
	800000, 850000, 900000, 950000, 1000000, 1050000, 1100000, 1150000,
	1200000, 1250000, 1300000, 1350000, 1400000, 1450000, 1500000, 1550000,
	1600000, 1650000, 1700000, 1750000, 1800000, 1850000, 1900000, 1950000,
	2000000, 2050000, 2100000, 2150000, 2200000, 2250000, 2300000, 2350000,
	2400000, 2450000, 2500000, 2550000, 2600000, 2650000, 2700000, 2750000,
	2800000, 2850000, 2900000, 2950000, 3000000, 3050000, 3100000, 3150000,
	3200000, 3250000, 3300000, 3300000, 3300000, 3300000, 3300000, 3300000,
	3300000, 3300000, 3300000, 3750000, 3800000, 3850000, 3900000, 3950000,
};
#else
static const int pfuze100_sw3[] = {
	400000, 425000, 450000, 475000, 500000, 525000, 550000, 575000,
	600000, 625000, 650000, 675000, 700000, 725000, 750000, 775000,
	800000, 825000, 850000, 875000, 900000, 925000, 950000, 975000,
	1000000, 1025000, 1050000, 1075000, 1100000, 1125000, 1150000, 1175000,
	1200000, 1225000, 1250000, 1275000, 1300000, 1325000, 1350000, 1375000,
	1400000, 1425000, 1450000, 1475000, 1500000, 1525000, 1550000, 1575000,
	1600000, 1625000, 1650000, 1675000, 1700000, 1725000, 1750000, 1775000,
	1800000, 1825000, 1850000, 1875000, 1900000, 1925000, 1950000, 1975000,
};
#endif

#if PFUZE100_SW4_VOL6
static const int pfuze100_sw4[] = {
	800000, 850000, 900000, 950000, 1000000, 1050000, 1100000, 1150000,
	1200000, 1250000, 1300000, 1350000, 1400000, 1450000, 1500000, 1550000,
	1600000, 1650000, 1700000, 1750000, 1800000, 1850000, 1900000, 1950000,
	2000000, 2050000, 2100000, 2150000, 2200000, 2250000, 2300000, 2350000,
	2400000, 2450000, 2500000, 2550000, 2600000, 2650000, 2700000, 2750000,
	2800000, 2850000, 2900000, 2950000, 3000000, 3050000, 3100000, 3150000,
	3200000, 3250000, 3300000, 3300000, 3300000, 3300000, 3300000, 3300000,
	3300000, 3300000, 3300000, 3300000, 3300000, 3300000, 3300000, 3950000,
};

#else
static const int pfuze100_sw4[] = {
	400000, 425000, 450000, 475000, 500000, 525000, 550000, 575000,
	600000, 625000, 650000, 675000, 700000, 725000, 750000, 775000,
	800000, 825000, 850000, 875000, 900000, 925000, 950000, 975000,
	1000000, 1025000, 1050000, 1075000, 1100000, 1125000, 1150000, 1175000,
	1200000, 1225000, 1250000, 1275000, 1300000, 1325000, 1350000, 1375000,
	1400000, 1425000, 1450000, 1475000, 1500000, 1525000, 1550000, 1575000,
	1600000, 1625000, 1650000, 1675000, 1700000, 1725000, 1750000, 1775000,
	1800000, 1825000, 1850000, 1875000, 1900000, 1925000, 1950000, 1975000,
};
#endif

static const int pfuze100_swbst[] = {
	5000000, 5050000, 5100000, 5150000,
};

static const int pfuze100_vsnvs[] = {
	1200000, 1500000, 1800000, 3000000,
};

static const int pfuze100_vrefddr[] = {
	750000,
};

static const int pfuze100_vgen12[] = {

#ifdef PFUZE100_FIRST_VERSION
	1200000, 1250000, 1300000, 1350000, 1400000, 1450000, 1500000, 1550000,
#else
	800000, 850000, 900000, 950000, 1000000, 1050000, 1100000, 1150000,
	1200000, 1250000, 1300000, 1350000, 1400000, 1450000, 1500000, 1550000,
#endif
};

static const int pfuze100_vgen36[] = {
	1800000, 1900000, 2000000, 2100000, 2200000, 2300000, 2400000, 2500000,
	2600000, 2700000, 2800000, 2900000, 3000000, 3100000, 3200000, 3300000,
};

static struct regulator_ops pfuze100_ldo_regulator_ops;
static struct regulator_ops pfuze100_fixed_regulator_ops;
static struct regulator_ops pfuze100_sw_regulator_ops;

#define PFUZE100_FIXED_VOL_DEFINE(name,	reg, voltages)		\
	PFUZE_FIXED_DEFINE(PFUZE100_, name, reg, voltages,	\
			pfuze100_fixed_regulator_ops)

#define PFUZE100_SW_DEFINE(name, reg, voltages)	\
	PFUZE_SW_DEFINE(PFUZE100_, name, reg, voltages,	\
			pfuze100_sw_regulator_ops)

#define PFUZE100_SWBST_DEFINE(name, reg, voltages)	\
	PFUZE_SWBST_DEFINE(PFUZE100_, name, reg, voltages,	\
			pfuze100_sw_regulator_ops)

#define PFUZE100_VGEN_DEFINE(name, reg, voltages)	\
	PFUZE_DEFINE(PFUZE100_, name, reg, voltages,	\
			pfuze100_ldo_regulator_ops)
/* SW1A */
#define PFUZE100_SW1AVOL	32
#define PFUZE100_SW1AVOL_VSEL	0
#define PFUZE100_SW1AVOL_VSEL_M	(0x3f<<0)

#define PFUZE100_SW1AVOL_STBY	33
#define PFUZE100_SW1AVOL_STBY_VSEL	0
#define PFUZE100_SW1AVOL_STBY_VSEL_M	(0x3f<<0)

#define PFUZE100_SW1AOFF	34
#define PFUZE100_SW1AOFF_OFF_VAL	(0x0<<0)
#define PFUZE100_SW1AOFF_OFF_M	(0x3f<<0)

#define PFUZE100_SW1AMODE	35
#define PFUZE100_SW1AMODE_OMODE_VAL	(0x0<<5)
#define PFUZE100_SW1AMODE_OMODE_M	(0x1<<5)
#define PFUZE100_SW1AMODE_MODE_VAL	0x7	/*Auto */
#define PFUZE100_SW1AMODE_MODE_M	(0xf<<0)

#define PFUZE100_SW1ACON		36
#define PFUZE100_SW1ACON_SPEED_VAL	(0x1<<6)	/*default */
#define PFUZE100_SW1ACON_SPEED_M	(0x3<<6)
#define PFUZE100_SW1ACON_PHASE_VAL	(0x1<<4)	/*default */
#define PFUZE100_SW1ACON_PHASE_M	(0x3<<4)
#define PFUZE100_SW1ACON_FREQ_VAL	(0x1<<2)	/*1Mhz */
#define PFUZE100_SW1ACON_FREQ_M	(0x3<<2)
#define PFUZE100_SW1ACON_LIM_VAL	(0x0<<0)	/*2Imax */
#define PFUZE100_SW1ACON_LIM_M	(0x3<<0)

/*SW1B*/
#define PFUZE100_SW1BVOL	39
#define PFUZE100_SW1BVOL_VSEL	0
#define PFUZE100_SW1BVOL_VSEL_M	(0x3f<<0)

#define PFUZE100_SW1BVOL_STBY	40
#define PFUZE100_SW1BVOL_STBY_VSEL	0
#define PFUZE100_SW1BVOL_STBY_VSEL_M	(0x3f<<0)

#define PFUZE100_SW1BOFF	41
#define PFUZE100_SW1BOFF_OFF_VAL	0x0
#define PFUZE100_SW1BOFF_OFF_M	(0x3f<<0)

#define PFUZE100_SW1BMODE	42
#define PFUZE100_SW1BMODE_OMODE_VAL	(0x0<<5)
#define PFUZE100_SW1BMODE_OMODE_M	(0x1<<5)
#define PFUZE100_SW1BMODE_MODE_VAL	(0x7<<0)
#define PFUZE100_SW1BMODE_MODE_M	(0xf<<0)

#define PFUZE100_SW1BCON		43
#define PFUZE100_SW1BCON_SPEED_VAL	(0x1<<6)
#define PFUZE100_SW1BCON_SPEED_M	(0x3<<6)
#define PFUZE100_SW1BCON_PHASE_VAL	(0x1<<4)
#define PFUZE100_SW1BCON_PHASE_M	(0x3<<4)
#define PFUZE100_SW1BCON_FREQ_VAL	(0x1<<2)
#define PFUZE100_SW1BCON_FREQ_M	(0x3<<2)
#define PFUZE100_SW1BCON_LIM_VAL	(0x0<<0)
#define PFUZE100_SW1BCON_LIM_M	(0x3<<0)

/*SW1C*/
#define PFUZE100_SW1CVOL	46
#define PFUZE100_SW1CVOL_VSEL	0
#define PFUZE100_SW1CVOL_VSEL_M	(0x3f<<0)

#define PFUZE100_SW1CVOL_STBY	47
#define PFUZE100_SW1CVOL_STBY_VSEL	0
#define PFUZE100_SW1CVOL_STBY_VSEL_M	(0x3f<<0)

#define PFUZE100_SW1COFF	48
#define PFUZE100_SW1COFF_OFF_VAL	0x0
#define PFUZE100_SW1COFF_OFF_M	(0x3f<<0)

#define PFUZE100_SW1CMODE	49
#define PFUZE100_SW1CMODE_OMODE_VAL	(0x0<<5)
#define PFUZE100_SW1CMODE_OMODE_M	(0x1<<5)
#define PFUZE100_SW1CMODE_MODE_VAL	(0x7<<0)
#define PFUZE100_SW1CMODE_MODE_M	(0xf<<0)

#define PFUZE100_SW1CCON		50
#define PFUZE100_SW1CCON_SPEED_VAL	(0x1<<6)
#define PFUZE100_SW1CCON_SPEED_M	(0x3<<6)
#define PFUZE100_SW1CCON_PHASE_VAL	(0x1<<4)
#define PFUZE100_SW1CCON_PHASE_M	(0x3<<4)
#define PFUZE100_SW1CCON_FREQ_VAL	(0x1<<2)
#define PFUZE100_SW1CCON_FREQ_M		(0x3<<2)
#define PFUZE100_SW1CCON_LIM_VAL	(0x0<<0)
#define PFUZE100_SW1CCON_LIM_M		(0x3<<0)

/*SW2*/
#define PFUZE100_SW2VOL		53
#define PFUZE100_SW2VOL_VSEL	0
#define PFUZE100_SW2VOL_VSEL_M	(0x3f<<0)

#define PFUZE100_SW2VOL_STBY		54
#define PFUZE100_SW2VOL_STBY_VSEL	0
#define PFUZE100_SW2VOL_STBY_VSEL_M	(0x3f<<0)

#define PFUZE100_SW2OFF	55
#define PFUZE100_SW2OFF_OFF_VAL	0x0
#define PFUZE100_SW2OFF_OFF_M	(0x7f<<0)

#define PFUZE100_SW2MODE	56
#define PFUZE100_SW2MODE_OMODE_VAL	(0x0<<5)
#define PFUZE100_SW2MODE_OMODE_M	(0x1<<5)
#define PFUZE100_SW2MODE_MODE_VAL	(0x7<<0)
#define PFUZE100_SW2MODE_MODE_M		(0xf<<0)

#define PFUZE100_SW2CON		57
#define PFUZE100_SW2CON_SPEED_VAL	(0x1<<6)
#define PFUZE100_SW2CON_SPEED_M		(0x3<<6)
#define PFUZE100_SW2CON_PHASE_VAL	(0x1<<4)
#define PFUZE100_SW2CON_PHASE_M		(0x3<<4)
#define PFUZE100_SW2CON_FREQ_VAL	(0x1<<2)
#define PFUZE100_SW2CON_FREQ_M		(0x3<<2)
#define PFUZE100_SW2CON_LIM_VAL		(0x0<<0)
#define PFUZE100_SW2CON_LIM_M		(0x3<<0)

/*SW3A*/
#define PFUZE100_SW3AVOL	60
#define PFUZE100_SW3AVOL_VSEL	0
#define PFUZE100_SW3AVOL_VSEL_M	(0x3f<<0)

#define PFUZE100_SW3AVOL_STBY	61
#define PFUZE100_SW3AVOL_STBY_VSEL	0
#define PFUZE100_SW3AVOL_STBY_VSEL_M	(0x3f<<0)

#define PFUZE100_SW3AOFF	62
#define PFUZE100_SW3AOFF_OFF_VAL	0x0
#define PFUZE100_SW3AOFF_OFF_M	(0x3f<<0)

#define PFUZE100_SW3AMODE	63
#define PFUZE100_SW3AMODE_OMODE_VAL	(0x0<<5)
#define PFUZE100_SW3AMODE_OMODE_M	(0x1<<5)
#define PFUZE100_SW3AMODE_MODE_VAL	(0x7<<0)
#define PFUZE100_SW3AMODE_MODE_M	(0xf<<0)

#define PFUZE100_SW3ACON		64
#define PFUZE100_SW3ACON_SPEED_VAL	(0x1<<6)
#define PFUZE100_SW3ACON_SPEED_M	(0x3<<6)
#define PFUZE100_SW3ACON_PHASE_VAL	(0x1<<4)
#define PFUZE100_SW3ACON_PHASE_M	(0x3<<4)
#define PFUZE100_SW3ACON_FREQ_VAL	(0x1<<2)
#define PFUZE100_SW3ACON_FREQ_M		(0x3<<2)
#define PFUZE100_SW3ACON_LIM_VAL	(0x0<<0)
#define PFUZE100_SW3ACON_LIM_M		(0x3<<0)

/*SW3B*/
#define PFUZE100_SW3BVOL	67
#define PFUZE100_SW3BVOL_VSEL	0
#define PFUZE100_SW3BVOL_VSEL_M	(0x3f<<0)

#define PFUZE100_SW3BVOL_STBY	68
#define PFUZE100_SW3BVOL_STBY_VSEL	0
#define PFUZE100_SW3BVOL_STBY_VSEL_M	(0x3f<<0)

#define PFUZE100_SW3BOFF	69
#define PFUZE100_SW3BOFF_OFF_VAL	0x0
#define PFUZE100_SW3BOFF_OFF_M	(0x3f<<0)

#define PFUZE100_SW3BMODE	70
#define PFUZE100_SW3BMODE_OMODE_VAL	(0x0<<5)
#define PFUZE100_SW3BMODE_OMODE_M	(0x1<<5)
#define PFUZE100_SW3BMODE_MODE_VAL	(0x7<<0)
#define PFUZE100_SW3BMODE_MODE_M	(0xf<<0)

#define PFUZE100_SW3BCON		71
#define PFUZE100_SW3BCON_SPEED_VAL	(0x1<<6)
#define PFUZE100_SW3BCON_SPEED_M	(0x3<<6)
#define PFUZE100_SW3BCON_PHASE_VAL	(0x1<<4)
#define PFUZE100_SW3BCON_PHASE_M	(0x3<<4)
#define PFUZE100_SW3BCON_FREQ_VAL	(0x1<<2)
#define PFUZE100_SW3BCON_FREQ_M		(0x3<<2)
#define PFUZE100_SW3BCON_LIM_VAL	(0x0<<0)
#define PFUZE100_SW3BCON_LIM_M		(0x3<<0)

/*SW4*/
#define PFUZE100_SW4VOL		74
#define PFUZE100_SW4VOL_VSEL	0
#define PFUZE100_SW4VOL_VSEL_M	(0x3f<<0)

#define PFUZE100_SW4VOL_STBY		75
#define PFUZE100_SW4VOL_STBY_VSEL	0
#define PFUZE100_SW4VOL_STBY_VSEL_M	(0x3f<<0)

#define PFUZE100_SW4OFF		76
#define PFUZE100_SW4OFF_OFF_VAL	0x0
#define PFUZE100_SW4OFF_OFF_M	(0x3f<<0)

#define PFUZE100_SW4MODE	77
#define PFUZE100_SW4MODE_OMODE_VAL	(0x0<<5)
#define PFUZE100_SW4MODE_OMODE_M	(0x1<<5)
#define PFUZE100_SW4MODE_MODE_VAL	(0x7<<0)
#define PFUZE100_SW4MODE_MODE_M		(0xf<<0)

#define PFUZE100_SW4CON		78
#define PFUZE100_SW4CON_SPEED_VAL	(0x1<<6)
#define PFUZE100_SW4CON_SPEED_M		(0x3<<6)
#define PFUZE100_SW4CON_PHASE_VAL	(0x1<<4)
#define PFUZE100_SW4CON_PHASE_M		(0x3<<4)
#define PFUZE100_SW4CON_FREQ_VAL	(0x1<<2)
#define PFUZE100_SW4CON_FREQ_M		(0x3<<2)
#define PFUZE100_SW4CON_LIM_VAL		(0x0<<0)
#define PFUZE100_SW4CON_LIM_M		(0x3<<0)

 /*SWBST*/
#define PFUZE100_SWBSTCON1	102
#define PFUZE100_SWBSTCON1_SWBSTMOD_VAL	(0x1<<2)
#define PFUZE100_SWBSTCON1_SWBSTMOD_M	(0x3<<2)
#define PFUZE100_SWBSTCON1_VSEL	0
#define PFUZE100_SWBSTCON1_VSEL_M	(0x3<<0)
     /*VREFDDR*/
#define PFUZE100_VREFDDRCON	106
#define PFUZE100_VREFDDRCON_EN	(0x1<<4)
     /*VSNVS*/
#define PFUZE100_VSNVSVOL		107
#define PFUZE100_VSNVSVOL_VSEL	0
#define PFUZE100_VSNVSVOL_VSEL_M	(0x3<<0)
/*VGEN1*/
#define PFUZE100_VGEN1VOL		108
#define PFUZE100_VGEN1VOL_STBY	(0x1<<5)
#define PFUZE100_VGEN1VOL_EN	(0x1<<4)
#define PFUZE100_VGEN1VOL_VSEL	0
#ifdef PFUZE100_FIRST_VERSION
#define PFUZE100_VGEN1VOL_VSEL_M	(0x7<<0)
#else
#define PFUZE100_VGEN1VOL_VSEL_M	(0xf<<0)
#endif
/*VGEN2*/
#define PFUZE100_VGEN2VOL		109
#define PFUZE100_VGEN2VOL_STBY	(0x1<<5)
#define PFUZE100_VGEN2VOL_EN	(0x1<<4)
#define PFUZE100_VGEN2VOL_VSEL	0
#ifdef PFUZE100_FIRST_VERSION
#define PFUZE100_VGEN2VOL_VSEL_M	(0x7<<0)
#else
#define PFUZE100_VGEN2VOL_VSEL_M	(0xf<<0)
#endif
/*VGEN3*/
#define PFUZE100_VGEN3VOL		110
#define PFUZE100_VGEN3VOL_STBY	(0x1<<5)
#define PFUZE100_VGEN3VOL_EN	(0x1<<4)
#define PFUZE100_VGEN3VOL_VSEL	0
#define PFUZE100_VGEN3VOL_VSEL_M	(0xf<<0)
/*VGEN4*/
#define PFUZE100_VGEN4VOL		111
#define PFUZE100_VGEN4VOL_STBY	(0x1<<5)
#define PFUZE100_VGEN4VOL_EN	(0x1<<4)
#define PFUZE100_VGEN4VOL_VSEL	0
#define PFUZE100_VGEN4VOL_VSEL_M	(0xf<<0)
/*VGEN5*/
#define PFUZE100_VGEN5VOL		112
#define PFUZE100_VGEN5VOL_STBY	(0x1<<5)
#define PFUZE100_VGEN5VOL_EN	(0x1<<4)
#define PFUZE100_VGEN5VOL_VSEL	0
#define PFUZE100_VGEN5VOL_VSEL_M	(0xf<<0)
/*VGEN6*/
#define PFUZE100_VGEN6VOL		113
#define PFUZE100_VGEN6VOL_STBY	(0x1<<5)
#define PFUZE100_VGEN6VOL_EN	(0x1<<4)
#define PFUZE100_VGEN6VOL_VSEL	0
#define PFUZE100_VGEN6VOL_VSEL_M	(0xf<<0)
static struct pfuze_regulator pfuze100_regulators[] = {
	PFUZE100_SW_DEFINE(SW1A, SW1AVOL, pfuze100_sw1),
	PFUZE100_SW_DEFINE(SW1B, SW1BVOL, pfuze100_sw1),
	PFUZE100_SW_DEFINE(SW1C, SW1CVOL, pfuze100_sw1),
	PFUZE100_SW_DEFINE(SW2, SW2VOL, pfuze100_sw2),
	PFUZE100_SW_DEFINE(SW3A, SW3AVOL, pfuze100_sw3),
	PFUZE100_SW_DEFINE(SW3B, SW3BVOL, pfuze100_sw3),
	PFUZE100_SW_DEFINE(SW4, SW4VOL, pfuze100_sw4),
	PFUZE100_SWBST_DEFINE(SWBST, SWBSTCON1, pfuze100_swbst),
	PFUZE100_SWBST_DEFINE(VSNVS, VSNVSVOL, pfuze100_vsnvs),
	PFUZE100_FIXED_VOL_DEFINE(VREFDDR, VREFDDRCON, pfuze100_vrefddr),
	PFUZE100_VGEN_DEFINE(VGEN1, VGEN1VOL, pfuze100_vgen12),
	PFUZE100_VGEN_DEFINE(VGEN2, VGEN2VOL, pfuze100_vgen12),
	PFUZE100_VGEN_DEFINE(VGEN3, VGEN3VOL, pfuze100_vgen36),
	PFUZE100_VGEN_DEFINE(VGEN4, VGEN4VOL, pfuze100_vgen36),
	PFUZE100_VGEN_DEFINE(VGEN5, VGEN5VOL, pfuze100_vgen36),
	PFUZE100_VGEN_DEFINE(VGEN6, VGEN6VOL, pfuze100_vgen36),
};

static int pfuze100_regulator_enable(struct regulator_dev *rdev)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);
	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_rmw(priv->pfuze, pfuze100_regulators[id].reg,
			    pfuze100_regulators[id].enable_bit,
			    pfuze100_regulators[id].enable_bit);
	pfuze_unlock(priv->pfuze);
	return ret;
}

static int pfuze100_regulator_disable(struct regulator_dev *rdev)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);
	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_rmw(priv->pfuze, pfuze100_regulators[id].reg,
			    pfuze100_regulators[id].enable_bit, 0);
	pfuze_unlock(priv->pfuze);
	return ret;
}

static int pfuze100_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;
	unsigned char val;

	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_read(priv->pfuze, pfuze100_regulators[id].reg, &val);
	pfuze_unlock(priv->pfuze);
	if (ret)
		return ret;
	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);
	return (val & pfuze100_regulators[id].enable_bit) != 0;
}

int pfuze100_regulator_list_voltage(struct regulator_dev *rdev,
				    unsigned selector)
{
	int id = rdev_get_id(rdev);

	if (selector >= pfuze100_regulators[id].desc.n_voltages)
		return -EINVAL;
	return pfuze100_regulators[id].voltages[selector];
}

int pfuze100_get_best_voltage_index(struct regulator_dev *rdev, int min_uV,
				    int max_uV)
{
	int reg_id = rdev_get_id(rdev);
	int i, bestmatch, bestindex;

	bestmatch = INT_MAX;
	bestindex = -1;
	for (i = 0; i < pfuze100_regulators[reg_id].desc.n_voltages; i++) {
		if (pfuze100_regulators[reg_id].voltages[i] >= min_uV &&
		    pfuze100_regulators[reg_id].voltages[i] < bestmatch) {
			bestmatch = pfuze100_regulators[reg_id].voltages[i];
			bestindex = i;
		}
	}
	if (bestindex < 0 || bestmatch > max_uV) {
		dev_warn(&rdev->dev, "no possible value for %d<=x<=%d uV\n",
			 min_uV, max_uV);
		return -EINVAL;
	}
	return bestindex;
}

EXPORT_SYMBOL_GPL(pfuze100_get_best_voltage_index);

static int
pfuze100_regulator_set_voltage(struct regulator_dev *rdev, int min_uV,
			       int max_uV, unsigned *selector)
{

	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int value, id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d min_uV: %d max_uV: %d\n",
		__func__, id, min_uV, max_uV);
	/* Find the best index */
	value = pfuze100_get_best_voltage_index(rdev, min_uV, max_uV);
	dev_dbg(rdev_get_dev(rdev), "%s best value: %d\n", __func__, value);
	if (value < 0)
		return value;
	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_rmw(priv->pfuze, pfuze100_regulators[id].reg,
			    pfuze100_regulators[id].vsel_mask,
			    value << pfuze100_regulators[id].vsel_shift);
	pfuze_unlock(priv->pfuze);
	return ret;

}

static int pfuze100_regulator_set_voltage_sel(struct regulator_dev *rdev,
					      unsigned selector)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int  id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d vol: %d\n",
		__func__, id, pfuze100_regulators[id].voltages[selector]);

	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_rmw(priv->pfuze, pfuze100_regulators[id].reg,
			    pfuze100_regulators[id].vsel_mask,
			    selector << pfuze100_regulators[id].vsel_shift);
	pfuze_unlock(priv->pfuze);
	return ret;
}

static int pfuze100_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int ret, id = rdev_get_id(rdev);
	unsigned char val;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);
	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_read(priv->pfuze, pfuze100_regulators[id].reg, &val);
	pfuze_unlock(priv->pfuze);
	if (ret)
		return ret;
	val = (val & pfuze100_regulators[id].vsel_mask)
	    >> pfuze100_regulators[id].vsel_shift;
	dev_dbg(rdev_get_dev(rdev), "%s id: %d val: %d\n", __func__, id, val);
	BUG_ON(val > pfuze100_regulators[id].desc.n_voltages);
	return pfuze100_regulators[id].voltages[val];
}

static int pfuze100_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;
	unsigned char val;

	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_read(priv->pfuze, pfuze100_regulators[id].reg, &val);
	pfuze_unlock(priv->pfuze);
	if (ret)
		return ret;

	val &= pfuze100_regulators[id].vsel_mask;
	val >>= pfuze100_regulators[id].vsel_shift;
	dev_dbg(rdev_get_dev(rdev), "%s id: %d, vol=%d\n", __func__, id,
		pfuze100_regulators[id].voltages[val]);
	return (int) val;
}

static int pfuze100_regulator_set_voltage_time_sel(struct regulator_dev *rdev,
					     unsigned int old_sel,
					     unsigned int new_sel)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;
	unsigned char step_delay;

	pfuze_lock(priv->pfuze);
	/*read SWxDVSSPEED from SWxCONF,got ramp step value*/
	ret = pfuze_reg_read(priv->pfuze, pfuze100_regulators[id].reg + 0x4,
				&step_delay);
	pfuze_unlock(priv->pfuze);

	if (ret)
		return ret;
	/*
	 * one step
	 * 00: 2us,
	 * 01: 4us,
	 * 02: 8us,
	 * 03: 16us,
	 */
	step_delay >>= 6;
	step_delay &= 0x3;
	step_delay = 2  <<  step_delay;

	if (pfuze100_regulators[id].voltages[old_sel] <
		pfuze100_regulators[id].voltages[new_sel])
		ret = DIV_ROUND_UP(pfuze100_regulators[id].voltages[new_sel] -
			pfuze100_regulators[id].voltages[old_sel], 25000)
			* step_delay;
	else
		ret = 0; /* no delay if voltage drop */
	dev_dbg(rdev_get_dev(rdev), "%s id: %d, new_sel = %d, old_sel = %d, \
		delay = %d\n", __func__, id, new_sel, old_sel, ret);
	return ret;
}

static int pfuze100_regulator_ldo_standby_enable(struct regulator_dev *rdev)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);
	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_rmw(priv->pfuze, pfuze100_regulators[id].reg,
			    pfuze100_regulators[id].stby_bit,
			    0);
	pfuze_unlock(priv->pfuze);
	return ret;
}

static int pfuze100_regulator_ldo_standby_disable(struct regulator_dev *rdev)
{
	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);
	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_rmw(priv->pfuze, pfuze100_regulators[id].reg,
			    pfuze100_regulators[id].stby_bit,
			    pfuze100_regulators[id].stby_bit);
	pfuze_unlock(priv->pfuze);
	return ret;
}

static struct regulator_ops pfuze100_ldo_regulator_ops = {
	.enable = pfuze100_regulator_enable,
	.disable = pfuze100_regulator_disable,
	.is_enabled = pfuze100_regulator_is_enabled,
	.list_voltage = pfuze100_regulator_list_voltage,
	.set_voltage = pfuze100_regulator_set_voltage,
	.get_voltage = pfuze100_regulator_get_voltage,
	.set_suspend_enable = pfuze100_regulator_ldo_standby_enable,
	.set_suspend_disable = pfuze100_regulator_ldo_standby_disable,
};

static int pfuze100_fixed_regulator_set_voltage(struct regulator_dev *rdev,
						int min_uV, int max_uV,
						unsigned *selector)
{
	int id = rdev_get_id(rdev);

	dev_dbg(rdev_get_dev(rdev), "%s id: %d min_uV: %d max_uV: %d\n",
		__func__, id, min_uV, max_uV);
	if (min_uV >= pfuze100_regulators[id].voltages[0] &&
	    max_uV <= pfuze100_regulators[id].voltages[0])
		return 0;
	else
		return -EINVAL;

}

static int pfuze100_fixed_regulator_get_voltage(struct regulator_dev *rdev)
{
	int id = rdev_get_id(rdev);

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);
	return pfuze100_regulators[id].voltages[0];
}

static struct regulator_ops pfuze100_fixed_regulator_ops = {
	.enable = pfuze100_regulator_enable,
	.disable = pfuze100_regulator_disable,
	.is_enabled = pfuze100_regulator_is_enabled,
	.set_voltage = pfuze100_fixed_regulator_set_voltage,
	.get_voltage = pfuze100_fixed_regulator_get_voltage,
};

static int pfuze100_sw_regulator_is_enabled(struct regulator_dev *rdev)
{
	return 1;
}

static int
pfuze100_regulator_sw_standby_voltage(struct regulator_dev *rdev, int uV)
{

	struct pfuze_regulator_priv *priv = rdev_get_drvdata(rdev);
	int value, id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d set standby: %d\n",
		__func__, id, uV);
	/* Find the best index */
	value = pfuze100_get_best_voltage_index(rdev, uV, uV);
	if (value < 0)
		return value;
	pfuze_lock(priv->pfuze);
	ret = pfuze_reg_rmw(priv->pfuze, pfuze100_regulators[id].stby_reg,
			    pfuze100_regulators[id].stby_vsel_mask,
			    value << pfuze100_regulators[id].stby_vsel_shift);
	pfuze_unlock(priv->pfuze);
	return ret;

}

static int pfuze100_regulator_sw_standby_enable(struct regulator_dev *rdev)
{
	return 0;
}
static int pfuze100_regulator_sw_standby_disable(struct regulator_dev *rdev)
{
	return 0;
}
static struct regulator_ops pfuze100_sw_regulator_ops = {
	.is_enabled = pfuze100_sw_regulator_is_enabled,
	.list_voltage = pfuze100_regulator_list_voltage,
	.set_voltage_sel = pfuze100_regulator_set_voltage_sel,
	.get_voltage_sel = pfuze100_regulator_get_voltage_sel,
	.set_suspend_enable = pfuze100_regulator_sw_standby_enable,
	.set_suspend_disable = pfuze100_regulator_sw_standby_disable,
	.set_suspend_voltage = pfuze100_regulator_sw_standby_voltage,
	.set_voltage_time_sel = pfuze100_regulator_set_voltage_time_sel,

};

static int __devinit pfuze100_regulator_probe(struct platform_device *pdev)
{
	struct pfuze_regulator_priv *priv;
	struct mc_pfuze *pfuze100 = dev_get_drvdata(pdev->dev.parent);
	struct pfuze_regulator_platform_data *pdata =
	    dev_get_platdata(&pdev->dev);
	struct pfuze_regulator_init_data *init_data;
	int i, ret;

	priv = kzalloc(sizeof(*priv) +
		       pdata->num_regulators * sizeof(priv->regulators[0]),
		       GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->pfuze_regulators = pfuze100_regulators;
	priv->pfuze = pfuze100;
	pfuze_lock(pfuze100);
	ret = pdata->pfuze_init(pfuze100);
	if (ret)
		goto err_free;
	pfuze_unlock(pfuze100);
	for (i = 0; i < pdata->num_regulators; i++) {
		init_data = &pdata->regulators[i];
		priv->regulators[i] =
		    regulator_register(&pfuze100_regulators[init_data->id].desc,
				       &pdev->dev, init_data->init_data, priv);
		if (IS_ERR(priv->regulators[i])) {
			dev_err(&pdev->dev, "failed to register regulator %s\n",
				pfuze100_regulators[i].desc.name);
			ret = PTR_ERR(priv->regulators[i]);
			goto err;
		}
	}
	platform_set_drvdata(pdev, priv);
	return 0;
err:
	while (--i >= 0)
		regulator_unregister(priv->regulators[i]);
err_free:
	pfuze_unlock(pfuze100);
	kfree(priv);
	return ret;
}

static int __devexit pfuze100_regulator_remove(struct platform_device *pdev)
{
	struct pfuze_regulator_priv *priv = platform_get_drvdata(pdev);
	struct pfuze_regulator_platform_data *pdata =
	    dev_get_platdata(&pdev->dev);
	int i;

	platform_set_drvdata(pdev, NULL);
	for (i = 0; i < pdata->num_regulators; i++)
		regulator_unregister(priv->regulators[i]);

	kfree(priv);
	return 0;
}

static struct platform_driver pfuze100_regulator_driver = {
	.driver = {
		   .name = "pfuze100-regulator",
		   .owner = THIS_MODULE,
		   },
	.remove = __devexit_p(pfuze100_regulator_remove),
	.probe = pfuze100_regulator_probe,
};

static int __init pfuze100_regulator_init(void)
{
	return platform_driver_register(&pfuze100_regulator_driver);
}

subsys_initcall(pfuze100_regulator_init);
static void __exit pfuze100_regulator_exit(void)
{
	platform_driver_unregister(&pfuze100_regulator_driver);
}

module_exit(pfuze100_regulator_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Regulator Driver for Freescale PFUZE100 PMIC");
MODULE_ALIAS("pfuze100-regulator");
