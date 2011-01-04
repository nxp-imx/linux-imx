/*
 * da9052 Power Management module declarations.
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_MFD_DA9052_PM_H
#define __LINUX_MFD_DA9052_PM_H

/* PM Device name and static Major number macros */
#define DRIVER_NAME				"da9052-regulator"

/* PM Device Macros */
#define DA9052_LDO1				0
#define DA9052_LDO2				1
#define DA9052_LDO3				2
#define DA9052_LDO4				3
#define DA9052_LDO5				4
#define DA9052_LDO6				5
#define DA9052_LDO7				6
#define DA9052_LDO8				7
#define DA9052_LDO9				8
#define DA9052_LDO10				9
#define DA9052_BUCK_CORE			10
#define DA9052_BUCK_PRO				11
#define DA9052_BUCK_MEM				12
#define DA9052_BUCK_PERI			13

/* PM Device Error Codes */

/* Buck Config Validation Macros */
#define DA9052_BUCK_CORE_PRO_VOLT_UPPER		2075
#define DA9052_BUCK_CORE_PRO_VOLT_LOWER		500
#define DA9052_BUCK_CORE_PRO_STEP		25
#define DA9052_BUCK_MEM_VOLT_UPPER		2500
#define DA9052_BUCK_MEM_VOLT_LOWER		925
#define DA9052_BUCK_MEM_STEP			25
#define DA9052_BUCK_PERI_VOLT_UPPER		3600
#define DA9052_BUCK_PERI_VOLT_LOWER		1800
#define DA9052_BUCK_PERI_STEP_BELOW_3000	50
#define DA9052_BUCK_PERI_STEP_ABOVE_3000	100000
#define DA9052_BUCK_PERI_VALUES_UPTO_3000	24
#define DA9052_BUCK_PERI_VALUES_3000		3000000
#define DA9052_LDO1_VOLT_UPPER			1800
#define DA9052_LDO1_VOLT_LOWER			600
#define DA9052_LDO1_VOLT_STEP			50
#define DA9052_LDO2_VOLT_UPPER			1800
#define DA9052_LDO2_VOLT_LOWER			600
#define DA9052_LDO2_VOLT_STEP			25
#define DA9052_LDO34_VOLT_UPPER			3300
#define DA9052_LDO34_VOLT_LOWER			1725
#define DA9052_LDO34_VOLT_STEP			25
#define DA9052_LDO567810_VOLT_UPPER		3600
#define DA9052_LDO567810_VOLT_LOWER		1200
#define DA9052_LDO567810_VOLT_STEP		50
#define DA9052_LDO9_VOLT_STEP			50
#define DA9052_LDO9_VOLT_LOWER			1250
#define DA9052_LDO9_VOLT_UPPER			3650

#endif /* __LINUX_MFD_DA9052_PM_H */
