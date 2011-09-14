/*
 * da9052 Event Handler module declarations.
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

#ifndef __LINUX_MFD_DA9052_EH_H
#define __LINUX_MFD_DA9052_EH_H

#define DA9052_EVE_REGISTERS		4
#define DA9052_EVE_REGISTER_SIZE	8

/* Define for all possible events */
#define DCIN_DET_EVE			0
#define VBUS_DET_EVE			1
#define DCIN_REM_EVE			2
#define VBUS_REM_EVE			3
#define VDD_LOW_EVE			4
#define ALARM_EVE			5
#define SEQ_RDY_EVE			6
#define COMP_1V2			7
#define ONKEY_EVE			8
#define ID_FLOAT_EVE			9
#define ID_GND_EVE			10
#define CHG_END_EVE			11
#define TBAT_EVE			12
#define ADC_EOM_EVE			13
#define PEN_DOWN_EVE			14
#define TSI_READY_EVE			15
#define GPI0_EVE			16
#define GPI1_EVE			17
#define GPI2_EVE			18
#define GPI3_EVE			19
#define GPI4_EVE			20
#define GPI5_EVE			21
#define GPI6_EVE			22
#define GPI7_EVE			23
#define GPI8_EVE			24
#define GPI9_EVE			25
#define GPI10_EVE			26
#define GPI11_EVE			27
#define GPI12_EVE			28
#define GPI13_EVE			29
#define GPI14_EVE			30
#define GPI15_EVE			31

/* Total number of events */
#define EVE_CNT				(GPI15_EVE+1)

/* Error code for register/unregister functions */
#define INVALID_NB			2
#define INVALID_EVE			3

/* State for EH thread */
#define ACTIVE				0
#define INACTIVE			1

/* Status of nIRQ line */
#define IRQ_HIGH			0
#define IRQ_LOW				1

#endif /* __LINUX_MFD_DA9052_EH_H */
