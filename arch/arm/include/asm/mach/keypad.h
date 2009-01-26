/*
 * include/asm-arm/mach/keypad.h
 *
 * Generic Keypad struct
 *
 * Author: Armin Kuster <Akuster@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __ASM_MACH_KEYPAD_H_
#define __ASM_MACH_KEYPAD_H_

#include <linux/input.h>

struct keypad_data {
	u16 rowmax;
	u16 colmax;
	u32 irq;
	u16 delay;
	u16 learning;
	u16 *matrix;
};

#endif /* __ARM_MACH_KEYPAD_H_ */
