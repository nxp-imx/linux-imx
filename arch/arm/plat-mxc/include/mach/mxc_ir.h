/*
 * Copyright 2004-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file arch-mxc/mxc_ir.h
 *
 * @brief This file contains the IR configuration structure definition.
 *
 *
 * @ingroup IR
 */

#ifndef __ASM_ARCH_MXC_IR_H__
#define __ASM_ARCH_MXC_IR_H__

#ifdef __KERNEL__

struct platform_ir_data {
	int pwm_id;     /* generate carry frequence */
	int epit_id;    /* generate payload pluse   */
	int gpio_id;    /* dedicate GPIO for IR     */
};

#endif				/* __KERNEL__ */

#endif				/* __ASM_ARCH_MXC_IR_H__ */
