/*
 * Freescale STMP37XX/STMP378X GPIO interface
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

extern int	gpio_request(unsigned gpio, char *label);
extern void	gpio_free(unsigned gpio);
extern void 	gpio_set_value(unsigned gpio, int value);
extern int 	gpio_get_value(unsigned gpio);
extern int 	gpio_to_irq(unsigned gpio);
extern void 	gpio_direction_input(unsigned id);
extern void 	gpio_direction_output(unsigned id, int value);

#endif /* __ASM_ARCH_GPIO_H */
