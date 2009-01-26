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
#ifndef __ASM_ARCH_MXC_GPIO_H__
#define __ASM_ARCH_MXC_GPIO_H__

/*!
 * @defgroup GPIO General Purpose Input Output (GPIO)
 */

/*!
 * @file arch-mxc/gpio.h
 * @brief This file contains the GPIO API functions.
 *
 * @ingroup GPIO
 */

#include <linux/interrupt.h>
#include <asm/sizes.h>

typedef unsigned int iomux_pin_name_t;

/* gpio related defines */

/*!
 * There are two queues for registered GPIO ISRs. One is for high priority and
 * the other is for low priority. The ISRs in the high priority queue will be
 * called first before the low priority queue if more than one GPIO interrupt
 * occurs at the same time.
 */
enum gpio_prio {
	GPIO_HIGH_PRIO = 0,	/*!< high priority queue */
	GPIO_LOW_PRIO		/*!< low priority queue */
};

/*!
 * This enumeration data type defines various different ways for interrupting
 * the ARM core from GPIO signals. The way to interrupt the core is dictated
 * by the external hardware.
 */
typedef enum gpio_int_cfg {
#if defined(CONFIG_ARCH_MX21) || defined(CONFIG_ARCH_MX27)
	GPIO_INT_LOW_LEV = 0x3,	/*!< low level sensitive */
	GPIO_INT_HIGH_LEV = 0x2,	/*!< high level sensitive */
	GPIO_INT_RISE_EDGE = 0x0,	/*!< rising edge sensitive */
	GPIO_INT_FALL_EDGE = 0x1,	/*!< falling edge sensitive */
	GPIO_INT_NONE = 0x4	/*!< No interrupt */
#else
	GPIO_INT_LOW_LEV = 0x0,	/*!< low level sensitive */
	GPIO_INT_HIGH_LEV = 0x1,	/*!< high level sensitive */
	GPIO_INT_RISE_EDGE = 0x2,	/*!< rising edge sensitive */
	GPIO_INT_FALL_EDGE = 0x3,	/*!< falling edge sensitive */
	GPIO_INT_NONE = 0x4	/*!< No interrupt */
#endif
} gpio_edge_t;

typedef irqreturn_t(*gpio_irq_handler) (int, void *);

/*!
 * This function configures the GPIO signal to be either input or output. For
 * input signals used for generating interrupts for the ARM core, how the
 * interrupts being triggered is also passed in via \a icr. For output signals,
 * the \a icr value doesn't matter.
 *
 * @param  port         specified port with 0-GPIO port 1; 1-GPIO port 2
 * @param  sig_no       specified GPIO signal (0 based)
 * @param  out          #true for output, #false for input
 * @param  icr          value defined in \b #gpio_int_cfg
 */
void gpio_config(__u32 port, __u32 sig_no, bool out, enum gpio_int_cfg icr);

/*!
 * This function sets a GPIO signal value.
 *
 * @param  port         specified port with 0-GPIO port 1; 1-GPIO port 2
 * @param  sig_no       specified GPIO signal (0 based)
 * @param  data         value to be set (only 0 or 1 is valid)
 */
void gpio_set_data(__u32 port, __u32 sig_no, __u32 data);

/*!
 * This function returns the value of the GPIO signal.
 *
 * @param  port         specified port with 0-GPIO port 1; 1-GPIO port 2
 * @param  sig_no       specified GPIO signal (0 based)
 *
 * @return Value of the GPIO signal
 */
__u32 gpio_get_data(__u32 port, __u32 sig_no);

/*!
 * This function is responsible for registering a GPIO signal's ISR.
 *
 * @param  port         specified port with 0-GPIO port 1; 1-GPIO port 2
 * @param  sig_no       specified GPIO signal (0 based)
 * @param  prio         priority as defined in \b enum \b #gpio_prio
 * @param  handler      GPIO ISR function pointer for the GPIO signal
 * @param  irq_flags    irq flags (not used)
 * @param  devname      device name associated with the interrupt
 * @param  dev_id       some unique information for the ISR
 *
 * @return 0 if successful; non-zero otherwise.
 */
int gpio_request_irq(__u32 port, __u32 sig_no, enum gpio_prio prio,
		     gpio_irq_handler handler, __u32 irq_flags,
		     const char *devname, void *dev_id);

/*!
 * This function un-registers an ISR with the GPIO interrupt module.
 *
 * @param  port         specified port with 0-GPIO port 1; 1-GPIO port 2
 * @param  sig_no       specified GPIO signal (0 based)
 * @param  prio         priority as defined in \b enum \b #gpio_prio
 */
void gpio_free_irq(__u32 port, __u32 sig_no, enum gpio_prio prio);

/*!
 * Request ownership for a GPIO pin. The caller has to check the return value
 * of this function to make sure it returns 0 before make use of that pin.
 * @param pin		a name defined by \b iomux_pin_name_t
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_gpio(iomux_pin_name_t pin);

/*!
 * Exported function to set a GPIO pin's direction
 * @param pin		a name defined by \b iomux_pin_name_t
 * @param is_input	1 (or non-zero) for input; 0 for output
 */
void mxc_set_gpio_direction(iomux_pin_name_t pin, int is_input);

/*!
 * Exported function to set a GPIO pin's data output
 * @param pin		a name defined by \b iomux_pin_name_t
 * @param data		value to be set (only 0 or 1 is valid)
 */
void mxc_set_gpio_dataout(iomux_pin_name_t pin, u32 data);

/*!
 * Return the data value of a GPIO signal.
 * @param pin	a name defined by \b iomux_pin_name_t
 *
 * @return 	value (0 or 1) of the GPIO signal; -1 if pass in invalid pin
 */
int mxc_get_gpio_datain(iomux_pin_name_t pin);

/*!
 * Release ownership for a GPIO pin
 * @param pin		a name defined by \b iomux_pin_name_t
 */
void mxc_free_gpio(iomux_pin_name_t pin);

/*!
 * GPIO driver initialization
 * @return    always 0
 */
int mxc_gpio_init(void);
#endif				/* __ASM_ARCH_MXC_GPIO_H__ */
