/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup GPIO_MX27 Board GPIO and Muxing Setup
 * @ingroup MSL_MX27
 */
/*!
 * @file mach-mx27/gpio_mux.c
 *
 * @brief I/O Muxing control functions
 *
 * @ingroup GPIO_MX27
 */

#include <linux/kernel.h>
#include <linux/cache.h>
#include <linux/spinlock.h>

#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include "gpio_mux.h"

/*!
 * This structure defines the offset of registers in gpio module.
 */
enum gpio_reg {
	GPIO_GIUS = 0x20,
	GPIO_GPR = 0x38,
	GPIO_PUEN = 0x40,
	GPIO_DDIR = 0x00,
	GPIO_OCR1 = 0x04,
	GPIO_OCR2 = 0x08,
	GPIO_ICONFA1 = 0x0C,
	GPIO_ICONFA2 = 0x10,
	GPIO_ICONFB1 = 0x14,
	GPIO_ICONFB2 = 0x18,
};

/*!
 * This enumeration data type defines the configuration for input mode.
 */
typedef enum {
	GPIO_INPUT_GPIO = 0x00,
	GPIO_INPUT_INTR = 0x01,
	GPIO_INPUT_LOW = 0x02,
	GPIO_INPUT_HIGH = 0x03
} gpio_input_cfg_t;

/*!
 * This enumeration data type defines the configuration for output mode.
 */
typedef enum {
	GPIO_OUTPUT_A = 0x00,
	GPIO_OUTPUT_B = 0x01,
	GPIO_OUTPUT_C = 0x02,
	GPIO_OUTPUT_DR = 0x03
} gpio_output_cfg_t;

extern struct mxc_gpio_port mxc_gpio_ports[];

/*!
 * defines a spinlock to protected the accessing to gpio pin.
 */
DEFINE_SPINLOCK(gpio_mux_lock);

/*!
 * This function enable or disable the pullup feature to the pin.
 * @param port  	a pointer of gpio port
 * @param index 	the index of the  pin in the port
 * @param en		0 if disable pullup, otherwise enable it.
 * @return		none
 */
static inline void _gpio_set_puen(struct mxc_gpio_port *port, u32 index,
				  bool en)
{
	u32 reg;

	reg = __raw_readl(port->base + GPIO_PUEN);
	if (en) {
		reg |= 1 << index;
	} else {
		reg &= ~(1 << index);
	}
	__raw_writel(reg, port->base + GPIO_PUEN);
}

/*!
 * This function set the input configuration A.
 * @param port  	a pointer of gpio port
 * @param index 	the index of the  pin in the port
 * @param config	a mode as define in \b #gpio_input_cfg_t
 * @return		none
 */
static inline void _gpio_set_iconfa(struct mxc_gpio_port *port, u32 index,
				    gpio_input_cfg_t config)
{
	u32 reg, val;
	u32 mask;

	mask = 0x3 << ((index % 16) << 1);

	if (index >= 16) {
		reg = port->base + GPIO_ICONFA2;
		val = config << ((index - 16) * 2);
	} else {
		reg = port->base + GPIO_ICONFA1;
		val = config << (index * 2);
	}
	val |= __raw_readl(reg) & ~(mask);
	__raw_writel(val, reg);
}

/*!
 * This function set the input configuration B.
 * @param port  	a pointer of gpio port
 * @param index 	the index of the  pin in the port
 * @param config	a mode as define in \b #gpio_input_cfg_t
 * @return		none
 */
static inline void _gpio_set_iconfb(struct mxc_gpio_port *port, u32 index,
				    gpio_input_cfg_t config)
{
	u32 reg, val;
	u32 mask;

	mask = 0x3 << ((index % 16) << 1);

	if (index >= 16) {
		reg = port->base + GPIO_ICONFB2;
		val = config << ((index - 16) * 2);
	} else {
		reg = port->base + GPIO_ICONFB1;
		val = config << (index * 2);
	}
	val |= __raw_readl(reg) & (~mask);
	__raw_writel(val, reg);
}

/*!
 * This function set the output configuration.
 * @param port  	a pointer of gpio port
 * @param index 	the index of the  pin in the port
 * @param config	a mode as define in \b #gpio_output_cfg_t
 * @return		none
 */
static inline void _gpio_set_ocr(struct mxc_gpio_port *port, u32 index,
				 gpio_output_cfg_t config)
{
	u32 reg, val;
	u32 mask;

	mask = 0x3 << ((index % 16) << 1);
	if (index >= 16) {
		reg = port->base + GPIO_OCR2;
		val = config << ((index - 16) * 2);
	} else {
		reg = port->base + GPIO_OCR1;
		val = config << (index * 2);
	}
	val |= __raw_readl(reg) & (~mask);
	__raw_writel(val, reg);
}

/*!
 *@brief gpio_config_mux - just configure the mode of the gpio pin.
 *@param pin   a pin number as defined in \b #iomux_pin_name_t
 *@param mode  a module as define in \b #gpio_mux_mode_t;
 *	GPIO_MUX_PRIMARY set pin to work as primary function.
 *	GPIO_MUX_ALT set pin to work as alternate function.
 *	GPIO_MUX_GPIO set pin to work as output function based the data register
 *	GPIO_MUX_INPUT1 set pin to work as input function connected with  A_OUT
 *	GPIO_MUX_INPUT2 set pin to work as input function connected with B_OUT
 *	GPIO_MUX_OUTPUT1 set pin to work as output function connected with A_IN
 *	GPIO_MUX_OUTPUT2 set pin to work as output function connected with B_IN
 *	GPIO_MUX_OUTPUT3 set pin to work as output function connected with C_IN
 *@return      0 if successful, Non-zero otherwise
 */

int gpio_config_mux(iomux_pin_name_t pin, gpio_mux_mode_t mode)
{
	unsigned long lock_flags;
	u32 gius_reg, gpr_reg;
	struct mxc_gpio_port *port;
	u32 index, gpio = IOMUX_TO_GPIO(pin);

	port = &(mxc_gpio_ports[GPIO_TO_PORT(gpio)]);
	index = GPIO_TO_INDEX(gpio);

	pr_debug("%s: Configuring PORT %c, bit %d\n",
		 __func__, GPIO_TO_PORT(gpio) + 'A', index);

	spin_lock_irqsave(&gpio_mux_lock, lock_flags);

	gius_reg = __raw_readl(port->base + GPIO_GIUS);
	gpr_reg = __raw_readl(port->base + GPIO_GPR);

	switch (mode) {
	case GPIO_MUX_PRIMARY:
		gius_reg &= ~(1L << index);
		gpr_reg &= ~(1L << index);
		break;
	case GPIO_MUX_ALT:
		gius_reg &= ~(1L << index);
		gpr_reg |= (1L << index);
		break;
	case GPIO_MUX_GPIO:
		gius_reg |= (1L << index);
		_gpio_set_ocr(port, index, GPIO_OUTPUT_DR);
		break;
	case GPIO_MUX_INPUT1:
		gius_reg |= (1L << index);
		_gpio_set_iconfa(port, index, GPIO_INPUT_GPIO);
		break;
	case GPIO_MUX_INPUT2:
		gius_reg |= (1L << index);
		_gpio_set_iconfb(port, index, GPIO_INPUT_GPIO);
		break;
	case GPIO_MUX_OUTPUT1:
		gius_reg |= (1L << index);
		_gpio_set_ocr(port, index, GPIO_OUTPUT_A);
		break;
	case GPIO_MUX_OUTPUT2:
		gius_reg |= (1L << index);
		_gpio_set_ocr(port, index, GPIO_OUTPUT_B);
		break;
	case GPIO_MUX_OUTPUT3:
		gius_reg |= (1L << index);
		_gpio_set_ocr(port, index, GPIO_OUTPUT_C);
		break;
	default:
		spin_unlock_irqrestore(&gpio_mux_lock, lock_flags);
		return -1;
	}

	__raw_writel(gius_reg, port->base + GPIO_GIUS);
	__raw_writel(gpr_reg, port->base + GPIO_GPR);

	spin_unlock_irqrestore(&gpio_mux_lock, lock_flags);
	return 0;
}

/*!
 * This function is just used to enable or disable the pull up feature .
 * @param pin   a pin number as defined in \b #iomux_pin_name_t
 * @param en    0 if disable, Non-zero enable
 * @return      0 if successful, Non-zero otherwise
 */
int gpio_set_puen(iomux_pin_name_t pin, bool en)
{
	unsigned long lock_flags;

	struct mxc_gpio_port *port;
	u32 index, gpio = IOMUX_TO_GPIO(pin);

	port = &(mxc_gpio_ports[GPIO_TO_PORT(gpio)]);
	index = GPIO_TO_INDEX(gpio);

	pr_debug("%s: Configuring output mode of PORT %c, bit %d\n",
		 __func__, GPIO_TO_PORT(gpio) + 'A', index);

	spin_lock_irqsave(&gpio_mux_lock, lock_flags);

	_gpio_set_puen(port, index, en);
	spin_unlock_irqrestore(&gpio_mux_lock, lock_flags);
	return 0;

}

/*!
 * This function is just used to request a pin and configure it.
 * @param pin	a pin number as defined in \b #iomux_pin_name_t
 * @param mode	a module as define in \b #gpio_mux_mode_t;
 * @return	0 if successful, Non-zero otherwise
 */
int gpio_request_mux(iomux_pin_name_t pin, gpio_mux_mode_t mode)
{
	int ret;
	ret = gpio_request(IOMUX_TO_GPIO(pin), NULL);
	if (ret == 0) {
		ret = gpio_config_mux(pin, mode);
		if (ret) {
			gpio_free(IOMUX_TO_GPIO(pin));
		}
	}
	return ret;
}

/*!
 * This function is just used to release a pin.
 * @param pin	a pin number as defined in \b #iomux_pin_name_t
 * @return	none
 */
void gpio_free_mux(iomux_pin_name_t pin)
{
	gpio_free(IOMUX_TO_GPIO(pin));
}
