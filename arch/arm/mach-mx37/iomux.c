/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup GPIO_MX37 Board GPIO and Muxing Setup
 * @ingroup MSL_MX37
 */
/*!
 * @file mach-mx37/iomux.c
 *
 * @brief I/O Muxing control functions
 *
 * @ingroup GPIO_MX37
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include "iomux.h"

/*!
 * IOMUX register (base) addresses
 */
#define IOMUXGPR0		(IO_ADDRESS(IOMUXC_BASE_ADDR))	/*!< General purpose 0 */
#define IOMUXGPR1		(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x004)	/*!< General purpose 1 */
#define IOMUXSW_MUX_CTL		(IO_ADDRESS(IOMUXC_BASE_ADDR) + MUX_I_START)	/*!< MUX control */
#define IOMUXSW_MUX_END		(IO_ADDRESS(IOMUXC_BASE_ADDR) + PAD_I_END)	/*!< last MUX control register */
#define IOMUXSW_PAD_CTL		(IO_ADDRESS(IOMUXC_BASE_ADDR) + PAD_I_START)	/*!< Pad control */
#define IOMUXSW_PAD_END		(IO_ADDRESS(IOMUXC_BASE_ADDR) + PAD_I_END)	/*!< last Pad control register */
#define IOMUXSW_INPUT_CTL	(IO_ADDRESS(IOMUXC_BASE_ADDR) + INPUT_CTL_START)	/*!< input select register */
#define IOMUXSW_INPUT_END	(IO_ADDRESS(IOMUXC_BASE_ADDR) + INPUT_CTL_END)	/*!< last input select register */

#define MUX_PIN_NUM_MAX		(((IOMUXSW_MUX_END - IOMUXSW_MUX_CTL) >> 2) + 1)
#define MUX_INPUT_NUM_MUX	(((IOMUXSW_INPUT_END - IOMUXSW_INPUT_CTL) >> 2) + 1)

static u8 iomux_pin_res_table[MUX_PIN_NUM_MAX];
static DEFINE_SPINLOCK(gpio_mux_lock);

/*!
 * This function is used to configure a pin through the IOMUX module.
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  config	a configuration as defined in \b #iomux_pin_cfg_t
 *
 * @return 		0 if successful; Non-zero otherwise
 */
static int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	u32 ret = 0;
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	void *mux_reg = IOMUXSW_MUX_CTL + PIN_TO_IOMUX_MUX(pin);
	u32 mux_data = 0;
	u8 *rp;

	BUG_ON((mux_reg > IOMUXSW_MUX_END) || (mux_reg < IOMUXSW_MUX_CTL));
	spin_lock(&gpio_mux_lock);

	if (config == IOMUX_CONFIG_GPIO) {
		mux_data = PIN_TO_ALT_GPIO(pin);
	} else {
		mux_data = config;
	}

	__raw_writel(mux_data, mux_reg);

	/*
	 * Log a warning if a pin changes ownership
	 */
	rp = iomux_pin_res_table + pin_index;
	if ((mux_data & *rp) && (*rp != mux_data)) {
		/*
		 * Don't call printk if we're tweaking the console uart or
		 * we'll deadlock.
		 */
		printk(KERN_ERR "iomux_config_mux: Warning: iomux pin"
		       " config changed, pin=%p, "
		       " prev=0x%x new=0x%x\n", mux_reg, *rp, mux_data);
		ret = -EINVAL;
	}
	*rp = mux_data;
	spin_unlock(&gpio_mux_lock);
	return ret;
}

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	a configuration as defined in \b #iomux_pin_cfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	int ret = iomux_config_mux(pin, config);
	int gpio = IOMUX_TO_GPIO(pin);

	if (!ret && (gpio < MXC_GPIO_IRQS) && ((config == IOMUX_CONFIG_GPIO)
		|| (config == PIN_TO_ALT_GPIO(pin)))) {
		ret |= gpio_request(gpio, NULL);
	}
	return ret;
}

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	config as defined in \b #iomux_pin_ocfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	u8 *rp = iomux_pin_res_table + pin_index;
	int gpio = IOMUX_TO_GPIO(pin);

	BUG_ON(pin_index > MUX_PIN_NUM_MAX);
	*rp = 0;
	if ((gpio < MXC_GPIO_IRQS) && ((config == IOMUX_CONFIG_GPIO)
		|| (config == PIN_TO_ALT_GPIO(pin)))) {
		gpio_free(gpio);
	}
}

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config)
{
	void *pad_reg = IOMUXSW_PAD_CTL + PIN_TO_IOMUX_PAD(pin);

	BUG_ON((pad_reg > IOMUXSW_PAD_END) || (pad_reg < IOMUXSW_PAD_CTL));
	spin_lock(&gpio_mux_lock);
	__raw_writel(config, pad_reg);
	spin_unlock(&gpio_mux_lock);
}

unsigned int mxc_iomux_get_pad(iomux_pin_name_t pin)
{
	void *pad_reg = IOMUXSW_PAD_CTL + PIN_TO_IOMUX_PAD(pin);
	return __raw_readl(pad_reg);
}

/*!
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp     one signal as defined in \b #iomux_gp_func_t
 * @param  en     \b #true to enable; \b #false to disable
 * @param  index  0 for GPR0 and 1 for GPR1
 */
void mxc_iomux_set_gpr(iomux_gp_func_t gp, bool en, u8 index)
{
	volatile u32 l;

	spin_lock(&gpio_mux_lock);
	l = __raw_readl(IOMUXGPR0 + (index << 2));
	if (en) {
		l |= gp;
	} else {
		l &= ~gp;
	}
	__raw_writel(l, IOMUXGPR0 + (index << 2));
	spin_unlock(&gpio_mux_lock);
}

/*!
 * This function configures input path.
 *
 * @param  input        index of input select register as defined in \b #iomux_input_select_t
 * @param  config       the binary value of elements defined in \b #iomux_input_config_t
 *      */
void mxc_iomux_set_input(iomux_input_select_t input, u32 config)
{
	void *reg = IOMUXSW_INPUT_CTL + (input << 2);

	BUG_ON(input >= MUX_INPUT_NUM_MUX);
	__raw_writel(config, reg);
}

EXPORT_SYMBOL(mxc_request_iomux);
EXPORT_SYMBOL(mxc_free_iomux);
EXPORT_SYMBOL(mxc_iomux_set_input);
EXPORT_SYMBOL(mxc_iomux_set_pad);
EXPORT_SYMBOL(mxc_iomux_set_gpr);
