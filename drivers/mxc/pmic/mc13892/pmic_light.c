/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mc13892/pmic_light.c
 * @brief This is the main file of PMIC(mc13783) Light and Backlight driver.
 *
 * @ingroup PMIC_LIGHT
 */

/*
 * Includes
 */
#define DEBUG
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/pmic_light.h>
#include <linux/pmic_status.h>

#define BIT_CL_MAIN_LSH		9
#define BIT_CL_AUX_LSH		21
#define BIT_CL_KEY_LSH		9
#define BIT_CL_RED_LSH		9
#define BIT_CL_GREEN_LSH	21
#define BIT_CL_BLUE_LSH		9

#define BIT_CL_MAIN_WID		3
#define BIT_CL_AUX_WID		3
#define BIT_CL_KEY_WID		3
#define BIT_CL_RED_WID		3
#define BIT_CL_GREEN_WID	3
#define BIT_CL_BLUE_WID		3

#define BIT_DC_MAIN_LSH		3
#define BIT_DC_AUX_LSH		15
#define BIT_DC_KEY_LSH		3
#define BIT_DC_RED_LSH		3
#define BIT_DC_GREEN_LSH	15
#define BIT_DC_BLUE_LSH		3

#define BIT_DC_MAIN_WID		6
#define BIT_DC_AUX_WID		6
#define BIT_DC_KEY_WID		6
#define BIT_DC_RED_WID		6
#define BIT_DC_GREEN_WID	6
#define BIT_DC_BLUE_WID		6

#define BIT_RP_MAIN_LSH		2
#define BIT_RP_AUX_LSH		14
#define BIT_RP_KEY_LSH		2
#define BIT_RP_RED_LSH		2
#define BIT_RP_GREEN_LSH	14
#define BIT_RP_BLUE_LSH		2

#define BIT_RP_MAIN_WID		1
#define BIT_RP_AUX_WID		1
#define BIT_RP_KEY_WID		1
#define BIT_RP_RED_WID		1
#define BIT_RP_GREEN_WID	1
#define BIT_RP_BLUE_WID		1

#define BIT_HC_MAIN_LSH		1
#define BIT_HC_AUX_LSH		13
#define BIT_HC_KEY_LSH		1

#define BIT_HC_MAIN_WID		1
#define BIT_HC_AUX_WID		1
#define BIT_HC_KEY_WID		1

#define BIT_BP_RED_LSH		0
#define BIT_BP_GREEN_LSH	12
#define BIT_BP_BLUE_LSH		0

#define BIT_BP_RED_WID		2
#define BIT_BP_GREEN_WID	2
#define BIT_BP_BLUE_WID		2

int pmic_light_init_reg(void)
{
	CHECK_ERROR(pmic_write_reg(REG_LED_CTL0, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_LED_CTL1, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_LED_CTL2, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_LED_CTL3, 0, PMIC_ALL_BITS));

	return 0;
}

static int pmic_light_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
};

static int pmic_light_resume(struct platform_device *pdev)
{
	return 0;
};

PMIC_STATUS mc13892_bklit_set_hi_current(enum lit_channel channel, int mode)
{
	unsigned int mask;
	unsigned int value;
	int reg;

	switch (channel) {
	case LIT_MAIN:
		value = BITFVAL(BIT_HC_MAIN, mode);
		mask = BITFMASK(BIT_HC_MAIN);
		reg = REG_LED_CTL0;
		break;
	case LIT_AUX:
		value = BITFVAL(BIT_HC_AUX, mode);
		mask = BITFMASK(BIT_HC_AUX);
		reg = REG_LED_CTL0;
		break;
	case LIT_KEY:
		value = BITFVAL(BIT_HC_KEY, mode);
		mask = BITFMASK(BIT_HC_KEY);
		reg = REG_LED_CTL1;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	CHECK_ERROR(pmic_write_reg(reg, value, mask));
	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_get_hi_current(enum lit_channel channel, int *mode)
{
	unsigned int mask;
	int reg;

	switch (channel) {
	case LIT_MAIN:
		mask = BITFMASK(BIT_HC_MAIN);
		reg = REG_LED_CTL0;
		break;
	case LIT_AUX:
		mask = BITFMASK(BIT_HC_AUX);
		reg = REG_LED_CTL0;
		break;
	case LIT_KEY:
		mask = BITFMASK(BIT_HC_KEY);
		reg = REG_LED_CTL1;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, mode, mask));
	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_set_current(enum lit_channel channel,
				      unsigned char level)
{
	unsigned int mask;
	unsigned int value;
	int reg;

	if (level > LIT_CURR_HI_42)
		return PMIC_PARAMETER_ERROR;
	else if (level >= LIT_CURR_HI_0) {
		CHECK_ERROR(mc13892_bklit_set_hi_current(channel, 1));
		level -= LIT_CURR_HI_0;
	}

	switch (channel) {
	case LIT_MAIN:
		value = BITFVAL(BIT_CL_MAIN, level);
		mask = BITFMASK(BIT_CL_MAIN);
		reg = REG_LED_CTL0;
		break;
	case LIT_AUX:
		value = BITFVAL(BIT_CL_AUX, level);
		mask = BITFMASK(BIT_CL_AUX);
		reg = REG_LED_CTL0;
		break;
	case LIT_KEY:
		value = BITFVAL(BIT_CL_KEY, level);
		mask = BITFMASK(BIT_CL_KEY);
		reg = REG_LED_CTL1;
		break;
	case LIT_RED:
		value = BITFVAL(BIT_CL_RED, level);
		mask = BITFMASK(BIT_CL_RED);
		reg = REG_LED_CTL2;
		break;
	case LIT_GREEN:
		value = BITFVAL(BIT_CL_GREEN, level);
		mask = BITFMASK(BIT_CL_GREEN);
		reg = REG_LED_CTL2;
		break;
	case LIT_BLUE:
		value = BITFVAL(BIT_CL_BLUE, level);
		mask = BITFMASK(BIT_CL_BLUE);
		reg = REG_LED_CTL3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	CHECK_ERROR(pmic_write_reg(reg, value, mask));

	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_get_current(enum lit_channel channel,
				      unsigned char *level)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;
	int reg, mode;

	CHECK_ERROR(mc13892_bklit_get_hi_current(channel, &mode));

	switch (channel) {
	case LIT_MAIN:
		mask = BITFMASK(BIT_CL_MAIN);
		reg = REG_LED_CTL0;
		break;
	case LIT_AUX:
		mask = BITFMASK(BIT_CL_AUX);
		reg = REG_LED_CTL0;
		break;
	case LIT_KEY:
		mask = BITFMASK(BIT_CL_KEY);
		reg = REG_LED_CTL1;
		break;
	case LIT_RED:
		mask = BITFMASK(BIT_CL_RED);
		reg = REG_LED_CTL2;
		break;
	case LIT_GREEN:
		mask = BITFMASK(BIT_CL_GREEN);
		reg = REG_LED_CTL2;
		break;
	case LIT_BLUE:
		mask = BITFMASK(BIT_CL_BLUE);
		reg = REG_LED_CTL3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_value, mask));

	switch (channel) {
	case LIT_MAIN:
		*level = BITFEXT(reg_value, BIT_CL_MAIN);
		break;
	case LIT_AUX:
		*level = BITFEXT(reg_value, BIT_CL_AUX);
		break;
	case LIT_KEY:
		*level = BITFEXT(reg_value, BIT_CL_KEY);
		break;
	case LIT_RED:
		*level = BITFEXT(reg_value, BIT_CL_RED);
		break;
	case LIT_GREEN:
		*level = BITFEXT(reg_value, BIT_CL_GREEN);
		break;
	case LIT_BLUE:
		*level = BITFEXT(reg_value, BIT_CL_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	if (mode == 1)
		*level += LIT_CURR_HI_0;

	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_set_dutycycle(enum lit_channel channel,
					unsigned char dc)
{
	unsigned int mask;
	unsigned int value;
	int reg;

	switch (channel) {
	case LIT_MAIN:
		value = BITFVAL(BIT_DC_MAIN, dc);
		mask = BITFMASK(BIT_DC_MAIN);
		reg = REG_LED_CTL0;
		break;
	case LIT_AUX:
		value = BITFVAL(BIT_DC_AUX, dc);
		mask = BITFMASK(BIT_DC_AUX);
		reg = REG_LED_CTL0;
		break;
	case LIT_KEY:
		value = BITFVAL(BIT_DC_KEY, dc);
		mask = BITFMASK(BIT_DC_KEY);
		reg = REG_LED_CTL1;
		break;
	case LIT_RED:
		value = BITFVAL(BIT_DC_RED, dc);
		mask = BITFMASK(BIT_DC_RED);
		reg = REG_LED_CTL2;
		break;
	case LIT_GREEN:
		value = BITFVAL(BIT_DC_GREEN, dc);
		mask = BITFMASK(BIT_DC_GREEN);
		reg = REG_LED_CTL2;
		break;
	case LIT_BLUE:
		value = BITFVAL(BIT_DC_BLUE, dc);
		mask = BITFMASK(BIT_DC_BLUE);
		reg = REG_LED_CTL3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	CHECK_ERROR(pmic_write_reg(reg, value, mask));
	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_get_dutycycle(enum lit_channel channel,
					unsigned char *dc)
{
	unsigned int mask;
	int reg;
	unsigned int reg_value = 0;

	switch (channel) {
	case LIT_MAIN:
		mask = BITFMASK(BIT_DC_MAIN);
		reg = REG_LED_CTL0;
		break;
	case LIT_AUX:
		mask = BITFMASK(BIT_DC_AUX);
		reg = REG_LED_CTL0;
		break;
	case LIT_KEY:
		mask = BITFMASK(BIT_DC_KEY);
		reg = REG_LED_CTL1;
		break;
	case LIT_RED:
		mask = BITFMASK(BIT_DC_RED);
		reg = REG_LED_CTL2;
		break;
	case LIT_GREEN:
		mask = BITFMASK(BIT_DC_GREEN);
		reg = REG_LED_CTL2;
		break;
	case LIT_BLUE:
		mask = BITFMASK(BIT_DC_BLUE);
		reg = REG_LED_CTL3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_value, mask));
	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_set_ramp(enum lit_channel channel, int flag)
{
	unsigned int mask;
	unsigned int value;
	int reg;

	switch (channel) {
	case LIT_MAIN:
		value = BITFVAL(BIT_RP_MAIN, flag);
		mask = BITFMASK(BIT_RP_MAIN);
		reg = REG_LED_CTL0;
		break;
	case LIT_AUX:
		value = BITFVAL(BIT_RP_AUX, flag);
		mask = BITFMASK(BIT_RP_AUX);
		reg = REG_LED_CTL0;
		break;
	case LIT_KEY:
		value = BITFVAL(BIT_RP_KEY, flag);
		mask = BITFMASK(BIT_RP_KEY);
		reg = REG_LED_CTL1;
		break;
	case LIT_RED:
		value = BITFVAL(BIT_RP_RED, flag);
		mask = BITFMASK(BIT_RP_RED);
		reg = REG_LED_CTL2;
		break;
	case LIT_GREEN:
		value = BITFVAL(BIT_RP_GREEN, flag);
		mask = BITFMASK(BIT_RP_GREEN);
		reg = REG_LED_CTL2;
		break;
	case LIT_BLUE:
		value = BITFVAL(BIT_RP_BLUE, flag);
		mask = BITFMASK(BIT_RP_BLUE);
		reg = REG_LED_CTL3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	CHECK_ERROR(pmic_write_reg(reg, value, mask));
	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_get_ramp(enum lit_channel channel, int *flag)
{
	unsigned int mask;
	int reg;

	switch (channel) {
	case LIT_MAIN:
		mask = BITFMASK(BIT_RP_MAIN);
		reg = REG_LED_CTL0;
		break;
	case LIT_AUX:
		mask = BITFMASK(BIT_RP_AUX);
		reg = REG_LED_CTL0;
		break;
	case LIT_KEY:
		mask = BITFMASK(BIT_RP_KEY);
		reg = REG_LED_CTL1;
		break;
	case LIT_RED:
		mask = BITFMASK(BIT_RP_RED);
		reg = REG_LED_CTL2;
		break;
	case LIT_GREEN:
		mask = BITFMASK(BIT_RP_GREEN);
		reg = REG_LED_CTL2;
		break;
	case LIT_BLUE:
		mask = BITFMASK(BIT_RP_BLUE);
		reg = REG_LED_CTL3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, flag, mask));
	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_set_blink_p(enum lit_channel channel, int period)
{
	unsigned int mask;
	unsigned int value;
	int reg;

	switch (channel) {
	case LIT_RED:
		value = BITFVAL(BIT_BP_RED, period);
		mask = BITFMASK(BIT_BP_RED);
		reg = REG_LED_CTL2;
		break;
	case LIT_GREEN:
		value = BITFVAL(BIT_BP_GREEN, period);
		mask = BITFMASK(BIT_BP_GREEN);
		reg = REG_LED_CTL2;
		break;
	case LIT_BLUE:
		value = BITFVAL(BIT_BP_BLUE, period);
		mask = BITFMASK(BIT_BP_BLUE);
		reg = REG_LED_CTL3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	CHECK_ERROR(pmic_write_reg(reg, value, mask));
	return PMIC_SUCCESS;
}

PMIC_STATUS mc13892_bklit_get_blink_p(enum lit_channel channel, int *period)
{
	unsigned int mask;
	int reg;

	switch (channel) {
	case LIT_RED:
		mask = BITFMASK(BIT_BP_RED);
		reg = REG_LED_CTL2;
		break;
	case LIT_GREEN:
		mask = BITFMASK(BIT_BP_GREEN);
		reg = REG_LED_CTL2;
		break;
	case LIT_BLUE:
		mask = BITFMASK(BIT_BP_BLUE);
		reg = REG_LED_CTL3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, period, mask));
	return PMIC_SUCCESS;
}

EXPORT_SYMBOL(mc13892_bklit_set_current);
EXPORT_SYMBOL(mc13892_bklit_get_current);
EXPORT_SYMBOL(mc13892_bklit_set_dutycycle);
EXPORT_SYMBOL(mc13892_bklit_get_dutycycle);
EXPORT_SYMBOL(mc13892_bklit_set_ramp);
EXPORT_SYMBOL(mc13892_bklit_get_ramp);
EXPORT_SYMBOL(mc13892_bklit_set_blink_p);
EXPORT_SYMBOL(mc13892_bklit_get_blink_p);

static int pmic_light_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef DEBUG
static ssize_t lit_info(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return 0;
}

enum {
	SET_CURR = 0,
	SET_DC,
	SET_RAMP,
	SET_BP,
	SET_CH,
	LIT_CMD_MAX
};

static const char *const lit_cmd[LIT_CMD_MAX] = {
	[SET_CURR] = "cur",
	[SET_DC] = "dc",
	[SET_RAMP] = "ra",
	[SET_BP] = "bp",
	[SET_CH] = "ch"
};

static int cmd(unsigned int index, int value)
{
	static int ch = LIT_MAIN;
	int ret = 0;

	switch (index) {
	case SET_CH:
		ch = value;
		break;
	case SET_CURR:
		pr_debug("set %d cur %d\n", ch, value);
		ret = mc13892_bklit_set_current(ch, value);
		break;
	case SET_DC:
		pr_debug("set %d dc %d\n", ch, value);
		ret = mc13892_bklit_set_dutycycle(ch, value);
		break;
	case SET_RAMP:
		pr_debug("set %d ramp %d\n", ch, value);
		ret = mc13892_bklit_set_ramp(ch, value);
		break;
	case SET_BP:
		pr_debug("set %d bp %d\n", ch, value);
		ret = mc13892_bklit_set_blink_p(ch, value);
		break;
	default:
		pr_debug("error command\n");
		break;
	}

	if (ret == PMIC_SUCCESS)
		pr_debug("command exec successfully!\n");

	return 0;
}

static ssize_t lit_ctl(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int state = 0;
	const char *const *s;
	char *p, *q;
	int error;
	int len, value = 0;

	pr_debug("lit_ctl\n");

	q = NULL;
	q = memchr(buf, ' ', count);

	if (q != NULL) {
		len = q - buf;
		q += 1;
		value = simple_strtoul(q, NULL, 10);
	} else {
		p = memchr(buf, '\n', count);
		len = p ? p - buf : count;
	}

	for (s = &lit_cmd[state]; state < LIT_CMD_MAX; s++, state++) {
		if (*s && !strncmp(buf, *s, len))
			break;
	}
	if (state < LIT_CMD_MAX && *s)
		error = cmd(state, value);
	else
		error = -EINVAL;

	return count;
}

#else
static ssize_t lit_info(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return 0;
}

static ssize_t lit_ctl(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	return count;
}

#endif

static DEVICE_ATTR(lit, 0644, lit_info, lit_ctl);

static int pmic_light_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_debug("PMIC ADC start probe\n");
	ret = device_create_file(&(pdev->dev), &dev_attr_lit);
	if (ret) {
		pr_debug("Can't create device file!\n");
		return -ENODEV;
	}

	pmic_light_init_reg();

	pr_debug("PMIC Light successfully loaded\n");
	return 0;
}

static struct platform_driver pmic_light_driver_ldm = {
	.driver = {
		   .name = "pmic_light",
		   },
	.suspend = pmic_light_suspend,
	.resume = pmic_light_resume,
	.probe = pmic_light_probe,
	.remove = pmic_light_remove,
};

/*
 * Initialization and Exit
 */

static int __init pmic_light_init(void)
{
	pr_debug("PMIC Light driver loading...\n");
	return platform_driver_register(&pmic_light_driver_ldm);
}
static void __exit pmic_light_exit(void)
{
	platform_driver_unregister(&pmic_light_driver_ldm);
	pr_debug("PMIC Light driver successfully unloaded\n");
}

/*
 * Module entry points
 */

subsys_initcall(pmic_light_init);
module_exit(pmic_light_exit);

MODULE_DESCRIPTION("PMIC_LIGHT");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
