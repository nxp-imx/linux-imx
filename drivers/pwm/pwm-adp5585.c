// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * PWM driver for Analog Devices ADP5585 MFD
 *
 * Copyright 2022 NXP
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/adp5585.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/time.h>

#define ADP5585_PWM_CHAN_NUM		1
#define ADP5585_PWM_FASTEST_PERIOD_NS	2000
#define ADP5585_PWM_SLOWEST_PERIOD_NS	131070000

struct adp5585_pwm_chip {
	struct device *parent;
	struct pwm_chip chip;
	struct mutex lock;
	u8 pin_config_val;
};

static inline struct adp5585_pwm_chip *
to_adp5585_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct adp5585_pwm_chip, chip);
}

static int adp5585_pwm_reg_read(struct adp5585_pwm_chip *adp5585_pwm, u8 reg, u8 *val)
{
	struct adp5585_dev *adp5585;
	adp5585 = dev_get_drvdata(adp5585_pwm->parent);

	return adp5585->read_reg(adp5585, reg, val);
}

static int adp5585_pwm_reg_write(struct adp5585_pwm_chip *adp5585_pwm, u8 reg, u8 val)
{
	struct adp5585_dev *adp5585;
	adp5585 = dev_get_drvdata(adp5585_pwm->parent);

	return adp5585->write_reg(adp5585, reg, val);
}

static int pwm_adp5585_get_state(struct pwm_chip *chip,
				  struct pwm_device *pwm,
				  struct pwm_state *state)
{
	struct adp5585_pwm_chip *adp5585_pwm = to_adp5585_pwm_chip(chip);
	u32 on, off;
	u8 temp;

	/* get period */
	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_PWM_OFFT_LOW, &temp);
	off = temp;
	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_PWM_OFFT_HIGH, &temp);
	off |= temp << 8;
	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_PWM_ONT_LOW, &temp);
	on = temp;
	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_PWM_ONT_HIGH, &temp);
	on |= temp << 8;
	state->period = (on + off) * NSEC_PER_USEC;

	/* get duty cycle */
	state->duty_cycle = on;

	/* get polarity */
	state->polarity = PWM_POLARITY_NORMAL;

	/* get channel status */
	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_PWM_CFG, &temp);
	state->enabled = temp & ADP5585_PWM_CFG_EN;

	return 0;
}

static int pwm_adp5585_apply(struct pwm_chip *chip,
			     struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	struct adp5585_pwm_chip *adp5585_pwm = to_adp5585_pwm_chip(chip);
	u8 enabled;
	int ret;
	u32 on, off;

	if (state->period > ADP5585_PWM_SLOWEST_PERIOD_NS ||
	    state->period < ADP5585_PWM_FASTEST_PERIOD_NS)
		return -EINVAL;

	mutex_lock(&adp5585_pwm->lock);
	/* set on/off cycle*/
	on = DIV_ROUND_CLOSEST_ULL(state->duty_cycle, NSEC_PER_USEC);
	off = DIV_ROUND_CLOSEST_ULL((state->period - state->duty_cycle),
				   NSEC_PER_USEC);
	ret = adp5585_pwm_reg_write(adp5585_pwm, ADP5585_PWM_OFFT_LOW,
				    off & ADP5585_REG_MASK);
	if (ret)
		goto ERROR_PATH;
	ret = adp5585_pwm_reg_write(adp5585_pwm, ADP5585_PWM_OFFT_HIGH,
				    (off >> 8) & ADP5585_REG_MASK);
	if (ret)
		goto ERROR_PATH;
	ret = adp5585_pwm_reg_write(adp5585_pwm, ADP5585_PWM_ONT_LOW,
				    on & ADP5585_REG_MASK);
	if (ret)
		goto ERROR_PATH;
	ret = adp5585_pwm_reg_write(adp5585_pwm, ADP5585_PWM_ONT_HIGH,
				    (on >> 8) & ADP5585_REG_MASK);
	if (ret)
		goto ERROR_PATH;

	/* enable PWM and set to continuous PWM mode*/
	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_PWM_CFG, &enabled);
	if (state->enabled)
		ret = adp5585_pwm_reg_write(adp5585_pwm, ADP5585_PWM_CFG,
					ADP5585_PWM_CFG_EN);
	else
		ret = adp5585_pwm_reg_write(adp5585_pwm, ADP5585_PWM_CFG, 0);

ERROR_PATH:
	mutex_unlock(&adp5585_pwm->lock);

	return ret;
}

static int pwm_adp5585_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct adp5585_pwm_chip *adp5585_pwm = to_adp5585_pwm_chip(chip);
	u8 reg_cfg;
	int ret;

	mutex_lock(&adp5585_pwm->lock);
	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_PIN_CONFIG_C,
			     &adp5585_pwm->pin_config_val);
	reg_cfg = adp5585_pwm->pin_config_val & ~ADP5585_PIN_CONFIG_R3_MASK;
	reg_cfg |= ADP5585_PIN_CONFIG_R3_PWM;
	ret = adp5585_pwm_reg_write(adp5585_pwm, ADP5585_PIN_CONFIG_C,
				    reg_cfg);

	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_GENERAL_CFG,
			     &adp5585_pwm->pin_config_val);
	reg_cfg |= ADP5585_GENERAL_CFG_OSC_EN;
	ret = adp5585_pwm_reg_write(adp5585_pwm, ADP5585_GENERAL_CFG, reg_cfg);
	mutex_unlock(&adp5585_pwm->lock);

	return ret;
}

static void pwm_adp5585_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct adp5585_pwm_chip *adp5585_pwm = to_adp5585_pwm_chip(chip);
	u8 reg_cfg;

	mutex_lock(&adp5585_pwm->lock);
	adp5585_pwm_reg_read(adp5585_pwm, ADP5585_PIN_CONFIG_C, &reg_cfg);
	reg_cfg &= ~ADP5585_PIN_CONFIG_R3_MASK;
	reg_cfg |= adp5585_pwm->pin_config_val & ADP5585_PIN_CONFIG_R3_MASK;
	adp5585_pwm_reg_write(adp5585_pwm, ADP5585_PIN_CONFIG_C, reg_cfg);
	mutex_unlock(&adp5585_pwm->lock);
}

static const struct pwm_ops adp5585_pwm_ops = {
	.request = pwm_adp5585_request,
	.free = pwm_adp5585_free,
	.get_state = pwm_adp5585_get_state,
	.apply = pwm_adp5585_apply,
	.owner = THIS_MODULE,
};

static int adp5585_pwm_probe(struct platform_device *pdev)
{
	struct adp5585_pwm_chip *adp5585_pwm;
	int ret;

	adp5585_pwm = devm_kzalloc(&pdev->dev, sizeof(*adp5585_pwm), GFP_KERNEL);
	if (!adp5585_pwm)
		return -ENOMEM;

	adp5585_pwm->parent = pdev->dev.parent;
	platform_set_drvdata(pdev, adp5585_pwm);

	adp5585_pwm->chip.dev = &pdev->dev;
	adp5585_pwm->chip.ops = &adp5585_pwm_ops;
	adp5585_pwm->chip.npwm = ADP5585_PWM_CHAN_NUM;

	mutex_init(&adp5585_pwm->lock);

	ret = pwmchip_add(&adp5585_pwm->chip);
	if (ret)
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);

	return ret;
}

static int adp5585_pwm_remove(struct platform_device *pdev)
{
	struct adp5585_pwm_chip *adp5585_pwm = platform_get_drvdata(pdev);

	pwmchip_remove(&adp5585_pwm->chip);

	return 0;
}

static const struct of_device_id adp5585_pwm_of_match[] = {
	{.compatible = "adp5585-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adp5585_of_match);

static struct platform_driver adp5585_pwm_driver = {
	.driver	= {
		.name	= "adp5585-pwm",
		.of_match_table = adp5585_pwm_of_match,
	},
	.probe		= adp5585_pwm_probe,
	.remove		= adp5585_pwm_remove,
};
module_platform_driver(adp5585_pwm_driver);

MODULE_AUTHOR("Xiaoning Wang <xiaoning.wang@nxp.com>");
MODULE_DESCRIPTION("ADP5585 PWM Driver");
MODULE_LICENSE("GPL v2");
