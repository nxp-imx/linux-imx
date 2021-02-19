// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the PCM512x CODECs
 *
 * Author:	Mark Brown <broonie@kernel.org>
 *		Copyright 2014 Linaro Ltd
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>

#include "rpmsg_pcm512x.h"
#include "../fsl/fsl_rpmsg_i2s.h"

static int rpmsg_pcm512x_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct regmap *regmap;
	struct regmap_config config = rpmsg_pcm512x_regmap;

	/* msb needs to be set to enable auto-increment of addresses */
	config.read_flag_mask = 0x80;
	config.write_flag_mask = 0x80;

	regmap = devm_regmap_init_i2c(i2c, &config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

        pr_info("rpmsg_pcm512x_i2c_probe  \n");

	return rpmsg_pcm512x_probe(&i2c->dev, regmap);
}

static int rpmsg_pcm512x_i2c_remove(struct i2c_client *i2c)
{
	rpmsg_pcm512x_remove(&i2c->dev);
	return 0;
}

static const struct i2c_device_id rpmsg_pcm512x_i2c_id[] = {
	{ "pcm5121", },
	{ "pcm5122", },
	{ "pcm5141", },
	{ "pcm5142", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rpmsg_pcm512x_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id rpmsg_pcm512x_of_match[] = {
	{ .compatible = "ti,pcm5121,lpa", },
	{ .compatible = "ti,pcm5122,lpa", },
	{ .compatible = "ti,pcm5141,lpa", },
	{ .compatible = "ti,pcm5142,lpa", },
	{ }
};
MODULE_DEVICE_TABLE(of, rpmsg_pcm512x_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id rpmsg_pcm512x_acpi_match[] = {
	{ "104C5121", 0 },
	{ "104C5122", 0 },
	{ "104C5141", 0 },
	{ "104C5142", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, rpmsg_pcm512x_acpi_match);
#endif

static struct i2c_driver rpmsg_pcm512x_i2c_driver = {
	.probe 		= rpmsg_pcm512x_i2c_probe,
	.remove 	= rpmsg_pcm512x_i2c_remove,
	.id_table	= rpmsg_pcm512x_i2c_id,
	.driver		= {
		.name	=  RPMSG_CODEC_DRV_NAME_PCM512X,
		.of_match_table = of_match_ptr(rpmsg_pcm512x_of_match),
		.acpi_match_table = ACPI_PTR(rpmsg_pcm512x_acpi_match),
		.pm     = &rpmsg_pcm512x_pm_ops,
	},
};

module_i2c_driver(rpmsg_pcm512x_i2c_driver);

MODULE_DESCRIPTION("ASoC PCM512x lpa codec driver - I2C");
MODULE_AUTHOR("Tom Zheng <haidong.zheng@nxp.com>");
MODULE_LICENSE("GPL v2");
