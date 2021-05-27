// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <soc/imx/src.h>

struct imx8_soc_data {
	char *name;
	u32 (*soc_revision)(void);
};

static u64 soc_uid;

static ssize_t soc_uid_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%016llX\n", soc_uid);
}

static DEVICE_ATTR_RO(soc_uid);

static void __init imx8ulp_soc_uid(void)
{

}

static u32 __init imx8ulp_soc_revision(void)
{
	u32 rev;

	rev = 0x01;
	imx8ulp_soc_uid();

	return rev;
}

static const struct imx8_soc_data imx8ulp_soc_data = {
	.name = "i.MX8ULP",
	.soc_revision = imx8ulp_soc_revision,
};

static const struct of_device_id imx8_soc_match[] = {
	{ .compatible = "fsl,imx8ulp", .data = &imx8ulp_soc_data, },
	{ }
};

#define imx8_revision(soc_rev) \
	soc_rev ? \
	kasprintf(GFP_KERNEL, "%d.%d", (soc_rev >> 4) & 0xf,  soc_rev & 0xf) : \
	"unknown"

static int __init imx8_soc_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	const struct of_device_id *id;
	u32 soc_rev = 0;
	const struct imx8_soc_data *data;
	int ret;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENOMEM;

	soc_dev_attr->family = "Freescale i.MX";

	ret = of_property_read_string(of_root, "model", &soc_dev_attr->machine);
	if (ret)
		goto free_soc;

	id = of_match_node(imx8_soc_match, of_root);
	if (!id) {
		ret = -ENODEV;
		goto free_soc;
	}

	data = id->data;
	if (data) {
		soc_dev_attr->soc_id = data->name;
		if (data->soc_revision)
			soc_rev = data->soc_revision();
	}

	soc_dev_attr->revision = imx8_revision(soc_rev);
	if (!soc_dev_attr->revision) {
		ret = -ENOMEM;
		goto free_soc;
	}

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		ret = PTR_ERR(soc_dev);
		goto free_rev;
	}

	ret = device_create_file(soc_device_to_device(soc_dev),
				 &dev_attr_soc_uid);
	if (ret)
		goto free_rev;

	return 0;

free_rev:
	if (strcmp(soc_dev_attr->revision, "unknown"))
		kfree(soc_dev_attr->revision);
free_soc:
	kfree(soc_dev_attr);
	return ret;
}
device_initcall(imx8_soc_init);
