// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtual power-domain for protected guest assigned devices.
 *
 * Author: Vincent Donnefort <vdonnefort@google.com>
 * Copyright (C) 2026 Google LLC
 */

#include <linux/arm-smccc.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>

#include <asm/hypervisor.h>

#include "pkvm-guest.h"

struct pkvm_device_pd {
	struct generic_pm_domain genpd;
#define INVALID_MMIO U64_MAX
	u64 mmio;
};

static int pkvm_device_pm_attach(struct generic_pm_domain *genpd, struct device *dev)
{
	struct pkvm_device_pd *pd = container_of(genpd, struct pkvm_device_pd, genpd);
	struct resource *res;

	if (!dev_is_platform(dev)) {
		dev_err(&genpd->dev, "%s is not a platform device\n", dev_name(dev));
		return -EINVAL;
	}

	res = platform_get_resource(to_platform_device(dev), IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&genpd->dev, "Couldn't find MMIO address for %s", dev_name(dev));
		return -EINVAL;
	}

	if (pd->mmio != INVALID_MMIO && pd->mmio != res->start) {
		dev_err(&genpd->dev, "Only a single device per pkvm,device-power domain\n");
		return -EBUSY;
	}

	pd->mmio = res->start;

	return 0;
}

static int pkvm_device_pm_toggle(struct pkvm_device_pd *pd, bool on)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(ARM_SMCCC_VENDOR_HYP_KVM_DEV_REQ_PWR_FUNC_ID,
			     on ? KVM_DEV_REQ_PWR_ON : KVM_DEV_REQ_PWR_OFF, pd->mmio, &res);

	return res.a0 == SMCCC_RET_SUCCESS ? 0 : -EPERM;
}

static int pkvm_device_pm_on(struct generic_pm_domain *genpd)
{
	return pkvm_device_pm_toggle(container_of(genpd, struct pkvm_device_pd, genpd), true);
}

static int pkvm_device_pm_off(struct generic_pm_domain *genpd)
{
	return pkvm_device_pm_toggle(container_of(genpd, struct pkvm_device_pd, genpd), false);
}

static int pkvm_device_pm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pkvm_device_pd *pd;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->mmio = INVALID_MMIO;
	pd->genpd.name = pdev->name;
	pd->genpd.attach_dev = pkvm_device_pm_attach;
	pd->genpd.power_on = pkvm_device_pm_on;
	pd->genpd.power_off = pkvm_device_pm_off;

	/* VM-assigned devices are powered-on by default */
	pm_genpd_init(&pd->genpd, NULL, false);

	return of_genpd_add_provider_simple(dev->of_node, &pd->genpd);
}

static const struct of_device_id pkvm_device_pm_match[] = {
	{ .compatible = "pkvm,device-power" },
	{}
};

static struct platform_driver pkvm_device_pm_driver = {
	.probe	= pkvm_device_pm_probe,
	.driver	= {
		.name = "pkvm-device-power",
		.of_match_table = pkvm_device_pm_match,
	},
};

int pkvm_device_pm_init(void)
{
	return platform_driver_register(&pkvm_device_pm_driver);
}
