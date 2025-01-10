// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Google, Inc.
 */

#include <linux/kconfig.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

static struct platform_device *ffa_pdev;
static struct platform_device *core_pdev;
static struct platform_device *virtio_pdev;
static struct platform_device *test_pdev;

int __init trusty_populate_init(void)
{
	int ret;

	ffa_pdev = platform_device_register_data(NULL, "trusty-ffa",
						 PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(ffa_pdev)) {
		ret = PTR_ERR(ffa_pdev);
		goto err_register_ffa_device;
	}

	/*
	 * Attach the device to the trusty-ffa driver, triggering a probe.
	 * Will fail if trusty-ffa.ko has not been loaded, which is intentional
	 * because we need the devices to be probed in the exact order here.
	 */
	ret = device_attach(&ffa_pdev->dev);
	if (!ret) /* Driver not found */
		ret = -ENODEV;
	if (ret < 0)
		goto err_attach_ffa_device;

	core_pdev = platform_device_register_data(&ffa_pdev->dev, "trusty-core",
						  PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(core_pdev)) {
		ret = PTR_ERR(core_pdev);
		goto err_register_core_device;
	}

	ret = device_attach(&core_pdev->dev);
	if (!ret)
		ret = -ENODEV;
	if (ret < 0)
		goto err_attach_core_device;


	/*
	 * The following devices do not need to be attached herer because
	 * their order does not matter, and some are even optional
	 * (trusty-test). We just need trusty-ffa and trusty-core to be
	 * the first two devices registered and probed, in that order.
	 */
	virtio_pdev = platform_device_register_data(&core_pdev->dev, "trusty-virtio",
						    PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(virtio_pdev)) {
		ret = PTR_ERR(virtio_pdev);
		goto err_register_virtio_device;
	}

	test_pdev = platform_device_register_data(&core_pdev->dev, "trusty-test",
						  PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(test_pdev)) {
		ret = PTR_ERR(test_pdev);
		goto err_register_test_device;
	}

	return 0;

err_register_test_device:
	platform_device_unregister(virtio_pdev);
err_register_virtio_device:
err_attach_core_device:
	platform_device_unregister(core_pdev);
err_register_core_device:
err_attach_ffa_device:
	platform_device_unregister(ffa_pdev);
err_register_ffa_device:
	return ret;
}

void trusty_populate_exit(void)
{
	platform_device_unregister(test_pdev);
	platform_device_unregister(virtio_pdev);
	platform_device_unregister(core_pdev);
	platform_device_unregister(ffa_pdev);
}

module_init(trusty_populate_init);
module_exit(trusty_populate_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty device populate driver");
