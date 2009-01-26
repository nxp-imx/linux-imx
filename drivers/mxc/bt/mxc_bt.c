/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_bt.c
 *
 * @brief MXC Thirty party Bluetooth
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/regulator/regulator.h>

static struct regulator *bt_vdd;
static struct regulator *bt_vdd_parent;
static struct regulator *bt_vusb;
static struct regulator *bt_vusb_parent;

/*!
  * This function poweron the bluetooth hardware module
  *
  * @param      pdev    Pointer to the platform device
  * @return              0 on success, -1 otherwise.
  */
static int mxc_bt_probe(struct platform_device *pdev)
{
	struct mxc_bt_platform_data *platform_data;
	platform_data = (struct mxc_bt_platform_data *)pdev->dev.platform_data;
	if (platform_data->bt_vdd) {
		bt_vdd = regulator_get(&pdev->dev, platform_data->bt_vdd);
		regulator_enable(bt_vdd);
	}
	if (platform_data->bt_vdd_parent) {
		bt_vdd_parent =
		    regulator_get(&pdev->dev, platform_data->bt_vdd_parent);
		regulator_enable(bt_vdd_parent);
	}
	if (platform_data->bt_vusb) {
		bt_vusb = regulator_get(&pdev->dev, platform_data->bt_vusb);
		regulator_enable(bt_vusb);
	}
	if (platform_data->bt_vusb_parent) {
		bt_vusb_parent =
		    regulator_get(&pdev->dev, platform_data->bt_vusb_parent);
		regulator_enable(bt_vusb_parent);
	}

	if (platform_data->bt_reset != NULL)
		platform_data->bt_reset();
	return 0;

}

/*!
  * This function poweroff the bluetooth hardware module
  *
  * @param      pdev    Pointer to the platform device
  * @return              0 on success, -1 otherwise.
  */
static int mxc_bt_remove(struct platform_device *pdev)
{
	struct mxc_bt_platform_data *platform_data;
	platform_data = (struct mxc_bt_platform_data *)pdev->dev.platform_data;
	if (bt_vdd) {
		regulator_disable(bt_vdd);
		regulator_put(bt_vdd, &pdev->dev);
	}
	if (bt_vdd_parent) {
		regulator_disable(bt_vdd_parent);
		regulator_put(bt_vdd_parent, &pdev->dev);
	}
	if (bt_vusb) {
		regulator_disable(bt_vusb);
		regulator_put(bt_vusb, &pdev->dev);
	}
	if (bt_vusb_parent) {
		regulator_disable(bt_vusb_parent);
		regulator_put(bt_vusb_parent, &pdev->dev);
	}
	return 0;

}

static struct platform_driver bluetooth_driver = {
	.driver = {
		   .name = "mxc_bt",
		   },
	.probe = mxc_bt_probe,
	.remove = mxc_bt_remove,
};

/*!
 * Register bluetooth driver module
 *
 */
static __init int bluetooth_init(void)
{
	return platform_driver_register(&bluetooth_driver);
}

/*!
 * Exit and free the bluetooth module
 *
 */
static void __exit bluetooth_exit(void)
{
	platform_driver_unregister(&bluetooth_driver);
}

module_init(bluetooth_init);
module_exit(bluetooth_exit);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Thirty party Bluetooth");
MODULE_LICENSE("GPL");
