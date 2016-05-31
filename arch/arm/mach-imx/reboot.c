/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 *
*/
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/module.h>
#include <linux/reboot.h>

#include "hardware.h"

#define MX6_AIPS1_ARB_BASE_ADDR			0x02000000
#define MX6_ATZ1_BASE_ADDR		MX6_AIPS1_ARB_BASE_ADDR
#define MX6_AIPS1_OFF_BASE_ADDR		(MX6_ATZ1_BASE_ADDR + 0x80000)
#define MX6_SNVS_BASE_ADDR		(MX6_AIPS1_OFF_BASE_ADDR + 0x4C000)
#define MX6_SNVS_LPGPR				0x68
#define MX6_SNVS_SIZE				(1024*16)
#define MX7_AIPS1_ARB_BASE_ADDR			0x30000000
#define MX7_ATZ1_BASE_ADDR		MX7_AIPS1_ARB_BASE_ADDR
#define MX7_AIPS1_OFF_BASE_ADDR		(MX7_ATZ1_BASE_ADDR + 0x200000)
#define MX7_SNVS_BASE_ADDR		(MX7_AIPS1_OFF_BASE_ADDR + 0x170000)
#define MX7_SNVS_LPGPR				0x68
#define MX7_SNVS_SIZE				(1024*16)

#define ANDROID_RECOVERY_BOOT  (1 << 7)
#define ANDROID_FASTBOOT_BOOT  (1 << 8)

void do_switch_mode(bool mode)
{
	u32 reg;
	void *addr;
	phys_addr_t snvs_base_addr;
	phys_addr_t snvs_lpgpr;
	size_t snvs_size;
	if (cpu_is_imx6()) {
		snvs_base_addr = MX6_SNVS_BASE_ADDR;
		snvs_size = MX6_SNVS_SIZE;
		snvs_lpgpr = MX6_SNVS_LPGPR;
	} else {
		snvs_base_addr = MX7_SNVS_BASE_ADDR;
		snvs_size = MX7_SNVS_SIZE;
		snvs_lpgpr = MX7_SNVS_LPGPR;
	}
	addr = ioremap(snvs_base_addr, snvs_size);
	if (!addr) {
		pr_warn("SNVS ioremap failed!\n");
		return;
	}
	reg = __raw_readl(addr + snvs_lpgpr);
	if (mode)
		reg |= ANDROID_RECOVERY_BOOT;
	else
		reg |= ANDROID_FASTBOOT_BOOT;
	__raw_writel(reg, (addr + snvs_lpgpr));
	iounmap(addr);
}

void do_switch_recovery(void)
{
	do_switch_mode(1);
}

void do_switch_fastboot(void)
{
	do_switch_mode(0);
}

static void restart_special_mode(const char *cmd)
{
	if (cmd && strcmp(cmd, "recovery") == 0)
		do_switch_recovery();
	else if (cmd && strcmp(cmd, "bootloader") == 0)
		do_switch_fastboot();
}

static int imx_reboot_notifier_call(
		struct notifier_block *notifier,
		unsigned long what, void *data)
{
	int ret = NOTIFY_DONE;
	char *cmd = (char *)data;
	restart_special_mode(cmd);

	return ret;
}


static struct notifier_block imx_reboot_notifier = {
	.notifier_call = imx_reboot_notifier_call,
};

static int __init reboot_init(void)
{
	if (register_reboot_notifier(&imx_reboot_notifier)) {
		pr_err("unable to register reboot notifier\n");
		return -1;
	}
	return 0;
}
module_init(reboot_init);

static void __exit reboot_exit(void)
{
	unregister_reboot_notifier(&imx_reboot_notifier);
}
module_exit(reboot_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("bootloader communication module");
MODULE_LICENSE("GPL v2");
