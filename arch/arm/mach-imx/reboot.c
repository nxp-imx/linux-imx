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

#define SNVS_LPGPR				0x68
#define SNVS_SIZE				(1024*16)

#define ANDROID_NORMAL_BOOT     6

void do_switch_mode(char mode)
{
	u32 reg;
	void *addr;
	phys_addr_t snvs_base_addr;
	phys_addr_t snvs_lpgpr;
	size_t snvs_size;
	if (cpu_is_imx6()) {
		snvs_base_addr = MX6_SNVS_BASE_ADDR;
	} else if (cpu_is_imx7d()) {
		snvs_base_addr = MX7D_SNVS_BASE_ADDR;
	} else if (cpu_is_imx7ulp()) {
		snvs_base_addr = MX7ULP_SNVS_BASE_ADDR;
	} else {
		pr_warn("do not support mode switch!\n");
		return;
	}
	snvs_size = SNVS_SIZE;
	snvs_lpgpr = SNVS_LPGPR;
	addr = ioremap(snvs_base_addr, snvs_size);
	if (!addr) {
		pr_warn("SNVS ioremap failed!\n");
		return;
	}
	reg = __raw_readl(addr + snvs_lpgpr);
	reg |= (1 <<  mode);
	__raw_writel(reg, (addr + snvs_lpgpr));
	iounmap(addr);
}


void do_switch_normal(void)
{
	do_switch_mode(ANDROID_NORMAL_BOOT);
}


static void restart_special_mode(const char *cmd)
{
	if (cmd && strcmp(cmd, "") == 0)
		do_switch_normal();
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
