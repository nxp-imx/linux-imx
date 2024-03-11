// SPDX-License-Identifier: GPL-2.0-only
/*
 * i.MX SoC reset code for platforms with system manager
 *
 * Copyright 2024 NXP
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/psci.h>
#include <asm/proc-fns.h>
#include <uapi/linux/psci.h>

#define PSCI_RESET2_SYSTEM_WARM_RESET   (0U)
#define PSCI_RESET2_SYSTEM_COLD_RESET   (0x80000001U)
#define PSCI_RESET2_SYSTEM_BOARD_RESET  (0x80000002U)

#define BOARD_RESET_MESSAGE "board_reset"

/*
 * While a 64-bit OS can make calls with SMC32 calling conventions, for some
 * calls it is necessary to use SMC64 to pass or return 64-bit values.
 * For such calls PSCI_FN_NATIVE(version, name) will choose the appropriate
 * (native-width) function ID.
 */
#ifdef CONFIG_64BIT
#define PSCI_FN_NATIVE(version, name)	PSCI_##version##_FN64_##name
#else
#define PSCI_FN_NATIVE(version, name)	PSCI_##version##_FN_##name
#endif

static __always_inline unsigned long
invoke_psci_fn_smc(unsigned long function_id,
		     unsigned long arg0, unsigned long arg1,
		     unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
	return res.a0;
}

static int psci_features(u32 psci_func_id)
{
	return invoke_psci_fn_smc(PSCI_1_0_FN_PSCI_FEATURES,
			      psci_func_id, 0, 0);
}

static int imx_restart_handler(struct notifier_block *this,
				unsigned long mode, void *cmd)
{
	uint32_t reset_type;
	bool board_reset;

	board_reset = !strncmp(cmd, BOARD_RESET_MESSAGE,
			            sizeof(BOARD_RESET_MESSAGE));

	if (board_reset)
		reset_type = PSCI_RESET2_SYSTEM_BOARD_RESET;
	else if (reboot_mode == REBOOT_WARM || reboot_mode == REBOOT_SOFT)
		reset_type = PSCI_RESET2_SYSTEM_WARM_RESET;
	else
		reset_type = PSCI_RESET2_SYSTEM_COLD_RESET;

	invoke_psci_fn_smc(PSCI_1_1_FN64_SYSTEM_RESET2, reset_type, 0, 0);

	return NOTIFY_DONE;
}


static struct notifier_block imx_restart_nb = {
	.notifier_call = imx_restart_handler,
	/*
	 * Set a higher priority than default PSCI reset
	 */
	.priority = 130,
};

static int imx_reboot_probe(struct platform_device *pdev)
{
	int err = 0;

	/*
	 * register the handler if we support SYSTEM_RESET2
	 */
	err = psci_features(PSCI_FN_NATIVE(1_1, SYSTEM_RESET2));
	if (err != PSCI_RET_NOT_SUPPORTED) {
		err = register_restart_handler(&imx_restart_nb);
		if (err) {
			dev_err(&pdev->dev, "cannot register restart handler (err=%d)\n",
				err);
		}
	} else
		err = 0;

	return err;
}

static const struct of_device_id imx_reboot_of_match[] = {
	{ .compatible = "imx95,resetctrl" },
	{ .compatible = "imx8q,resetctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, imx_reboot_of_match);

static struct platform_driver imx_reboot_driver = {
	.probe = imx_reboot_probe,
	.driver = {
		.name = "imx-reboot",
		.of_match_table = imx_reboot_of_match,
	},
};
module_platform_driver(imx_reboot_driver);

MODULE_AUTHOR("Ji Luo <ji.luo@nxp.com>");
MODULE_DESCRIPTION("NXP Reset Driver for Platforms with System Manager");
MODULE_LICENSE("GPL v2");
