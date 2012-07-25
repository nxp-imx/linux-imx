/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>
#include <linux/fsl_devices.h>

#ifdef CONFIG_SOC_IMX50
#define BANK(a, b, c, d, e, f, g, h)   \
	{\
	("HW_OCOTP_"#a), ("HW_OCOTP_"#b), ("HW_OCOTP_"#c), ("HW_OCOTP_"#d), \
	("HW_OCOTP_"#e), ("HW_OCOTP_"#f), ("HW_OCOTP_"#g), ("HW_OCOTP_"#h) \
	}

#define BANKS          (5)
#define BANK_ITEMS     (8)
static const char *bank_reg_desc[BANKS][BANK_ITEMS] = {
	BANK(LOCK, CFG0, CFG1, CFG2, CFG3, CFG4, CFG5, CFG6),
	BANK(MEM0, MEM1, MEM2, MEM3, MEM4, MEM5, GP0, GP1),
	BANK(SCC0, SCC1, SCC2, SCC3, SCC4, SCC5, SCC6, SCC7),
	BANK(SRK0, SRK1, SRK2, SRK3, SRK4, SRK5, SRK6, SRK7),
	BANK(SJC0, SJC1, MAC0, MAC1, HWCAP0, HWCAP1, HWCAP2, SWCAP),
};

static const struct mxc_otp_platform_data imx50_otp_platform_data = {
	.fuse_name = (char **)bank_reg_desc,
	.clock_name = "ocotp_ctrl_ahb_clk",
	.fuse_num = BANKS * BANK_ITEMS,
};

const struct imx_otp_data imx50_otp_data = {
	.iobase = MX50_OCOTP_CTRL_BASE_ADDR,
	.pdata = &imx50_otp_platform_data,
};
#undef BANK
#undef BANKS
#undef BANK_ITEMS
#endif /* ifdef CONFIG_SOC_IMX50 */

#ifdef CONFIG_SOC_IMX6Q
#define BANK(a, b, c, d, e, f, g, h)   \
	{\
	("HW_OCOTP_"#a), ("HW_OCOTP_"#b), ("HW_OCOTP_"#c), ("HW_OCOTP_"#d), \
	("HW_OCOTP_"#e), ("HW_OCOTP_"#f), ("HW_OCOTP_"#g), ("HW_OCOTP_"#h) \
	}

#define BANKS          (16)
#define BANK_ITEMS     (8)
static const char *bank_reg_desc[BANKS][BANK_ITEMS] = {
	BANK(LOCK, CFG0, CFG1, CFG2, CFG3, CFG4, CFG5, CFG6),
	BANK(MEM0, MEM1, MEM2, MEM3, MEM4, ANA0, ANA1, ANA2),
	BANK(OTPMK0, OTPMK1, OTPMK2, OTPMK3, OTPMK4, OTPMK5, OTPMK6, OTPMK7),
	BANK(SRK0, SRK1, SRK2, SRK3, SRK4, SRK5, SRK6, SRK7),
	BANK(RESP0, HSJC_RESP1, MAC0, MAC1, HDCP_KSV0, HDCP_KSV1, GP1, GP2),
	BANK(DTCP_KEY0,  DTCP_KEY1,  DTCP_KEY2,  DTCP_KEY3,  DTCP_KEY4,  MISC_CONF,  FIELD_RETURN, SRK_REVOKE),
	BANK(HDCP_KEY0,  HDCP_KEY1,  HDCP_KEY2,  HDCP_KEY3,  HDCP_KEY4,  HDCP_KEY5,  HDCP_KEY6,  HDCP_KEY7),
	BANK(HDCP_KEY8,  HDCP_KEY9,  HDCP_KEY10, HDCP_KEY11, HDCP_KEY12, HDCP_KEY13, HDCP_KEY14, HDCP_KEY15),
	BANK(HDCP_KEY16, HDCP_KEY17, HDCP_KEY18, HDCP_KEY19, HDCP_KEY20, HDCP_KEY21, HDCP_KEY22, HDCP_KEY23),
	BANK(HDCP_KEY24, HDCP_KEY25, HDCP_KEY26, HDCP_KEY27, HDCP_KEY28, HDCP_KEY29, HDCP_KEY30, HDCP_KEY31),
	BANK(HDCP_KEY32, HDCP_KEY33, HDCP_KEY34, HDCP_KEY35, HDCP_KEY36, HDCP_KEY37, HDCP_KEY38, HDCP_KEY39),
	BANK(HDCP_KEY40, HDCP_KEY41, HDCP_KEY42, HDCP_KEY43, HDCP_KEY44, HDCP_KEY45, HDCP_KEY46, HDCP_KEY47),
	BANK(HDCP_KEY48, HDCP_KEY49, HDCP_KEY50, HDCP_KEY51, HDCP_KEY52, HDCP_KEY53, HDCP_KEY54, HDCP_KEY55),
	BANK(HDCP_KEY56, HDCP_KEY57, HDCP_KEY58, HDCP_KEY59, HDCP_KEY60, HDCP_KEY61, HDCP_KEY62, HDCP_KEY63),
	BANK(HDCP_KEY64, HDCP_KEY65, HDCP_KEY66, HDCP_KEY67, HDCP_KEY68, HDCP_KEY69, HDCP_KEY70, HDCP_KEY71),
	BANK(CRC0, CRC1, CRC2, CRC3, CRC4, CRC5, CRC6, CRC7),
};

static const struct mxc_otp_platform_data imx6q_otp_platform_data = {
	.fuse_name = (char **)bank_reg_desc,
	.clock_name = "iim_clk",
	.fuse_num = BANKS * BANK_ITEMS,
};

const struct imx_otp_data imx6q_otp_data = {
	.iobase = OCOTP_BASE_ADDR,
	.pdata = (struct mxc_otp_platform_data *)&imx6q_otp_platform_data,
};
#undef BANK
#undef BANKS
#undef BANK_ITEMS
#endif /* ifdef CONFIG_SOC_IMX6Q */


struct platform_device *__init imx_add_otp(
		const struct imx_otp_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_16 - 1,
			.flags = IORESOURCE_MEM,
		}
	};

	return imx_add_platform_device("imx-ocotp", 0,
			res, ARRAY_SIZE(res), data->pdata,
			sizeof(struct mxc_otp_platform_data));
}

