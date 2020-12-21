// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/platform_device.h>
#include <linux/arm-smccc.h>
#include <linux/of.h>

#include <soc/imx/src.h>

#define REV_B1				0x21

#define IMX8MQ_SW_INFO_B1		0x40
#define IMX8MQ_SW_MAGIC_B1		0xff0055aa

#define IMX_SIP_GET_SOC_INFO		0xc2000006

#define IMX_SIP_NOC			0xc2000008
#define IMX_SIP_NOC_LCDIF		0x0
#define IMX_SIP_NOC_PRIORITY		0x1
#define NOC_GPU_PRIORITY		0x10
#define NOC_DCSS_PRIORITY		0x11
#define NOC_VPU_PRIORITY		0x12
#define NOC_CPU_PRIORITY		0x13
#define NOC_MIX_PRIORITY		0x14

#define OCOTP_UID_LOW			0x410
#define OCOTP_UID_HIGH			0x420

#define OCOTP_IMX8MP_ANA_TRIM_1		0xd70
#define OCOTP_IMX8MP_LDO_MASK		0x1f
#define OCOTP_IMX8MP_LDO0_SHIFT		16
#define OCOTP_IMX8MP_LDO1_SHIFT		24
#define OCOTP_IMX8MP_ANA_TRIM_2		0xd80
#define OCOTP_IMX8MP_LDO2_SHIFT		0

#define OCOTP_IMX8MP_LDO_NUM		3

#define IMX8MP_OCOTP_UID_OFFSET		0x10

/* Same as ANADIG_DIGPROG_IMX7D */
#define ANADIG_DIGPROG_IMX8MM	0x800

struct imx8_soc_data {
	char *name;
	u32 (*soc_revision)(void);
};

static u64 soc_uid;
static int ldo_trim[3] = { -1, -1, -1 };

static ssize_t soc_uid_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%016llX\n", soc_uid);
}

static DEVICE_ATTR_RO(soc_uid);

static u32 imx8mq_soc_revision_from_atf(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(IMX_SIP_GET_SOC_INFO, 0, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0 == SMCCC_RET_NOT_SUPPORTED)
		return 0;
	else
		return res.a0 & 0xff;
}

static u32 __init imx8mq_soc_revision(void)
{
	struct device_node *np;
	void __iomem *ocotp_base;
	u32 magic;
	u32 rev = 0;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mq-ocotp");
	if (!np)
		goto out;

	ocotp_base = of_iomap(np, 0);
	WARN_ON(!ocotp_base);

	/*
	 * SOC revision on older imx8mq is not available in fuses so query
	 * the value from ATF instead.
	 */
	rev = imx8mq_soc_revision_from_atf();
	if (!rev) {
		magic = readl_relaxed(ocotp_base + IMX8MQ_SW_INFO_B1);
		if (magic == IMX8MQ_SW_MAGIC_B1)
			rev = REV_B1;
	}

	soc_uid = readl_relaxed(ocotp_base + OCOTP_UID_HIGH);
	soc_uid <<= 32;
	soc_uid |= readl_relaxed(ocotp_base + OCOTP_UID_LOW);

	iounmap(ocotp_base);

out:
	of_node_put(np);
	return rev;
}

static void __init imx8mm_soc_uid(void)
{
	void __iomem *ocotp_base;
	struct device_node *np;
	u32 offset = of_machine_is_compatible("fsl,imx8mp") ?
		     IMX8MP_OCOTP_UID_OFFSET : 0;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mm-ocotp");
	if (!np)
		return;

	ocotp_base = of_iomap(np, 0);
	WARN_ON(!ocotp_base);

	soc_uid = readl_relaxed(ocotp_base + OCOTP_UID_HIGH + offset);
	soc_uid <<= 32;
	soc_uid |= readl_relaxed(ocotp_base + OCOTP_UID_LOW + offset);

	iounmap(ocotp_base);
	of_node_put(np);
}

static void __init imx8mp_read_ldo_trim(void)
{
	void __iomem *ocotp_base;
	struct device_node *np;
	u32 fuse;

	if (!of_machine_is_compatible("fsl,imx8mp"))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mp-ocotp");
	if (!np)
		goto out;

	ocotp_base = of_iomap(np, 0);
	if (!ocotp_base){
		WARN_ON(!ocotp_base);
		goto out;
	}

	fuse = readl_relaxed(ocotp_base + OCOTP_IMX8MP_ANA_TRIM_2);
	ldo_trim[2] = fuse & OCOTP_IMX8MP_LDO_MASK;

	fuse = readl_relaxed(ocotp_base + OCOTP_IMX8MP_ANA_TRIM_1);
	ldo_trim[1] = (fuse >> OCOTP_IMX8MP_LDO1_SHIFT) &
		OCOTP_IMX8MP_LDO_MASK;
	ldo_trim[0] = (fuse >> OCOTP_IMX8MP_LDO0_SHIFT) &
		OCOTP_IMX8MP_LDO_MASK;

	iounmap(ocotp_base);

out:
	of_node_put(np);
}

static u32 __init imx8mm_soc_revision(void)
{
	struct device_node *np;
	void __iomem *anatop_base;
	u32 rev;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mm-anatop");
	if (!np)
		return 0;

	anatop_base = of_iomap(np, 0);
	WARN_ON(!anatop_base);

	rev = readl_relaxed(anatop_base + ANADIG_DIGPROG_IMX8MM);

	iounmap(anatop_base);
	of_node_put(np);

	imx8mm_soc_uid();

	imx8mp_read_ldo_trim();

	return rev;
}

static const struct imx8_soc_data imx8mq_soc_data = {
	.name = "i.MX8MQ",
	.soc_revision = imx8mq_soc_revision,
};

static const struct imx8_soc_data imx8mm_soc_data = {
	.name = "i.MX8MM",
	.soc_revision = imx8mm_soc_revision,
};

static const struct imx8_soc_data imx8mn_soc_data = {
	.name = "i.MX8MN",
	.soc_revision = imx8mm_soc_revision,
};

static const struct imx8_soc_data imx8mp_soc_data = {
	.name = "i.MX8MP",
	.soc_revision = imx8mm_soc_revision,
};

static const struct of_device_id imx8_soc_match[] = {
	{ .compatible = "fsl,imx8mq", .data = &imx8mq_soc_data, },
	{ .compatible = "fsl,imx8mm", .data = &imx8mm_soc_data, },
	{ .compatible = "fsl,imx8mn", .data = &imx8mn_soc_data, },
	{ .compatible = "fsl,imx8mp", .data = &imx8mp_soc_data, },
	{ }
};

#define imx8_revision(soc_rev) \
	soc_rev ? \
	kasprintf(GFP_KERNEL, "%d.%d", (soc_rev >> 4) & 0xf,  soc_rev & 0xf) : \
	"unknown"

static void __init imx8mq_noc_init(void)
{
	struct arm_smccc_res res;

	pr_info("Config NOC for VPU and CPU\n");

	arm_smccc_smc(IMX_SIP_NOC, IMX_SIP_NOC_PRIORITY, NOC_CPU_PRIORITY,
			0x80000300, 0, 0, 0, 0, &res);
	if (res.a0)
		pr_err("Config NOC for CPU fail!\n");

	arm_smccc_smc(IMX_SIP_NOC, IMX_SIP_NOC_PRIORITY, NOC_VPU_PRIORITY,
			0x80000300, 0, 0, 0, 0, &res);
	if (res.a0)
		pr_err("Config NOC for VPU fail!\n");
}

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

	if (IS_ENABLED(CONFIG_ARM_IMX_CPUFREQ_DT))
		platform_device_register_simple("imx-cpufreq-dt", -1, NULL, 0);

	if (of_machine_is_compatible("fsl,imx8mq"))
		imx8mq_noc_init();

	return 0;

free_rev:
	if (strcmp(soc_dev_attr->revision, "unknown"))
		kfree(soc_dev_attr->revision);
free_soc:
	kfree(soc_dev_attr);
	return ret;
}
device_initcall(imx8_soc_init);

#define FSL_SIP_SRC                    0xc2000005
#define FSL_SIP_SRC_M4_START           0x00
#define FSL_SIP_SRC_M4_STARTED         0x01

/* To indicate M4 enabled or not on i.MX8MQ */
static bool m4_is_enabled;
bool imx_src_is_m4_enabled(void)
{
	return m4_is_enabled;
}
EXPORT_SYMBOL_GPL(imx_src_is_m4_enabled);

int check_m4_enabled(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(FSL_SIP_SRC, FSL_SIP_SRC_M4_STARTED, 0,
		      0, 0, 0, 0, 0, &res);
		      m4_is_enabled = !!res.a0;

	if (m4_is_enabled)
		printk("M4 is started\n");

	return 0;
}
EXPORT_SYMBOL_GPL(check_m4_enabled);

int imx8mp_get_ldo_trim(int ldo)
{
	return ldo_trim[ldo];
}
EXPORT_SYMBOL_GPL(imx8mp_get_ldo_trim);
