/*
 * Freescale On-Chip OTP driver
 *
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#define HW_OCOTP_CTRL			0x00000000
#define HW_OCOTP_CTRL_SET		0x00000004
#define BP_OCOTP_CTRL_WR_UNLOCK		16
#define BM_OCOTP_CTRL_WR_UNLOCK		0xFFFF0000
#define BM_OCOTP_CTRL_RELOAD_SHADOWS	0x00000400
#define BM_OCOTP_CTRL_ERROR		0x00000200
#define BM_OCOTP_CTRL_BUSY		0x00000100
#define BP_OCOTP_CTRL_ADDR		0
#define BM_OCOTP_CTRL_ADDR		0x0000007F

#define HW_OCOTP_TIMING			0x00000010
#define BP_OCOTP_TIMING_STROBE_READ	16
#define BM_OCOTP_TIMING_STROBE_READ	0x003F0000
#define BP_OCOTP_TIMING_RELAX		12
#define BM_OCOTP_TIMING_RELAX		0x0000F000
#define BP_OCOTP_TIMING_STROBE_PROG	0
#define BM_OCOTP_TIMING_STROBE_PROG	0x00000FFF

#define HW_OCOTP_DATA			0x00000020

#define HW_OCOTP_CUST_N(n)	(0x00000400 + (n) * 0x10)
#define BF(value, field)	(((value) << BP_##field) & BM_##field)

#define DEF_RELAX		20	/* > 16.5ns */

#define BANK(a, b, c, d, e, f, g, h) { \
	"HW_OCOTP_"#a, "HW_OCOTP_"#b, "HW_OCOTP_"#c, "HW_OCOTP_"#d, \
	"HW_OCOTP_"#e, "HW_OCOTP_"#f, "HW_OCOTP_"#g, "HW_OCOTP_"#h, \
}

static const char *imx6q_otp_desc[16][8] = {
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

static DEFINE_MUTEX(otp_mutex);
static void __iomem *otp_base;
static struct clk *otp_clk;
struct kobject *otp_kobj;
struct kobj_attribute *otp_kattr;
struct attribute_group *otp_attr_group;

static void set_otp_timing(void)
{
	unsigned long clk_rate = 0;
	unsigned long strobe_read, relex, strobe_prog;
	u32 timing = 0;

	clk_rate = clk_get_rate(otp_clk);

	/* do optimization for too many zeros */
	relex = clk_rate / (1000000000 / DEF_RELAX) - 1;
	strobe_prog = clk_rate / (1000000000 / 10000) + 2 * (DEF_RELAX + 1) - 1;
	strobe_read = clk_rate / (1000000000 / 40) + 2 * (DEF_RELAX + 1) - 1;

	timing = BF(relex, OCOTP_TIMING_RELAX);
	timing |= BF(strobe_read, OCOTP_TIMING_STROBE_READ);
	timing |= BF(strobe_prog, OCOTP_TIMING_STROBE_PROG);

	__raw_writel(timing, otp_base + HW_OCOTP_TIMING);
}

static int otp_wait_busy(u32 flags)
{
	int count;
	u32 c;

	for (count = 10000; count >= 0; count--) {
		c = __raw_readl(otp_base + HW_OCOTP_CTRL);
		if (!(c & (BM_OCOTP_CTRL_BUSY | BM_OCOTP_CTRL_ERROR | flags)))
			break;
		cpu_relax();
	}

	if (count < 0)
		return -ETIMEDOUT;

	return 0;
}

static ssize_t fsl_otp_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	unsigned int index = attr - otp_kattr;
	u32 value = 0;
	int ret;

	ret = clk_prepare_enable(otp_clk);
	if (ret)
		return 0;

	mutex_lock(&otp_mutex);

	set_otp_timing();
	ret = otp_wait_busy(0);
	if (ret)
		goto out;

	value = __raw_readl(otp_base + HW_OCOTP_CUST_N(index));

out:
	mutex_unlock(&otp_mutex);
	clk_disable_unprepare(otp_clk);
	return ret ? 0 : sprintf(buf, "0x%x\n", value);
}

static int otp_write_bits(int addr, u32 data, u32 magic)
{
	u32 c; /* for control register */

	/* init the control register */
	c = __raw_readl(otp_base + HW_OCOTP_CTRL);
	c &= ~BM_OCOTP_CTRL_ADDR;
	c |= BF(addr, OCOTP_CTRL_ADDR);
	c |= BF(magic, OCOTP_CTRL_WR_UNLOCK);
	__raw_writel(c, otp_base + HW_OCOTP_CTRL);

	/* init the data register */
	__raw_writel(data, otp_base + HW_OCOTP_DATA);
	otp_wait_busy(0);

	mdelay(2); /* Write Postamble */

	return 0;
}

static ssize_t fsl_otp_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int index = attr - otp_kattr;
	u32 value;
	int ret;

	sscanf(buf, "0x%x", &value);

	ret = clk_prepare_enable(otp_clk);
	if (ret)
		return 0;

	mutex_lock(&otp_mutex);

	set_otp_timing();
	ret = otp_wait_busy(0);
	if (ret)
		goto out;

	otp_write_bits(index, value, 0x3e77);

	/* Reload all the shadow registers */
	__raw_writel(BM_OCOTP_CTRL_RELOAD_SHADOWS,
		     otp_base + HW_OCOTP_CTRL_SET);
	udelay(1);
	otp_wait_busy(BM_OCOTP_CTRL_RELOAD_SHADOWS);

out:
	mutex_unlock(&otp_mutex);
	clk_disable_unprepare(otp_clk);
	return ret ? 0 : count;
}

static int fsl_otp_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct attribute **attrs;
	const char **desc;
	int i, num;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	otp_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(otp_base)) {
		ret = PTR_ERR(otp_base);
		dev_err(&pdev->dev, "failed to ioremap resource: %d\n", ret);
		return ret;
	}

	otp_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(otp_clk)) {
		ret = PTR_ERR(otp_clk);
		dev_err(&pdev->dev, "failed to get clock: %d\n", ret);
		return ret;
	}

	desc = (const char **) imx6q_otp_desc;
	num = sizeof(imx6q_otp_desc) / sizeof(void *);

	/* The last one is NULL, which is used to detect the end */
	attrs = devm_kzalloc(&pdev->dev, (num + 1) * sizeof(*attrs),
			     GFP_KERNEL);
	otp_kattr = devm_kzalloc(&pdev->dev, num * sizeof(*otp_kattr),
				 GFP_KERNEL);
	otp_attr_group = devm_kzalloc(&pdev->dev, sizeof(*otp_attr_group),
				      GFP_KERNEL);
	if (!attrs || !otp_kattr || !otp_attr_group)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		sysfs_attr_init(&otp_kattr[i].attr);
		otp_kattr[i].attr.name = desc[i];
		otp_kattr[i].attr.mode = 0600;
		otp_kattr[i].show = fsl_otp_show;
		otp_kattr[i].store = fsl_otp_store;
		attrs[i] = &otp_kattr[i].attr;
	}
	otp_attr_group->attrs = attrs;

	otp_kobj = kobject_create_and_add("fsl_otp", NULL);
	if (!otp_kobj) {
		dev_err(&pdev->dev, "failed to add kobject\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(otp_kobj, otp_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs group: %d\n", ret);
		kobject_put(otp_kobj);
		return ret;
	}

	mutex_init(&otp_mutex);

	return 0;
}

static int fsl_otp_remove(struct platform_device *pdev)
{
	sysfs_remove_group(otp_kobj, otp_attr_group);
	kobject_put(otp_kobj);

	return 0;
}

static const struct of_device_id fsl_otp_dt_ids[] = {
	{ .compatible = "fsl,imx6q-ocotp", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_otp_dt_ids);

static struct platform_driver fsl_otp_driver = {
	.driver		= {
		.name   = "imx-ocotp",
		.owner	= THIS_MODULE,
		.of_match_table = fsl_otp_dt_ids,
	},
	.probe		= fsl_otp_probe,
	.remove		= fsl_otp_remove,
};
module_platform_driver(fsl_otp_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huang Shijie <b32955@freescale.com>");
MODULE_DESCRIPTION("Freescale i.MX OCOTP driver");
