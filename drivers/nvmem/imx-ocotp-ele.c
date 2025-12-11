// SPDX-License-Identifier: GPL-2.0-only
/*
 * i.MX9 OCOTP fusebox driver
 *
 * Copyright 2023 NXP
 */

#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/firmware/imx/se_api.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/if_ether.h>	/* ETH_ALEN */

#define MAC1_ADDR_OFFSET_BYTE 0x4ec
#define MAC2_ADDR_OFFSET_BYTE 0x4f2
#define IMX94_MAC_ADDR_OFFSET 0x854
#define IMX95_MAC_ADDR_OFFSET 0x514

enum fuse_type {
	FUSE_FSB = BIT(0),
	FUSE_ELE = BIT(1),
	FUSE_ECC = BIT(2),
	FUSE_UID = BIT(3),
	FUSE_INVALID = -1
};

struct ocotp_map_entry {
	u32 start; /* start word */
	u32 num; /* num words */
	enum fuse_type type;
	u32 off_or_id_8ulp; /* used for storing FUSEa (0-255) offset or Fuse ID */
};

struct ocotp_devtype_data {
	u32 reg_off;
	char *name;
	u32 size;
	u32 num_entry;
	u32 flag;
	nvmem_reg_read_t reg_read;
	nvmem_reg_read_t reg_write;
	uint32_t se_soc_id;
	bool fuse_mac_addr_93;
	bool fuse_mac_addr_netc;
	bool reverse_mac_address;
	bool increase_mac_address;
	const u8 *pf_mac_offset_list;
	struct ocotp_map_entry entry[];
};

struct imx_ocotp_priv {
	struct device *dev;
	void __iomem *base;
	struct nvmem_config config;
	struct mutex lock;
	const struct ocotp_devtype_data *data;
	void *se_data;
	u8 pfn;
};

static enum fuse_type imx_ocotp_fuse_type(void *context, u32 index,
					  const struct ocotp_map_entry **entry)
{
	struct imx_ocotp_priv *priv = context;
	const struct ocotp_devtype_data *data = priv->data;
	u32 start, end;
	int i;

	for (i = 0; i < data->num_entry; i++) {
		start = data->entry[i].start;
		end = data->entry[i].start + data->entry[i].num;

		if (index >= start && index < end) {
			*entry = data->entry + i;
			return (*entry)->type;
		}
	}

	return FUSE_INVALID;
}

static int imx_ocotp_reg_read(void *context, unsigned int offset, void *val, size_t bytes)
{
	struct imx_ocotp_priv *priv = context;
	const struct ocotp_map_entry *entry;
	void __iomem *reg = priv->base + priv->data->reg_off;
	u32 count, index, num_bytes, off_or_id;
	enum fuse_type type;
	u32 *buf;
	void *p;
	int i;
	u8 skipbytes;
	int ret = 0;

	if (priv->data->fuse_mac_addr_93) {
		if (offset == MAC2_ADDR_OFFSET_BYTE) {
			offset = offset / 4;
			u32 mac2_addr[2];

			ret = imx_se_read_fuse(priv->se_data, offset, &mac2_addr[0]);
			if (ret)
				return ret;

			ret = imx_se_read_fuse(priv->se_data, offset + 1, &mac2_addr[1]);
			if (ret)
				return ret;

			memcpy(val, (u8 *)(mac2_addr) + 2, bytes);
			return ret;
		}
	}

	if (priv->data->fuse_mac_addr_netc) {
		if ((offset & 0xfff) == IMX95_MAC_ADDR_OFFSET ||
		    (offset & 0xfff) == IMX94_MAC_ADDR_OFFSET) {
			priv->pfn = offset >> 12 & 0xf;
			offset = offset & 0xfff;
		}
	}

	if (offset + bytes > priv->data->size)
		bytes = priv->data->size - offset;

	index = offset >> 2;
	skipbytes = offset - (index << 2);
	num_bytes = round_up(bytes + skipbytes, 4);
	count = num_bytes >> 2;

	p = kzalloc(num_bytes, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	mutex_lock(&priv->lock);

	buf = p;

	for (i = index; i < (index + count); i++) {
		type = imx_ocotp_fuse_type(context, i, &entry);

		if (type == FUSE_INVALID) {
			*buf++ = 0;
			continue;
		}

		if (type == (FUSE_ELE | FUSE_UID)) {
			u32 uid[4];

			ret = imx_se_read_fuse(priv->se_data, OTP_UNIQ_ID, uid);
			if (ret)
				goto err;

			*buf++ = uid[i - entry->start];
			continue;
		}

		if (priv->data->se_soc_id == SOC_ID_OF_IMX8ULP)
			off_or_id = entry->off_or_id_8ulp + (i - entry->start);
		else
			off_or_id = i;

		if (type == (FUSE_FSB | FUSE_ECC)) {
			*buf++ = readl_relaxed(reg + (off_or_id << 2)) & GENMASK(15, 0);
		} else if (type == FUSE_FSB) {
			*buf++ = readl_relaxed(reg + (off_or_id << 2));
		} else if (type == FUSE_ELE) {
			ret = imx_se_read_fuse(priv->se_data, off_or_id, buf++);
			if (ret)
				goto err;
		}
	}

	memcpy(val, ((u8 *)p) + skipbytes, bytes);
err:
	mutex_unlock(&priv->lock);

	kfree(p);

	return ret;
};

static int imx_ocotp_cell_pp(void *context, const char *id, int index,
			     unsigned int offset, void *data, size_t bytes)
{
	u8 *buf = data;
	int i;

	/* Deal with some post processing of nvmem cell data */
	if (id && !strcmp(id, "mac-address")) {
		bytes = min(bytes, ETH_ALEN);
		for (i = 0; i < bytes / 2; i++)
			swap(buf[i], buf[bytes - i - 1]);
	}

	return 0;
}

static void imx_ocotp_fixup_dt_cell_info(struct nvmem_device *nvmem,
					 struct nvmem_cell_info *cell)
{
	cell->read_post_process = imx_ocotp_cell_pp;
}

static int imx_ocotp_reg_write(void *context, unsigned int offset, void *val, size_t bytes)
{
	struct imx_ocotp_priv *priv = context;
	u32 *buf = val;
	u32 index;
	int ret;

	/* allow only writing one complete OTP word at a time */
	if (bytes != 4)
		return -EINVAL;

	/* divide the offset by the word size to get the word count */
	index = offset / 4;

	mutex_lock(&priv->lock);
	ret = imx_se_write_fuse(priv->se_data, index, *buf, false);
	mutex_unlock(&priv->lock);

	return ret;
}

static int imx_ocotp_ele_post_process(void *context, const char *id, int index,
				      unsigned int offset, void *data, size_t bytes)
{
	struct imx_ocotp_priv *priv = context;
	u8 *buf = data;
	int i;

	if (!priv)
		return -EINVAL;

	/* Deal with some post processing of nvmem cell data */
	if (id && !strcmp(id, "mac-address")) {
		if (priv->data->reverse_mac_address) {
			for (i = 0; i < bytes / 2; i++)
				swap(buf[i], buf[bytes - i - 1]);
		}

		if (priv->data->increase_mac_address && priv->data->pf_mac_offset_list) {
			if (priv->pfn >= sizeof(priv->data->pf_mac_offset_list))
				return -EINVAL;
			if (is_zero_ether_addr(buf))
				return 0;
			eth_addr_add(buf, priv->data->pf_mac_offset_list[priv->pfn]);
		}
	}

	return 0;
}

struct imx_ocotp_priv *gpriv;
static void imx_ocotp_ele_fixup_dt_cell(struct nvmem_device *nvmem,
					struct nvmem_cell_info *cell)
{
	cell->priv = gpriv;
	cell->read_post_process = imx_ocotp_ele_post_process;
}

static int imx_ele_ocotp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx_ocotp_priv *priv;
	struct nvmem_device *nvmem;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	gpriv = priv;

	priv->data = of_device_get_match_data(dev);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->se_data = imx_get_se_data_info(priv->data->se_soc_id, 0);
	if (!priv->se_data)
		return -EPERM;

	priv->config.dev = dev;
	priv->config.name = "ELE-OCOTP";
	priv->config.id = NVMEM_DEVID_AUTO;
	priv->config.owner = THIS_MODULE;
	priv->config.size = priv->data->size;
	priv->config.add_legacy_fixed_of_cells = true;
	priv->config.reg_read = priv->data->reg_read;
	priv->config.word_size = 1;
	priv->config.reg_write = priv->data->reg_write;
	priv->config.priv = priv;
	priv->config.add_legacy_fixed_of_cells = true;
	priv->config.fixup_dt_cell_info = imx_ocotp_fixup_dt_cell_info;
	if (priv->data->reverse_mac_address || priv->data->increase_mac_address)
		priv->config.fixup_dt_cell_info = &imx_ocotp_ele_fixup_dt_cell;
	mutex_init(&priv->lock);

	nvmem = devm_nvmem_register(dev, &priv->config);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	return 0;
}

static const struct ocotp_devtype_data imx8ulp_ocotp_data = {
	.reg_off = 0x800,
	.reg_read = imx_ocotp_reg_read,
	.size = 2048, /* 64 Banks */
	.num_entry = 17,
	.se_soc_id = SOC_ID_OF_IMX8ULP,
	.entry = {
		{ 8, 8, FUSE_ELE, 8 }, /* Lock */
		{ 16, 8, FUSE_ELE, 16 }, /* ECID */
		{ 24, 8, FUSE_FSB, 0 }, /* Boot cfg */
		{ 32, 8, FUSE_FSB, 8 }, /* Boot cfg */
		{ 40, 8, FUSE_FSB, 64 }, /* PLL, etc */
		{ 48, 8, FUSE_FSB, 72 }, /* Osc Trim */
		{ 56, 4, FUSE_ELE | FUSE_UID, 1 }, /* OTP_UNIQ_ID */
		{ 64, 4, FUSE_FSB, 80 }, /* ELE Misc Ctrl 1(non-redundant addresses) */
		{ 66, 1, FUSE_ELE, 66 }, /* OEM SRK RVK */
		{ 192, 16, FUSE_ELE, 192 }, /* Tester config */
		{ 208, 8, FUSE_ELE, 208 }, /* PMU */
		{ 216, 8, FUSE_ELE, 216 }, /* Test flow/USB */
		{ 224, 32, FUSE_FSB, 96 },
		{ 256, 40, FUSE_ELE, 256 }, /* GP fuses */
		{ 296, 64, FUSE_FSB, 128 },
		{ 360, 16, FUSE_FSB, 192 },
		{ 392, 24, FUSE_ELE, 392 }, /* GP fuses */
	},
};

static const struct ocotp_devtype_data imx93_ocotp_data = {
	.reg_off = 0x8000,
	.reg_read = imx_ocotp_reg_read,
	.reg_write = imx_ocotp_reg_write,
	.size = 2048,
	.num_entry = 8,
	.se_soc_id = SOC_ID_OF_IMX93,
	.fuse_mac_addr_93 = true,
	.reverse_mac_address = true,
	.entry = {
		{ 0, 1, FUSE_ELE },
		{ 2, 50, FUSE_ELE },
		{ 58, 1, FUSE_ELE },
		{ 63, 1, FUSE_ELE},
		{ 128, 16, FUSE_ELE },
		{ 182, 1, FUSE_ELE },
		{ 188, 1, FUSE_ELE },
		{ 312, 200, FUSE_ELE }
	},
};

/*
 * i.MX94 uses the following mac address offset list:
 * | No.    | Module      | Mac address user             |
 * |--------|-------------|------------------------------|
 * | 0 ~ 1  | ethercat    | port0/port1                  |
 * | 2      | netc switch | internal enetc3 mac OR swp0  |
 * | 3 ~ 6  |             | enetc3 vf1~3 AND swp1        |
 * | 7      | enetc mac   | enetc0 pf                    |
 * | 8      |             | enetc1 pf                    |
 * | 9      |             | enetc2 pf                    |
 * | 10     | netc switch | swp2                         |
 */
static const u8 imx94_pf_mac_offset_list[] = { 2, 7, 8, 9 };
static const struct ocotp_devtype_data imx94_ocotp_data = {
	.reg_off = 0x8000,
	.reg_read = imx_ocotp_reg_read,
	.reg_write = imx_ocotp_reg_write,
	.size = 3296, /* 103 Banks */
	.num_entry = 10,
	.se_soc_id = SOC_ID_OF_IMX94,
	.fuse_mac_addr_netc = true,
	.increase_mac_address = true,
	.pf_mac_offset_list = imx94_pf_mac_offset_list,
	.entry = {
		{ 0, 1, FUSE_FSB | FUSE_ECC },
		{ 7, 1, FUSE_FSB | FUSE_ECC },
		{ 9, 3, FUSE_FSB | FUSE_ECC },
		{ 12, 24, FUSE_FSB },
		{ 36, 2, FUSE_FSB  | FUSE_ECC },
		{ 38, 14, FUSE_FSB },
		{ 59, 1, FUSE_ELE },
		{ 525, 2, FUSE_FSB | FUSE_ECC },
		{ 528, 7, FUSE_FSB },
		{ 536, 280, FUSE_FSB },
	},
};

/*
 * i.MX95 uses the following mac address offset list:
 * | No. | Mac address user  |
 * |-----|-------------------|
 * | 0   | enetc mac pf0     |
 * | 1   | enetc mac vf0     |
 * | 2   | enetc mac vf1     |
 * | 3   | enetc mac pf1     |
 * | 4   | enetc mac vf2     |
 * | 5   | enetc mac vf3     |
 * | 6   | enetc mac pf2     |
 * | 7   | enetc mac vf4     |
 * | 8   | enetc mac vf5     |
 */
static const u8 imx95_pf_mac_offset_list[] = { 0, 3, 6 };
static const struct ocotp_devtype_data imx95_ocotp_data = {
	.reg_off = 0x8000,
	.reg_read = imx_ocotp_reg_read,
	.reg_write = imx_ocotp_reg_write,
	.size = 2440, /* 610 words */
	.num_entry = 15,
	.se_soc_id = SOC_ID_OF_IMX95,
	.fuse_mac_addr_netc = true,
	.increase_mac_address = true,
	.pf_mac_offset_list = imx95_pf_mac_offset_list,
	.entry = {
		{ 0, 1, FUSE_FSB | FUSE_ECC },
		{ 7, 1, FUSE_FSB | FUSE_ECC },
		{ 9, 3, FUSE_FSB | FUSE_ECC },
		{ 12, 24, FUSE_FSB },
		{ 36, 2, FUSE_FSB  | FUSE_ECC },
		{ 38, 14, FUSE_FSB },
		{ 128, 16, FUSE_ELE },
		{ 188, 1, FUSE_ELE },
		{ 317, 2, FUSE_FSB | FUSE_ECC },
		{ 320, 7, FUSE_FSB },
		{ 328, 64, FUSE_FSB },
		{ 448, 143, FUSE_FSB},
		{ 591, 1, FUSE_FSB | FUSE_ECC },
		{ 592, 16, FUSE_FSB }
	},
};

/*
 * i.MX952 uses the following mac address offset list:
 * | No. | Mac address user  |
 * |-----|-------------------|
 * | 0   | enetc mac pf0     |
 * | 1   | enetc mac vf0     |
 * | 2   | enetc mac pf1     |
 * | 3   | enetc mac vf1     |
 */
static const u8 imx952_pf_mac_offset_list[] = { 0, 2 };
static const struct ocotp_devtype_data imx952_ocotp_data = {
	.reg_off = 0x8000,
	.reg_read = imx_ocotp_reg_read,
	.reg_write = imx_ocotp_reg_write,
	.size = 2440, /* 610 words */
	.num_entry = 12,
	.fuse_mac_addr_netc = true,
	.increase_mac_address = true,
	.pf_mac_offset_list = imx952_pf_mac_offset_list,
	.se_soc_id = SOC_ID_OF_IMX952,
	.entry = {
		{ 0, 1, FUSE_FSB | FUSE_ECC },
		{ 7, 1, FUSE_FSB | FUSE_ECC },
		{ 9, 3, FUSE_FSB | FUSE_ECC },
		{ 12, 24, FUSE_FSB },
		{ 36, 2, FUSE_FSB  | FUSE_ECC },
		{ 38, 14, FUSE_FSB },
		{ 317, 2, FUSE_FSB | FUSE_ECC },
		{ 320, 7, FUSE_FSB },
		{ 328, 64, FUSE_FSB },
		{ 448, 143, FUSE_FSB },
		{ 591, 1, FUSE_FSB | FUSE_ECC },
		{ 592, 16, FUSE_FSB }
	},
};

static const struct of_device_id imx_ele_ocotp_dt_ids[] = {
	{ .compatible = "fsl,imx8ulp-ocotp", .data = &imx8ulp_ocotp_data, },
	{ .compatible = "fsl,imx93-ocotp", .data = &imx93_ocotp_data, },
	{ .compatible = "fsl,imx94-ocotp", .data = &imx94_ocotp_data, },
	{ .compatible = "fsl,imx95-ocotp", .data = &imx95_ocotp_data, },
	{ .compatible = "fsl,imx952-ocotp", .data = &imx952_ocotp_data, },
	{},
};
MODULE_DEVICE_TABLE(of, imx_ele_ocotp_dt_ids);

static struct platform_driver imx_ele_ocotp_driver = {
	.driver = {
		.name = "imx_ele_ocotp",
		.of_match_table = imx_ele_ocotp_dt_ids,
	},
	.probe = imx_ele_ocotp_probe,
};
module_platform_driver(imx_ele_ocotp_driver);

MODULE_DESCRIPTION("i.MX OCOTP/ELE driver");
MODULE_AUTHOR("Peng Fan <peng.fan@nxp.com>");
MODULE_LICENSE("GPL");
