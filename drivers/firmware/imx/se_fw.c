// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021-2024 NXP
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/firmware/imx/ele_base_msg.h>
#include <linux/firmware/imx/ele_mu_ioctl.h>
#include <linux/firmware/imx/se_fw_inc.h>
#include <linux/firmware/imx/v2x_base_msg.h>
#include <linux/firmware/imx/svc/seco.h>
#include <linux/genalloc.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sys_soc.h>
#include <linux/fs_struct.h>

#include "ele_common.h"
#include "ele_fw_api.h"
#include "se_fw.h"

static uint32_t v2x_fw_state;

#define SOC_ID_OF_IMX8ULP		0x084D
#define SOC_ID_OF_IMX93			0x9300
#define SOC_ID_OF_IMX8DXL		0xE
#define SOC_ID_OF_IMX8QXP		0x2
#define SOC_REV_A1			0xA100
#define SOC_REV_B0			0xB000
#define SOC_REV_C0			0xC000
#define SOC_VER_MASK			0xFFFF0000
#define SOC_ID_MASK			0x0000FFFF
#define RESERVED_DMA_POOL		BIT(1)
#define SCU_MEM_CFG			BIT(2)
#define SCU_SIGNED_MSG_CFG		BIT(3)
#define IMX_ELE_FW_DIR                 "/lib/firmware/imx/ele/"
#define SECURE_RAM_BASE_ADDRESS_SCU	(0x20800000u)
#define V2X_NON_FIPS			0x00000c00
#define SECO_NON_FIPS			0x00000018

struct imx_info {
	const uint8_t pdev_name[10];
	bool socdev;
	uint8_t mu_id;
	uint8_t mu_did;
	uint8_t max_dev_ctx;
	uint8_t cmd_tag;
	uint8_t rsp_tag;
	uint8_t success_tag;
	uint8_t base_api_ver;
	uint8_t fw_api_ver;
	uint8_t *se_name;
	uint8_t *mbox_tx_name;
	uint8_t *mbox_rx_name;
	uint8_t *pool_name;
	uint32_t mu_buff_size;
	bool reserved_dma_ranges;
	bool init_fw;
	int (*pre_if_config)(struct device *dev);
	int (*post_if_config)(struct device *dev);
	/* platform specific flag to enable/disable the ELE True RNG */
	bool v2x_state_check;
	int (*start_rng)(struct device *dev);
	bool enable_ele_trng;
	uint8_t *fw_name_in_rfs;
	uint8_t *primary_fw_rfs;
};

struct imx_info_list {
	uint8_t num_mu;
	uint16_t soc_id;
	uint16_t soc_rev;
	uint8_t board_type;
	struct imx_info info[];
};

#define IMX8DXL_DL1    0x1
#define IMX8DXL_DL2    0x2
#define IMX8DXL_DL3    0x4

struct seco_soc_info {
	u16 soc_id;
	u16 soc_rev;
	u8  board_type;
};

static LIST_HEAD(priv_data_list);

static struct imx_info_list imx8ulp_info = {
	.num_mu = 1,
	.soc_id = SOC_ID_OF_IMX8ULP,
	.board_type = 0,
	.info = {
			{
				.pdev_name = {"se-fw2"},
				.socdev = true,
				.mu_id = 2,
				.mu_did = 7,
				.max_dev_ctx = 4,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = 0xd6,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "hsm1",
				.mbox_tx_name = "tx",
				.mbox_rx_name = "rx",
				.pool_name = "sram",
				.reserved_dma_ranges = true,
				.pre_if_config = NULL,
				.post_if_config = NULL,
				.v2x_state_check = false,
				.start_rng = ele_start_rng,
				.enable_ele_trng = false,
				.mu_buff_size = 0,
				.fw_name_in_rfs = IMX_ELE_FW_DIR\
						  "mx8ulpa2ext-ahab-container.img",
				.primary_fw_rfs = IMX_ELE_FW_DIR"mx8ulpa2-ahab-container.img",
			},
	},
};

static struct imx_info_list imx93_info = {
	.num_mu = 1,
	.soc_id = SOC_ID_OF_IMX93,
	.board_type = 0,
	.info = {
			{
				.pdev_name = {"se-fw2"},
				.socdev = false,
				.mu_id = 2,
				.mu_did = 3,
				.max_dev_ctx = 4,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = 0xd6,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "hsm1",
				.mbox_tx_name = "tx",
				.mbox_rx_name = "rx",
				.pool_name = NULL,
				.reserved_dma_ranges = true,
				.pre_if_config = NULL,
				.post_if_config = ele_init_fw,
				.v2x_state_check = false,
				.start_rng = ele_start_rng,
				.enable_ele_trng = true,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
	},
};

static const struct imx_info_list imx8dxl_info = {
	.num_mu = 7,
	.soc_id = SOC_ID_OF_IMX8DXL,
	.board_type = 0,
	.info = {
			{
				.pdev_name = {"seco-she"},
				.socdev = false,
				.mu_id = 1,
				.mu_did = 0,
				.max_dev_ctx = 4,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = 0x00,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "she1",
				.mbox_tx_name = "txdb",
				.mbox_rx_name = "rxdb",
				.pool_name = NULL,
				.reserved_dma_ranges = false,
				.pre_if_config = imx_scu_init_fw,
				.post_if_config = NULL,
				.v2x_state_check = false,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"se-fw2"},
				.socdev = false,
				.mu_id = 2,
				.mu_did = 0,
				.max_dev_ctx = 2,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = 0x00,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "hsm1",
				.mbox_tx_name = "txdb",
				.mbox_rx_name = "rxdb",
				.pool_name = NULL,
				.reserved_dma_ranges = false,
				.pre_if_config = imx_scu_init_fw,
				.post_if_config = NULL,
				.v2x_state_check = false,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"v2x-sv0"},
				.socdev = false,
				.mu_id = 4,
				.mu_did = 0,
				.max_dev_ctx = 2,
				.cmd_tag = 0x18,
				.rsp_tag = 0xe2,
				.success_tag = 0x00,
				.base_api_ver = 0x02,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "v2x_sv0",
				.mbox_tx_name = "txdb",
				.mbox_rx_name = "rxdb",
				.pool_name = NULL,
				.reserved_dma_ranges = false,
				.pre_if_config = imx_scu_init_fw,
				.post_if_config = NULL,
				.v2x_state_check = false,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"v2x-sv1"},
				.socdev = false,
				.mu_id = 5,
				.mu_did = 0,
				.max_dev_ctx = 2,
				.cmd_tag = 0x19,
				.rsp_tag = 0xe3,
				.success_tag = 0x00,
				.base_api_ver = 0x02,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "v2x_sv1",
				.mbox_tx_name = "txdb",
				.mbox_rx_name = "rxdb",
				.pool_name = NULL,
				.reserved_dma_ranges = false,
				.pre_if_config = imx_scu_init_fw,
				.post_if_config = NULL,
				.v2x_state_check = false,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"v2x-she"},
				.socdev = false,
				.mu_id = 6,
				.mu_did = 0,
				.max_dev_ctx = 2,
				.cmd_tag = 0x1a,
				.rsp_tag = 0xe4,
				.success_tag = 0x00,
				.base_api_ver = 0x02,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "v2x_she",
				.mbox_tx_name = "txdb",
				.mbox_rx_name = "rxdb",
				.pool_name = NULL,
				.reserved_dma_ranges = false,
				.pre_if_config = imx_scu_init_fw,
				.post_if_config = NULL,
				.v2x_state_check = false,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 16,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"v2x-sg0"},
				.socdev = false,
				.mu_id = 7,
				.mu_did = 0,
				.max_dev_ctx = 2,
				.cmd_tag = 0x1d,
				.rsp_tag = 0xe7,
				.success_tag = 0x00,
				.base_api_ver = 0x02,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "v2x_sg0",
				.mbox_tx_name = "txdb",
				.mbox_rx_name = "rxdb",
				.pool_name = NULL,
				.reserved_dma_ranges = false,
				.pre_if_config = imx_scu_init_fw,
				.post_if_config = NULL,
				.v2x_state_check = false,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"v2x-sg1"},
				.socdev = false,
				.mu_id = 8,
				.mu_did = 0,
				.max_dev_ctx = 2,
				.cmd_tag = 0x1e,
				.rsp_tag = 0xe8,
				.success_tag = 0x00,
				.base_api_ver = 0x02,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "v2x_sg1",
				.mbox_tx_name = "txdb",
				.mbox_rx_name = "rxdb",
				.pool_name = NULL,
				.reserved_dma_ranges = false,
				.pre_if_config = imx_scu_init_fw,
				.post_if_config = NULL,
				.v2x_state_check = false,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
	},
};

static struct imx_info_list imx95_info = {
	.num_mu = 4,
	.soc_id = SOC_ID_OF_IMX95,
	.board_type = 0,
	.info = {
			{
				.pdev_name = {"se-fw2"},
				.socdev = false,
				.mu_id = 2,
				.mu_did = 3,
				.max_dev_ctx = 4,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = 0xd6,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
				.se_name = "hsm1",
				.mbox_tx_name = "tx",
				.mbox_rx_name = "rx",
				.pool_name = NULL,
				.reserved_dma_ranges = false,
				.pre_if_config = NULL,
				.post_if_config = ele_init_fw,
				.v2x_state_check = true,
				.start_rng = ele_start_rng,
				.enable_ele_trng = true,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"v2x-fw0"},
				.socdev = false,
				.mu_id = 0,
				.mu_did = 0,
				.max_dev_ctx = 0,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = 0xd6,
				.base_api_ver = 0x2,
				.fw_api_ver = 0x2,
				.se_name = "v2x_dbg",
				.pool_name = NULL,
				.mbox_tx_name = "tx",
				.mbox_rx_name = "rx",
				.reserved_dma_ranges = false,
				.pre_if_config = NULL,
				.post_if_config = NULL,
				.v2x_state_check = true,
				.start_rng = v2x_start_rng,
				.enable_ele_trng = false,
				.mu_buff_size = 0,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"v2x-fw4"},
				.socdev = false,
				.mu_id = 4,
				.mu_did = 0,
				.max_dev_ctx = 4,
				.cmd_tag = 0x18,
				.rsp_tag = 0xe2,
				.success_tag = 0xd6,
				.base_api_ver = 0x2,
				.fw_api_ver = 0x2,
				.se_name = "v2x_sv0",
				.pool_name = NULL,
				.mbox_tx_name = "tx",
				.mbox_rx_name = "rx",
				.reserved_dma_ranges = false,
				.pre_if_config = NULL,
				.post_if_config = NULL,
				.v2x_state_check = true,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 16,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
			{
				.pdev_name = {"v2x-fw6"},
				.socdev = false,
				.mu_id = 6,
				.mu_did = 0,
				.max_dev_ctx = 4,
				.cmd_tag = 0x1a,
				.rsp_tag = 0xe4,
				.success_tag = 0xd6,
				.base_api_ver = 0x2,
				.fw_api_ver = 0x2,
				.se_name = "v2x_she",
				.pool_name = NULL,
				.mbox_tx_name = "tx",
				.mbox_rx_name = "rx",
				.reserved_dma_ranges = false,
				.pre_if_config = NULL,
				.post_if_config = NULL,
				.v2x_state_check = true,
				.start_rng = NULL,
				.enable_ele_trng = false,
				.mu_buff_size = 256,
				.fw_name_in_rfs = NULL,
				.primary_fw_rfs = NULL,
			},
	}
};

static const struct of_device_id se_fw_match[] = {
	{ .compatible = "fsl,imx8ulp-se-fw", .data = (void *)&imx8ulp_info},
	{ .compatible = "fsl,imx93-se-fw", .data = (void *)&imx93_info},
	{ .compatible = "fsl,imx95-se-fw", .data = (void *)&imx95_info},
	{ .compatible = "fsl,imx8dxl-se-fw", .data = (void *)&imx8dxl_info},
	{},
};

static const struct soc_device_attribute soc_info_matches[] = {
	{ .soc_id = "i.MX8DXL",
	  .revision = "1.1",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8DXL, .soc_rev = SOC_REV_A1,}
	},
	{ .soc_id = "i.MX8DXL",
	  .revision = "1.2",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8DXL, .soc_rev = SOC_REV_B0,}
	},
	{ .soc_id = "i.MX8QXP",
	  .revision = "1.1",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8QXP, .soc_rev = SOC_REV_B0,}
	},
	{ .soc_id = "i.MX8QXP",
	  .revision = "1.2",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8QXP, .soc_rev = SOC_REV_C0,}
	},
};

/*
 * get_se_soc_id() - to fetch the soc_id of the platform
 *
 * @dev  : reference to the device for se interface.
 *
 * This function returns the SoC ID.
 *
 * Context: Other module, requiring to access the secure services based on SoC Id.
 *
 * Return: SoC Id of the device.
 */
uint32_t get_se_soc_id(struct device *dev)
{
	struct imx_info_list *info_list
		= (struct imx_info_list *) device_get_match_data(dev);

	return info_list->soc_id;
}

static struct imx_info *get_imx_info(struct imx_info_list *info_list,
				     const uint8_t *pdev_name,
				     uint8_t len)
{
	int i = 0;

	for (i = 0; i < info_list->num_mu; i++)
		if (!memcmp(info_list->info[i].pdev_name, pdev_name, len))
			return &info_list->info[i];

	return NULL;
}

/*
 * Callback called by mailbox FW when data are received
 */
static void ele_mu_rx_callback(struct mbox_client *c, void *msg)
{
	struct device *dev = c->dev;
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	struct ele_mu_device_ctx *dev_ctx;
	bool is_response = false;
	int msg_size;
	struct mu_hdr header;

	/* The function can be called with NULL msg */
	if (!msg) {
		dev_err(dev, "Message is invalid\n");
		return;
	}

	header.tag = ((u8 *)msg)[TAG_OFFSET];
	header.command = ((u8 *)msg)[CMD_OFFSET];
	header.size = ((u8 *)msg)[SZ_OFFSET];
	header.ver = ((u8 *)msg)[VER_OFFSET];

	/* Incoming command: wake up the receiver if any. */
	if (header.tag == priv->cmd_tag) {
		dev_dbg(dev, "Selecting cmd receiver\n");
		dev_ctx = priv->cmd_receiver_dev;
	} else if (header.tag == priv->rsp_tag) {
		if (priv->waiting_rsp_dev) {
			/* Capture response timer for user space interaction */
			ktime_get_ts64(&priv->time_frame.t_end);
			dev_dbg(dev, "Selecting rsp waiter\n");
			dev_ctx = priv->waiting_rsp_dev;
			is_response = true;
		} else {
			/*
			 * Reading the EdgeLock Enclave response
			 * to the command, sent by other
			 * linux kernel services.
			 */
			spin_lock(&priv->lock);
			if (priv->rx_msg)
				memcpy(priv->rx_msg, msg, header.size << 2);
			else
				dev_err(dev, "No RX buffer to save response.\n");

			complete(&priv->done);
			spin_unlock(&priv->lock);
			return;
		}
	} else {
		dev_err(dev, "Failed to select a device for message: %.8x\n",
				*((u32 *) &header));
		return;
	}

	if (!dev_ctx) {
		dev_err(dev, "No device context selected for message: %.8x\n",
				*((u32 *)&header));
		return;
	}
	/* Init reception */
	msg_size = header.size;
	if (msg_size > MAX_RECV_SIZE) {
		dev_err(dev, "%s: Message is too big (%d > %d)",
				dev_ctx->miscdev.name,
				msg_size,
				MAX_RECV_SIZE);
		return;
	}

	memcpy(dev_ctx->temp_resp, msg, msg_size << 2);
	dev_ctx->temp_resp_size = msg_size;

	/* Allow user to read */
	dev_ctx->pending_hdr = dev_ctx->temp_resp[0];
	wake_up_interruptible(&dev_ctx->wq);

	if (is_response)
		priv->waiting_rsp_dev = NULL;

}

phys_addr_t get_phy_buf_mem_pool(struct device *dev,
					char *mem_pool_name,
					u32 **buf,
					uint32_t size)
{
	struct device_node *of_node = dev->of_node;
	struct gen_pool *mem_pool = of_gen_pool_get(of_node,
						    mem_pool_name, 0);
	if (!mem_pool) {
		dev_err(dev, "Unable to get sram pool\n");
		return 0;
	}

	*buf = (u32 *)gen_pool_alloc(mem_pool, size);
	if (!buf) {
		dev_err(dev, "Unable to alloc sram from sram pool\n");
		return 0;
	}

	return gen_pool_virt_to_phys(mem_pool, (ulong)*buf);
}

void free_phybuf_mem_pool(struct device *dev,
				 char *mem_pool_name,
				 u32 *buf,
				 uint32_t size)
{
	struct device_node *of_node = dev->of_node;
	struct gen_pool *mem_pool = of_gen_pool_get(of_node,
						    mem_pool_name, 0);

	if (!mem_pool)
		dev_err(dev, "%s failed: Unable to get sram pool.\n", __func__);

	gen_pool_free(mem_pool, (unsigned long)buf, size);
}

static int read_fuse(struct ele_mu_priv *priv, u32 id, u32 *value, u8 mul)
{
	struct nvmem_device *nvmem;
	u32 size_to_read = mul * sizeof(u32);
	struct device_node *np;
	int ret = 0;

	np = priv->dev->of_node;
	if (!of_get_property(np, "nvmem", NULL))
		return 0;

	nvmem = devm_nvmem_device_get(priv->dev, NULL);
	if (IS_ERR(nvmem)) {
		ret = PTR_ERR(nvmem);

		if (ret != -EPROBE_DEFER)
			dev_err(priv->dev, "Failed to retrieve nvmem node\n");

		return ret;
	}

	ret = nvmem_device_read(nvmem, id, size_to_read, value);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to read fuse %d: %d\n", id, ret);
		return ret;
	}

	if (ret != size_to_read) {
		dev_err(priv->dev, "Read only %d instead of %d\n", ret,
			size_to_read);
		return -ENOMEM;
	}

	dev_dbg(priv->dev, "FUSE value 0x%x 0x%x 0x%x 0x%x\n",
		value[0], value[1],
		value[2], value[3]);

	devm_nvmem_device_put(priv->dev, nvmem);

	return 0;
}

/* function to fetch SoC revision on the SECO platform */
static int seco_fetch_soc_info(uint16_t *soc_id, uint16_t *soc_rev)
{
	const struct soc_device_attribute *imx_soc_match;
	int err = 0;

	imx_soc_match = soc_device_match(soc_info_matches);
	if (!soc_id || !soc_rev || !imx_soc_match || !imx_soc_match->data) {
		err = -EINVAL;
		goto exit;
	}

	*soc_id = ((const struct seco_soc_info *)imx_soc_match->data)->soc_id;
	*soc_rev = ((const struct seco_soc_info *)imx_soc_match->data)->soc_rev;
exit:
	return err;
}

static int imx_fetch_soc_info(struct ele_mu_priv *priv,
			      struct imx_info *info)
{
	struct soc_device_attribute *attr;
	struct soc_device *sdev = NULL;
	const struct of_device_id *of_id = of_match_device(se_fw_match, priv->dev);
	struct imx_info_list *info_list;
	phys_addr_t get_info_addr = 0;
	u32 *get_info_data = NULL;
	u8 major_ver, minor_ver;
	int err = 0;
	u32 fuse_val[4] = {0xFF};

	info_list = (struct imx_info_list *)of_id->data;

	if (info_list->soc_id == SOC_ID_OF_IMX8DXL) {
		if (!info_list->board_type) {
			/* Read the fuse values to check fips compliant */
			err = read_fuse(priv, 8, (u32 *)&fuse_val, 4);
			if (err) {
				dev_err(priv->dev, "Fail to read FIPS fuse\n");
				return err;
			}

			if (fuse_val[2] & SECO_NON_FIPS)
				info_list->board_type = IMX8DXL_DL1;
			else if (fuse_val[0] & V2X_NON_FIPS)
				info_list->board_type = IMX8DXL_DL3;
			else if (!fuse_val[0] && !fuse_val[2])
				info_list->board_type = IMX8DXL_DL2;
		}
		return err;
	}

	if (info_list->soc_rev)
		return err;

	if (info->pool_name) {
		get_info_addr = get_phy_buf_mem_pool(priv->dev,
						     info->pool_name,
						     &get_info_data,
						     DEVICE_GET_INFO_SZ);
	} else {
		get_info_data = dmam_alloc_coherent(priv->dev,
						    DEVICE_GET_INFO_SZ,
						    &get_info_addr,
						    GFP_KERNEL);
	}
	if (!get_info_addr) {
		dev_err(priv->dev, "Unable to alloc buffer for device info.\n");
		return -ENOMEM;
	}

	attr = kzalloc(sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	err = ele_get_info(priv->dev, get_info_addr, ELE_GET_INFO_READ_SZ);
	if (err) {
		attr->revision = kasprintf(GFP_KERNEL, "A0");
	} else {
		major_ver = (get_info_data[GET_INFO_SOC_INFO_WORD_OFFSET]
				& SOC_VER_MASK) >> 24;
		minor_ver = ((get_info_data[GET_INFO_SOC_INFO_WORD_OFFSET]
				& SOC_VER_MASK) >> 16) & 0xFF;
		/* SoC revision need to be stored in the info list structure */
		info_list->soc_rev = (get_info_data[GET_INFO_SOC_INFO_WORD_OFFSET]
					& SOC_VER_MASK) >> 16;

		if (minor_ver)
			attr->revision = kasprintf(GFP_KERNEL,
						   "%x.%x",
						   major_ver,
						   minor_ver);
		else
			attr->revision = kasprintf(GFP_KERNEL,
						   "%x",
						   major_ver);

		switch (get_info_data[GET_INFO_SOC_INFO_WORD_OFFSET]
				& SOC_ID_MASK) {
			case SOC_ID_OF_IMX8ULP:
				attr->soc_id = kasprintf(GFP_KERNEL,
							 "i.MX8ULP");
				priv->imem.state = (get_info_data[ELE_IMEM_STATE_WORD]
							& ELE_IMEM_STATE_MASK) >> 16;
				break;
			case SOC_ID_OF_IMX93:
				attr->soc_id = kasprintf(GFP_KERNEL,
							 "i.MX93");
				break;
			case SOC_ID_OF_IMX95:
				attr->soc_id = kasprintf(GFP_KERNEL,
							 "i.MX95");
				break;
		}
	}

	err = of_property_read_string(of_root, "model",
				      &attr->machine);
	if (err) {
		kfree(attr);
		return -EINVAL;
	}
	attr->family = kasprintf(GFP_KERNEL, "Freescale i.MX");

	attr->serial_number
		= kasprintf(GFP_KERNEL, "%016llX",
			    (u64)get_info_data[GET_INFO_SL_NUM_MSB_WORD_OFF] << 32
			    | get_info_data[GET_INFO_SL_NUM_LSB_WORD_OFF]);

	if (info->pool_name) {
		free_phybuf_mem_pool(priv->dev, info->pool_name,
				     get_info_data, DEVICE_GET_INFO_SZ);
	} else {
		dmam_free_coherent(priv->dev,
				   DEVICE_GET_INFO_SZ,
				   get_info_data,
				   get_info_addr);
	}

	if (!info->socdev)
		return 0;

	sdev = soc_device_register(attr);
	if (IS_ERR(sdev)) {
		kfree(attr->soc_id);
		kfree(attr->serial_number);
		kfree(attr->revision);
		kfree(attr->family);
		kfree(attr->machine);
		kfree(attr);
		return PTR_ERR(sdev);
	}

	return 0;
}

/*
 * File operations for user-space
 */

/* Write a message to the MU. */
static ssize_t ele_mu_fops_write(struct file *fp, const char __user *buf,
				  size_t size, loff_t *ppos)
{
	struct ele_mu_device_ctx *dev_ctx
		= container_of(fp->private_data,
			       struct ele_mu_device_ctx,
			       miscdev);
	struct ele_mu_priv *ele_mu_priv = dev_ctx->priv;
	u32 nb_words = 0;
	struct mu_hdr header;
	int err = 0;

	dev_dbg(ele_mu_priv->dev,
		"%s: write from buf (%p)%zu, ppos=%lld\n",
			dev_ctx->miscdev.name,
			buf, size, ((ppos) ? *ppos : 0));

	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx->status != MU_OPENED) {
		err = -EINVAL;
		goto exit;
	}

	if (size < ELE_MU_HDR_SZ) {
		dev_err(ele_mu_priv->dev,
			"%s: User buffer too small(%zu < %d)\n",
				dev_ctx->miscdev.name,
				size, ELE_MU_HDR_SZ);
		err = -ENOSPC;
		goto exit;
	}

	if (size > MAX_MESSAGE_SIZE_BYTES) {
		dev_err(ele_mu_priv->dev,
			"%s: User buffer too big(%zu > %d)\n",
				dev_ctx->miscdev.name,
				size,
				MAX_MESSAGE_SIZE_BYTES);
		err = -ENOSPC;
		goto exit;
	}

	/* Copy data to buffer */
	if (copy_from_user(dev_ctx->temp_cmd, buf, size)) {
		err = -EFAULT;
		dev_err(ele_mu_priv->dev,
			"%s: Fail copy message from user\n",
				dev_ctx->miscdev.name);
		goto exit;
	}

	print_hex_dump_debug("from user ", DUMP_PREFIX_OFFSET, 4, 4,
			     dev_ctx->temp_cmd, size, false);

	header = *((struct mu_hdr *) (&dev_ctx->temp_cmd[0]));

	/* Check the message is valid according to tags */
	if (header.tag == ele_mu_priv->cmd_tag) {
		/*
		 * Command-lock will be unlocked in ele_mu_fops_read
		 * when the response to this command, is read.
		 *
		 * This command lock is taken to serialize
		 * the command execution over an MU.
		 *
		 * A command execution considered completed, when the
		 * response to the command is received.
		 */
		mutex_lock(&ele_mu_priv->mu_cmd_lock);
		ele_mu_priv->waiting_rsp_dev = dev_ctx;
	} else if (header.tag == ele_mu_priv->rsp_tag) {
		/* Check the device context can send the command */
		if (dev_ctx != ele_mu_priv->cmd_receiver_dev) {
			dev_err(ele_mu_priv->dev,
				"%s: Channel not configured to send resp to FW.",
				dev_ctx->miscdev.name);
			err = -EPERM;
			goto exit;
		}
	} else {
		dev_err(ele_mu_priv->dev,
			"%s: The message does not have a valid TAG\n",
				dev_ctx->miscdev.name);
		err = -EINVAL;
		goto exit;
	}

	/*
	 * Check that the size passed as argument matches the size
	 * carried in the message.
	 */
	nb_words = header.size;
	if (nb_words << 2 != size) {
		err = -EINVAL;
		dev_err(ele_mu_priv->dev,
			"%s: User buffer too small\n",
				dev_ctx->miscdev.name);
		goto exit;
	}

	mutex_lock(&ele_mu_priv->mu_lock);

	/* Send message */
	dev_dbg(ele_mu_priv->dev,
		"%s: sending message\n",
			dev_ctx->miscdev.name);

	/* Capture request timer here to not include time for mutex lock */
	if (header.tag == ele_mu_priv->cmd_tag)
		ktime_get_ts64(&ele_mu_priv->time_frame.t_start);

	err = mbox_send_message(ele_mu_priv->tx_chan, dev_ctx->temp_cmd);
	if (err < 0) {
		dev_err(ele_mu_priv->dev,
			"%s: Failed to send message\n",
				dev_ctx->miscdev.name);
		goto unlock;
	}

	err = nb_words << 2;

unlock:
	mutex_unlock(&ele_mu_priv->mu_lock);

exit:
	if (err < 0)
		mutex_unlock(&ele_mu_priv->mu_cmd_lock);
	up(&dev_ctx->fops_lock);
	return err;
}

/*
 * Read a message from the MU.
 * Blocking until a message is available.
 */
static ssize_t ele_mu_fops_read(struct file *fp, char __user *buf,
				 size_t size, loff_t *ppos)
{
	struct ele_mu_device_ctx *dev_ctx
		= container_of(fp->private_data,
			       struct ele_mu_device_ctx,
			       miscdev);
	struct ele_mu_priv *ele_mu_priv = dev_ctx->priv;
	u32 data_size = 0, size_to_copy = 0;
	struct ele_buf_desc *b_desc;
	int err;
	struct mu_hdr header = {0};

	dev_dbg(ele_mu_priv->dev,
		"%s: read to buf %p(%zu), ppos=%lld\n",
			dev_ctx->miscdev.name,
			buf, size, ((ppos) ? *ppos : 0));

	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx->status != MU_OPENED) {
		err = -EINVAL;
		goto exit;
	}

	do {
		err = wait_event_interruptible(dev_ctx->wq, dev_ctx->pending_hdr != 0);
		if (err == -ERESTARTSYS) {
			dev_dbg(dev_ctx->dev,
				"%s: err[0x%x]:interrupted by CTRL + C.\n",
				dev_ctx->miscdev.name, err);
			if (dev_ctx->priv->waiting_rsp_dev) {
				dev_ctx->priv->waiting_rsp_dev->signal_recvd = true;
				continue;
			} else {
				dev_dbg(dev_ctx->dev,
					"Command receiver Dev ctx %s, is getting killed.\n",
					dev_ctx->priv->cmd_receiver_dev->miscdev.name);
				err = -EINTR;
				goto exit;
			}
		} else if (err) {
			dev_err(dev_ctx->dev,
				"%s: err[0x%x]: other than signal interruption.\n",
				dev_ctx->miscdev.name, err);
			goto exit;
		}
	} while (!dev_ctx->pending_hdr);

	dev_dbg(ele_mu_priv->dev,
			"%s: %s %s\n",
			dev_ctx->miscdev.name,
			__func__,
			"message received, start transmit to user");

	/*
	 * Check that the size passed as argument is larger than
	 * the one carried in the message.
	 */
	data_size = dev_ctx->temp_resp_size << 2;
	size_to_copy = data_size;
	if (size_to_copy > size) {
		dev_dbg(ele_mu_priv->dev,
			"%s: User buffer too small (%zu < %d)\n",
				dev_ctx->miscdev.name,
				size, size_to_copy);
		size_to_copy = size;
	}

	/*
	 * We may need to copy the output data to user before
	 * delivering the completion message.
	 */
	while (!list_empty(&dev_ctx->pending_out)) {
		b_desc = list_first_entry_or_null(&dev_ctx->pending_out,
						  struct ele_buf_desc,
						  link);
		if (!b_desc)
			continue;

		if (b_desc->usr_buf_ptr && b_desc->shared_buf_ptr) {

			dev_dbg(ele_mu_priv->dev,
				"%s: Copy output data to user\n",
				dev_ctx->miscdev.name);
			if (copy_to_user(b_desc->usr_buf_ptr,
					 b_desc->shared_buf_ptr,
					 b_desc->size)) {
				dev_err(ele_mu_priv->dev,
					"%s: Failure copying output data to user.",
					dev_ctx->miscdev.name);
				err = -EFAULT;
				goto exit;
			}
		}

		/*
		 * Variable "mu_buff_offset" is set while dealing with MU's device memory.
		 * For device type memory, it is recommended to use memset_io.
		 */
		if (b_desc->shared_buf_ptr) {
			if (dev_ctx->mu_buff_offset)
				memset_io(b_desc->shared_buf_ptr, 0, b_desc->size);
			else
				memset(b_desc->shared_buf_ptr, 0, b_desc->size);
		}

		__list_del_entry(&b_desc->link);
		devm_kfree(dev_ctx->dev, b_desc);
	}

	header = *((struct mu_hdr *) (&dev_ctx->temp_resp[0]));

	/* Copy data from the buffer */
	print_hex_dump_debug("to user ", DUMP_PREFIX_OFFSET, 4, 4,
			     dev_ctx->temp_resp, size_to_copy, false);
	if (copy_to_user(buf, dev_ctx->temp_resp, size_to_copy)) {
		dev_err(ele_mu_priv->dev,
			"%s: Failed to copy to user\n",
				dev_ctx->miscdev.name);
		err = -EFAULT;
		goto exit;
	}

	err = size_to_copy;
	if (dev_ctx->priv->waiting_rsp_dev &&
			dev_ctx == dev_ctx->priv->waiting_rsp_dev &&
			dev_ctx->signal_recvd == true) {
		dev_ctx->signal_recvd = false;
		err = -EINTR;
	}

	/* free memory allocated on the shared buffers. */
	dev_ctx->secure_mem.pos = 0;
	dev_ctx->non_secure_mem.pos = 0;

	dev_ctx->pending_hdr = 0;

exit:
	/*
	 * Clean the used Shared Memory space,
	 * whether its Input Data copied from user buffers, or
	 * Data received from FW.
	 */
	while (!list_empty(&dev_ctx->pending_in) ||
	       !list_empty(&dev_ctx->pending_out)) {
		if (!list_empty(&dev_ctx->pending_in))
			b_desc = list_first_entry_or_null(&dev_ctx->pending_in,
							  struct ele_buf_desc,
							  link);
		else
			b_desc = list_first_entry_or_null(&dev_ctx->pending_out,
							  struct ele_buf_desc,
							  link);

		if (!b_desc)
			continue;

		/*
		 * Variable "mu_buff_offset" is set while dealing with MU's device memory.
		 * For device type memory, it is recommended to use memset_io.
		 */
		if (b_desc->shared_buf_ptr) {
			if (dev_ctx->mu_buff_offset)
				memset_io(b_desc->shared_buf_ptr, 0, b_desc->size);
			else
				memset(b_desc->shared_buf_ptr, 0, b_desc->size);
		}

		__list_del_entry(&b_desc->link);
		devm_kfree(dev_ctx->dev, b_desc);
	}
	if (header.tag == ele_mu_priv->rsp_tag)
		mutex_unlock(&ele_mu_priv->mu_cmd_lock);

	dev_ctx->mu_buff_offset = 0;

	up(&dev_ctx->fops_lock);
	return err;
}

/* Configure the shared memory according to user config */
static int ele_mu_ioctl_shared_mem_cfg_handler(struct file *fp,
					       struct ele_mu_device_ctx *dev_ctx,
					       unsigned long arg)
{
	struct ele_mu_ioctl_shared_mem_cfg cfg;
	int err = -EINVAL;

	/* Check if not already configured. */
	if (dev_ctx->secure_mem.dma_addr != 0u) {
		dev_err(dev_ctx->priv->dev, "Shared memory not configured\n");
		return err;
	}

	err = (int)copy_from_user(&cfg, (u8 *)arg, sizeof(cfg));
	if (err) {
		dev_err(dev_ctx->priv->dev, "Fail copy memory config\n");
		err = -EFAULT;
		return err;
	}

	dev_dbg(dev_ctx->priv->dev, "cfg offset: %u(%d)\n", cfg.base_offset, cfg.size);

	err = imx_scu_sec_mem_cfg(fp, cfg.base_offset, cfg.size);
	if (err) {
		dev_err(dev_ctx->priv->dev, "Failt to map memory\n");
		err = -ENOMEM;
		return err;
	}

	return err;
}

static int ele_mu_ioctl_get_mu_info(struct ele_mu_device_ctx *dev_ctx,
				    unsigned long arg)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	struct ele_mu_ioctl_get_mu_info info;
	struct imx_info *imx_info = (struct imx_info *)priv->info;
	int err = 0;

	info.ele_mu_id = imx_info->mu_id;
	info.interrupt_idx = 0;
	info.tz = 0;
	info.did = imx_info->mu_did;
	info.cmd_tag = imx_info->cmd_tag;
	info.rsp_tag = imx_info->rsp_tag;
	info.success_tag = imx_info->success_tag;
	info.base_api_ver = imx_info->base_api_ver;
	info.fw_api_ver = imx_info->fw_api_ver;

	dev_dbg(priv->dev,
		"%s: info [mu_idx: %d, irq_idx: %d, tz: 0x%x, did: 0x%x]\n",
			dev_ctx->miscdev.name,
			info.ele_mu_id, info.interrupt_idx, info.tz, info.did);

	if (copy_to_user((u8 *)arg, &info, sizeof(info))) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to copy mu info to user\n",
				dev_ctx->miscdev.name);
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

static int ele_mu_ioctl_signed_msg_handler(struct file *fp,
					   struct ele_mu_device_ctx *dev_ctx,
					   unsigned long arg)
{
	struct ele_mu_ioctl_signed_message msg;
	int err;

	err = copy_from_user(&msg, (u8 *)arg, sizeof(msg));
	if (err) {
		dev_err(dev_ctx->priv->dev, "Failed to copy from user: %d\n", err);
		return -EFAULT;
	}

	err = imx_scu_signed_msg(fp, msg.message, msg.msg_size, &msg.error_code);
	if (err) {
		dev_err(dev_ctx->priv->dev,
			"Failed to send signed message: %d\n",
			err);
		return err;
	}

	err = copy_to_user((u8 *)arg, &msg, sizeof(msg));
	if (err) {
		dev_err(dev_ctx->priv->dev, "Failed to copy to user: %d\n", err);
		return -EFAULT;
	}

	return err;
}

/*
 * Copy a buffer of data to/from the user and return the address to use in
 * messages
 */
static int ele_mu_ioctl_setup_iobuf_handler(struct ele_mu_device_ctx *dev_ctx,
					    unsigned long arg)
{
	struct ele_buf_desc *b_desc;
	struct ele_mu_ioctl_setup_iobuf io = {0};
	struct ele_shared_mem *shared_mem;
	int err = 0;
	u32 pos;
	u8 *addr;

	struct ele_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	struct imx_info *imx_info = (struct imx_info *)priv->info;

	if (copy_from_user(&io, (u8 *)arg, sizeof(io))) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed copy iobuf config from user\n",
				dev_ctx->miscdev.name);
		err = -EFAULT;
		goto exit;
	}

	/* Function call to retrieve MU Buffer address */
	if (io.flags & ELE_MU_IO_DATA_BUF_SHE_V2X) {
		addr = get_mu_buf(priv->tx_chan);
		addr = addr + dev_ctx->mu_buff_offset;
		dev_ctx->mu_buff_offset = dev_ctx->mu_buff_offset + io.length;
		if (dev_ctx->mu_buff_offset > imx_info->mu_buff_size) {
			err = -ENOMEM;
			goto exit;
		}
	}

	dev_dbg(dev_ctx->priv->dev,
			"%s: io [buf: %p(%d) flag: %x]\n",
			dev_ctx->miscdev.name,
			io.user_buf, io.length, io.flags);

	if (io.length == 0 || !io.user_buf) {
		/*
		 * Accept NULL pointers since some buffers are optional
		 * in FW commands. In this case we should return 0 as
		 * pointer to be embedded into the message.
		 * Skip all data copy part of code below.
		 */
		io.ele_addr = 0;
		goto copy;
	}

	/* Select the shared memory to be used for this buffer. */
	if (!(io.flags & ELE_MU_IO_DATA_BUF_SHE_V2X)) {
		if ((io.flags & ELE_MU_IO_FLAGS_USE_SEC_MEM) &&
		    (priv->flags & SCU_MEM_CFG)) {
			/* App requires to use secure memory for this buffer.*/
			shared_mem = &dev_ctx->secure_mem;
		} else {
			/* No specific requirement for this buffer. */
			shared_mem = &dev_ctx->non_secure_mem;
		}
	}

	/* Check there is enough space in the shared memory. */
	if (!(io.flags & ELE_MU_IO_DATA_BUF_SHE_V2X) &&
	     (shared_mem->size < shared_mem->pos
	      || io.length >= shared_mem->size - shared_mem->pos)) {
		dev_err(dev_ctx->priv->dev,
			"%s: Not enough space in shared memory\n",
				dev_ctx->miscdev.name);
		err = -ENOMEM;
		goto exit;
	}

	if (!(io.flags & ELE_MU_IO_DATA_BUF_SHE_V2X)) {
		/* Allocate space in shared memory. 8 bytes aligned. */
		pos = shared_mem->pos;
		shared_mem->pos += round_up(io.length, 8u);
		io.ele_addr = (u64)shared_mem->dma_addr + pos;
	} else {
		io.ele_addr = (u64)addr;
	}

	if (priv->flags & SCU_MEM_CFG) {
		if ((io.flags & ELE_MU_IO_FLAGS_USE_SEC_MEM) &&
		    !(io.flags & ELE_MU_IO_FLAGS_USE_SHORT_ADDR)) {
			/*Add base address to get full address.#TODO: Add API*/
			io.ele_addr += SECURE_RAM_BASE_ADDRESS_SCU;
		}
	}

	if (!(io.flags & ELE_MU_IO_DATA_BUF_SHE_V2X)) {
		memset(shared_mem->ptr + pos, 0, io.length);
	}

	if ((io.flags & ELE_MU_IO_FLAGS_IS_INPUT) ||
	    (io.flags & ELE_MU_IO_FLAGS_IS_IN_OUT)) {
		/*
		 * buffer is input:
		 * copy data from user space to this allocated buffer.
		 */
		if (io.flags & ELE_MU_IO_DATA_BUF_SHE_V2X) {
			err = (int)copy_from_user(addr, io.user_buf, io.length);
		} else {
			err = (int)copy_from_user(shared_mem->ptr + pos, io.user_buf, io.length);
		}

		if (err) {
			dev_err(dev_ctx->priv->dev,
				"%s: Failed copy data to shared memory\n",
				dev_ctx->miscdev.name);
			err = -EFAULT;
			goto exit;
		}
	}

	b_desc = devm_kmalloc(dev_ctx->dev, sizeof(*b_desc), GFP_KERNEL);
	if (!b_desc) {
		err = -ENOMEM;
		dev_err(dev_ctx->priv->dev,
			"%s: Failed allocating mem for pending buffer\n",
			dev_ctx->miscdev.name);
		goto exit;
	}

	if (io.flags & ELE_MU_IO_DATA_BUF_SHE_V2X)
		b_desc->shared_buf_ptr = addr;
	else
		b_desc->shared_buf_ptr = shared_mem->ptr + pos;
	b_desc->usr_buf_ptr = io.user_buf;
	b_desc->size = io.length;

	if (io.flags & ELE_MU_IO_FLAGS_IS_INPUT) {
		/*
		 * buffer is input:
		 * add an entry in the "pending input buffers" list so
		 * that copied data can be cleaned from shared memory
		 * later.
		 */
		list_add_tail(&b_desc->link, &dev_ctx->pending_in);
	} else {
		/*
		 * buffer is output:
		 * add an entry in the "pending out buffers" list so data
		 * can be copied to user space when receiving ELE
		 * response.
		 */
		list_add_tail(&b_desc->link, &dev_ctx->pending_out);
	}

copy:
	/* Provide the EdgeLock Enclave address to user space only if success.*/
	if (copy_to_user((u8 *)arg, &io, sizeof(io))) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to copy iobuff setup to user\n",
				dev_ctx->miscdev.name);
		err = -EFAULT;
		goto exit;
	}
exit:
	return err;
}

/* IOCTL to provide SoC information */
static int ele_mu_ioctl_get_soc_info_handler(struct ele_mu_device_ctx *dev_ctx,
					     unsigned long arg)
{
	const struct of_device_id *of_id = of_match_device(se_fw_match,
							   dev_ctx->priv->dev);
	struct imx_info_list *info_list;
	struct ele_mu_ioctl_get_soc_info soc_info;
	int err = -EINVAL;

	if (!of_id || !of_id->data)
		goto exit;

	info_list = (struct imx_info_list *)of_id->data;

	if (info_list->soc_id == SOC_ID_OF_IMX8DXL) {
		err = seco_fetch_soc_info(&soc_info.soc_id, &soc_info.soc_rev);
		if (err)
			dev_err(dev_ctx->priv->dev,
			"%s: Failed[%d] to fetch SoC Info\n",
			dev_ctx->miscdev.name, err);

		soc_info.board_type = info_list->board_type;
	} else {
		soc_info.soc_id = info_list->soc_id;
		soc_info.soc_rev = info_list->soc_rev;
		soc_info.board_type = info_list->board_type;
	}

	err = (int)copy_to_user((u8 *)arg, (u8 *)(&soc_info), sizeof(soc_info));
	if (err) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to copy soc info to user\n",
			dev_ctx->miscdev.name);
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

/* IOCTL to provide request and response timestamps from FW for a crypto
 * operation
 */
static int ele_mu_ioctl_get_time(struct ele_mu_device_ctx *dev_ctx, unsigned long arg)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	int err = -EINVAL;
	struct ele_time_frame time_frame;

	if (!priv) {
		err = -EINVAL;
		goto exit;
	}

	time_frame.t_start = priv->time_frame.t_start;
	time_frame.t_end = priv->time_frame.t_end;
	err = (int)copy_to_user((u8 *)arg, (u8 *)(&time_frame), sizeof(time_frame));
	if (err) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to copy timer to user\n",
			dev_ctx->miscdev.name);
		err  = -EFAULT;
		goto exit;
	}
exit:
	return err;
}

/* Open a char device. */
static int ele_mu_fops_open(struct inode *nd, struct file *fp)
{
	struct ele_mu_device_ctx *dev_ctx
		= container_of(fp->private_data,
			       struct ele_mu_device_ctx,
			       miscdev);
	struct ele_mu_priv *priv = dev_ctx->priv;
	int err = 0;

	/* Avoid race if opened at the same time */
	if (down_trylock(&dev_ctx->fops_lock))
		return -EBUSY;

	/* Authorize only 1 instance. */
	if (dev_ctx->status != MU_FREE) {
		err = -EBUSY;
		goto exit;
	}

	/*
	 * Allocate some memory for data exchanges with S40x.
	 * This will be used for data not requiring secure memory.
	 */
	dev_ctx->non_secure_mem.ptr = dmam_alloc_coherent(dev_ctx->dev,
					MAX_DATA_SIZE_PER_USER,
					&dev_ctx->non_secure_mem.dma_addr,
					GFP_KERNEL);
	if (!dev_ctx->non_secure_mem.ptr) {
		err = -ENOMEM;
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to map shared memory with FW.\n",
				dev_ctx->miscdev.name);
		goto exit;
	}

	if (priv->flags & SCU_MEM_CFG) {
		err = imx_scu_mem_access(fp);
		if (err) {
			err = -EPERM;
			dev_err(dev_ctx->priv->dev,
				"%s: Failed to share access to shared memory\n",
				dev_ctx->miscdev.name);
			goto free_coherent;
		}
	}

	dev_ctx->non_secure_mem.size = MAX_DATA_SIZE_PER_USER;
	dev_ctx->non_secure_mem.pos = 0;
	dev_ctx->status = MU_OPENED;

	dev_ctx->pending_hdr = 0;

	goto exit;

free_coherent:
	dmam_free_coherent(dev_ctx->priv->dev, MAX_DATA_SIZE_PER_USER,
			   dev_ctx->non_secure_mem.ptr,
			   dev_ctx->non_secure_mem.dma_addr);

exit:
	up(&dev_ctx->fops_lock);
	return err;
}

/* Close a char device. */
static int ele_mu_fops_close(struct inode *nd, struct file *fp)
{
	struct ele_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct ele_mu_device_ctx, miscdev);
	struct ele_mu_priv *priv = dev_ctx->priv;
	struct ele_buf_desc *b_desc;

	/* Avoid race if closed at the same time */
	if (down_trylock(&dev_ctx->fops_lock))
		return -EBUSY;

	/* The device context has not been opened */
	if (dev_ctx->status != MU_OPENED)
		goto exit;

	/* check if this device was registered as command receiver. */
	if (priv->cmd_receiver_dev == dev_ctx)
		priv->cmd_receiver_dev = NULL;

	/* check if this device was registered as waiting response. */
	if (priv->waiting_rsp_dev == dev_ctx) {
		priv->waiting_rsp_dev = NULL;
		mutex_unlock(&priv->mu_cmd_lock);
	}

	/* Unmap secure memory shared buffer. */
	if (dev_ctx->secure_mem.ptr)
		devm_iounmap(dev_ctx->dev, dev_ctx->secure_mem.ptr);

	dev_ctx->secure_mem.ptr = NULL;
	dev_ctx->secure_mem.dma_addr = 0;
	dev_ctx->secure_mem.size = 0;
	dev_ctx->secure_mem.pos = 0;

	/* Free non-secure shared buffer. */
	dmam_free_coherent(dev_ctx->priv->dev, MAX_DATA_SIZE_PER_USER,
			   dev_ctx->non_secure_mem.ptr,
			   dev_ctx->non_secure_mem.dma_addr);

	dev_ctx->non_secure_mem.ptr = NULL;
	dev_ctx->non_secure_mem.dma_addr = 0;
	dev_ctx->non_secure_mem.size = 0;
	dev_ctx->non_secure_mem.pos = 0;

	while (!list_empty(&dev_ctx->pending_in) ||
	       !list_empty(&dev_ctx->pending_out)) {
		if (!list_empty(&dev_ctx->pending_in))
			b_desc = list_first_entry_or_null(&dev_ctx->pending_in,
							  struct ele_buf_desc,
							  link);
		else
			b_desc = list_first_entry_or_null(&dev_ctx->pending_out,
							  struct ele_buf_desc,
							  link);

		if (!b_desc)
			continue;

		/*
		 * Variable "mu_buff_offset" is set while dealing with MU's device memory.
		 * For device type memory, it is recommended to use memset_io.
		 */
		if (b_desc->shared_buf_ptr) {
			if (dev_ctx->mu_buff_offset)
				memset_io(b_desc->shared_buf_ptr, 0, b_desc->size);
			else
				memset(b_desc->shared_buf_ptr, 0, b_desc->size);
		}

		__list_del_entry(&b_desc->link);
		devm_kfree(dev_ctx->dev, b_desc);
	}

	dev_ctx->status = MU_FREE;

exit:
	up(&dev_ctx->fops_lock);
	return 0;
}

/* IOCTL entry point of a char device */
static long ele_mu_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct ele_mu_device_ctx *dev_ctx
		= container_of(fp->private_data,
			       struct ele_mu_device_ctx,
			       miscdev);
	struct ele_mu_priv *ele_mu_priv = dev_ctx->priv;
	int err = -EINVAL;

	/* Prevent race during change of device context */
	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	switch (cmd) {
	case ELE_MU_IOCTL_ENABLE_CMD_RCV:
		if (!ele_mu_priv->cmd_receiver_dev) {
			ele_mu_priv->cmd_receiver_dev = dev_ctx;
			err = 0;
		}
		break;
	case ELE_MU_IOCTL_SHARED_BUF_CFG:
		if (ele_mu_priv->flags & SCU_MEM_CFG)
			err = ele_mu_ioctl_shared_mem_cfg_handler(fp, dev_ctx, arg);
		break;
	case ELE_MU_IOCTL_GET_MU_INFO:
		err = ele_mu_ioctl_get_mu_info(dev_ctx, arg);
		break;
	case ELE_MU_IOCTL_SETUP_IOBUF:
		err = ele_mu_ioctl_setup_iobuf_handler(dev_ctx, arg);
		break;
	case ELE_MU_IOCTL_GET_SOC_INFO:
		err = ele_mu_ioctl_get_soc_info_handler(dev_ctx, arg);
		break;
	case ELE_MU_IOCTL_GET_TIMER:
		err = ele_mu_ioctl_get_time(dev_ctx, arg);
		break;
	case ELE_MU_IOCTL_SIGNED_MESSAGE:
		if (ele_mu_priv->flags & SCU_SIGNED_MSG_CFG)
			err = ele_mu_ioctl_signed_msg_handler(fp, dev_ctx, arg);
		break;
	default:
		err = -EINVAL;
		dev_dbg(ele_mu_priv->dev,
			"%s: IOCTL %.8x not supported\n",
				dev_ctx->miscdev.name,
				cmd);
	}

	up(&dev_ctx->fops_lock);
	return (long)err;
}

/* Char driver setup */
static const struct file_operations ele_mu_fops = {
	.open		= ele_mu_fops_open,
	.owner		= THIS_MODULE,
	.release	= ele_mu_fops_close,
	.unlocked_ioctl = ele_mu_ioctl,
	.read		= ele_mu_fops_read,
	.write		= ele_mu_fops_write,
};

/* interface for managed res to free a mailbox channel */
static void if_mbox_free_channel(void *mbox_chan)
{
	mbox_free_channel(mbox_chan);
}

/* interface for managed res to unregister a char device */
static void if_misc_deregister(void *miscdevice)
{
	misc_deregister(miscdevice);
}

static int ele_mu_request_channel(struct device *dev,
				 struct mbox_chan **chan,
				 struct mbox_client *cl,
				 const char *name)
{
	struct mbox_chan *t_chan;
	int ret = 0;

	t_chan = mbox_request_channel_byname(cl, name);
	if (IS_ERR(t_chan)) {
		ret = PTR_ERR(t_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev,
				"Failed to request chan %s ret %d\n", name,
				ret);
		goto exit;
	}

	ret = devm_add_action(dev, if_mbox_free_channel, t_chan);
	if (ret) {
		dev_err(dev, "failed to add devm removal of mbox %s\n", name);
		goto exit;
	}

	*chan = t_chan;

exit:
	return ret;
}

static int se_probe_cleanup(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct device *dev = &pdev->dev;
	struct ele_mu_priv *priv;

	priv = dev_get_drvdata(dev);

	if (!priv) {
		ret = -EINVAL;
		dev_err(dev, "Invalid ELE-MU Priv data");
		return ret;
	}

	if (priv->tx_chan)
		mbox_free_channel(priv->tx_chan);
	if (priv->rx_chan)
		mbox_free_channel(priv->rx_chan);

	/* free the buffer in ele-mu remove, previously allocated
	 * in ele-mu probe to store encrypted IMEM
	 */
	if (priv->imem.buf) {
		dmam_free_coherent(&pdev->dev,
				   ELE_IMEM_SIZE,
				   priv->imem.buf,
				   priv->imem.phyaddr);
		priv->imem.buf = NULL;
	}

	if (priv->ctxs) {
		for (i = 0; i < priv->max_dev_ctx; i++) {
			if (priv->ctxs[i]) {
				devm_remove_action(dev, if_misc_deregister,
						   &priv->ctxs[i]->miscdev);
				misc_deregister(&priv->ctxs[i]->miscdev);
				devm_kfree(dev, priv->ctxs[i]);
			}
		}
		devm_kfree(dev, priv->ctxs);
	}

	__list_del_entry(&priv->priv_data);

	if (priv->flags & RESERVED_DMA_POOL) {
		of_reserved_mem_device_release(dev);
		priv->flags &= (~RESERVED_DMA_POOL);
	}

	devm_kfree(dev, priv);
	return ret;
}

static int init_device_context(struct device *dev)
{
	int ret = 0;
	int i;
	struct ele_mu_device_ctx *dev_ctx;
	struct ele_mu_priv *priv;
	const struct imx_info *info;
	char *devname;

	priv = dev_get_drvdata(dev);

	if (!priv) {
		ret = -EINVAL;
		dev_err(dev, "Invalid ELE-MU Priv data");
		return ret;
	}
	info = priv->info;

	priv->ctxs = devm_kzalloc(dev, sizeof(dev_ctx) * priv->max_dev_ctx,
				  GFP_KERNEL);

	if (!priv->ctxs) {
		ret = -ENOMEM;
		dev_err(dev, "Fail allocate mem for private dev-ctxs.\n");
		return ret;
	}

	/* Create users */
	for (i = 0; i < priv->max_dev_ctx; i++) {
		dev_ctx = devm_kzalloc(dev, sizeof(*dev_ctx), GFP_KERNEL);
		if (!dev_ctx) {
			ret = -ENOMEM;
			dev_err(dev,
				"Fail to allocate memory for device context\n");
			return ret;
		}

		dev_ctx->dev = dev;
		dev_ctx->status = MU_FREE;
		dev_ctx->priv = priv;

		priv->ctxs[i] = dev_ctx;

		/* Default value invalid for an header. */
		init_waitqueue_head(&dev_ctx->wq);

		INIT_LIST_HEAD(&dev_ctx->pending_out);
		INIT_LIST_HEAD(&dev_ctx->pending_in);
		sema_init(&dev_ctx->fops_lock, 1);

		devname = devm_kasprintf(dev, GFP_KERNEL, "%s_ch%d",
					 info->se_name,
					 i);
		if (!devname) {
			ret = -ENOMEM;
			dev_err(dev,
				"Fail to allocate memory for misc dev name\n");
			return ret;
		}

		dev_ctx->miscdev.name = devname;
		dev_ctx->miscdev.minor = MISC_DYNAMIC_MINOR;
		dev_ctx->miscdev.fops = &ele_mu_fops;
		dev_ctx->miscdev.parent = dev;
		ret = misc_register(&dev_ctx->miscdev);
		if (ret) {
			dev_err(dev, "failed to register misc device %d\n",
				ret);
			return ret;
		}

		ret = devm_add_action(dev, if_misc_deregister,
				      &dev_ctx->miscdev);
		if (ret) {
			dev_err(dev,
				"failed[%d] to add action to the misc-dev\n",
				ret);
			return ret;
		}
	}

	return ret;
}

static void se_load_firmware(const struct firmware *fw, void *context)
{
	struct ele_mu_priv *priv = context;
	const struct imx_info *info = priv->info;
	phys_addr_t ele_fw_phyaddr;
	u8 *se_fw_buf;
	int ret;

	if (!fw) {
		if (priv->fw_fail)
			dev_dbg(priv->dev,
				 "External FW not found, using ROM FW.\n");
		else {
			/*add a bit delay to wait for firmware priv released */
			msleep(20);

			/* Load firmware one more time if timeout */
			request_firmware_nowait(THIS_MODULE,
					FW_ACTION_UEVENT, priv->se_img_file_to_load,
					priv->dev, GFP_KERNEL, priv,
					se_load_firmware);
			priv->fw_fail++;
			dev_dbg(priv->dev, "Value of retries = 0x%x\n",
				priv->fw_fail);
		}

		return;
	}

	dev_info(priv->dev, "loading firmware %s\n", priv->se_img_file_to_load);

	/* allocate buffer to store the SE FW */
	se_fw_buf = dma_alloc_coherent(priv->dev, fw->size,
			&ele_fw_phyaddr, GFP_KERNEL);
	if (!se_fw_buf)
		goto exit;

	memcpy(se_fw_buf, fw->data, fw->size);
	ret = ele_fw_authenticate(priv->dev, ele_fw_phyaddr);
	if (ret < 0)
		dev_err(priv->dev,
			"Error %pe: Authenticate & load SE firmware %s.\n",
			ERR_PTR(ret),
			priv->se_img_file_to_load);

	dma_free_coherent(priv->dev,
			fw->size,
			se_fw_buf,
			ele_fw_phyaddr);

	if (priv->imem.state == ELE_IMEM_STATE_BAD &&
	    priv->se_img_file_to_load == info->primary_fw_rfs) {
		priv->se_img_file_to_load = info->fw_name_in_rfs;
		request_firmware_nowait(THIS_MODULE,
				FW_ACTION_UEVENT, priv->se_img_file_to_load,
				priv->dev, GFP_KERNEL, priv,
				se_load_firmware);
	}

exit:
	release_firmware(fw);
}


static int se_fw_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ele_mu_priv *priv;
	const struct of_device_id *of_id = of_match_device(se_fw_match, dev);
	struct imx_info *info = NULL;
	int ret;
	struct device_node *np;
	struct imx_info_list *info_list;

	info = get_imx_info((struct imx_info_list *)of_id->data,
			    pdev->name, strlen(pdev->name) + 1);

	if (!info) {
		dev_err(dev, "Cannot find matching dev-info. %s\n",
			pdev->name);
		ret = -1;
		return ret;
	}

	if (info->v2x_state_check) {
		/* Check if it is the V2X MU, but V2X-FW is not
		 * loaded, then exit.
		 */
		if (v2x_fw_state != V2X_FW_STATE_RUNNING &&
			!memcmp(info->se_name, "v2x_dbg", 8)) {
			ret = -1;
			dev_err(dev, "Failure: V2X FW is not loaded.");
			return ret;
		}
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		dev_err(dev, "Fail allocate mem for private data\n");
		return ret;
	}
	priv->dev = dev;
	priv->info = info;
	memcpy(priv->pdev_name, pdev->name, strlen(pdev->name) + 1);

	dev_set_drvdata(dev, priv);

	list_add_tail(&priv->priv_data, &priv_data_list);

	/*
	 * Get the address of MU.
	 */
	np = pdev->dev.of_node;
	if (!np) {
		dev_err(dev, "Cannot find MU User entry in device tree\n");
		ret = -EOPNOTSUPP;
		goto exit;
	}

	info_list = (struct imx_info_list *)of_id->data;

	if (info->pre_if_config) {
		/* start initializing ele fw */
		ret = info->pre_if_config(dev);
		if (ret) {
			dev_err(dev, "Failed to initialize scu config.\n");
			priv->flags &= (~SCU_MEM_CFG);
			if ((info_list->soc_id == SOC_ID_OF_IMX8DXL) &&
			    ((memcmp(info->se_name, "she1", 5)) ||
			    (memcmp(info->se_name, "hsm1", 5)))) {
				priv->flags &= (~SCU_SIGNED_MSG_CFG);
			}
			goto exit;
		}
		priv->flags |= SCU_MEM_CFG;
		if ((info_list->soc_id == SOC_ID_OF_IMX8DXL) &&
		    ((memcmp(info->se_name, "she1", 5)) ||
		    (memcmp(info->se_name, "hsm1", 5)))) {
			priv->flags |= SCU_SIGNED_MSG_CFG;
		}
	}

	/* Initialize the mutex. */
	mutex_init(&priv->mu_cmd_lock);
	mutex_init(&priv->mu_lock);

	priv->cmd_receiver_dev = NULL;
	priv->waiting_rsp_dev = NULL;

	/* Mailbox client configuration */
	priv->ele_mb_cl.dev		= dev;
	priv->ele_mb_cl.tx_block	= false;
	priv->ele_mb_cl.knows_txdone	= true;
	priv->ele_mb_cl.rx_callback	= ele_mu_rx_callback;

	priv->max_dev_ctx = info->max_dev_ctx;
	priv->cmd_tag = info->cmd_tag;
	priv->rsp_tag = info->rsp_tag;
	priv->success_tag = info->success_tag;
	priv->base_api_ver = info->base_api_ver;
	priv->fw_api_ver = info->fw_api_ver;

	ret = ele_mu_request_channel(dev, &priv->tx_chan,
				     &priv->ele_mb_cl, info->mbox_tx_name);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request tx channel\n");

		goto exit;
	}

	ret = ele_mu_request_channel(dev, &priv->rx_chan,
				     &priv->ele_mb_cl, info->mbox_rx_name);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request rx channel\n");

		goto exit;
	}

	init_completion(&priv->done);
	spin_lock_init(&priv->lock);

	if (info->reserved_dma_ranges) {
		ret = of_reserved_mem_device_init(dev);
		if (ret) {
			dev_err(dev,
				"failed to init reserved memory region %d\n",
				ret);
			priv->flags &= (~RESERVED_DMA_POOL);
			goto exit;
		}
		priv->flags |= RESERVED_DMA_POOL;
	}

	if (info->post_if_config) {
		/* start initializing ele fw */
		ret = info->post_if_config(dev);
		if (ret) {
			dev_err(dev, "Failed to initialize ele fw.\n");
			goto exit;
		}
	}

	ret = imx_fetch_soc_info(priv, info);
	if (ret) {
		dev_err(dev,
			"failed[%d] to register SoC device\n", ret);
		goto exit;
	}

	/* Assumed v2x_state_check is enabled for i.MX95 only. */
	if (info->v2x_state_check) {
		if (v2x_fw_state == V2X_FW_STATE_UNKNOWN &&
				!memcmp(info->se_name, "hsm1", 5)) {
			ret = ele_get_v2x_fw_state(dev, &v2x_fw_state);
			if (ret)
				dev_err(dev, "Failed to start ele rng\n");
		}

		/* Check if it is the V2X MU, but V2X-FW is not
		 * loaded, then exit.
		 */
		if (v2x_fw_state != V2X_FW_STATE_RUNNING &&
			!memcmp(info->se_name, "v2x_she", 8)) {
			ret = -1;
			dev_err(dev, "Failure: V2X FW is not loaded.");
			goto exit;
		}
	}

	/* start ele rng */
	if (info->start_rng) {
		ret = info->start_rng(dev);
		if (ret)
			dev_err(dev, "Failed to start ele rng\n");
	}

	if (!ret && info->enable_ele_trng) {
		ret = ele_trng_init(dev);
		if (ret)
			dev_err(dev, "Failed to init ele-trng\n");
	}

	priv->se_img_file_to_load = info->fw_name_in_rfs;
	if (info->primary_fw_rfs) {
		/* allocate buffer where SE store encrypted IMEM */
		priv->imem.buf = dmam_alloc_coherent(dev, ELE_IMEM_SIZE,
						     &priv->imem.phyaddr,
						     GFP_KERNEL);
		if (!priv->imem.buf) {
			dev_err(dev,
				"dmam-alloc-failed: To store encr-IMEM.\n");
			ret = -ENOMEM;
			goto exit;
		}
		if (priv->imem.state == ELE_IMEM_STATE_BAD)
			priv->se_img_file_to_load = info->primary_fw_rfs;
	}

	if (priv->se_img_file_to_load) {
		ret = request_firmware_nowait(THIS_MODULE,
					      FW_ACTION_UEVENT,
					      priv->se_img_file_to_load,
					      dev, GFP_KERNEL, priv,
					      se_load_firmware);
		if (ret)
			dev_warn(dev, "Failed to get firmware [%s].\n",
				 priv->se_img_file_to_load);
		ret = 0;
	}

	if (info->max_dev_ctx) {
		ret = init_device_context(dev);
		if (ret) {
			dev_err(dev,
				"Failed to create device contexts.\n");
			ret = -ENOMEM;
			goto exit;
		}
	}

	dev_info(dev, "i.MX secure-enclave: %s interface to firmware, configured.\n",
		 info->se_name);

	return devm_of_platform_populate(dev);

exit:
	/* if execution control reaches here, ele-mu probe fail.
	 * hence doing the cleanup
	 */
	if (se_probe_cleanup(pdev))
		dev_err(dev, "Failed clean-up.\n");

	return ret;
}

/**
 * get_se_dev() - to fetch the refernce to the device, corresponding to an MU.
 * @pdev_name: Physical device name of the secure-enclave pNode.
 *
 * This function returns the reference to the device, probed against an MU.
 * Outside this driver or any kernel service requiring to access the secure
 * services over an MU, needs the reference to the device.
 *
 * Context: Other module, requiring to access the secure services over an
 *          MU, needs the reference to the device.
 *
 * Return: Reference to the device.
 */
struct device *get_se_dev(const uint8_t *pdev_name)
{
	struct ele_mu_priv *priv;

	list_for_each_entry(priv, &priv_data_list, priv_data) {
		if (!memcmp(priv->pdev_name, pdev_name, strlen(pdev_name) + 1))
			return priv->dev;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(get_se_dev);

static int se_fw_remove(struct platform_device *pdev)
{
	se_probe_cleanup(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int se_fw_suspend(struct device *dev)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	const struct imx_info *info = priv->info;

	if (info && info->primary_fw_rfs)
		priv->imem.size = save_imem(dev);

	return 0;
}

static int se_fw_resume(struct device *dev)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	const struct imx_info *info = priv->info;
	int i;

	for (i = 0; i < priv->max_dev_ctx; i++)
		wake_up_interruptible(&priv->ctxs[i]->wq);

	if (info && info->primary_fw_rfs)
		restore_imem(dev, info->pool_name);

	return 0;
}
#endif

static const struct dev_pm_ops se_fw_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(se_fw_suspend, se_fw_resume)
};

static struct platform_driver se_fw_driver = {
	.driver = {
		.name = "fsl-se-fw",
		.of_match_table = se_fw_match,
		.pm = &se_fw_pm,
	},
	.probe = se_fw_probe,
	.remove = se_fw_remove,
};
MODULE_DEVICE_TABLE(of, se_fw_match);

module_platform_driver(se_fw_driver);

MODULE_AUTHOR("Pankaj Gupta <pankaj.gupta@nxp.com>");
MODULE_DESCRIPTION("iMX Secure Enclave FW Driver.");
MODULE_LICENSE("GPL");
