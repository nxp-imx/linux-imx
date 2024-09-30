// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020, 2024 NXP
 *
 * File containing client-side RPC functions for the SECO service. These
 * function are ported to clients that communicate to the SC.
 */

#include <linux/firmware/imx/sci.h>
#include <linux/io.h>
#include "se_fw.h"

struct imx_sc_msg_seco_get_build_id {
	struct imx_sc_rpc_msg hdr;
	u32 version;
	u32 commit;
} __packed __aligned(4);

int imx_sc_seco_build_info(struct imx_sc_ipc *ipc, uint32_t *version,
			   uint32_t *commit)
{
	struct imx_sc_msg_seco_get_build_id msg = {0};
	struct imx_sc_rpc_msg *hdr = &msg.hdr;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_SECO;
	hdr->func = IMX_SC_SECO_FUNC_BUILD_INFO;
	hdr->size = 1;

	imx_scu_call_rpc(ipc, &msg, true);

	if (version)
		*version = msg.version;
	if (commit)
		*commit = msg.commit;

	return 0;
}
EXPORT_SYMBOL(imx_sc_seco_build_info);

struct imx_sc_msg_seco_sab_msg {
	struct imx_sc_rpc_msg hdr;
	u32 smsg_addr_hi;
	u32 smsg_addr_lo;
} __packed __aligned(4);

int imx_sc_seco_sab_msg(struct imx_sc_ipc *ipc, u64 smsg_addr)
{
	struct imx_sc_msg_seco_sab_msg msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_SECO;
	hdr->func = IMX_SC_SECO_FUNC_SAB_MSG;
	hdr->size = 3;

	msg.smsg_addr_hi = smsg_addr >> 32;
	msg.smsg_addr_lo = smsg_addr;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	return ret;
}
EXPORT_SYMBOL(imx_sc_seco_sab_msg);

int imx_sc_seco_secvio_enable(struct imx_sc_ipc *ipc)
{
	struct imx_sc_rpc_msg msg;
	struct imx_sc_rpc_msg *hdr = &msg;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = (uint8_t)IMX_SC_RPC_SVC_SECO;
	hdr->func = (uint8_t)IMX_SC_SECO_FUNC_SECVIO_ENABLE;
	hdr->size = 1;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL(imx_sc_seco_secvio_enable);

struct imx_sc_msg_req_seco_config {
	struct imx_sc_rpc_msg hdr;
	u32 data0;
	u32 data1;
	u32 data2;
	u32 data3;
	u32 data4;
	u8 id;
	u8 access;
	u8 size;
} __packed __aligned(4);

struct imx_sc_msg_resp_seco_config {
	struct imx_sc_rpc_msg hdr;
	u32 data0;
	u32 data1;
	u32 data2;
	u32 data3;
	u32 data4;
} __packed __aligned(4);

int imx_sc_seco_secvio_config(struct imx_sc_ipc *ipc, u8 id, u8 access,
			      u32 *data0, u32 *data1, u32 *data2, u32 *data3,
			      u32 *data4, u8 size)
{
	struct imx_sc_msg_req_seco_config msg;
	struct imx_sc_msg_resp_seco_config *resp;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	if (!ipc)
		return -EINVAL;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = (uint8_t)IMX_SC_RPC_SVC_SECO;
	hdr->func = (uint8_t)IMX_SC_SECO_FUNC_SECVIO_CONFIG;
	hdr->size = 7;

	/* Check the pointers on data are valid and set it if doing a write */
	switch (size) {
	case 5:
		if (data4) {
			if (access)
				msg.data4 = *data4;
		} else {
			return -EINVAL;
		}
		fallthrough;
	case 4:
		if (data3) {
			if (access)
				msg.data3 = *data3;
		} else {
			return -EINVAL;
		}
		fallthrough;
	case 3:
		if (data2) {
			if (access)
				msg.data2 = *data2;
		} else {
			return -EINVAL;
		}
		fallthrough;
	case 2:
		if (data1) {
			if (access)
				msg.data1 = *data1;
		} else {
			return -EINVAL;
		}
		fallthrough;
	case 1:
		if (data0) {
			if (access)
				msg.data0 = *data0;
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	msg.id = id;
	msg.access = access;
	msg.size = size;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	resp = (struct imx_sc_msg_resp_seco_config *)&msg;

	/* Pointers already checked so we just copy the data if reading */
	if (!access)
		switch (size) {
		case 5:
			*data4 = resp->data4;
		fallthrough;
		case 4:
			*data3 = resp->data3;
		fallthrough;
		case 3:
			*data2 = resp->data2;
		fallthrough;
		case 2:
			*data1 = resp->data1;
		fallthrough;
		case 1:
			*data0 = resp->data0;
		}

	return 0;
}
EXPORT_SYMBOL(imx_sc_seco_secvio_config);

struct imx_sc_msg_req_seco_dgo_config {
	struct imx_sc_rpc_msg hdr;
	u32 data;
	u8 id;
	u8 access;
} __packed __aligned(4);

struct imx_sc_msg_resp_seco_dgo_config {
	struct imx_sc_rpc_msg hdr;
	u32 data;
} __packed __aligned(4);

int imx_sc_seco_secvio_dgo_config(struct imx_sc_ipc *ipc, u8 id, u8 access,
				  u32 *data)
{
	struct imx_sc_msg_req_seco_dgo_config msg;
	struct imx_sc_msg_resp_seco_dgo_config *resp;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	if (!ipc)
		return -EINVAL;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = (uint8_t)IMX_SC_RPC_SVC_SECO;
	hdr->func = (uint8_t)IMX_SC_SECO_FUNC_SECVIO_DGO_CONFIG;
	hdr->size = 3;

	if (access) {
		if (data)
			msg.data = *data;
		else
			return -EINVAL;
	}

	msg.access = access;
	msg.id = id;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	resp = (struct imx_sc_msg_resp_seco_dgo_config *)&msg;

	if (!access && data)
		*data = resp->data;

	return 0;
}
EXPORT_SYMBOL(imx_sc_seco_secvio_dgo_config);

int imx_scu_init_fw(struct device *dev)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	int ret;

	ret = imx_scu_get_handle(&priv->ipc_scu);
	if (ret) {
		dev_err(dev, "Fail to retrieve IPC handle\n");
		return ret;
	}

	ret = imx_sc_rm_get_resource_owner(priv->ipc_scu, IMX_SC_R_SECO, &priv->part_owner);
	if (ret) {
		dev_err(dev, "Fail get owner of SECO resource\n");
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(imx_scu_init_fw);

int imx_scu_sec_mem_cfg(struct file *fp, uint32_t offset, uint32_t size)
{
	struct ele_mu_device_ctx *dev_ctx
		= container_of(fp->private_data,
			       struct ele_mu_device_ctx,
			       miscdev);
	u64 high_boundary;
	int ret = 0;

	high_boundary = offset;
	if (high_boundary > SECURE_RAM_SIZE) {
		dev_err(dev_ctx->priv->dev, "base offset is over secure memory\n");
		return -ENOMEM;
	}

	high_boundary += size;
	if (high_boundary > SECURE_RAM_SIZE) {
		dev_err(dev_ctx->priv->dev, "total memory is over secure memory\n");
		return -ENOMEM;
	}

	dev_ctx->secure_mem.dma_addr = (dma_addr_t)offset;
	dev_ctx->secure_mem.size = size;
	dev_ctx->secure_mem.pos = 0;
	dev_ctx->secure_mem.ptr = devm_ioremap(dev_ctx->dev,
					      (phys_addr_t)(SECURE_RAM_BASE_ADDRESS +
					      (u64)dev_ctx->secure_mem.dma_addr),
					      dev_ctx->secure_mem.size);
	if (!dev_ctx->secure_mem.ptr) {
		dev_err(dev_ctx->priv->dev, "Failed to map secure memory\n");
		return -ENOMEM;
	}

	return ret;
}
EXPORT_SYMBOL(imx_scu_sec_mem_cfg);

int imx_scu_mem_access(struct file *fp)
{
	struct ele_mu_device_ctx *dev_ctx
		= container_of(fp->private_data,
			       struct ele_mu_device_ctx,
			       miscdev);
	struct ele_mu_priv *priv = dev_ctx->priv;
	u8 mr;
	u64 addr;
	int ret;

	addr = dev_ctx->non_secure_mem.dma_addr;

	ret = imx_sc_rm_find_memreg(priv->ipc_scu,
				    &mr,
				    addr,
				    addr + MAX_DATA_SIZE_PER_USER);
	if (ret) {
		dev_err(dev_ctx->priv->dev,
			"%s: Fail find memreg\n", dev_ctx->miscdev.name);
		return ret;
	}

	ret = imx_sc_rm_set_memreg_permissions(priv->ipc_scu, mr,
					       priv->part_owner,
					       IMX_SC_RM_PERM_FULL);
	if (ret) {
		dev_err(dev_ctx->priv->dev,
			"%s: Fail set permission for resource\n",
				dev_ctx->miscdev.name);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(imx_scu_mem_access);

int imx_scu_signed_msg(struct file *fp,
		       uint8_t *msg,
		       uint32_t size,
		       uint32_t *error)
{
	struct ele_mu_device_ctx *dev_ctx
		= container_of(fp->private_data,
			       struct ele_mu_device_ctx,
			       miscdev);
	struct ele_mu_priv *priv = dev_ctx->priv;
	struct ele_shared_mem *shared_mem = &dev_ctx->non_secure_mem;
	int err;
	u64 addr;
	u32 pos;

	/* Check there is enough space in the shared memory. */
	if (size >= shared_mem->size - shared_mem->pos) {
		dev_err(dev_ctx->priv->dev,
			"Not enough mem: %d left, %d required\n",
			shared_mem->size - shared_mem->pos, size);
		return -ENOMEM;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;

	/* get physical address from the pos */
	addr = (u64)shared_mem->dma_addr + pos;

	/* copy signed message from user space to this allocated buffer */
	err = copy_from_user(shared_mem->ptr + pos, msg, size);
	if (err) {
		dev_err(dev_ctx->priv->dev,
			"Failed to signed message from user: %d\n",
			err);
		return -EFAULT;
	}

	*error = imx_sc_seco_sab_msg(priv->ipc_scu, addr);
	err = *error;
	if (err) {
		dev_err(dev_ctx->priv->dev, "Failt to send signed message\n");
	}

	return err;
}
EXPORT_SYMBOL(imx_scu_signed_msg);
