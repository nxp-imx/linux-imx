/*
 * Copyright 2019 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *
 * @file vpu_mu.c
 *
 */

#include "vpu_mu.h"

static void vpu_mu_inq_msg(struct vpu_dev *dev, void *msg)
{
	if (&dev->mu_msg_fifo == NULL) {
		vpu_err("&dev->mu_msg_fifo == NULL\n");
		return;
	}

	if (kfifo_in(&dev->mu_msg_fifo, msg, sizeof(u_int32)) != sizeof(u_int32)) {
		vpu_err("No memory for mu msg fifo\n");
		return;
	}

	queue_work(dev->workqueue, &dev->msg_work);
}

static irqreturn_t vpu_mu_irq_handle(int irq, void *This)
{
	struct vpu_dev *dev = This;
	u_int32 msg;

	MU_ReceiveMsg(dev->mu_base_virtaddr, 0, &msg);

	vpu_mu_inq_msg(dev, &msg);

	return IRQ_HANDLED;
}

static int vpu_mu_init(struct vpu_dev *dev)
{
	struct device_node *np;
	unsigned int	vpu_mu_id;
	int ret = 0;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu0-vpu-m0");
	if (!np) {
		vpu_err("error: Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
	dev->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!dev->mu_base_virtaddr);

	ret = of_property_read_u32_index(np, "fsl,vpu_ap_mu_id", 0, &vpu_mu_id);
	if (ret) {
		vpu_err("error: Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	dev->vpu_mu_id = vpu_mu_id;

	dev->vpu_irq = of_irq_get(np, 0);

	ret = devm_request_irq(&dev->plat_dev->dev, dev->vpu_irq, vpu_mu_irq_handle,
				IRQF_EARLY_RESUME, "vpu_mu_isr", (void *)dev);
	if (ret) {
		vpu_err("error: request_irq failed %d, error = %d\n", dev->vpu_irq, ret);
		return -EINVAL;
	}

	if (!dev->vpu_mu_init) {
		vpu_mu_enable_rx(dev);
		dev->vpu_mu_init = 1;
	}

	return ret;
}

int vpu_mu_request(struct vpu_dev *dev)
{
	int ret = 0;

	ret = vpu_mu_init(dev);
	if (ret) {
		vpu_err("error: %s vpu mu init failed\n", __func__);
		return ret;
	}

	return 0;
}

void vpu_mu_free(struct vpu_dev *dev)
{
	devm_free_irq(&dev->plat_dev->dev, dev->vpu_irq, (void *)dev);
}

void vpu_mu_send_msg(struct vpu_dev *dev, MSG_Type type, u_int32 value)
{
	if (value != 0xffff)
		MU_SendMessage(dev->mu_base_virtaddr, 1, value);
	if (type != 0xffff)
		MU_SendMessage(dev->mu_base_virtaddr, 0, type);
}

u_int32 vpu_mu_receive_msg(struct vpu_dev *dev, void *msg)
{
	int ret = 0;

	if (kfifo_len(&dev->mu_msg_fifo) >= sizeof(u_int32)) {
		ret = kfifo_out(&dev->mu_msg_fifo, msg, sizeof(u_int32));
		if (ret != sizeof(u_int32))
			vpu_err("error: kfifo_out mu msg failed\n, ret=%d\n",
				ret);
	} else {
		ret = kfifo_len(&dev->mu_msg_fifo);
	}

	return ret;
}

int vpu_sc_check_fuse(struct vpu_dev *dev,
			    struct vpu_v4l2_fmt *pformat_table,
			    u_int32 table_size)
{
	sc_err_t ret;
	sc_ipc_t ipc;
	unsigned int mu_id;
	u_int32 fuse;
	u_int32 val;
	u_int32 i;

	ret = sc_ipc_getMuID(&mu_id);
	if (ret) {
		vpu_err("error: sc_ipc_getMuID() cannot obtain mu id SCI error! (%d)\n", ret);
		return ret;
	}
	ret = sc_ipc_open(&ipc, mu_id);
	if (ret) {
		vpu_err("error: sc_ipc_getMuID() cannot open MU channel to SCU error! (%d)\n", ret);
		return ret;
	}

	ret = sc_misc_otp_fuse_read(ipc, VPU_DISABLE_BITS, &fuse);
	if (ret) {
		vpu_dbg(LVL_WARN, "warning: %s() read value fail: %d\n",
			__func__, ret);
		return ret;
	}

	val = (fuse >> VPU_IMX_DECODER_FUSE_OFFSET) & 0x3UL;
	if (val == 0x1UL) {
		for (i = 0; i < table_size; i++)
			if (pformat_table[i].fourcc == VPU_PIX_FMT_HEVC)
				pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "H265 is disabled\n");
	} else if (val == 0x2UL) {
		for (i = 0; i < table_size; i++)
			if (pformat_table[i].fourcc == V4L2_PIX_FMT_H264)
				pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "H264 is disabled\n");
	} else if (val == 0x3UL) {
		for (i = 0; i < table_size; i++)
			pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "All decoder disabled\n");
	}

	sc_ipc_close(ipc);

	return 0;
}

void vpu_mu_enable_rx(struct vpu_dev *dev)
{
	MU_Init(dev->mu_base_virtaddr);
	MU_EnableRxFullInt(dev->mu_base_virtaddr, 0);
}
