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
 * @file vpu_encoder_mu.h
 *
 */

#include "vpu_encoder_mu.h"

static void vpu_enc_mu_inq_msg(struct core_device *core_dev, void *msg)
{
	if (&core_dev->mu_msg_fifo == NULL) {
		vpu_err("mu_msg_fifo is NULL\n");
		return;
	}

	if (kfifo_in(&core_dev->mu_msg_fifo, msg, sizeof(u_int32)) != sizeof(u_int32)) {
		vpu_err("No memory for mu msg fifo\n");
		return;
	}

	queue_work(core_dev->workqueue, &core_dev->msg_work);
}

static irqreturn_t vpu_enc_mu_irq_handle(int irq, void *This)
{
	struct core_device *dev = This;
	u32 msg;

	MU_ReceiveMsg(dev->mu_base_virtaddr, 0, &msg);
	vpu_enc_mu_inq_msg(dev, &msg);

	return IRQ_HANDLED;
}

static int vpu_enc_mu_init(struct core_device *core_dev)
{
	int ret = 0;

	core_dev->mu_base_virtaddr =
		core_dev->vdev->regs_base + core_dev->reg_base;
	WARN_ON(!core_dev->mu_base_virtaddr);

	vpu_dbg(LVL_INFO, "core[%d] irq : %d\n", core_dev->id, core_dev->irq);

	ret = devm_request_irq(core_dev->generic_dev, core_dev->irq,
				vpu_enc_mu_irq_handle,
				IRQF_EARLY_RESUME,
				"vpu_mu_isr",
				(void *)core_dev);
	if (ret) {
		vpu_err("request_irq failed %d, error = %d\n",
			core_dev->irq, ret);
		return -EINVAL;
	}

	if (!core_dev->vpu_mu_init) {
		vpu_enc_mu_enable_rx(core_dev);
		core_dev->vpu_mu_init = 1;
	}

	return ret;
}

int vpu_enc_mu_request(struct core_device *core_dev)
{
	int ret = 0;

	ret = vpu_enc_mu_init(core_dev);
	if (ret)
		vpu_err("error: vpu mu init failed\n");

	return ret;
}

void vpu_enc_mu_free(struct core_device *core_dev)
{
	devm_free_irq(core_dev->generic_dev, core_dev->irq, (void *)core_dev);
}

void vpu_enc_mu_send_msg(struct core_device *core_dev, MSG_Type type, uint32_t value)
{
	if (value != 0xffff)
		MU_SendMessage(core_dev->mu_base_virtaddr, 1, value);
	if (type != 0xffff)
		MU_SendMessage(core_dev->mu_base_virtaddr, 0, type);
}

u_int32 vpu_enc_mu_receive_msg(struct core_device *core_dev, void *msg)
{
	int ret = 0;

	if (kfifo_len(&core_dev->mu_msg_fifo) >= sizeof(u_int32)) {
		ret = kfifo_out(&core_dev->mu_msg_fifo, msg, sizeof(u_int32));
		if (ret != sizeof(u_int32))
			vpu_err("error: kfifo_out mu msg failed\n, ret=%d\n",
				ret);
	} else {
		ret = kfifo_len(&core_dev->mu_msg_fifo);
	}

	return ret;
}

int vpu_enc_sc_check_fuse(void)
{
	sc_ipc_t mu_ipc;
	sc_ipc_id_t mu_id;
	uint32_t fuse = 0xffff;
	int ret;

	ret = sc_ipc_getMuID(&mu_id);
	if (ret) {
		vpu_err("error: can't obtain mu id SCI! ret=%d\n",
				ret);
		return -EINVAL;
	}

	ret = sc_ipc_open(&mu_ipc, mu_id);
	if (ret) {
		vpu_err("error: can't open MU channel to SCU! ret=%d\n",
				ret);
		return -EINVAL;
	}

	ret = sc_misc_otp_fuse_read(mu_ipc, VPU_DISABLE_BITS, &fuse);
	sc_ipc_close(mu_ipc);
	if (ret) {
		vpu_err("error: sc_misc_otp_fuse_read fail! ret=%d\n", ret);
		return -EINVAL;
	}

	vpu_dbg(LVL_INFO, "mu_id = %d, fuse[7] = 0x%x\n", mu_id, fuse);
	if (fuse & VPU_ENCODER_MASK) {
		vpu_err("error, VPU Encoder is disabled\n");
		return -EINVAL;
	}

	return ret;
}

void vpu_enc_mu_enable_rx(struct core_device *core_dev)
{
	MU_Init(core_dev->mu_base_virtaddr);
	MU_EnableRxFullInt(core_dev->mu_base_virtaddr, 0);
}
