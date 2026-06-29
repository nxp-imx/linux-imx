// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2017-2019 NXP */

#include "enetc_pf_common.h"
#include "enetc_msg.h"

static void enetc_msg_enable_mr_int(struct enetc_pf *pf, bool en)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 val, i, mr_mask = 0;

	for (i = 0; i < pf->num_vfs; i++)
		mr_mask |= PSIIER_MR(i);

	val = enetc_rd(hw, ENETC_PSIIER);
	if (en)
		val |= mr_mask;
	else
		val &= ~mr_mask;

	enetc_wr(hw, ENETC_PSIIER, val);
}

static void enetc_msg_enable_flr_int(struct enetc_pf *pf, bool en)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 val;

	val = enetc_rd(hw, ENETC_PSIIER);
	if (en)
		val |= PSIIDR_VSI_FLR(pf->num_vfs);
	else
		val &= ~PSIIDR_VSI_FLR(pf->num_vfs);

	enetc_wr(hw, ENETC_PSIIER, val);
}

static irqreturn_t enetc_msg_psi_msix(int irq, void *data)
{
	struct enetc_si *si = (struct enetc_si *)data;
	struct enetc_pf *pf = enetc_si_priv(si);

	enetc_msg_enable_mr_int(pf, false);
	if (!is_enetc_rev1(si))
		enetc_msg_enable_flr_int(pf, false);

	schedule_work(&si->msg_task);

	return IRQ_HANDLED;
}

static bool enetc_pf_is_vf_trusted(struct enetc_pf *pf, int vf_id)
{
	if (vf_id >= pf->total_vfs)
		return false;

	return !!(pf->vf_state[vf_id].flags & ENETC_VF_FLAG_TRUSTED);
}

static u16 enetc_msg_pf_set_vf_primary_mac_addr(struct enetc_pf *pf,
						int vf_id)
{
	struct enetc_vf_state *vf_state = &pf->vf_state[vf_id];
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_msg_mac_exact_filter *msg;
	char *addr;

	if (!enetc_pf_is_vf_trusted(pf, vf_id))
		return ENETC_MSG_CODE_PERMISSION_DENY;

	msg = (struct enetc_msg_mac_exact_filter *)msg_swbd->vaddr;
	addr = msg->mac[0].addr;
	if (vf_state->flags & ENETC_VF_FLAG_PF_SET_MAC)
		dev_warn(dev, "Attempt to override PF set mac addr for VF%d\n",
			 vf_id);

	pf->ops->set_si_primary_mac(&pf->si->hw, vf_id + 1, addr);

	return ENETC_MSG_CODE_SUCCESS;
}

static u16 enetc_msg_pf_set_vf_mac_hash_filter(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_mac_hash_filter *msg;
	struct enetc_hw *hw = &pf->si->hw;
	int si_id = vf_id + 1;
	u64 uc_hash, mc_hash;

	if (is_enetc_rev1(pf->si))
		return ENETC_MSG_CODE_NOT_SUPPORT;

	if (!enetc_pf_is_vf_trusted(pf, vf_id))
		return ENETC_MSG_CODE_PERMISSION_DENY;

	msg = (struct enetc_msg_mac_hash_filter *)msg_swbd->vaddr;
	/* Currently, hardware only supports 64 bits table size */
	if (msg->size != ENETC_MAC_HASH_TABLE_SIZE_64)
		return ENETC_MSG_CODE_NOT_SUPPORT;

	if (msg->type == ENETC_MAC_FILTER_TYPE_UC) {
		uc_hash = (u64)msg->hash_tbl[1] << 32 | msg->hash_tbl[0];
		pf->ops->set_si_mac_hash_filter(hw, si_id, UC, uc_hash);
	} else if (msg->type == ENETC_MAC_FILTER_TYPE_MC) {
		mc_hash = (u64)msg->hash_tbl[1] << 32 | msg->hash_tbl[0];
		pf->ops->set_si_mac_hash_filter(hw, si_id, MC, mc_hash);
	} else {
		uc_hash = (u64)msg->hash_tbl[1] << 32 | msg->hash_tbl[0];
		pf->ops->set_si_mac_hash_filter(hw, si_id, UC, uc_hash);
		mc_hash = (u64)msg->hash_tbl[3] << 32 | msg->hash_tbl[2];
		pf->ops->set_si_mac_hash_filter(hw, si_id, MC, mc_hash);
	}

	return ENETC_MSG_CODE_SUCCESS;
}

static u16 enetc_msg_pf_set_vf_mac_promisc_mode(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_mac_promisc_mode *msg;
	struct enetc_hw *hw = &pf->si->hw;
	bool promisc_mode = false;
	int si_id = vf_id + 1;

	msg = (struct enetc_msg_mac_promisc_mode *)msg_swbd->vaddr;
	if (msg->promisc_mode == ENETC_MAC_PROMISC_MODE_ENABLE) {
		if (!enetc_pf_is_vf_trusted(pf, vf_id))
			return ENETC_MSG_CODE_PERMISSION_DENY;

		promisc_mode = true;
	}

	if (msg->type & ENETC_MAC_FILTER_TYPE_UC) {
		pf->ops->set_si_mac_promisc(hw, si_id, UC, promisc_mode);
		if (msg->flush_macs)
			pf->ops->set_si_mac_hash_filter(hw, si_id, UC, 0);
	}

	if (msg->type & ENETC_MAC_FILTER_TYPE_MC) {
		pf->ops->set_si_mac_promisc(hw, si_id, MC, promisc_mode);
		if (msg->flush_macs)
			pf->ops->set_si_mac_hash_filter(hw, si_id, MC, 0);
	}

	return ENETC_MSG_CODE_SUCCESS;
}

static bool enetc_msg_check_crc16(void *msg_addr, u32 msg_size)
{
	u8 *data_buf = msg_addr + 2;
	u8 data_size = msg_size - 2;
	u16 verify_val;

	if (msg_size > ENETC_DEFAULT_MSG_SIZE)
		return false;

	verify_val = crc_itu_t(ENETC_CRC_INIT, data_buf, data_size);
	verify_val = crc_itu_t(verify_val, msg_addr, 2);
	if (verify_val)
		return false;

	return true;
}

static u16 enetc_msg_handle_mac_filter(struct enetc_msg_header *msg_hdr,
				       struct enetc_pf *pf, int vf_id)
{
	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_SET_PRIMARY_MAC:
		return enetc_msg_pf_set_vf_primary_mac_addr(pf, vf_id);
	case ENETC_MSG_SET_MAC_HASH_TABLE:
		return enetc_msg_pf_set_vf_mac_hash_filter(pf, vf_id);
	case ENETC_MSG_SET_MAC_PROMISC_MODE:
		return enetc_msg_pf_set_vf_mac_promisc_mode(pf, vf_id);
	default:
		return ENETC_MSG_CODE_NOT_SUPPORT;
	}
}

static u16 enetc_msg_handle_ip_revision(struct enetc_msg_header *msg_hdr,
					struct enetc_pf *pf)
{
	union enetc_pf_msg pf_msg;

	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_GET_IP_MN:
		pf_msg.class_id = ENETC_MSG_CLASS_ID_IP_REVISION;
		pf_msg.class_code_u8 = pf->si->revision & 0xff;

		return pf_msg.code;
	default:
		return ENETC_MSG_CODE_NOT_SUPPORT;
	}
}

static u16 enetc_msg_pf_set_vf_vlan_hash_filter(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_vlan_hash_filter *msg;
	int si_id = vf_id + 1;
	u64 hash;

	msg = (struct enetc_msg_vlan_hash_filter *)msg_swbd->vaddr;
	/* Currently, hardware only supports 64 bits table size */
	if (msg->size != ENETC_VLAN_HASH_TABLE_SIZE_64)
		return ENETC_MSG_CODE_NOT_SUPPORT;

	hash = (u64)msg->hash_tbl[1] << 32 | msg->hash_tbl[0];
	pf->ops->set_si_vlan_hash_filter(pf->si, si_id, hash);

	return ENETC_MSG_CODE_SUCCESS;
}

static u16 enetc_msg_pf_set_vf_vlan_promisc_mode(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_vlan_promisc_mode *msg;
	struct enetc_si *si = pf->si;
	bool promisc_mode = false;
	int si_id = vf_id + 1;

	msg = (struct enetc_msg_vlan_promisc_mode *)msg_swbd->vaddr;
	if (msg->promisc_mode == ENETC_VLAN_PROMISC_MODE_ENABLE)
		promisc_mode = true;

	pf->ops->set_si_vlan_promisc(&si->hw, si_id, promisc_mode);
	if (msg->flush_vlans)
		pf->ops->set_si_vlan_hash_filter(si, si_id, 0);

	return ENETC_MSG_CODE_SUCCESS;
}

static u16 enetc_msg_handle_vlan_filter(struct enetc_msg_header *msg_hdr,
					struct enetc_pf *pf, int vf_id)
{
	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_SET_VLAN_HASH_TABLE:
		return enetc_msg_pf_set_vf_vlan_hash_filter(pf, vf_id);
	case ENETC_MSG_SET_VLAN_PROMISC_MODE:
		return enetc_msg_pf_set_vf_vlan_promisc_mode(pf, vf_id);
	default:
		return ENETC_MSG_CODE_NOT_SUPPORT;
	}
}

static u16 enetc_msg_pf_reply_link_status(struct enetc_pf *pf)
{
	struct net_device *ndev = pf->si->ndev;
	union enetc_pf_msg pf_msg;

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_STATUS;
	if (netif_carrier_ok(ndev))
		pf_msg.class_code_u8 = ENETC_PF_NC_LINK_STATUS_UP;
	else
		pf_msg.class_code_u8 = ENETC_PF_NC_LINK_STATUS_DOWN;

	return pf_msg.code;
}

int enetc_pf_send_msg(struct enetc_pf *pf, u32 msg_code, u16 ms_mask)
{
	struct enetc_si *si = pf->si;
	u32 psimsgsr;
	int err;

	psimsgsr = PSIMSGSR_SET_MC(msg_code);
	psimsgsr |= ms_mask;

	mutex_lock(&si->msg_lock);

	enetc_wr(&si->hw, ENETC_PSIMSGSR, psimsgsr);
	err = read_poll_timeout(enetc_rd, psimsgsr,
				!(psimsgsr & ms_mask),
				100, 100000, false, &si->hw, ENETC_PSIMSGSR);

	mutex_unlock(&si->msg_lock);

	return err;
}

static void enetc_send_link_status_msg(struct enetc_pf *pf, u16 ms_mask)
{
	struct device *dev = &pf->si->pdev->dev;
	struct net_device *ndev = pf->si->ndev;
	union enetc_pf_msg pf_msg = {};
	int err;

	if (!ms_mask)
		return;

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_STATUS;
	if (netif_carrier_ok(ndev))
		pf_msg.class_code_u8 = ENETC_PF_NC_LINK_STATUS_UP;
	else
		pf_msg.class_code_u8 = ENETC_PF_NC_LINK_STATUS_DOWN;

	err = enetc_pf_send_msg(pf, pf_msg.code, ms_mask);
	if (err)
		dev_err(dev, "PF notifies link status failed\n");
}

static u16 enetc_msg_register_link_status_notify(struct enetc_pf *pf, int vf_id,
						 bool notify)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 val;

	pf->vf_link_status_notify[vf_id] = notify;

	/* Reply to VF */
	val = ENETC_SIMSGSR_SET_MC(ENETC_MSG_CODE_SUCCESS);
	val |= ENETC_PSIMSGRR_MR(vf_id); /* w1c */
	enetc_wr(hw, ENETC_PSIMSGRR, val);

	/* Notify VF the current link status */
	if (notify)
		enetc_send_link_status_msg(pf, PSIMSGSR_MS(vf_id));

	return 0;
}

static u16 enetc_msg_handle_link_status(struct enetc_msg_header *msg_hdr,
					struct enetc_pf *pf, int vf_id)
{
	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_GET_CURRENT_LINK_STATUS:
		return enetc_msg_pf_reply_link_status(pf);
	case ENETC_MSG_REGISTER_LINK_CHANGE_NOTIFY:
		return enetc_msg_register_link_status_notify(pf, vf_id, true);
	case ENETC_MSG_UNREGISTER_LINK_CHANGE_NOTIFY:
		return enetc_msg_register_link_status_notify(pf, vf_id, false);
	default:
		return ENETC_MSG_CODE_NOT_SUPPORT;
	}
}

static u32 enetc_msg_get_speed_code(u32 link_speed)
{
	u32 speed;

	if (link_speed < SPEED_5000)
		return ENETC_MSG_SPEED_UNKNOWN;

	speed = (link_speed - SPEED_5000) / 1000 + ENETC_MSG_SPEED_5G;
	if (speed > ENETC_MSG_SPEED_MAX)
		return ENETC_MSG_SPEED_UNKNOWN;

	return speed;
}

static u16 enetc_msg_pf_reply_link_speed(struct enetc_pf *pf)
{
	struct enetc_ndev_priv *priv = netdev_priv(pf->si->ndev);
	struct ethtool_link_ksettings link_info = {};
	union enetc_pf_msg pf_msg = {};
	u8 speed_code;

	rtnl_lock();
	if (!priv->phylink ||
	    phylink_ethtool_ksettings_get(priv->phylink, &link_info)) {
		rtnl_unlock();

		return ENETC_MSG_CODE_NOT_SUPPORT;
	}
	rtnl_unlock();

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_SPEED;

	switch (link_info.base.speed) {
	case SPEED_10:
		if (link_info.base.duplex == DUPLEX_HALF)
			speed_code = ENETC_MSG_SPEED_10M_HD;
		else
			speed_code = ENETC_MSG_SPEED_10M_FD;
		break;
	case SPEED_100:
		if (link_info.base.duplex == DUPLEX_HALF)
			speed_code = ENETC_MSG_SPEED_100M_HD;
		else
			speed_code = ENETC_MSG_SPEED_100M_FD;
		break;
	case SPEED_1000:
		speed_code = ENETC_MSG_SPEED_1000M;
		break;
	case SPEED_2500:
		speed_code = ENETC_MSG_SPEED_2500M;
		break;
	case SPEED_5000:
		speed_code = ENETC_MSG_SPEED_5G;
		break;
	default:
		speed_code = enetc_msg_get_speed_code(link_info.base.speed);
	}

	pf_msg.class_code_u8 = speed_code;

	return pf_msg.code;
}

static u16 enetc_msg_handle_link_speed(struct enetc_msg_header *msg_hdr,
				       struct enetc_pf *pf, int vf_id)
{
	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_GET_CURRENT_LINK_SPEED:
		return enetc_msg_pf_reply_link_speed(pf);
	default:
		return ENETC_MSG_CODE_NOT_SUPPORT;
	}
}

static void enetc_msg_handle_rxmsg(struct enetc_pf *pf, int vf_id,
				   u16 *msg_code)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_msg_header *msg_hdr;
	u32 msg_size;

	msg_hdr = (struct enetc_msg_header *)msg_swbd->vaddr;
	msg_size = ENETC_MSG_SIZE(msg_hdr->len);
	if (!enetc_msg_check_crc16(msg_swbd->vaddr, msg_size)) {
		dev_err(dev, "VSI to PSI Message CRC16 error\n");
		*msg_code = ENETC_MSG_CODE_CRC_ERROR;

		return;
	}

	/* Currently, we don't support asynchronous action */
	if (msg_hdr->cookie) {
		dev_err(dev, "Cookie field is not supported yet\n");
		*msg_code = ENETC_MSG_CODE_NOT_SUPPORT;

		return;
	}

	switch (msg_hdr->class_id) {
	case ENETC_MSG_CLASS_ID_MAC_FILTER:
		*msg_code = enetc_msg_handle_mac_filter(msg_hdr, pf, vf_id);
		break;
	case ENETC_MSG_CLASS_ID_IP_REVISION:
		*msg_code = enetc_msg_handle_ip_revision(msg_hdr, pf);
		break;
	case ENETC_MSG_CLASS_ID_LINK_STATUS:
		*msg_code = enetc_msg_handle_link_status(msg_hdr, pf, vf_id);
		break;
	case ENETC_MSG_CLASS_ID_LINK_SPEED:
		*msg_code = enetc_msg_handle_link_speed(msg_hdr, pf, vf_id);
		break;
	case ENETC_MSG_CLASS_ID_VLAN_FILTER:
		*msg_code = enetc_msg_handle_vlan_filter(msg_hdr, pf, vf_id);
		break;
	default:
		*msg_code = ENETC_MSG_CODE_NOT_SUPPORT;
	}
}

static void enetc_msg_task(struct work_struct *work)
{
	struct enetc_si *si = container_of(work, struct enetc_si, msg_task);
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	u32 mr_mask = 0, mr_status;
	int i;

	/* VF FLR support is required for ENEC v4.4 */
	if (!is_enetc_rev1(si)) {
		u32 flr_status = enetc_rd(hw, ENETC_PSIIDR) &
				 PSIIDR_VSI_FLR(pf->num_vfs);

		if (flr_status) {
			u32 pmr = enetc_port_rd(hw, ENETC4_PMR);

			/* ack FLR int (w1c) */
			enetc_wr(hw, ENETC_PSIIDR, flr_status);

			/* Re-enable SInEN bits, disabled by VF FLR */
			enetc_port_wr(hw, ENETC4_PMR, pmr | flr_status);
		}

		/* re-arm FLR interrupts */
		enetc_msg_enable_flr_int(pf, true);
	}

	/* Messaging */
	for (i = 0; i < pf->num_vfs; i++)
		mr_mask |= ENETC_PSIMSGRR_MR(i);

	for (;;) {
		mr_status = enetc_rd(hw, ENETC_PSIMSGRR) & mr_mask;
		if (!mr_status) {
			/* re-arm MR interrupts, w1c the IDR reg */
			enetc_wr(hw, ENETC_PSIIDR, mr_mask);
			enetc_msg_enable_mr_int(pf, true);
			return;
		}

		for (i = 0; i < pf->num_vfs; i++) {
			u32 psimsgrr;
			u16 msg_code;

			if (!(ENETC_PSIMSGRR_MR(i) & mr_status))
				continue;

			enetc_msg_handle_rxmsg(pf, i, &msg_code);

			/* If msg_code is 0, it means that PF has already replied
			 * to VF, and we don't need to reply here.
			 */
			if (!msg_code)
				continue;

			psimsgrr = ENETC_SIMSGSR_SET_MC(msg_code);
			psimsgrr |= ENETC_PSIMSGRR_MR(i); /* w1c */
			enetc_wr(hw, ENETC_PSIMSGRR, psimsgrr);
		}
	}
}

/* Init */
static int enetc_msg_alloc_mbx(struct enetc_si *si, int idx)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct device *dev = &si->pdev->dev;
	struct enetc_hw *hw = &si->hw;
	struct enetc_msg_swbd *msg;
	u32 val;

	msg = &pf->rxmsg[idx];
	/* allocate and set receive buffer */
	msg->size = ENETC_DEFAULT_MSG_SIZE;

	msg->vaddr = dma_alloc_coherent(dev, msg->size, &msg->dma,
					GFP_KERNEL);
	if (!msg->vaddr) {
		dev_err(dev, "msg: fail to alloc dma buffer of size: %d\n",
			msg->size);
		return -ENOMEM;
	}

	/* set multiple of 32 bytes */
	val = lower_32_bits(msg->dma);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR0(idx), val);
	val = upper_32_bits(msg->dma);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR1(idx), val);

	return 0;
}

static void enetc_msg_free_mbx(struct enetc_si *si, int idx)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	struct enetc_msg_swbd *msg;

	msg = &pf->rxmsg[idx];
	dma_free_coherent(&si->pdev->dev, msg->size, msg->vaddr, msg->dma);
	memset(msg, 0, sizeof(*msg));

	enetc_wr(hw, ENETC_PSIVMSGRCVAR0(idx), 0);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR1(idx), 0);
}

#if IS_ENABLED(CONFIG_PCI_IOV)
int enetc_msg_psi_init(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;
	int vector, i, err;

	/* register message passing interrupt handler */
	snprintf(si->msg_int_name, sizeof(si->msg_int_name), "%s-vfmsg",
		 si->ndev->name);
	vector = pci_irq_vector(si->pdev, ENETC_SI_INT_IDX);
	err = request_irq(vector, enetc_msg_psi_msix, 0, si->msg_int_name, si);
	if (err) {
		dev_err(&si->pdev->dev,
			"PSI messaging: request_irq() failed!\n");
		return err;
	}

	/* set one IRQ entry for PSI message receive notification (SI int) */
	enetc_wr(&si->hw, ENETC_SIMSIVR, ENETC_SI_INT_IDX);

	/* initialize PSI mailbox */
	INIT_WORK(&si->msg_task, enetc_msg_task);

	for (i = 0; i < pf->num_vfs; i++) {
		err = enetc_msg_alloc_mbx(si, i);
		if (err)
			goto err_init_mbx;
	}

	/* enable MR interrupts */
	enetc_msg_enable_mr_int(pf, true);

	/* enable VF FLR interrupts */
	if (!is_enetc_rev1(si))
		enetc_msg_enable_flr_int(pf, true);

	return 0;

err_init_mbx:
	for (i--; i >= 0; i--)
		enetc_msg_free_mbx(si, i);

	free_irq(vector, si);

	return err;
}
EXPORT_SYMBOL_GPL(enetc_msg_psi_init);

void enetc_msg_psi_free(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;
	int i;

	cancel_work_sync(&si->msg_task);

	/* disable MR interrupts */
	enetc_msg_enable_mr_int(pf, false);

	/* disable FLR interrupts */
	if (!is_enetc_rev1(si))
		enetc_msg_enable_flr_int(pf, false);

	for (i = 0; i < pf->num_vfs; i++)
		enetc_msg_free_mbx(si, i);

	/* de-register message passing interrupt handler */
	free_irq(pci_irq_vector(si->pdev, ENETC_SI_INT_IDX), si);
}
EXPORT_SYMBOL_GPL(enetc_msg_psi_free);
#endif
