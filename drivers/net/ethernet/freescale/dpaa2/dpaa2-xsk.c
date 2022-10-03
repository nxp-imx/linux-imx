// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2014-2016 Freescale Semiconductor Inc.
 * Copyright 2016-2022 NXP
 */
#include <linux/filter.h>
#include <linux/compiler.h>
#include <linux/bpf_trace.h>
#include <net/xdp.h>
#include <net/xdp_sock_drv.h>

#include "dpaa2-eth.h"

static u32 dpaa2_xsk_run_xdp(struct dpaa2_eth_priv *priv,
			     struct dpaa2_eth_channel *ch,
			     struct dpaa2_eth_fq *rx_fq,
			     struct dpaa2_fd *fd, void *vaddr)
{
	dma_addr_t addr = dpaa2_fd_get_addr(fd);
	struct bpf_prog *xdp_prog;
	struct xdp_buff *xdp_buff;
	struct dpaa2_eth_swa *swa;
	u32 xdp_act = XDP_PASS;
	int err;

	xdp_prog = READ_ONCE(ch->xdp.prog);
	if (!xdp_prog)
		goto out;

	swa = (struct dpaa2_eth_swa *)(vaddr + DPAA2_ETH_RX_HWA_SIZE +
				       ch->xsk_pool->umem->headroom);
	xdp_buff = swa->xsk.xdp_buff;

	xdp_buff->data_hard_start = vaddr;
	xdp_buff->data = vaddr + dpaa2_fd_get_offset(fd);
	xdp_buff->data_end = xdp_buff->data + dpaa2_fd_get_len(fd);
	xdp_set_data_meta_invalid(xdp_buff);
	xdp_buff->rxq = &ch->xdp_rxq;

	xsk_buff_dma_sync_for_cpu(xdp_buff, ch->xsk_pool);
	xdp_act = bpf_prog_run_xdp(xdp_prog, xdp_buff);

	/* xdp.data pointer may have changed */
	dpaa2_fd_set_offset(fd, xdp_buff->data - vaddr);
	dpaa2_fd_set_len(fd, xdp_buff->data_end - xdp_buff->data);

	if (likely(xdp_act == XDP_REDIRECT)) {
		err = xdp_do_redirect(priv->net_dev, xdp_buff, xdp_prog);
		if (unlikely(err)) {
			ch->stats.xdp_drop++;
			dpaa2_eth_recycle_buf(priv, ch, addr);
		} else {
			ch->buf_count--;
			ch->stats.xdp_redirect++;
		}

		goto xdp_redir;
	}

	switch (xdp_act) {
	case XDP_PASS:
		break;
	case XDP_TX:
		dpaa2_eth_xdp_enqueue(priv, ch, fd, vaddr, rx_fq->flowid);
		break;
	default:
		bpf_warn_invalid_xdp_action(xdp_act);
		fallthrough;
	case XDP_ABORTED:
		trace_xdp_exception(priv->net_dev, xdp_prog, xdp_act);
		fallthrough;
	case XDP_DROP:
		dpaa2_eth_recycle_buf(priv, ch, addr);
		ch->stats.xdp_drop++;
		break;
	}

xdp_redir:
	ch->xdp.res |= xdp_act;
out:
	return xdp_act;
}

/* Rx frame processing routine for the AF_XDP fast path */
static void dpaa2_xsk_rx(struct dpaa2_eth_priv *priv,
			 struct dpaa2_eth_channel *ch,
			 const struct dpaa2_fd *fd,
			 struct dpaa2_eth_fq *fq)
{
	dma_addr_t addr = dpaa2_fd_get_addr(fd);
	u8 fd_format = dpaa2_fd_get_format(fd);
	struct rtnl_link_stats64 *percpu_stats = NULL;
	u32 fd_length = dpaa2_fd_get_len(fd);
	struct sk_buff *skb;
	void *vaddr;
	u32 xdp_act;

	/* Tracing point */
	trace_dpaa2_rx_xsk_fd(priv->net_dev, fd);

	vaddr = dpaa2_iova_to_virt(priv->iommu_domain, addr);
	percpu_stats = this_cpu_ptr(priv->percpu_stats);

	if (fd_format != dpaa2_fd_single) {
		WARN_ON(priv->xdp_prog);
		/* AF_XDP doesn't support any other formats */
		goto err_frame_format;
	}

	xdp_act = dpaa2_xsk_run_xdp(priv, ch, fq, (struct dpaa2_fd *)fd, vaddr);
	if (xdp_act != XDP_PASS) {
		percpu_stats->rx_packets++;
		percpu_stats->rx_bytes += dpaa2_fd_get_len(fd);
		return;
	}

	/* Build skb */
	skb = dpaa2_eth_alloc_skb(priv, ch, fd, fd_length, vaddr);
	if (!skb)
		/* Nothing else we can do, recycle the buffer and drop the frame */
		goto err_alloc_skb;

	/* Send the skb to the Linux networking stack */
	dpaa2_eth_receive_skb(priv, ch, fd, vaddr, fq, percpu_stats, skb);
	return;

err_alloc_skb:
	dpaa2_eth_recycle_buf(priv, ch, addr);
err_frame_format:
	percpu_stats->rx_dropped++;
}

static void dpaa2_xsk_set_bp_per_qdbin(struct dpaa2_eth_priv *priv,
				       struct dpni_pools_cfg *pools_params)
{
	int curr_bp = 0, i, j;

	pools_params->pool_options = DPNI_POOL_ASSOC_QDBIN;
	for (i = 0; i < priv->num_bps; i++) {
		for (j = 0; j < priv->num_channels; j++)
			if (priv->bp[i] == priv->channel[j]->bp)
				pools_params->pools[curr_bp].priority_mask |= (1 << j);
		if (!pools_params->pools[curr_bp].priority_mask)
			continue;

		pools_params->pools[curr_bp].dpbp_id = priv->bp[i]->bpid;
		pools_params->pools[curr_bp].buffer_size = priv->rx_buf_size;
		pools_params->pools[curr_bp++].backup_pool = 0;
	}
	pools_params->num_dpbp = curr_bp;
}

static int dpaa2_xsk_disable_pool(struct net_device *dev, u16 qid)
{
	struct xsk_buff_pool *pool = xsk_get_pool_from_qid(dev, qid);
	struct dpaa2_eth_priv *priv = netdev_priv(dev);
	struct dpni_pools_cfg pools_params = { 0 };
	int i, err;
	bool up;

	up = netif_running(dev);
	if (up)
		dpaa2_eth_stop(dev);

	xsk_pool_dma_unmap(pool, 0);
	err = xdp_rxq_info_reg_mem_model(&priv->channel[qid]->xdp_rxq,
					 MEM_TYPE_PAGE_ORDER0, NULL);
	if (err)
		netdev_err(dev, "xsk_rxq_info_reg_mem_model() failed (err = %d)\n",
			   err);

	dpaa2_eth_free_dpbp(priv, priv->channel[qid]->bp);

	priv->channel[qid]->xsk_zc = false;
	priv->channel[qid]->xsk_pool = NULL;
	priv->channel[qid]->bp = priv->bp[DPAA2_ETH_DEFAULT_BP];

	/* Restore Rx callback to slow path */
	for (i = 0; i < priv->num_fqs; i++) {
		if (priv->fq[i].type != DPAA2_RX_FQ)
			continue;

		priv->fq[i].consume = dpaa2_eth_rx;
	}

	dpaa2_xsk_set_bp_per_qdbin(priv, &pools_params);
	err = dpni_set_pools(priv->mc_io, 0, priv->mc_token, &pools_params);
	if (err)
		netdev_err(dev, "dpni_set_pools() failed\n");

	if (up) {
		err = dpaa2_eth_open(dev);
		if (err)
			return err;
	}

	return 0;
}

static int dpaa2_xsk_enable_pool(struct net_device *dev,
				 struct xsk_buff_pool *pool,
				 u16 qid)
{
	struct dpaa2_eth_priv *priv = netdev_priv(dev);
	struct dpni_pools_cfg pools_params = { 0 };
	int i, err, err2;
	bool up;

	if (priv->dpni_attrs.wriop_version != DPAA2_WRIOP_VERSION(3, 0, 0))
		return -EOPNOTSUPP;

	if (priv->dpni_attrs.num_queues > 8) {
		netdev_err(dev, "Create a DPNI with maximum 8 queues for AF_XDP\n");
		return -EOPNOTSUPP;
	}

	up = netif_running(dev);
	if (up)
		dpaa2_eth_stop(dev);

	err = xsk_pool_dma_map(pool, priv->net_dev->dev.parent, 0);
	if (err) {
		netdev_err(dev, "xsk_pool_dma_map() failed (err = %d)\n",
			   err);
		goto err_dma_unmap;
	}

	err = xdp_rxq_info_reg_mem_model(&priv->channel[qid]->xdp_rxq,
					 MEM_TYPE_XSK_BUFF_POOL, NULL);
	if (err) {
		netdev_err(dev, "xdp_rxq_info_reg_mem_model() failed (err = %d)\n", err);
		goto err_mem_model;
	}
	xsk_pool_set_rxq_info(pool, &priv->channel[qid]->xdp_rxq);

	priv->bp[priv->num_bps] = dpaa2_eth_allocate_dpbp(priv);
	if (IS_ERR(priv->bp[priv->num_bps])) {
		err = PTR_ERR(priv->bp[priv->num_bps]);
		goto err_bp_alloc;
	}
	priv->channel[qid]->xsk_zc = true;
	priv->channel[qid]->xsk_pool = pool;
	priv->channel[qid]->bp = priv->bp[priv->num_bps++];

	/* Set Rx callback to AF_XDP fast path */
	for (i = 0; i < priv->num_fqs; i++) {
		if (priv->fq[i].type != DPAA2_RX_FQ)
			continue;

		priv->fq[i].consume = dpaa2_xsk_rx;
	}

	dpaa2_xsk_set_bp_per_qdbin(priv, &pools_params);
	err = dpni_set_pools(priv->mc_io, 0, priv->mc_token, &pools_params);
	if (err) {
		netdev_err(dev, "dpni_set_pools() failed\n");
		goto err_set_pools;
	}

	if (up) {
		err = dpaa2_eth_open(dev);
		if (err)
			return err;

		while (!READ_ONCE(priv->link_state.up))
			cpu_relax();
	}

	return 0;

err_set_pools:
	err2 = dpaa2_xsk_disable_pool(dev, qid);
	if (err2)
		netdev_err(dev, "dpaa2_xsk_disable_pool() failed %d\n", err2);
err_bp_alloc:
	err2 = xdp_rxq_info_reg_mem_model(&priv->channel[qid]->xdp_rxq,
					  MEM_TYPE_PAGE_ORDER0, NULL);
	if (err2)
		netdev_err(dev, "xsk_rxq_info_reg_mem_model() failed with %d)\n", err2);
err_mem_model:
	xsk_pool_dma_unmap(pool, 0);
err_dma_unmap:
	if (up)
		dpaa2_eth_open(dev);

	return err;
}

int dpaa2_xsk_setup_pool(struct net_device *dev, struct xsk_buff_pool *pool, u16 qid)
{
	return pool ? dpaa2_xsk_enable_pool(dev, pool, qid) :
		      dpaa2_xsk_disable_pool(dev, qid);
}

int dpaa2_xsk_wakeup(struct net_device *dev, u32 qid, u32 flags)
{
	return 0;
}
