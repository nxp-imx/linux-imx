/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef _NETC_SWITCH_H
#define _NETC_SWITCH_H

#include <linux/dsa/tag_netc.h>
#include <linux/fsl/netc_global.h>
#include <linux/fsl/netc_lib.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/pci.h>

#include "netc_switch_hw.h"

#define NETC_REGS_BAR			0
#define NETC_MSIX_TBL_BAR		2
#define NETC_REGS_PORT_BASE		0x4000
/* register block size per port  */
#define NETC_REGS_PORT_SIZE		0x4000
#define PORT_IOBASE(p)			(NETC_REGS_PORT_SIZE * (p))
#define NETC_REGS_GLOBAL_BASE		0x70000

#define NETC_SWITCH_REV_4_3		0x0403

#define NETC_TC_NUM			8
#define NETC_CBDR_NUM			2

/* read data snoop and command buffer descriptor read snoop, coherent
 * copy of cacheable memory, lookup in downstream cache, no allocate
 * on miss.
 * write data snoop, coherent write of cacheable memory, lookup in
 * downstream cache, no allocate on miss (full cache line update)
 * command buffer descriptor write snoop, coherent write of cacheable
 * memory, lookup in downstream cache, no allocate on miss (partial
 * cache line update or unkonwn)
 */
#define NETC_DEFAULT_CMD_CACHE_ATTR	0x2b2b6727

#define NETC_MAX_FRAME_LEN		9600

#define NETC_STG_STATE_DISABLED		0
#define NETC_STG_STATE_LEARNING		1
#define NETC_STG_STATE_FORWARDING	2

#define NETC_STANDALONE_PVID		0
#define NETC_CPU_PORT_PVID		1
#define NETC_VLAN_UNAWARE_PVID		4095

#define NETC_FDBT_CLEAN_INTERVAL	(3 * HZ)
#define NETC_FDBT_AGING_ACT_CNT		100

#define NETC_DEFULT_BUFF_POOL_MAP0	0x03020100
#define NETC_DEFULT_BUFF_POOL_MAP1	0x07060504

/* The FC_ON threshold is about 3 * NETC_MAX_FRAME_LEN
 * The FC_OFF threshold is about 1 * NETC_MAX_FRAME_LEN
 */
#define NETC_PORT_FC_ON_THRESH		0xb43
#define NETC_PORT_FC_OFF_THRESH		0x3c3

#define NETC_MM_VERIFY_RETRIES		3

/* Software defined host reason */
#define NETC_HR_TRAP			0x8

struct netc_switch_info {
	u32 cpu_port_num;
	u32 usr_port_num;
	u32 tmr_devfn;
	void (*phylink_get_caps)(int port, struct phylink_config *config);
};

struct netc_port_caps {
	u32 half_duplex:1; /* indicates the port whether support half-duplex */
	u32 pmac:1;	  /* indicates the port whether has preemption MAC */
	u32 pseudo_link:1;
};

enum netc_port_offloads {
	NETC_FLAG_QAV			= BIT(0),
	NETC_FLAG_QBU			= BIT(1),
	NETC_FLAG_QBV			= BIT(2),

	NETC_FLAG_TX_TSTAMP		= BIT(9),
	NETC_FLAG_TX_ONESTEP_SYNC	= BIT(10),
};

enum netc_ptp_type {
	NETC_PTP_L2,
	NETC_PTP_L4_IPV4_EVENT,
	NETC_PTP_L4_IPV4_GENERAL,
	NETC_PTP_L4_IPV6_EVENT,
	NETC_PTP_L4_IPV6_GENERAL,
	NETC_PTP_MAX,
};

struct netc_switch;

struct netc_port {
	struct netc_switch *switch_priv;
	struct netc_port_caps caps;
	struct dsa_port *dp;
	struct clk *ref_clk; /* RGMII/RMII reference clock */
	struct net_device *bridge;
	int index;

	void __iomem *iobase;
	struct mii_bus *imdio;
	struct phylink_pcs *pcs;

	u32 speed;
	phy_interface_t phy_mode;

	u16 pvid;
	u16 vlan_aware:1;
	u16 tx_pause:1;

	enum netc_port_offloads offloads;

	/* Serialize access to MAC Merge state between ethtool requests
	 * and link state updates
	 */
	struct mutex mm_lock;
	unsigned long preemptible_tcs;

	/* Timestamp Request Identifier lock */
	spinlock_t ts_req_id_lock;
	/* skb queue for two-step timestamp frames */
	struct sk_buff_head skb_txtstamp_queue;
	int ptp_filter;
	u32 ptp_ipft_eid[NETC_PTP_MAX];
};

enum netc_port_mac {
	NETC_PORT_EMAC = 0,
	NETC_PORT_PMAC,
};

struct netc_switch_regs {
	void __iomem *base;
	void __iomem *port;
	void __iomem *global;
};

struct netc_switch_caps {
	int num_bp;
	int num_sbp;
};

struct netc_switch_dbgfs {
	u32 port;
	u16 vft_vid;
	u32 ett_eid;
	u32 fmt_eid;
	u32 ect_eid;
	u32 isit_eid;
	u32 ist_eid;
	u32 isft_eid;
	u32 sgit_eid;
	u32 sgclt_eid;
	u32 isct_eid;
	u32 rpt_eid;
	u32 ipft_eid;
};

struct netc_switch {
	struct pci_dev *pdev;
	struct device *dev;
	struct dsa_switch *ds;
	u16 revision;

	const struct netc_switch_info *info;
	struct netc_switch_regs regs;
	enum dsa_tag_protocol tag_proto;
	struct netc_port **ports;
	u32 num_ports;

	struct ntmp_priv ntmp;

	struct hlist_head fdb_list;
	struct hlist_head vlan_list;
	struct mutex fdbt_lock; /* FDB table lock */
	struct mutex vft_lock; /* VLAN filter table lock */
	struct delayed_work fdbt_clean;
	/* interval times act_cnt is aging time */
	unsigned long fdbt_acteu_interval;
	u8 fdbt_aging_act_cnt; /* maximum is 127 */

	struct netc_switch_caps caps;
	struct bpt_cfge_data *bpt_list;
	struct mutex bpt_lock; /* buffer pool table lock */

	struct netc_switch_dbgfs dbg_params;
	struct dentry *debugfs_root;
};

#define NETC_PRIV(ds)			((struct netc_switch *)((ds)->priv))
#define NETC_PORT(priv, port_id)	((priv)->ports[(port_id)])

struct netc_fdb_entry {
	u32 entry_id;
	struct fdbt_cfge_data cfge;
	struct fdbt_keye_data keye;
	struct hlist_node node;
};

struct netc_vlan_entry {
	u16 vid;
	u32 entry_id;
	u32 ect_base_eid;
	u32 untagged_port_bitmap;
	struct vft_cfge_data cfge;
	struct hlist_node node;
};

/* Generic interfaces for writing/reading Switch registers */
#define netc_reg_rd(addr)		netc_read(addr)
#define netc_reg_wr(addr, v)		netc_write(addr, v)

/* Write/Read Switch base registers */
#define netc_base_rd(r, o)		netc_read((r)->base + (o))
#define netc_base_wr(r, o, v)		netc_write((r)->base + (o), v)

/* Write/Read registers of Switch Port (including pseudo MAC port) */
#define netc_port_rd(p, o)		netc_read((p)->iobase + (o))
#define netc_port_rd64(p, o)		netc_read64((p)->iobase + (o))
#define netc_port_wr(p, o, v)		netc_write((p)->iobase + (o), v)

/* Write/Read Switch global registers */
#define netc_glb_rd(r, o)		netc_read((r)->global + (o))
#define netc_glb_wr(r, o, v)		netc_write((r)->global + (o), v)

#define ntmp_to_netc_switch(ntmp_priv)	\
	container_of((ntmp_priv), struct netc_switch, ntmp)

int netc_switch_platform_probe(struct netc_switch *priv);
void netc_port_set_tx_pause(struct netc_port *port, bool tx_pause);
void netc_port_set_all_tc_msdu(struct netc_port *port, u32 *max_sdu);
struct pci_dev *netc_switch_get_timer(struct netc_switch *priv);
void netc_mac_port_wr(struct netc_port *port, u32 reg, u32 val);

/* TC APIs */
int netc_tc_query_caps(struct tc_query_caps_base *base);
int netc_tc_setup_mqprio(struct netc_switch *priv, int port,
			 struct tc_mqprio_qopt_offload *mqprio);
int netc_tc_setup_cbs(struct netc_switch *priv, int port,
		      struct tc_cbs_qopt_offload *cbs);
int netc_tc_setup_taprio(struct netc_switch *priv, int port,
			 struct tc_taprio_qopt_offload *taprio);
int netc_port_flow_cls_replace(struct netc_port *port,
			       struct flow_cls_offload *f);
int netc_port_flow_cls_destroy(struct netc_port *port,
			       struct flow_cls_offload *f);
int netc_port_flow_cls_stats(struct netc_port *port,
			     struct flow_cls_offload *f);
void netc_destroy_flower_list(struct netc_switch *priv);

/* ethtool APIs */
void netc_port_mm_commit_preemptible_tcs(struct netc_port *port);
int netc_port_get_mm(struct dsa_switch *ds, int port_id,
		     struct ethtool_mm_state *state);
int netc_port_set_mm(struct dsa_switch *ds, int port_id,
		     struct ethtool_mm_cfg *cfg,
		     struct netlink_ext_ack *extack);
void netc_port_get_mm_stats(struct dsa_switch *ds, int port_id,
			    struct ethtool_mm_stats *stats);
void netc_port_get_pause_stats(struct dsa_switch *ds, int port_id,
			       struct ethtool_pause_stats *pause_stats);
void netc_port_get_rmon_stats(struct dsa_switch *ds, int port_id,
			      struct ethtool_rmon_stats *rmon_stats,
			      const struct ethtool_rmon_hist_range **ranges);
void netc_port_get_eth_ctrl_stats(struct dsa_switch *ds, int port_id,
				  struct ethtool_eth_ctrl_stats *ctrl_stats);
void netc_port_get_eth_mac_stats(struct dsa_switch *ds, int port_id,
				 struct ethtool_eth_mac_stats *mac_stats);
void netc_port_get_strings(struct dsa_switch *ds, int port_id, u32 sset, u8 *data);
void netc_port_get_ethtool_stats(struct dsa_switch *ds, int port_id, u64 *data);
int netc_port_get_sset_count(struct dsa_switch *ds, int port_id, int sset);

/* PTP APIs */
int netc_get_ts_info(struct dsa_switch *ds, int port_id,
		     struct kernel_ethtool_ts_info *info);
int netc_port_hwtstamp_set(struct dsa_switch *ds, int port_id,
			   struct ifreq *ifr);
int netc_port_hwtstamp_get(struct dsa_switch *ds, int port_id,
			   struct ifreq *ifr);
bool netc_port_rxtstamp(struct dsa_switch *ds, int port,
			struct sk_buff *skb, unsigned int type);
void netc_port_txtstamp(struct dsa_switch *ds, int port_id,
			struct sk_buff *skb);

#if IS_ENABLED(CONFIG_DEBUG_FS)
void netc_create_debugfs(struct netc_switch *priv);
void netc_remove_debugfs(struct netc_switch *priv);
#else
static inline void netc_create_debugfs(struct netc_switch *priv)
{
}

static inline void netc_remove_debugfs(struct netc_switch *priv)
{
}
#endif

static inline bool is_netc_pseudo_port(struct netc_port *port)
{
	return port->caps.pseudo_link;
}

#endif
