/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2022 NXP */
#ifndef __NETC_NTMP_H
#define __NETC_NTMP_H

#include <linux/bitops.h>
#include <net/tc_act/tc_gate.h>

#define NTMP_NULL_ENTRY_ID		0xffffffffU
#define NETC_CBDR_BD_NUM		256

#define ISIT_FRAME_KEY_LEN		16
#define IPFT_MAX_PLD_LEN		24
#define FDBT_MAX_ACT_CNT		0x7f

/* NTMP errata */
#define NTMP_ERR052134			BIT(0)

#pragma pack(1)

/* The format of conctrol buffer descriptor */
union netc_cbd {
	struct {
		__le64 addr;
		__le32 len;
#define NTMP_RESP_LEN		GENMASK(19, 0)
#define NTMP_REQ_LEN		GENMASK(31, 20)
#define NTMP_LEN(req, resp)	(FIELD_PREP(NTMP_REQ_LEN, (req)) | \
				((resp) & NTMP_RESP_LEN))
		u8 cmd;
#define NTMP_CMD_DELETE		BIT(0)
#define NTMP_CMD_UPDATE		BIT(1)
#define NTMP_CMD_QUERY		BIT(2)
#define NTMP_CMD_ADD		BIT(3)
#define NTMP_CMD_QD		(NTMP_CMD_QUERY | NTMP_CMD_DELETE)
#define NTMP_CMD_QU		(NTMP_CMD_QUERY | NTMP_CMD_UPDATE)
#define NTMP_CMD_AU		(NTMP_CMD_ADD | NTMP_CMD_UPDATE)
#define NTMP_CMD_AQ		(NTMP_CMD_ADD | NTMP_CMD_QUERY)
#define NTMP_CMD_AQU		(NTMP_CMD_AQ | NTMP_CMD_UPDATE)
		u8 access_method;
#define NTMP_ACCESS_METHOD	GENMASK(7, 4)
#define NTMP_AM_ENTRY_ID	0
#define NTMP_AM_EXACT_KEY	1
#define NTMP_AM_SEARCH		2
#define NTMP_AM_TERNARY_KEY	3
		u8 table_id;
		u8 ver_cci_rr;
#define NTMP_HDR_VERSION	GENMASK(5, 0)
#define NTMP_HDR_VER2		2
#define NTMP_CCI		BIT(6)
#define NTMP_RR			BIT(7)
		__le32 resv2[3];
		__le32 npf;
#define NTMP_NPF		BIT(15)
	} req_hdr;	/* NTMP Request Message Header Format */

	struct {
		__le32 resv1[3];
		__le16 num_matched;
		__le16 error_rr;
#define NTMP_RESP_ERROR		GENMASK(11, 0)
#define NTMP_RESP_RR		BIT(15)
		__le32 resv3[4];
	} resp_hdr; /* NTMP Response Message Header Format */
};

struct maft_keye_data {
	u8 mac_addr[ETH_ALEN];
	__le16 resv;
};

struct maft_cfge_data {
	__le16 si_bitmap;
	__le16 resv;
};

struct vaft_keye_data {
	__le16 vlan_id;
#define VAFT_VLAN_ID		GENMASK(11, 0)
	u8 tpid;
#define VAFT_TPID		GENMASK(1, 0)
	u8 resv;
};

struct vaft_cfge_data {
	__le16 si_bitmap;
	__le16 resv;
};

struct rfst_keye_data {
	__le32 resv0[6];
	__be32 source_ip_addr[4];
	__be32 source_ip_addr_mask[4];
	__be32 dest_ip_addr[4];
	__be32 dest_ip_addr_mask[4];
	__le32 resv1[2];
	__be16 l4_source_port;
	__be16 l4_source_port_mask;
	__be16 l4_dest_port;
	__be16 l4_dest_port_mask;
	__le32 resv2;
	u8 l4_protocol;
	u8 l4_protocol_mask;
	__le16 l3_l4_protocol;
#define RFST_IP_PRESENT			BIT(2)
#define RFST_IP_PRESENT_MASK		BIT(3)
#define RFST_L4_PROTOCOL_PRESENT	BIT(4)
#define RFST_L4_PROTOCOL_PRESENT_MASK	BIT(5)
#define RFST_TCP_OR_UDP_PRESENT		BIT(6)
#define RFST_TCP_OR_UDP_PRESENT_MASK	BIT(7)
#define RFST_IPV4_IPV6			BIT(8)
#define RFST_IPV4_IPV6_MASK		BIT(9)
#define RFST_UDP_TCP			BIT(10)
#define RFST_UDP_TCP_MASK		BIT(11)
};

struct rfst_cfge_data {
	__le32 cfg;
#define RFST_RESULT		GENMASK(7, 0)
#define RFST_MODE		GENMASK(17, 16)
};

struct isit_keye_data {
	__le32 key_aux;
#define ISIT_KEY_TYPE			GENMASK(1, 0)
#define ISIT_KEY_TYPE0_SMAC_VLAN	0
#define ISIT_KEY_TYPE1_DMAC_VLAN	1
#define ISIT_SRC_PORT_ID		GENMASK(6, 2)
#define ISIT_SPM			BIT(7)
	u8 frame_key[ISIT_FRAME_KEY_LEN];
};

struct ist_cfge_data {
	__le32 cfg;
#define IST_SFE			BIT(0)
#define IST_RRT			BIT(1)
#define IST_BL2F		BIT(2)
#define IST_IPV			GENMASK(7, 4)
#define	IST_OIPV		BIT(8)
#define IST_DR			GENMASK(10, 9)
#define IST_ODR			BIT(11)
#define IST_IMIRE		BIT(12)
#define IST_TIMERCAPE		BIT(13)
#define IST_SPPD		BIT(15)
#define IST_ISQGA		GENMASK(17, 16)
#define IST_ORP			BIT(18)
#define IST_OSGI		BIT(19)
#define IST_HR			GENMASK(23, 20)
/* VERSION 0 */
#define IST_V0_FA		GENMASK(26, 24)
#define IST_V0_SDU_TYPE		GENMASK(28, 27)
/* VERSION 1 */
#define IST_V1_FA		GENMASK(27, 24)
#define	IST_V1_SDU_TYPE		GENMASK(29, 28)
#define IST_FA_NO_SI_BITMAP	1
#define IST_SWITCH_FA_SF	2
#define IST_SWITCH_FA_BF	3
#define IST_SWITCH_FA_SF_COPY	4
#define IST_SDFA		BIT(30)
#define IST_OSDFA		BIT(31)
	__le16 msdu;
	__le16 switch_cfg; /* Only applicable to NETC switch */
#define IST_IFME_LEN_CHANGE	GENMASK(6, 0)
#define	IST_EPORT		GENMASK(11, 7)
#define IST_OETEID		GENMASK(13, 12)
#define IST_CTD			GENMASK(15, 14)
	__le32 isqg_eid; /* Only applicable to NETC switch */
	__le32 rp_eid;
	__le32 sgi_eid;
	__le32 ifm_eid; /* Only applicable to NETC switch */
	__le32 et_eid; /* Only applicable to NETC switch */
	__le32 isc_eid;
	__le32 bitmap_evmeid; /* Only applicable to NETC switch */
#define IST_EGRESS_PORT_BITMAP	GENMASK(23, 0)
#define IST_EVMEID		GENMASK(27, 24)
	__le16 si_bitmap;
};

struct isft_keye_data {
	__le32 is_eid;
	u8 pcp;
#define ISFT_PCP		GENMASK(2, 0)
	u8 resv[3];
};

struct isft_cfge_data {
	__le16 cfg;
#define ISFT_IPV		GENMASK(3, 0)
#define ISFT_OIPV		BIT(4)
#define ISFT_DR			GENMASK(6, 5)
#define ISFT_ODR		BIT(7)
#define ISFT_IMIRE		BIT(8)
#define ISFT_TIMECAPE		BIT(9)
#define ISFT_OSGI		BIT(10)
#define ISFT_CTD		BIT(11)
#define ISFT_ORP		BIT(12)
#define ISFT_SDU_TYPE		GENMASK(14, 13)
	__le16 msdu;
	__le32 rp_eid;
	__le32 sgi_eid;
	__le32 isc_eid;
};

struct sgit_acfge_data {
	__le32 admin_sgcl_eid;
	__le64 admin_base_time;
	__le32 admin_cycle_time_ext;
};

struct sgit_cfge_data {
	u8 cfg;
#define SGIT_OEXEN		BIT(0)
#define SGIT_IRXEN		BIT(1)
#define SGIT_SDU_TYPE		GENMASK(3, 2)
};

struct sgit_icfge_data {
	u8 icfg;
#define SGIT_IPV		GENMASK(3, 0)
#define SGIT_OIPV		BIT(4)
#define SGIT_GST		BIT(5)
#define SGIT_CTD		BIT(6)
};

struct sgit_sgise_data {
	__le32 oper_sgcl_eid;
	__le64 config_change_time;
	__le64 oper_base_time;
	__le32 oper_cycle_time_ext;
	u8 info;
#define SGIT_OEX		BIT(0)
#define SGIT_IRX		BIT(1)
#define SGIT_STATE		GENMASK(4, 2)
};

struct sgclt_ge {
	__le32 interval;
	__le32 cfg;
#define SGCLT_IOM		GENMASK(23, 0)
#define SGCLT_IPV		GENMASK(27, 24)
#define SGCLT_OIPV		BIT(28)
#define SGCLT_CTD		BIT(29)
#define SGCLT_IOMEN		BIT(30)
#define SGCLT_GTST		BIT(31)
};

struct sgclt_cfge_data {
	__le32 cycle_time;
	u8 list_length;
	u8 resv0;
	u8 ext_cfg;
#define SGCLT_EXT_OIPV		BIT(0)
#define SGCLT_EXT_IPV		GENMASK(4, 1)
#define SGCLT_EXT_CTD		BIT(5)
#define SGCLT_EXT_GTST		BIT(6)
	u8 resv1;
	struct sgclt_ge ge[];
};

struct rpt_cfge_data {
	__le32 cir;
	__le32 cbs;
	__le32 eir;
	__le32 ebs;
	__le16 cfg;
#define RPT_MREN	BIT(0)
#define RPT_DOY		BIT(1)
#define RPT_CM		BIT(2)
#define RPT_CF		BIT(3)
#define RPT_NDOR	BIT(4)
#define RPT_SDU_TYPE	GENMASK(6, 5)
};

struct rpt_fee_data {
	u8 fen;
#define RPT_FEN		BIT(0)
};

struct rpt_stse_data {
	__le64 byte_count;
	__le32 drop_frames;
	__le32 rev0;
	__le32 dr0_grn_frames;
	__le32 rev1;
	__le32 dr1_grn_frames;
	__le32 rev2;
	__le32 dr2_ylw_frames;
	__le32 rev3;
	__le32 remark_ylw_frames;
	__le32 rev4;
	__le32 dr3_red_frames;
	__le32 rev5;
	__le32 remark_red_frames;
	__le32 rev6;
	__le32 lts;
	__le32 bci;
	__le32 bcf_bcs;
#define RPT_BCF		GENMASK(30, 0)
#define RPT_BCS		BIT(31)
	__le32 bei;
	__le32 bef_bes;
#define RPT_BEF		GENMASK(30, 0)
#define RPT_BES		BIT(31)
};

struct rpt_pse_data {
	u8 mr;
#define RPT_MR		BIT(0)
};

struct isct_stse_data {
	__le32 rx_count;
	__le32 resv0;
	__le32 msdu_drop_count;
	__le32 resv1;
	__le32 policer_drop_count;
	__le32 resv2;
	__le32 sg_drop_count;
	__le32 resv3;
};

struct ipft_pld_byte {
	u8 data;
	u8 mask;
};

struct ipft_keye_data {
	__le16 precedence;
	__le16 resv0[3];
	__le16 frm_attr_flags;
#define IPFT_FAF_OVLAN		BIT(2)
#define IPFT_FAF_IVLAN		BIT(3)
#define IPFT_FAF_IP_HDR		BIT(7)
#define IPFT_FAF_IP_VER6	BIT(8)
#define IPFT_FAF_L4_CODE	GENMASK(11, 10)
#define  IPFT_FAF_TCP_HDR	1
#define  IPFT_FAF_UDP_HDR	2
#define  IPFT_FAF_SCTP_HDR	3
#define IPFT_FAF_WOL_MAGIC	BIT(12)
	__le16 frm_attr_flags_mask;
	__le16 dscp;
#define IPFT_DSCP		GENMASK(5, 0)
#define IPFT_DSCP_MASK		GENMASK(11, 0)
#define IPFT_DSCP_MASK_ALL	0x3f
	__le16 src_port; /* This field is reserved for ENETC */
#define	IPFT_SRC_PORT		GENMASK(4, 0)
#define IPFT_SRC_PORT_MASK	GENMASK(9, 5)
	__be16 outer_vlan_tci;
	__be16 outer_vlan_tci_mask;
	u8 dmac[ETH_ALEN];
	u8 dmac_mask[ETH_ALEN];
	u8 smac[ETH_ALEN];
	u8 smac_mask[ETH_ALEN];
	__be16 inner_vlan_tci;
	__be16 inner_vlan_tci_mask;
	__be16 ethertype;
	__be16 ethertype_mask;
	u8 ip_protocol;
	u8 ip_protocol_mask;
	__le16 resv1[7];
	__be32 ip_src[4];
	__le32 resv2[2];
	__be32 ip_src_mask[4];
	__be16 l4_src_port;
	__be16 l4_src_port_mask;
	__le32 resv3;
	__be32 ip_dst[4];
	__le32 resv4[2];
	__be32 ip_dst_mask[4];
	__be16 l4_dst_port;
	__be16 l4_dst_port_mask;
	__le32 resv5;
	struct ipft_pld_byte byte[IPFT_MAX_PLD_LEN];
};

struct ipft_cfge_data {
	__le32 cfg;
#define IPFT_IPV		GENMASK(3, 0)
#define IPFT_OIPV		BIT(4)
#define IPFT_DR			GENMASK(6, 5)
#define IPFT_ODR		BIT(7)
#define IPFT_FLTFA		GENMASK(10, 8)
#define  IPFT_FLTFA_DISCARD	0
#define  IPFT_FLTFA_PERMIT	1
/* Redirect is only for switch */
#define  IPFT_FLTFA_REDIRECT	2
#define IPFT_IMIRE		BIT(11)
#define IPFT_WOLTE		BIT(12)
#define IPFT_FLTA		GENMASK(14, 13)
#define  IPFT_FLTA_RP		1
#define  IPFT_FLTA_IS		2
#define  IPFT_FLTA_SI_BITMAP	3
#define IPFT_RPR		GENMASK(16, 15)
#define IPFT_CTD		BIT(17)
#define IPFT_HR			GENMASK(21, 18)
#define IPFT_TIMECAPE		BIT(22)
#define IPFT_RRT		BIT(23)
#define IPFT_BL2F		BIT(24)
#define IPFT_EVMEID		GENMASK(31, 28)
	__le32 flta_tgt;
};

struct fdbt_keye_data {
	u8 mac_addr[ETH_ALEN]; /* big-endian */
	__le16 resv0;
	__le16 fid;
#define FDBT_FID		GENMASK(11, 0)
	__le16 resv1;
};

struct fdbt_cfge_data {
	__le32 port_bitmap;
#define FDBT_PORT_BITMAP	GENMASK(23, 0)
	__le32 cfg;
#define FDBT_OETEID		GENMASK(1, 0)
#define FDBT_EPORT		GENMASK(6, 2)
#define FDBT_IMIRE		BIT(7)
#define FDBT_CTD		GENMASK(10, 9)
#define FDBT_DYNAMIC		BIT(11)
#define FDBT_TIMECAPE		BIT(12)
	__le32 et_eid;
};

struct fdbt_acte_data {
	u8 act;
#define FDBT_ACT_CNT		GENMASK(6, 0)
#define FDBT_ACT_FLAG		BIT(7)
};

struct vft_cfge_data {
	__le32 bitmap_stg;
#define	VFT_PORT_MEMBERSHIP	GENMASK(23, 0)
#define	VFT_STG_ID_MASK		GENMASK(27, 24)
#define VFT_STG_ID(g)		FIELD_PREP(VFT_STG_ID_MASK, (g))
	__le16 fid;
#define VFT_FID			GENMASK(11, 0)
	__le16 cfg;
#define VFT_MLO			GENMASK(2, 0)
#define VFT_MFO			GENMASK(4, 3)
#define VFT_IPMFE		BIT(6)
#define VFT_IPMFLE		BIT(7)
#define VFT_PGA			BIT(8)
#define VFT_SFDA		BIT(10)
#define VFT_OSFDA		BIT(11)
#define VFT_FDBAFSS		BIT(12)
	__le32 eta_port_bitmap;
#define VFT_ETA_PORT_BITMAP	GENMASK(23, 0)
	__le32 et_eid;
};

struct ett_cfge_data {
	__le16 efm_cfg;
#define	ETT_EFM_MODE		GENMASK(1, 0)
#define ETT_ESQA		GENMASK(5, 4)
#define ETT_ECA			GENMASK(8, 6)
#define ETT_ECA_INC		1
#define ETT_EFM_LEN_CHANGE	GENMASK(15, 9)
#define ETT_FRM_LEN_DEL_VLAN	0x7c
	__le16 efm_data_len;
#define ETT_EFM_DATA_LEN	GENMASK(10, 0)
	__le32 efm_eid;
	__le32 ec_eid;
	__le32 esqa_tgt_eid;
};

struct esrt_cfge_data {
	__le32 sqr_cfg;
#define ESRT_SQ_TAG		GENMASK(2, 0)
#define ESRT_SQR_TNSQ		BIT(3)
#define ESRT_SQR_ALG		BIT(4)
#define ESRT_SQR_TYPE		BIT(5)
#define ESRT_SQR_HL		GENMASK(14, 8)
#define ESRT_SQR_FWL		GENMASK(27, 16)
	__le32	sqr_tp;
#define SRT_SQR_TP		GENMASK(11, 0)
};

struct esrt_stse_data {
	__le64 in_order_packets;
	__le64 out_of_order_packets;
	__le64 rogue_packets;
	__le64 duplicate_packets;
	__le64 lost_packets;
	__le64 tagless_packets;
	__le32 srec_resets;
};

struct esrt_srse_data {
	__le16 sqr_num;
	__le16 ts_lce_take;
#define ESRT_TAKE_ANY		BIT(0)
#define ESRT_LCE		BIT(1)
#define ESRT_SQR_TS		GENMASK(13, 2)
	__le32 sqr_history[4];
};

struct ect_stse_data {
	__le64 enq_frm_cnt;
	__le64 rej_frm_cnt;
};

struct fmt_cfge_data {
	__le16 act0;
#define FMT_L2_ACT		BIT(0)
#define FMT_MAC_HDR_ACT		GENMASK(3, 1)
#define FMT_VLAN_HDR_ACT	GENMASK(5, 4)
#define FMT_OUTER_VID_ACT	GENMASK(7, 6)
#define FMT_SQT_ACT		GENMASK(10, 8)
#define FMT_SMAC_PORT		GENMASK(15, 11)
	u8 dest_mac_addr[ETH_ALEN]; /* big-endian */
	__le16 outer_vlan;
#define	FMT_OUTER_VLAN_VID	GENMASK(11, 0)
#define FMT_OUTER_VLAN_PCP	GENMASK(14, 12)
#define FMT_OUTER_VLAN_DEI	BIT(15)
	__le16 act1;
#define FMT_OUTER_TPID_ACT	GENMASK(2, 0)
#define FMT_OUTER_PCP_ACT	GENMASK(5, 3)
#define FMT_OUTER_DEI_ACT	GENMASK(7, 6)
#define FMT_PLD_ACT		GENMASK(10, 8)
#define FMT_OPCUA_MSG_CNT	GENMASK(15, 11)
	__le16 pld_offset;
	__le16 opcua_fms;
	__le16 fmd_bytes;
	__le16 opcua_param;
#define FMT_OPCUA_PARAM		GENMASK(5, 0)
	__le32 fmd_eid;
};

union ntmp_fmt_eid {
	__le32 index;
#define	FMTEID_INDEX		GENMASK(12, 0)
	__le32 vuda_sqta;
#define FMTEID_VUDA		GENMASK(1, 0)
#define FMTEID_VUDA_DEL_OTAG	2
#define FMTEID_SQTA		GENMASK(4, 2)
#define FMTEID_VUDA_SQTA	BIT(13)
	__le32 vara_vid;
#define FMTEID_VID		GENMASK(11, 0)
#define FMTEID_VARA		GENMASK(13, 12)
#define FRMEOD_VARA_VID		BIT(14)
};

struct bpt_cfge_data {
	u8 fccfg_sbpen;
#define BPT_SBP_EN		BIT(0)
#define BPT_FC_CFG		GENMASK(2, 1)
#define BPT_FC_CFG_EN_BPFC	1
	u8 pfc_vector;
	__le16 max_thresh;
	__le16 fc_on_thresh;
	__le16 fc_off_thresh;
	__le16 sbp_thresh;
	__le16 resv;
	__le32 sbp_eid;
	__le32 fc_ports;
};

struct bpt_bpse_data {
	__le32 amount_used;
	__le32 amount_used_hwm;
	u8 bpd_fc_state;
#define BPT_FC_STATE		BIT(0)
#define BPT_BPD			BIT(1)
};

struct sbpt_cfge_data {
	__le16 resv;
	__le16 max_thresh;
	__le16 fc_on_thresh;
	__le16 fc_off_thresh;
};

struct sbpt_sbpse_data {
	__le32 amount_used;
	__le32 amount_used_hwm;
	u8 fc_state;
#define SBPT_FC_STATE		BIT(0)
};

#pragma pack()

struct netc_cbdr_regs {
	void __iomem *pir;
	void __iomem *cir;
	void __iomem *mr;

	void __iomem *bar0;
	void __iomem *bar1;
	void __iomem *lenr;
};

enum ntmp_table_version {
	NTMP_TBL_VER0 = 0, /* MUST be 0 */
	NTMP_TBL_VER1,
};

struct netc_tbl_vers {
	u8 maft_ver;
	u8 vaft_ver;
	u8 rsst_ver;
	u8 rfst_ver;
	u8 tgst_ver;
	u8 rpt_ver;
	u8 ipft_ver;
	u8 fdbt_ver;
	u8 vft_ver;
	u8 isit_ver;
	u8 ist_ver;
	u8 isft_ver;
	u8 sgit_ver;
	u8 sgclt_ver;
	u8 isct_ver;
	u8 ett_ver;
	u8 esrt_ver;
	u8 ect_ver;
	u8 fmt_ver;
	u8 bpt_ver;
	u8 sbpt_ver;
	u8 fmdt_ver;
};

struct netc_cbdr {
	struct netc_cbdr_regs regs;

	int bd_num;
	int next_to_use;
	int next_to_clean;

	int dma_size;
	void *addr_base;
	void *addr_base_align;
	dma_addr_t dma_base;
	dma_addr_t dma_base_align;

	spinlock_t ring_lock; /* Avoid race condition */
};

struct netc_cbdrs {
	int cbdr_num;	/* number of control BD ring */
	int cbdr_size;	/* number of BDs per control BD ring */
	struct device *dma_dev;
	struct netc_cbdr *ring;
	struct netc_tbl_vers tbl;
};

enum netc_dev_type {
	NETC_DEV_ENETC,
	NETC_DEV_SWITCH
};

struct ntmp_caps {
	int rpt_num_entries;
	int isct_num_entries;
	int ist_num_entries;
	int sgit_num_entries;
	int sgclt_num_words;
	int ett_num_entries;
	int ect_num_entries;
};

struct ntmp_priv {
	enum netc_dev_type dev_type;
	struct netc_cbdrs cbdrs;
	u32 errata;

	struct ntmp_caps caps;
	/* bitmap of table entry ID */
	unsigned long *ist_eid_bitmap;
	unsigned long *rpt_eid_bitmap;
	unsigned long *sgit_eid_bitmap;
	unsigned long *isct_eid_bitmap;
	unsigned long *sgclt_word_bitmap;
	unsigned long *ett_eid_bitmap;
	unsigned long *ect_eid_bitmap;
	u32 ett_bitmap_size;
	u32 ect_bitmap_size;

	struct hlist_head flower_list;
	struct mutex flower_lock; /* flower_list lock */

	u64 (*adjust_base_time)(struct ntmp_priv *priv, u64 bt, u32 ct);
	u32 (*get_tgst_free_words)(struct ntmp_priv *priv);
};

struct maft_entry_data {
	struct maft_keye_data keye;
	struct maft_cfge_data cfge;
};

struct vaft_entry_data {
	struct vaft_keye_data keye;
	struct vaft_cfge_data cfge;
};

struct rfst_entry_data {
	struct rfst_keye_data keye;
	struct rfst_cfge_data cfge;
	/* STSE_DATA, Only valid for query action */
	__le64 matched_frames;
};

struct ntmp_isit_entry {
	u32 entry_id;  /* hardware assigns entry ID */
	struct isit_keye_data keye;
	__le32 is_eid; /* cfge data */
};

struct ntmp_ist_entry {
	u32 entry_id; /* software assigns entry ID */
	struct ist_cfge_data cfge;
};

struct ntmp_isft_entry {
	u32 entry_id; /* hardware assigns entry ID */
	struct isft_keye_data keye;
	struct isft_cfge_data cfge;
};

struct ntmp_sgit_entry {
	u32 entry_id; /* software assigns entry ID */
	struct sgit_acfge_data acfge;
	struct sgit_cfge_data cfge;
	struct sgit_icfge_data icfge;
	struct sgit_sgise_data sgise;
};

struct ntmp_sgclt_entry {
	u32 entry_id;
	u8 ref_count; /* SGCLSE_DATA */
	struct sgclt_cfge_data cfge; /* Must be last member */
};

struct ntmp_rpt_entry {
	u32 entry_id;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
	struct rpt_stse_data stse;
	struct rpt_pse_data pse;
};

struct ntmp_isct_entry {
	u32 entry_id;
	struct isct_stse_data stse;
};

struct ntmp_ipft_entry {
	u32 entry_id;
	struct ipft_keye_data keye;
	struct ipft_cfge_data cfge;
	__le64 match_count; /* STSE_DATA */
};

struct fdbt_query_data {
	struct fdbt_keye_data keye;
	struct fdbt_cfge_data cfge;
	struct fdbt_acte_data acte;
};

struct esrt_query_data {
	struct esrt_stse_data stse;
	struct esrt_cfge_data cfge;
	struct esrt_srse_data srse;
};

struct bpt_query_data {
	struct bpt_bpse_data bpse;
	struct bpt_cfge_data cfge;
};

struct sbpt_query_data {
	struct sbpt_sbpse_data sbpse;
	struct sbpt_cfge_data cfge;
};

#if IS_ENABLED(CONFIG_NXP_NETC_LIB)
int netc_setup_cbdr(struct device *dev, int cbd_num, struct netc_cbdr_regs *regs,
		    struct netc_cbdr *cbdr);
void netc_teardown_cbdr(struct device *dev, struct netc_cbdr *cbdr);

/* NTMP APIs */
u32 ntmp_lookup_free_eid(unsigned long *bitmap, u32 bitmap_size);
void ntmp_clear_eid_bitmap(unsigned long *bitmap, u32 entry_id);
int ntmp_maft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct maft_entry_data *data);
int ntmp_maft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct maft_entry_data *data);
int ntmp_maft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_vaft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct vaft_entry_data *data);
int ntmp_vaft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct vaft_entry_data *data);
int ntmp_vaft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_rsst_query_or_update_entry(struct netc_cbdrs *cbdrs, u32 *table,
				    int count, bool query);
int ntmp_rfst_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct rfst_entry_data *data);
int ntmp_rfst_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct rfst_entry_data *data);
int ntmp_rfst_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_rpt_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_rpt_entry *entry);
int ntmp_rpt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_isit_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
				  struct ntmp_isit_entry *entry);
int ntmp_ist_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_ist_entry *entry);
int ntmp_ist_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_isct_operate_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			    int cmd, struct isct_stse_data *data);
int ntmp_ipft_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
			struct ntmp_ipft_entry *entry);
int ntmp_ipft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  bool update, struct ntmp_ipft_entry *entry);
int ntmp_ipft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_fdbt_update_activity_element(struct netc_cbdrs *cbdrs);
int ntmp_fdbt_delete_aging_entries(struct netc_cbdrs *cbdrs, u8 act_cnt);
int ntmp_fdbt_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
			struct fdbt_keye_data *keye,
			struct fdbt_cfge_data *cfge);
int ntmp_fdbt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			   struct fdbt_cfge_data *cfge);
int ntmp_fdbt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_fdbt_delete_port_dynamic_entries(struct netc_cbdrs *cbdrs, int port);
int ntmp_fdbt_search_port_entry(struct netc_cbdrs *cbdrs, int port,
				u32 *resume_entry_id, u32 *entry_id,
				struct fdbt_query_data *data);
int ntmp_vft_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
		       u16 vid, struct vft_cfge_data *cfge);
int ntmp_vft_update_entry(struct netc_cbdrs *cbdrs, u16 vid,
			  struct vft_cfge_data *cfge);
int ntmp_vft_delete_entry(struct netc_cbdrs *cbdrs, u16 vid);
int ntmp_vft_search_entry(struct netc_cbdrs *cbdrs, u32 *resume_eid,
			  u32 *entry_id, u16 *vid, struct vft_cfge_data *cfge);
int ntmp_vft_query_entry_by_vid(struct netc_cbdrs *cbdrs, u16 vid,
				u32 *entry_id, struct vft_cfge_data *cfge);
int ntmp_ett_add_or_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				 bool add, struct ett_cfge_data *cfge);
int ntmp_ett_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_ett_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct ett_cfge_data *cfge);
int ntmp_esrt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			   struct esrt_cfge_data *cfge);
int ntmp_esrt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct esrt_query_data *data);
int ntmp_ect_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_ect_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct ect_stse_data *stse, bool update);
int ntmp_fmt_add_or_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				 bool add, struct fmt_cfge_data *cfge);
int ntmp_fmt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_fmt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct fmt_cfge_data *cfge);
int ntmp_bpt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct bpt_cfge_data *cfge);
int ntmp_bpt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct bpt_query_data *data);
int ntmp_sbpt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			   struct sbpt_cfge_data *cfge);
int ntmp_sbpt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct sbpt_query_data *data);
int ntmp_fmdt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			   u8 *data, u32 data_len);
int ntmp_fmdt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  u8 *data_buff, u32 data_len);
#else
static inline int netc_setup_cbdr(struct device *dev, int cbd_num,
				  struct netc_cbdr_regs *regs,
				  struct netc_cbdr *cbdr)
{
	return 0;
}

static inline void netc_teardown_cbdr(struct device *dev, struct netc_cbdr *cbdr)
{
}

/* NTMP APIs */
static inline u32 ntmp_lookup_free_eid(unsigned long *bitmap, u32 size)
{
	return 0;
}

static inline void ntmp_clear_eid_bitmap(unsigned long *bitmap, u32 entry_id)
{
}

static inline int ntmp_maft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				      struct maft_entry_data *data)
{
	return 0;
}

static inline int ntmp_maft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct maft_entry_data *data)
{
	return 0;
}

static inline int ntmp_maft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_vaft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				      struct vaft_entry_data *data)
{
	return 0;
}

static inline int ntmp_vaft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct vaft_entry_data *data)
{
	return 0;
}

static inline int ntmp_vaft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_rsst_query_or_update_entry(struct netc_cbdrs *cbdrs,
						  u32 *table, int count,
						  bool query)
{
	return 0;
}

static inline int ntmp_rfst_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				      struct rfst_entry_data *data)
{
	return 0;
}

static inline int ntmp_rfst_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct rfst_entry_data *data)
{
	return 0;
}

static inline int ntmp_rfst_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_rpt_add_or_update_entry(struct netc_cbdrs *cbdrs,
					       struct ntmp_rpt_entry *entry)
{
	return 0;
}

static inline int ntmp_rpt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_isit_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
						struct ntmp_isit_entry *entry)
{
	return 0;
}

static inline int ntmp_ist_add_or_update_entry(struct netc_cbdrs *cbdrs,
					       struct ntmp_ist_entry *entry)
{
	return 0;
}

static inline int ntmp_ist_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_isft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_isct_operate_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					  int cmd, struct isct_stse_data *stse)
{
	return 0;
}

static inline int ntmp_ipft_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
				      struct ntmp_ipft_entry *entry)
{
	return 0;
}

static inline int ntmp_ipft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					bool update, struct ntmp_ipft_entry *entry)
{
	return 0;
}

static inline int ntmp_ipft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_fdbt_update_activity_element(struct netc_cbdrs *cbdrs)
{
	return 0;
}

static inline int ntmp_fdbt_delete_aging_entries(struct netc_cbdrs *cbdrs,
						 u8 act_cnt)
{
	return 0;
}

static inline int ntmp_fdbt_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
				      struct fdbt_keye_data *keye,
				      struct fdbt_cfge_data *data)
{
	return 0;
}

static inline int ntmp_fdbt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					 struct fdbt_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_fdbt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_fdbt_delete_port_dynamic_entries(struct netc_cbdrs *cbdrs,
							int port)
{
	return 0;
}

static inline int ntmp_fdbt_search_port_entry(struct netc_cbdrs *cbdrs, int port,
					      u32 *resume_entry_id, u32 *entry_id,
					      struct fdbt_query_data *data)
{
	return 0;
}

static inline int ntmp_vft_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
				     u16 vid, struct vft_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_vft_update_entry(struct netc_cbdrs *cbdrs, u16 vid,
					struct vft_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_vft_delete_entry(struct netc_cbdrs *cbdrs, u16 vid)
{
	return 0;
}

static inline int ntmp_vft_search_entry(struct netc_cbdrs *cbdrs, u32 *resume_eid,
					u32 *entry_id, u16 *vid,
					struct vft_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_vft_query_entry_by_vid(struct netc_cbdrs *cbdrs, u16 vid,
					      u32 *entry_id,
					      struct vft_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_ett_add_or_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					       bool add, struct ett_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_ett_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_ett_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				       struct ett_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_esrt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					 struct esrt_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_esrt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct esrt_query_data *data)
{
	return 0;
}

static inline int ntmp_ect_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_ect_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				       struct ect_stse_data *stse, bool update)
{
	return 0;
}

static inline int ntmp_fmt_add_or_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					       bool add, struct fmt_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_fmt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_fmt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				       struct fmt_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_bpt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct bpt_bpse_data *cfge)
{
	return 0;
}

static inline int ntmp_bpt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				      struct bpt_query_data *data)
{
	return 0;
}

static inline int ntmp_sbpt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					 struct sbpt_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_sbpt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct sbpt_query_data *data)
{
	return 0;
}

static inline int ntmp_fmdt_update_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					 u8 *data, u32 data_len)
{
	return 0;
}

static inline int ntmp_fmdt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  u8 *data_buff, u32 data_len)
{
	return 0;
}
#endif

#endif /* ENETC_NTMP_H */
