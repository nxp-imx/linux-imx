/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Synopsys, Inc. and/or its affiliates.
 * Synopsys DesignWare XPCS helpers
 */

#ifndef __LINUX_PCS_XPCS_H
#define __LINUX_PCS_XPCS_H

#include <linux/clk.h>
#include <linux/fwnode.h>
#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/types.h>

/* AN mode */
#define DW_AN_C73			1
#define DW_AN_C37_SGMII			2
#define DW_2500BASEX			3
#define DW_AN_C37_1000BASEX		4
#define DW_10GBASER			5

enum dw_xpcs_pcs_version {
	DW_XPCS_VER_DEFAULT = 0,
	DW_XPCS_VER_MX95,
	DW_XPCS_VER_MX94,
};

enum dw_xpcs_pcs_id {
	DW_XPCS_ID_NATIVE = 0,
	NXP_SJA1105_XPCS_ID = 0x00000010,
	NXP_SJA1110_XPCS_ID = 0x00000020,
	NXP_MX95_XPCS_ID = 0x1b3274cd + DW_XPCS_VER_MX95,
	NXP_MX94_XPCS_ID = 0x1b3274cd + DW_XPCS_VER_MX94,
	NXP_MX952_XPCS_ID = 0x182d74cd,
	DW_XPCS_ID = 0x7996ced0,
	DW_XPCS_ID_MASK = 0xffffffff,
};

enum dw_xpcs_pma_id {
	DW_XPCS_PMA_ID_NATIVE = 0,
	DW_XPCS_PMA_GEN1_3G_ID,
	DW_XPCS_PMA_GEN2_3G_ID,
	DW_XPCS_PMA_GEN2_6G_ID,
	DW_XPCS_PMA_GEN4_3G_ID,
	DW_XPCS_PMA_GEN4_6G_ID,
	DW_XPCS_PMA_GEN5_10G_ID,
	DW_XPCS_PMA_GEN5_12G_ID,
	WX_TXGBE_XPCS_PMA_10G_ID = 0x0018fc80,
};

struct dw_xpcs_info {
	u32 pcs;
	u32 pma;
	u8 version;
};

struct dw_xpcs;

struct phylink_pcs *xpcs_to_phylink_pcs(struct dw_xpcs *xpcs);
int xpcs_get_an_mode(struct dw_xpcs *xpcs, phy_interface_t interface);
void xpcs_config_eee_mult_fact(struct dw_xpcs *xpcs, u8 mult_fact);
struct dw_xpcs *xpcs_create_mdiodev(struct mii_bus *bus, int addr);
struct dw_xpcs *xpcs_create_fwnode(struct fwnode_handle *fwnode);
void xpcs_destroy(struct dw_xpcs *xpcs);

struct phylink_pcs *xpcs_create_pcs_mdiodev(struct mii_bus *bus, int addr);
void xpcs_destroy_pcs(struct phylink_pcs *pcs);

#if IS_ENABLED(CONFIG_PCS_XPCS)
struct phylink_pcs *xpcs_create_mdiodev_with_phy(struct mii_bus *bus,
						 int mdioaddr, int phyaddr,
						 int portid, int version,
						 phy_interface_t interface);
void xpcs_pcs_destroy(struct phylink_pcs *pcs);
#else
static inline struct phylink_pcs *xpcs_create_mdiodev_with_phy(struct mii_bus *bus,
						 int mdioaddr, int phyaddr,
						 int portid, int version,
						 phy_interface_t interface)
{
	return ERR_PTR(-EOPNOTSUPP);
}

static inline void xpcs_pcs_destroy(struct phylink_pcs *pcs)
{
}
#endif /* IS_ENABLED(CONFIG_PCS_XPCS) */

#endif /* __LINUX_PCS_XPCS_H */
