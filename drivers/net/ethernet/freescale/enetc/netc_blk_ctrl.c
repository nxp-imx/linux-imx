// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC Blocks Control Driver
 *
 * Copyright 2024 NXP
 */
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fsl/netc_global.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/seq_file.h>

/* NETCMIX registers */
#define IMX95_CFG_LINK_IO_VAR		0x0
#define  IO_VAR_16FF_16G_SERDES		0x1
#define  IO_VAR(port, var)		(((var) & 0xf) << ((port) << 2))

#define IMX95_CFG_LINK_MII_PROT		0x4
#define CFG_LINK_MII_PORT_0		GENMASK(3, 0)
#define CFG_LINK_MII_PORT_1		GENMASK(7, 4)
#define  MII_PROT_MII			0x0
#define  MII_PROT_RMII			0x1
#define  MII_PROT_RGMII			0x2
#define  MII_PROT_SERIAL		0x3
#define  MII_PROT(port, prot)		(((prot) & 0xf) << ((port) << 2))

#define IMX95_CFG_LINK_PCS_PROT(a)	(0x8 + (a) * 4)
#define PCS_PROT_1G_SGMII		BIT(0)
#define PCS_PROT_2500M_SGMII		BIT(1)
#define PCS_PROT_XFI			BIT(3)
#define PCS_PROT_SFI			BIT(4)
#define PCS_PROT_10G_SXGMII		BIT(6)

#define IMX94_MISC_SOC_CONTROL		0x0
#define  SEL_XPCS_1			BIT(1)
#define   IMX94_XPCS_PORT_0		0x0
#define   IMX94_XPCS_PORT_1		0x1

#define IMX94_EXT_PIN_CONTROL		0x10
#define  MAC2_MAC3_SEL			BIT(1)

#define IMX94_CFG_LINK_PCS_PROT(a)	(0x14 + (a) * 4)
#define IMX94_NETC_LINK_CFG(a)		(0x4c + (a) * 4)
#define  NETC_LINK_CFG_MII_PROT		GENMASK(3, 0)
#define  NETC_LINK_CFG_IO_VAR		GENMASK(19, 16)

/* NETC privileged register block register */
#define PRB_NETCRR			0x100
#define  NETCRR_SR			BIT(0)
#define  NETCRR_LOCK			BIT(1)

#define PRB_NETCSR			0x104
#define  NETCSR_ERROR			BIT(0)
#define  NETCSR_STATE			BIT(1)

/* NETC integrated endpoint register block register */
#define IERB_EMDIOFAUXR			0x344
#define IERB_T0FAUXR			0x444
#define IERB_ETBCR(a)			(0x300c + 0x100 * (a))
#define IERB_EFAUXR(a)			(0x3044 + 0x100 * (a))
#define IERB_VFAUXR(a)			(0x4004 + 0x40 * (a))
#define FAUXR_LDID			GENMASK(3, 0)

/* Platform information */
#define IMX95_ENETC0_BUS_DEVFN		0x0
#define IMX95_ENETC1_BUS_DEVFN		0x40
#define IMX95_ENETC2_BUS_DEVFN		0x80
#define IMX95_LINK_NUM			3

#define IMX94_ENETC3_BUS_DEVFN		0x0
#define IMX94_TIMER0_BUS_DEVFN		0x1
#define IMX94_SWITCH_BUS_DEVFN		0x2
#define IMX94_ENETC0_BUS_DEVFN		0x100
#define IMX94_TIMER1_BUS_DEVFN		0x101
#define IMX94_ENETC1_BUS_DEVFN		0x140
#define IMX94_ENETC2_BUS_DEVFN		0x180
#define IMX94_TIMER2_BUS_DEVFN		0x181
#define IMX94_ENETC0_LINK		3
#define IMX94_ENETC1_LINK		4
#define IMX94_ENETC2_LINK		5
#define IMX94_ENETC0_OFFSET		0
#define IMX94_ENETC1_OFFSET		1
#define IMX94_ENETC2_OFFSET		2
#define IMX94_SWITCH_PORT2		2
#define IMX94_SWITCH_CPU_PORT		3
#define IMX94_TIMER0_ID			0
#define IMX94_TIMER1_ID			1
#define IMX94_TIMER2_ID			2

/* Flags for different platforms */
#define NETC_HAS_NETCMIX		BIT(0)

struct netc_blk_ctrl {
	void __iomem *prb;
	void __iomem *ierb;
	void __iomem *netcmix;
	struct clk *ipg_clk;

	const struct netc_devinfo *devinfo;
	atomic_t wakeonlan_count;
	struct platform_device *pdev;
	struct dentry *debugfs_root;
};

struct netc_devinfo {
	u32 flags;
	int num_link; /* Internal links are not included */
	int (*netcmix_init)(struct platform_device *pdev);
	int (*ierb_init)(struct platform_device *pdev);
	void (*xpcs_port_init)(struct netc_blk_ctrl *priv, int port);
};

static struct netc_blk_ctrl *netc_bc;

static void netc_reg_write(void __iomem *base, u32 offset, u32 val)
{
	iowrite32(val, base + offset);
}

static u32 netc_reg_read(void __iomem *base, u32 offset)
{
	return ioread32(base + offset);
}

static int netc_of_pci_get_bus_devfn(struct device_node *np)
{
	u32 reg[5];
	int error;

	error = of_property_read_u32_array(np, "reg", reg, ARRAY_SIZE(reg));
	if (error)
		return error;

	return (reg[0] >> 8) & 0xffff;
}

static int netc_get_link_mii_protocol(phy_interface_t interface)
{
	switch (interface) {
	case PHY_INTERFACE_MODE_MII:
		return MII_PROT_MII;
	case PHY_INTERFACE_MODE_RMII:
		return MII_PROT_RMII;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		return MII_PROT_RGMII;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_XGMII:
	case PHY_INTERFACE_MODE_USXGMII:
		return MII_PROT_SERIAL;
	default:
		return -EINVAL;
	}
}

static int imx95_netcmix_init(struct platform_device *pdev)
{
	struct netc_blk_ctrl *priv = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child, *gchild;
	phy_interface_t interface;
	int bus_devfn, mii_proto;
	u32 val;
	int err;

	/* Default setting */
	val = MII_PROT(0, MII_PROT_RGMII) | MII_PROT(1, MII_PROT_RGMII) |
	      MII_PROT(2, MII_PROT_SERIAL);

	/* Update the link MII protocol through parsing phy-mode */
	for_each_available_child_of_node(np, child) {
		for_each_available_child_of_node(child, gchild) {
			if (!of_device_is_compatible(gchild, "fsl,imx95-enetc"))
				continue;

			bus_devfn = netc_of_pci_get_bus_devfn(gchild);
			if (bus_devfn < 0) {
				err = -EINVAL;
				goto err_out;
			}

			if (bus_devfn == IMX95_ENETC2_BUS_DEVFN)
				continue;

			err = of_get_phy_mode(gchild, &interface);
			if (err)
				continue;

			mii_proto = netc_get_link_mii_protocol(interface);
			if (mii_proto < 0) {
				err = -EINVAL;
				goto err_out;
			}

			switch (bus_devfn) {
			case IMX95_ENETC0_BUS_DEVFN:
				val = u32_replace_bits(val, mii_proto,
						       CFG_LINK_MII_PORT_0);
				break;
			case IMX95_ENETC1_BUS_DEVFN:
				val = u32_replace_bits(val, mii_proto,
						       CFG_LINK_MII_PORT_1);
				break;
			default:
				err = -EINVAL;
				goto err_out;
			}
		}
	}

	/* Configure Link I/O variant */
	netc_reg_write(priv->netcmix, IMX95_CFG_LINK_IO_VAR,
		       IO_VAR(2, IO_VAR_16FF_16G_SERDES));
	/* Configure Link 2 PCS protocol */
	netc_reg_write(priv->netcmix, IMX95_CFG_LINK_PCS_PROT(2),
		       PCS_PROT_10G_SXGMII);
	netc_reg_write(priv->netcmix, IMX95_CFG_LINK_MII_PROT, val);

	return 0;

err_out:
	of_node_put(gchild);
	of_node_put(child);

	return err;
}

static int imx94_enetc_get_link_num(struct device_node *np)
{
	int bus_devfn;

	bus_devfn = netc_of_pci_get_bus_devfn(np);
	if (bus_devfn < 0)
		return -EINVAL;

	/* Parse ENETC link number */
	switch (bus_devfn) {
	case IMX94_ENETC0_BUS_DEVFN:
		return IMX94_ENETC0_LINK;
	case IMX94_ENETC1_BUS_DEVFN:
		return IMX94_ENETC1_LINK;
	case IMX94_ENETC2_BUS_DEVFN:
		return IMX94_ENETC2_LINK;
	default:
		return -EINVAL;
	}
}

static int imx94_link_config(struct netc_blk_ctrl *priv,
			     struct device_node *np, int link_id)
{
	phy_interface_t interface;
	int mii_proto, err;
	u32 val;

	err = of_get_phy_mode(np, &interface);
	if (err)
		return err;

	mii_proto = netc_get_link_mii_protocol(interface);
	if (mii_proto < 0)
		return -EINVAL;

	val = mii_proto & NETC_LINK_CFG_MII_PROT;
	if (mii_proto == MII_PROT_SERIAL) {
		int pcs_proto = PCS_PROT_1G_SGMII;

		if (pcs_proto == PHY_INTERFACE_MODE_2500BASEX)
			pcs_proto = PCS_PROT_2500M_SGMII;

		netc_reg_write(priv->netcmix, IMX94_CFG_LINK_PCS_PROT(link_id),
			       pcs_proto);
		val = u32_replace_bits(val, IO_VAR_16FF_16G_SERDES,
				       NETC_LINK_CFG_IO_VAR);
	}

	netc_reg_write(priv->netcmix, IMX94_NETC_LINK_CFG(link_id), val);

	if (link_id == IMX94_ENETC0_LINK) {
		val = netc_reg_read(priv->netcmix, IMX94_EXT_PIN_CONTROL);
		val |= MAC2_MAC3_SEL;
		netc_reg_write(priv->netcmix, IMX94_EXT_PIN_CONTROL, val);
	}

	return 0;
}

static int imx94_enetc_link_config(struct netc_blk_ctrl *priv,
				   struct device_node *np, bool *enetc0_en)
{
	int link_id;

	link_id = imx94_enetc_get_link_num(np);
	if (link_id < 0)
		return -EINVAL;

	if (link_id == IMX94_ENETC0_LINK)
		*enetc0_en = true;

	return imx94_link_config(priv, np, link_id);
}

static int imx94_switch_link_config(struct netc_blk_ctrl *priv,
				    struct device_node *np, bool *swp2_en)
{
	struct device_node *ports;
	int port_id, err = 0;

	ports = of_get_child_by_name(np, "ports");
	if (!ports)
		ports = of_get_child_by_name(np, "ethernet-ports");
	if (!ports)
		return -ENODEV;

	for_each_available_child_of_node_scoped(ports, child) {
		if (of_property_read_u32(child, "reg", &port_id) < 0) {
			err = -ENODEV;
			goto end;
		}

		if (port_id == IMX94_SWITCH_CPU_PORT)
			continue;

		if (port_id == IMX94_SWITCH_PORT2)
			*swp2_en = true;

		err = imx94_link_config(priv, child, port_id);
		if (err)
			goto end;
	}

end:
	of_node_put(ports);

	return err;
}

static int imx94_netcmix_init(struct platform_device *pdev)
{
	struct netc_blk_ctrl *priv = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	bool enetc0_en = false, swp2_en = false;
	int err;

	for_each_available_child_of_node_scoped(np, child) {
		for_each_available_child_of_node_scoped(child, gchild) {
			if (of_device_is_compatible(gchild, "pci1131,e101")) {
				err = imx94_enetc_link_config(priv, gchild, &enetc0_en);
				if (err)
					return err;
			} else if (of_device_is_compatible(gchild, "pci1131,eef2")) {
				err = imx94_switch_link_config(priv, gchild, &swp2_en);
				if (err)
					return err;
			}
		}
	}

	if (enetc0_en && swp2_en) {
		dev_err(&pdev->dev, "Cannot enable swp2 and enetc0 at the same time\n");

		return -EINVAL;
	}

	return 0;
}

static bool netc_ierb_is_locked(struct netc_blk_ctrl *priv)
{
	return !!(netc_reg_read(priv->prb, PRB_NETCRR) & NETCRR_LOCK);
}

static int netc_lock_ierb(struct netc_blk_ctrl *priv)
{
	u32 val;

	netc_reg_write(priv->prb, PRB_NETCRR, NETCRR_LOCK);

	return read_poll_timeout(netc_reg_read, val, !(val & NETCSR_STATE),
				 100, 2000, false, priv->prb, PRB_NETCSR);
}

static int netc_unlock_ierb_with_warm_reset(struct netc_blk_ctrl *priv)
{
	u32 val;

	netc_reg_write(priv->prb, PRB_NETCRR, 0);

	return read_poll_timeout(netc_reg_read, val, !(val & NETCRR_LOCK),
				 1000, 100000, true, priv->prb, PRB_NETCRR);
}

static int imx95_ierb_init(struct platform_device *pdev)
{
	struct netc_blk_ctrl *priv = platform_get_drvdata(pdev);

	/* EMDIO : No MSI-X intterupt */
	netc_reg_write(priv->ierb, IERB_EMDIOFAUXR, 0);
	/* ENETC0 PF */
	netc_reg_write(priv->ierb, IERB_EFAUXR(0), 0);
	/* ENETC0 VF0 */
	netc_reg_write(priv->ierb, IERB_VFAUXR(0), 1);
	/* ENETC0 VF1 */
	netc_reg_write(priv->ierb, IERB_VFAUXR(1), 2);
	/* ENETC1 PF */
	netc_reg_write(priv->ierb, IERB_EFAUXR(1), 3);
	/* ENETC1 VF0 : Disabled on 19x19 board dts */
	netc_reg_write(priv->ierb, IERB_VFAUXR(2), 5);
	/* ENETC1 VF1 : Disabled on 19x19 board dts */
	netc_reg_write(priv->ierb, IERB_VFAUXR(3), 6);
	/* ENETC2 PF */
	netc_reg_write(priv->ierb, IERB_EFAUXR(2), 4);
	/* ENETC2 VF0 : Disabled on 15x15 board dts */
	netc_reg_write(priv->ierb, IERB_VFAUXR(4), 5);
	/* ENETC2 VF1 : Disabled on 15x15 board dts */
	netc_reg_write(priv->ierb, IERB_VFAUXR(5), 6);
	/* NETC TIMER */
	netc_reg_write(priv->ierb, IERB_T0FAUXR, 7);

	return 0;
}

static int imx94_enetc_get_enetc_offset(struct device_node *np)
{
	int bus_devfn;

	bus_devfn = netc_of_pci_get_bus_devfn(np);
	if (bus_devfn < 0)
		return -EINVAL;

	/* Parse ENETC offset */
	switch (bus_devfn) {
	case IMX94_ENETC0_BUS_DEVFN:
		return IMX94_ENETC0_OFFSET;
	case IMX94_ENETC1_BUS_DEVFN:
		return IMX94_ENETC1_OFFSET;
	case IMX94_ENETC2_BUS_DEVFN:
		return IMX94_ENETC2_OFFSET;
	default:
		return -EINVAL;
	}
}

static int imx94_enetc_get_timer_id(struct device_node *np)
{
	int bus_devfn;

	bus_devfn = netc_of_pci_get_bus_devfn(np);
	if (bus_devfn < 0)
		return -EINVAL;

	/* Parse ENETC PTP timer ID */
	switch (bus_devfn) {
	case IMX94_TIMER0_BUS_DEVFN:
		return IMX94_TIMER0_ID;
	case IMX94_TIMER1_BUS_DEVFN:
		return IMX94_TIMER1_ID;
	case IMX94_TIMER2_BUS_DEVFN:
		return IMX94_TIMER2_ID;
	default:
		return -EINVAL;
	}
}

static int imx94_enetc_update_tid(struct netc_blk_ctrl *priv, struct device_node *pf_np)
{
	struct device_node *timer_np;
	int offset, tid;

	offset = imx94_enetc_get_enetc_offset(pf_np);
	if (offset < 0) {
		dev_err(&priv->pdev->dev, "Find unknown PF node.\n");
		return offset;
	}

	timer_np = of_parse_phandle(pf_np, "nxp,ptp-timer", 0);
	if (!timer_np) {
		/*
		 * If nxp,ptp-timer is not set, the first timer of the bus
		 * where enetc is located will be used as the default timer.
		 */
		tid = IMX94_TIMER1_ID;
		goto update_reg;
	}

	tid = imx94_enetc_get_timer_id(timer_np);
	of_node_put(timer_np);
	if (tid < 0) {
		dev_err(&priv->pdev->dev, "Incorrect bus/devfn of ptp-timer.\n");
		return tid;
	}

update_reg:
	netc_reg_write(priv->ierb, IERB_ETBCR(offset), tid);

	return 0;
}

static int imx94_ierb_init(struct platform_device *pdev)
{
	struct netc_blk_ctrl *priv = platform_get_drvdata(pdev);
	struct device_node *blk_np = pdev->dev.of_node;
	int ret = 0;

	if (!blk_np)
		return -ENODEV;

	for_each_available_child_of_node_scoped(blk_np, bus_np)
		for_each_available_child_of_node_scoped(bus_np, pf_np)
			if (of_device_is_compatible(pf_np, "pci1131,e101"))
				ret = imx94_enetc_update_tid(priv, pf_np);

	return ret;
}

static int netc_ierb_init(struct platform_device *pdev)
{
	struct netc_blk_ctrl *priv = platform_get_drvdata(pdev);
	const struct netc_devinfo *devinfo = priv->devinfo;
	int err;

	if (netc_ierb_is_locked(priv)) {
		err = netc_unlock_ierb_with_warm_reset(priv);
		if (err) {
			dev_err(&pdev->dev, "Unlock IERB failed.\n");
			return err;
		}
	}

	if (devinfo->ierb_init) {
		err = devinfo->ierb_init(pdev);
		if (err)
			return err;
	}

	err = netc_lock_ierb(priv);
	if (err) {
		dev_err(&pdev->dev, "Lock IERB failed.\n");
		return err;
	}

	atomic_set(&priv->wakeonlan_count, 0);

	return 0;
}

static void imx94_netc_xpcs_port_init(struct netc_blk_ctrl *priv, int port)
{
	u32 val;

	val = netc_reg_read(priv->netcmix, IMX94_MISC_SOC_CONTROL);
	if (port == IMX94_XPCS_PORT_1)
		val |= SEL_XPCS_1;
	else
		val &= ~SEL_XPCS_1;
	netc_reg_write(priv->netcmix, IMX94_MISC_SOC_CONTROL, val);
}

void netc_xpcs_port_init(int port)
{
	struct netc_blk_ctrl *priv = netc_bc;
	const struct netc_devinfo *devinfo;

	if (!priv)
		return;
	devinfo = priv->devinfo;

	if (devinfo->xpcs_port_init)
		devinfo->xpcs_port_init(priv, port);
}
EXPORT_SYMBOL_GPL(netc_xpcs_port_init);

void netc_ierb_enable_wakeonlan(void)
{
	struct netc_blk_ctrl *priv = netc_bc;

	if (!priv)
		return;

	atomic_inc(&priv->wakeonlan_count);
}
EXPORT_SYMBOL_GPL(netc_ierb_enable_wakeonlan);

void netc_ierb_disable_wakeonlan(void)
{
	struct netc_blk_ctrl *priv = netc_bc;

	if (!priv)
		return;

	atomic_dec(&priv->wakeonlan_count);
	if (atomic_read(&priv->wakeonlan_count) < 0) {
		atomic_set(&priv->wakeonlan_count, 0);
		dev_warn(&priv->pdev->dev, "Wake-on-LAN count underflow.\n");
	}
}
EXPORT_SYMBOL_GPL(netc_ierb_disable_wakeonlan);

int netc_ierb_may_wakeonlan(void)
{
	struct netc_blk_ctrl *priv = netc_bc;

	if (!priv)
		return -ENXIO;

	return atomic_read(&priv->wakeonlan_count);
}
EXPORT_SYMBOL_GPL(netc_ierb_may_wakeonlan);

#if IS_ENABLED(CONFIG_DEBUG_FS)
static int netc_prb_show(struct seq_file *s, void *data)
{
	struct netc_blk_ctrl *priv = s->private;
	u32 val;

	val = netc_reg_read(priv->prb, PRB_NETCRR);
	seq_printf(s, "[PRB NETCRR] Lock:%d SR:%d\n",
		   (val & NETCRR_LOCK) ? 1 : 0,
		   (val & NETCRR_SR) ? 1 : 0);

	val = netc_reg_read(priv->prb, PRB_NETCSR);
	seq_printf(s, "[PRB NETCSR] State:%d Error:%d\n",
		   (val & NETCSR_STATE) ? 1 : 0,
		   (val & NETCSR_ERROR) ? 1 : 0);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(netc_prb);

static void netc_blk_ctrl_create_debugfs(struct netc_blk_ctrl *priv)
{
	struct dentry *root;

	root = debugfs_create_dir("netc_blk_ctrl", NULL);
	if (IS_ERR(root))
		return;

	priv->debugfs_root = root;

	debugfs_create_file("prb", 0444, root, priv, &netc_prb_fops);
}

static void netc_blk_ctrl_remove_debugfs(struct netc_blk_ctrl *priv)
{
	debugfs_remove_recursive(priv->debugfs_root);
	priv->debugfs_root = NULL;
}

#else

static void netc_blk_ctrl_create_debugfs(struct netc_blk_ctrl *priv)
{
}

static void netc_blk_ctrl_remove_debugfs(struct netc_blk_ctrl *priv)
{
}
#endif

static int netc_prb_check_error(struct netc_blk_ctrl *priv)
{
	u32 val;

	val = netc_reg_read(priv->prb, PRB_NETCSR);
	if (val & NETCSR_ERROR)
		return -1;

	return 0;
}

static const struct netc_devinfo imx95_devinfo = {
	.flags = NETC_HAS_NETCMIX,
	.num_link = IMX95_LINK_NUM,
	.netcmix_init = imx95_netcmix_init,
	.ierb_init = imx95_ierb_init,
};

static const struct netc_devinfo imx94_devinfo = {
	.flags = NETC_HAS_NETCMIX,
	.netcmix_init = imx94_netcmix_init,
	.ierb_init = imx94_ierb_init,
	.xpcs_port_init = imx94_netc_xpcs_port_init,
};

static const struct of_device_id netc_blk_ctrl_match[] = {
	{ .compatible = "nxp,imx95-netc-blk-ctrl", .data = &imx95_devinfo },
	{ .compatible = "nxp,imx94-netc-blk-ctrl", .data = &imx94_devinfo },
	{},
};
MODULE_DEVICE_TABLE(of, netc_blk_ctrl_match);

static int netc_blk_ctrl_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct netc_devinfo *devinfo;
	struct device *dev = &pdev->dev;
	const struct of_device_id *id;
	struct netc_blk_ctrl *priv;
	void __iomem *regs;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;
	priv->ipg_clk = devm_clk_get_optional(dev, "ipg_clk");
	if (IS_ERR(priv->ipg_clk)) {
		dev_err(dev, "Get ipg_clk failed\n");
		err = PTR_ERR(priv->ipg_clk);
		return err;
	}

	err = clk_prepare_enable(priv->ipg_clk);
	if (err) {
		dev_err(dev, "Enable ipg_clk failed\n");
		goto disable_ipg_clk;
	}

	id = of_match_device(netc_blk_ctrl_match, dev);
	if (!id) {
		dev_err(dev, "Cannot match device\n");
		err = -EINVAL;
		goto disable_ipg_clk;
	}

	devinfo = (struct netc_devinfo *)id->data;
	if (!devinfo) {
		dev_err(dev, "No device information\n");
		err = -EINVAL;
		goto disable_ipg_clk;
	}
	priv->devinfo = devinfo;

	regs = devm_platform_ioremap_resource_byname(pdev, "ierb");
	if (IS_ERR(regs)) {
		err = PTR_ERR(regs);
		dev_err(dev, "Missing IERB resource\n");
		goto disable_ipg_clk;
	}
	priv->ierb = regs;

	regs = devm_platform_ioremap_resource_byname(pdev, "prb");
	if (IS_ERR(regs)) {
		err = PTR_ERR(regs);
		dev_err(dev, "Missing PRB resource\n");
		goto disable_ipg_clk;
	}
	priv->prb = regs;

	if (devinfo->flags & NETC_HAS_NETCMIX) {
		regs = devm_platform_ioremap_resource_byname(pdev, "netcmix");
		if (IS_ERR(regs)) {
			err = PTR_ERR(regs);
			dev_err(dev, "Missing NETCMIX resource\n");
			goto disable_ipg_clk;
		}
		priv->netcmix = regs;
	}

	platform_set_drvdata(pdev, priv);

	if (devinfo->netcmix_init) {
		err = devinfo->netcmix_init(pdev);
		if (err) {
			dev_err(dev, "Initializing NETCMIX failed\n");
			goto disable_ipg_clk;
		}
	}

	err = netc_ierb_init(pdev);
	if (err) {
		dev_err(dev, "Initializing IERB failed.\n");
		goto disable_ipg_clk;
	}

	if (netc_prb_check_error(priv) < 0)
		dev_warn(dev, "The current IERB configuration is invalid.\n");

	netc_bc = priv;
	netc_blk_ctrl_create_debugfs(priv);

	err = of_platform_populate(node, NULL, NULL, dev);
	if (err) {
		dev_err(dev, "of_platform_populate failed\n");
		goto remove_debugfs;
	}

	return 0;

remove_debugfs:
	netc_blk_ctrl_remove_debugfs(priv);
	netc_bc = NULL;
disable_ipg_clk:
	clk_disable_unprepare(priv->ipg_clk);

	return err;
}

static void netc_blk_ctrl_remove(struct platform_device *pdev)
{
	struct netc_blk_ctrl *priv = platform_get_drvdata(pdev);

	of_platform_depopulate(&pdev->dev);
	netc_blk_ctrl_remove_debugfs(priv);
	netc_bc = NULL;
	clk_disable_unprepare(priv->ipg_clk);
}

static int netc_blk_ctrl_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct netc_blk_ctrl *priv = platform_get_drvdata(pdev);

	if (netc_ierb_may_wakeonlan())
		return 0;

	clk_disable_unprepare(priv->ipg_clk);

	return 0;
}

static int netc_blk_ctrl_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct netc_blk_ctrl *priv = platform_get_drvdata(pdev);
	const struct netc_devinfo *devinfo = priv->devinfo;
	int err;

	if (netc_ierb_may_wakeonlan())
		return 0;

	err = clk_prepare_enable(priv->ipg_clk);
	if (err) {
		dev_err(dev, "Enable ipg_clk failed\n");
		return err;
	}

	if (devinfo->netcmix_init) {
		err = devinfo->netcmix_init(pdev);
		if (err) {
			dev_err(dev, "Initializing NETCMIX failed\n");
			goto disable_ipg_clk;
		}
	}

	err = netc_ierb_init(pdev);
	if (err) {
		dev_err(&pdev->dev, "Initializing IERB failed.\n");
		goto disable_ipg_clk;
	}

	if (netc_prb_check_error(priv) < 0)
		dev_warn(&pdev->dev,
			 "The current IERB configuration is invalid.\n");

	return 0;

disable_ipg_clk:
	clk_disable_unprepare(priv->ipg_clk);

	return err;
}

static const struct dev_pm_ops __maybe_unused netc_blk_ctrl_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(netc_blk_ctrl_suspend_noirq,
				      netc_blk_ctrl_resume_noirq)
};

static struct platform_driver netc_blk_ctrl_driver = {
	.driver = {
		.name = "nxp-netc-blk-ctrl",
		.of_match_table = netc_blk_ctrl_match,
		.pm = pm_ptr(&netc_blk_ctrl_pm_ops),
	},
	.probe = netc_blk_ctrl_probe,
	.remove = netc_blk_ctrl_remove,
};

module_platform_driver(netc_blk_ctrl_driver);

MODULE_DESCRIPTION("NXP NETC Blocks Control Driver");
MODULE_LICENSE("Dual BSD/GPL");
