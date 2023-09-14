// SPDX-License-Identifier: GPL-2.0
/* Driver for MoreThanIP copper backplane AN/LT (Auto-Negotiation and
 * Link Training) core
 *
 * Copyright 2023 NXP
 */

#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/clk.h>

#define BP_ETH_STAT_ALWAYS_1		BIT(0)
#define BP_ETH_STAT_1GKX		BIT(1)
#define BP_ETH_STAT_10GKX4		BIT(2)
#define BP_ETH_STAT_10GKR		BIT(3)
#define BP_ETH_STAT_FC_FEC		BIT(4)
#define BP_ETH_STAT_40GKR4		BIT(5)
#define BP_ETH_STAT_40GCR4		BIT(6)
#define BP_ETH_STAT_RS_FEC		BIT(7)
#define BP_ETH_STAT_100GCR10		BIT(8)
#define BP_ETH_STAT_100GKP4		BIT(9)
#define BP_ETH_STAT_100GKR4		BIT(10)
#define BP_ETH_STAT_100GCR4		BIT(11)
#define BP_ETH_STAT_25GKRS		BIT(12)
#define BP_ETH_STAT_25GKR		BIT(13)

#define BP_ETH_STAT_PARALLEL_DETECT	(BP_ETH_STAT_ALWAYS_1 | \
					 BP_ETH_STAT_1GKX | \
					 BP_ETH_STAT_10GKX4)

#define LT_CTRL_RESTART_TRAINING	BIT(0)
#define LT_CTRL_TRAINING_ENABLE		BIT(1)

#define LT_STAT_RX_STATUS		BIT(0)
#define LT_STAT_FRAME_LOCK		BIT(1)
#define LT_STAT_IN_PROGRESS		BIT(2)
#define LT_STAT_FAIL			BIT(3)
#define LT_STAT_SIGNAL_DETECT		BIT(15)

#define LT_COM1_MASK			GENMASK(1, 0)
#define LT_COZ_MASK			GENMASK(3, 2)
#define LT_COP1_MASK			GENMASK(5, 4)
#define LT_COM1(x)			((x) & LT_COM1_MASK)
#define LT_COM1_X(x)			((x) & LT_COM1_MASK)
#define LT_COZ(x)			(((x) << 2) & LT_COZ_MASK)
#define LT_COZ_X(x)			(((x) & LT_COZ_MASK) >> 2)
#define LT_COP1(x)			(((x) << 4) & LT_COP1_MASK)
#define LT_COP1_X(x)			(((x) & LT_COP1_MASK) >> 4)

#define LT_COEF_STAT_MASK		(LT_COM1_MASK | LT_COZ_MASK | LT_COP1_MASK)
#define LT_COEF_STAT_ALL_NOT_UPDATED(x)	(((x) & LT_COEF_STAT_MASK) == 0)
#define LT_COEF_STAT_ANY_UPDATED(x)	(((x) & LT_COEF_STAT_MASK) != 0)

#define LT_COEF_UPD_MASK		(LT_COM1_MASK | LT_COZ_MASK | LT_COP1_MASK)
#define LT_COEF_UPD_ALL_HOLD		(LT_COM1(COEF_UPD_HOLD) | \
					LT_COZ(COEF_UPD_HOLD) | \
					LT_COP1(COEF_UPD_HOLD))

#define LT_COEF_UPD_ANYTHING(x)		((x) != 0)
#define LT_COEF_UPD_NOTHING(x)		((x) == 0)

#define LT_COEF_UPD_INIT		BIT(12)
#define LT_COEF_UPD_PRESET		BIT(13)

#define LT_COEF_STAT_RX_READY		BIT(15)

#define C73_ADV_0(x)			(u16)((x) & GENMASK(15, 0))
#define C73_ADV_1(x)			(u16)(((x) & GENMASK(31, 16)) >> 16)
#define C73_ADV_2(x)			(u16)(((x) & GENMASK_ULL(47, 32)) >> 32)

#define IRQPOLL_INTERVAL		(HZ / 4)

#define MTIP_CDR_SLEEP_US		100
#define MTIP_CDR_TIMEOUT_US		100000

#define MTIP_LT_RESTART_SLEEP_US	10
#define MTIP_LT_RESTART_TIMEOUT_US	100

#define MTIP_FRAME_LOCK_SLEEP_US	10
#define MTIP_FRAME_LOCK_TIMEOUT_US	100

#define MTIP_RESET_SLEEP_US		100
#define MTIP_RESET_TIMEOUT_US		100000

#define MTIP_BP_ETH_STAT_SLEEP_US	10
#define MTIP_BP_ETH_STAT_TIMEOUT_US	100

#define MTIP_COEF_STAT_SLEEP_US		10
#define MTIP_COEF_STAT_TIMEOUT_US	500000

#define MTIP_AN_TIMEOUT_MS		10000

enum mtip_an_reg {
	AN_CTRL,
	AN_STAT,
	AN_ADV_0,
	AN_ADV_1,
	AN_ADV_2,
	AN_LPA_0,
	AN_LPA_1,
	AN_LPA_2,
	AN_MS_CNT,
	AN_ADV_XNP_0,
	AN_ADV_XNP_1,
	AN_ADV_XNP_2,
	AN_LPA_XNP_0,
	AN_LPA_XNP_1,
	AN_LPA_XNP_2,
	AN_BP_ETH_STAT,
};

enum mtip_lt_reg {
	LT_CTRL,
	LT_STAT,
	LT_LP_COEF,
	LT_LP_STAT,
	LT_LD_COEF,
	LT_LD_STAT,
	LT_TRAIN_PATTERN,
	LT_RX_PATTERN,
	LT_RX_PATTERN_ERR,
	LT_RX_PATTERN_BEGIN,
};

struct mtip_irqpoll {
	struct phy_device *phydev;
	struct mutex lock;
	struct delayed_work work;
	u16 old_an_stat;
	u16 old_pcs_stat;
	bool link;
	bool link_ack;
	bool cdr_locked;
};

struct mtip_lt_work {
	struct phy_device *phydev;
	struct kthread_work work;
};

struct mtip_backplane {
	struct phy *serdes;
	struct mdio_device *pcs;
	const u16 *an_regs;
	const u16 *lt_regs;
	int lt_mmd;
	enum ethtool_link_mode_bit_indices link_mode;
	bool link_mode_resolved;
	unsigned long last_an_restart;
	struct mtip_irqpoll irqpoll;
	struct kthread_worker *local_tx_lt_worker;
	struct kthread_worker *remote_tx_lt_worker;
	/* Serialized by an_restart_lock */
	bool an_restart_pending;
	bool an_enabled;
	/* Used for orderly shutdown of LT threads. Modified without any
	 * locking. Set to true only by the irqpoll thread, set to false
	 * by irqpoll and by the LT threads.
	 */
	bool lt_enabled;
	bool local_tx_lt_done;
	bool remote_tx_lt_done;
	/* Serialize concurrent attempts from the local TX and remote TX
	 * kthreads to finalize their side of the link training
	 */
	struct mutex lt_lock;
	struct mutex an_restart_lock;
};

#define phydev_to_irqpoll(phydev)	\
	(&((struct mtip_backplane *)(phydev)->priv)->irqpoll)

/* Auto-Negotiation Control and Status Registers are on page 0: 0x0 */
static const u16 mtip_lx2160a_an_regs[] = {
	[AN_CTRL] = 0,
	[AN_STAT] = 1,
	[AN_ADV_0] = 2,
	[AN_ADV_1] = 3,
	[AN_ADV_2] = 4,
	[AN_LPA_0] = 5,
	[AN_LPA_1] = 6,
	[AN_LPA_2] = 7,
	[AN_MS_CNT] = 8,
	[AN_ADV_XNP_0] = 9,
	[AN_ADV_XNP_1] = 10,
	[AN_ADV_XNP_2] = 11,
	[AN_LPA_XNP_0] = 12,
	[AN_LPA_XNP_1] = 13,
	[AN_LPA_XNP_2] = 14,
	[AN_BP_ETH_STAT] = 15,
};

/* Link Training Control and Status Registers are on page 1: 256 = 0x100 */
static const u16 mtip_lx2160a_lt_regs[] = {
	[LT_CTRL] = 0x100,
	[LT_STAT] = 0x101,
	[LT_LP_COEF] = 0x102,
	[LT_LP_STAT] = 0x103,
	[LT_LD_COEF] = 0x104,
	[LT_LD_STAT] = 0x105,
	[LT_TRAIN_PATTERN] = 0x108,
	[LT_RX_PATTERN] = 0x109,
	[LT_RX_PATTERN_ERR] =  0x10a,
	[LT_RX_PATTERN_BEGIN] = 0x10b,
};

static const enum ethtool_link_mode_bit_indices mtip_backplane_link_modes[] = {
	ETHTOOL_LINK_MODE_10000baseKR_Full_BIT,
	ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
};

static const char *
ethtool_link_mode_str(enum ethtool_link_mode_bit_indices link_mode)
{
	switch (link_mode) {
	case ETHTOOL_LINK_MODE_1000baseKX_Full_BIT:
		return "1000base-KX";
	case ETHTOOL_LINK_MODE_10000baseKR_Full_BIT:
		return "10Gbase-KR";
	case ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT:
		return "40Gbase-KR4";
	case ETHTOOL_LINK_MODE_25000baseKR_Full_BIT:
		return "25Gbase-KR";
	case ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT:
		return "100Gbase-KR4";
	default:
		return "unknown";
	}
}

static bool
link_mode_needs_training(enum ethtool_link_mode_bit_indices link_mode)
{
	if (link_mode == ETHTOOL_LINK_MODE_1000baseKX_Full_BIT)
		return false;

	return true;
}

static int mtip_read_an(struct phy_device *phydev, enum mtip_an_reg reg)
{
	struct mtip_backplane *priv = phydev->priv;

	return phy_read_mmd(phydev, MDIO_MMD_AN, priv->an_regs[reg]);
}

static int mtip_write_an(struct phy_device *phydev, enum mtip_an_reg reg,
			 u16 val)
{
	struct mtip_backplane *priv = phydev->priv;

	return phy_write_mmd(phydev, MDIO_MMD_AN, priv->an_regs[reg], val);
}

static int mtip_modify_an(struct phy_device *phydev, enum mtip_an_reg reg,
			  u16 mask, u16 set)
{
	struct mtip_backplane *priv = phydev->priv;

	return phy_modify_mmd(phydev, MDIO_MMD_AN, priv->an_regs[reg],
			      mask, set);
}

static int mtip_read_lt(struct phy_device *phydev, enum mtip_lt_reg reg)
{
	struct mtip_backplane *priv = phydev->priv;

	return phy_read_mmd(phydev, priv->lt_mmd, priv->lt_regs[reg]);
}

static int mtip_write_lt(struct phy_device *phydev, enum mtip_lt_reg reg,
			 u16 val)
{
	struct mtip_backplane *priv = phydev->priv;

	return phy_write_mmd(phydev, priv->lt_mmd, priv->lt_regs[reg], val);
}

static int mtip_modify_lt(struct phy_device *phydev, enum mtip_lt_reg reg,
			  u16 mask, u16 set)
{
	struct mtip_backplane *priv = phydev->priv;

	return phy_modify_mmd(phydev, priv->lt_mmd, priv->lt_regs[reg],
			      mask, set);
}

static int mtip_read_pcs(struct phy_device *phydev, int reg)
{
	struct mtip_backplane *priv = phydev->priv;
	struct mdio_device *pcs = priv->pcs;

	return mdiodev_c45_read(pcs, MDIO_MMD_PCS, reg);
}

static int mtip_modify_pcs(struct phy_device *phydev, int reg, u16 mask,
			   u16 set)
{
	struct mtip_backplane *priv = phydev->priv;
	struct mdio_device *pcs = priv->pcs;

	return mdiodev_c45_modify(pcs, MDIO_MMD_PCS, reg, mask, set);
}

static int mtip_reset_an(struct phy_device *phydev)
{
	int err, val;

	err = mtip_modify_an(phydev, AN_CTRL, MDIO_CTRL1_RESET,
			     MDIO_CTRL1_RESET);
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_an, val,
				val < 0 || !(val & MDIO_CTRL1_RESET),
				MTIP_RESET_SLEEP_US, MTIP_RESET_TIMEOUT_US,
				false, phydev, AN_CTRL);

	return (val < 0) ? val : err;
}

static int mtip_reset_pcs(struct phy_device *phydev)
{
	int err, val;

	err = mtip_modify_pcs(phydev, MDIO_CTRL1, MDIO_CTRL1_RESET,
			      MDIO_CTRL1_RESET);
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_pcs, val,
				val < 0 || !(val & MDIO_CTRL1_RESET),
				MTIP_RESET_SLEEP_US, MTIP_RESET_TIMEOUT_US,
				false, phydev, MDIO_CTRL1);

	return (val < 0) ? val : err;
}

static int mtip_wait_for_cdr_lock(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	bool cdr_locked;
	int err, val;

	err = read_poll_timeout(phy_check_cdr_lock, val,
				val < 0 || cdr_locked,
				MTIP_CDR_SLEEP_US, MTIP_CDR_TIMEOUT_US,
				false, priv->serdes, &cdr_locked);

	return (val < 0) ? val : err;
}

static int mtip_lx2160a_get_features(struct phy_device *phydev)
{
	const enum ethtool_link_mode_bit_indices *link_modes;
	struct mtip_backplane *priv = phydev->priv;
	int i, err;

	linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Backplane_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, phydev->supported);

	link_modes = mtip_backplane_link_modes;

	/* Ask the SerDes driver what link modes are supported,
	 * based on the current PLL configuration.
	 */
	for (i = 0; i < ARRAY_SIZE(mtip_backplane_link_modes); i++) {
		err = phy_validate(priv->serdes, PHY_MODE_ETHERNET_PHY,
				   link_modes[i], NULL);
		if (err)
			continue;

		linkmode_set_bit(link_modes[i], phydev->supported);
	}

	return 0;
}

static int mtip_lt_frame_lock(struct phy_device *phydev)
{
	int err, val;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || (val & LT_STAT_FRAME_LOCK),
				MTIP_FRAME_LOCK_SLEEP_US, MTIP_FRAME_LOCK_TIMEOUT_US,
				false, phydev, LT_CTRL);

	return (val < 0) ? val : err;
}

static int mtip_restart_lt(struct phy_device *phydev, bool enable)
{
	u16 mask = LT_CTRL_RESTART_TRAINING | LT_CTRL_TRAINING_ENABLE;
	u16 set = LT_CTRL_RESTART_TRAINING;
	int err, val;

	if (enable)
		set |= LT_CTRL_TRAINING_ENABLE;

	err = mtip_modify_lt(phydev, LT_CTRL, mask, set);
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || !(val & LT_CTRL_RESTART_TRAINING),
				MTIP_LT_RESTART_SLEEP_US, MTIP_LT_RESTART_TIMEOUT_US,
				false, phydev, LT_CTRL);

	return (val < 0) ? val : err;
}

/* Enable the lane datapath by disconnecting it from the AN/LT block
 * and connecting it to the PCS. This is called both from the irqpoll thread,
 * as well as from the last link training kthread to finish.
 */
static int mtip_finish_lt(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	int err;

	err = mtip_restart_lt(phydev, false);
	if (err) {
		phydev_err(phydev, "Failed to disable link training: %pe\n",
			   ERR_PTR(err));
		return err;
	}

	/* Subsequent attempts to disable LT will time out, so stop them */
	WRITE_ONCE(priv->lt_enabled, false);

	return 0;
}

static int mtip_stop_lt(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);
	struct mtip_backplane *priv = phydev->priv;
	int err;

	lockdep_assert_held(&irqpoll->lock);

	kthread_flush_worker(priv->remote_tx_lt_worker);
	kthread_flush_worker(priv->local_tx_lt_worker);

	err = mtip_finish_lt(phydev);
	if (err)
		return err;

	phydev_dbg(phydev, "Link training disabled\n");

	return 0;
}

static int mtip_reset_lt(struct phy_device *phydev)
{
	int err;

	/* Don't allow AN to complete without training */
	err = mtip_modify_lt(phydev, LT_STAT, LT_STAT_RX_STATUS, 0);
	if (err < 0)
		return err;

	err = mtip_write_lt(phydev, LT_LD_COEF, 0);
	if (err < 0)
		return err;

	err = mtip_write_lt(phydev, LT_LD_STAT, 0);
	if (err < 0)
		return err;

	return 0;
}

/* Reset state when detecting that the previously determined link mode
 * is no longer valid
 */
static int mtip_state_reset(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	int err;

	priv->link_mode_resolved = false;

	if (READ_ONCE(priv->lt_enabled)) {
		err = mtip_stop_lt(phydev);
		if (err)
			return err;
	}

	err = mtip_reset_lt(phydev);
	if (err < 0)
		return err;

	return 0;
}

/* Make sure we don't act on old event bits from previous runs when
 * we restart autoneg.
 */
static int mtip_unlatch_an_stat(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);
	int val;

	lockdep_assert_held(&irqpoll->lock);

	val = mtip_read_an(phydev, AN_STAT);
	if (val < 0)
		return val;

	/* Discard the current AN status, it will become invalid soon */
	irqpoll->old_an_stat = 0;

	return 0;
}

/* Suppress a "PCS link dropped, restarting autoneg" event when initiating
 * an autoneg restart locally.
 */
static int mtip_unlatch_pcs_stat(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);

	irqpoll->old_pcs_stat = 0;

	return 0;
}

static int mtip_read_adv(struct phy_device *phydev, u64 *base_page)
{
	int val;

	val = mtip_read_an(phydev, AN_ADV_0);
	if (val < 0)
		return val;

	*base_page = (u64)val;

	val = mtip_read_an(phydev, AN_ADV_1);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 16;

	val = mtip_read_an(phydev, AN_ADV_2);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 32;

	return 0;
}

static int mtip_write_adv(struct phy_device *phydev, u64 base_page)
{
	int val;

	val = mtip_write_an(phydev, AN_ADV_0, C73_ADV_0(base_page));
	if (val < 0)
		return val;

	val = mtip_write_an(phydev, AN_ADV_1, C73_ADV_1(base_page));
	if (val < 0)
		return val;

	val = mtip_write_an(phydev, AN_ADV_2, C73_ADV_2(base_page));
	if (val < 0)
		return val;

	return 0;
}

static int mtip_read_lpa(struct phy_device *phydev, u64 *base_page)
{
	int val;

	val = mtip_read_an(phydev, AN_LPA_0);
	if (val < 0)
		return val;

	*base_page = (u64)val;

	val = mtip_read_an(phydev, AN_LPA_1);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 16;

	val = mtip_read_an(phydev, AN_LPA_2);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 32;

	return 0;
}

static int mtip_config_an_adv(struct phy_device *phydev)
{
	u64 base_page = linkmode_adv_to_c73_base_page(phydev->advertising);
	u8 nonce;

	/* The transmitted nonce must not be equal with the one transmitted by
	 * the link partner, otherwise AN will not complete (nonce_match=true).
	 * With purely randomly generated nonces, that is always a chance
	 * though. 98.2.1.2.3 Transmitted Nonce Field gives a way for
	 * management to avoid that.
	 */
	get_random_bytes(&nonce, sizeof(nonce));

	switch (phydev->master_slave_set) {
	case MASTER_SLAVE_CFG_MASTER_PREFERRED:
	case MASTER_SLAVE_CFG_MASTER_FORCE:
		nonce |= BIT(4);
		break;
	case MASTER_SLAVE_CFG_SLAVE_PREFERRED:
	case MASTER_SLAVE_CFG_SLAVE_FORCE:
		nonce &= ~BIT(4);
		break;
	}

	base_page |= C73_BASE_PAGE_TRANSMITTED_NONCE(nonce);
	/* According to Annex 28A, set Selector to "IEEE 802.3" */
	base_page |= C73_BASE_PAGE_SELECTOR(1);
	/* C73_BASE_PAGE_ACK and C73_BASE_PAGE_ECHOED_NONCE seem to have
	 * a life of their own, regardless of what we set them to.
	 */

	return mtip_write_adv(phydev, base_page);
}

static int mtip_an_restart(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	int err;

	err = mtip_state_reset(phydev);
	if (err)
		return err;

	/* Make sure AN is temporarily disabled, so that we can safely
	 * unlatch the previous status without losing real events
	 */
	err = mtip_reset_an(phydev);
	if (err < 0)
		return err;

	err = mtip_unlatch_an_stat(phydev);
	if (err)
		return err;

	err = mtip_unlatch_pcs_stat(phydev);
	if (err)
		return err;

	err = mtip_config_an_adv(phydev);
	if (err < 0)
		return err;

	err = mtip_modify_an(phydev, AN_CTRL,
			     MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART,
			     MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART);
	if (err < 0)
		return err;

	priv->last_an_restart = jiffies;
	priv->an_restart_pending = false;

	return 0;
}

/* The reason for deferral is that the irqpoll thread waits for the LT kthreads
 * to finish with irqpoll->lock held, and AN restart also requires holding the
 * irqpoll->lock. So the kthreads cannot directly restart autoneg without
 * deadlocking with the irqpoll thread, they must signal to the irqpoll thread
 * to do so.
 */
static void mtip_an_restart_from_lt(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;

	mutex_lock(&priv->an_restart_lock);
	priv->an_restart_pending = true;
	mutex_unlock(&priv->an_restart_lock);
}

static int mtip_finalize_lt(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	union phy_configure_opts opts = {
		.xgkr = {
			.type = XGKR_CONFIGURE_LT_DONE,
		},
	};
	int err, val;

	lockdep_assert_held(&priv->lt_lock);

	if (!priv->local_tx_lt_done || !priv->remote_tx_lt_done)
		return 0;

	/* Let the local state machine know we're done */
	err = mtip_modify_lt(phydev, LT_STAT, LT_STAT_RX_STATUS,
			     LT_STAT_RX_STATUS);
	if (err < 0) {
		phydev_err(phydev, "Failed to update LT_STAT: %pe\n",
			   ERR_PTR(err));
		return err;
	}

	err = mtip_finish_lt(phydev);
	if (err)
		return err;

	val = mtip_read_lt(phydev, LT_STAT);
	if (val < 0)
		return val;

	if (!(val & LT_STAT_SIGNAL_DETECT) || (val & LT_STAT_FAIL)) {
		phydev_err(phydev,
			   "Link training did not succeed: LT_STAT = 0x%x\n",
			   val);
		return -ENETDOWN;
	}

	return phy_configure(priv->serdes, &opts);
}

static int mtip_tx_c72_coef_update(struct phy_device *phydev,
				   const struct c72_coef_update *upd,
				   struct c72_coef_status *stat)
{
	char upd_buf[C72_COEF_UPDATE_BUFSIZ], stat_buf[C72_COEF_STATUS_BUFSIZ];
	int err, val;
	u16 ld_coef;

	c72_coef_update_print(upd, upd_buf);

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || LT_COEF_STAT_ALL_NOT_UPDATED(val),
				MTIP_COEF_STAT_SLEEP_US, MTIP_COEF_STAT_TIMEOUT_US,
				false, phydev, LT_LP_STAT);
	if (val < 0)
		return val;
	if (err) {
		phydev_err(phydev,
			   "LP did not set coefficient status to NOT_UPDATED, couldn't send %s; LP_STAT = 0x%x\n",
			   upd_buf, val);
		return err;
	}

	ld_coef = LT_COM1(upd->com1) | LT_COZ(upd->coz) | LT_COP1(upd->cop1);
	if (upd->init)
		ld_coef |= LT_COEF_UPD_INIT;
	if (upd->preset)
		ld_coef |= LT_COEF_UPD_PRESET;

	err = mtip_write_lt(phydev, LT_LD_COEF, ld_coef);
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || LT_COEF_STAT_ANY_UPDATED(val),
				MTIP_COEF_STAT_SLEEP_US, MTIP_COEF_STAT_TIMEOUT_US,
				false, phydev, LT_LP_STAT);
	if (val < 0)
		return val;
	if (err) {
		phydev_err(phydev,
			   "LP did not update coefficient status for %s; LP_STAT = 0x%x\n",
			   upd_buf, val);
		return err;
	}

	stat->com1 = LT_COM1_X(val);
	stat->coz = LT_COZ_X(val);
	stat->cop1 = LT_COP1_X(val);
	c72_coef_status_print(stat, stat_buf);

	ld_coef = LT_COM1(COEF_UPD_HOLD) | LT_COZ(COEF_UPD_HOLD) |
		  LT_COP1(COEF_UPD_HOLD);
	err = mtip_write_lt(phydev, LT_LD_COEF, ld_coef);
	if (err < 0)
		return err;

	phydev_dbg(phydev, "Sent update: %s, got status: %s\n",
		   upd_buf, stat_buf);

	return 0;
}

static int mtip_c72_process_request(struct phy_device *phydev,
				    const struct c72_coef_update *upd,
				    struct c72_coef_status *stat)
{
	struct mtip_backplane *priv = phydev->priv;
	union phy_configure_opts opts = {
		.xgkr = {
			.type = XGKR_CONFIGURE_LOCAL_TX,
			.local_tx = {
				.update = *upd,
			},
		},
	};
	int err;

	err = phy_configure(priv->serdes, &opts);
	if (err)
		return err;

	*stat = opts.xgkr.local_tx.status;

	return 0;
}

static int mtip_read_lt_lp_coef_if_not_ready(struct phy_device *phydev,
					     bool *rx_ready)
{
	int val;

	val = mtip_read_lt(phydev, LT_LP_STAT);
	if (val < 0)
		return val;

	*rx_ready = !!(val & LT_COEF_STAT_RX_READY);
	if (*rx_ready)
		return 0;

	return mtip_read_lt(phydev, LT_LP_COEF);
}

static int mtip_rx_c72_coef_update(struct phy_device *phydev,
				   struct c72_coef_update *upd,
				   bool *rx_ready)
{
	char upd_buf[C72_COEF_UPDATE_BUFSIZ], stat_buf[C72_COEF_STATUS_BUFSIZ];
	struct c72_coef_status stat = {};
	int err, val;

	err = read_poll_timeout(mtip_read_lt_lp_coef_if_not_ready,
				val, val < 0 || *rx_ready || LT_COEF_UPD_ANYTHING(val),
				MTIP_COEF_STAT_SLEEP_US, MTIP_COEF_STAT_TIMEOUT_US,
				false, phydev, rx_ready);
	if (val < 0)
		return val;
	if (*rx_ready) {
		phydev_dbg(phydev, "LP says its RX is ready\n");
		return 0;
	}
	if (err) {
		phydev_err(phydev,
			   "LP did not request coefficient updates; LP_COEF = 0x%x\n",
			   val);
		return err;
	}

	upd->com1 = LT_COM1_X(val);
	upd->coz = LT_COZ_X(val);
	upd->cop1 = LT_COP1_X(val);
	upd->init = !!(val & LT_COEF_UPD_INIT);
	upd->preset = !!(val & LT_COEF_UPD_PRESET);
	c72_coef_update_print(upd, upd_buf);

	if ((upd->com1 || upd->coz || upd->cop1) + upd->init + upd->preset > 1) {
		phydev_warn(phydev, "Ignoring illegal request: %s (LP_COEF = 0x%x)\n",
			    upd_buf, val);
		return 0;
	}

	err = mtip_c72_process_request(phydev, upd, &stat);
	if (err)
		return err;

	c72_coef_status_print(&stat, stat_buf);
	phydev_dbg(phydev, "Received update: %s, responded with status: %s\n",
		   upd_buf, stat_buf);

	err = mtip_modify_lt(phydev, LT_LD_STAT, LT_COEF_STAT_MASK,
			     LT_COM1(stat.com1) | LT_COZ(stat.coz) |
			     LT_COP1(stat.cop1));
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || LT_COEF_UPD_NOTHING(val),
				MTIP_COEF_STAT_SLEEP_US, MTIP_COEF_STAT_TIMEOUT_US,
				false, phydev, LT_LP_COEF);
	if (val < 0)
		return val;
	if (err) {
		phydev_err(phydev, "LP did not revert to HOLD; LP_COEF = 0x%x\n",
			   val);
		return err;
	}

	err = mtip_modify_lt(phydev, LT_LD_STAT, LT_COEF_STAT_MASK,
			     LT_COM1(COEF_STAT_NOT_UPDATED) |
			     LT_COZ(COEF_STAT_NOT_UPDATED) |
			     LT_COP1(COEF_STAT_NOT_UPDATED));
	if (err < 0)
		return err;

	return 0;
}

static int mtip_local_tx_lt_done(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	int err;

	mutex_lock(&priv->lt_lock);

	priv->local_tx_lt_done = true;

	err = mtip_finalize_lt(phydev);

	mutex_unlock(&priv->lt_lock);

	return err;
}

static int mtip_remote_tx_lt_done(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	int err;

	mutex_lock(&priv->lt_lock);

	priv->remote_tx_lt_done = true;

	err = mtip_finalize_lt(phydev);

	mutex_unlock(&priv->lt_lock);

	return err;
}

/* This is our hardware-based 500 ms timer for the link training */
static bool mtip_lt_in_progress(struct phy_device *phydev)
{
	int val;

	val = mtip_read_lt(phydev, LT_STAT);
	if (val < 0)
		return false;

	return !!(val & LT_STAT_IN_PROGRESS);
}

/* Make adjustments to the local TX according to link partner requests,
 * so that its RX improves
 */
static void mtip_local_tx_lt_work(struct kthread_work *work)
{
	struct mtip_lt_work *lt_work = container_of(work, struct mtip_lt_work,
						    work);
	struct phy_device *phydev = lt_work->phydev;
	struct mtip_backplane *priv = phydev->priv;
	int err;

	err = mtip_lt_frame_lock(phydev);
	if (err) {
		phydev_err(phydev,
			   "Failed to acquire training frame delineation: %pe\n",
			   ERR_PTR(err));
		goto out;
	}

	while (READ_ONCE(priv->lt_enabled)) {
		struct c72_coef_update upd = {};
		bool rx_ready;

		if (!mtip_lt_in_progress(phydev)) {
			phydev_err(phydev, "Local TX LT timed out\n");
			break;
		}

		err = mtip_rx_c72_coef_update(phydev, &upd, &rx_ready);
		if (err)
			goto out;

		if (rx_ready) {
			err = mtip_local_tx_lt_done(phydev);
			if (err) {
				phydev_err(phydev, "Failed to finalize local LT: %pe\n",
					   ERR_PTR(err));
				goto out;
			}
			break;
		}
	}

out:
	if (err && READ_ONCE(priv->lt_enabled))
		mtip_an_restart_from_lt(phydev);

	kfree(lt_work);
}

/* Train the link partner TX, so that the local RX quality improves */
static void mtip_remote_tx_lt_work(struct kthread_work *work)
{
	struct mtip_lt_work *lt_work = container_of(work, struct mtip_lt_work,
						    work);
	struct phy_device *phydev = lt_work->phydev;
	struct mtip_backplane *priv = phydev->priv;
	int err;

	err = mtip_lt_frame_lock(phydev);
	if (err) {
		phydev_err(phydev,
			   "Failed to acquire training frame delineation: %pe\n",
			   ERR_PTR(err));
		goto out;
	}

	while (true) {
		struct c72_coef_status status = {};
		union phy_configure_opts opts = {
			.xgkr = {
				.type = XGKR_CONFIGURE_REMOTE_TX,
			},
		};

		if (!READ_ONCE(priv->lt_enabled))
			goto out;

		if (!mtip_lt_in_progress(phydev)) {
			phydev_err(phydev, "Remote TX LT timed out\n");
			goto out;
		}

		err = phy_configure(priv->serdes, &opts);
		if (err) {
			phydev_err(phydev,
				   "Failed to get remote TX training request from SerDes: %pe\n",
				   ERR_PTR(err));
			goto out;
		}

		if (opts.xgkr.remote_tx.rx_ready)
			break;

		err = mtip_tx_c72_coef_update(phydev, &opts.xgkr.remote_tx.update,
					      &status);
		if (opts.xgkr.remote_tx.cb)
			opts.xgkr.remote_tx.cb(opts.xgkr.remote_tx.cb_priv, err,
					       opts.xgkr.remote_tx.update,
					       status);
		if (err)
			goto out;
	}

	/* Let the link partner know we're done */
	err = mtip_modify_lt(phydev, LT_LD_STAT, LT_COEF_STAT_RX_READY,
			     LT_COEF_STAT_RX_READY);
	if (err < 0) {
		phydev_err(phydev, "Failed to update LT_LD_STAT: %pe\n",
			   ERR_PTR(err));
		goto out;
	}

	err = mtip_remote_tx_lt_done(phydev);
	if (err) {
		phydev_err(phydev, "Failed to finalize remote LT: %pe\n",
			   ERR_PTR(err));
		goto out;
	}

out:
	if (err && READ_ONCE(priv->lt_enabled))
		mtip_an_restart_from_lt(phydev);

	kfree(lt_work);
}

static int mtip_start_lt(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);
	struct mtip_backplane *priv = phydev->priv;
	struct mtip_lt_work *remote_tx_lt_work;
	struct mtip_lt_work *local_tx_lt_work;
	int err;

	lockdep_assert_held(&irqpoll->lock);

	local_tx_lt_work = kzalloc(sizeof(*local_tx_lt_work), GFP_KERNEL);
	if (!local_tx_lt_work) {
		err = -ENOMEM;
		goto out;
	}

	remote_tx_lt_work = kzalloc(sizeof(*remote_tx_lt_work), GFP_KERNEL);
	if (!remote_tx_lt_work) {
		err = -ENOMEM;
		goto out_free_local_tx_lt;
	}

	err = mtip_reset_lt(phydev);
	if (err)
		goto out_free_remote_tx_lt;

	err = mtip_restart_lt(phydev, true);
	if (err)
		goto out_free_remote_tx_lt;

	priv->local_tx_lt_done = false;
	priv->remote_tx_lt_done = false;
	WRITE_ONCE(priv->lt_enabled, true);

	local_tx_lt_work->phydev = phydev;
	kthread_init_work(&local_tx_lt_work->work, mtip_local_tx_lt_work);
	kthread_queue_work(priv->local_tx_lt_worker, &local_tx_lt_work->work);

	remote_tx_lt_work->phydev = phydev;
	kthread_init_work(&remote_tx_lt_work->work, mtip_remote_tx_lt_work);
	kthread_queue_work(priv->remote_tx_lt_worker, &remote_tx_lt_work->work);

	phydev_dbg(phydev, "Link training enabled\n");

	return 0;

out_free_remote_tx_lt:
	kfree(remote_tx_lt_work);
out_free_local_tx_lt:
	kfree(local_tx_lt_work);
out:
	return err;
}

static void mtip_update_link_latch(struct phy_device *phydev,
				   bool cdr_locked, bool phy_los,
				   bool an_complete, bool pcs_lstat)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);
	struct mtip_backplane *priv = phydev->priv;

	lockdep_assert_held(&irqpoll->lock);

	/* Update irqpoll->link if true, or if false
	 * and mtip_read_status() saw that already.
	 */
	if (irqpoll->link || irqpoll->link_ack) {
		irqpoll->link = phy_los && cdr_locked && an_complete && pcs_lstat;
		irqpoll->link_ack = false;
	}

	phydev_dbg(phydev, "PCS link%s: %d, PHY LOS: %d, CDR locked: %d, AN complete: %d\n",
		   priv->link_mode_resolved ? "" : " (ignored)",
		   pcs_lstat, phy_los, cdr_locked, an_complete);
}

static bool mtip_cached_an_complete(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);

	lockdep_assert_held(&irqpoll->lock);

	return !!(irqpoll->old_an_stat & MDIO_AN_STAT1_COMPLETE);
}

static bool mtip_read_link_unlatch(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);
	bool old_link = irqpoll->link;

	lockdep_assert_held(&irqpoll->lock);

	/* A change to the link status may have occurred while a link
	 * loss was latched, so update it again after reading it
	 */
	irqpoll->link = !!(irqpoll->old_an_stat & MDIO_STAT1_LSTATUS) &&
			!!(irqpoll->old_an_stat & MDIO_AN_STAT1_COMPLETE) &&
			!!(irqpoll->old_pcs_stat & MDIO_STAT1_LSTATUS) &&
			irqpoll->cdr_locked;
	irqpoll->link_ack = true;

	return old_link;
}

static u16 mtip_expected_bp_eth_stat(enum ethtool_link_mode_bit_indices link_mode)
{
	switch (link_mode) {
	case ETHTOOL_LINK_MODE_1000baseKX_Full_BIT:
		return BP_ETH_STAT_ALWAYS_1 | BP_ETH_STAT_1GKX;
	case ETHTOOL_LINK_MODE_10000baseKR_Full_BIT:
		return BP_ETH_STAT_ALWAYS_1 | BP_ETH_STAT_10GKR;
	case ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT:
		return BP_ETH_STAT_ALWAYS_1 | BP_ETH_STAT_40GKR4;
	case ETHTOOL_LINK_MODE_25000baseKR_Full_BIT:
		return BP_ETH_STAT_ALWAYS_1 | BP_ETH_STAT_25GKR;
	case ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT:
		return BP_ETH_STAT_ALWAYS_1 | BP_ETH_STAT_100GKR4;
	default:
		return 0;
	}
}

static int mtip_wait_bp_eth_stat(struct phy_device *phydev,
				 enum ethtool_link_mode_bit_indices link_mode)
{
	u16 expected = mtip_expected_bp_eth_stat(link_mode);
	int err, val;

	err = read_poll_timeout(mtip_read_an, val,
				val < 0 || val == expected,
				MTIP_BP_ETH_STAT_SLEEP_US,
				MTIP_BP_ETH_STAT_TIMEOUT_US,
				false, phydev, AN_BP_ETH_STAT);
	if (val < 0)
		return val;

	if (err) {
		phydev_warn(phydev,
			    "BP_ETH_STAT did not become 0x%x to indicate resolved link mode %s, instead it shows 0x%x%s\n",
			    expected, ethtool_link_mode_str(link_mode), val,
			    val == BP_ETH_STAT_PARALLEL_DETECT ? " (parallel detection)" : "");
		/* It seems less consequential to ignore the error
		 * than to restart autoneg...
		 */
	}

	return 0;
}

static int mtip_c73_page_received(struct phy_device *phydev, bool *restart_an)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(lp_advertising);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(advertising);
	enum ethtool_link_mode_bit_indices resolved;
	struct mtip_backplane *priv = phydev->priv;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(common);
	u64 base_page, lpa;
	int err;

	err = mtip_state_reset(phydev);
	if (err)
		return err;

	err = mtip_read_adv(phydev, &base_page);
	if (err < 0)
		return err;

	err = mtip_read_lpa(phydev, &lpa);
	if (err < 0)
		return err;

	if (lpa & C73_BASE_PAGE_NP)
		phydev_warn(phydev, "Next Page exchange not implemented!\n");

	mii_c73_mod_linkmode_lpa_t(advertising, base_page);
	mii_c73_mod_linkmode_lpa_t(lp_advertising, lpa);
	linkmode_and(common, advertising, lp_advertising);

	err = linkmode_c73_priority_resolution(common, &resolved);
	if (err) {
		phydev_dbg(phydev, "C73 page received, no common link mode\n");
		return 0;
	}

	err = mtip_wait_bp_eth_stat(phydev, resolved);
	if (err) {
		*restart_an = true;
		return 0;
	}

	phydev_dbg(phydev,
		   "C73 page received, LD %04x:%04x:%04x, LP %04x:%04x:%04x, resolved link mode %s\n",
		   C73_ADV_2(base_page), C73_ADV_1(base_page), C73_ADV_0(base_page),
		   C73_ADV_2(lpa), C73_ADV_1(lpa), C73_ADV_0(lpa),
		   ethtool_link_mode_str(resolved));

	err = phy_set_mode_ext(priv->serdes, PHY_MODE_ETHERNET_PHY, resolved);
	if (err) {
		phydev_err(phydev, "phy_set_mode_ext(%s) returned %pe\n",
			   ethtool_link_mode_str(resolved), ERR_PTR(err));
		return err;
	}

	err = mtip_wait_for_cdr_lock(phydev);
	if (err)
		phydev_warn(phydev, "Failed to reacquire CDR lock after protocol change: %pe\n",
			    ERR_PTR(err));

	if (link_mode_needs_training(resolved)) {
		err = mtip_start_lt(phydev);
		if (err)
			return err;
	} else {
		/* Allow the datapath to come up without link training */
		err = mtip_modify_lt(phydev, LT_STAT, LT_STAT_RX_STATUS,
				     LT_STAT_RX_STATUS);
		if (err < 0)
			return err;
	}

	priv->link_mode = resolved;
	priv->link_mode_resolved = true;

	return 0;
}

static void mtip_c73_remote_fault(struct phy_device *phydev, bool fault)
{
	phydev_err(phydev, "Remote fault: %d\n", fault);
}

static void mtip_c73_parallel_detection_fault(struct phy_device *phydev,
					      bool fault)
{
	phydev_err(phydev, "Parallel detection fault: %d\n", fault);
}

static void mtip_irqpoll_work(struct work_struct *work)
{
	struct mtip_irqpoll *irqpoll = container_of(work, struct mtip_irqpoll, work.work);
	struct phy_device *phydev = irqpoll->phydev;
	struct mtip_backplane *priv = phydev->priv;
	bool restart_an = false;
	int val, err = 0;
	int pcs_stat = 0;
	bool cdr_locked;

	/* Check for AN restart requests from the link training kthreads */
	mutex_lock(&priv->an_restart_lock);
	if (priv->an_restart_pending) {
		restart_an = true;
		priv->an_restart_pending = false;
	}
	mutex_unlock(&priv->an_restart_lock);

	/* Then enter the irqpoll logic per se
	 * (PCS MDIO_STAT1, AN/LT MDIO_STAT1 and CDR lock)
	 */
	mutex_lock(&irqpoll->lock);

	err = phy_check_cdr_lock(priv->serdes, &cdr_locked);
	if (err)
		goto out_unlock;

	if (priv->link_mode_resolved) {
		pcs_stat = mtip_read_pcs(phydev, MDIO_STAT1);
		if (pcs_stat < 0) {
			err = pcs_stat;
			goto out_unlock;
		}
	}

	val = mtip_read_an(phydev, AN_STAT);
	if (val < 0) {
		err = val;
		goto out_unlock;
	}

	if ((irqpoll->cdr_locked != cdr_locked) ||
	    ((irqpoll->old_an_stat ^ val) & (MDIO_STAT1_LSTATUS |
					     MDIO_AN_STAT1_COMPLETE)) ||
	    ((irqpoll->old_pcs_stat ^ pcs_stat) & MDIO_STAT1_LSTATUS)) {
		mtip_update_link_latch(phydev, cdr_locked,
				       !!(val & MDIO_STAT1_LSTATUS),
				       !!(val & MDIO_AN_STAT1_COMPLETE),
				       !!(pcs_stat & MDIO_STAT1_LSTATUS));
	}

	/* The manual says that this bit is latched high, but experimentation
	 * shows that reads will not unlatch it while link training is in
	 * progress; only reading it after link training has completed will.
	 * Only act upon bit transitions, to avoid processing a false "page
	 * received" event during link training.
	 */
	if (((irqpoll->old_an_stat ^ val) & MDIO_AN_STAT1_PAGE) &&
	    (val & MDIO_AN_STAT1_PAGE)) {
		err = mtip_c73_page_received(phydev, &restart_an);
		if (err)
			goto out_unlock;
	}

	if ((irqpoll->old_an_stat ^ val) & MDIO_AN_STAT1_RFAULT)
		mtip_c73_remote_fault(phydev, val & MDIO_AN_STAT1_RFAULT);

	if ((irqpoll->old_an_stat ^ val) & MDIO_AN_STAT1_PDET_FAULT)
		mtip_c73_parallel_detection_fault(phydev, val &
						  MDIO_AN_STAT1_PDET_FAULT);

	/* Checks that result in AN restart should go at the end */

	/* Make sure the lane goes back into DME page exchange mode
	 * after a link drop
	 */
	if (priv->link_mode_resolved &&
	    (irqpoll->old_pcs_stat & MDIO_STAT1_LSTATUS) &&
	    !(pcs_stat & MDIO_STAT1_LSTATUS)) {
		phydev_dbg(phydev, "PCS link dropped, restarting autoneg\n");
		restart_an = true;
	}

	/* Paranoid workaround for undetermined issue */
	if (!priv->link_mode_resolved && (val & MDIO_AN_STAT1_COMPLETE) &&
	    priv->an_enabled && time_after(jiffies, priv->last_an_restart +
					   msecs_to_jiffies(MTIP_AN_TIMEOUT_MS))) {
		phydev_err(phydev,
			   "Hardware says AN has completed, but we never saw a base page, and that's bogus\n");
		restart_an = true;
	}

	if (restart_an) {
		err = mtip_an_restart(phydev);
		if (err)
			goto out_unlock;

		/* don't overwrite what was set by mtip_unlatch_an_stat() */
		goto ignore_an_and_pcs_stat;
	}

	irqpoll->old_an_stat = val;
	irqpoll->old_pcs_stat = pcs_stat;
ignore_an_and_pcs_stat:
	irqpoll->cdr_locked = cdr_locked;

out_unlock:
	mutex_unlock(&irqpoll->lock);

	if (err) {
		phy_error(phydev);
		return;
	}

	schedule_delayed_work(&irqpoll->work, IRQPOLL_INTERVAL);
}

static int mtip_parse_dt(struct phy_device *phydev)
{
	struct device_node *dn = phydev->mdio.dev.of_node;
	struct mtip_backplane *priv = phydev->priv;
	struct device_node *pcs_node;

	priv->serdes = of_phy_get(dn, NULL);
	if (IS_ERR(priv->serdes))
		return PTR_ERR(priv->serdes);

	pcs_node = of_parse_phandle(dn, "pcs-handle", 0);
	if (pcs_node) {
		if (!of_device_is_available(pcs_node)) {
			phydev_err(phydev, "pcs-handle node not available\n");
			of_node_put(pcs_node);
			return -ENODEV;
		}

		priv->pcs = of_mdio_find_device(pcs_node);
		of_node_put(pcs_node);
		if (!priv->pcs) {
			phydev_err(phydev, "missing PCS device\n");
			return -EPROBE_DEFER;
		}
	} else {
		priv->pcs = &phydev->mdio;
	}

	return 0;
}

static bool mtip_is_lx2160a(struct phy_device *phydev)
{
	return of_device_is_compatible(phydev->mdio.dev.of_node,
				       "fsl,lx2160a-backplane-anlt");
}

static int mtip_lx2160a_match_phy_device(struct phy_device *phydev)
{
	return mtip_is_lx2160a(phydev);
}

static void mtip_irqpoll_init(struct phy_device *phydev,
			      struct mtip_irqpoll *irqpoll)
{
	mutex_init(&irqpoll->lock);
	INIT_DELAYED_WORK(&irqpoll->work, mtip_irqpoll_work);
	irqpoll->phydev = phydev;
}

static void mtip_start_irqpoll(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);

	schedule_delayed_work(&irqpoll->work, 0);
}

static void mtip_stop_irqpoll(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);

	cancel_delayed_work_sync(&irqpoll->work);
}

static int mtip_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct mtip_backplane *priv;
	int err;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto out;
	}

	phydev->port = PORT_DA;

	if (mtip_is_lx2160a(phydev)) {
		priv->an_regs = mtip_lx2160a_an_regs;
		priv->lt_regs = mtip_lx2160a_lt_regs;
		priv->lt_mmd = MDIO_MMD_AN;
	} /* else TODO */

	mtip_irqpoll_init(phydev, &priv->irqpoll);
	mutex_init(&priv->an_restart_lock);
	mutex_init(&priv->lt_lock);
	phydev->priv = priv;

	priv->local_tx_lt_worker = kthread_create_worker(0, "%s_local_tx_lt",
							 dev_name(dev));
	if (IS_ERR(priv->local_tx_lt_worker)) {
		err = PTR_ERR(priv->local_tx_lt_worker);
		goto out_free_priv;
	}

	priv->remote_tx_lt_worker = kthread_create_worker(0, "%s_remote_tx_lt",
							  dev_name(dev));
	if (IS_ERR(priv->remote_tx_lt_worker)) {
		err = PTR_ERR(priv->remote_tx_lt_worker);
		goto out_destroy_local_tx_lt;
	}

	err = mtip_parse_dt(phydev);
	if (err)
		goto out_destroy_remote_tx_lt;

	err = phy_init(priv->serdes);
	if (err) {
		phydev_err(phydev, "Failed to initialize SerDes: %pe\n",
			   ERR_PTR(err));
		goto out_put_phy;
	}

	mtip_start_irqpoll(phydev);

	return 0;

out_put_phy:
	of_phy_put(priv->serdes);
out_destroy_remote_tx_lt:
	kthread_destroy_worker(priv->remote_tx_lt_worker);
out_destroy_local_tx_lt:
	kthread_destroy_worker(priv->local_tx_lt_worker);
out_free_priv:
	kfree(priv);
out:
	return err;
}

static void mtip_remove(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;

	mtip_stop_irqpoll(phydev);
	phy_exit(priv->serdes);
	of_phy_put(priv->serdes);
	kthread_destroy_worker(priv->remote_tx_lt_worker);
	kthread_destroy_worker(priv->local_tx_lt_worker);
	kfree(priv);
}

static int mtip_config_aneg(struct phy_device *phydev)
{
	u16 mask = MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART;
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);
	struct mtip_backplane *priv = phydev->priv;
	int err;

	mutex_lock(&irqpoll->lock);

	/* We support anything */
	phydev->master_slave_get = phydev->master_slave_set;

	/* Only allow advertising what this PHY supports. The device tree
	 * property "max-speed" may further limit the speed and thus the
	 * link modes. Similar to genphy_config_advert().
	 */
	linkmode_and(phydev->advertising, phydev->advertising,
		     phydev->supported);

	if (phydev->autoneg == AUTONEG_ENABLE) {
		err = mtip_an_restart(phydev);
		if (err)
			goto out_unlock;

		priv->an_enabled = true;
	} else {
		err = mtip_modify_an(phydev, AN_CTRL, mask, 0);
		if (err < 0)
			goto out_unlock;

		priv->an_enabled = false;
	}

out_unlock:
	mutex_unlock(&irqpoll->lock);

	return err;
}

static int mtip_resolve_aneg_linkmode(struct phy_device *phydev)
{
	u64 base_page;
	int err;

	linkmode_zero(phydev->lp_advertising);

	err = mtip_read_lpa(phydev, &base_page);
	if (err)
		return err;

	mii_c73_mod_linkmode_lpa_t(phydev->lp_advertising, base_page);
	phy_resolve_aneg_linkmode(phydev);

	return 0;
}

static int mtip_read_status(struct phy_device *phydev)
{
	struct mtip_irqpoll *irqpoll = phydev_to_irqpoll(phydev);
	u64 base_page;
	int err = 0;

	mutex_lock(&irqpoll->lock);

	err = mtip_read_adv(phydev, &base_page);
	if (err)
		goto out_unlock;

	if (C73_BASE_PAGE_TRANSMITTED_NONCE_X(base_page) & BIT(4))
		phydev->master_slave_state = MASTER_SLAVE_STATE_MASTER;
	else
		phydev->master_slave_state = MASTER_SLAVE_STATE_SLAVE;

	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->pause = 0;
	phydev->asym_pause = 0;

	phydev->link = mtip_read_link_unlatch(phydev);
	if (!phydev->link)
		goto out_unlock;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		phydev->autoneg_complete = mtip_cached_an_complete(phydev);

		if (phydev->autoneg_complete)
			err = mtip_resolve_aneg_linkmode(phydev);
	}

out_unlock:
	mutex_unlock(&irqpoll->lock);

	return err;
}

static int mtip_suspend(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	int err;

	err = phy_power_off(priv->serdes);
	if (err) {
		phydev_err(phydev, "Failed to power off SerDes: %pe\n",
			   ERR_PTR(err));
		return err;
	}

	return 0;
}

static int mtip_resume(struct phy_device *phydev)
{
	struct mtip_backplane *priv = phydev->priv;
	int err;

	err = phy_power_on(priv->serdes);
	if (err) {
		phydev_err(phydev, "Failed to power on SerDes: %pe\n",
			   ERR_PTR(err));
		return err;
	}

	return 0;
}

static int mtip_config_init(struct phy_device *phydev)
{
	int err;

	err = mtip_reset_an(phydev);
	if (err < 0)
		return err;

	err = mtip_reset_pcs(phydev);
	if (err < 0)
		return err;

	return 0;
}

static struct phy_driver mtip_backplane_driver[] = {
	{
		.match_phy_device	= mtip_lx2160a_match_phy_device,
		.flags			= PHY_IS_INTERNAL,
		.name			= "MTIP AN/LT",
		.probe			= mtip_probe,
		.remove			= mtip_remove,
		.get_features		= mtip_lx2160a_get_features,
		.suspend		= mtip_suspend,
		.resume			= mtip_resume,
		.config_aneg		= mtip_config_aneg,
		.read_status		= mtip_read_status,
		.config_init		= mtip_config_init,
	},
};

static const struct of_device_id mtip_backplane_of_match[] = {
	{ .compatible = "fsl,lx2160a-backplane-anlt" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mtip_backplane_of_match);

module_phy_driver(mtip_backplane_driver);

MODULE_AUTHOR("Vladimir Oltean <vladimir.oltean@nxp.com>");
MODULE_DESCRIPTION("MTIP Backplane PHY driver");
MODULE_LICENSE("GPL");
