/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 NXP
 */

#ifndef __PHY_XGKR_H_
#define __PHY_XGKR_H_

struct phy;

enum coef_status {
	COEF_STAT_NOT_UPDATED = 0,
	COEF_STAT_UPDATED = 1,
	COEF_STAT_MIN = 2,
	COEF_STAT_MAX = 3,
};

enum coef_update {
	COEF_UPD_HOLD = 0,
	COEF_UPD_INC = 1,
	COEF_UPD_DEC = 2,
};

struct c72_coef_update {
	enum coef_update com1;
	enum coef_update coz;
	enum coef_update cop1;
	bool preset;
	bool init;
};

struct c72_coef_status {
	enum coef_status com1;
	enum coef_status coz;
	enum coef_status cop1;
};

enum xgkr_phy_configure_type {
	XGKR_CONFIGURE_LOCAL_TX,
	XGKR_CONFIGURE_REMOTE_TX,
	XGKR_CONFIGURE_LT_DONE,
};

/* Adjust PHY TX equalization in response to a C72 coefficient
 * update request from the link partner
 */
struct xgkr_phy_configure_local_tx {
	/* input to PHY */
	struct c72_coef_update update;
	/* output from PHY */
	struct c72_coef_status status;
};

/* Query the PHY RX quality in order to compute a C72 coefficient update
 * request to the link partner to improve that. Optional callback to see
 * how the link partner reacted to the update request (which is echoed back
 * unmodified). The coefficient status is only valid if there was no error
 * during its propagation.
 */
struct xgkr_phy_configure_remote_tx {
	/* output from PHY */
	bool rx_ready;
	struct c72_coef_update update;
	/* input to PHY */
	void (*cb)(void *cb_priv, int err, struct c72_coef_update update,
		   struct c72_coef_status status);
	void *cb_priv;
};

/**
 * struct phy_configure_opts_xgkr - 10GBase-KR configuration
 *
 * This structure is used to represent the configuration state of a
 * 10GBase-KR Ethernet Copper Backplane PHY.
 */
struct phy_configure_opts_xgkr {
	enum xgkr_phy_configure_type type;
	union {
		struct xgkr_phy_configure_local_tx local_tx;
		struct xgkr_phy_configure_remote_tx remote_tx;
	};
};

/* Some helpers */
static inline enum coef_update coef_update_opposite(enum coef_update update)
{
	switch (update) {
	case COEF_UPD_INC:
		return COEF_UPD_DEC;
	case COEF_UPD_DEC:
		return COEF_UPD_INC;
	default:
		return COEF_UPD_HOLD;
	}
}

static inline void coef_update_clamp(enum coef_update *update,
				     enum coef_status status)
{
	if (*update == COEF_UPD_INC && status == COEF_STAT_MAX)
		*update = COEF_UPD_HOLD;
	if (*update == COEF_UPD_DEC && status == COEF_STAT_MIN)
		*update = COEF_UPD_HOLD;
}

static inline bool coef_update_is_all_hold(const struct c72_coef_update *update)
{
	return update->coz == COEF_UPD_HOLD &&
	       update->com1 == COEF_UPD_HOLD &&
	       update->cop1 == COEF_UPD_HOLD;
}

#define C72_COEF_UPDATE_BUFSIZ 64
#define C72_COEF_STATUS_BUFSIZ 64

static inline const char *coef_update_to_string(enum coef_update coef)
{
	switch (coef) {
	case COEF_UPD_HOLD:
		return "HOLD";
	case COEF_UPD_INC:
		return "INC";
	case COEF_UPD_DEC:
		return "DEC";
	default:
		return "unknown";
	}
}

static inline const char *coef_status_to_string(enum coef_status coef)
{
	switch (coef) {
	case COEF_STAT_NOT_UPDATED:
		return "NOT_UPDATED";
	case COEF_STAT_UPDATED:
		return "UPDATED";
	case COEF_STAT_MIN:
		return "MIN";
	case COEF_STAT_MAX:
		return "MAX";
	default:
		return "unknown";
	}
}

static void inline c72_coef_update_print(const struct c72_coef_update *update,
					 char buf[C72_COEF_UPDATE_BUFSIZ])
{
	sprintf(buf, "INIT %d, PRESET %d, C(-1) %s, C(0) %s, C(+1) %s",
		update->init, update->preset,
		coef_update_to_string(update->com1),
		coef_update_to_string(update->coz),
		coef_update_to_string(update->cop1));
}

static inline void c72_coef_status_print(const struct c72_coef_status *status,
					 char buf[C72_COEF_STATUS_BUFSIZ])
{
	sprintf(buf, "C(-1) %s, C(0) %s, C(+1) %s",
		coef_status_to_string(status->com1),
		coef_status_to_string(status->coz),
		coef_status_to_string(status->cop1));
}

#endif /* __PHY_XGKR_H_ */
