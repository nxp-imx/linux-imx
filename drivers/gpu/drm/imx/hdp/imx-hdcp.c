/*
 * Copyright 2019 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_hdcp.h>
#include <linux/i2c.h>
#include <linux/random.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/of.h>
#include "imx-hdp.h"
#include "imx-hdcp-private.h"

static uint8_t recv_id[] = {
	0x8B, 0xA4, 0x47, 0x42, 0xFB
};

static uint8_t m[] = {
	0xF9, 0xF1, 0x30, 0xA8,
	0x2D, 0x5B, 0xE5, 0xC3,
	0xE1, 0x7A, 0xB0, 0xFD,
	0x0F, 0x54, 0x40, 0x52
};

static uint8_t km[] = {
	0xCA, 0x9F, 0x83, 0x95,
	0x70, 0xD0, 0xD0, 0xF9,
	0xCF, 0xE4, 0xEB, 0x54,
	0x7E, 0x09, 0xFA, 0x3B
};

static uint8_t ekhkm[] = {
	0xE6, 0x57, 0x8E, 0xBC,
	0xC7, 0x68, 0x44, 0x87,
	0x88, 0x8A, 0x9B, 0xD7,
	0xD6, 0xAE, 0x38, 0xBE
};

static char const *g_last_error[16] = {
	"No Error",
	"HPD is down",
	"SRM failure",
	"Signature verification error",
	"h tag != h",
	"V tag diff v",
	"Locality check",
	"DDC error",
	"REAUTH_REQ",
	"Topology error",
	"Verify receiver ID list failed",
	"HDCP_RSVD1 was not 0,0,0",
	"HDMI capability or mode",
	"RI result was different than expected",
	"WatchDog expired",
	"Repeater integrity failed"
};

static char const *g_rx_type[4] = {
	"Unknown", "HDCP 1", "HDCP 2", "Unknown" };

#define HDCP_MAX_RECEIVERS 32
#define HDCP_RECEIVER_ID_SIZE_BYTES 5
#define HPD_EVENT 1
#define HDCP_STATUS_SIZE         0x5
#define HDCP_PORT_STS_AUTH       0x1
#define HDCP_PORT_STS_REPEATER   0x2
#define HDCP_PORT_STS_REPEATER   0x2
#define HDCP_PORT_STS_TYPE_MASK  0xc
#define HDCP_PORT_STS_TYPE_SHIFT 0x2
#define HDCP_PORT_STS_AUTH_STREAM_ID_SHIFT 0x4
#define HDCP_PORT_STS_AUTH_STREAM_ID_MASK 0x10
#define HDCP_PORT_STS_LAST_ERR_SHIFT 0x5
#define HDCP_PORT_STS_LAST_ERR_MASK  (0x0F << 5)
#define GET_HDCP_PORT_STS_LAST_ERR(__sts__) \
	(((__sts__) & HDCP_PORT_STS_LAST_ERR_MASK) >> \
	 HDCP_PORT_STS_LAST_ERR_SHIFT)
#define HDCP_PORT_STS_1_1_FEATURES   0x200

#define HDCP_DPCD_CERTRX_ADDRESS 0x6900B

#define HDCP_CONFIG_NONE    ((u8) 0)
#define HDCP_CONFIG_1_4     ((u8) 1) /* use HDCP 1.4 only */
#define HDCP_CONFIG_2_2     ((u8) 2) /* use HDCP 2.2 only */
/* use All HDCP versions */
#define HDCP_CONFIG_ALL     (HDCP_CONFIG_1_4 | HDCP_CONFIG_2_2)

static void imx_hdcp_print_port_status(uint16_t sts)
{
	DRM_DEBUG_KMS("INFO: HDCP Port Status: 0x%04x\n", sts);
	DRM_DEBUG_KMS(" Authenticated: %d\n",
		      (bool)(sts & HDCP_PORT_STS_AUTH));
	DRM_DEBUG_KMS(" Receiver is repeater: %d\n",
		      (bool)(sts & HDCP_PORT_STS_REPEATER));
	DRM_DEBUG_KMS(" RX Type: %s\n",
		      g_rx_type[((sts & HDCP_PORT_STS_TYPE_MASK)
				 >> HDCP_PORT_STS_TYPE_SHIFT)]);
	DRM_DEBUG_KMS(" AuthStreamId: %d\n",
		      (bool)(sts & HDCP_PORT_STS_AUTH_STREAM_ID_MASK));
	DRM_DEBUG_KMS(" Last Error: %s\n",
		      g_last_error[((sts & HDCP_PORT_STS_LAST_ERR_MASK)
				    >> HDCP_PORT_STS_LAST_ERR_SHIFT)]);
	DRM_DEBUG_KMS(" Enable 1.1 Features: %d\n",
		      (bool)(sts & HDCP_PORT_STS_1_1_FEATURES));
}

static unsigned int wait4event(struct imx_hdp *hdp,
			       unsigned int *events,
			       unsigned int event_to_wait)
{
	unsigned int reg_events;
	unsigned int returned_events;
	unsigned int event_mask = 1U << event_to_wait |
		1U << EVENT_ID_HDCPTX_STATUS;
	int timeout = 50000; /* 50000 * 10 u is at least ~500 miliseconds */

	while ((event_mask & *events) == 0) {
		if (timeout-- == 0)
			return 0;
		udelay(10);
		CDN_API_Get_Event(&hdp->state, &reg_events);
		*events |= reg_events;
	}

	returned_events = *events & event_mask;
	*events &= ~event_mask;
	return returned_events;
}

static inline
uint16_t imx_hdcp_get_port_status(u8 hdcp_status[HDCP_STATUS_SIZE])
{
	return (hdcp_status[0] << 8) | hdcp_status[1];
}

static u16 imx_hdcp_get_status(struct imx_hdp *hdp)
{
	CDN_BUS_TYPE bt = hdp->hdcp.bus_type;
	u8 hdcp_status[HDCP_STATUS_SIZE];
	u16 hdcp_port_status;

	DRM_DEBUG_KMS("INFO: Reading HDCP TX status\n");
	CDN_API_HDCP_TX_STATUS_REQ_blocking(&hdp->state, hdcp_status, bt);

	hdcp_port_status = imx_hdcp_get_port_status(hdcp_status);

	return hdcp_port_status;
}

static inline unsigned char check_event(unsigned int events,
					unsigned int tested)
{
	if ((events & (1 << tested)) == 0)
		return 0;
	return 1;
}

/* Prints status. Returns error code (0 = no error) */
static unsigned char imx_hdcp_handle_status(unsigned short status)
{
	imx_hdcp_print_port_status(status);
	if (status & HDCP_PORT_STS_LAST_ERR_MASK)
		DRM_ERROR("ERROR: HDCP error was set to %s\n",
			  g_last_error[((status & HDCP_PORT_STS_LAST_ERR_MASK)
					>> HDCP_PORT_STS_LAST_ERR_SHIFT)]);
	return GET_HDCP_PORT_STS_LAST_ERR(status);
}

static int imx_hdcp_set_config(struct imx_hdp *hdp, u8 hdcp_config)
{
	CDN_API_STATUS ret;
	CDN_BUS_TYPE bt = hdp->hdcp.bus_type;
	uint8_t apb_cfg_dpcd_sts = 0;
	uint8_t apb_cfg_hdcp_sts = 0;
	uint8_t apb_cfg_capb_sts = 0;
	uint8_t hdcp_cfg;

	unsigned int events = 0;
	unsigned int retEvents;
	unsigned short hdcp_port_status;

	if (bt == CDN_BUS_TYPE_SAPB) {
		DRM_DEBUG_KMS("INFO: Switching HDCP Commands to SAPB.\n");
		ret = CDN_API_ApbConf_blocking
			(&hdp->state,
			 0, 0, /* set DPCD to APB (don't lock) */
			 1, 0, /* set HDCP to SAPB (don't lock) */
			 0, 0, /* set CAPB owner CPU (don't lock) */
			 &apb_cfg_dpcd_sts,
			 &apb_cfg_hdcp_sts,
			 &apb_cfg_capb_sts);

		if (ret != CDN_OK) {
			DRM_ERROR("Failed to set APB configuration.\n");
			return -1;
		}
		if (apb_cfg_hdcp_sts == 1) { /* 1 - locked */
			DRM_ERROR("Failed to switch HDCP to SAPB Mailbox\n");
			return -1;
		}
		DRM_DEBUG_KMS("INFO: HDCP switched to SAPB\n");
	} else
		DRM_DEBUG_KMS("INFO: Leaving HDCP to APB\n");

	/* HDCP 2.2(and/or 1.4) | activate | km-key |  0 */
	hdcp_cfg = hdcp_config | 0x04 | 0x10 | (HDCP_CONTENT_TYPE_0 << 3);

	DRM_DEBUG_KMS("INFO: Enabling HDCP...\n");
	CDN_API_HDCP_TX_CONFIGURATION_blocking(&hdp->state, hdcp_cfg, bt);

	/* Wait until HDCP_TX_STATUS EVENT appears */
	DRM_DEBUG_KMS("INFO: wait4event -> EVENT_ID_HDCPTX_STATUS\n");
	retEvents = wait4event(hdp, &events, EVENT_ID_HDCPTX_STATUS);

	/* Set TX STATUS REQUEST */
	DRM_DEBUG_KMS("INFO: Getting port status\n");
	hdcp_port_status = imx_hdcp_get_status(hdp);
	if (imx_hdcp_handle_status(hdcp_port_status) != 0)
		return -1;

	return 0;
}

static int imx_hdcp_auth_check(struct imx_hdp *hdp)
{
	u32 events;
	u16 hdcp_port_status;

	/* Wait until HDCP_TX_STATUS EVENT appears */
	events = wait4event(hdp, &events, EVENT_ID_HDCPTX_STATUS);
	if (events == 0)
		return -1;
	DRM_DEBUG_KMS("HDCP: EVENT_ID_HDCPTX_STATUS\n");

	hdcp_port_status = imx_hdcp_get_status(hdp);
	if (imx_hdcp_handle_status(hdcp_port_status) != 0)
		return -1;

	if (hdcp_port_status & 1) {
		DRM_INFO("Authentication completed successfully!\n");
		return 0;
	}
	DRM_ERROR("Authentication failed\n");
	return -1;
}

static int imx_hdcp_check_receviers(struct imx_hdp *hdp)
{
	CDN_BUS_TYPE bt = hdp->hdcp.bus_type;
	unsigned int events = 0;
	unsigned int ret_events;
	uint8_t hdcp_num_rec;
	uint8_t hdcp_rec_id[HDCP_MAX_RECEIVERS][HDCP_RECEIVER_ID_SIZE_BYTES];
	uint8_t i;

	DRM_DEBUG_KMS("INFO: Waiting for Receiver ID valid event\n");
	ret_events = wait4event(hdp, &events,
				EVENT_ID_HDCPTX_IS_RECEIVER_ID_VALID);
	DRM_DEBUG_KMS("INFO: Requesting Receivers ID's\n");

	hdcp_num_rec = 0;
	memset(&hdcp_rec_id, 0, sizeof(hdcp_rec_id));

	CDN_API_HDCP_TX_IS_RECEIVER_ID_VALID_REQ_blocking
		(&hdp->state,
		 &hdcp_num_rec,
		 (unsigned char *)
		 hdcp_rec_id,
		 bt
		);

	DRM_DEBUG_KMS("INFO: Number of Receivers: %d\n", hdcp_num_rec);

	for (i = 0; i < hdcp_num_rec; ++i) {
		DRM_INFO("\tReveiver ID%2d: %.2X%.2X%.2X%.2X%.2X\n",
			 i,
			 hdcp_rec_id[i][0],
			 hdcp_rec_id[i][1],
			 hdcp_rec_id[i][2],
			 hdcp_rec_id[i][3],
			 hdcp_rec_id[i][4]
			);
	}

	/* fixme: Check Receiver ID's against revocation list */

	DRM_DEBUG_KMS("INFO: Responding with Receiver ID's OK!\n");
	CDN_API_HDCP_TX_RESPOND_RECEIVER_ID_VALID_blocking(&hdp->state, 1,
							   hdp->hdcp.bus_type);

	return 0;
}

static inline int imx_hdcp_auth_22(struct imx_hdp *hdp)
{
	CDN_BUS_TYPE bt = hdp->hdcp.bus_type;
	unsigned char kmStored = 0;
	unsigned int events = 0;
	unsigned int retEvents;
	unsigned short hdcp_port_status;
	u8 resp[HDCP_STATUS_SIZE];
	S_HDCP_TRANS_PAIRING_DATA pairing;

	DRM_DEBUG_KMS("HDCP: Start 2.2 Authentication\n");

	/* Wait until HDCP2_TX_IS_KM_STORED EVENT appears */
	retEvents = 0;
	DRM_DEBUG_KMS("INFO: Wait until HDCP2_TX_IS_KM_STORED EVENT appears\n");
	while (check_event(retEvents, EVENT_ID_HDCPTX_IS_KM_STORED) == 0) {
		DRM_DEBUG_KMS("INFO: Waiting FOR _IS_KM_STORED EVENT\n");
		retEvents = wait4event(hdp, &events,
				       EVENT_ID_HDCPTX_IS_KM_STORED);
		if (retEvents == 0) {
			/* time out occurred, stop hdcp and return error */
			CDN_API_HDCP_TX_CONFIGURATION_blocking(&hdp->state,
							       0, bt);
			return -1;
		}
		if (check_event(retEvents, EVENT_ID_HDCPTX_STATUS) != 0) {
			hdcp_port_status = imx_hdcp_get_status(hdp);
			if (imx_hdcp_handle_status(hdcp_port_status) != 0)
				return -1;
		}
	}

	DRM_DEBUG_KMS("HDCP: EVENT_ID_HDCPTX_IS_KM_STORED\n");

	/* Set HDCP2 TX KM STORED REQUEST */
	CDN_API_HDCP2_TX_IS_KM_STORED_REQ_blocking(&hdp->state, resp, bt);
	DRM_DEBUG_KMS("HDCP: CDN_API_HDCP2_TX_IS_KM_STORED_REQ_blocking\n");
	DRM_DEBUG_KMS("HDCP: Receiver ID: 0x%x%x%x%x%x\n",
		      resp[0], resp[1], resp[2], resp[3], resp[4]);

	/* Check if KM is stored */
	if (kmStored != 0) {
		DRM_DEBUG_KMS("INFO: KM is stored\n");
		/* Set HDCP2 TX RESPOND KM with stored KM */
		memcpy(pairing.receiver_id, recv_id, sizeof(recv_id));
		memcpy(pairing.m, m, sizeof(m));
		memcpy(pairing.km, km, sizeof(km));
		memcpy(pairing.ekh, ekhkm, sizeof(ekhkm));
		CDN_API_HDCP2_TX_RESPOND_KM_blocking(&hdp->state, &pairing, bt);
		DRM_DEBUG_KMS("HDCP: CDN_API_HDCP2_TX_RESPOND_KM_blocking\n");
	} else { /* KM is not stored */
		DRM_DEBUG_KMS("INFO: KM is not stored\n");
		/* Set HDCP2 TX RESPOND KM with empty data */
		CDN_API_HDCP2_TX_RESPOND_KM_blocking(&hdp->state, NULL, bt);
	}

	imx_hdcp_check_receviers(hdp);

	/* Check if KM is not stored */
	if (kmStored == 0) {
		/* Wait until HDCP2_TX_STORE_KM EVENT appears */
		retEvents = 0;
		while (check_event(retEvents, EVENT_ID_HDCPTX_STORE_KM) == 0) {
			retEvents = wait4event(hdp, &events,
					       EVENT_ID_HDCPTX_STORE_KM);
			if (check_event(retEvents, EVENT_ID_HDCPTX_STATUS)
			    != 0) {
				hdcp_port_status = imx_hdcp_get_status(hdp);
				if (imx_hdcp_handle_status(hdcp_port_status)
				    != 0)
					return -1;
			}
		}
		DRM_DEBUG_KMS("HDCP: EVENT_ID_HDCPTX_STORE_KM\n");

		/* Set HDCP2_TX_STORE_KM REQUEST */
		CDN_API_HDCP2_TX_STORE_KM_REQ_blocking(&hdp->state, &pairing,
						       bt);
		DRM_DEBUG_KMS("HDCP: CDN_API_HDCP2_TX_STORE_KM_REQ_blocking");
	}

	return 0;
}

static inline int imx_hdcp_auth_14(struct imx_hdp *hdp)
{
	DRM_DEBUG_KMS("HDCP: Starting 1.4 Authentication\n");
	return imx_hdcp_check_receviers(hdp);
}

static int imx_hdcp_auth(struct imx_hdp *hdp, u8 hdcp_config)
{
	int ret;

	DRM_DEBUG_KMS("HDCP: Start Authentication\n");
	ret = imx_hdcp_set_config(hdp, hdcp_config);
	if (ret) {
		DRM_ERROR("imx_hdcp_set_config failed\n");
		return -1;
	}
	if (hdcp_config == HDCP_TX_1)
		ret = imx_hdcp_auth_14(hdp);
	else
		ret = imx_hdcp_auth_22(hdp);

	if (ret) {
		DRM_ERROR("imx_hdcp_auth_%s failed\n",
			  (hdcp_config == HDCP_TX_1) ? "14" : "22");
		return -1;
	}

	ret = imx_hdcp_auth_check(hdp);
	if (ret)
		ret = imx_hdcp_auth_check(hdp);
	return ret;
}

static int _imx_hdcp_disable(struct imx_hdp *hdp)
{
	int ret = 0;
	uint8_t hdcp_cfg = 0;
	CDN_BUS_TYPE bt = hdp->hdcp.bus_type;

	DRM_DEBUG_KMS("[%s:%d] HDCP is being disabled...\n",
		      hdp->connector.name, hdp->connector.base.id);
	DRM_DEBUG_KMS("INFO: Disabling HDCP...\n");

	ret = CDN_API_HDCP_TX_CONFIGURATION_blocking(&hdp->state, hdcp_cfg, bt);

	DRM_DEBUG_KMS("HDCP is disabled\n");

	return ret;
}


static int _imx_hdcp_enable(struct imx_hdp *hdp)
{
	int i, ret = 0, tries = 3;

	DRM_DEBUG_KMS("[%s:%d] HDCP is being enabled...\n",
		      hdp->connector.name, hdp->connector.base.id);

	/* Incase of authentication failures, HDCP spec expects reauth. */
	for (i = 0; i < tries; i++) {
		if (hdp->hdcp.config & HDCP_CONFIG_2_2) {
			ret = imx_hdcp_auth(hdp, HDCP_TX_2);
			if (ret == 0)
				return 0;
		}
		_imx_hdcp_disable(hdp);
		if (hdp->hdcp.config & HDCP_CONFIG_1_4) {
			ret = imx_hdcp_auth(hdp, HDCP_TX_1);
			if (!ret)
				return 0;
		}
		DRM_DEBUG_KMS("HDCP Auth failure (%d)\n", ret);

		/* Ensuring HDCP encryption and signalling are stopped. */
		_imx_hdcp_disable(hdp);
	}

	DRM_ERROR("HDCP authentication failed (%d tries/%d)\n", tries, ret);
	return ret;
}

static void imx_hdcp_check_work(struct work_struct *work)
{
	struct imx_hdcp *hdcp = container_of(work,
					     struct imx_hdcp,
					     check_work.work);
	struct imx_hdp *hdp = container_of(hdcp,
					   struct imx_hdp,
					   hdcp);
	if (!imx_hdcp_check_link(hdp))
		schedule_delayed_work(&hdcp->check_work,
				      DRM_HDCP_CHECK_PERIOD_MS);
}


static void imx_hdcp_prop_work(struct work_struct *work)
{
	struct imx_hdcp *hdcp = container_of(work,
					     struct imx_hdcp,
					     prop_work);
	struct imx_hdp *hdp = container_of(hdcp,
					   struct imx_hdp,
					   hdcp);

	struct drm_device *dev = hdp->drm_dev;
	struct drm_connector_state *state;

	drm_modeset_lock(&dev->mode_config.connection_mutex, NULL);
	mutex_lock(&hdp->hdcp.mutex);

	/*
	 * This worker is only used to flip between ENABLED/DESIRED. Either of
	 * those to UNDESIRED is handled by core. If hdcp_value == UNDESIRED,
	 * we're running just after hdcp has been disabled, so just exit
	 */
	if (hdp->hdcp.value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		state = hdp->connector.state;
		state->content_protection = hdp->hdcp.value;
	}

	mutex_unlock(&hdp->hdcp.mutex);
	drm_modeset_unlock(&dev->mode_config.connection_mutex);
}

static void show_hdcp_supported(struct imx_hdp *hdp)
{
	if ((hdp->hdcp.config & (HDCP_CONFIG_1_4 | HDCP_CONFIG_2_2)) ==
		    (HDCP_CONFIG_1_4 | HDCP_CONFIG_2_2))
		DRM_INFO("Both HDCP 1.4 and 2 2 are enabled\n");
	else if (hdp->hdcp.config & HDCP_CONFIG_1_4)
		DRM_INFO("Only HDCP 1.4 is enabled\n");
	else if (hdp->hdcp.config & HDCP_CONFIG_2_2)
		DRM_INFO("Only HDCP 2.2 is enabled\n");
	else
		DRM_INFO("HDCP is disabled\n");
}

int imx_hdcp_init(struct imx_hdp *hdp, struct device_node *of_node)
{
	int ret;
	const char *compat;
	u32 temp;

	ret = of_property_read_string(of_node, "compatible", &compat);
	if (ret) {
		DRM_ERROR("Failed to compatible dts string\n");
		return ret;
	}

	ret = of_property_read_u32(of_node, "hdcp-config", &temp);
	if (ret) {
		/* hdp->hdcp_config = HDCP_CONFIG_ALL; */
		/* using highest level by default */
		hdp->hdcp.config = HDCP_CONFIG_2_2;
		DRM_INFO("Failed to get HDCP config - using HDCP 2.2 only\n");
	} else {
		hdp->hdcp.config = temp;
		show_hdcp_supported(hdp);
	}

	if (!check_hdcp_enabled())
		return -EPERM;

	if (!strstr(compat, "hdmi"))
		return -EPERM;

	if (strstr(compat, "imx8mq"))
		hdp->hdcp.bus_type = CDN_BUS_TYPE_SAPB;
	else
		hdp->hdcp.bus_type = CDN_BUS_TYPE_APB;

	ret = drm_connector_attach_content_protection_property(&hdp->connector);
	if (ret)
		return ret;

	/*connector->hdcp_shim = hdcp_shim;*/
	mutex_init(&hdp->hdcp.mutex);
	INIT_DELAYED_WORK(&hdp->hdcp.check_work, imx_hdcp_check_work);
	INIT_WORK(&hdp->hdcp.prop_work, imx_hdcp_prop_work);
	return 0;
}

int imx_hdcp_enable(struct imx_hdp *hdp)
{
	int ret;

	mutex_lock(&hdp->hdcp.mutex);

	ret = _imx_hdcp_enable(hdp);
	if (ret)
		goto out;

	hdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_ENABLED;
	schedule_work(&hdp->hdcp.prop_work);
	schedule_delayed_work(&hdp->hdcp.check_work,
			      DRM_HDCP_CHECK_PERIOD_MS);
out:
	mutex_unlock(&hdp->hdcp.mutex);
	return ret;
}

int imx_hdcp_disable(struct imx_hdp *hdp)
{
	int ret = 0;

	mutex_lock(&hdp->hdcp.mutex);
	if (hdp->hdcp.value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		hdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_UNDESIRED;
		ret = _imx_hdcp_disable(hdp);
	}
	mutex_unlock(&hdp->hdcp.mutex);
	cancel_delayed_work_sync(&hdp->hdcp.check_work);

	return ret;
}

void imx_hdcp_atomic_check(struct drm_connector *connector,
			   struct drm_connector_state *old_state,
			   struct drm_connector_state *new_state)
{
	uint64_t old_cp = old_state->content_protection;
	uint64_t new_cp = new_state->content_protection;
	struct drm_crtc_state *crtc_state;

	if (!new_state->crtc) {
		/*
		 * If the connector is being disabled with CP enabled, mark it
		 * desired so it's re-enabled when the connector is brought back
		 */
		if (old_cp == DRM_MODE_CONTENT_PROTECTION_ENABLED)
			new_state->content_protection =
				DRM_MODE_CONTENT_PROTECTION_DESIRED;
		return;
	}

	/*
	 * Nothing to do if the state didn't change, or HDCP was activated since
	 * the last commit
	 */
	if (old_cp == new_cp ||
	    (old_cp == DRM_MODE_CONTENT_PROTECTION_DESIRED &&
	     new_cp == DRM_MODE_CONTENT_PROTECTION_ENABLED))
		return;

	crtc_state = drm_atomic_get_new_crtc_state(new_state->state,
						   new_state->crtc);
	crtc_state->mode_changed = true;
}

int imx_hdcp_check_link(struct imx_hdp *hdp)
{
	int ret = 0;
	u16 hdcp_port_status;
	u8 hdcp_last_error;

	mutex_lock(&hdp->hdcp.mutex);

	if (hdp->hdcp.value == DRM_MODE_CONTENT_PROTECTION_UNDESIRED)
		goto out;

	/* get port status */
	hdcp_port_status = imx_hdcp_get_status(hdp);
	hdcp_last_error = GET_HDCP_PORT_STS_LAST_ERR(hdcp_port_status);
	if (hdcp_last_error)
		DRM_ERROR("HDCP error no: %u\n", hdcp_last_error);

	if (hdcp_port_status &  HDCP_PORT_STS_AUTH) {
		if (hdp->hdcp.value !=
			    DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
			hdp->hdcp.value =
				DRM_MODE_CONTENT_PROTECTION_ENABLED;
			schedule_work(&hdp->hdcp.prop_work);
		}
		DRM_DEBUG_KMS("INFO: HDCP authenticated\n");
		/* HDCP is authenticated*/
		goto out;
	}

	DRM_DEBUG_KMS("[%s:%d] HDCP link failed, retrying authentication\n",
		      hdp->connector.name, hdp->connector.base.id);

	ret = _imx_hdcp_disable(hdp);
	if (ret) {
		DRM_ERROR("Failed to disable hdcp (%d)\n", ret);
		hdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_DESIRED;
		schedule_work(&hdp->hdcp.prop_work);
		goto out;
	}

	ret = _imx_hdcp_enable(hdp);
	if (ret) {
		DRM_ERROR("Failed to enable hdcp (%d)\n", ret);
		hdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_DESIRED;
		schedule_work(&hdp->hdcp.prop_work);
		goto out;
	}

out:
	mutex_unlock(&hdp->hdcp.mutex);
	return ret;
}

