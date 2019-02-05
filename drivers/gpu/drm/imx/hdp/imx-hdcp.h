/*
 * Copyright 2017-2019 NXP
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
#ifndef _IMX_HDCP_H_
#define _IMX_HDCP_H_

bool is_hdcp_supported(struct imx_hdp *hdp);
int imx_hdcp_init(struct imx_hdp *hdp, struct device_node *of_node);
int imx_hdcp_enable(struct imx_hdp *hdp);
int imx_hdcp_disable(struct imx_hdp *hdp);
void imx_hdcp_atomic_check(struct drm_connector *connector,
			   struct drm_connector_state *old_state,
			   struct drm_connector_state *new_state);
int imx_hdcp_check_link(struct imx_hdp *hdp);

#endif
