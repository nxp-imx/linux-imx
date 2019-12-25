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

#include "API_AFE_ss28fdsoi_hdmirx.h"
#include "mxc-hdmi-rx.h"

#include <asm/unaligned.h>
#include <linux/ktime.h>

void hdmirx_hdcp_enable(state_struct *state)
{
	int version =
		/*CDN_API_HDCP_REC_VERSION_BOTH;*/
		/*CDN_API_HDCP_REC_VERSION_1;*/
		CDN_API_HDCPRX_VERSION_2;

	CDN_API_HDCPRX_Config config = { 0 };

	pr_info("%s()\n", __func__);

	config.activate = 1;
	config.version = version; /* Support 1.x */
	config.repeater = 0;
	config.use_secondary_link = 0;
	config.use_km_key = 1;
	config.bcaps = 0x80;
	config.bstatus = 0x1000; /* HDMI mode */

	/* This is currently handled by the SECO
	 * CDN_API_HDCPRX_SetConfiguration_blocking(state, 4, &config,
	 *					    CDN_BUS_TYPE_APB);
	 */

}

void hdmirx_hdcp_disable(state_struct *state)
{
	pr_info("%s()\n", __func__);

	/* This is currently handled by the SECO
	 * hdmirx_hdcp_set(state, 0, CDN_API_HDCPRX_VERSION_2);
	 */
}

int hdmirx_hdcp_get_status(state_struct *state,
			   CDN_API_HDCPRX_Status *hdcp_status)
{
	/* todo: implement HDCP status checking if needed */

	/*
	 * pr_info("%s()\n", __func__);
	 * CDN_API_HDCPRX_GetStatus(state,
	 * memset(hdcp_status, 0, sizeof(CDN_API_HDCPRX_Status));
	 *
	 * CDN_API_HDCP_REC_GetStatus(state,
	 *                            hdcp_status,
	 *                            CDN_BUS_TYPE_APB);
	 *
	 * pr_info("HCDP key_arrived 0x%02x\n", hdcp_status->key_arrived);
	 * pr_info("HCDP hdcp_ver    0x%02x\n", hdcp_status->hdcp_ver);
	 * pr_info("HCDP error       0x%02x\n", hdcp_status->error);
	 * pr_info("HCDP aksv[] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
	 *         hdcp_status->aksv[0],
	 *         hdcp_status->aksv[1],
	 *         hdcp_status->aksv[2],
	 *         hdcp_status->aksv[3],
	 *         hdcp_status->aksv[4]);
	 * pr_info("HCDP ainfo      0x%02x\n", hdcp_status->ainfo);
	 */
	return 0;
}
