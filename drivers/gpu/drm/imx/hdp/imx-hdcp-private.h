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
#ifndef _IMX_HDCP_PRIVATE_H_
#define _IMX_HDCP_PRIVATE_H_

struct imx_hdcp {
	struct mutex mutex;
	u64 value; /* protected by hdcp_mutex */
	struct delayed_work check_work;
	struct work_struct prop_work;
	u8 bus_type;
	u8 config;
};
#endif
