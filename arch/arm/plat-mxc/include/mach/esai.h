/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#ifndef __MACH_ESAI_H
#define __MACH_ESAI_H

struct imx_esai_platform_data {
	unsigned int flags;
#define IMX_ESAI_NET            (1 << 0)
#define IMX_ESAI_SYN            (1 << 1)
};

#endif
