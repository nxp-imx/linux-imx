/*
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

#ifndef RADIO_SI4763_H
#define RADIO_SI4763_H

#include <linux/i2c.h>

#define SI4763_NAME "radio-si4763"

/*
 * Platform dependent definition
 */
struct radio_si4763_platform_data {
	int i2c_bus;
	struct i2c_board_info *subdev_board_info;
};

#endif /* ifndef RADIO_SI4763_H*/
