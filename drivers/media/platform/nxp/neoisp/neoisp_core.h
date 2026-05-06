/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NEOISP context definition
 *
 * Copyright 2026 NXP
 */

#ifndef __NXP_NEOISP_CORE_H
#define __NXP_NEOISP_CORE_H

/**
 * neoisp_core_media_register - Register neoisp subdevice into another media device
 * @dev: the neoisp device pointer
 * @sd: The V4L2 subdevice which shares the media device to neoisp core
 *
 * This function allows a V4L2 subdevice sharing the media device it belongs to
 * with neo isp core devices. Thus, the neoisp graph can be registered in that media
 * device, even thought it is independent without any media link connections to the
 * other devices.
 */
#if IS_ENABLED(CONFIG_VIDEO_NXP_NEOISP)
int neoisp_core_media_register(struct device *dev, struct v4l2_subdev *sd);
#else
static inline int neoisp_core_media_register(struct device *dev, struct v4l2_subdev *sd)
{
	return 0;
}
#endif

#endif /* __NXP_NEOISP_CORE_H */
