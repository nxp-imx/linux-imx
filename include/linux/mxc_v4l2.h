/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @file arch-mxc/mxc_v4l2.h
 *
 * @brief mxc V4L2 private structures
 *
 * @ingroup MXC_V4L2_CAPTURE
 */

#ifndef __ASM_ARCH_MXC_V4L2_H__
#define __ASM_ARCH_MXC_V4L2_H__

#define V4L2_CID_MXC_ROT		(V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_MXC_FLASH		(V4L2_CID_PRIVATE_BASE + 1)

#define V4L2_MXC_ROTATE_NONE			0
#define V4L2_MXC_ROTATE_VERT_FLIP		1
#define V4L2_MXC_ROTATE_HORIZ_FLIP		2
#define V4L2_MXC_ROTATE_180			3
#define V4L2_MXC_ROTATE_90_RIGHT		4
#define V4L2_MXC_ROTATE_90_RIGHT_VFLIP		5
#define V4L2_MXC_ROTATE_90_RIGHT_HFLIP		6
#define V4L2_MXC_ROTATE_90_LEFT			7

struct v4l2_mxc_offset {
	uint32_t u_offset;
	uint32_t v_offset;
};

#endif
