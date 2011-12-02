/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __INCLUDE_MIPI_CSI2_H
#define __INCLUDE_MIPI_CSI2_H

/* mipi csi2 register */
#define	CSI2_VERSION					0x000
#define	CSI2_N_LANES					(0x004/4)
#define	CSI2_PHY_SHUTDOWNZ				(0x008/4)
#define	CSI2_DPHY_RSTZ					(0x00c/4)
#define	CSI2_RESETN					(0x010/4)
#define	CSI2_PHY_STATE					(0x014/4)
#define	CSI2_DATA_IDS_1					(0x018/4)
#define	CSI2_DATA_IDS_2					(0x01c/4)
#define	CSI2_ERR1					(0x020/4)
#define	CSI2_ERR2					(0x024/4)
#define	CSI2_MSK1					(0x028/4)
#define	CSI2_MSK2					(0x02c/4)
#define	CSI2_PHY_TST_CTRL0				(0x030/4)
#define	CSI2_PHY_TST_CTRL1				(0x034/4)
#define	CSI2_SFT_RESET					(0xf00/4)

/* mipi data type */
#define MIPI_DT_YUV420		0x18 /* YYY.../UYVY.... */
#define MIPI_DT_YUV420_LEGACY	0x1a /* UYY.../VYY...   */
#define MIPI_DT_YUV422		0x1e /* UYVY...		*/
#define MIPI_DT_RGB444		0x20
#define MIPI_DT_RGB555		0x21
#define MIPI_DT_RGB565		0x22
#define MIPI_DT_RGB666		0x23
#define MIPI_DT_RGB888		0x24
#define MIPI_DT_RAW6		0x28
#define MIPI_DT_RAW7		0x29
#define MIPI_DT_RAW8		0x2a
#define MIPI_DT_RAW10		0x2b
#define MIPI_DT_RAW12		0x2c
#define MIPI_DT_RAW14		0x2d


struct mipi_csi2_info;
/* mipi csi2 API */
struct mipi_csi2_info *mipi_csi2_get_info(void);

bool mipi_csi2_enable(struct mipi_csi2_info *info);

bool mipi_csi2_disable(struct mipi_csi2_info *info);

bool mipi_csi2_get_status(struct mipi_csi2_info *info);

int mipi_csi2_get_bind_ipu(struct mipi_csi2_info *info);

unsigned int mipi_csi2_get_bind_csi(struct mipi_csi2_info *info);

unsigned int mipi_csi2_get_virtual_channel(struct mipi_csi2_info *info);

unsigned int mipi_csi2_set_lanes(struct mipi_csi2_info *info);

unsigned int mipi_csi2_set_datatype(struct mipi_csi2_info *info,
					unsigned int datatype);

unsigned int mipi_csi2_get_datatype(struct mipi_csi2_info *info);

unsigned int mipi_csi2_dphy_status(struct mipi_csi2_info *info);

unsigned int mipi_csi2_get_error1(struct mipi_csi2_info *info);

unsigned int mipi_csi2_get_error2(struct mipi_csi2_info *info);

int mipi_csi2_pixelclk_enable(struct mipi_csi2_info *info);

void mipi_csi2_pixelclk_disable(struct mipi_csi2_info *info);

int mipi_csi2_reset(struct mipi_csi2_info *info);

/**
 * struct mipi_csi2_platform_data - MIPI CSI2 driver platform data.
 *
 * This structure communicates platform-specific information to the
 * MIPI CSI2 driver that can't be expressed as resources.
 *
 * @ipu_id:	the id of IPUs on which this MIPI CSI2 works
 *
 * @csi_id:	the id of IPU CSI interfaces
 *
 * @v_channel:	mipi virtual channel num
 *
 * @lanes:	mipi lanes
 *
 * @init:	A pointer to a function the driver will call to
 *              initialize the platform (e.g., set up the pin mux).
 * @exit:	A pointer to a function the driver will call to
 *                           exit the platform (e.g., free pins).
 */
struct mipi_csi2_platform_data {
	int	ipu_id;
	unsigned int	csi_id;
	unsigned int	v_channel;
	unsigned int	lanes;
	char	*dphy_clk;
	char	*pixel_clk;
	int	(*init)(struct platform_device *);
	void	(*exit)(struct platform_device *);
};

#endif
