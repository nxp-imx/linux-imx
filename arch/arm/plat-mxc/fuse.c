/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <mach/hardware.h>

#define HW_OCOTP_CFGn(n)	(0x00000410 + (n) * 0x10)

/* Note: the names oder is the same as device enum order defined in mxc.h */
static char *names[] = {
	"pxp", "ovg", "dsi_csi2", "enet", "mlb",
	"epdc", "hdmi", "pcie", "sata", "dtcp",
	"2d", "3d", "vpu", "divx3", "rv",
	"sorensen",
};

int fuse_dev_is_available(enum mxc_dev_type dev)
{
	u32 uninitialized_var(reg);
	u32 uninitialized_var(mask);
	int ret;

	if (!cpu_is_mx6())
		return 1;

	/* mx6sl is still not supported */
	if (cpu_is_mx6sl())
		return 1;

	switch (dev) {
	case MXC_DEV_PXP:
		if (cpu_is_mx6q())
			return 0;

		if (cpu_is_mx6dl()) {
			reg = HW_OCOTP_CFGn(2);
			mask = 0x80000000;
		}
		break;
	case MXC_DEV_OVG:
		if (cpu_is_mx6dl())
			return 0;

		if (cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(2);
			mask = 0x40000000;
		}
		break;
	case MXC_DEV_DSI_CSI2:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(2);
			mask = 0x10000000;
		}
		break;
	case MXC_DEV_ENET:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(2);
			mask = 0x08000000;
		}
		break;
	case MXC_DEV_MLB:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(2);
			mask = 0x04000000;
		}
		break;
	case MXC_DEV_EPDC:
		if (cpu_is_mx6q())
			return 0;

		if (cpu_is_mx6dl()) {
			reg = HW_OCOTP_CFGn(2);
			mask = 0x02000000;
		}
		break;
	case MXC_DEV_HDMI:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000080;
		}
		break;
	case MXC_DEV_PCIE:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000040;
		}
		break;
	case MXC_DEV_SATA:
		if (cpu_is_mx6dl())
			return 0;

		if (cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000020;
		}
		break;
	case MXC_DEV_DTCP:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000010;
		}
		break;
	case MXC_DEV_2D:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000008;
		}
		break;
	case MXC_DEV_3D:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000004;
		}
		break;
	case MXC_DEV_VPU:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00008000;
		}
		break;
	case MXC_DEV_DIVX3:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000400;
		}
		break;
	case MXC_DEV_RV:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000200;
		}
		break;
	case MXC_DEV_SORENSEN:
		if (cpu_is_mx6dl() || cpu_is_mx6q()) {
			reg = HW_OCOTP_CFGn(3);
			mask = 0x00000100;
		}
		break;
	default:
		/* we treat the unkown device is avaiable by default */
		return 1;
	}

	ret = readl(MX6_IO_ADDRESS(OCOTP_BASE_ADDR) + reg) & mask;
	pr_debug("fuse_check: %s is %s\n", names[dev], ret ?
					"unavailable" : "available");

	return !ret;
}
