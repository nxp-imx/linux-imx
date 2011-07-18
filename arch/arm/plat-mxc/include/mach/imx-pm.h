/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#ifndef __ASM_ARCH_IMX_PM_H
#define __ASM_ARCH_IMX_PM_H

/**
 * struct pm_platform_data - optional platform data for pm on i.MX
 *
 */

struct pm_platform_data {
	char *name;
	void (*suspend_enter) (void);
	void (*suspend_exit) (void);
};
#endif /* __ASM_ARCH_IMX_PM_H */
