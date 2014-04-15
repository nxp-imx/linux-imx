/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Freescale IMX Linux-specific MCC implementation.
 * The main MCC configuration file
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MCC_CONFIG__
#define __MCC_CONFIG__

/*
 * OS Selection
 */
#define MCC_LINUX		(1) /* Linux OS used */
#define MCC_MQX			(2) /* MQX RTOS used */
#define MCC_NODE_LINUX_NUMBER	(0) /* Linux OS used */
#define MCC_NODE_MQX_NUMBER	(0) /* MQX RTOS used */

/* used OS */
#define MCC_OS_USED                    (MCC_LINUX)

#endif /* __MCC_CONFIG__ */
