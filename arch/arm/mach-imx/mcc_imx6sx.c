/*
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * SPDX-License-Identifier: GPL-2.0+ and/or BSD-3-Clause
 * The GPL-2.0+ license for this file can be found in the COPYING.GPL file
 * included with this distribution or at
 * http://www.gnu.org/licenses/gpl-2.0.html
 * The BSD-3-Clause License for this file can be found in the COPYING.BSD file
 * included with this distribution or at
 * http://opensource.org/licenses/BSD-3-Clause
 */

#include <linux/mcc_config_linux.h>
#include <linux/mcc_common.h>
#include <linux/mcc_linux.h>

/*!
 * \brief This function returns the core number
 *
 * \return int
 */
unsigned int _psp_core_num(void)
{
    return 0;
}

/*!
 * \brief This function returns the node number
 *
 * \return unsigned int
 */
unsigned int _psp_node_num(void)
{
    return MCC_LINUX_NODE_NUMBER;
}
