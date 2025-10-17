/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2025 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __LINUX_IMX_VPU_MEMORY_USAGE_H
#define __LINUX_IMX_VPU_MEMORY_USAGE_H

#include <media/v4l2-ctrls.h>

struct imx_mur_node;

struct imx_mur_node *imx_mur_create_node(struct imx_mur_node *parent, const char *name);
void imx_mur_destroy_node(struct imx_mur_node *node);
void imx_mur_long_add(struct imx_mur_node *node, long val);
void imx_mur_long_sub(struct imx_mur_node *node, long val);
void imx_mur_long_new_and_add(struct imx_mur_node *node, long val, const char *label);
void imx_mur_long_sub_and_del(struct imx_mur_node *node, long val);
void imx_mur_long_set(struct imx_mur_node *node, long val);
long imx_mur_long_read(struct imx_mur_node *node);
struct v4l2_ctrl *imx_mur_new_v4l2_ctrl(struct v4l2_ctrl_handler *hdl, struct imx_mur_node *node);
void imx_mur_release_v4l2_ctrl(struct imx_mur_node *node);

#endif
