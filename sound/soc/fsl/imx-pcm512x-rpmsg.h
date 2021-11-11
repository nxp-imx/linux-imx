/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2017-2021 NXP
 */

#ifndef __IMX_PCM512X_RPMSG_H
#define __IMX_PCM512X_RPMSG_H

int imx_pcm512x_rpmsg_init_data(struct platform_device *pdev, void ** _data);
int imx_pcm512x_rpmsg_probe(struct platform_device *pdev, void *_data);


#endif /* __IMX_PCM512X_RPMSG_H  */
