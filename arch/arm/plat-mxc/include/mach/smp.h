/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef FSL_ARCH_SMP_H
#define FSL_ARCH_SMP_H

#include <asm/hardware/gic.h>

/* Needed for secondary core boot */
extern void mx6_secondary_startup(void);

#endif
