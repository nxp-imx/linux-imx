/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ASM_PLAT_PLATFORM_H
#define __ASM_PLAT_PLATFORM_H

#include <asm/sizes.h>

#define STMP378X_OCRAM_BASE	0x00000000
#define STMP378X_OCRAM_SIZE	(32 * SZ_1K)

#define STMP378X_REGS_BASE	0x80000000
#define STMP378X_REGS_SIZE	SZ_1M

/* Virtual address where registers are mapped */
#define STMP3XXX_REGS_VA_BASE	0xf0000000

/* Virtual address where OCRAM is mapped */
#define STMP3XXX_OCRAM_VA_BASE	0xf1000000

#endif /* __ASM_ARCH_PLATFORM_H */
