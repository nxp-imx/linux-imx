/*
 * Freescale STMP37XX/STMP378X OCRAM allocator interface
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
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
#ifndef __ASM_PLAT_OCRAM_MALLOC_H
#define __ASM_PLAT_OCRAM_MALLOC_H

extern int ocram_malloc_init(void);

extern void *ocram_malloc(size_t size, dma_addr_t *phys);
extern void ocram_free(void *tofree);

#endif /* __ASM_PLAT_OCRAM_MALLOC_H */
