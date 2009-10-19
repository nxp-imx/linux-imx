/*
 * Freescale STMP37XX/STMP378X dev board rotary encoder arch-dependent
 * structure and functions declarations
 *
 * Author: Drew Benedetti <drewb@embeddedalley.com>
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
#ifndef __ASM_PLAT_ROTDEC_H
#define __ASM_PLAT_ROTDEC_H

extern int rotdec_pinmux_request(void);
extern void rotdec_pinmux_free(void);

#endif /* __ASM_PLAT_ROTDEC_H */
