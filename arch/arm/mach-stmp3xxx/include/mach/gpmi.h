/*
 * Freescale STMP37XX/STMP378X GPMI (General-Purpose-Media-Interface)
 *
 * Arch-dependent structure and functions declarations
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
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
#ifndef __ASM_PLAT_GPMI_H
#define __ASM_PLAT_GPMI_H

#define GPMI_PART_CONCAT 		0x8000	/* indicates that partitions
						   should be concatenated    */
extern int gpmi_pinmux_request(char *);
extern void gpmi_pinmux_free(char *);

struct gpmi_platform_data {

	u_int32_t	uid_offset;
	u_int32_t	uid_size;

	int items;
	int io_uA;
	char *concat_name;
	char **concat_parts;

	struct {
		const char **part_probe_types;
		int nr_partitions;
		struct mtd_partition *partitions;
	} parts[];

};
#endif /* __ASM_PLAT_GPMI_H */
