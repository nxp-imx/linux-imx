#ifndef __MACH_GPMI_H

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <mach/regs-gpmi.h>

#define GPMI_PART_CONCAT 		0x8000	/* indicates that partitions
						   should be concatenated    */

struct gpmi_platform_data {

	u_int32_t	uid_offset;
	u_int32_t	uid_size;

	int items;
	int io_uA;
	char *concat_name;
	char **concat_parts;
	int (*pinmux) (int req);

	struct {
		const char **part_probe_types;
		int nr_partitions;
		struct mtd_partition *partitions;
	} parts[];

};
#endif
