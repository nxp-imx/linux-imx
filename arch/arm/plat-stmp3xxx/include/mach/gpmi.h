#ifndef __MACH_GPMI_H

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <mach/regs-gpmi.h>

#define GPMI_PART_CONCAT 		0x8000	/* indicates that partitions
						   should be concatenated    */
extern int gpmi_pinmux_request(char *);
extern void gpmi_pinmux_free(char *);

/**
 * struct gpmi_platform_data - GPMI driver platform data.
 *
 * This structure communicates information to the GPMI driver that can't be
 * expressed as resources.
 *
 * @io_uA:             The current limit, in uA.
 * @uid_offset:        The offset into the physical medium of the "Unique ID"
 *                     area.
 * @uid_size:          The size of the "Unique ID" area.
 * @concat_name:       The name of an MTD the driver will create by
 *                     concatenating all the chip partitions named by the
 *                     concat_parts field.
 * @concat_parts:      A list of chip partition names that will be concatenated
 *                     into a single MTD. The partitions are described in the
 *                     array at the end of this structure.
 * @chip_count:        The number of physical chips for which partitioning
 *                     information is provided.
 * @chip_parts:        An array indexed by physical chip number that contains
 *                     descriptions of partitions to be applied to the
 *                     corresponding chip.
 * @part_probe_types:  A pointer to a list of strings that identify possible
 *                     sources of partitioning information. As it happens, only
 *                     the information appearing in element zero is used.
 * @nr_partitions:     The number of partitions to apply to the current chip.
 * @partitions:        A pointer to an array of partition descriptions for the
 *                     current chip.
 */

struct gpmi_platform_data {

	int  io_uA;

	u_int32_t	uid_offset;
	u_int32_t	uid_size;

	char *concat_name;
	char **concat_parts;

	int (*pinmux) (int req);

	int  chip_count;

	struct {
		const char **part_probe_types;
		int nr_partitions;
		struct mtd_partition *partitions;
	} chip_partitions[];

};
#endif
