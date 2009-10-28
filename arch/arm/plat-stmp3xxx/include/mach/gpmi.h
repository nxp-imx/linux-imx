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
 * This structure communicates platform-specific information to the GPMI driver
 * that can't be expressed as resources.
 *
 * @io_uA:                   The current limit, in uA.
 * @pinmux_handler:          A pointer to a function the driver will call to
 *                           request or release the pins it needs. Pass true
 *                           to request pins, and false to release them.
 * @boot_area_size_in_bytes: The amount of space reserved for use by the boot
 *                           ROM on the first and second chips. If this value is
 *                           zero, it indicates we're not reserving any space
 *                           for the boot area.
 * @partition_source_types:  An array of strings that name sources of
 *                           partitioning information (e.g., the boot loader,
 *                           the kernel command line, etc.). The function
 *                           parse_mtd_partitions() recognizes these names and
 *                           applies the appropriate "plugins" to discover
 *                           partitioning information. If any is found, it will
 *                           be applied to the "general use" MTD (it will NOT
 *                           override the boot area protection mechanism).
 * @partitions:              An optional pointer to an array of partition
 *                           descriptions. If the driver finds no partitioning
 *                           information elsewhere, it will apply these to the
 *                           "general use" MTD (they do NOT override the boot
 *                           area protection mechanism).
 * @partition_count:         The number of elements in the partitions array.
 *
 * ----- Stay away from the "Unique ID" -- it will be going away soon. -----
 *
 * @uid_offset:              The offset into the physical medium of the
 *                           "Unique ID" area.
 * @uid_size:                The size of the "Unique ID" area.
 */

struct gpmi_platform_data {

	int                   io_uA;

	int                   (*pinmux_handler)(bool request);

	uint32_t              boot_area_size_in_bytes;

	const char            **partition_source_types;

	struct mtd_partition  *partitions;
	unsigned              partition_count;

	/* Stay away from the Unique ID - it will be going away soon. */

	u_int32_t             uid_offset;
	u_int32_t             uid_size;

};
#endif
