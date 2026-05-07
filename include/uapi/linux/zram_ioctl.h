/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef _UAPI_LINUX_ZRAM_IOCTL_H
#define _UAPI_LINUX_ZRAM_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define ZRAM_ANDROID_IOC_VERSION 1

/* Legacy for process writeback */
struct zram_android_ioc_data_process_writeback {
	__aligned_u64	pidfd;
	__u64		written_bytes;
};

/*
 * Legacy for process writeback.
 * Modify the members or the size of the 'data' union can break the ABI
 * compatibility.
 *
 * Deprecated: Use struct zram_android_ioc_process_range_writeback for new
 * implementations.
 */
struct zram_android_ioc_data {
	union {
		struct zram_android_ioc_data_process_writeback process_writeback;
	} data;
};

struct zram_android_ioc_process_range_writeback {
	/* The pidfd of the process to scan for writeback candidates */
	__aligned_u64	pidfd;
	/* The starting virtual address for the scan */
	__u64		start_addr;
	/**
	 * Maximum number of candidate bytes to scan for writeback. The scan
	 * process will iterate the process address space from the @start_addr,
	 * and stop when the number of writeback candidates is larger than
	 * @size. If the @size is zero, the scan range is unlimited.
	 */
	__u64		size;
	/**
	 * The address where the scan stopped exclusively. Can be used as the
	 * @start_addr for the next call. If the scan reaches the end of the
	 * address space, it will be set to zero.
	 */
	__u64		next_addr;
	/* Total number of bytes successfully written back */
	__u64		written_bytes;
};

struct zram_android_ioc_process_prefetch {
	__aligned_u64	pidfd;
};

#define ZRAM_ANDROID_IOC_MAGIC 0xBB

/* Legacy API: keep it for compatibility */
#define ZRAM_ANDROID_IOC_PROCESS_WRITEBACK \
	_IOWR(ZRAM_ANDROID_IOC_MAGIC, 1, struct zram_android_ioc_data)

#define ZRAM_ANDROID_IOC_PROCESS_RANGE_WRITEBACK \
	_IOWR(ZRAM_ANDROID_IOC_MAGIC, 2, struct zram_android_ioc_process_range_writeback)

#define ZRAM_ANDROID_IOC_PROCESS_PREFETCH \
	_IOW(ZRAM_ANDROID_IOC_MAGIC, 3, struct zram_android_ioc_process_prefetch)

#define ZRAM_ANDROID_IOC_GET_VERSION _IO(ZRAM_ANDROID_IOC_MAGIC, 4)

#endif /* _UAPI_LINUX_ZRAM_IOCTL_H */

