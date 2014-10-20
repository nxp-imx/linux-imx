/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Freescale IMX Linux-specific MCC implementation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MCC_LINUX__
#define __MCC_LINUX__

#include <linux/sched.h>
#include <linux/string.h>

/* Define the kinds of cache macros */
#define MCC_DCACHE_ENABLE(n)
#define MCC_DCACHE_DISABLE()
#define MCC_DCACHE_FLUSH()
#define MCC_DCACHE_FLUSH_LINE(p)
#define MCC_DCACHE_FLUSH_MLINES(p, m)
#define MCC_DCACHE_INVALIDATE()
#define MCC_DCACHE_INVALIDATE_LINE(p)
#define MCC_DCACHE_INVALIDATE_MLINES(p, m)

#define MCC_LINUX_NODE_NUMBER                (0)

void * mcc_virt_to_phys(void *);
void * mcc_phys_to_virt(void *);
#define MCC_MEM_VIRT_TO_PHYS(x) mcc_virt_to_phys(x)
#define MCC_MEM_PHYS_TO_VIRT(x) mcc_phys_to_virt(x)

/* Semaphore-related functions */
int mcc_init_semaphore(unsigned int);
int mcc_deinit_semaphore(unsigned int);
int mcc_get_semaphore(void);
int mcc_release_semaphore(void);

/* CPU-to-CPU interrupt-related functions */
int mcc_register_cpu_to_cpu_isr(void);
int mcc_generate_cpu_to_cpu_interrupt(void);

/* Memory management-related functions */
void mcc_memcpy(void *, void *, unsigned int);

int mcc_init_os_sync(void);
int mcc_deinit_os_sync(void);
void mcc_clear_os_sync_for_ep(MCC_ENDPOINT *);
int mcc_wait_for_buffer_freed(MCC_RECEIVE_BUFFER **, unsigned int);
int mcc_wait_for_buffer_queued(MCC_ENDPOINT *, unsigned int);

MCC_BOOKEEPING_STRUCT *mcc_get_bookeeping_data(void);
#endif /* __MCC_LINUX__ */
