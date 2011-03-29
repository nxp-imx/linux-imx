/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __GSL_KMOD_CLEANUP_H
#define __GSL_KMOD_CLEANUP_H
#include "gsl_types.h"

#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/list.h>

#if (GSL_CONTEXT_MAX > 127)
    #error created_contexts_array supports context numbers only 127 or less.
#endif

static const s8 EMPTY_ENTRY = -1;

/* A structure to make list of allocated memory blocks. List per fd. */
/* should probably be allocated from slab cache to minimise fragmentation */
struct gsl_kmod_alloc_list
{
    struct list_head node;
    gsl_memdesc_t allocated_block;
    u32 allocation_number;
};

/* A structure to hold abovementioned list of blocks. Contain per fd data. */
struct gsl_kmod_per_fd_data
{
    struct list_head allocated_blocks_head; // list head
    u32 maximum_number_of_blocks;
    u32 number_of_allocated_blocks;
    s8 created_contexts_array[GSL_DEVICE_MAX][GSL_CONTEXT_MAX];
};


/* 
 * prototypes 
 */

/* allocated memory block tracking */
int add_memblock_to_allocated_list(struct file *fd,
                                   gsl_memdesc_t *allocated_block);

int del_memblock_from_allocated_list(struct file *fd,
                                     gsl_memdesc_t *freed_block);

int del_all_memblocks_from_allocated_list(struct file *fd);

/* created contexts tracking */
void init_created_contexts_array(s8 *array);

void add_device_context_to_array(struct file *fd,
                                 gsl_deviceid_t device_id,
                                 unsigned int context_id);

void del_device_context_from_array(struct file *fd, 
                                   gsl_deviceid_t device_id,
                                   unsigned int context_id);

void del_all_devices_contexts(struct file *fd);

#endif  // __GSL_KMOD_CLEANUP_H

