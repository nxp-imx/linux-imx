/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
 
#include "gsl.h"
#include "gsl_kmod_cleanup.h"

#include <linux/kernel.h>
#include <linux/fs.h>

/*
 * Local helper functions to check and convert device/context id's (1 based)
 * to index (0 based).
 */
static u32 device_id_to_device_index(gsl_deviceid_t device_id)
{
    KOS_ASSERT((GSL_DEVICE_ANY < device_id) && 
               (device_id <= GSL_DEVICE_MAX));
    return (u32)(device_id - 1);
}

/* 
 * Local helper function to check and get pointer to per file descriptor data 
 */
static struct gsl_kmod_per_fd_data *get_fd_private_data(struct file *fd)
{
    struct gsl_kmod_per_fd_data *datp; 

    KOS_ASSERT(fd);
    datp = (struct gsl_kmod_per_fd_data *)fd->private_data;
    KOS_ASSERT(datp);
    return datp;
}

static s8 *find_first_entry_with(s8 *subarray, s8 context_id)
{
    s8 *entry = NULL;
    int i;

//printk(KERN_DEBUG "At %s, ctx_id = %d\n", __func__, context_id);

    KOS_ASSERT(context_id >= EMPTY_ENTRY);    
    KOS_ASSERT(context_id <= GSL_CONTEXT_MAX);  // TODO: check the bound.

    for(i = 0; i < GSL_CONTEXT_MAX; i++)        // TODO: check the bound.
    {
        if(subarray[i] == (s8)context_id)
        {
            entry = &subarray[i];
            break;
        }
    }

    return entry;
}


/*
 * Add a memdesc into a list of allocated memory blocks for this file 
 * descriptor. The list is build in such a way that it implements FIFO (i.e.
 * list). Traces of tiger, tiger_ri and VG11 CTs should be analysed to make
 * informed choice.
 *
 * NOTE! gsl_memdesc_ts are COPIED so user space should NOT change them.
 */
int add_memblock_to_allocated_list(struct file *fd,
                                   gsl_memdesc_t *allocated_block)
{
    int err = 0;
    struct gsl_kmod_per_fd_data *datp;
    struct gsl_kmod_alloc_list *lisp;
    struct list_head *head;

    KOS_ASSERT(allocated_block);

    datp = get_fd_private_data(fd);

    head = &datp->allocated_blocks_head;
    KOS_ASSERT(head);

    /* allocate and put new entry in the list of allocated memory descriptors */
    lisp = (struct gsl_kmod_alloc_list *)kzalloc(sizeof(struct gsl_kmod_alloc_list), GFP_KERNEL);
    if(lisp)
    {
        INIT_LIST_HEAD(&lisp->node);

        /* builds FIFO (list_add() would build LIFO) */
        list_add_tail(&lisp->node, head);
        memcpy(&lisp->allocated_block, allocated_block, sizeof(gsl_memdesc_t));
        lisp->allocation_number = datp->maximum_number_of_blocks;
//        printk(KERN_DEBUG "List entry #%u allocated\n", lisp->allocation_number);

        datp->maximum_number_of_blocks++;
        datp->number_of_allocated_blocks++;

        err = 0;
    }
    else
    {
        printk(KERN_ERR "%s: Could not allocate new list element\n", __func__);
        err = -ENOMEM;
    }

    return err;
}

/* Delete a previously allocated memdesc from a list of allocated memory blocks */
int del_memblock_from_allocated_list(struct file *fd,
                                     gsl_memdesc_t *freed_block)
{
    struct gsl_kmod_per_fd_data *datp;
    struct gsl_kmod_alloc_list *cursor, *next;
    struct list_head *head;
//    int is_different;

    KOS_ASSERT(freed_block);

    datp = get_fd_private_data(fd);

    head = &datp->allocated_blocks_head;
    KOS_ASSERT(head);

    KOS_ASSERT(datp->number_of_allocated_blocks > 0);

    if(!list_empty(head))
    {
        list_for_each_entry_safe(cursor, next, head, node)
        {
            if(cursor->allocated_block.gpuaddr == freed_block->gpuaddr)
            {
//                is_different = memcmp(&cursor->allocated_block, freed_block, sizeof(gsl_memdesc_t));
//                KOS_ASSERT(!is_different);

                list_del(&cursor->node);
//                printk(KERN_DEBUG "List entry #%u freed\n", cursor->allocation_number);
                kfree(cursor);
                datp->number_of_allocated_blocks--;
                return 0;
            }
        }
    }
    return -EINVAL; // tried to free entry not existing or from empty list.
}

/* Delete all previously allocated memdescs from a list */
int del_all_memblocks_from_allocated_list(struct file *fd)
{
    struct gsl_kmod_per_fd_data *datp;
    struct gsl_kmod_alloc_list *cursor, *next;
    struct list_head *head;

    datp = get_fd_private_data(fd);

    head = &datp->allocated_blocks_head;
    KOS_ASSERT(head);

    if(!list_empty(head))
    {
        printk(KERN_INFO "Not all allocated memory blocks were freed. Doing it now.\n");
        list_for_each_entry_safe(cursor, next, head, node)
        {
		printk(KERN_DEBUG "Freeing list entry #%u, gpuaddr=%x\n",
		(u32)cursor->allocation_number,
		cursor->allocated_block.gpuaddr);

		kgsl_sharedmem_free(&cursor->allocated_block);
		list_del(&cursor->node);
		kfree(cursor);
        }
    }

    KOS_ASSERT(list_empty(head));
    datp->number_of_allocated_blocks = 0;

    return 0;
}

void init_created_contexts_array(s8 *array)
{
    memset((void*)array, EMPTY_ENTRY, GSL_DEVICE_MAX * GSL_CONTEXT_MAX);
}


void add_device_context_to_array(struct file *fd,
                                 gsl_deviceid_t device_id,
                                 unsigned int context_id)
{
    struct gsl_kmod_per_fd_data *datp;
    s8 *entry;
    s8 *subarray;
    u32 device_index = device_id_to_device_index(device_id);

    datp = get_fd_private_data(fd);

    subarray = datp->created_contexts_array[device_index];
    entry = find_first_entry_with(subarray, EMPTY_ENTRY);

    KOS_ASSERT(entry);
    KOS_ASSERT((datp->created_contexts_array[device_index] <= entry) &&
               (entry < datp->created_contexts_array[device_index] + GSL_CONTEXT_MAX));
    KOS_ASSERT(context_id < 127);
    *entry = (s8)context_id;
}

void del_device_context_from_array(struct file *fd, 
                                   gsl_deviceid_t device_id,
                                   unsigned int context_id)
{
    struct gsl_kmod_per_fd_data *datp;
    u32 device_index = device_id_to_device_index(device_id);
    s8 *entry;
    s8 *subarray;

    datp = get_fd_private_data(fd);

    KOS_ASSERT(context_id < 127);
    subarray = &(datp->created_contexts_array[device_index][0]);
    entry = find_first_entry_with(subarray, context_id);
    KOS_ASSERT(entry);
    KOS_ASSERT((datp->created_contexts_array[device_index] <= entry) &&
               (entry < datp->created_contexts_array[device_index] + GSL_CONTEXT_MAX));
    *entry = EMPTY_ENTRY;
}

void del_all_devices_contexts(struct file *fd)
{
    struct gsl_kmod_per_fd_data *datp;
    gsl_deviceid_t id;
    u32 device_index;
    u32 ctx_array_index;
    s8 ctx;
    int err;
    
    datp = get_fd_private_data(fd);

    /* device_id is 1 based */
    for(id = GSL_DEVICE_ANY + 1; id <= GSL_DEVICE_MAX; id++)
    {
        device_index = device_id_to_device_index(id);
        for(ctx_array_index = 0; ctx_array_index < GSL_CONTEXT_MAX; ctx_array_index++)
        {
            ctx = datp->created_contexts_array[device_index][ctx_array_index];
            if(ctx != EMPTY_ENTRY)
            {
                err = kgsl_context_destroy(id, ctx);
                if(err != GSL_SUCCESS)
                {
                    printk(KERN_ERR "%s: could not destroy context %d on device id = %u\n", __func__, ctx, id);
                }
                else
                {
                    printk(KERN_DEBUG "%s: Destroyed context %d on device id = %u\n", __func__, ctx, id);
                }
            }
        }
    }
}

