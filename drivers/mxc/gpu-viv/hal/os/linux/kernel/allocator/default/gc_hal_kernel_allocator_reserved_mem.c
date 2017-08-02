/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2017 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2017 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/


#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_allocator.h"
#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/mman.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/ioport.h>

#define _GC_OBJ_ZONE    gcvZONE_OS

/*
 * reserved_mem is for contiguous pool, internal pool and external pool, etc.
 */

/* mdl private. */
struct reserved_mem
{
    unsigned long start;
    unsigned long size;
    char name[32];
    int  release;

    /* Link together. */
    struct list_head link;
};

/* allocator info. */
struct reserved_mem_alloc
{
    /* Record allocated reserved memory regions. */
    struct list_head region;
    struct mutex lock;
};

static int reserved_mem_show(struct seq_file* m, void* data)
{
    struct list_head *pos;
    gcsINFO_NODE *node = m->private;
    gckALLOCATOR Allocator = node->device;
    struct reserved_mem_alloc *alloc = Allocator->privateData;

    list_for_each(pos, &alloc->region)
    {
        struct reserved_mem * res= list_entry(pos, struct reserved_mem, link);

        seq_printf(m, "0x%08lx-0x%08lx : %s\n",
            res->start, res->start + res->size -1, res->name);
    }

    return 0;
}

static gcsINFO info_list[] =
{
    {"reserved-mem", reserved_mem_show},
};

static void
reserved_mem_debugfs_init(
    IN gckALLOCATOR Allocator,
    IN gckDEBUGFS_DIR Root
    )
{
    gcmkVERIFY_OK(
        gckDEBUGFS_DIR_Init(&Allocator->debugfsDir, Root->root, "reserved-mem"));

    gcmkVERIFY_OK(gckDEBUGFS_DIR_CreateFiles(
        &Allocator->debugfsDir,
        info_list,
        gcmCOUNTOF(info_list),
        Allocator
        ));
}

static void
reserved_mem_debugfs_cleanup(
    IN gckALLOCATOR Allocator
    )
{
    gcmkVERIFY_OK(gckDEBUGFS_DIR_RemoveFiles(
        &Allocator->debugfsDir,
        info_list,
        gcmCOUNTOF(info_list)
        ));

    gckDEBUGFS_DIR_Deinit(&Allocator->debugfsDir);
}

static gceSTATUS
reserved_mem_attach(
    IN gckALLOCATOR Allocator,
    IN gcsATTACH_DESC_PTR Desc,
    IN PLINUX_MDL Mdl
    )
{
    struct reserved_mem_alloc *alloc = Allocator->privateData;
    struct reserved_mem *res;
    struct resource *region = NULL;

    res = kzalloc(sizeof(struct reserved_mem), GFP_KERNEL | gcdNOWARN);

    if (!res)
        return gcvSTATUS_OUT_OF_MEMORY;

    res->start = Desc->reservedMem.start;
    res->size  = Desc->reservedMem.size;
    strncpy(res->name, Desc->reservedMem.name, sizeof(res->name)-1);
    res->release = 1;

    if (!Desc->reservedMem.requested)
    {
        region = request_mem_region(res->start, res->size, res->name);

        if (!region)
        {
            printk("request mem %s(0x%lx - 0x%lx) failed\n",
                res->name, res->start, res->start + res->size - 1);

            kfree(res);
            return gcvSTATUS_OUT_OF_RESOURCES;
        }

        res->release = 1;
    }

    mutex_lock(&alloc->lock);
    list_add(&res->link, &alloc->region);
    mutex_unlock(&alloc->lock);

    Mdl->priv = res;

    return gcvSTATUS_OK;
}

static void
reserved_mem_detach(
    IN gckALLOCATOR Allocator,
    IN OUT PLINUX_MDL Mdl
    )
{
    struct reserved_mem_alloc *alloc = Allocator->privateData;
    struct reserved_mem *res = Mdl->priv;

    /* unlink from region list. */
    mutex_lock(&alloc->lock);
    list_del_init(&res->link);
    mutex_unlock(&alloc->lock);

    if (res->release)
    {
        release_mem_region(res->start, res->size);
    }

    kfree(res);
}

static void
munmap_user(unsigned long vaddr, unsigned long size)
{
    if (unlikely(!current->mm))
        return;

#if defined(CONFIG_X86)
    if (set_memory_wb(vaddr, size >> PAGE_SHIFT))
    {
        printk("%s: failed to set_memory_wb\n", __func__);
    }
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
    if (vm_munmap(vaddr, size) < 0)
    {
        printk("%s: vm_munmap failed\n", __func__);
    }
#else
    down_write(&current->mm->mmap_sem);
    if (do_munmap(current->mm, vaddr, size) < 0)
    {
        printk("%s: do_munmap failed\n", __func__);
    }
    up_write(&current->mm->mmap_sem);
#endif
}

static gctINT
reserved_mem_map_user(
    gckALLOCATOR Allocator,
    PLINUX_MDL Mdl,
    gctBOOL Cacheable,
    OUT gctPOINTER *UserLogical
    )
{
    struct reserved_mem *res = Mdl->priv;
    unsigned long vaddr;
    struct vm_area_struct *vma;
    unsigned int npages;
    unsigned long pfn;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p Cacheable=%d", Allocator, Mdl, Cacheable);

    npages = res->size >> PAGE_SHIFT;
    pfn = res->start >> PAGE_SHIFT;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
    vaddr = vm_mmap(NULL, 0L, res->size,
                PROT_READ | PROT_WRITE, MAP_SHARED, 0);
#else
    down_write(&current->mm->mmap_sem);

    vaddr = do_mmap_pgoff(NULL, 0L, res->size,
                PROT_READ | PROT_WRITE, MAP_SHARED, 0);

    up_write(&current->mm->mmap_sem);
#endif

    if (IS_ERR_VALUE(vaddr))
    {
        vaddr = 0;
        gcmkFOOTER_ARG("*status=%d", gcvSTATUS_OUT_OF_MEMORY);
        return gcvSTATUS_OUT_OF_MEMORY;
    }

    /* Remap here since there is no file and ops->mmap(). */
    down_write(&current->mm->mmap_sem);

    vma = find_vma(current->mm, vaddr);

    if (!vma)
    {
        up_write(&current->mm->mmap_sem);

        gcmkFOOTER_ARG("*status=%d", gcvSTATUS_OUT_OF_RESOURCES);
        return gcvSTATUS_OUT_OF_RESOURCES;
    }

    /* Make this mapping non-cached. */
    vma->vm_flags |= gcdVM_FLAGS;
    vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

#if defined(CONFIG_X86)
    if (set_memory_wc(vma->vm_start, npages))
    {
        printk("%s: failed to set_memory_wc\n", __func__);
    }
#endif

    if (remap_pfn_range(vma, vma->vm_start,
            pfn, res->size, vma->vm_page_prot) < 0)
    {
        up_write(&current->mm->mmap_sem);

        munmap_user(vaddr, res->size);

        gcmkFOOTER_ARG("*status=%d", gcvSTATUS_OUT_OF_MEMORY);
        return gcvSTATUS_OUT_OF_MEMORY;
    }

    up_write(&current->mm->mmap_sem);

    *UserLogical = (void *)vaddr;
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

static void
reserved_mem_unmap_user(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical,
    IN gctUINT32 Size
    )
{
    munmap_user((unsigned long)Logical, (unsigned long)Size);
}

static gceSTATUS
reserved_mem_map_kernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    OUT gctPOINTER *Logical
    )
{
    struct reserved_mem *res = Mdl->priv;
    void *vaddr;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
    vaddr = memremap(res->start, res->size, MEMREMAP_WC);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4,3,0)
    vaddr = memremap(res->start, res->size, MEMREMAP_WT);
#else
    vaddr = ioremap_nocache(res->start, res->size);
#endif

    if (!vaddr)
    {
        return gcvSTATUS_OUT_OF_MEMORY;
    }

    *Logical = vaddr;
    return gcvSTATUS_OK;;
}

static gceSTATUS
reserved_mem_unmap_kernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,3,0)
    memunmap((void *)Logical);
#else
    iounmap((void *)Logical);
#endif
    return gcvSTATUS_OK;
}

static gceSTATUS
reserved_mem_cache_op(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical,
    IN gctUINT32 Physical,
    IN gctUINT32 Bytes,
    IN gceCACHEOPERATION Operation
    )
{
    return gcvSTATUS_OK;
}

static gceSTATUS
reserved_mem_get_physical(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctUINT32 Offset,
    OUT gctPHYS_ADDR_T * Physical
    )
{
    struct reserved_mem *res = Mdl->priv;
    *Physical = res->start + Offset;

    return gcvSTATUS_OK;
}

static void
reserved_mem_dtor(
    gcsALLOCATOR *Allocator
    )
{
    reserved_mem_debugfs_cleanup(Allocator);

    if (Allocator->privateData)
    {
        kfree(Allocator->privateData);
    }

    kfree(Allocator);
}

/* GFP allocator operations. */
static gcsALLOCATOR_OPERATIONS reserved_mem_ops = {
    .Alloc              = NULL,
    .Attach             = reserved_mem_attach,
    .Free               = reserved_mem_detach,
    .MapUser            = reserved_mem_map_user,
    .UnmapUser          = reserved_mem_unmap_user,
    .MapKernel          = reserved_mem_map_kernel,
    .UnmapKernel        = reserved_mem_unmap_kernel,
    .Cache              = reserved_mem_cache_op,
    .Physical           = reserved_mem_get_physical,
};

/* GFP allocator entry. */
gceSTATUS
_ReservedMemoryAllocatorInit(
    IN gckOS Os,
    IN gcsDEBUGFS_DIR *Parent,
    OUT gckALLOCATOR * Allocator
    )
{
    gceSTATUS status;
    gckALLOCATOR allocator = gcvNULL;
    struct reserved_mem_alloc *alloc = NULL;

    gcmkONERROR(
        gckALLOCATOR_Construct(Os, &reserved_mem_ops, &allocator));

    alloc = kzalloc(sizeof(*alloc), GFP_KERNEL | gcdNOWARN);

    if (!alloc)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    INIT_LIST_HEAD(&alloc->region);
    mutex_init(&alloc->lock);

    /* Register private data. */
    allocator->privateData = alloc;
    allocator->destructor = reserved_mem_dtor;

    reserved_mem_debugfs_init(allocator, Parent);

    allocator->capability = gcvALLOC_FLAG_LINUX_RESERVED_MEM;

    *Allocator = allocator;

    return gcvSTATUS_OK;

OnError:
    if (allocator)
    {
        kfree(allocator);
    }
    return status;
}

