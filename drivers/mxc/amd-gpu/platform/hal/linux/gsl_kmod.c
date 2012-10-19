/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
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
 
#include "gsl_types.h"
#include "gsl.h"
#include "gsl_buildconfig.h"
#include "gsl_halconfig.h"
#include "gsl_ioctl.h"
#include "gsl_kmod_cleanup.h"
#include "gsl_linux_map.h"

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>

#include <mach/mxc_gpu.h>
#include <linux/fsl_devices.h>

int gpu_2d_irq, gpu_3d_irq;

phys_addr_t gpu_2d_regbase;
int gpu_2d_regsize;
phys_addr_t gpu_3d_regbase;
int gpu_3d_regsize;
int gmem_size;
phys_addr_t gpu_reserved_mem;
int gpu_reserved_mem_size;
int z160_version;
int enable_mmu;

static ssize_t gsl_kmod_read(struct file *fd, char __user *buf, size_t len, loff_t *ptr);
static ssize_t gsl_kmod_write(struct file *fd, const char __user *buf, size_t len, loff_t *ptr);
static long gsl_kmod_ioctl(struct file *fd, unsigned int cmd, unsigned long arg);
static int gsl_kmod_mmap(struct file *fd, struct vm_area_struct *vma);
static int gsl_kmod_fault(struct vm_area_struct *vma, struct vm_fault *vmf);
static int gsl_kmod_open(struct inode *inode, struct file *fd);
static int gsl_kmod_release(struct inode *inode, struct file *fd);
static irqreturn_t z160_irq_handler(int irq, void *dev_id);
static irqreturn_t z430_irq_handler(int irq, void *dev_id);

static int gsl_kmod_major;
static struct class *gsl_kmod_class;
DEFINE_MUTEX(gsl_mutex);

static const struct file_operations gsl_kmod_fops =
{
    .owner = THIS_MODULE,
    .read = gsl_kmod_read,
    .write = gsl_kmod_write,
    .unlocked_ioctl = gsl_kmod_ioctl,
    .mmap = gsl_kmod_mmap,
    .open = gsl_kmod_open,
    .release = gsl_kmod_release
};

static struct vm_operations_struct gsl_kmod_vmops =
{
	.fault = gsl_kmod_fault,
};

static ssize_t gsl_kmod_read(struct file *fd, char __user *buf, size_t len, loff_t *ptr)
{
    return 0;
}

static ssize_t gsl_kmod_write(struct file *fd, const char __user *buf, size_t len, loff_t *ptr)
{
    return 0;
}

static long gsl_kmod_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
    int kgslStatus = GSL_FAILURE;

    switch (cmd) {
    case IOCTL_KGSL_DEVICE_START:
        {
            kgsl_device_start_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_start_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_start(param.device_id, param.flags);
            break;
        }
    case IOCTL_KGSL_DEVICE_STOP:
        {
            kgsl_device_stop_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_stop_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_stop(param.device_id);
            break;
        }
    case IOCTL_KGSL_DEVICE_IDLE:
        {
            kgsl_device_idle_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_idle_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_idle(param.device_id, param.timeout);
            break;
        }
    case IOCTL_KGSL_DEVICE_ISIDLE:
        {
            kgsl_device_isidle_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_isidle_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_isidle(param.device_id);
            break;
        }
    case IOCTL_KGSL_DEVICE_GETPROPERTY:
        {
            kgsl_device_getproperty_t param;
            void *tmp;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_getproperty_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            tmp = kmalloc(param.sizebytes, GFP_KERNEL);
            if (!tmp)
            {
                printk(KERN_ERR "%s:kmalloc error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_getproperty(param.device_id, param.type, tmp, param.sizebytes);
            if (kgslStatus == GSL_SUCCESS)
            {
                if (copy_to_user(param.value, tmp, param.sizebytes))
                {
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    kfree(tmp);
                    break;
                }
            }
            else
            {
                printk(KERN_ERR "%s: kgsl_device_getproperty error\n", __func__);
            }
            kfree(tmp);
            break;
        }
    case IOCTL_KGSL_DEVICE_SETPROPERTY:
        {
            kgsl_device_setproperty_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_setproperty_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_setproperty(param.device_id, param.type, param.value, param.sizebytes);
            if (kgslStatus != GSL_SUCCESS)
            {
                printk(KERN_ERR "%s: kgsl_device_setproperty error\n", __func__);
            }
            break;
        }
    case IOCTL_KGSL_DEVICE_REGREAD:
        {
            kgsl_device_regread_t param;
            unsigned int tmp;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_regread_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_regread(param.device_id, param.offsetwords, &tmp);
            if (kgslStatus == GSL_SUCCESS)
            {
                if (copy_to_user(param.value, &tmp, sizeof(unsigned int)))
                {
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    break;
                }
            }
            break;
        }
    case IOCTL_KGSL_DEVICE_REGWRITE:
        {
            kgsl_device_regwrite_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_regwrite_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_regwrite(param.device_id, param.offsetwords, param.value);
            break;
        }
    case IOCTL_KGSL_DEVICE_WAITIRQ:
        {
            kgsl_device_waitirq_t param;
            unsigned int count;

            printk(KERN_ERR "IOCTL_KGSL_DEVICE_WAITIRQ obsoleted!\n");
//          kgslStatus = -ENOTTY; break;

            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_waitirq_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_waitirq(param.device_id, param.intr_id, &count, param.timeout);
            if (kgslStatus == GSL_SUCCESS)
            {
                if (copy_to_user(param.count, &count, sizeof(unsigned int)))
                {
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    break;
                }
            }
            break;
        }
    case IOCTL_KGSL_CMDSTREAM_ISSUEIBCMDS:
        {
            kgsl_cmdstream_issueibcmds_t param;
            gsl_timestamp_t tmp;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_cmdstream_issueibcmds_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_cmdstream_issueibcmds(param.device_id, param.drawctxt_index, param.ibaddr, param.sizedwords, &tmp, param.flags);
            if (kgslStatus == GSL_SUCCESS)
            {
                if (copy_to_user(param.timestamp, &tmp, sizeof(gsl_timestamp_t)))
                {
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    break;
                }
            }
            break;
        }
    case IOCTL_KGSL_CMDSTREAM_READTIMESTAMP:
        {
            kgsl_cmdstream_readtimestamp_t param;
            gsl_timestamp_t tmp;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_cmdstream_readtimestamp_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            tmp = kgsl_cmdstream_readtimestamp(param.device_id, param.type);
            if (copy_to_user(param.timestamp, &tmp, sizeof(gsl_timestamp_t)))
            {
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    break;
            }
            kgslStatus = GSL_SUCCESS;
            break;
        }
    case IOCTL_KGSL_CMDSTREAM_FREEMEMONTIMESTAMP:
        {
            int err;
            kgsl_cmdstream_freememontimestamp_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_cmdstream_freememontimestamp_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            err = del_memblock_from_allocated_list(fd, param.memdesc);
            if(err)
            {
                /* tried to remove a block of memory that is not allocated! 
                 * NOTE that -EINVAL is Linux kernel's error codes! 
                 * the drivers error codes COULD mix up with kernel's. */
                kgslStatus = -EINVAL;
            }
            else
            {
                kgslStatus = kgsl_cmdstream_freememontimestamp(param.device_id,
                                                               param.memdesc,
                                                               param.timestamp,
                                                               param.type);
            }
            break;
        }
    case IOCTL_KGSL_CMDSTREAM_WAITTIMESTAMP:
        {
            kgsl_cmdstream_waittimestamp_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_cmdstream_waittimestamp_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_cmdstream_waittimestamp(param.device_id, param.timestamp, param.timeout);
            break;
        }
    case IOCTL_KGSL_CMDWINDOW_WRITE:
        {
            kgsl_cmdwindow_write_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_cmdwindow_write_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_cmdwindow_write(param.device_id, param.target, param.addr, param.data);
            break;
        }
    case IOCTL_KGSL_CONTEXT_CREATE:
        {
            kgsl_context_create_t param;
            unsigned int tmp;
            int tmpStatus;

            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_context_create_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_context_create(param.device_id, param.type, &tmp, param.flags);
            if (kgslStatus == GSL_SUCCESS)
            {
                if (copy_to_user(param.drawctxt_id, &tmp, sizeof(unsigned int)))
                {
                    tmpStatus = kgsl_context_destroy(param.device_id, tmp);
                    /* is asserting ok? Basicly we should return the error from copy_to_user
                     * but will the user space interpret it correctly? Will the user space 
                     * always check against GSL_SUCCESS  or GSL_FAILURE as they are not the only
                     * return values.
                     */
                    KOS_ASSERT(tmpStatus == GSL_SUCCESS);
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    break;
                }
                else
                {
                    add_device_context_to_array(fd, param.device_id, tmp);
                }
            }
            break;
        }
    case IOCTL_KGSL_CONTEXT_DESTROY:
        {
            kgsl_context_destroy_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_context_destroy_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_context_destroy(param.device_id, param.drawctxt_id);
            del_device_context_from_array(fd, param.device_id, param.drawctxt_id);
            break;
        }
    case IOCTL_KGSL_DRAWCTXT_BIND_GMEM_SHADOW:
        {
            kgsl_drawctxt_bind_gmem_shadow_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_drawctxt_bind_gmem_shadow_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_drawctxt_bind_gmem_shadow(param.device_id, param.drawctxt_id, param.gmem_rect, param.shadow_x, param.shadow_y, param.shadow_buffer, param.buffer_id);
            break;
        }
    case IOCTL_KGSL_SHAREDMEM_ALLOC:
        {
            kgsl_sharedmem_alloc_t param;
            gsl_memdesc_t tmp;
            int tmpStatus;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_sharedmem_alloc_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_sharedmem_alloc(param.device_id, param.flags, param.sizebytes, &tmp);
            if (kgslStatus == GSL_SUCCESS)
            {
                if (copy_to_user(param.memdesc, &tmp, sizeof(gsl_memdesc_t)))
                {
                    tmpStatus = kgsl_sharedmem_free(&tmp);
                    KOS_ASSERT(tmpStatus == GSL_SUCCESS);
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    break;
                }
                else
                {
                    add_memblock_to_allocated_list(fd, &tmp);
                }
	    } else {
		    printk(KERN_ERR "GPU %s:%d kgsl_sharedmem_alloc failed!\n", __func__, __LINE__);
	    }
            break;
        }
    case IOCTL_KGSL_SHAREDMEM_FREE:
        {
            kgsl_sharedmem_free_t param;
            gsl_memdesc_t tmp;
            int err;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_sharedmem_free_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            if (copy_from_user(&tmp, (void __user *)param.memdesc, sizeof(gsl_memdesc_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            err = del_memblock_from_allocated_list(fd, &tmp);
            if(err)
            {
                printk(KERN_ERR "%s: tried to free memdesc that was not allocated!\n", __func__);
                kgslStatus = err;
                break;
            }
            kgslStatus = kgsl_sharedmem_free(&tmp);
            if (kgslStatus == GSL_SUCCESS)
            {
                if (copy_to_user(param.memdesc, &tmp, sizeof(gsl_memdesc_t)))
                {
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    break;
                }
            }
            break;
        }
    case IOCTL_KGSL_SHAREDMEM_READ:
        {
            kgsl_sharedmem_read_t param;
            gsl_memdesc_t memdesc;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_sharedmem_read_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            if (copy_from_user(&memdesc, (void __user *)param.memdesc, sizeof(gsl_memdesc_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_sharedmem_read(&memdesc, param.dst, param.offsetbytes, param.sizebytes, true);
            if (kgslStatus != GSL_SUCCESS)
            {
                printk(KERN_ERR "%s: kgsl_sharedmem_read failed\n", __func__);
            }
            break;
        }
    case IOCTL_KGSL_SHAREDMEM_WRITE:
        {
            kgsl_sharedmem_write_t param;
            gsl_memdesc_t memdesc;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_sharedmem_write_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            if (copy_from_user(&memdesc, (void __user *)param.memdesc, sizeof(gsl_memdesc_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_sharedmem_write(&memdesc, param.offsetbytes, param.src, param.sizebytes, true);
            if (kgslStatus != GSL_SUCCESS)
            {
                printk(KERN_ERR "%s: kgsl_sharedmem_write failed\n", __func__);
            }
            
            break;
        }
    case IOCTL_KGSL_SHAREDMEM_SET:
        {
            kgsl_sharedmem_set_t param;
            gsl_memdesc_t memdesc;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_sharedmem_set_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            if (copy_from_user(&memdesc, (void __user *)param.memdesc, sizeof(gsl_memdesc_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_sharedmem_set(&memdesc, param.offsetbytes, param.value, param.sizebytes);
            break;
        }
    case IOCTL_KGSL_SHAREDMEM_LARGESTFREEBLOCK:
        {
            kgsl_sharedmem_largestfreeblock_t param;
            unsigned int largestfreeblock;

            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_sharedmem_largestfreeblock_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            largestfreeblock = kgsl_sharedmem_largestfreeblock(param.device_id, param.flags);
            if (copy_to_user(param.largestfreeblock, &largestfreeblock, sizeof(unsigned int)))
            {
                printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = GSL_SUCCESS;
            break;
        }
    case IOCTL_KGSL_SHAREDMEM_CACHEOPERATION:
        {
            kgsl_sharedmem_cacheoperation_t param;
            gsl_memdesc_t memdesc;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_sharedmem_cacheoperation_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            if (copy_from_user(&memdesc, (void __user *)param.memdesc, sizeof(gsl_memdesc_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_sharedmem_cacheoperation(&memdesc, param.offsetbytes, param.sizebytes, param.operation);
            break;
        }
    case IOCTL_KGSL_SHAREDMEM_FROMHOSTPOINTER:
        {
            kgsl_sharedmem_fromhostpointer_t param;
            gsl_memdesc_t memdesc;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_sharedmem_fromhostpointer_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            if (copy_from_user(&memdesc, (void __user *)param.memdesc, sizeof(gsl_memdesc_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_sharedmem_fromhostpointer(param.device_id, &memdesc, param.hostptr);
            break;
        }
    case IOCTL_KGSL_ADD_TIMESTAMP:
        {
            kgsl_add_timestamp_t param;
            gsl_timestamp_t tmp;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_add_timestamp_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            tmp = kgsl_add_timestamp(param.device_id, &tmp);
            if (copy_to_user(param.timestamp, &tmp, sizeof(gsl_timestamp_t)))
            {
                    printk(KERN_ERR "%s: copy_to_user error\n", __func__);
                    kgslStatus = GSL_FAILURE;
                    break;
            }
            kgslStatus = GSL_SUCCESS;
            break;
        }
    
    case IOCTL_KGSL_DEVICE_CLOCK:
        {
            kgsl_device_clock_t param;
            if (copy_from_user(&param, (void __user *)arg, sizeof(kgsl_device_clock_t)))
            {
                printk(KERN_ERR "%s: copy_from_user error\n", __func__);
                kgslStatus = GSL_FAILURE;
                break;
            }
            kgslStatus = kgsl_device_clock(param.device, param.enable);
            break;
        }
    default:
        kgslStatus = -ENOTTY;
        break;
    }

    return kgslStatus;
}

static int gsl_kmod_mmap(struct file *fd, struct vm_area_struct *vma)
{
    int status = 0;
    unsigned long start = vma->vm_start;
    unsigned long pfn = vma->vm_pgoff;
    unsigned long size = vma->vm_end - vma->vm_start;
    unsigned long prot = pgprot_writecombine(vma->vm_page_prot);
    unsigned long addr = vma->vm_pgoff << PAGE_SHIFT;
    void *va = NULL;

    if (gsl_driver.enable_mmu && (addr < GSL_LINUX_MAP_RANGE_END) && (addr >= GSL_LINUX_MAP_RANGE_START)) {
	va = gsl_linux_map_find(addr);
	while (size > 0) {
	    if (remap_pfn_range(vma, start, vmalloc_to_pfn(va), PAGE_SIZE, prot)) {
		return -EAGAIN;
	    }
	    start += PAGE_SIZE;
	    va += PAGE_SIZE;
	    size -= PAGE_SIZE;
	}
    } else {
	if (remap_pfn_range(vma, start, pfn, size, prot)) {
	    status = -EAGAIN;
	}
    }

    vma->vm_ops = &gsl_kmod_vmops;

    return status;
}

static int gsl_kmod_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    return VM_FAULT_SIGBUS;
}

static int gsl_kmod_open(struct inode *inode, struct file *fd)
{
    gsl_flags_t flags = 0;
    struct gsl_kmod_per_fd_data *datp;
    int err = 0;

    if(mutex_lock_interruptible(&gsl_mutex))
    {
        return -EINTR;
    }

    if (kgsl_driver_entry(flags) != GSL_SUCCESS)
    {
        printk(KERN_INFO "%s: kgsl_driver_entry error\n", __func__);
        err = -EIO;  // TODO: not sure why did it fail?
    }
    else
    {
        /* allocate per file descriptor data structure */
        datp = (struct gsl_kmod_per_fd_data *)kzalloc(
                                             sizeof(struct gsl_kmod_per_fd_data),
                                             GFP_KERNEL);
        if(datp)
        {
            init_created_contexts_array(datp->created_contexts_array[0]);
            INIT_LIST_HEAD(&datp->allocated_blocks_head);

            fd->private_data = (void *)datp;
        }
        else
        {
            err = -ENOMEM;
        }
    }

    mutex_unlock(&gsl_mutex);

    return err;
}

static int gsl_kmod_release(struct inode *inode, struct file *fd)
{
    struct gsl_kmod_per_fd_data *datp;
    int err = 0;

    if(mutex_lock_interruptible(&gsl_mutex))
    {
        return -EINTR;
    }

    /* make sure contexts are destroyed */
    del_all_devices_contexts(fd);

    if (kgsl_driver_exit() != GSL_SUCCESS)
    {
        printk(KERN_INFO "%s: kgsl_driver_exit error\n", __func__);
        err = -EIO; // TODO: find better error code
    }
    else
    {
        /* release per file descriptor data structure */
        datp = (struct gsl_kmod_per_fd_data *)fd->private_data;
        del_all_memblocks_from_allocated_list(fd);
        kfree(datp);
        fd->private_data = 0;
    }

    mutex_unlock(&gsl_mutex);

    return err;
}

static struct class *gsl_kmod_class;

static irqreturn_t z160_irq_handler(int irq, void *dev_id)
{
    kgsl_intr_isr(&gsl_driver.device[GSL_DEVICE_G12-1]);
    return IRQ_HANDLED;
}

static irqreturn_t z430_irq_handler(int irq, void *dev_id)
{
    kgsl_intr_isr(&gsl_driver.device[GSL_DEVICE_YAMATO-1]);
    return IRQ_HANDLED;
}

static int gpu_probe(struct platform_device *pdev)
{
    int i;
    struct resource *res;
    struct device *dev;
    struct mxc_gpu_platform_data *gpu_data = NULL;

    gpu_data = (struct mxc_gpu_platform_data *)pdev->dev.platform_data;

    if (gpu_data == NULL)
	return 0;

    z160_version = gpu_data->z160_revision;
    enable_mmu = gpu_data->enable_mmu;

    for(i = 0; i < 2; i++){
        res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
        if (!res) {
            if (i == 0) {
                printk(KERN_ERR "gpu: unable to get gpu irq\n");
                return -ENODEV;
            } else {
                break;
            }
        }
        if(strcmp(res->name, "gpu_2d_irq") == 0){
            gpu_2d_irq = res->start;
        }else if(strcmp(res->name, "gpu_3d_irq") == 0){
            gpu_3d_irq = res->start;
        }
    }

	for (i = 0; i < 4; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
        if (!res) {
            gpu_2d_regbase = 0;
            gpu_2d_regsize = 0;
            gpu_3d_regbase = 0;
            gpu_2d_regsize = 0;
            gmem_size = 0;
            gpu_reserved_mem = 0;
            gpu_reserved_mem_size = 0;
            break;
        }else{
		if (strcmp(res->name, "gpu_2d_registers") == 0) {
			gpu_2d_regbase = res->start;
			gpu_2d_regsize = res->end - res->start + 1;
		} else if (strcmp(res->name, "gpu_3d_registers") == 0) {
			gpu_3d_regbase = res->start;
			gpu_3d_regsize = res->end - res->start + 1;
		} else if (strcmp(res->name, "gpu_graphics_mem") == 0) {
			gmem_size = res->end - res->start + 1;
		} else if (strcmp(res->name, "gpu_reserved_mem") == 0) {
			gpu_reserved_mem = res->start;
			gpu_reserved_mem_size = res->end - res->start + 1;
            }
        }
    }

    if (gpu_3d_irq > 0)
    {
	if (request_irq(gpu_3d_irq, z430_irq_handler, 0, "ydx", NULL) < 0) {
	    printk(KERN_ERR "%s: request_irq error\n", __func__);
	    gpu_3d_irq = 0;
	    goto request_irq_error;
	}
    }

    if (gpu_2d_irq > 0)
    {
	if (request_irq(gpu_2d_irq, z160_irq_handler, 0, "g12", NULL) < 0) {
	    printk(KERN_ERR "DO NOT use uio_pdrv_genirq kernel module for X acceleration!\n");
	    gpu_2d_irq = 0;
	}
    }

    if (kgsl_driver_init() != GSL_SUCCESS) {
	printk(KERN_ERR "%s: kgsl_driver_init error\n", __func__);
	goto kgsl_driver_init_error;
    }

    gsl_kmod_major = register_chrdev(0, "gsl_kmod", &gsl_kmod_fops);
    gsl_kmod_vmops.fault = gsl_kmod_fault;

    if (gsl_kmod_major <= 0)
    {
        pr_err("%s: register_chrdev error\n", __func__);
        goto register_chrdev_error;
    }

    gsl_kmod_class = class_create(THIS_MODULE, "gsl_kmod");

    if (IS_ERR(gsl_kmod_class))
    {
        pr_err("%s: class_create error\n", __func__);
        goto class_create_error;
    }

    #if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28))
        dev = device_create(gsl_kmod_class, NULL, MKDEV(gsl_kmod_major, 0), "gsl_kmod");
    #else
        dev = device_create(gsl_kmod_class, NULL, MKDEV(gsl_kmod_major, 0), NULL,"gsl_kmod");
    #endif

    if (!IS_ERR(dev))
    {
    //    gsl_kmod_data.device = dev;
        return 0;
    }

    pr_err("%s: device_create error\n", __func__);

class_create_error:
    class_destroy(gsl_kmod_class);

register_chrdev_error:
    unregister_chrdev(gsl_kmod_major, "gsl_kmod");

kgsl_driver_init_error:
    kgsl_driver_close();
    if (gpu_2d_irq > 0) {
	free_irq(gpu_2d_irq, NULL);
    }
    if (gpu_3d_irq > 0) {
	free_irq(gpu_3d_irq, NULL);
    }
request_irq_error:
    return 0;   // TODO: return proper error code
}

static int gpu_remove(struct platform_device *pdev)
{
    device_destroy(gsl_kmod_class, MKDEV(gsl_kmod_major, 0));
    class_destroy(gsl_kmod_class);
    unregister_chrdev(gsl_kmod_major, "gsl_kmod");

    if (gpu_3d_irq)
    {
        free_irq(gpu_3d_irq, NULL);
    }

    if (gpu_2d_irq)
    {
        free_irq(gpu_2d_irq, NULL);
    }

    kgsl_driver_close();
    return 0;
}

#ifdef CONFIG_PM
static int gpu_suspend(struct platform_device *pdev, pm_message_t state)
{
    int              i;
    gsl_powerprop_t  power;

    power.flags = GSL_PWRFLAGS_POWER_OFF;
    for (i = 0; i < GSL_DEVICE_MAX; i++)
    {
        kgsl_device_setproperty(
                        (gsl_deviceid_t) (i+1),
                        GSL_PROP_DEVICE_POWER,
                        &power,
                        sizeof(gsl_powerprop_t));
    }   

    return 0;
}

static int gpu_resume(struct platform_device *pdev)
{
    int              i;
    gsl_powerprop_t  power;

    power.flags = GSL_PWRFLAGS_POWER_ON;
    for (i = 0; i < GSL_DEVICE_MAX; i++)
    {
        kgsl_device_setproperty(
                        (gsl_deviceid_t) (i+1),
                        GSL_PROP_DEVICE_POWER,
                        &power,
                        sizeof(gsl_powerprop_t));
    }   

    return 0;
}
#else
#define	gpu_suspend	NULL
#define	gpu_resume	NULL
#endif /* !CONFIG_PM */

/*! Driver definition
 */
static struct platform_driver gpu_driver = {
    .driver = {
        .name = "mxc_gpu",
        },
    .probe = gpu_probe,
    .remove = gpu_remove,
    .suspend = gpu_suspend,
    .resume = gpu_resume,
};

static int __init gsl_kmod_init(void)
{
     return platform_driver_register(&gpu_driver);
}

static void __exit gsl_kmod_exit(void)
{
     platform_driver_unregister(&gpu_driver);
}

module_init(gsl_kmod_init);
module_exit(gsl_kmod_exit);
MODULE_AUTHOR("Advanced Micro Devices");
MODULE_DESCRIPTION("AMD graphics core driver for i.MX");
MODULE_LICENSE("GPL v2");
