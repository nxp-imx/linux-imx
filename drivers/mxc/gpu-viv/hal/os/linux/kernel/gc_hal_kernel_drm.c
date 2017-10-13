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


#if gcdENABLE_DRM

#include <drm/drmP.h>
#include <drm/drm_gem.h>
#include <linux/dma-buf.h>
#include "gc_hal_kernel_linux.h"
#include "gc_hal_drm.h"

#define _GC_OBJ_ZONE    gcvZONE_KERNEL

/******************************************************************************\
******************************* gckKERNEL DRM Code ******************************
\******************************************************************************/

struct viv_gem_object {
    struct drm_gem_object base;

    uint32_t node;
};

struct dma_buf *viv_gem_prime_export(struct drm_device *drm,
                struct drm_gem_object *gem_obj,
                int flags)
{
    struct viv_gem_object *viv_obj = container_of(gem_obj, struct viv_gem_object, base);
    struct dma_buf *dmabuf = gcvNULL;
    gckGALDEVICE gal_dev = (gckGALDEVICE)drm->dev_private;

    if (gal_dev)
    {
        gckKERNEL kernel = gal_dev->device->map[gal_dev->device->defaultHwType].kernels[0];
        gcmkVERIFY_OK(gckVIDMEM_NODE_Export(kernel, viv_obj->node, flags,
                                            (gctPOINTER*)&dmabuf, gcvNULL));
    }

    return dmabuf;
}

struct drm_gem_object *viv_gem_prime_import(struct drm_device *drm,
                                            struct dma_buf *dmabuf)
{
    struct drm_gem_object *gem_obj = gcvNULL;
    struct viv_gem_object *viv_obj;

    gcsHAL_INTERFACE iface;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE gal_dev = gcvNULL;

    gal_dev = (gckGALDEVICE)drm->dev_private;
    if (!gal_dev)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gckOS_ZeroMemory(&iface, sizeof(iface));
    iface.command = gcvHAL_WRAP_USER_MEMORY;
    iface.hardwareType = gal_dev->device->defaultHwType;
    iface.u.WrapUserMemory.desc.flag = gcvALLOC_FLAG_DMABUF;
    iface.u.WrapUserMemory.desc.handle = -1;
    iface.u.WrapUserMemory.desc.dmabuf = gcmPTR_TO_UINT64(dmabuf);
    gcmkONERROR(gckDEVICE_Dispatch(gal_dev->device, &iface));

    /* ioctl output */
    gem_obj = kzalloc(sizeof(struct viv_gem_object), GFP_KERNEL);
    drm_gem_private_object_init(drm, gem_obj, dmabuf->size);
    viv_obj = container_of(gem_obj, struct viv_gem_object, base);
    viv_obj->node = iface.u.WrapUserMemory.node;

OnError:
    return gem_obj;
}

void viv_gem_free_object(struct drm_gem_object *gem_obj)
{
    struct viv_gem_object *viv_obj = container_of(gem_obj, struct viv_gem_object, base);
    struct drm_device *drm = gem_obj->dev;

    gcsHAL_INTERFACE iface;
    gckGALDEVICE gal_dev = (gckGALDEVICE)drm->dev_private;

    gckOS_ZeroMemory(&iface, sizeof(iface));
    iface.command = gcvHAL_RELEASE_VIDEO_MEMORY;
    iface.hardwareType = gal_dev->device->defaultHwType;
    iface.u.ReleaseVideoMemory.node = viv_obj->node;
    gcmkVERIFY_OK(gckDEVICE_Dispatch(gal_dev->device, &iface));

    drm_gem_object_release(gem_obj);
    kfree(gem_obj);
}

static int viv_ioctl_gem_create(struct drm_device *drm, void *data,
                                struct drm_file *file)
{
    int ret = 0;
    struct drm_viv_gem_create *args = (struct drm_viv_gem_create*)data;
    struct drm_gem_object *gem_obj;
    struct viv_gem_object *viv_obj;

    gcsHAL_INTERFACE iface;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE gal_dev = gcvNULL;

    gal_dev = (gckGALDEVICE)drm->dev_private;
    if (!gal_dev)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gckOS_ZeroMemory(&iface, sizeof(iface));
    iface.command = gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY;
    iface.hardwareType = gal_dev->device->defaultHwType;
    iface.u.AllocateLinearVideoMemory.bytes = PAGE_ALIGN(args->size);
    iface.u.AllocateLinearVideoMemory.alignment = 256;
    iface.u.AllocateLinearVideoMemory.type = gcvSURF_RENDER_TARGET; /* should be general */
    iface.u.AllocateLinearVideoMemory.flag = args->flags;
    iface.u.AllocateLinearVideoMemory.pool = gcvPOOL_DEFAULT;
    gcmkONERROR(gckDEVICE_Dispatch(gal_dev->device, &iface));

    /* ioctl output */
    gem_obj = kzalloc(sizeof(struct viv_gem_object), GFP_KERNEL);
    drm_gem_private_object_init(drm, gem_obj, iface.u.AllocateLinearVideoMemory.bytes);
    ret = drm_gem_handle_create(file, gem_obj, &args->handle);

    viv_obj = container_of(gem_obj, struct viv_gem_object, base);
    viv_obj->node = iface.u.AllocateLinearVideoMemory.node;

    /* drop reference from allocate - handle holds it now */
    drm_gem_object_unreference_unlocked(gem_obj);

OnError:
    return gcmIS_ERROR(status) ? -ENOTTY : 0;
}

static int viv_ioctl_gem_lock(struct drm_device *drm, void *data,
                              struct drm_file *file)
{
    struct drm_viv_gem_lock *args = (struct drm_viv_gem_lock*)data;
    struct drm_gem_object *gem_obj;
    struct viv_gem_object *viv_obj;

    gcsHAL_INTERFACE iface;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE gal_dev = gcvNULL;

    gal_dev = (gckGALDEVICE)drm->dev_private;
    if (!gal_dev)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gem_obj = drm_gem_object_lookup(file, args->handle);
    if (!gem_obj)
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }
    viv_obj = container_of(gem_obj, struct viv_gem_object, base);

    gckOS_ZeroMemory(&iface, sizeof(iface));
    iface.command = gcvHAL_LOCK_VIDEO_MEMORY;
    iface.hardwareType = gal_dev->device->defaultHwType;
    iface.u.LockVideoMemory.node = viv_obj->node;
    iface.u.LockVideoMemory.cacheable = args->cacheable;
    gcmkONERROR(gckDEVICE_Dispatch(gal_dev->device, &iface));

    args->cpu_va = iface.u.LockVideoMemory.memory;
    args->gpu_va = iface.u.LockVideoMemory.address;

OnError:
    return gcmIS_ERROR(status) ? -ENOTTY : 0;
}

static int viv_ioctl_gem_unlock(struct drm_device *drm, void *data,
                                struct drm_file *file)
{
    struct drm_viv_gem_unlock *args = (struct drm_viv_gem_unlock*)data;
    struct drm_gem_object *gem_obj;
    struct viv_gem_object *viv_obj;

    gcsHAL_INTERFACE iface;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE gal_dev = gcvNULL;

    gal_dev = (gckGALDEVICE)drm->dev_private;
    if (!gal_dev)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gem_obj = drm_gem_object_lookup(file, args->handle);
    if (!gem_obj)
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }
    drm_gem_object_unreference_unlocked(gem_obj);
    viv_obj = container_of(gem_obj, struct viv_gem_object, base);

    gckOS_ZeroMemory(&iface, sizeof(iface));
    iface.command = gcvHAL_UNLOCK_VIDEO_MEMORY;
    iface.hardwareType = gal_dev->device->defaultHwType;
    iface.u.UnlockVideoMemory.node = (gctUINT64)viv_obj->node;
    iface.u.UnlockVideoMemory.type = gcvSURF_TYPE_UNKNOWN;
    gcmkONERROR(gckDEVICE_Dispatch(gal_dev->device, &iface));

    /*
     * decrease obj->refcount one more time because we has already
     * increased it at viv_ioctl_gem_lock().
     */
    drm_gem_object_unreference_unlocked(gem_obj);

OnError:
    return gcmIS_ERROR(status) ? -ENOTTY : 0;
}

static int viv_ioctl_gem_cache(struct drm_device *drm, void *data,
                               struct drm_file *file)
{
    struct drm_viv_gem_cache *args = (struct drm_viv_gem_cache*)data;
    struct drm_gem_object *gem_obj;
    struct viv_gem_object *viv_obj;

    gcsHAL_INTERFACE iface;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE gal_dev = gcvNULL;

    gal_dev = (gckGALDEVICE)drm->dev_private;
    if (!gal_dev)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gem_obj = drm_gem_object_lookup(file, args->handle);
    if (!gem_obj)
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }
    drm_gem_object_unreference_unlocked(gem_obj);
    viv_obj = container_of(gem_obj, struct viv_gem_object, base);

    gckOS_ZeroMemory(&iface, sizeof(iface));
    iface.command = gcvHAL_CACHE;
    iface.hardwareType = gal_dev->device->defaultHwType;
    iface.u.Cache.node       = viv_obj->node;
    iface.u.Cache.operation  = args->op;
    iface.u.Cache.logical    = args->logical;
    iface.u.Cache.bytes      = args->bytes;
    gcmkONERROR(gckDEVICE_Dispatch(gal_dev->device, &iface));

OnError:
    return gcmIS_ERROR(status) ? -ENOTTY : 0;
}

static int viv_ioctl_gem_getinfo(struct drm_device *drm, void *data,
                                  struct drm_file *file)
{
    struct drm_viv_gem_getinfo *args = (struct drm_viv_gem_getinfo*)data;
    struct drm_gem_object *gem_obj;
    struct viv_gem_object *viv_obj;

    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE gal_dev = gcvNULL;
    gckKERNEL kernel;
    gctUINT32 processID;
    gckVIDMEM_NODE node = gcvNULL;

    gal_dev = (gckGALDEVICE)drm->dev_private;
    if (!gal_dev)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }
    kernel = gal_dev->device->map[gal_dev->device->defaultHwType].kernels[0];

    gem_obj = drm_gem_object_lookup(file, args->handle);
    if (!gem_obj)
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }
    drm_gem_object_unreference_unlocked(gem_obj);
    viv_obj = container_of(gem_obj, struct viv_gem_object, base);

    gcmkONERROR(gckOS_GetProcessID(&processID));
    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(kernel, processID, viv_obj->node, &node));
    switch (args->param)
    {
    case VIV_GEM_PARAM_NODE:
        args->value = (__u64)viv_obj->node;
        break;
    case VIV_GEM_PARAM_POOL:
        args->value = (__u64)node->pool;
        break;
    case VIV_GEM_PARAM_SIZE:
        args->value = (node->node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
                    ? (__u64)node->node->VidMem.bytes
                    : (__u64)node->node->Virtual.bytes;;
        break;
    default:
        break;
    }

OnError:
    return gcmIS_ERROR(status) ? -ENOTTY : 0;
}

static const struct drm_ioctl_desc viv_ioctls[] =
{
    DRM_IOCTL_DEF_DRV(VIV_GEM_CREATE,   viv_ioctl_gem_create,   DRM_AUTH | DRM_RENDER_ALLOW),
    DRM_IOCTL_DEF_DRV(VIV_GEM_LOCK,     viv_ioctl_gem_lock,     DRM_AUTH | DRM_RENDER_ALLOW),
    DRM_IOCTL_DEF_DRV(VIV_GEM_UNLOCK,   viv_ioctl_gem_unlock,   DRM_AUTH | DRM_RENDER_ALLOW),
    DRM_IOCTL_DEF_DRV(VIV_GEM_CACHE,    viv_ioctl_gem_cache,    DRM_AUTH | DRM_RENDER_ALLOW),
    DRM_IOCTL_DEF_DRV(VIV_GEM_GETINFO,  viv_ioctl_gem_getinfo,  DRM_AUTH | DRM_RENDER_ALLOW),
};

int viv_drm_open(struct drm_device *drm, struct drm_file *file)
{
    gctINT i;
    gctUINT32 pid = _GetProcessID();
    gckGALDEVICE gal_dev = (gckGALDEVICE)drm->dev_private;
    gceSTATUS status = gcvSTATUS_OK;

    for (i = 0; i < gcdMAX_GPU_COUNT; ++i)
    {
        if (gal_dev->kernels[i])
        {
            gcmkONERROR(gckKERNEL_AttachProcessEx(gal_dev->kernels[i], gcvTRUE, pid));
        }
    }
    file->driver_priv = gcmINT2PTR(pid);

OnError:
    return gcmIS_ERROR(status) ? -ENODEV :  0;
}

void viv_drm_postclose(struct drm_device *drm, struct drm_file *file)
{
    gctINT i;
    gctUINT32 pid = gcmPTR2INT(file->driver_priv);
    gckGALDEVICE gal_dev = (gckGALDEVICE)drm->dev_private;

    for (i = 0; i < gcdMAX_GPU_COUNT; ++i)
    {
        if (gal_dev->kernels[i])
        {
            gcmkVERIFY_OK(gckKERNEL_AttachProcessEx(gal_dev->kernels[i], gcvFALSE, pid));
        }
    }
}

static const struct file_operations viv_drm_fops = {
    .owner              = THIS_MODULE,
    .open               = drm_open,
    .release            = drm_release,
    .unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl       = drm_compat_ioctl,
#endif
    .poll               = drm_poll,
    .read               = drm_read,
    .llseek             = no_llseek,
};

static struct drm_driver viv_drm_driver = {
    .driver_features    = DRIVER_GEM | DRIVER_PRIME | DRIVER_RENDER,
    .open = viv_drm_open,
    .postclose = viv_drm_postclose,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0)
    .gem_free_object_unlocked = viv_gem_free_object,
#else
    .gem_free_object    = viv_gem_free_object,
#endif
    .prime_handle_to_fd = drm_gem_prime_handle_to_fd,
    .prime_fd_to_handle = drm_gem_prime_fd_to_handle,
    .gem_prime_export   = viv_gem_prime_export,
    .gem_prime_import   = viv_gem_prime_import,
    .ioctls             = viv_ioctls,
    .num_ioctls         = DRM_VIV_NUM_IOCTLS,
    .fops               = &viv_drm_fops,
    .name               = "vivante",
    .desc               = "vivante DRM",
    .date               = "20170808",
    .major              = 1,
    .minor              = 0,
};

int viv_drm_probe(struct device *dev)
{
    int ret = 0;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE gal_dev = gcvNULL;
    struct drm_device *drm = gcvNULL;

    gal_dev = (gckGALDEVICE)dev_get_drvdata(dev);
    if (!gal_dev)
    {
        ret = -ENODEV;
        gcmkONERROR(gcvSTATUS_INVALID_OBJECT);
    }

    drm = drm_dev_alloc(&viv_drm_driver, dev);
    if (IS_ERR(drm))
    {
        ret = PTR_ERR(drm);
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }
    drm->dev_private = (void*)gal_dev;

    ret = drm_dev_register(drm, 0);
    if (ret)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    gal_dev->drm = (void*)drm;

OnError:
    if (gcmIS_ERROR(status))
    {
        if (drm)
        {
            drm_dev_unref(drm);
        }
        printk(KERN_ERR "galcore: Failed to setup drm device.\n");
    }
    return ret;
}

int viv_drm_remove(struct device *dev)
{
    gckGALDEVICE gal_dev = (gckGALDEVICE)dev_get_drvdata(dev);

    if (gal_dev)
    {
        struct drm_device *drm = (struct drm_device*)gal_dev->drm;

        drm_dev_unregister(drm);
        drm_dev_unref(drm);
    }

    return 0;
}

#endif
