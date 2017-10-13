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


/*
 * Copyright (C) 2015 Etnaviv Project
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __GC_HAL_DRM_H__
#define __GC_HAL_DRM_H__

#if defined(__cplusplus)
extern "C" {
#endif

enum VIV_GEM_PARAM {
    VIV_GEM_PARAM_NODE = 0,
    VIV_GEM_PARAM_POOL,
    VIV_GEM_PARAM_SIZE,
};

struct drm_viv_gem_create {
    __u64 size;         /* in */
    __u32 flags;        /* in */
    __u32 handle;       /* out */
};

struct drm_viv_gem_lock {
    __u32 handle;
    __u32 cacheable;
    __u32 gpu_va;
    __u64 cpu_va;
};

struct drm_viv_gem_unlock {
    __u32 handle;
};

struct drm_viv_gem_cache {
    __u32 handle;
    __u32 op;
    __u64 logical;
    __u64 bytes;
};

struct drm_viv_gem_getinfo {
    __u32 handle;
    __u32 param;
    __u64 value;
};

#define DRM_VIV_GEM_CREATE          0x00
#define DRM_VIV_GEM_LOCK            0x01
#define DRM_VIV_GEM_UNLOCK          0x02
#define DRM_VIV_GEM_CACHE           0x03
#define DRM_VIV_GEM_GETINFO         0x04
#define DRM_VIV_NUM_IOCTLS          0x05

#define DRM_IOCTL_VIV_GEM_CREATE    DRM_IOWR(DRM_COMMAND_BASE + DRM_VIV_GEM_CREATE,     struct drm_viv_gem_create)
#define DRM_IOCTL_VIV_GEM_LOCK      DRM_IOWR(DRM_COMMAND_BASE + DRM_VIV_GEM_LOCK,       struct drm_viv_gem_lock)
#define DRM_IOCTL_VIV_GEM_UNLOCK    DRM_IOWR(DRM_COMMAND_BASE + DRM_VIV_GEM_UNLOCK,     struct drm_viv_gem_unlock)
#define DRM_IOCTL_VIV_GEM_CACHE     DRM_IOWR(DRM_COMMAND_BASE + DRM_VIV_GEM_CACHE,      struct drm_viv_gem_cache)
#define DRM_IOCTL_VIV_GEM_GETINFO   DRM_IOWR(DRM_COMMAND_BASE + DRM_VIV_GEM_GETINFO,    struct drm_viv_gem_getinfo)

#if defined(__cplusplus)
}
#endif

#endif /* __ETNAVIV_DRM_H__ */
