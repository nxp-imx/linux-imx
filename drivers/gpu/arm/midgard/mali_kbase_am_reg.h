/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2025 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#ifndef _MALI_KBASE_AM_REG_H_
#define _MALI_KBASE_AM_REG_H_

#include <linux/types.h>

struct kbase_device;
enum kbase_reg_ext_type;

/* AM subpage offsets and sizes*/
#define KBASE_AM_REG_OFFSET_SYS (0x000000)
#define KBASE_AM_REG_OFFSET_GOV (0x020000)
#define KBASE_AM_REG_OFFSET_PTC (0x050000)
#define KBASE_AM_REG_OFFSET_AW0 (0x100000)
#define KBASE_AM_REG_SIZE_SYS (0x010000)
#define KBASE_AM_REG_SIZE_GOV (0x010000)
#define KBASE_AM_REG_SIZE_PTC (0x010000)
#define KBASE_AM_REG_SIZE_AW0 (0x480000)

/* System subpage register offsets */
#define AM_SYSTEM__PWR_OVERRIDE0 (0x104)
#define AM_SYSTEM__PWR_OVERRIDE1 (0x108)
#define AM_SYSTEM__AMBA_ENABLE (0x300)
#define AM_SYSTEM__L2_CONFIG (0x304)
#define AM_SYSTEM__L2C_SLICE_HASH_0 (0x310)
#define AM_SYSTEM__L2C_SLICE_HASH(n) (AM_SYSTEM__L2C_SLICE_HASH_0 + (n * 4))
#define AM_SYSTEM__CSF_CONFIG_0 (0x410)
#define AM_SYSTEM__CSF_CONFIG(n) (AM_SYSTEM__CSF_CONFIG_0 + (n * 4))
#define AM_SYSTEM__SHADER_CONFIG_0 (0x420)
#define AM_SYSTEM__SHADER_CONFIG(n) (AM_SYSTEM__SHADER_CONFIG_0 + (n * 4))
#define AM_SYSTEM__TILER_CONFIG_0 (0x430)
#define AM_SYSTEM__TILER_CONFIG(n) (AM_SYSTEM__TILER_CONFIG_0 + (n * 4))
#define AM_SYSTEM__L2_MMU_CONFIG_0 (0x440)
#define AM_SYSTEM__L2_MMU_CONFIG(n) (AM_SYSTEM__L2_MMU_CONFIG_0 + (n * 4))
#define AM_SYSTEM__NEURAL_CONFIG_0 (0x450)
#define AM_SYSTEM__NEURAL_CONFIG(n) (AM_SYSTEM__NEURAL_CONFIG_0 + (n * 4))

/* GOVERNOR sub page register offsets */
#define AM_GOVERNOR__AM_GOV_CORE_MASK (0x120)

/**
 * kbase_am_reg_read32 - read from 32-bit GPU register on AM subpages
 * @kbdev: Kbase device pointer
 * @reg_type: AM sub-page that the register belongs to
 * @reg_offset: Register address offset within AM sub-page
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_AM_OFF is not set).
 *
 * Return: Value in desired register
 */
u32 kbase_am_reg_read32(struct kbase_device *kbdev, enum kbase_reg_ext_type reg_type,
			u32 reg_offset);

/**
 * kbase_am_reg_read64 - read from 64-bit GPU register on AM subpages
 * @kbdev: Kbase device pointer
 * @reg_type: AM sub-page that the register belongs to
 * @reg_offset: Register address offset within AM sub-page
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_AM_OFF is not set).
 *
 * Return: Value in desired register
 */
u64 kbase_am_reg_read64(struct kbase_device *kbdev, enum kbase_reg_ext_type reg_type,
			u32 reg_offset);

/**
 * kbase_am_reg_write32 - write to 32-bit GPU register on AM subpages
 * @kbdev: Kbase device pointer
 * @reg_type: AM sub-page that the register belongs to
 * @reg_offset: Register address offset within AM sub-page
 * @value: Value to write
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_AM_OFF is not set).
 */
void kbase_am_reg_write32(struct kbase_device *kbdev, enum kbase_reg_ext_type reg_type,
			  u32 reg_offset, u32 value);

/**
 * kbase_am_reg_write64 - write to 64-bit GPU register on AM subpages
 * @kbdev: Kbase device pointer
 * @reg_type: AM sub-page that the register belongs to
 * @reg_offset: Register address offset within AM sub-page
 * @value: Value to write
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_AM_OFF is not set).
 */
void kbase_am_reg_write64(struct kbase_device *kbdev, enum kbase_reg_ext_type reg_type,
			  u32 reg_offset, u64 value);

/**
 * kbase_am_reg_read32_ipa - read from 32-bit IPA_CONTROL register on Governor subpage
 * @kbdev: Kbase device pointer
 * @reg_enum: hw_access register enum based.
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_AM_OFF is not set).
 *
 * Return: Value in desired register
 */
u32 kbase_am_reg_read32_ipa(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_am_reg_read64_ipa - read from 64-bit IPA_CONTROL register on Governor subpage
 * @kbdev: Kbase device pointer
 * @reg_enum: hw_access register enum based.
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_AM_OFF is not set).
 *
 * Return: Value in desired register
 */
u64 kbase_am_reg_read64_ipa(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_am_reg_write32_ipa - write to 32-bit IPA_CONTROL register on Governor subpage
 * @kbdev: Kbase device pointer
 * @reg_enum: hw_access register enum based.
 * @value: Value to write
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_AM_OFF is not set).
 */
void kbase_am_reg_write32_ipa(struct kbase_device *kbdev, u32 reg_enum, u32 value);

/**
 * kbase_am_reg_write64_ipa - write to 64-bit IPA_CONTROL register on Governor subpage
 * @kbdev: Kbase device pointer
 * @reg_enum: hw_access register enum based.
 * @value: Value to write
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_AM_OFF is not set).
 */
void kbase_am_reg_write64_ipa(struct kbase_device *kbdev, u32 reg_enum, u64 value);

#endif /* _MALI_KBASE_AM_REG_H_ */
