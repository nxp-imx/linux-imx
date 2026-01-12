/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2025-2026 ARM Limited. All rights reserved.
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

#ifndef _MALI_KBASE_AM_H_
#define _MALI_KBASE_AM_H_

#include <linux/types.h>

struct kbase_device;

/**
 * kbase_am_system_reset() - Resets the GPU on system level
 * @kbdev: Kbase device pointer
 *
 * Resets the GPU on system level. Try SOFT_RESET first then HARD_RESET.
 * hwaccess_lock is required to be held when called.
 *
 * Return: 0 if success or a Linux error code
 */
int kbase_am_system_reset(struct kbase_device *kbdev);

/**
 * kbase_am_irq_handler_aw() - Access Window interrupt handler
 * @kbdev: Kbase device pointer
 * @val: The value of the AW IRQ status register which triggered the call
 *
 * Called when an interrupt raised for Access Window.
 */
void kbase_am_irq_handler_aw(struct kbase_device *kbdev, u32 val);

/**
 * kbase_am_hw_issues_apply() - Program AM-SYSTEM hardware quirk registers
 * @kbdev: Kbase device pointer
 *
 * Writes the per-subsystem HW_QUIRKS values from @kbdev into the AM-SYSTEM
 * configuration registers to apply any required hardware issue workarounds.
 */
void kbase_am_hw_issues_apply(struct kbase_device *kbdev);

/**
 * kbase_am_cache_set_coherency_mode() - Set the AM coherency protocol
 * @kbdev: Kbase device pointer
 *
 * Updates the AMBA_ENABLE coherency protocol field on the AM-SYSTEM page using
 * the device's current system coherency mode and tracks the applied mode.
 */
void kbase_am_cache_set_coherency_mode(struct kbase_device *kbdev);

/**
 * kbase_am_amba_set_shareable_cache_support() - Enable AMBA shareable cache
 *                                               support on AM-SYSTEM
 * @kbdev: Kbase device pointer
 *
 * Sets the AMBA shareable cache support bit when coherency is enabled and the
 * interconnect advertises Shareable_Cache_Support capability.
 */
void kbase_am_amba_set_shareable_cache_support(struct kbase_device *kbdev);

/**
 * kbase_am_l2_config_override() - update L2_CONFIG register
 * @kbdev: Kbase device pointer
 *
 * This function updates L2_CONFIG register of AM-SYSTEM page
 * to update settings such as pbha, L2C_SLICE_HASH, etc.
 */
void kbase_am_l2_config_override(struct kbase_device *kbdev);

/**
 * kbase_am_power_on() - Kbase Access Manager submodule power-on handler
 * @kbdev: Kbase device pointer
 *
 * This function handles power-on for Kbase Access manager submodule.
 * hwaccess_lock is required to be held when called.
 *
 * Return: 0 if successful, otherwise a negative error code.
 */
int kbase_am_power_on(struct kbase_device *kbdev);

/**
 * kbase_am_power_off() - Kbase Access Manager submodule power-off handler
 * @kbdev: Kbase device pointer
 *
 * This function handles power-off for Kbase Access manager submodule.
 * hwaccess_lock is required to be held when called.
 */
void kbase_am_power_off(struct kbase_device *kbdev);

#endif /* _MALI_KBASE_AM_H_ */
