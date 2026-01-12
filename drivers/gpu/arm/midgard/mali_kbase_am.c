// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
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

#include <mali_kbase.h>

#include <linux/mali_hw_access.h>
#include <linux/version_compat_defs.h>

#include "mali_kbase_am.h"
#include "mali_kbase_am_reg.h"

#define ACCESS_WINDOW_ID (0x0)

/* Timeout values for System */
#define REG_POLL_RESET_TIMEOUT_SYS_US 20000

/* Timeout values for Partition-control */
#define REG_POLL_RESET_TIMEOUT_PTC_US 5000000

/* System register access macros */
#define AM_SYSTEM_IRQ_RAWSTAT (0x0040)
#define AM_SYSTEM_IRQ_CLEAR (0x004C)
#define AM_SYSTEM_IRQ_RESET_COMPLETED (1 << 1)

#define AM_SYSTEM_STATUS (0x00C0)
#define AM_SYSTEM_STATUS_SOFT_RESET (0x1 << 0)
#define AM_SYSTEM_STATUS_HARD_RESET (0x1 << 1)

#define AM_SYSTEM_COMMAND (0x00C4)
#define AM_SYSTEM_COMMAND_SET(command, idx) (command | (idx << 8))
#define AM_SYSTEM_COMMAND_SOFT_RESET (0x10)
#define AM_SYSTEM_COMMAND_HARD_RESET (0x11)

/* Partition control register access macros */
#define AM_PARTITION_STATE 0x60

#define AM_PARTITION_STATE_STATE_SHIFT 8
#define AM_PARTITION_STATE_STATE_MASK (0xF << AM_PARTITION_STATE_STATE_SHIFT)
#define AM_PARTITION_STATE_GET(reg_val) \
	((reg_val & AM_PARTITION_STATE_STATE_MASK) >> AM_PARTITION_STATE_STATE_SHIFT)
#define AM_PARTITION_STATE_WINDOW_SHIFT 12
#define AM_PARTITION_STATE_WINDOW_MASK (0xF << AM_PARTITION_STATE_WINDOW_SHIFT)
#define AM_PARTITION_STATE_WINDOW_GET(reg_val) \
	((reg_val & AM_PARTITION_STATE_WINDOW_MASK) >> AM_PARTITION_STATE_WINDOW_SHIFT)

#define AM_PARTITION_COMMAND_WINDOW_SHIFT 8
#define AM_PARTITION_COMMAND_WINDOW_MASK (0xF << AM_PARTITION_COMMAND_WINDOW_SHIFT)
#define AM_PARTITION_COMMAND_COMMAND_MASK (0xFF)

#define AM_PARTITION_COMMAND_SET(aw, cmd)                                                 \
	(((aw << AM_PARTITION_COMMAND_WINDOW_SHIFT) & AM_PARTITION_COMMAND_WINDOW_MASK) | \
	 (cmd & AM_PARTITION_COMMAND_COMMAND_MASK))

enum am_partition_states {
	AM_PARTITION_STATE_RESET = 0,
	AM_PARTITION_STATE_WINDOW_OPENING = 1,
	AM_PARTITION_STATE_WINDOW_OPEN = 7,
	AM_PARTITION_STATE_WINDOW_CLOSED = 8
};

#define AM_PARTITION_COMMAND 0x0100

#define AM_PARTITION_COMMAND_CLOSE_WINDOW 0x20
#define AM_PARTITION_COMMAND_OPEN_WINDOW 0x21

/* Access-Window register access */
#define AM_AW_WINDOW_IRQ_CLEAR 0x444
#define AM_AW_WINDOW_IRQ_MASK 0x448
#define IRQ_MESSAGE_ALL_MASK 0xFF

/**
 * kbasep_am_reset_partition() - Reset partition.
 * @kbdev: Kbase device pointer
 *
 * Resets the partition if a window is open, or if a window is currently opening,
 * it will wait and close it once it opens.
 * Does not reset if the partition is already reset and waits for RESET if the window is closing.
 *
 * Return: 0 if successful, otherwise a negative error code.
 */
static int kbasep_am_reset_partition(struct kbase_device *kbdev)
{
	int error;
	uint32_t partition_state;
	uint32_t command;

	if (!kbdev)
		return -EINVAL;

	partition_state = mali_readl(kbdev->reg_ext[KBASE_REG_EXT_PTC] + AM_PARTITION_STATE);

	/* If we are in any states that will change soon we need to wait for them to complete */
	switch (AM_PARTITION_STATE_GET(partition_state)) {
	case AM_PARTITION_STATE_RESET:
		return 0;
	case AM_PARTITION_STATE_WINDOW_CLOSED:
		/* Await already running reset */
		error = mali_read_poll_timeout_atomic(
			mali_readl, partition_state,
			AM_PARTITION_STATE_GET(partition_state) == AM_PARTITION_STATE_RESET, 0,
			REG_POLL_RESET_TIMEOUT_PTC_US, false,
			kbdev->reg_ext[KBASE_REG_EXT_PTC] + AM_PARTITION_STATE);

		if (error) {
			dev_err(kbdev->dev,
				"Awaiting window close timed out, Error: %d State: %d\n", error,
				partition_state);
			return -EIO;
		}

		/* Return 0 as the state is now RESET */
		return 0;
	case AM_PARTITION_STATE_WINDOW_OPENING:
		/* Await window open */
		error = mali_read_poll_timeout_atomic(
			mali_readl, partition_state,
			AM_PARTITION_STATE_GET(partition_state) == AM_PARTITION_STATE_WINDOW_OPEN,
			0, REG_POLL_RESET_TIMEOUT_PTC_US, false,
			kbdev->reg_ext[KBASE_REG_EXT_PTC] + AM_PARTITION_STATE);

		if (error) {
			dev_err(kbdev->dev, "Awaiting window open timed out, Error: %d State: %d\n",
				error, partition_state);
			return -EIO;
		}
		break;
	default:
		break;
	}

	command = AM_PARTITION_COMMAND_SET(0, AM_PARTITION_COMMAND_CLOSE_WINDOW);
	mali_writel(command, kbdev->reg_ext[KBASE_REG_EXT_PTC] + AM_PARTITION_COMMAND);

	error = mali_read_poll_timeout_atomic(
		mali_readl, partition_state,
		AM_PARTITION_STATE_GET(partition_state) == AM_PARTITION_STATE_RESET, 0,
		REG_POLL_RESET_TIMEOUT_PTC_US, false,
		kbdev->reg_ext[KBASE_REG_EXT_PTC] + AM_PARTITION_STATE);
	if (error) {
		dev_err(kbdev->dev, "Partition reset timed out. Error: %d, State: %d\n", error,
			AM_PARTITION_STATE_GET(partition_state));
		return error;
	}

	return error;
}

/**
 * kbasep_am_unassign() - Unassign Partition from any AW.
 * @kbdev: Kbase device pointer
 *
 * Reset this partition. Return when partition is ready.
 *
 * Return:
 * * 0		- successful.
 * * -EINVAL	- invalid argument.
 * * -EIO	- failed with hardware fault.
 */
static int kbasep_am_unassign(struct kbase_device *kbdev)
{
	int error;

	if (WARN_ON(!kbdev))
		return -EINVAL;

	/* Reset the partition if necessary */
	error = kbasep_am_reset_partition(kbdev);
	if (error) {
		dev_err(kbdev->dev, "Error resetting partition: %d\n", error);
		return -EIO;
	}

	return 0;
}

/**
 * kbasep_am_assign_aw0() - Assign Partition to the AW0.
 * @kbdev: Kbase device pointer
 *
 * Reset and set the active Access Window for this partition. Return when
 * partition is ready.
 *
 * Return:
 * * 0		- successful.
 * * -EINVAL	- invalid argument.
 * * -ETIMEDOUT - not yet settling to ready, wait timed out.
 * * -EIO	- failed with hardware fault.
 */
static int kbasep_am_assign_aw0(struct kbase_device *kbdev)
{
	int error;
	uint32_t partition_state;
	uint32_t command;

	if (WARN_ON(!kbdev))
		return -EINVAL;

	/* Check current partition state */
	partition_state = mali_readl(kbdev->reg_ext[KBASE_REG_EXT_PTC] + AM_PARTITION_STATE);
	if (AM_PARTITION_STATE_GET(partition_state) == AM_PARTITION_STATE_WINDOW_OPEN &&
	    AM_PARTITION_STATE_WINDOW_GET(partition_state) == ACCESS_WINDOW_ID) {
		dev_dbg(kbdev->dev, "Partition is already assigned to AW0.\n");
		return 0;
	}

	/* Unassign partition if a partition is currently assigned */
	if (AM_PARTITION_STATE_GET(partition_state) == AM_PARTITION_STATE_WINDOW_OPEN) {
		error = kbasep_am_unassign(kbdev);
		if (error) {
			dev_err(kbdev->dev, "Failed to unassign partition from AW%d. Error: %d\n",
				AM_PARTITION_STATE_WINDOW_GET(partition_state), error);
			return error;
		}
	}

	/* Assign partition to Access-window 0 */
	command = AM_PARTITION_COMMAND_SET(ACCESS_WINDOW_ID, AM_PARTITION_COMMAND_OPEN_WINDOW);
	mali_writel(command, kbdev->reg_ext[KBASE_REG_EXT_PTC] + AM_PARTITION_COMMAND);

	/* Command to open the AW0 is issued, timed-wait for the state to settle */
	error = mali_read_poll_timeout_atomic(
		mali_readl, partition_state,
		AM_PARTITION_STATE_GET(partition_state) == AM_PARTITION_STATE_WINDOW_OPEN &&
			AM_PARTITION_STATE_WINDOW_GET(partition_state) == ACCESS_WINDOW_ID,
		0, REG_POLL_RESET_TIMEOUT_PTC_US, false,
		kbdev->reg_ext[KBASE_REG_EXT_PTC] + AM_PARTITION_STATE);

	if (error) {
		if (AM_PARTITION_STATE_GET(partition_state) == AM_PARTITION_STATE_WINDOW_OPENING) {
			dev_err(kbdev->dev, "Partition ready timed out. Error: %d, State: %d\n",
				error, AM_PARTITION_STATE_GET(partition_state));

			return -ETIMEDOUT;
		}

		dev_err(kbdev->dev, "Unexpected state after setting AW, state: %d\n",
			AM_PARTITION_STATE_GET(partition_state));

		return -EIO;
	}

	return 0;
}

int kbase_am_system_reset(struct kbase_device *kbdev)
{
	int ret;
	int val;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	if (WARN_ON(!kbdev || !kbdev->am_standalone))
		return -EINVAL;

	/* Clear any previous reset IRQ */
	mali_writel(AM_SYSTEM_IRQ_RESET_COMPLETED,
		    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM_IRQ_CLEAR);

	/* Check if any SOFT-RESET or HARD-RESET is already active */
	val = mali_readl(kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM_STATUS);
	if (val & AM_SYSTEM_STATUS_SOFT_RESET || val & AM_SYSTEM_STATUS_HARD_RESET)
		dev_dbg(kbdev->dev, "Reset already in progress. (STATUS = %x)", val);
	else {
		/* Suspend AM first before sending reset */
		kbase_am_power_off(kbdev);

		/* Attempt soft reset */
		mali_writel(AM_SYSTEM_COMMAND_SET(AM_SYSTEM_COMMAND_SOFT_RESET, 0),
			    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM_COMMAND);
	}

	ret = readx_poll_timeout(mali_readl,
				 kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM_IRQ_RAWSTAT, val,
				 (val & AM_SYSTEM_IRQ_RESET_COMPLETED), 0,
				 REG_POLL_RESET_TIMEOUT_SYS_US);

	if (ret) {
		dev_err(kbdev->dev, "Failed system soft reset, attempting hard reset\n");

		/* Attempt hard reset */
		mali_writel(AM_SYSTEM_COMMAND_SET(AM_SYSTEM_COMMAND_HARD_RESET, 0),
			    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM_COMMAND);

		ret = readx_poll_timeout(mali_readl,
					 kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM_IRQ_RAWSTAT,
					 val, (val & AM_SYSTEM_IRQ_RESET_COMPLETED), 0,
					 REG_POLL_RESET_TIMEOUT_SYS_US);

		if (ret) {
			dev_err(kbdev->dev, "Failed system hard reset\n");
			return -EIO;
		}
	}

	/* Resume AM */
	ret = kbase_am_power_on(kbdev);
	if (ret)
		return ret;

	return 0;
}

void kbase_am_hw_issues_apply(struct kbase_device *kbdev)
{
	int i;

	for (i = 0; i < kbdev->hw_quirks_reg_size; i++) {
		mali_writel(kbdev->hw_quirks_sc[i],
			    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__SHADER_CONFIG(i));
		mali_writel(kbdev->hw_quirks_tiler[i],
			    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__TILER_CONFIG(i));
		mali_writel(kbdev->hw_quirks_mmu[i],
			    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__L2_MMU_CONFIG(i));
		mali_writel(kbdev->hw_quirks_gpu[i],
			    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__CSF_CONFIG(i));
		mali_writel(kbdev->hw_quirks_nx[i],
			    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__NEURAL_CONFIG(i));
	}
}

void kbase_am_cache_set_coherency_mode(struct kbase_device *kbdev)
{
	u32 mode = kbdev->system_coherency;
	u32 val;

	/* Override current mode */
	kbdev->current_gpu_coherency_mode = mode;

	/* AMBA_ENABLE present on SYSTEM subpage */
	val = mali_readl(kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__AMBA_ENABLE);

	val = AMBA_ENABLE_COHERENCY_PROTOCOL_SET(val, mode);
	mali_writel(val, kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__AMBA_ENABLE);
}

void kbase_am_amba_set_shareable_cache_support(struct kbase_device *kbdev)
{
	if (kbdev->system_coherency != COHERENCY_NONE) {
		if (kbdev->am_shareable_cache) {
			u32 val = mali_readl(kbdev->reg_ext[KBASE_REG_EXT_SYS] +
					     AM_SYSTEM__AMBA_ENABLE);
			val = AMBA_ENABLE_SHAREABLE_CACHE_SUPPORT_SET(val, 1);
			mali_writel(val,
				    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__AMBA_ENABLE);
		}
	}
}

void kbase_am_l2_config_override(struct kbase_device *kbdev)
{
	u32 val;

	/*
	 * Skip if it is not supported
	 */
	if (!kbase_hw_has_feature(kbdev, KBASE_HW_FEATURE_L2_CONFIG))
		return;

	if (kbase_hw_has_feature(kbdev, KBASE_HW_FEATURE_PBHA_HWU)) {
		val = mali_readl(kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__L2_CONFIG);
		mali_writel(L2_CONFIG_PBHA_HWU_SET(val, kbdev->pbha_propagate_bits),
			    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__L2_CONFIG);
	}

	/*
	 * Skip if size and hash are not given explicitly,
	 * which means default values are used.
	 */
	if ((kbdev->l2_size_override == 0) && (kbdev->l2_hash_override == 0) &&
	    (!kbdev->l2_hash_values_override))
		return;

	val = mali_readl(kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__L2_CONFIG);

	if (kbdev->l2_size_override) {
		val &= ~L2_CONFIG_SIZE_MASK;
		val |= (kbdev->l2_size_override << L2_CONFIG_SIZE_SHIFT);
	}

	if (kbdev->l2_hash_override) {
		WARN_ON(kbase_hw_has_l2_slice_hash_feature(kbdev));
		val &= ~L2_CONFIG_HASH_MASK;
		val |= (kbdev->l2_hash_override << L2_CONFIG_HASH_SHIFT);
	} else if (kbdev->l2_hash_values_override) {
		uint i;

		WARN_ON(!kbase_hw_has_l2_slice_hash_feature(kbdev));

		val &= ~L2_CONFIG_L2_SLICE_HASH_ENABLE_MASK;
		val |= (0x1 << L2_CONFIG_L2_SLICE_HASH_ENABLE_SHIFT);
		for (i = 0; i < GPU_L2_SLICE_HASH_COUNT; i++) {
			/* L2_SLICE_HASH, L2C_SLICE_HASH and ASN_HASH alias each other */
			dev_dbg(kbdev->dev, "Program 0x%x to L2C_SLICE_HASH[%u]\n",
				kbdev->l2_hash_values[i], i);
			mali_writel(val, kbdev->reg_ext[KBASE_REG_EXT_SYS] +
						 AM_SYSTEM__L2C_SLICE_HASH(i));
		}
	}

	dev_dbg(kbdev->dev, "Program 0x%x to L2_CONFIG\n", val);
	mali_writel(val, kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM__L2_CONFIG);
}

void kbase_am_irq_handler_aw(struct kbase_device *kbdev, u32 val)
{
	if (WARN_ON(!kbdev || !kbdev->am_standalone))
		return;

	KBASE_KTRACE_ADD(kbdev, CORE_WINDOW_IRQ, NULL, val);

	/* Messaging IRQ is not used when kbase works standalone for AM GPUs. */
	if (val & WINDOW_IRQ_MESSAGE)
		WARN(1, "Window IRQ - MESSAGE.");

	/* WINDOW IRQs should not be raised unless RESET is on-going. */
	if (val & WINDOW_IRQ_INVALID_ACCESS)
		if (!WARN(!atomic_read(&kbdev->pm.backend.reset_in_progress),
			  "Window IRQ - INVALID_ACCESS"))
			dev_dbg(kbdev->dev, "Window IRQ - INVALID_ACCESS (during reset)");

	if (val & WINDOW_IRQ_WINDOW_OPENING)
		dev_dbg(kbdev->dev, "Window IRQ - OPENING");

	if (val & WINDOW_IRQ_WINDOW_CLOSED)
		if (!WARN(!atomic_read(&kbdev->pm.backend.reset_in_progress),
			  "Window IRQ - CLOSED"))
			dev_dbg(kbdev->dev, "Window IRQ - CLOSED (during reset)");

	if (val & WINDOW_IRQ_WINDOW_OPENED)
		if (!WARN(!atomic_read(&kbdev->pm.backend.reset_in_progress),
			  "Window IRQ - OPENED"))
			dev_dbg(kbdev->dev, "Window IRQ - OPENED (during reset)");

	/* Just clear every interrupt */
	mali_writel(val, kbdev->reg + AM_AW_WINDOW_IRQ_CLEAR);
}

int kbase_am_power_on(struct kbase_device *kbdev)
{
	int err;
	int val;

	if (WARN_ON(!kbdev || !kbdev->am_standalone))
		return -EINVAL;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* Wait for any pending system resets */
	err = readx_poll_timeout(mali_readl, kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM_STATUS,
				 val, (val == 0), 0, REG_POLL_RESET_TIMEOUT_SYS_US);
	if (err) {
		dev_err(kbdev->dev, "Reset still on-going, failed to resume AM.\n");
		return -EIO;
	}

	/* Clear reset IRQ */
	mali_writel(AM_SYSTEM_IRQ_RESET_COMPLETED,
		    kbdev->reg_ext[KBASE_REG_EXT_SYS] + AM_SYSTEM_IRQ_CLEAR);

	/* Configuration of AM-SYSTEM must be deferred until after device init due to HW
	 * race condition
	 */
	if (kbase_hw_has_issue(kbdev, KBASE_HW_ISSUE_MAGNIHW_2434) && kbdev->device_inited) {
		kbase_am_hw_issues_apply(kbdev);
		kbase_am_cache_set_coherency_mode(kbdev);
		kbase_am_amba_set_shareable_cache_support(kbdev);
		kbase_am_l2_config_override(kbdev);
	}

	/* Assign AW0 to partition */
	err = kbasep_am_assign_aw0(kbdev);
	if (err)
		return err;

	/* Clear and enable Access-window IRQs */
	mali_writel(IRQ_MESSAGE_ALL_MASK, kbdev->reg + AM_AW_WINDOW_IRQ_CLEAR);
	mali_writel(IRQ_MESSAGE_ALL_MASK, kbdev->reg + AM_AW_WINDOW_IRQ_MASK);

	return 0;
}

void kbase_am_power_off(struct kbase_device *kbdev)
{
	if (WARN_ON(!kbdev || !kbdev->am_standalone))
		return;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* Disable Access-window IRQs */
	mali_writel(0x0, kbdev->reg + AM_AW_WINDOW_IRQ_MASK);

	/* Unassign partition */
	kbasep_am_unassign(kbdev);
}
