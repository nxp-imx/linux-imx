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

#include <linux/io.h>
#include <linux/types.h>
#include <linux/mali_hw_access.h>

#include <mali_kbase.h>

#include "mali_kbase_am_reg.h"
#include "mali_kbase_am_reg_ipa.h"

u32 kbase_am_reg_read32(struct kbase_device *kbdev, enum kbase_reg_ext_type reg_type,
			u32 reg_offset)
{
	void __iomem *iomem_addr;
	u32 val;

	if (WARN_ON(!kbdev || !kbdev->am_standalone))
		return 0;
	if (WARN_ON(!kbase_io_is_am_powered(kbdev)))
		return 0;

	iomem_addr = kbdev->reg_ext[reg_type] + reg_offset;

	val = mali_readl(iomem_addr);

#if defined(CONFIG_DEBUG_FS) && !IS_ENABLED(CONFIG_MALI_NO_MALI)
	if (unlikely(kbdev->io_history.enabled))
		kbase_io_history_add(&kbdev->io_history, iomem_addr, val, 0);
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "r32: type %d reg %08x val %08x", reg_type, reg_offset, val);

	return val;
}

u64 kbase_am_reg_read64(struct kbase_device *kbdev, enum kbase_reg_ext_type reg_type,
			u32 reg_offset)
{
	void __iomem *iomem_addr;
	u64 val;

	if (WARN_ON(!kbdev || !kbdev->am_standalone))
		return 0;
	if (WARN_ON(!kbase_io_is_am_powered(kbdev)))
		return 0;

	iomem_addr = kbdev->reg_ext[reg_type] + reg_offset;

	val = mali_readq(iomem_addr);

#if defined(CONFIG_DEBUG_FS) && !IS_ENABLED(CONFIG_MALI_NO_MALI)
	if (unlikely(kbdev->io_history.enabled)) {
		kbase_io_history_add(&kbdev->io_history, iomem_addr, (u32)val, 0);
		kbase_io_history_add(&kbdev->io_history, iomem_addr + 4, (u32)(val >> 32), 0);
	}
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "r64: type %d reg %08x val %016llx", reg_type, reg_offset, val);

	return val;
}

void kbase_am_reg_write32(struct kbase_device *kbdev, enum kbase_reg_ext_type reg_type,
			  u32 reg_offset, u32 value)
{
	void __iomem *iomem_addr;

	if (WARN_ON(!kbdev || !kbdev->am_standalone))
		return;
	if (WARN_ON(!kbase_io_is_am_powered(kbdev)))
		return;

	iomem_addr = kbdev->reg_ext[reg_type] + reg_offset;

	mali_writel(value, iomem_addr);

#if defined(CONFIG_DEBUG_FS) && !IS_ENABLED(CONFIG_MALI_NO_MALI)
	if (unlikely(kbdev->io_history.enabled))
		kbase_io_history_add(&kbdev->io_history, iomem_addr, value, 1);
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "w32: type %d reg %08x val %08x", reg_type, reg_offset, value);
}

void kbase_am_reg_write64(struct kbase_device *kbdev, enum kbase_reg_ext_type reg_type,
			  u32 reg_offset, u64 value)
{
	void __iomem *iomem_addr;

	if (WARN_ON(!kbdev || !kbdev->am_standalone))
		return;
	if (WARN_ON(!kbase_io_is_am_powered(kbdev)))
		return;

	iomem_addr = kbdev->reg_ext[reg_type] + reg_offset;

	mali_writeq(value, iomem_addr);

#if defined(CONFIG_DEBUG_FS) && !IS_ENABLED(CONFIG_MALI_NO_MALI)
	if (unlikely(kbdev->io_history.enabled)) {
		kbase_io_history_add(&kbdev->io_history, iomem_addr, (u32)value, 1);
		kbase_io_history_add(&kbdev->io_history, iomem_addr + 4, (u32)(value >> 32), 1);
	}
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "w64: type %d reg %08x val %016llx", reg_type, reg_offset, value);
}
KBASE_EXPORT_TEST_API(kbase_am_reg_write64);

/* Register offsets on GOVERNOR submodule */
static const u32 gov_ipa_reg_offsets[GOV_IPA_REG_SIZE + GOV_IPA_REG_SIZE_NX] = {
	[GOV_IPA_REG__COMMAND] = 0x1000, /* 32-bit */
	[GOV_IPA_REG__STATUS] = 0x1004, /* 32-bit */
	[GOV_IPA_REG__TIMER] = 0x1008, /* 32-bit */
	[GOV_IPA_REG__SELECT_CSHW] = 0x1010, /* 64-bit */
	[GOV_IPA_REG__SELECT_MEMSYS] = 0x1018, /* 64-bit */
	[GOV_IPA_REG__SELECT_TILER] = 0x1020, /* 64-bit */
	[GOV_IPA_REG__SELECT_SHADER] = 0x1028, /* 64-bit */
	[GOV_IPA_REG__SELECT_NEURAL] = 0x1030, /* 64-bit */
	[GOV_IPA_REG__VALUE_CSHW_0] = 0x1100, /* 64-bit */
	[GOV_IPA_REG__VALUE_CSHW_1] = 0x1108, /* 64-bit */
	[GOV_IPA_REG__VALUE_CSHW_2] = 0x1110, /* 64-bit */
	[GOV_IPA_REG__VALUE_CSHW_3] = 0x1118, /* 64-bit */
	[GOV_IPA_REG__VALUE_CSHW_4] = 0x1120, /* 64-bit */
	[GOV_IPA_REG__VALUE_CSHW_5] = 0x1128, /* 64-bit */
	[GOV_IPA_REG__VALUE_CSHW_6] = 0x1130, /* 64-bit */
	[GOV_IPA_REG__VALUE_CSHW_7] = 0x1138, /* 64-bit */
	[GOV_IPA_REG__VALUE_MEMSYS_0] = 0x1140, /* 64-bit */
	[GOV_IPA_REG__VALUE_MEMSYS_1] = 0x1148, /* 64-bit */
	[GOV_IPA_REG__VALUE_MEMSYS_2] = 0x1150, /* 64-bit */
	[GOV_IPA_REG__VALUE_MEMSYS_3] = 0x1158, /* 64-bit */
	[GOV_IPA_REG__VALUE_MEMSYS_4] = 0x1160, /* 64-bit */
	[GOV_IPA_REG__VALUE_MEMSYS_5] = 0x1168, /* 64-bit */
	[GOV_IPA_REG__VALUE_MEMSYS_6] = 0x1170, /* 64-bit */
	[GOV_IPA_REG__VALUE_MEMSYS_7] = 0x1178, /* 64-bit */
	[GOV_IPA_REG__VALUE_TILER_0] = 0x1180, /* 64-bit */
	[GOV_IPA_REG__VALUE_TILER_1] = 0x1188, /* 64-bit */
	[GOV_IPA_REG__VALUE_TILER_2] = 0x1190, /* 64-bit */
	[GOV_IPA_REG__VALUE_TILER_3] = 0x1198, /* 64-bit */
	[GOV_IPA_REG__VALUE_TILER_4] = 0x11a0, /* 64-bit */
	[GOV_IPA_REG__VALUE_TILER_5] = 0x11a8, /* 64-bit */
	[GOV_IPA_REG__VALUE_TILER_6] = 0x11b0, /* 64-bit */
	[GOV_IPA_REG__VALUE_TILER_7] = 0x11b8, /* 64-bit */
	[GOV_IPA_REG__VALUE_SHADER_0] = 0x11c0, /* 64-bit */
	[GOV_IPA_REG__VALUE_SHADER_1] = 0x11c8, /* 64-bit */
	[GOV_IPA_REG__VALUE_SHADER_2] = 0x11d0, /* 64-bit */
	[GOV_IPA_REG__VALUE_SHADER_3] = 0x11d8, /* 64-bit */
	[GOV_IPA_REG__VALUE_SHADER_4] = 0x11e0, /* 64-bit */
	[GOV_IPA_REG__VALUE_SHADER_5] = 0x11e8, /* 64-bit */
	[GOV_IPA_REG__VALUE_SHADER_6] = 0x11f0, /* 64-bit */
	[GOV_IPA_REG__VALUE_SHADER_7] = 0x11f8, /* 64-bit */
	[GOV_IPA_REG__VALUE_NEURAL_0] = 0x1200, /* 64-bit */
	[GOV_IPA_REG__VALUE_NEURAL_1] = 0x1208, /* 64-bit */
	[GOV_IPA_REG__VALUE_NEURAL_2] = 0x1210, /* 64-bit */
	[GOV_IPA_REG__VALUE_NEURAL_3] = 0x1218, /* 64-bit */
	[GOV_IPA_REG__VALUE_NEURAL_4] = 0x1220, /* 64-bit */
	[GOV_IPA_REG__VALUE_NEURAL_5] = 0x1228, /* 64-bit */
	[GOV_IPA_REG__VALUE_NEURAL_6] = 0x1230, /* 64-bit */
	[GOV_IPA_REG__VALUE_NEURAL_7] = 0x1238, /* 64-bit */
};

static inline u32 get_ipa_reg_offset(u32 reg_enum)
{
	const u32 reg_arr_size = GOV_IPA_REG_SIZE + GOV_IPA_REG_SIZE_NX;
	u32 reg_arr_index;

	if (reg_enum < GOV_IPA_REG_BASE_NX)
		reg_arr_index = reg_enum - GOV_IPA_REG_BASE;
	else
		reg_arr_index = reg_enum - GOV_IPA_REG_BASE_NX + GOV_IPA_REG_SIZE;

	if (reg_arr_index >= reg_arr_size)
		return 0;

	return gov_ipa_reg_offsets[reg_arr_index];
}

u32 kbase_am_reg_read32_ipa(struct kbase_device *kbdev, u32 reg_enum)
{
	u32 reg_offset = get_ipa_reg_offset(reg_enum);

	if (WARN_ON(!reg_offset))
		return 0;

	return kbase_am_reg_read32(kbdev, KBASE_REG_EXT_GOV, reg_offset);
}

u64 kbase_am_reg_read64_ipa(struct kbase_device *kbdev, u32 reg_enum)
{
	u32 reg_offset = get_ipa_reg_offset(reg_enum);

	if (WARN_ON(!reg_offset))
		return 0;

	return kbase_am_reg_read64(kbdev, KBASE_REG_EXT_GOV, reg_offset);
}

void kbase_am_reg_write32_ipa(struct kbase_device *kbdev, u32 reg_enum, u32 value)
{
	u32 reg_offset = get_ipa_reg_offset(reg_enum);

	if (WARN_ON(!reg_offset))
		return;

	kbase_am_reg_write32(kbdev, KBASE_REG_EXT_GOV, reg_offset, value);
}

void kbase_am_reg_write64_ipa(struct kbase_device *kbdev, u32 reg_enum, u64 value)
{
	u32 reg_offset = get_ipa_reg_offset(reg_enum);

	if (WARN_ON(!reg_offset))
		return;

	kbase_am_reg_write64(kbdev, KBASE_REG_EXT_GOV, reg_offset, value);
}
