// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - vpu control device
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/genalloc.h>
#include <linux/thermal.h>
#include <linux/units.h>
#include <linux/pm_opp.h>
#include <linux/freezer.h>
#include <linux/vmalloc.h>

#include "wave5-vpuconfig.h"
#include "wave5-regdefine.h"
#include "wave5-vdi.h"
#include "wave5-vpu-ctrl.h"

#define VPU_CTRL_PLATFORM_DEVICE_NAME "wave5-vpu-ctrl"

static unsigned int debug;
module_param(debug, uint, 0644);

#define call_read_reg(entity, args...)					\
	((entity)->read_reg ? (entity)->read_reg((entity)->dev, ##args) : 0)

#define call_void_op(entity, op, args...)				\
	do {								\
		if ((entity)->op)					\
			(entity)->op((entity)->dev, ##args);		\
	} while (0)

#define dprintk(dev, fmt, arg...)					\
	do {								\
		if (debug)						\
			dev_info(dev, fmt, ## arg);			\
	} while (0)

#define wave5_wait_event_freezable_timeout(wq_head, condition, timeout)		\
({										\
	int wave5_wait_ret = 0;							\
	unsigned long _timeout = timeout;					\
	unsigned long stop;							\
	stop = jiffies + _timeout;						\
	do {									\
		if (wave5_wait_ret == -ERESTARTSYS && freezing(current))	\
			clear_thread_flag(TIF_SIGPENDING);			\
		_timeout = stop - jiffies;					\
		if ((long)_timeout <= 0) {					\
			wave5_wait_ret = -ERESTARTSYS;				\
			break;							\
		}								\
		wave5_wait_ret = wait_event_freezable_timeout(wq_head, condition, _timeout);	\
	} while (wave5_wait_ret == -ERESTARTSYS && freezing(current));		\
	wave5_wait_ret;								\
})

struct vpu_ctrl_resource {
	const char *fw_name;
	u32 sram_size;
};

#define WAVE5_ENABLE_SW_UART		1
#if WAVE5_ENABLE_SW_UART
#include <linux/debugfs.h>
#define W5_NXP_SW_UART_LOGER                         (W5_REG_BASE + 0x00f0)
#define TRACEBUF_SIZE                                SZ_64K

static unsigned int enable_fwlog;
module_param(enable_fwlog, uint, 0644);

struct loger_t {
	u32 size;
	u32 wptr;
	u32 rptr;
	u32 anchor;
	u32 count;
	u32 reserved[3];
	char vbase[];
};
#endif

struct vpu_ctrl {
	struct device *dev;
	void __iomem *reg_base;
	struct clk_bulk_data *clks;
	int num_clks;
	struct vpu_buf boot_mem;
	u32 state;
	struct mutex ctrl_lock; /* the lock for vpu control device */
	struct wave5_vpu_entity *current_entity;
	struct list_head entities;
	const struct vpu_ctrl_resource *res;
	struct gen_pool *sram_pool;
	struct vpu_buf sram_buf;
	struct vpu_buf buffers[MAX_NUM_INSTANCE];
	u32 acquired_buffer_count;
	u32 required_buffer_count;
	bool support_follower;
	wait_queue_head_t load_fw_wq;
#if WAVE5_ENABLE_SW_UART
	struct vpu_buf loger_buf;
	struct loger_t *loger;
	struct dentry *debugfs;
#endif
};

static const struct vpu_ctrl_resource nxp_wave511_ctrl_data = {
	.fw_name = "cnm/wave511_dec_fw.bin",
	/* For AVC/HEVC, 4096x2304, 8bit */
	.sram_size = (72 * 1024),  //0x12000
};

#if WAVE5_ENABLE_SW_UART
static void wave5_vpu_ctrl_init_loger(struct vpu_ctrl *ctrl)
{
	ctrl->loger_buf.size = TRACEBUF_SIZE;
	ctrl->loger_buf.vaddr = dma_alloc_coherent(ctrl->dev,
						   ctrl->loger_buf.size,
						   &ctrl->loger_buf.daddr,
						   GFP_KERNEL);
	if (!ctrl->loger_buf.vaddr) {
		ctrl->loger_buf.size = 0;
		return;
	}
	ctrl->loger = ctrl->loger_buf.vaddr;
	ctrl->loger->size = TRACEBUF_SIZE - sizeof(struct loger_t);
	dev_info(ctrl->dev, "sw uart at %pad, 0x%zx\n",
		 &ctrl->loger_buf.daddr, ctrl->loger_buf.size);
}

static void wave5_vpu_ctrl_free_loger(struct vpu_ctrl *ctrl)
{
	ctrl->loger = NULL;
	if (ctrl->loger_buf.vaddr)
		dma_free_coherent(ctrl->dev,
				  ctrl->loger_buf.size,
				  ctrl->loger_buf.vaddr,
				  ctrl->loger_buf.daddr);

	memset(&ctrl->loger_buf, 0, sizeof(ctrl->loger_buf));
}

static void wave5_vpu_ctrl_start_loger(struct vpu_ctrl *ctrl, struct wave5_vpu_entity *entity)
{
	if (enable_fwlog)
		call_void_op(entity, write_reg, W5_NXP_SW_UART_LOGER, (u32)ctrl->loger_buf.daddr);
}

static void wave5_vpu_ctrl_stop_loger(struct vpu_ctrl *ctrl, struct wave5_vpu_entity *entity)
{
	call_void_op(entity, write_reg, W5_NXP_SW_UART_LOGER, 0);
}

static int wave5_vpu_loger_show(struct seq_file *s, void *data)
{
	struct vpu_ctrl *ctrl = s->private;
	u32 rptr;
	u32 wptr;
	int length;

	if (!ctrl->loger)
		return 0;

	rptr = ctrl->loger->rptr;
	wptr = ctrl->loger->wptr;

	if (rptr == wptr)
		return 0;

	if (rptr < wptr)
		length = wptr - rptr;
	else
		length = ctrl->loger->size - rptr;

	if (s->count + length >= s->size) {
		s->count = s->size;
		return 0;
	}

	if (!seq_write(s, ctrl->loger->vbase + rptr, length)) {
		rptr += length;
		if (rptr == ctrl->loger->size)
			rptr = 0;
		ctrl->loger->rptr = rptr;
	}

	return 0;
}

static int wave5_vpu_loger_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, wave5_vpu_loger_show, inode->i_private);
}

static const struct file_operations wave5_vpu_loger_fops = {
	.owner = THIS_MODULE,
	.open = wave5_vpu_loger_open,
	.release = single_release,
	.read = seq_read,
};

static void wave5_vpu_ctrl_create_debugfs(struct vpu_ctrl *ctrl)
{
	struct dentry *wave5_dbgfs = debugfs_lookup("wave5", NULL);

	if (!wave5_dbgfs)
		wave5_dbgfs = debugfs_create_dir("wave5", NULL);
	if (!wave5_dbgfs)
		return;

	ctrl->debugfs = debugfs_create_file("fwlog",
					    VERIFY_OCTAL_PERMISSIONS(0444),
					    wave5_dbgfs,
					    ctrl,
					    &wave5_vpu_loger_fops);
}

static void wave5_vpu_ctrl_remove_debugfs(struct vpu_ctrl *ctrl)
{
	if (!ctrl->debugfs)
		return;

	debugfs_remove(ctrl->debugfs);
	ctrl->debugfs = NULL;
}
#endif

static void wave5_vpu_ctrl_writel(struct device *dev, u32 addr, u32 data)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	writel(data, ctrl->reg_base + addr);
}

static void byte_swap(unsigned char *data, int len)
{
	u8 temp;
	int i;

	for (i = 0; i < len; i += 2) {
		temp = data[i];
		data[i] = data[i + 1];
		data[i + 1] = temp;
	}
}

static void word_swap(unsigned char *data, int len)
{
	u16 temp;
	u16 *ptr = (u16 *)data;
	int i;
	s32 size = len / sizeof(uint16_t);

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

static void dword_swap(unsigned char *data, int len)
{
	u32 temp;
	u32 *ptr = (u32 *)data;
	s32 size = len / sizeof(uint32_t);
	int i;

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

static void lword_swap(unsigned char *data, int len)
{
	u64 temp;
	u64 *ptr = (u64 *)data;
	s32 size = len / sizeof(uint64_t);
	int i;

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

int wave5_convert_endian(unsigned int endian)
{
	switch (endian) {
	case VDI_LITTLE_ENDIAN:
		endian = 0x00;
		break;
	case VDI_BIG_ENDIAN:
		endian = 0x0f;
		break;
	case VDI_32BIT_LITTLE_ENDIAN:
		endian = 0x04;
		break;
	case VDI_32BIT_BIG_ENDIAN:
		endian = 0x03;
		break;
	}

	return (endian & 0x0f);
}
EXPORT_SYMBOL_GPL(wave5_convert_endian);

void wave5_swap_endian(u8 *data, int len, int endian)
{
	int changes;
	int sys_endian;
	bool byte_change, word_change, dword_change, lword_change;

	sys_endian = VDI_128BIT_LITTLE_ENDIAN;

	endian = wave5_convert_endian(endian);
	sys_endian = wave5_convert_endian(sys_endian);
	if (endian == sys_endian)
		return;

	changes = endian ^ sys_endian;
	byte_change = changes & 0x01;
	word_change = ((changes & 0x02) == 0x02);
	dword_change = ((changes & 0x04) == 0x04);
	lword_change = ((changes & 0x08) == 0x08);

	if (byte_change)
		byte_swap(data, len);
	if (word_change)
		word_swap(data, len);
	if (dword_change)
		dword_swap(data, len);
	if (lword_change)
		lword_swap(data, len);
}
EXPORT_SYMBOL_GPL(wave5_swap_endian);

static const char *wave5_vpu_ctrl_state_name(u32 state)
{
	switch (state) {
	case WAVE5_VPU_STATE_OFF:
		return "off";
	case WAVE5_VPU_STATE_PREPARE:
		return "prepare";
	case WAVE5_VPU_STATE_ON:
		return "on";
	case WAVE5_VPU_STATE_SLEEP:
		return "sleep";
	default:
		return "unknown";
	}
}

static void wave5_vpu_ctrl_set_state(struct vpu_ctrl *ctrl, u32 state)
{
	dprintk(ctrl->dev, "set state: %s -> %s\n",
		wave5_vpu_ctrl_state_name(ctrl->state), wave5_vpu_ctrl_state_name(state));
	ctrl->state = state;
}

static int wave5_vpu_ctrl_wait_busy(struct wave5_vpu_entity *entity, unsigned int addr)
{
	u32 val;

	if (!entity || !entity->read_reg)
		return -EINVAL;

	return read_poll_timeout(entity->read_reg, val, val == 0,
				 VPU_POLL_CHECK_INTERVAL, VPU_BUSY_CHECK_TIMEOUT, false,
				 entity->dev, addr);
}

static int wave5_vpu_ctrl_check_result(struct wave5_vpu_entity *entity)
{
	if (!entity)
		return -EINVAL;

	if (call_read_reg(entity, W5_RET_SUCCESS))
		return 0;

	return call_read_reg(entity, W5_RET_FAIL_REASON);
}

static u32 wave5_vpu_ctrl_get_code_buf_size(struct vpu_ctrl *ctrl)
{
	return min_t(u32, ctrl->boot_mem.size, WAVE521_MAX_CODE_BUF_SIZE);
}

static void wave5_vpu_ctrl_remap_code_buffer(struct vpu_ctrl *ctrl)
{
	dma_addr_t code_base = ctrl->boot_mem.daddr;
	u32 i, reg_val, remap_size;

	for (i = 0; i < wave5_vpu_ctrl_get_code_buf_size(ctrl) / W5_REMAP_MAX_SIZE; i++) {
		remap_size = (W5_REMAP_MAX_SIZE >> 12) & 0x1ff;
		reg_val = 0x80000000 |
			  (WAVE5_UPPER_PROC_AXI_ID << 20) |
			  (0 << 16) |
			  (i << 12) |
			  BIT(11) |
			  remap_size;
		wave5_vpu_ctrl_writel(ctrl->dev, W5_VPU_REMAP_CTRL_GB, reg_val);
		wave5_vpu_ctrl_writel(ctrl->dev, W5_VPU_REMAP_VADDR_GB, i * W5_REMAP_MAX_SIZE);
		wave5_vpu_ctrl_writel(ctrl->dev, W5_VPU_REMAP_PADDR_GB,
				      code_base + i * W5_REMAP_MAX_SIZE);
	}
}

static int wave5_vpu_ctrl_init_vpu(struct vpu_ctrl *ctrl)
{
	struct wave5_vpu_entity *entity = ctrl->current_entity;
	int ret;

	dprintk(ctrl->dev, "cold boot vpu\n");

	if (!ctrl->current_entity)
		return -EINVAL;

	wave5_vpu_ctrl_remap_code_buffer(ctrl);

	call_void_op(entity, write_reg, W5_VPU_BUSY_STATUS, 1);
	call_void_op(entity, write_reg, W5_VPU_PO_CONF, 0);
	call_void_op(entity, write_reg, W5_CMD_INIT_ADDR_CODE_BASE, ctrl->boot_mem.daddr);
	call_void_op(entity, write_reg, W5_CMD_INIT_CODE_SIZE, WAVE521_MAX_CODE_BUF_SIZE);
	call_void_op(entity, write_reg, W5_CMD_INIT_PARAM, (WAVE5_UPPER_PROC_AXI_ID << 4) | 0);
	call_void_op(entity, write_reg, W5_CMD_INIT_HW_OPTION, 0);
	call_void_op(entity, write_reg, W5_CMD_INIT_SEC_AXI_ADDR, ctrl->sram_buf.daddr);
	call_void_op(entity, write_reg, W5_CMD_INIT_SEC_AXI_SIZE, ctrl->sram_buf.size);

	wave5_vpu_ctrl_writel(ctrl->dev, W5_COMMAND_GB, W5_INIT_VPU);
	wave5_vpu_ctrl_writel(ctrl->dev, W5_VPU_REMAP_CORE_START_GB, 1);

	ret = wave5_vpu_ctrl_wait_busy(entity, W5_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(ctrl->dev, "init vpu timeout\n");
		return -EINVAL;
	}

	ret = wave5_vpu_ctrl_check_result(entity);
	if (ret) {
		dev_err(ctrl->dev, "init vpu fail, reason 0x%x\n", ret);
		return -EIO;
	}

	return 0;
}

static void wave5_vpu_ctrl_on_boot(struct wave5_vpu_entity *entity)
{
	if (!entity->on_boot)
		return;

	if (!entity->booted) {
		call_void_op(entity, on_boot);
		entity->booted = true;
	}
}

static void wave5_vpu_ctrl_clear_firmware_buffers(struct vpu_ctrl *ctrl,
						  struct wave5_vpu_entity *entity)
{
	int ret;

	dprintk(ctrl->dev, "clear firmware work buffers\n");

	call_void_op(entity, write_reg, W5_VPU_BUSY_STATUS, 1);
	call_void_op(entity, write_reg, W5_COMMAND, W5_INIT_WORK_BUF);
	call_void_op(entity, write_reg, W5_VPU_HOST_INT_REQ, 1);

	ret = wave5_vpu_ctrl_wait_busy(entity, W5_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(ctrl->dev, "set buffer failed\n");
		return;
	}

	ret = wave5_vpu_ctrl_check_result(entity);
	if (ret) {
		dev_err(ctrl->dev, "set buffer failed, reason 0x%x\n", ret);
		return;
	}
}

static void wave5_vpu_ctrl_acquire_buffers(struct vpu_ctrl *ctrl)
{
	struct vpu_buf *buf;
	int i;

	for (i = 0; i < MAX_NUM_INSTANCE; i++) {
		buf = &ctrl->buffers[i];
		buf->size = WAVE517_WORKBUF_SIZE;
		if (wave5_vdi_allocate_dma_memory(ctrl->dev, buf))
			return;

		ctrl->acquired_buffer_count++;
	}
}

static void wave5_vpu_ctrl_free_buffers(struct vpu_ctrl *ctrl)
{
	int i;

	for (i = 0; i < ctrl->acquired_buffer_count; i++)
		wave5_vdi_free_dma_memory(&ctrl->buffers[i]);

	ctrl->acquired_buffer_count = 0;
}

int wave5_vpu_ctrl_require_buffer(struct device *dev, struct wave5_vpu_entity *entity)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);
	struct vpu_buf *pbuf;
	u32 size;
	int ret = -ENOMEM;

	if (!ctrl || !entity)
		return -EINVAL;

	size = call_read_reg(entity, W5_CMD_SET_CTRL_WORK_BUF_SIZE);
	if (!size)
		return 0;

	if (size > WAVE517_WORKBUF_SIZE)
		goto exit;

	if (ctrl->required_buffer_count >= ctrl->acquired_buffer_count)
		goto exit;

	pbuf = &ctrl->buffers[ctrl->required_buffer_count++];
	call_void_op(entity, write_reg, W5_CMD_SET_CTRL_WORK_BUF_ADDR, pbuf->daddr);
	ret = 0;
exit:
	call_void_op(entity, write_reg, W5_CMD_SET_CTRL_WORK_BUF_SIZE, 0);
	return ret;
}
EXPORT_SYMBOL_GPL(wave5_vpu_ctrl_require_buffer);

static void wave5_vpu_ctrl_clear_buffers(struct vpu_ctrl *ctrl)
{
	struct wave5_vpu_entity *entity;

	dprintk(ctrl->dev, "clear all buffers\n");

	entity = list_first_entry_or_null(&ctrl->entities,
					  struct wave5_vpu_entity, list);
	if (entity)
		wave5_vpu_ctrl_clear_firmware_buffers(ctrl, entity);

	ctrl->required_buffer_count = 0;
}

static void wave5_vpu_ctrl_boot_done(struct vpu_ctrl *ctrl, int wakeup)
{
	struct wave5_vpu_entity *entity;

	if (ctrl->state == WAVE5_VPU_STATE_ON)
		return;

	if (!wakeup)
		wave5_vpu_ctrl_clear_buffers(ctrl);

	list_for_each_entry(entity, &ctrl->entities, list)
		wave5_vpu_ctrl_on_boot(entity);

	dprintk(ctrl->dev, "boot done from %s\n", wakeup ? "wakeup" : "cold boot");

	wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_ON);
}

static bool wave5_vpu_ctrl_find_entity(struct vpu_ctrl *ctrl, struct wave5_vpu_entity *entity)
{
	struct wave5_vpu_entity *tmp;

	list_for_each_entry(tmp, &ctrl->entities, list) {
		if (tmp == entity)
			return true;
	}

	return false;
}

static void wave5_vpu_ctrl_load_firmware(const struct firmware *fw, void *context)
{
	struct vpu_ctrl *ctrl = context;
	struct wave5_vpu_entity *entity = ctrl->current_entity;
	u32 product_code;
	int ret;

	if (!ctrl->current_entity) {
		dev_err(ctrl->dev, "No vpu core.\n");
		release_firmware(fw);
		return;
	}

	ret = pm_runtime_resume_and_get(ctrl->dev);
	if (ret) {
		dev_err(ctrl->dev, "pm runtime resume fail, ret = %d\n", ret);
		scoped_guard(mutex, &ctrl->ctrl_lock) {
			wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_OFF);
			ctrl->current_entity = NULL;
		}
		release_firmware(fw);
		return;
	}

	if (!fw || !fw->data) {
		dev_err(ctrl->dev, "No firmware.\n");
		ret = -EINVAL;
		goto exit;
	}

	if (!ctrl->boot_mem.vaddr || !ctrl->boot_mem.daddr || !ctrl->boot_mem.size) {
		dev_err(ctrl->dev, "No boot memory.\n");
		ret = -EINVAL;
		goto exit;
	}

	if (fw->size + WAVE5_EXTRA_CODE_BUF_SIZE > wave5_vpu_ctrl_get_code_buf_size(ctrl)) {
		dev_err(ctrl->dev, "firmware size (%ld > %zd) is too big\n",
			fw->size, ctrl->boot_mem.size);
		ret = -EINVAL;
		goto exit;
	}

	product_code = call_read_reg(entity, W5_VPU_RET_PRODUCT_VERSION);
	if (!PRODUCT_CODE_W_SERIES(product_code)) {
		dev_err(ctrl->dev, "unknown product id : %08x\n", product_code);
		ret = -EINVAL;
		goto exit;
	}

	wave5_swap_endian((u8 *)fw->data, fw->size, VDI_128BIT_LITTLE_ENDIAN);
	memcpy(ctrl->boot_mem.vaddr, fw->data, fw->size);

exit:
	scoped_guard(mutex, &ctrl->ctrl_lock) {
		if (!ret && wave5_vpu_ctrl_find_entity(ctrl, ctrl->current_entity))
			ret = wave5_vpu_ctrl_init_vpu(ctrl);
		else
			ret = -EINVAL;
	}

	pm_runtime_put_sync(ctrl->dev);
	release_firmware(fw);

	scoped_guard(mutex, &ctrl->ctrl_lock) {
		if (ret)
			wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_OFF);
		else
			wave5_vpu_ctrl_boot_done(ctrl, 0);
		if (ctrl->state == WAVE5_VPU_STATE_ON && ctrl->current_entity)
			call_void_op(ctrl->current_entity, scan_instances);
		ctrl->current_entity = NULL;
	}

	wake_up_interruptible_all(&ctrl->load_fw_wq);
}

static int wave5_vpu_ctrl_sleep(struct vpu_ctrl *ctrl, struct wave5_vpu_entity *entity)
{
	int ret;

	dprintk(ctrl->dev, "sleep firmware\n");

	call_void_op(entity, write_reg, W5_VPU_BUSY_STATUS, 1);
	call_void_op(entity, write_reg, W5_COMMAND, W5_SLEEP_VPU);
	call_void_op(entity, write_reg, W5_VPU_HOST_INT_REQ, 1);

	ret = wave5_vpu_ctrl_wait_busy(entity, W5_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(ctrl->dev, "sleep vpu timeout\n");
		wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_OFF);
		return -EINVAL;
	}

	ret = wave5_vpu_ctrl_check_result(entity);
	if (ret) {
		dev_err(ctrl->dev, "sleep vpu fail, reason 0x%x\n", ret);
		wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_OFF);
		return -EIO;
	}

	wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_SLEEP);

	return 0;
}

static int wave5_vpu_ctrl_wakeup(struct vpu_ctrl *ctrl, struct wave5_vpu_entity *entity)
{
	int ret;

	dprintk(ctrl->dev, "wakeup firmware\n");

	wave5_vpu_ctrl_remap_code_buffer(ctrl);

	call_void_op(entity, write_reg, W5_VPU_BUSY_STATUS, 1);
	call_void_op(entity, write_reg, W5_VPU_PO_CONF, 0);
	call_void_op(entity, write_reg, W5_CMD_INIT_ADDR_CODE_BASE, ctrl->boot_mem.daddr);
	call_void_op(entity, write_reg, W5_CMD_INIT_CODE_SIZE, WAVE521_MAX_CODE_BUF_SIZE);
	call_void_op(entity, write_reg, W5_CMD_INIT_PARAM, (WAVE5_UPPER_PROC_AXI_ID << 4) | 0);
	call_void_op(entity, write_reg, W5_CMD_INIT_HW_OPTION, 0);
	call_void_op(entity, write_reg, W5_CMD_INIT_SEC_AXI_ADDR, ctrl->sram_buf.daddr);
	call_void_op(entity, write_reg, W5_CMD_INIT_SEC_AXI_SIZE, ctrl->sram_buf.size);

	wave5_vpu_ctrl_writel(ctrl->dev, W5_COMMAND_GB, W5_WAKEUP_VPU);
	wave5_vpu_ctrl_writel(ctrl->dev, W5_VPU_REMAP_CORE_START_GB, 1);

	ret = wave5_vpu_ctrl_wait_busy(entity, W5_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(ctrl->dev, "wakeup vpu timeout\n");
		return -EINVAL;
	}

	ret = wave5_vpu_ctrl_check_result(entity);
	if (ret) {
		dev_err(ctrl->dev, "wakeup vpu fail, reason 0x%x\n", ret);
		return -EIO;
	}

	wave5_vpu_ctrl_boot_done(ctrl, 1);

	return 0;
}

static int wave5_vpu_ctrl_try_boot(struct vpu_ctrl *ctrl, struct wave5_vpu_entity *entity)
{
	int ret;

	if (ctrl->state != WAVE5_VPU_STATE_OFF && ctrl->state != WAVE5_VPU_STATE_SLEEP)
		return 0;

	if (call_read_reg(entity, W5_VCPU_CUR_PC)) {
		dprintk(ctrl->dev, "try boot directly as firmware is running\n");
		wave5_vpu_ctrl_boot_done(ctrl, ctrl->state == WAVE5_VPU_STATE_SLEEP);
		return 0;
	}

	if (ctrl->state == WAVE5_VPU_STATE_SLEEP) {
		ret = wave5_vpu_ctrl_wakeup(ctrl, entity);
		return ret;
	}

	wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_PREPARE);
	ctrl->current_entity = entity;
	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_UEVENT,
				      ctrl->res->fw_name,
				      ctrl->dev, GFP_KERNEL,
				      ctrl,
				      wave5_vpu_ctrl_load_firmware);
	if (ret) {
		dev_err(ctrl->dev, "request firmware %s fail, ret = %d\n", ctrl->res->fw_name, ret);
		wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_OFF);
		return ret;
	}

	return 0;
}

bool wave5_vpu_ctrl_support_follower(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	if (!ctrl)
		return false;

	return ctrl->support_follower;
}
EXPORT_SYMBOL_GPL(wave5_vpu_ctrl_support_follower);

int wave5_vpu_ctrl_resume_and_get(struct device *dev, struct wave5_vpu_entity *entity)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);
	bool boot_flag;
	int ret = 0;

	if (!ctrl)
		return -EINVAL;

	if (!entity || !entity->dev || !entity->read_reg || !entity->write_reg)
		return -EINVAL;

	guard(mutex)(&ctrl->ctrl_lock);

	ret = pm_runtime_resume_and_get(ctrl->dev);
	if (ret) {
		dev_err(dev, "pm runtime resume fail, ret = %d\n", ret);
		return ret;
	}

#if WAVE5_ENABLE_SW_UART
	wave5_vpu_ctrl_start_loger(ctrl, entity);
#endif

	entity->booted = false;

	if (ctrl->current_entity)
		boot_flag = false;
	else
		boot_flag = list_empty(&ctrl->entities) ? true : false;

	list_add_tail(&entity->list, &ctrl->entities);
	if (boot_flag)
		ret = wave5_vpu_ctrl_try_boot(ctrl, entity);

	if (ctrl->state == WAVE5_VPU_STATE_ON)
		wave5_vpu_ctrl_on_boot(entity);

	if (ret)
		pm_runtime_put_sync(ctrl->dev);

	return ret;
}
EXPORT_SYMBOL_GPL(wave5_vpu_ctrl_resume_and_get);

void wave5_vpu_ctrl_put_sync(struct device *dev, struct wave5_vpu_entity *entity)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	if (!ctrl || !entity || !entity->read_reg)
		return;

	if (entity == ctrl->current_entity)
		wave5_vpu_ctrl_wait_done(dev);

	guard(mutex)(&ctrl->ctrl_lock);

	if (!wave5_vpu_ctrl_find_entity(ctrl, entity))
		return;

	list_del_init(&entity->list);
	if (list_empty(&ctrl->entities)) {
		if (!call_read_reg(entity, W5_VCPU_CUR_PC))
			wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_OFF);
		else
			wave5_vpu_ctrl_sleep(ctrl, entity);
	}

#if WAVE5_ENABLE_SW_UART
	wave5_vpu_ctrl_stop_loger(ctrl, entity);
#endif
	if (!pm_runtime_suspended(ctrl->dev))
		pm_runtime_put_sync(ctrl->dev);
}
EXPORT_SYMBOL_GPL(wave5_vpu_ctrl_put_sync);

int wave5_vpu_ctrl_wait_done(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);
	int ret;

	if (!ctrl)
		return -EINVAL;

	if (ctrl->state == WAVE5_VPU_STATE_OFF)
		return -EINVAL;

	if (ctrl->state == WAVE5_VPU_STATE_ON)
		return 0;

	ret = wave5_wait_event_freezable_timeout(ctrl->load_fw_wq,
						 wave5_vpu_ctrl_get_state(dev) ==
						 WAVE5_VPU_STATE_ON,
						 msecs_to_jiffies(VPU_BOOT_WAIT_TIMEOUT));
	if (ret == -ERESTARTSYS || ret == 0) {
		dev_err(ctrl->dev, "fail to wait vcpu boot done,ret %d\n", ret);
		scoped_guard(mutex, &ctrl->ctrl_lock)
			wave5_vpu_ctrl_set_state(ctrl, WAVE5_VPU_STATE_OFF);
		return -EINVAL;
	}

	scoped_guard(mutex, &ctrl->ctrl_lock)
		wave5_vpu_ctrl_boot_done(ctrl, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(wave5_vpu_ctrl_wait_done);

int wave5_vpu_ctrl_get_state(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	if (!ctrl)
		return -EINVAL;

	return ctrl->state;
}
EXPORT_SYMBOL_GPL(wave5_vpu_ctrl_get_state);

static void wave5_vpu_ctrl_init_reserved_boot_region(struct vpu_ctrl *ctrl, struct resource mem)
{
	phys_addr_t phys_addr = mem.start;
	size_t size = resource_size(&mem);

	if (size < WAVE521_SIZE_COMMON) {
		dev_warn(ctrl->dev, "boot memory size (%zu) is too small\n", size);
		memset(&ctrl->boot_mem, 0, sizeof(ctrl->boot_mem));
		return;
	}

	ctrl->boot_mem.size = size;
	ctrl->boot_mem.vaddr = devm_memremap(ctrl->dev, phys_addr, size, MEMREMAP_WC);
	if (!ctrl->boot_mem.vaddr) {
		memset(&ctrl->boot_mem, 0, sizeof(ctrl->boot_mem));
		return;
	}

	ctrl->boot_mem.daddr = dma_map_resource(ctrl->dev, phys_addr, size, DMA_BIDIRECTIONAL, 0);
	if (!ctrl->boot_mem.daddr) {
		memset(&ctrl->boot_mem, 0, sizeof(ctrl->boot_mem));
		return;
	}

	dev_info(ctrl->dev, "boot phys_addr: %pad, dma_addr: %pad, size: 0x%zx\n",
		 &phys_addr, &ctrl->boot_mem.daddr, ctrl->boot_mem.size);
}

static int wave5_vpu_ctrl_probe(struct platform_device *pdev)
{
	struct vpu_ctrl *ctrl;
	struct device_node *np;
	const struct vpu_ctrl_resource *res;
	int ret;

	/* physical addresses limited to 32 bits */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret < 0) {
		dev_err(&pdev->dev, "dma_set_mask_and_coherent failed: %d\n", ret);
		return ret;
	}

	res = of_device_get_match_data(&pdev->dev);
	if (!res)
		return -ENODEV;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	mutex_init(&ctrl->ctrl_lock);
	init_waitqueue_head(&ctrl->load_fw_wq);
	INIT_LIST_HEAD(&ctrl->entities);
	dev_set_drvdata(&pdev->dev, ctrl);
	ctrl->dev = &pdev->dev;
	ctrl->res = res;
	ctrl->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->reg_base))
		return PTR_ERR(ctrl->reg_base);
	ret = devm_clk_bulk_get_all(&pdev->dev, &ctrl->clks);
	if (ret < 0) {
		dev_warn(&pdev->dev, "unable to get clocks: %d\n", ret);
		ret = 0;
	}

	ctrl->num_clks = ret;

	np = of_parse_phandle(pdev->dev.of_node, "boot", 0);
	if (np) {
		struct resource mem;

		ret = of_address_to_resource(np, 0, &mem);
		of_node_put(np);
		if (!ret)
			wave5_vpu_ctrl_init_reserved_boot_region(ctrl, mem);
		else
			dev_warn(&pdev->dev, "boot resource is not available.\n");
	}

	ctrl->sram_pool = of_gen_pool_get(pdev->dev.of_node, "sram", 0);
	if (ctrl->sram_pool) {
		phys_addr_t phys_addr;

		ctrl->sram_buf.size = ctrl->res->sram_size;
		ctrl->sram_buf.vaddr = gen_pool_dma_alloc(ctrl->sram_pool,
							  ctrl->sram_buf.size,
							  &phys_addr);
		if (!ctrl->sram_buf.vaddr)
			ctrl->sram_buf.size = 0;
		else
			ctrl->sram_buf.daddr = dma_map_resource(&pdev->dev,
								phys_addr,
								ctrl->sram_buf.size,
								DMA_BIDIRECTIONAL,
								0);

		dev_info(&pdev->dev, "sram 0x%pad, 0x%pad, size 0x%lx\n",
			 &phys_addr, &ctrl->sram_buf.daddr, ctrl->sram_buf.size);
	}

	if (of_find_property(pdev->dev.of_node, "support-follower", NULL))
		ctrl->support_follower = true;

#if WAVE5_ENABLE_SW_UART
	wave5_vpu_ctrl_init_loger(ctrl);
	wave5_vpu_ctrl_create_debugfs(ctrl);
#endif

	wave5_vpu_ctrl_acquire_buffers(ctrl);
	pm_runtime_enable(&pdev->dev);

	return 0;
}

static void wave5_vpu_ctrl_remove(struct platform_device *pdev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(&pdev->dev);

#if WAVE5_ENABLE_SW_UART
	wave5_vpu_ctrl_remove_debugfs(ctrl);
	wave5_vpu_ctrl_free_loger(ctrl);
#endif

	pm_runtime_disable(&pdev->dev);

	wave5_vpu_ctrl_clear_buffers(ctrl);
	wave5_vpu_ctrl_free_buffers(ctrl);
	if (ctrl->sram_pool && ctrl->sram_buf.vaddr) {
		dma_unmap_resource(&pdev->dev,
				   ctrl->sram_buf.daddr,
				   ctrl->sram_buf.size,
				   DMA_BIDIRECTIONAL,
				   0);
		gen_pool_free(ctrl->sram_pool,
			      (unsigned long)ctrl->sram_buf.vaddr,
			      ctrl->sram_buf.size);
	}
	if (ctrl->boot_mem.daddr)
		dma_unmap_resource(&pdev->dev,
				   ctrl->boot_mem.daddr,
				   ctrl->boot_mem.size,
				   DMA_BIDIRECTIONAL,
				   0);
	mutex_destroy(&ctrl->ctrl_lock);
}

#ifdef CONFIG_PM
static int wave5_vpu_ctrl_runtime_suspend(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ctrl->num_clks, ctrl->clks);
	return 0;
}

static int wave5_vpu_ctrl_runtime_resume(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(ctrl->num_clks, ctrl->clks);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int wave5_vpu_ctrl_suspend(struct device *dev)
{
	return 0;
}

static int wave5_vpu_ctrl_resume(struct device *dev)
{
	return 0;
}
#endif
static const struct dev_pm_ops wave5_vpu_ctrl_pm_ops = {
	SET_RUNTIME_PM_OPS(wave5_vpu_ctrl_runtime_suspend, wave5_vpu_ctrl_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(wave5_vpu_ctrl_suspend, wave5_vpu_ctrl_resume)
};

static const struct of_device_id wave5_ctrl_ids[] = {
	{ .compatible = "nxp,wave511-vpu-ctrl", .data = &nxp_wave511_ctrl_data},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wave5_ctrl_ids);

static struct platform_driver wave5_vpu_ctrl_driver = {
	.driver = {
		.name = VPU_CTRL_PLATFORM_DEVICE_NAME,
		.of_match_table = of_match_ptr(wave5_ctrl_ids),
		.pm = &wave5_vpu_ctrl_pm_ops,
	},
	.probe = wave5_vpu_ctrl_probe,
	.remove = wave5_vpu_ctrl_remove,
};

module_platform_driver(wave5_vpu_ctrl_driver);
MODULE_DESCRIPTION("chips&media VPU CTRL driver");
MODULE_LICENSE("Dual BSD/GPL");
