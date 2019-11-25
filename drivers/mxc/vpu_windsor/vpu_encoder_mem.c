/*
 * Copyright 2018-2019 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#define TAG	"[VPU Encoder Mem]\t "
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include "vpu_encoder_config.h"
#include "vpu_encoder_b0.h"
#include "vpu_encoder_mem.h"

int vpu_enc_init_reserved_memory(struct vpu_enc_mem_info *info)
{
	if (!info || !info->phy_addr || !info->size)
		return -EINVAL;

	info->virt_addr = ioremap_wc(info->phy_addr, info->size);
	if (!info->virt_addr)
		return -EINVAL;
	memset_io(info->virt_addr, 0, info->size);
	info->bytesused = 0;
	INIT_LIST_HEAD(&info->memorys);
	mutex_init(&info->lock);

	return 0;
}

void vpu_enc_release_reserved_memory(struct vpu_enc_mem_info *info)
{
	struct vpu_enc_mem_item *item = NULL;
	struct vpu_enc_mem_item *tmp = NULL;

	if (!info)
		return;

	mutex_lock(&info->lock);
	list_for_each_entry_safe(item, tmp, &info->memorys, list) {
		list_del_init(&item->list);
		info->bytesused -= item->size;
		vpu_dbg(LVL_MEM, "free reserved memory %ld\n", item->size);
		VPU_SAFE_RELEASE(item, vfree);
	}
	mutex_unlock(&info->lock);

	mutex_destroy(&info->lock);
	if (info->virt_addr) {
		iounmap(info->virt_addr);
		info->virt_addr = NULL;
	}
}

int vpu_enc_alloc_reserved_mem(struct vpu_enc_mem_info *info,
				struct buffer_addr *buffer)
{
	struct vpu_enc_mem_item *item = NULL;
	struct list_head *pos = NULL;
	unsigned long offset = 0;
	int ret;

	if (!info || !buffer)
		return -EINVAL;

	mutex_lock(&info->lock);
	if (buffer->size + info->bytesused > info->size) {
		ret = -ENOMEM;
		goto exit;
	}

	list_for_each_entry(item, &info->memorys, list) {
		if (item->offset - offset >= buffer->size) {
			pos = &item->list;
			break;
		}
		offset = item->offset + item->size;
	}
	if (!pos && info->size - offset >= buffer->size)
		pos = &info->memorys;
	if (!pos) {
		ret = -ENOMEM;
		goto exit;
	}
	item = vzalloc(sizeof(*item));
	if (!item) {
		ret = -EINVAL;
		goto exit;
	}
	item->offset = offset;
	item->virt_addr = info->virt_addr + offset;
	item->phy_addr = info->phy_addr + offset;
	item->size = buffer->size;
	list_add_tail(&item->list, pos);
	info->bytesused += buffer->size;
	vpu_dbg(LVL_MEM, "alloc reserved memory <0x%lx 0x%lx(%ld)>\n",
			item->phy_addr, item->size, item->size);
	buffer->virt_addr = item->virt_addr;
	buffer->phy_addr = item->phy_addr;
	ret = 0;
exit:
	mutex_unlock(&info->lock);
	return ret;
}

int vpu_enc_free_reserved_mem(struct vpu_enc_mem_info *info,
				struct buffer_addr *buffer)
{
	struct vpu_enc_mem_item *item = NULL;
	struct vpu_enc_mem_item *tmp = NULL;
	unsigned long offset;
	int ret = -EINVAL;

	if (!info || !buffer)
		return -EINVAL;
	if (!buffer->virt_addr)
		return 0;

	if (buffer->phy_addr < info->phy_addr) {
		vpu_err("invalid reserved memory addr : 0x%llx %d\n",
				buffer->phy_addr, buffer->size);
		return -EINVAL;
	}

	offset = buffer->phy_addr - info->phy_addr;
	if (offset + buffer->size > info->size) {
		vpu_err("invalid reserved memory addr : 0x%llx %d\n",
				buffer->phy_addr, buffer->size);
		return -EINVAL;
	}

	mutex_lock(&info->lock);
	list_for_each_entry_safe(item, tmp, &info->memorys, list) {
		if (offset < item->offset)
			continue;
		if (offset + buffer->size > item->offset + item->size)
			continue;
		list_del_init(&item->list);
		info->bytesused -= item->size;
		vpu_dbg(LVL_MEM, "free reserved memory <0x%lx 0x%lx(%ld)>\n",
			item->phy_addr, item->size, item->size);
		VPU_SAFE_RELEASE(item, vfree);
		ret = 0;
		break;
	}
	mutex_unlock(&info->lock);

	return ret;
}

void vpu_enc_add_dma_size(struct vpu_attr *attr, unsigned long size)
{
	if (!attr)
		return;

	atomic64_add(size, &attr->total_dma_size);
}

void vpu_enc_sub_dma_size(struct vpu_attr *attr, unsigned long size)
{
	if (!attr)
		return;

	atomic64_sub(size, &attr->total_dma_size);
}

int vpu_enc_alloc_dma_buffer(struct vpu_ctx *ctx, struct buffer_addr *buffer)
{
	if (!ctx || !ctx->dev || !buffer || !buffer->size)
		return -EINVAL;

	vpu_dbg(LVL_MEM, "alloc coherent dma %d\n", buffer->size);
	buffer->virt_addr = dma_alloc_coherent(ctx->dev->generic_dev,
						buffer->size,
						(dma_addr_t *)&buffer->phy_addr,
						GFP_KERNEL | GFP_DMA32);
	if (!buffer->virt_addr) {
		vpu_err("encoder alloc coherent dma(%d) fail\n",
				buffer->size);
		return -ENOMEM;
	}
	memset_io(buffer->virt_addr, 0, buffer->size);
	vpu_enc_add_dma_size(get_vpu_ctx_attr(ctx), buffer->size);

	return 0;
}

void vpu_enc_init_dma_buffer(struct buffer_addr *buffer)
{
	if (!buffer)
		return;

	buffer->virt_addr = NULL;
	buffer->phy_addr = 0;
	buffer->size = 0;
}

int vpu_enc_free_dma_buffer(struct vpu_ctx *ctx, struct buffer_addr *buffer)
{
	if (!ctx || !ctx->dev || !buffer)
		return -EINVAL;

	if (!buffer->virt_addr)
		return 0;

	vpu_dbg(LVL_MEM, "free coherent dma %d\n", buffer->size);
	vpu_enc_sub_dma_size(get_vpu_ctx_attr(ctx), buffer->size);
	dma_free_coherent(ctx->dev->generic_dev, buffer->size,
				buffer->virt_addr, buffer->phy_addr);

	vpu_enc_init_dma_buffer(buffer);

	return 0;
}

static bool check_mem_resource_is_valid(MEDIAIP_ENC_MEM_RESOURCE *resource)
{
	if (!resource)
		return false;
	if (resource->uMemVirtAddr >= VPU_MU_MAX_ADDRESS)
		return false;
	if (resource->uMemVirtAddr + resource->uMemSize > VPU_MU_MAX_ADDRESS)
		return false;
	return true;
}

static u32 get_enc_alloc_size(u32 size)
{
	u32 esize = ALIGN(size, PAGE_SIZE);

	if (esize < size + sizeof(u32))
		esize += PAGE_SIZE;

	return esize;
}

static int alloc_mem_res(struct vpu_ctx *ctx, struct buffer_addr *buffer,
			MEDIAIP_ENC_MEM_RESOURCE *resource, u32 size)
{
	int ret;

	if (!ctx || !buffer || !resource)
		return -EINVAL;

	if (!size) {
		vpu_err("invalid memory resource size : %d\n", size);
		return -EINVAL;
	}

	buffer->size = get_enc_alloc_size(size);
	ret = vpu_enc_alloc_dma_buffer(ctx, buffer);
	if (ret)
		return ret;

	resource->uMemPhysAddr = buffer->phy_addr;
	resource->uMemVirtAddr = cpu_phy_to_mu(ctx->core_dev, buffer->phy_addr);
	resource->uMemSize = size;

	return 0;
}

static int free_mem_res(struct vpu_ctx *ctx, struct buffer_addr *buffer,
			MEDIAIP_ENC_MEM_RESOURCE *resource)
{
	if (!ctx || !buffer || !resource)
		return -EINVAL;

	vpu_enc_free_dma_buffer(ctx, buffer);

	resource->uMemPhysAddr = 0;
	resource->uMemVirtAddr = 0;
	resource->uMemSize = 0;

	return 0;
}

static int alloc_reserved_mem_res(struct vpu_ctx *ctx,
				struct buffer_addr *buffer,
				MEDIAIP_ENC_MEM_RESOURCE *resource,
				u32 size)
{
	int ret;

	if (!ctx || !ctx->dev || !buffer || !resource)
		return -EINVAL;

	if (!size) {
		vpu_err("invalid memory resource size : %d\n", size);
		return -EINVAL;
	}

	buffer->size = get_enc_alloc_size(size);
	ret = vpu_enc_alloc_reserved_mem(&ctx->dev->reserved_mem, buffer);
	if (ret)
		return ret;

	resource->uMemPhysAddr = buffer->phy_addr;
	resource->uMemVirtAddr = cpu_phy_to_mu(ctx->core_dev, buffer->phy_addr);
	resource->uMemSize = size;

	return 0;
}

static int free_reserved_mem_res(struct vpu_ctx *ctx,
				struct buffer_addr *buffer,
				MEDIAIP_ENC_MEM_RESOURCE *resource)
{
	if (!ctx || !ctx->dev || !buffer || !resource)
		return -EINVAL;

	vpu_enc_free_reserved_mem(&ctx->dev->reserved_mem, buffer);

	resource->uMemPhysAddr = 0;
	resource->uMemVirtAddr = 0;
	resource->uMemSize = 0;

	return 0;
}

static int free_enc_frames(struct vpu_ctx *ctx, pMEDIAIP_ENC_MEM_POOL pool)
{
	int i;

	vpu_log_func();
	for (i = 0; i < ctx->mem_req.uEncFrmNum; i++)
		free_mem_res(ctx, &ctx->encFrame[i],
				&pool->tEncFrameBuffers[i]);

	return 0;
}

static int alloc_enc_frames(struct vpu_ctx *ctx, pMEDIAIP_ENC_MEM_POOL pool)
{
	int i;
	int ret;

	vpu_log_func();
	for (i = 0; i < ctx->mem_req.uEncFrmNum; i++) {
		ret = alloc_mem_res(ctx,
				&ctx->encFrame[i],
				&pool->tEncFrameBuffers[i],
				ctx->mem_req.uEncFrmSize);
		if (ret) {
			vpu_err("alloc enc frame[%d] fail\n", i);
			goto error;
		}
		vpu_dbg(LVL_MEM, "encFrame[%d]: 0x%llx,%d(%d)\n", i,
				ctx->encFrame[i].phy_addr,
				ctx->mem_req.uEncFrmSize,
				ctx->encFrame[i].size);
	}

	return 0;
error:
	free_enc_frames(ctx, pool);
	return ret;
}

static int free_ref_frames(struct vpu_ctx *ctx, pMEDIAIP_ENC_MEM_POOL pool)
{
	int i;

	vpu_log_func();
	for (i = 0; i < ctx->mem_req.uRefFrmNum; i++)
		free_mem_res(ctx, &ctx->refFrame[i],
				&pool->tRefFrameBuffers[i]);

	return 0;
}

static int alloc_ref_frames(struct vpu_ctx *ctx, pMEDIAIP_ENC_MEM_POOL pool)
{
	int i;
	int ret;

	vpu_log_func();
	for (i = 0; i < ctx->mem_req.uRefFrmNum; i++) {
		ret = alloc_mem_res(ctx,
				&ctx->refFrame[i],
				&pool->tRefFrameBuffers[i],
				ctx->mem_req.uRefFrmSize);
		if (ret) {
			vpu_err("alloc ref frame[%d] fail\n", i);
			goto error;
		}
		vpu_dbg(LVL_MEM, "refFrame[%d]: 0x%llx,%d(%d)\n", i,
				ctx->refFrame[i].phy_addr,
				ctx->mem_req.uRefFrmSize,
				ctx->refFrame[i].size);
	}

	return 0;
error:
	free_ref_frames(ctx, pool);
	return ret;
}

static int free_act_frame(struct vpu_ctx *ctx, pMEDIAIP_ENC_MEM_POOL pool)
{
	if (!ctx || !pool)
		return -EINVAL;

	vpu_log_func();
	free_reserved_mem_res(ctx, &ctx->actFrame, &pool->tActFrameBufferArea);

	return 0;
}

static int alloc_act_frame(struct vpu_ctx *ctx, pMEDIAIP_ENC_MEM_POOL pool)
{
	int ret = 0;

	vpu_log_func();
	ret = alloc_reserved_mem_res(ctx,
			&ctx->actFrame,
			&pool->tActFrameBufferArea,
			ctx->mem_req.uActBufSize);
	if (ret) {
		vpu_err("alloc act frame fail\n");
		return ret;
	}

	if (!check_mem_resource_is_valid(&pool->tActFrameBufferArea)) {
		vpu_err("invalid actFrames address, 0x%x, 0x%x, 0x%x\n",
				pool->tActFrameBufferArea.uMemPhysAddr,
				pool->tActFrameBufferArea.uMemVirtAddr,
				pool->tActFrameBufferArea.uMemSize);
		free_act_frame(ctx, pool);
		return -EINVAL;
	}

	vpu_dbg(LVL_MEM, "actFrame: 0x%llx, %d(%d)\n",
			ctx->actFrame.phy_addr,
			ctx->mem_req.uActBufSize,
			ctx->actFrame.size);
	return 0;
}

static void set_mem_pattern(u32 *ptr)
{
	if (!ptr)
		return;
	*ptr = VPU_MEM_PATTERN;
}

static int check_mem_pattern(u32 *ptr)
{
	if (!ptr)
		return -EINVAL;

	if (*ptr != VPU_MEM_PATTERN)
		return -EINVAL;

	return 0;
}

static void vpu_enc_set_mem_pattern(struct vpu_ctx *ctx)
{
	int i;

	if (!ctx)
		return;

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES; i++) {
		if (!ctx->encFrame[i].virt_addr)
			continue;
		set_mem_pattern(ctx->encFrame[i].virt_addr +
				ctx->mem_req.uEncFrmSize);
	}

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++) {
		if (!ctx->refFrame[i].virt_addr)
			continue;
		set_mem_pattern(ctx->refFrame[i].virt_addr +
				ctx->mem_req.uRefFrmSize);
	}

	if (ctx->actFrame.virt_addr)
		set_mem_pattern(ctx->actFrame.virt_addr +
				ctx->mem_req.uActBufSize);
}

int vpu_enc_check_mem_overstep(struct vpu_ctx *ctx)
{
	int i;
	int ret;
	int flag = 0;

	if (!ctx)
		return -EINVAL;

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES; i++) {
		if (!ctx->encFrame[i].virt_addr)
			continue;
		ret = check_mem_pattern(ctx->encFrame[i].virt_addr +
					ctx->mem_req.uEncFrmSize);
		if (ret) {
			vpu_err("***error:[%d][%d]encFrame[%d] out of bounds\n",
					ctx->core_dev->id, ctx->str_index, i);
			flag = 1;
		}
	}

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++) {
		if (!ctx->refFrame[i].virt_addr)
			continue;
		ret = check_mem_pattern(ctx->refFrame[i].virt_addr +
					ctx->mem_req.uRefFrmSize);
		if (ret) {
			vpu_err("***error:[%d][%d]refFrame[%d] out of bounds\n",
					ctx->core_dev->id, ctx->str_index, i);
			flag = 1;
		}
	}

	if (ctx->actFrame.virt_addr) {
		ret = check_mem_pattern(ctx->actFrame.virt_addr +
					ctx->mem_req.uActBufSize);
		if (ret) {
			vpu_err("***error:[%d][%d]actFrame out of bounds\n",
					ctx->core_dev->id, ctx->str_index);
			flag = 1;
		}
	}

	if (flag) {
		vpu_err("Error:Memory out of bounds in [%d][%d]\n",
			ctx->core_dev->id, ctx->str_index);
		vpu_enc_set_mem_pattern(ctx);
	}

	return 0;
}

int vpu_enc_alloc_mem(struct vpu_ctx *ctx,
			MEDIAIP_ENC_MEM_REQ_DATA *req_data,
			pMEDIAIP_ENC_MEM_POOL pool)
{
	int ret;

	if (!ctx || !req_data || !pool)
		return -EINVAL;

	if (ctx->mem_req.uEncFrmSize < req_data->uEncFrmSize ||
			ctx->mem_req.uEncFrmNum < req_data->uEncFrmNum) {
		free_enc_frames(ctx, pool);
		ctx->mem_req.uEncFrmSize = req_data->uEncFrmSize;
		ctx->mem_req.uEncFrmNum = req_data->uEncFrmNum;
		ret = alloc_enc_frames(ctx, pool);
		if (ret)
			return ret;
	}

	if (ctx->mem_req.uRefFrmSize < req_data->uRefFrmSize ||
			ctx->mem_req.uRefFrmNum < req_data->uRefFrmNum) {
		free_ref_frames(ctx, pool);
		ctx->mem_req.uRefFrmSize = req_data->uRefFrmSize;
		ctx->mem_req.uRefFrmNum = req_data->uRefFrmNum;
		ret = alloc_ref_frames(ctx, pool);
		if (ret)
			goto error_alloc_refs;
	}

	if (ctx->mem_req.uActBufSize < req_data->uActBufSize) {
		free_act_frame(ctx, pool);
		ctx->mem_req.uActBufSize = req_data->uActBufSize;
		ret = alloc_act_frame(ctx, pool);
		if (ret)
			goto error_alloc_act;
	}

	vpu_enc_set_mem_pattern(ctx);

	return 0;
error_alloc_act:
	free_ref_frames(ctx, pool);
error_alloc_refs:
	free_enc_frames(ctx, pool);
	return ret;
}

int vpu_enc_free_mem(struct vpu_ctx *ctx, pMEDIAIP_ENC_MEM_POOL pool)
{
	if (!ctx || !pool)
		return -EINVAL;

	free_act_frame(ctx, pool);
	free_ref_frames(ctx, pool);
	free_enc_frames(ctx, pool);

	return 0;
}

int vpu_enc_alloc_stream(struct vpu_ctx *ctx)
{
	int ret;

	if (ctx->encoder_stream.virt_addr)
		return 0;

	ctx->encoder_stream.size =
		max_t(u32, ctx->cpb_size * CPB_COUNT, STREAM_SIZE);
	ret = vpu_enc_alloc_dma_buffer(ctx, &ctx->encoder_stream);
	if (ret) {
		vpu_err("alloc encoder stream buffer fail\n");
		return -ENOMEM;
	}
	vpu_dbg(LVL_MEM, "encoder_stream: 0x%llx, %d\n",
			ctx->encoder_stream.phy_addr, ctx->encoder_stream.size);

	return 0;
}

void vpu_enc_free_stream(struct vpu_ctx *ctx)
{
	vpu_enc_free_dma_buffer(ctx, &ctx->encoder_stream);
}
