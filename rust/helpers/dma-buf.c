// SPDX-License-Identifier: GPL-2.0

#include <linux/dma-buf.h>

__rust_helper void rust_helper_get_dma_buf(struct dma_buf *dmabuf)
{
	get_dma_buf(dmabuf);
}

__rust_helper void rust_helper_iosys_map_memcpy_to(struct iosys_map *dst, size_t dst_offset,
						   const void *src, size_t len)
{
	iosys_map_memcpy_to(dst, dst_offset, src, len);
}

__rust_helper void rust_helper_iosys_map_memcpy_from(void *dst, const struct iosys_map *src,
						     size_t src_offset, size_t len)
{
	iosys_map_memcpy_from(dst, src, src_offset, len);
}
