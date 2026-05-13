/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 NXP
 */

#ifndef __ATU_H__
#define __ATU_H__

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/bitmap.h>
#include <linux/types.h>

#define CHUNK_SIZE_IN_MB	1
#define CHUNK_SIZE		(CHUNK_SIZE_IN_MB * 1024 * 1024)
#define TOTAL_MEM_SIZE_GB	1
#define TOTAL_MEM_SIZE		(TOTAL_MEM_SIZE_GB * 1024ULL * 1024 * 1024)
#define NUM_CHUNKS		(TOTAL_MEM_SIZE / CHUNK_SIZE)

#define OWBAR_BASE   0x110u
#define OWAR_BASE    0x114u
#define OTEAR_BASE   0x118u
#define OTAR_BASE    0x11Cu
#define WIN_STRIDE   0x10u

#define OWAR_VALID_BIT      BIT(31)

#define OWBAR_DOM_SHIFT     28u
#define OWBAR_WBA_SHIFT     16u
#define OWBAR_WBA_MASK      0x0FFFu
#define OWBAR_WIN_SIZE_MASK 0x1Fu

#define OTAR_TA_SHIFT       16u
/*
 * OTARn[27:16] is TA (Translated Address).
 * Provides up to 12-bits (chip addr[31:20]) of a 36-bit chip address for this chip
 */
#define OTAR_TA_MASK        0x0FFFu
/*
 * OTEARn[3:0] is TEA (Translated Extended Address).
 * Provides 4-bits (chip addr[35:32]) of a 36-bit chip address
 */
#define OTEAR_TEA_MASK      0x0Fu

#define ATU_NUM_WINS 7

#define DOMAIN_ID_A55 0x3

#define DEFAULT_WINDOW_SIZE	0x10000000ULL /* 256MB inbound spacing */
#define ATU_INBOUND_START	0x00000000ULL
#define ATU_INBOUND_END		0x80000000ULL /* exclusive */

#define ATU_IOC_MAGIC 'A'

/* IOCTL commands */
/* Configure a new ATU mapping (phys,size -> virt)  */
#define ATU_IOC_CONFIG		_IOWR(ATU_IOC_MAGIC, 1, struct atu_conf_params)

/* Deconfigure an ATU mapping (by phys) */
#define ATU_IOC_DECONFIG	_IOR(ATU_IOC_MAGIC, 2, struct atu_conf_params)

#define ATU_GET_RESV_MEM	_IOWR(ATU_IOC_MAGIC, 3, struct atu_resv_mem_req)
#define ATU_FREE_RESV_MEM	_IOW(ATU_IOC_MAGIC, 4, struct atu_resv_mem_req)
#define ATU_GET_RESV_MEM_INFO	_IOR(ATU_IOC_MAGIC, 5, struct atu_resv_mem_info)

/* Window size encodings */
enum atu_win_code {
	ATU_WINDOW_SIZE_1MB   = 0x00,
	ATU_WINDOW_SIZE_2MB   = 0x01,
	ATU_WINDOW_SIZE_4MB   = 0x02,
	ATU_WINDOW_SIZE_8MB   = 0x03,
	ATU_WINDOW_SIZE_16MB  = 0x04,
	ATU_WINDOW_SIZE_32MB  = 0x05,
	ATU_WINDOW_SIZE_64MB  = 0x06,
	ATU_WINDOW_SIZE_128MB = 0x07,
	ATU_WINDOW_SIZE_256MB = 0x08,
	ATU_WINDOW_SIZE_512MB = 0x09,
	ATU_WINDOW_SIZE_1GB   = 0x0A,
	ATU_WINDOW_SIZE_2GB   = 0x0B,
};

/* One mapping request/response */
struct atu_conf_params {
	__u64 outbound_addr;	/* IN: physical (outbound) base */
	__u32 win_size;		/* IN: window size (power-of-two, >=1MB) */
	__u32 inbound_addr;	/* OUT: inbound (CPU-visible) base in 0x40000000..0x80000000 */
};

/* IOCTL data structures */
struct atu_resv_mem_req {
	__u64 phys_addr;	/* Physical address (output for reserve, input for free) */
	__u32 size;	/* Size in bytes (input for reserve, input for free) */
	__u32 inbound_addr;	/* OUT: inbound (CPU-visible) base in 0x0..0x40000000 */
};

struct atu_resv_mem_info {
	__u64 phys_addr;	/* Physical start address of reserved memory */
	__u32 size;	/* Total size of reserved memory */
};

struct atu_mapping {
	struct list_head list;
	u64     outbound_base;   /* >= 32-bit */
	u32     size;            /* bytes */
	u32     inbound_base;    /* <= 32-bit */
	u8      win;             /* 1..7 */
};

struct atu_regs {
	u32 atucr;
	u32 atusr;
	u32 ip_rev_1;   /* read-only in HW, kept as plain u32 per request */
	u32 ip_rev_2;   /* read-only in HW, kept as plain u32 per request */
	u32 reserved_010_018[3];
	u32 pmcr;
	u32 reserved_020_103[57];
	u32 acore_owar0;
	u32 acore_otear0;
	u32 reserved_10c;
	u32 acore_owbar1;
	u32 acore_owar1;
	u32 acore_otear1;
	u32 acore_otar1;
	u32 acore_owbar2;
	u32 acore_owar2;
	u32 acore_otear2;
	u32 acore_otar2;
	u32 acore_owbar3;
	u32 acore_owar3;
	u32 acore_otear3;
	u32 acore_otar3;
	u32 acore_owbar4;
	u32 acore_owar4;
	u32 acore_otear4;
	u32 acore_otar4;
	u32 acore_owbar5;
	u32 acore_owar5;
	u32 acore_otear5;
	u32 acore_otar5;
	u32 acore_owbar6;
	u32 acore_owar6;
	u32 acore_otear6;
	u32 acore_otar6;
	u32 acore_owbar7;
	u32 acore_owar7;
	u32 acore_otear7;
	u32 acore_otar7;
};

struct atu_dev {
	void __iomem *atu_base;
	phys_addr_t  atu_phys;
	u32          atu_size;
	/* Memory management fields */
	u64 resv_mem_phys_addr;		/* Physical address of reserved memory */
	u32 resv_mem_size;		/* Size of reserved memory */
	bool resv_mem_atu_configured;	/* flag: ATU for resv memory has been initialized */
	unsigned long *resv_mem_bitmap;	/* Bitmap for memory allocation (1 bit = 1 MB) */
	unsigned long resv_num_chunks;	/* Number of 1MB chunks */

	unsigned long *inbound_mem_bitmap;
	unsigned long inbound_num_chunks;

	struct mutex mem_lock;
	DECLARE_BITMAP(win_bmap, ATU_NUM_WINS + 1); /* we use bits 1..7 */
	u8  domain_id;
	u64 max_window_size;
	struct list_head mappings;	/* active mappings */
};

int configure_atu(struct atu_dev *atu_dev, struct atu_conf_params *atu_params);
int deconfigure_atu(struct atu_dev *atu_dev,
		    struct atu_conf_params *atu_params);
int atu_ll_init(struct atu_dev *ad);

int atu_resv_mem(struct atu_dev *atu_dev, struct atu_resv_mem_req *req);
int atu_free_resv_mem(struct atu_dev *atu_dev, struct atu_resv_mem_req *req);
int atu_get_resv_mem_info(struct atu_dev *atu_dev,
			  struct atu_resv_mem_info *info);

void print_atu_regs(struct atu_dev *atu_dev);

int atu_mmap(struct atu_dev *atu_dev, struct vm_area_struct *vma);
long atu_ioctl(struct atu_dev *atu_dev, unsigned int cmd, unsigned long arg);

#endif /* __ATU_H__ */
