/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 */

#ifndef __HVM_DRV_H__
#define __HVM_DRV_H__

#include <linux/eventfd.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/srcu.h>
#include <linux/rbtree.h>
#include <linux/clocksource.h>
#include <linux/const.h>
#include <linux/ioctl.h>
#include <linux/types.h>


/* UAPI data structure between kernel drvier and user space */
/* HVM ioctls */
#define HVM_IOC_MAGIC			0x4C	/* halla */

/* ioctls for /dev/hvm fds */
#define HVM_CREATE_VM			_IO(HVM_IOC_MAGIC,   0x01)
#define HVM_CHECK_EXTENSION		_IO(HVM_IOC_MAGIC,   0x02)

/* ioctls for VM fds */
#define HVM_CREATE_VCPU			_IO(HVM_IOC_MAGIC,   0x20)
#define HVM_SET_USER_MEMORY_REGION	_IOW(HVM_IOC_MAGIC, 0x41, \
					struct hvm_userspace_memory_region)

#define HVM_IRQFD			_IOW(HVM_IOC_MAGIC, 0x61, \
					struct hvm_irqfd)
#define HVM_IRQ_LINE			_IOW(HVM_IOC_MAGIC,  0x62, \
					struct hvm_irq_level)

#define HVM_CREATE_DEVICE		_IOWR(HVM_IOC_MAGIC,  0x80, \
					struct hvm_create_device)

#define HVM_IOEVENTFD			_IOW(HVM_IOC_MAGIC, 0xa0, \
					struct hvm_ioeventfd)

#define HVM_ENABLE_CAP			_IOW(HVM_IOC_MAGIC,  0x03, \
					struct hvm_enable_cap)
#define HVM_SET_DTB_CONFIG		_IOW(HVM_IOC_MAGIC, 0x04, \
					struct hvm_dtb_config)

/* ioctls for vcpu fds */
#define HVM_VCPU_RUN			_IO(HVM_IOC_MAGIC,   0x21)

#define HVM_GET_ONE_REG			_IOW(HVM_IOC_MAGIC,  0x28, \
					struct hvm_one_reg)
#define HVM_SET_ONE_REG			_IOW(HVM_IOC_MAGIC,  0x29, \
					struct hvm_one_reg)

/* Capability for HVM_CHECK_EXTENSION */
#define HVM_CAP_VM_GPA_SIZE	0x475041	// 'GPA'
#define HVM_CAP_PROTECTED_VM	0x50564d	// 'PVM'

/* sub-commands put in args[0] for HVM_CAP_PROTECTED_VM */
#define HVM_CAP_PVM_SET_PVMFW_GPA		0
#define HVM_CAP_PVM_GET_PVMFW_SIZE		1
/* HVM_CAP_PVM_SET_PROTECTED_VM only sets protected but not load pvmfw */
#define HVM_CAP_PVM_SET_PROTECTED_VM		2

/*
 * Reg size = BIT((reg.id & HVM_REG_SIZE_MASK) >> HVM_REG_SIZE_SHIFT) bytes
 */
#define HVM_REG_SIZE_SHIFT	52
#define HVM_REG_SIZE_MASK	GENMASK_ULL(55, 52)

/* bit field for HVM_IRQ_LINE */
#define HVM_IRQ_VCPU_MASK		0xff
#define HVM_IRQ_LINE_TYPE		GENMASK(27, 24)
#define HVM_IRQ_LINE_VCPU		GENMASK(23, 16)
#define HVM_IRQ_LINE_VCPU2		GENMASK(31, 28)
#define HVM_IRQ_LINE_NUM		GENMASK(15, 0)

/* Flags for irqfd */
#define HVM_IRQFD_FLAG_DEASSIGN	BIT(0)
#define HVM_IRQFD_FLAG_RESAMPLE	BIT(1)

/* Flags for ioeventfd */
#define HVM_IOEVENTFD_FLAG_DATAMATCH	BIT(0)
#define HVM_IOEVENTFD_FLAG_PIO		BIT(1)
#define HVM_IOEVENTFD_FLAG_DEASSIGN	BIT(2)
#define HVM_IOEVENTFD_VALID_FLAG_MASK	GENMASK(2, 0)

/* Timer PPI */
#define HVM_IRQ_PPI_START_NUM		(16)

/* Halla dev name */
#define HALLA_KERNEL_DEV_NAME		"halla"

/* VM exit reason */
enum {
	HVM_EXIT_HALLA = 0x484c0000,
	HVM_EXIT_MMIO = 0x484c0001,
	HVM_EXIT_IRQ = 0x484c0002,
	HVM_EXIT_EXCEPTION = 0x484c0003,
	HVM_EXIT_INTERNAL_ERROR = 0x484c0004,
	HVM_EXIT_SYSTEM_EVENT = 0x484c0005,
	HVM_EXIT_SHUTDOWN = 0x484c0006,
	HVM_EXIT_IDLE = 0x484c0007,
	HVM_EXIT_SGI = 0x484c0008,
};

/* exception definitions of HVM_EXIT_EXCEPTION */
enum {
	HVM_EXCEPTION_UNKNOWN = 0x0,
	HVM_EXCEPTION_PAGE_FAULT = 0x1,
};

/**
 * struct hvm_userspace_memory_region: For HVM_SET_USER_MEMORY_REGION.
 */
struct hvm_userspace_memory_region {
	__u32 slot;
	__u32 flags;
	__u64 guest_phys_addr;
	__u64 memory_size;
	__u64 userspace_addr;
};

/**
 * struct hvm_irq_level: For HVM_IRQ_LINE
 */
struct hvm_irq_level {
	__u32 irq;
	__u32 level;
};

/**
 * struct hvm_create_device: For HVM_CREATE_DEVICE.
 */
struct hvm_create_device {
	__u32 dev_type;
	__u32 id;
	__u64 flags;
	__u64 dev_addr;
	__u64 dev_reg_size;
	__u64 attr_addr;
	__u64 attr_size;
};

/**
 * struct hvm_irqfd: For HVM_IRQFD
 */
struct hvm_irqfd {
	__u32 fd;
	__u32 gsi;
	__u32 flags;
	__u32 resamplefd;
	__u8  pad[16];
};

/**
 * struct hvm_ioeventfd: For HVM_IOEVENTFD
 */
struct hvm_ioeventfd {
	__u64 datamatch;
	/* private: legal pio/mmio address */
	__u64 addr;
	/* private: 1, 2, 4, or 8 bytes; or 0 to ignore length */
	__u32 len;
	__s32 fd;
	__u32 flags;
	__u8  pad[36];
};

/**
 * struct hvm_enable_cap: For HVM_ENABLE_CAP
 */
struct hvm_enable_cap {
	__u64 cap;
	__u64 args[5];
};

/**
 * struct hvm_dtb_config: For HVM_SET_DTB_CONFIG
 */
struct hvm_dtb_config {
	__u64 dtb_addr;
	__u64 dtb_size;
};

/**
 * Structures for guest exit reason
 * This is used in hvm_vcpu_run structure's union
 */
/* HVM_EXIT_MMIO */
struct mmio {
	/* From FAR_EL2 */
	/* The address guest tries to access */
	__u64 phys_addr;
	/* The value to be written (is_write is 1) or
	 * be filled by user for reads (is_write is 0)
	 */
	__u8 data[8];
	/* From ESR_EL2 as */
	/* The size of written data.
	 * Only the first `size` bytes of `data` are handled
	 */
	__u64 size;
	/* From ESR_EL2 */
	/* The register number where the data is stored */
	__u32 reg_nr;
	/* From ESR_EL2 */
	/* 1 for VM to perform a write or 0 for VM to perform a read */
	__u8 is_write;
};

/* HVM_EXIT_EXCEPTION */
struct exception {
	/* Which exception vector */
	__u32 exception;
	/* Exception error codes */
	__u32 error_code;
	/* Fault GPA (guest physical address or IPA in ARM) */
	__u64 fault_gpa;
	/* Future-proof reservation and reset to zero in hypervisor.
	 * Fill up to the union size, 256 bytes.
	 */
	__u64 reserved[30];
};

/* System Event types */
enum {
	SYSTEM_EVENT_SHUTDOWN = 1,
	SYSTEM_EVENT_RESET,
	SYSTEM_EVENT_CRASH,
};

/* HVM_EXIT_SYSTEM_EVENT */
struct system_event {
	/* System event type.
	 * Ex. HVM_SYSTEM_EVENT_SHUTDOWN or HVM_SYSTEM_EVENT_RESET...etc.
	 */
	__u32 type;
	/* The number of elements used in data[] */
	__u32 ndata;
	/* Keep the detailed information about HVM_EXIT_SYSTEM_EVENT */
	__u64 data[16];
};

/**
 * struct hvm_vcpu_run: For HVM_VCPU_RUN
 * This struct is shared between Crosvm, kernel and Halla
 */
struct hvm_vcpu_run {
	/* to userspace */
	__u32 exit_reason;
	__u8 immediate_exit;
	__u8 pad[3];
	/* union structure of collection of guest exit reason */
	union {
		struct mmio mmio;
		struct exception exception;
		struct system_event system_event;
		char padding[256];
	};
};

/**
 * struct hvm_one_reg: For HVM_GET/SET_ONE_REG
 */
struct hvm_one_reg {
	__u64 id;
	__u64 addr;
};


/* Data structure for kernel driver and Halla */
#define GIC_V3_NR_LRS			16

#define HVM_VCPU_MMAP_SIZE  PAGE_SIZE
#define INVALID_VM_ID   0xffff

/*
 * The following data structures are for data transferring between driver and
 * hypervisor, and they're aligned with hypervisor definitions
 */
#define HVM_MAX_VCPUS		10
#define HVM_MAX_MEM_REGION	10

#define HVM_VCPU_RUN_MAP_SIZE		(PAGE_SIZE * 2)

#define HVM_MAX_VM_PGTABLE_SIZE		(64)

#define VM_S2_PTDUMP_DSS_NAME		"log_vm_s2pt"

/* structure for ioevent */
struct hvm_ioevent {
	struct list_head list;
	u64 addr;
	u32 len;
	struct eventfd_ctx  *evt_ctx;
	u64 datamatch;
	bool wildcard;
};

/* structures for memory region */
struct mem_region_addr_range {
	u64 address;
	u32 pg_cnt;
	u32 reserved;
};

struct hvm_memory_region_ranges {
	u32 slot;
	u32 constituent_cnt;
	u64 total_pages;
	u64 gpa;
	struct mem_region_addr_range constituents[];
};

/* structure for ppage */
struct hvm_pinned_page {
	struct rb_node node;
	struct page *page;
	u64 ipa;
};

/* vm debug types */
enum {
	HVM_DEBUG_TRANSLATE_IPA2PA,
	HVM_DEBUG_IS_SHARED_PAGE,
};

/* structure for vm debug */
struct hvm_vm_debug {
	u64 ipa_requested;
	u64 pa_translated;
};

/*
 * A reasonable and large enough limit for the maximum number of pages a
 * guest can use.
 */
#define HVM_MEM_MAX_NR_PAGES		((1UL << 31) - 1)

#define HVM_RECLAIM_MIN_SIZE		(0x10000)

/* structure for VM's memory slot */
struct hvm_memslot {
	u64 base_gfn;
	unsigned long npages;
	unsigned long userspace_addr;
	struct vm_area_struct *vma;
	u32 flags;
	u32 slot_id;
};

/* structure for VM's virt timer */
struct hvm_vtimer {
	u64 cntp_ctl_el0;
	u64 cntp_cval_el0;
	u64 cntp_tval_el0;
};

/* structure for management of vcpus */
struct hvm_vcpu {
	struct hvm *hvm;
	u32 vcpuid;
	/* lock of vcpu*/
	struct mutex lock;
	struct hvm_vcpu_run *run;
	struct rcuwait wait;

	struct hvm_vtimer vtimer;
	struct hrtimer idle_hrtimer;
	atomic_t idle_state;		// >=1 need to wakeup
};

/**
 * structure for management of VMs
 */
struct hvm {
	struct hvm_vcpu *vcpus[HVM_MAX_VCPUS];
	struct mm_struct *mm;
	struct hvm_memslot memslot[HVM_MAX_MEM_REGION];
	struct mutex lock;

	struct {
		spinlock_t        lock;
		struct list_head  items;
		struct list_head  resampler_list;
		struct mutex      resampler_lock;
	} irqfds;

	struct list_head ioevents;
	struct mutex ioevent_lock;

	struct list_head vm_list;
	u16 vm_id;

	struct srcu_struct irq_srcu;

	struct rb_root pinned_pages;
	struct mutex mem_lock;

	unsigned long *s2table_buf;

	/* Each pointer points to PAGE_SIZE * 2 of memory */
	unsigned long *vm_pgtable[HVM_MAX_VM_PGTABLE_SIZE];
	unsigned int pgtable_num;

	struct hvm_vm_debug vm_debug;

	struct {
		u32 mult;
		u32 shift;
	} clock_scale_factor;
	u32 nr_vcpus;
};

void hvm_vtimer_init_regs(struct hvm_vcpu *vcpu);
void hvm_restore_vtimer(struct hvm_vcpu *vcpu);
void hvm_save_vtimer(struct hvm_vcpu *vcpu);
u32 __percpu *hvm_get_running_vcpus(void);
u32 hvm_get_guest_vtimer_irq(void);
int hvm_vtimer_init(struct device *dev);
void hvm_vtimer_exit(void);

int hvm_dev_ioctl_create_vm(unsigned long vm_type);

void hvm_destroy_all_vms(void);

void hvm_destroy_vcpus(struct hvm *hvm);

int hvm_check_extension(struct hvm *hvm,
			u64 cap,
			void __user *argp);

void *hvm_alloc_pages(size_t size);
void hvm_free_pages(void *virt, size_t size);
int hvm_init_vm_stage2_mmu(struct hvm *hvm);

int hvm_vm_ioctl_create_vcpu(struct hvm *hvm, u32 cpuid);

int hvm_handle_page_fault(struct hvm_vcpu *vcpu);

void hvm_notify_acked_irq(struct hvm *hvm,
			  unsigned int gsi);
int hvm_irqfd(struct hvm *hvm,
	      struct hvm_irqfd *args);
int hvm_drv_irqfd_init(void);
void hvm_drv_irqfd_exit(void);
int hvm_vm_irqfd_init(struct hvm *hvm);
void hvm_vm_irqfd_release(struct hvm *hvm);

int hvm_init_ioeventfd(struct hvm *hvm);
int hvm_ioeventfd(struct hvm *hvm,
		  struct hvm_ioeventfd *args);
bool hvm_ioevent_write(struct hvm_vcpu *vcpu,
		       u64 addr,
		       int len,
		       const void *val);
int hvm_inject_irq(struct hvm *hvm,
		   unsigned int vcpu_idx,
		   u32 irq,
		   bool level);

void eventfd_ctx_do_read(struct eventfd_ctx *ctx,
			 u64 *cnt);
struct vm_area_struct *vma_lookup(struct mm_struct *mm,
				  unsigned long addr);
void add_wait_queue_priority(struct wait_queue_head *wq_head,
			     struct wait_queue_entry *wq_entry);
void hvm_vcpu_wakeup(struct hvm_vcpu *vcpu);

/* hvm debug */
int hvm_drv_debug_init(void);
void hvm_drv_debug_exit(void);
int hvm_create_vm_debugfs(struct hvm *hvm);
int hvm_destroy_vm_debugfs(struct hvm *hvm);

#endif /* __HVM_DRV_H__ */
