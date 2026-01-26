// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 */

#include <linux/anon_inodes.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/preempt.h>
#include <linux/percpu.h>
#include <linux/rcuwait.h>

#include "exynos-hvc.h"
#include "hvm_drv.h"

/* maximum size needed for holding an integer */
#define ITOA_MAX_LEN 12

static long hvm_vcpu_update_one_reg(struct hvm_vcpu *vcpu,
				     void __user *argp,
				     bool is_write)
{
	struct hvm_one_reg reg;
	void __user *reg_addr;
	u64 data = 0;
	u64 reg_size;
	long ret;

	if (copy_from_user(&reg, argp, sizeof(reg)))
		return -EFAULT;

	reg_addr = u64_to_user_ptr(reg.addr);
	reg_size = (reg.id & HVM_REG_SIZE_MASK) >> HVM_REG_SIZE_SHIFT;
	reg_size = BIT(reg_size);

	if (reg_size != 1 && reg_size != 2 && reg_size != 4 && reg_size != 8)
		return -EINVAL;

	if (is_write) {
		/* Halla hypervisor would filter out invalid vcpu register access */
		if (copy_from_user(&data, reg_addr, reg_size))
			return -EFAULT;

		ret = exynos_hvc(HVC_FID_HVM_SET_ONE_REG,
				 vcpu->hvm->vm_id, vcpu->vcpuid,
				 reg.id, data);
	} else {
		/* Read is not supported yet */
		return -EOPNOTSUPP;
	}

	if (ret)
		return ret;

	return 0;
}

/**
 * hvm_vcpu_handle_mmio() - Handle mmio in kernel space.
 * @vcpu: Pointer to vcpu.
 *
 * Return:
 * * true - This mmio exit has been processed.
 * * false - This mmio exit has not been processed, require userspace.
 */
static bool hvm_vcpu_handle_mmio(struct hvm_vcpu *vcpu)
{
	u64 addr;
	u32 len;
	const void *val_ptr;

	/* So far, we don't have in-kernel mmio read handler */
	if (!vcpu->run->mmio.is_write)
		return false;
	addr = vcpu->run->mmio.phys_addr;
	len = vcpu->run->mmio.size;
	val_ptr = &vcpu->run->mmio.data;

	return hvm_ioevent_write(vcpu, addr, len, val_ptr);
}

/**
 * hvm_handle_guest_exception() - Handle guest exception
 * @vcpu: Pointer to struct hvm_vcpu_run in userspace
 * Return:
 * * true - This exception has been processed, no need to back to VMM.
 * * false - This exception has not been processed, require userspace.
 */
static bool hvm_handle_guest_exception(struct hvm_vcpu *vcpu)
{
	int ret;

	for (int i = 0; i < ARRAY_SIZE(vcpu->run->exception.reserved); i++) {
		if (vcpu->run->exception.reserved[i])
			return false;
	}

	switch (vcpu->run->exception.exception) {
	case HVM_EXCEPTION_PAGE_FAULT:
		ret = hvm_handle_page_fault(vcpu);
		break;
	case HVM_EXCEPTION_UNKNOWN:
		fallthrough;
	default:
		ret = -EFAULT;
	}

	if (!ret)
		return true;
	else
		return false;
}

static void hvm_vcpu_load(struct hvm_vcpu *vcpu)
{
	u32 tuple = ((u32)vcpu->hvm->vm_id << 16 | (u16)vcpu->vcpuid);

	__this_cpu_write(*hvm_get_running_vcpus(), tuple);

	hvm_restore_vtimer(vcpu);
}

static void hvm_vcpu_put(struct hvm_vcpu *vcpu)
{
	hvm_save_vtimer(vcpu);

	__this_cpu_write(*hvm_get_running_vcpus(), (u32)(~0));
}

void hvm_vcpu_wakeup(struct hvm_vcpu *vcpu)
{
	atomic_inc(&vcpu->idle_state);
	rcuwait_wake_up(&vcpu->wait);
}

static enum hrtimer_restart hvm_idle_hrtimer_expire(struct hrtimer *hrt)
{
	struct hvm_vcpu *vcpu;

	vcpu = container_of(hrt, struct hvm_vcpu, idle_hrtimer);

	hvm_vcpu_wakeup(vcpu);

	return HRTIMER_NORESTART;
}

static void hvm_idle_hrtimer_init(struct hvm_vcpu *vcpu)
{
	hrtimer_init(&vcpu->idle_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_HARD);
	vcpu->idle_hrtimer.function = hvm_idle_hrtimer_expire;
	atomic_set(&vcpu->idle_state, 0);
}

static void hvm_idle_hrtimer_set(struct hvm_vcpu *vcpu, u64 ns)
{
	hrtimer_start(&vcpu->idle_hrtimer, ktime_add_ns(ktime_get(), ns), HRTIMER_MODE_ABS_HARD);
}

static void hvm_idle_hrtimer_release(struct hvm_vcpu *vcpu)
{
	hrtimer_cancel(&vcpu->idle_hrtimer);
}

static void vcpu_block_wait(struct hvm_vcpu *vcpu)
{
	struct rcuwait *wait = &vcpu->wait;

	rcuwait_wait_event(wait,
			   atomic_read(&vcpu->idle_state),
			   TASK_INTERRUPTIBLE);
	atomic_set(&vcpu->idle_state, 0);
}

static int hvm_handle_guest_idle(struct hvm_vcpu *vcpu)
{
	int ret = 0;
	u64 ns;
	u64 next_cval;
	u64 cur_val;

	ns = exynos_hvc(HVC_FID_HYPERVISOR_GET_VM_IDLE_TIME,
			((vcpu->hvm->vm_id << 16) | vcpu->vcpuid),
			0, 0, 0);
	next_cval = vcpu->vtimer.cntp_cval_el0;
	cur_val = read_sysreg(cntpct_el0);

	if (next_cval > cur_val) {
		ns = clocksource_cyc2ns(next_cval - cur_val,
				vcpu->hvm->clock_scale_factor.mult,
				vcpu->hvm->clock_scale_factor.shift);
	}

	hvm_idle_hrtimer_set(vcpu, ns);
	vcpu_block_wait(vcpu);
	hvm_idle_hrtimer_release(vcpu);

	return ret;
}

static void hvm_handle_guest_gsi(struct hvm_vcpu *vcpu)
{
	struct hvm *hvm = vcpu->hvm;

	for (int i = 0; i < hvm->nr_vcpus; i++) {
		if (hvm->vcpus[i])
			hvm_vcpu_wakeup(hvm->vcpus[i]);
	}
}

static bool handle_guest_exit(struct hvm_vcpu *vcpu, u64 exit_reason)
{
	switch (exit_reason) {
	case HVM_EXIT_MMIO:
		if (!hvm_vcpu_handle_mmio(vcpu))
			return true;
		break;
	/**
	 * it's Halla's responsibility to fill corresponding data
	 * structure
	 */
	case HVM_EXIT_EXCEPTION:
		if (!hvm_handle_guest_exception(vcpu))
			return true;
		break;
	case HVM_EXIT_HALLA:
		fallthrough;
	case HVM_EXIT_IRQ:
		break;
	case HVM_EXIT_IDLE:
		hvm_handle_guest_idle(vcpu);
		break;
	case HVM_EXIT_SGI:
		hvm_handle_guest_gsi(vcpu);
		break;
	default:
		return true;
	}

	return false;
}

/**
 * hvm_vcpu_run() - Handle vcpu run ioctl, entry point to guest and exit
 *		     point from guest
 * @vcpu: Pointer to struct hvm_vcpu
 * @argp: Pointer to struct hvm_vcpu_run in userspace
 *
 * Return:
 * * 0			- Success.
 * * Negative		- Failure.
 */
static long hvm_vcpu_run(struct hvm_vcpu *vcpu, void __user *argp)
{
	bool need_userspace = false;
	u64 exit_reason;

	if (copy_from_user(vcpu->run, argp, sizeof(struct hvm_vcpu_run)))
		return -EFAULT;

	for (int i = 0; i < ARRAY_SIZE(vcpu->run->pad); i++) {
		if (vcpu->run->pad[i])
			return -EINVAL;
	}

	if (vcpu->run->immediate_exit == 1)
		return -EINTR;

	while (!need_userspace && !signal_pending(current)) {
		preempt_disable();
		hvm_vcpu_load(vcpu);

		exit_reason = exynos_hvc(HVC_FID_HVM_VCPU_RUN,
				vcpu->hvm->vm_id, vcpu->vcpuid, 0, 0);

		/**
		 * Need to give guarantee for processing VM timer
		 * interrupt handler.
		 */
		isb();

		hvm_vcpu_put(vcpu);
		preempt_enable();

		need_userspace = handle_guest_exit(vcpu, exit_reason);
	}

	if (copy_to_user(argp, vcpu->run, sizeof(struct hvm_vcpu_run)))
		return -EFAULT;

	if (signal_pending(current)) {
		exynos_hvc(HVC_FID_HVM_NOTIFY_EXIT,
				vcpu->hvm->vm_id, 0, 0, 0);
		return -ERESTARTSYS;
	}

	return 0;
}

static long hvm_vcpu_ioctl(struct file *filp, unsigned int ioctl,
			    unsigned long arg)
{
	int ret = -EOPNOTSUPP;
	void __user *argp = (void __user *)arg;
	struct hvm_vcpu *vcpu = filp->private_data;

	switch (ioctl) {
	case HVM_VCPU_RUN:
		ret = hvm_vcpu_run(vcpu, argp);
		break;
	case HVM_SET_ONE_REG:
		/* is_write */
		ret = hvm_vcpu_update_one_reg(vcpu, argp, true);
		break;
	default:
		break;
	}

	return ret;
}

static const struct file_operations hvm_vcpu_fops = {
	.unlocked_ioctl = hvm_vcpu_ioctl,
};

/* caller must hold the vm lock */
static void hvm_destroy_vcpu(struct hvm_vcpu *vcpu)
{
	if (!vcpu)
		return;

	/* Destroy vcpu in Halla */
	exynos_hvc(HVC_FID_HVM_DESTROY_VCPU,
		   ((vcpu->hvm->vm_id << 16) | vcpu->vcpuid),
		   0, 0, 0);

	/*
	 * Clear the shared memory region. This is done to prevent any sensitive
	 * data from the guest or hypervisor from lingering in the memory pages
	 * after the vCPU is destroyed, improving security and preventing
	 * potential information leaks.
	 */
	memset(vcpu->run, 0, HVM_VCPU_RUN_MAP_SIZE);
	free_pages_exact(vcpu->run, HVM_VCPU_RUN_MAP_SIZE);
	kfree(vcpu);
}

/**
 * hvm_destroy_vcpus() - Destroy all vcpus, caller has to hold the vm lock
 *
 * @hvm: vm struct that owns the vcpus
 */
void hvm_destroy_vcpus(struct hvm *hvm)
{
	int i;

	for (i = 0; i < hvm->nr_vcpus; i++) {
		hvm_destroy_vcpu(hvm->vcpus[i]);
		hvm->vcpus[i] = NULL;
	}
	hvm->nr_vcpus = 0;
}

/* create_vcpu_fd() - Allocates an inode for the vcpu. */
static int create_vcpu_fd(struct hvm_vcpu *vcpu)
{
	/* sizeof("hvm-vcpu:") + max(strlen(itoa(vcpuid))) + null */
	char name[10 + ITOA_MAX_LEN + 1];

	snprintf(name, sizeof(name), "hvm-vcpu:%d", vcpu->vcpuid);
	return anon_inode_getfd(name, &hvm_vcpu_fops, vcpu, O_RDWR | O_CLOEXEC);
}

/**
 * hvm_vm_ioctl_create_vcpu() - for HVM_CREATE_VCPU
 * @hvm: Pointer to struct hvm
 * @cpuid: equals arg
 *
 * Return: Fd of vcpu, negative errno if error occurs
 */
int hvm_vm_ioctl_create_vcpu(struct hvm *hvm, u32 cpuid)
{
	struct hvm_vcpu *vcpu;
	int ret;

	if (cpuid > HVM_MAX_VCPUS)
		return -EINVAL;

	vcpu = kzalloc(sizeof(*vcpu), GFP_KERNEL);
	if (!vcpu)
		return -ENOMEM;

	/**
	 * Allocate 2 pages for data sharing between driver and halla hypervisor
	 *
	 * |- page 0           -|- page 1      -|
	 * |hvm_vcpu_run|......|not_used_yet|.......|
	 *
	 */
	vcpu->run = alloc_pages_exact(HVM_VCPU_RUN_MAP_SIZE,
				      GFP_KERNEL_ACCOUNT | __GFP_ZERO);
	if (!vcpu->run) {
		ret = -ENOMEM;
		goto free_vcpu;
	}
	vcpu->vcpuid = cpuid;
	vcpu->hvm = hvm;
	mutex_init(&vcpu->lock);

	ret = (int)exynos_hvc(HVC_FID_HVM_CREATE_VCPU,
			 hvm->vm_id, vcpu->vcpuid,
			 (u64)virt_to_phys(vcpu->run), 0);
	if (ret < 0)
		goto free_vcpu_run;

	ret = create_vcpu_fd(vcpu);
	if (ret < 0)
		goto free_vcpu_run;
	hvm->vcpus[cpuid] = vcpu;
	hvm->nr_vcpus += 1;

	hvm_vtimer_init_regs(vcpu);

	hvm_idle_hrtimer_init(vcpu);

	return ret;

free_vcpu_run:
	free_pages_exact(vcpu->run, HVM_VCPU_RUN_MAP_SIZE);
free_vcpu:
	kfree(vcpu);
	return ret;
}
