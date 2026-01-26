// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/smp.h>
#include <linux/percpu.h>
#include <linux/cpuhotplug.h>
#include <clocksource/arm_arch_timer.h>
#include <asm/sysreg.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include "exynos-hvc.h"
#include "hvm_drv.h"

static u32 host_vtimer_irq;
static u32 guest_vtimer_irq;
static u32 host_vtimer_irq_flags;

// hvm_running_vcpu(u32) : vmid(bit 31~16), vcpuid(bit 15~0)
static u32 __percpu *hvm_running_vcpu;

void hvm_vtimer_init_regs(struct hvm_vcpu *vcpu)
{
	vcpu->vtimer.cntp_ctl_el0 = (u64)0;
	vcpu->vtimer.cntp_cval_el0 = (u64)0;
}

void hvm_restore_vtimer(struct hvm_vcpu *vcpu)
{
	unsigned long flags;

	local_irq_save(flags);

	write_sysreg(vcpu->vtimer.cntp_cval_el0, cntp_cval_el0);
	write_sysreg(vcpu->vtimer.cntp_ctl_el0, cntp_ctl_el0);
	isb();

	local_irq_restore(flags);
}

void hvm_save_vtimer(struct hvm_vcpu *vcpu)
{
	unsigned long flags;

	local_irq_save(flags);

	vcpu->vtimer.cntp_ctl_el0 = read_sysreg(cntp_ctl_el0);
	vcpu->vtimer.cntp_cval_el0 = read_sysreg(cntp_cval_el0);

	write_sysreg(0, cntp_ctl_el0);
	isb();

	local_irq_restore(flags);
}

u32 __percpu *hvm_get_running_vcpus(void)
{
	return (u32 __percpu *)hvm_running_vcpu;
}

u32 hvm_get_guest_vtimer_irq(void)
{
	return guest_vtimer_irq;
}

static irqreturn_t hvm_arch_timer_handler(int irq, void *dev_id)
{
	unsigned long ret;
	u32 tuple = *(u32 *)dev_id;
	u32 timer_ppi = guest_vtimer_irq - HVM_IRQ_PPI_START_NUM;

	if (tuple == (u32)(~0) || (read_sysreg(cntp_ctl_el0) & (u64)(2)))
		return IRQ_NONE;

	// mask timer
	write_sysreg(read_sysreg(cntp_ctl_el0) | (u64)(2), cntp_ctl_el0);
	isb();

	// call IRQ_LINE HVC
	ret = exynos_hvc(HVC_FID_HVM_IRQ_LINE, tuple,
			((u32)(1) << 31) | timer_ppi, true, 0);

	return IRQ_HANDLED;
}

static int hvm_vtimer_starting_cpu(u32 cpu)
{
	enable_percpu_irq(host_vtimer_irq, host_vtimer_irq_flags);

	return 0;
}

static int hvm_vtimer_dying_cpu(u32 cpu)
{
	disable_percpu_irq(host_vtimer_irq);

	return 0;
}

static void hvm_irq_fixup_flags(u32 virq, u32 *flags)
{
	*flags = irq_get_trigger_type(virq);
	if (*flags != IRQF_TRIGGER_HIGH && *flags != IRQF_TRIGGER_LOW)
		*flags = IRQF_TRIGGER_LOW;
}

int hvm_vtimer_init(struct device *dev)
{
	int err;
	// mapping INTID 27 to Linux irq (host_vtimer_irq)
	struct device_node *vtimer_node = of_find_node_by_path("/timer");
	struct irq_data *irqd;

	// 2: INTID 30(PPI 14) index in timer dt
	host_vtimer_irq = (u32)of_irq_get(vtimer_node, ARCH_TIMER_PHYS_NONSECURE_PPI);
	hvm_irq_fixup_flags(host_vtimer_irq, &host_vtimer_irq_flags);

	irqd = irq_get_irq_data(host_vtimer_irq);
	if (!irqd) {
		dev_err(dev, "Fail to get vtimer irq data (%d)\n",
				host_vtimer_irq);
		return IRQ_NONE;
	}

	guest_vtimer_irq = irqd_to_hwirq(irqd);

	hvm_running_vcpu = alloc_percpu(u32);
	if (!hvm_running_vcpu) {
		dev_err(dev, "memory allocation failed for running_vcpu\n");
		return -ENOMEM;
	}

	// register EL1 virtual timer irq
	err = request_percpu_irq(host_vtimer_irq, hvm_arch_timer_handler,
				"hvm guest vtimer", hvm_get_running_vcpus());
	if (err) {
		dev_err(dev, "can't request vtimer interrupt %d (%d)\n",
				host_vtimer_irq, err);
		return err;
	}

	err = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "exynos/arch_timer:starting",
			hvm_vtimer_starting_cpu, hvm_vtimer_dying_cpu);
	if (err < 0)
		dev_err(dev, "can't setup cpuhp state (%d)\n", err);

	return 0;
}

void hvm_vtimer_exit(void)
{
	free_percpu_irq(host_vtimer_irq, hvm_get_running_vcpus());

	free_percpu(hvm_running_vcpu);
}
