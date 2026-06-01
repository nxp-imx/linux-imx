/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Tracepoints for the Gunyah vCPU driver.
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM gunyah_vcpu

#if !defined(_TRACE_GUNYAH_VCPU_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_GUNYAH_VCPU_H

#include <linux/tracepoint.h>

/*
 * gh_vcpu_timer_arm - about to arm the hrtimer and block on a completion.
 *
 * Emitted whenever the vCPU run loop arms the wakeup hrtimer before calling
 * wait_for_completion_interruptible(), regardless of which hypervisor state
 * triggered the wait.
 *
 * @vmid:           VM identifier
 * @vcpu_id:        vCPU index within the VM
 * @vcpu_state:     raw hypervisor vCPU state that caused the arm
 * @deadline_ticks: absolute deadline in system counter ticks as supplied by
 *                  the hypervisor
 * @delta_ns:       nanoseconds between now and the deadline
 * @expires_ns:     absolute CLOCK_MONOTONIC expiry programmed into the hrtimer
 */
TRACE_EVENT(gh_vcpu_timer_arm,
	TP_PROTO(u16 vmid, u32 vcpu_id, u64 vcpu_state,
		 u64 deadline_ticks, u64 delta_ns, s64 expires_ns),
	TP_ARGS(vmid, vcpu_id, vcpu_state, deadline_ticks, delta_ns, expires_ns),
	TP_STRUCT__entry(
		__field(u16, vmid)
		__field(u32, vcpu_id)
		__field(u64, vcpu_state)
		__field(u64, deadline_ticks)
		__field(u64, delta_ns)
		__field(s64, expires_ns)
	),
	TP_fast_assign(
		__entry->vmid           = vmid;
		__entry->vcpu_id        = vcpu_id;
		__entry->vcpu_state     = vcpu_state;
		__entry->deadline_ticks = deadline_ticks;
		__entry->delta_ns       = delta_ns;
		__entry->expires_ns     = expires_ns;
	),
	TP_printk("vmid=%u vcpu=%u vcpu_state=0x%llx deadline_ticks=%llu "
		  "delta_ns=%llu expires_ns=%lld",
		  __entry->vmid, __entry->vcpu_id, __entry->vcpu_state,
		  __entry->deadline_ticks, __entry->delta_ns, __entry->expires_ns)
);

/*
 * gh_vcpu_timer_deadline_passed - hypervisor deadline already elapsed on entry.
 *
 * Emitted when gunyah_arch_deadline_to_ktime() returns false because the deadline
 * tick count is already in the past. The hypercall will be re-entered
 * immediately without blocking.
 *
 * @vmid:           VM identifier
 * @vcpu_id:        vCPU index within the VM
 * @vcpu_state:     raw hypervisor vCPU state that carried the deadline
 * @deadline_ticks: absolute deadline in system counter ticks
 */
TRACE_EVENT(gh_vcpu_timer_deadline_passed,
	TP_PROTO(u16 vmid, u32 vcpu_id, u64 vcpu_state, u64 deadline_ticks),
	TP_ARGS(vmid, vcpu_id, vcpu_state, deadline_ticks),
	TP_STRUCT__entry(
		__field(u16, vmid)
		__field(u32, vcpu_id)
		__field(u64, vcpu_state)
		__field(u64, deadline_ticks)
	),
	TP_fast_assign(
		__entry->vmid           = vmid;
		__entry->vcpu_id        = vcpu_id;
		__entry->vcpu_state     = vcpu_state;
		__entry->deadline_ticks = deadline_ticks;
	),
	TP_printk("vmid=%u vcpu=%u vcpu_state=0x%llx deadline_ticks=%llu",
		  __entry->vmid, __entry->vcpu_id, __entry->vcpu_state,
		  __entry->deadline_ticks)
);

/*
 * gh_vcpu_timer_wake - woke from the completion wait after gh_vcpu_timer_arm.
 *
 * @vmid:             VM identifier
 * @vcpu_id:          vCPU index within the VM
 * @vcpu_state:       raw hypervisor vCPU state that originally caused the wait
 * @wait_ret:         return value of wait_for_completion_interruptible():
 *                      0            -> woken by VIRQ or timer callback
 *                      -ERESTARTSYS -> interrupted by a signal
 * @timer_was_active: true  -> hrtimer_cancel() found the timer still queued,
 *                             meaning a VIRQ woke us before the timer fired;
 *                    false -> timer had already fired (or was never active)
 */
TRACE_EVENT(gh_vcpu_timer_wake,
	TP_PROTO(u16 vmid, u32 vcpu_id, u64 vcpu_state,
		 int wait_ret, bool timer_was_active),
	TP_ARGS(vmid, vcpu_id, vcpu_state, wait_ret, timer_was_active),
	TP_STRUCT__entry(
		__field(u16,  vmid)
		__field(u32,  vcpu_id)
		__field(u64,  vcpu_state)
		__field(int,  wait_ret)
		__field(bool, timer_was_active)
	),
	TP_fast_assign(
		__entry->vmid             = vmid;
		__entry->vcpu_id          = vcpu_id;
		__entry->vcpu_state       = vcpu_state;
		__entry->wait_ret         = wait_ret;
		__entry->timer_was_active = timer_was_active;
	),
	TP_printk("vmid=%u vcpu=%u vcpu_state=0x%llx wait_ret=%d timer_was_active=%d",
		  __entry->vmid, __entry->vcpu_id, __entry->vcpu_state,
		  __entry->wait_ret, __entry->timer_was_active)
);

/*
 * gh_vcpu_timer_fired - hrtimer callback executed for this vCPU.
 *
 * Emitted from gunyah_vcpu_wakeup_timer_fn(). Together with gh_vcpu_timer_wake
 * this lets you distinguish a timer-driven wakeup (timer_fired present,
 * timer_was_active=false) from a VIRQ-driven one (timer_fired absent,
 * timer_was_active=true).
 *
 * @vmid:    VM identifier
 * @vcpu_id: vCPU index within the VM
 */
TRACE_EVENT(gh_vcpu_timer_fired,
	TP_PROTO(u16 vmid, u32 vcpu_id),
	TP_ARGS(vmid, vcpu_id),
	TP_STRUCT__entry(
		__field(u16, vmid)
		__field(u32, vcpu_id)
	),
	TP_fast_assign(
		__entry->vmid    = vmid;
		__entry->vcpu_id = vcpu_id;
	),
	TP_printk("vmid=%u vcpu=%u", __entry->vmid, __entry->vcpu_id)
);

#endif /* _TRACE_GUNYAH_VCPU_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/virt/gunyah
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace_vcpu

/* This part must be outside the header guard */
#include <trace/define_trace.h>
