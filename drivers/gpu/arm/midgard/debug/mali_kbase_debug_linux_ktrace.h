/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2014-2025 ARM Limited. All rights reserved.
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

/*
 * NOTE: This must **only** be included through mali_linux_trace.h,
 * otherwise it will fail to setup tracepoints correctly
 */

#if !defined(_KBASE_DEBUG_LINUX_KTRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _KBASE_DEBUG_LINUX_KTRACE_H_

#if KBASE_KTRACE_TARGET_FTRACE

DECLARE_EVENT_CLASS(mali_add_template, TP_PROTO(struct kbase_context *kctx, u64 info_val),
		    TP_ARGS(kctx, info_val),
		    TP_STRUCT__entry(__field(pid_t, kctx_tgid) __field(u32, kctx_id)
					     __field(u64, info_val)),
		    TP_fast_assign(__entry->kctx_id = (kctx) ? kctx->id : 0u;
				   __entry->kctx_tgid = (kctx) ? kctx->tgid : 0;
				   __entry->info_val = info_val;),
		    TP_printk("kctx=%d_%u info=0x%llx", __entry->kctx_tgid, __entry->kctx_id,
			      __entry->info_val));

/* DEFINE_MALI_ADD_EVENT is available also to backends for backend-specific
 * simple trace codes
 */
#define DEFINE_MALI_ADD_EVENT(name)                  \
	DEFINE_EVENT(mali_add_template, mali_##name, \
		     TP_PROTO(struct kbase_context *kctx, u64 info_val), TP_ARGS(kctx, info_val))
DEFINE_MALI_ADD_EVENT(CORE_CTX_DESTROY);
DEFINE_MALI_ADD_EVENT(CORE_CTX_HWINSTR_TERM);
DEFINE_MALI_ADD_EVENT(CORE_GPU_IRQ);
DEFINE_MALI_ADD_EVENT(CORE_PWR_IRQ);
DEFINE_MALI_ADD_EVENT(CORE_WINDOW_IRQ);
DEFINE_MALI_ADD_EVENT(CORE_GPU_IRQ_CLEAR);
DEFINE_MALI_ADD_EVENT(CORE_GPU_IRQ_DONE);
DEFINE_MALI_ADD_EVENT(CORE_GPU_SOFT_RESET);
DEFINE_MALI_ADD_EVENT(CORE_GPU_HARD_RESET);
DEFINE_MALI_ADD_EVENT(CORE_GPU_PRFCNT_SAMPLE);
DEFINE_MALI_ADD_EVENT(CORE_GPU_PRFCNT_CLEAR);
DEFINE_MALI_ADD_EVENT(CORE_GPU_CLEAN_INV_CACHES);
DEFINE_MALI_ADD_EVENT(PM_CORES_CHANGE_DESIRED);
DEFINE_MALI_ADD_EVENT(PM_JOB_SUBMIT_AFTER_POWERING_UP);
DEFINE_MALI_ADD_EVENT(PM_JOB_SUBMIT_AFTER_POWERED_UP);
DEFINE_MALI_ADD_EVENT(PM_PWRON);
DEFINE_MALI_ADD_EVENT(PM_PWRON_TILER);
DEFINE_MALI_ADD_EVENT(PM_PWRON_L2);
DEFINE_MALI_ADD_EVENT(PM_PWROFF);
DEFINE_MALI_ADD_EVENT(PM_PWROFF_TILER);
DEFINE_MALI_ADD_EVENT(PM_PWROFF_L2);
DEFINE_MALI_ADD_EVENT(PM_CORES_POWERED);
DEFINE_MALI_ADD_EVENT(PM_CORES_POWERED_TILER);
DEFINE_MALI_ADD_EVENT(PM_CORES_POWERED_L2);
DEFINE_MALI_ADD_EVENT(PM_CORES_POWERED_STACK);
DEFINE_MALI_ADD_EVENT(PM_CORES_POWERED_BASE);
DEFINE_MALI_ADD_EVENT(PM_PWRON_NEURAL);
DEFINE_MALI_ADD_EVENT(PM_PWROFF_NEURAL);
DEFINE_MALI_ADD_EVENT(PM_CORES_POWERED_NEURAL);
DEFINE_MALI_ADD_EVENT(PM_DESIRED_REACHED);
DEFINE_MALI_ADD_EVENT(PM_DESIRED_REACHED_TILER);
DEFINE_MALI_ADD_EVENT(PM_REQUEST_CHANGE_SHADER_NEEDED);
DEFINE_MALI_ADD_EVENT(PM_REQUEST_CHANGE_TILER_NEEDED);
DEFINE_MALI_ADD_EVENT(PM_RELEASE_CHANGE_SHADER_NEEDED);
DEFINE_MALI_ADD_EVENT(PM_RELEASE_CHANGE_TILER_NEEDED);
DEFINE_MALI_ADD_EVENT(PM_CORES_AVAILABLE);
DEFINE_MALI_ADD_EVENT(PM_CORES_AVAILABLE_TILER);
DEFINE_MALI_ADD_EVENT(PM_CORES_CHANGE_AVAILABLE);
DEFINE_MALI_ADD_EVENT(PM_CORES_CHANGE_AVAILABLE_TILER);
DEFINE_MALI_ADD_EVENT(PM_CORES_CHANGE_AVAILABLE_L2);
DEFINE_MALI_ADD_EVENT(PM_GPU_ON);
DEFINE_MALI_ADD_EVENT(PM_GPU_OFF);
DEFINE_MALI_ADD_EVENT(PM_SET_POLICY);
DEFINE_MALI_ADD_EVENT(PM_CURRENT_POLICY_INIT);
DEFINE_MALI_ADD_EVENT(PM_CURRENT_POLICY_TERM);
DEFINE_MALI_ADD_EVENT(PM_CA_SET_POLICY);
DEFINE_MALI_ADD_EVENT(PM_CONTEXT_ACTIVE);
DEFINE_MALI_ADD_EVENT(PM_CONTEXT_IDLE);
DEFINE_MALI_ADD_EVENT(PM_WAKE_WAITERS);
DEFINE_MALI_ADD_EVENT(PM_POWEROFF_WAIT_WQ);
DEFINE_MALI_ADD_EVENT(PM_RUNTIME_SUSPEND_CALLBACK);
DEFINE_MALI_ADD_EVENT(PM_RUNTIME_RESUME_CALLBACK);
#define KBASEP_L2_STATE(n) DEFINE_MALI_ADD_EVENT(PM_L2_##n);
#include "backend/gpu/mali_kbase_pm_l2_states.h"
#undef KBASEP_L2_STATE
DEFINE_MALI_ADD_EVENT(SCHED_RETAIN_CTX_NOLOCK);
DEFINE_MALI_ADD_EVENT(SCHED_RELEASE_CTX);

DEFINE_MALI_ADD_EVENT(ARB_VM_STATE);
DEFINE_MALI_ADD_EVENT(ARB_VM_EVT);
DEFINE_MALI_ADD_EVENT(ARB_GPU_GRANTED);
DEFINE_MALI_ADD_EVENT(ARB_GPU_LOST);
DEFINE_MALI_ADD_EVENT(ARB_GPU_STARTED);
DEFINE_MALI_ADD_EVENT(ARB_GPU_STOP_REQUESTED);
DEFINE_MALI_ADD_EVENT(ARB_GPU_STOPPED);
DEFINE_MALI_ADD_EVENT(ARB_GPU_REQUESTED);

#include "backend/mali_kbase_debug_linux_ktrace_csf.h"

/* Memory map/unmap event class */
DECLARE_EVENT_CLASS(mali_mem_template,
		    TP_PROTO(struct kbase_context *kctx, size_t pages, unsigned long long va,
			     unsigned long long flags, pid_t tgid, int ctx_id, int as_nr),
		    TP_ARGS(kctx, pages, va, flags, tgid, ctx_id, as_nr),
		    TP_STRUCT__entry(__field(pid_t, tgid) __field(int, ctx_id) __field(int, as_nr)
					     __field(size_t, pages) __field(unsigned long long, va)
						     __field(unsigned long long, flags)),
		    TP_fast_assign(__entry->tgid = tgid; __entry->ctx_id = ctx_id;
				   __entry->as_nr = as_nr; __entry->pages = pages; __entry->va = va;
				   __entry->flags = flags;),
		    TP_printk("%zu pages at VA %#llx flags %#llx for ctx %d_%d as_nr %d",
			      __entry->pages, __entry->va, __entry->flags, __entry->tgid,
			      __entry->ctx_id, __entry->as_nr));

/* Instantiate individual MEM tracepoints from mali_mem_template               */
#define DEFINE_MALI_MEM_EVENT(name)                                                            \
	DEFINE_EVENT(mali_mem_template, mali_mem_##name,                                       \
		     TP_PROTO(struct kbase_context *kctx, size_t pages, unsigned long long va, \
			      unsigned long long flags, pid_t tgid, int ctx_id, int as_nr),    \
		     TP_ARGS(kctx, pages, va, flags, tgid, ctx_id, as_nr))

/* Creating MEM events: */
DEFINE_MALI_MEM_EVENT(MEM_MAPPED);
DEFINE_MALI_MEM_EVENT(MEM_UNMAPPED);

#undef DEFINE_MALI_ADD_EVENT

#endif /* KBASE_KTRACE_TARGET_FTRACE */

#endif /* !defined(_KBASE_DEBUG_LINUX_KTRACE_H_)  || defined(TRACE_HEADER_MULTI_READ) */
