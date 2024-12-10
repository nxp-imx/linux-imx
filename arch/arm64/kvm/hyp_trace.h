/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __ARM64_KVM_HYP_TRACE_H__
#define __ARM64_KVM_HYP_TRACE_H__

#include <asm/kvm_hyptrace.h>
#include <asm/kvm_hypevents_defs.h>

#ifdef CONFIG_TRACING
int hyp_trace_init_tracefs(void);
int hyp_trace_init_events(void);
struct hyp_event *hyp_trace_find_event(int id);
void hyp_trace_init_event_tracefs(struct dentry *parent);
int hyp_trace_init_mod_events(struct hyp_event *event,
			      struct hyp_event_id *event_id, int nr_events,
			      struct hyp_printk_fmt *fmt, int nr_fmts);
bool hyp_event_early_probe(void);
void hyp_trace_enable_event_early(void);
#else
static inline int hyp_trace_init_tracefs(void) { return 0; }
static inline int hyp_trace_init_events(void) { return 0; }
static inline int
hyp_trace_init_mod_events(struct hyp_event *event,
			  struct hyp_event_id *event_id, int nr_events,
			  struct hyp_printk_fmt *fmt, int nr_fmts)
{
	return 0;
}
static inline void hyp_trace_enable_event_early(void) { }
#endif
#endif
