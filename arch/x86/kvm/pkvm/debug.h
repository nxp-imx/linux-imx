/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_DEBUG_H
#define __PKVM_X86_DEBUG_H

#include <linux/smp.h>

#define pkvm_err(f, x...)		pr_err("pkvm [CPU%d]: " f, raw_smp_processor_id(), ## x)
#define pkvm_err_ratelimited(f, x...)	pr_err_ratelimited("pkvm [CPU%d]: " f, \
							   raw_smp_processor_id(), ## x)
#define pkvm_warn(f, x...)		pr_warn("pkvm [CPU%d]: " f, raw_smp_processor_id(), ## x)
#define pkvm_info(f, x...)		pr_info("pkvm [CPU%d]: " f, raw_smp_processor_id(), ## x)

#ifdef CONFIG_PKVM_X86_DEBUG
#define pkvm_dbg(f, x...)		pr_debug("pkvm [CPU%d]: " f, raw_smp_processor_id(), ## x)
#else
#define pkvm_dbg(f, x...)		no_printk(KERN_DEBUG pr_fmt(f), ## x)
#endif

#endif /* __PKVM_X86_DEBUG_H */
