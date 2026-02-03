/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_DEBUG_H
#define __PKVM_X86_DEBUG_H

#include <linux/smp.h>

#define pkvm_err(f, x...)		pr_err("pkvm [CPU%d]: " f, raw_smp_processor_id(), ## x)
#define pkvm_err_ratelimited(f, x...)	pr_err_ratelimited("pkvm [CPU%d]: " f, \
							   raw_smp_processor_id(), ## x)

#endif /* __PKVM_X86_DEBUG_H */
