/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_x86_PKVM_IMAGE_VARS_H
#define _ASM_x86_PKVM_IMAGE_VARS_H

#ifdef CONFIG_PKVM_X86_DEBUG

#include <asm/pkvm_image.h>
/*
 * Defines a linker script alias of a kernel-proper symbol referenced by
 * PKVM code.
 */
#define PKVM_ALIAS(sym)  pkvm_sym(sym) = sym;

PKVM_ALIAS(_printk);
PKVM_ALIAS(__warn_printk);
PKVM_ALIAS(___ratelimit);
PKVM_ALIAS(__trace_bputs);
PKVM_ALIAS(__trace_bprintk);
PKVM_ALIAS(__dynamic_pr_debug);
PKVM_ALIAS(mem_dump_obj);
PKVM_ALIAS(vmalloc_base);
PKVM_ALIAS(get_cpu_entry_area);
#endif

#endif /* _ASM_x86_PKVM_IMAGE_VARS_H */
