/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_PANIC_H
#define __PKVM_X86_PANIC_H

extern atomic_t pkvm_panic_in_progress;

void __noreturn pkvm_panic(const char *fmt, ...);

#endif /* __PKVM_X86_PANIC_H */
