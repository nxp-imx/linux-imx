/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_KVM_PKVM_REDEF_H
#define _ASM_X86_KVM_PKVM_REDEF_H

#ifdef __PKVM_HYP__

#undef kvm_err
#undef kvm_info
#undef kvm_debug
#undef kvm_debug_ratelimited
#undef kvm_pr_unimpl

#ifdef CONFIG_PKVM_X86_DEBUG

#define kvm_err(fmt, ...) \
	pr_err("pkvm: " fmt, ## __VA_ARGS__)
#define kvm_info(fmt, ...) \
	pr_info("pkvm: " fmt, ## __VA_ARGS__)
#define kvm_debug(fmt, ...) \
	pr_debug("pkvm: " fmt, ## __VA_ARGS__)
#define kvm_debug_ratelimited(fmt, ...) \
	pr_debug_ratelimited("pkvm: " fmt, ## __VA_ARGS__)
#define kvm_pr_unimpl(fmt, ...) \
	pr_err_ratelimited("pkvm: " fmt, ## __VA_ARGS__)

#else /* CONFIG_PKVM_X86_DEBUG */

#define kvm_err(fmt, ...) do {} while(0)
#define kvm_info(fmt, ...) do {} while(0)
#define kvm_debug(fmt, ...) do {} while(0)
#define kvm_debug_ratelimited(fmt, ...) do {} while(0)
#define kvm_pr_unimpl(fmt, ...) do {} while(0)

#undef WARN_ON
#undef WARN
#undef WARN_ON_ONCE
#undef WARN_ONCE

#define WARN_ON(condition) ({						\
	int __ret_warn_on = !!(condition);				\
	unlikely(__ret_warn_on);					\
})

#define WARN(condition, format...) ({					\
	int __ret_warn_on = !!(condition);				\
	no_printk(format);						\
	unlikely(__ret_warn_on);					\
})

#define WARN_ON_ONCE(condition) WARN_ON(condition)
#define WARN_ONCE(condition, format...) WARN(condition, format)

#endif /* CONFIG_PKVM_X86_DEBUG */

void __noreturn pkvm_panic(const char *fmt, ...);

/*
 * Directly call the panic handler with file/line info. This avoids the use
 * of 'ud2' instructions and associated 'bug_table' metadata parsing, which
 * would unnecessarily increase the TCB and complexity of the hypervisor's
 * emergency recovery path. This is also critical for production (non-debug)
 * environments where hypervisor doesn't have its own kallsyms or access to
 * the host's metadata.
 */
#undef BUG
#define BUG() do { pkvm_panic("\n==================================\n"	\
			      "pKVM BUG at %s:%u\n"			\
			      "==================================\n",	\
			       __FILE__, __LINE__);			\
			      __builtin_unreachable();			\
		} while (0)

#undef BUG_ON
#define BUG_ON(condition) do { if (unlikely(condition)) BUG(); } while (0)

#undef KVM_BUG_ON
#define KVM_BUG_ON(cond, kvm)						\
({									\
	bool __ret = !!(cond);						\
									\
	BUG_ON(__ret);							\
	unlikely(__ret);						\
})

#undef KVM_BUG
#define KVM_BUG(cond, kvm, fmt...)		KVM_BUG_ON(cond, kvm)

#endif /* __PKVM_HYP__ */

#endif /* _ASM_X86_KVM_PKVM_REDEF_H */
