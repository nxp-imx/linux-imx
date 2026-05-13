// SPDX-License-Identifier: GPL-2.0

#include <linux/compat.h>

#ifdef CONFIG_COMPAT
__rust_helper void __user *rust_helper_compat_ptr(compat_uptr_t uptr)
{
	return compat_ptr(uptr);
}
#endif
