/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __PKVM_GUEST_H__
#define __PKVM_GUEST_H__

#ifdef CONFIG_PM_GENERIC_DOMAINS
int pkvm_device_pm_init(void);
#else
static inline int pkvm_device_pm_init(void) { return 0; }
#endif

#endif /* __PKVM_GUEST_H__ */
