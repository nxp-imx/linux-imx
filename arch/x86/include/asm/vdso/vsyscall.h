/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_VDSO_VSYSCALL_H
#define __ASM_VDSO_VSYSCALL_H

#ifdef CONFIG_X86_64
#define VDSO_PAGE_SIZE_PER_MAX_PAGE	(__MAX_PAGE_SIZE / PAGE_SIZE)
#else
#define VDSO_PAGE_SIZE_PER_MAX_PAGE	1
#endif
#define __VDSO_PAGES			(6 * VDSO_PAGE_SIZE_PER_MAX_PAGE)

#define VDSO_NR_VCLOCK_PAGES		(2 * VDSO_PAGE_SIZE_PER_MAX_PAGE)
#define VDSO_VCLOCK_PAGES_START(_b)	((_b) + (__VDSO_PAGES - VDSO_NR_VCLOCK_PAGES) * PAGE_SIZE)
#define VDSO_PAGE_PVCLOCK_OFFSET	0
#define VDSO_PAGE_HVCLOCK_OFFSET	VDSO_PAGE_SIZE_PER_MAX_PAGE

#ifndef __ASSEMBLER__

#include <vdso/datapage.h>
#include <asm/vgtod.h>

/* The asm-generic header needs to be included after the definitions above */
#include <asm-generic/vdso/vsyscall.h>

#endif /* !__ASSEMBLER__ */

#endif /* __ASM_VDSO_VSYSCALL_H */
