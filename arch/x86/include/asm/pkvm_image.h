/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_X86_PKVM_IMAGE_H
#define _ASM_X86_PKVM_IMAGE_H

#include <linux/types.h>

#if defined(__PKVM_HYP__)
/*
 * For the pKVM hypervisor code to use the pKVM hypervisor symbols.
 * There is no need to manually add the suffix as the Makefile will
 * automatically do this for symbols in the pKVM hypervisor
 * text.
 */
#define PKVM_DECLARE(type, f, params)	type f params
#define pkvm_sym(sym)			sym
#else
/*
 * For the host code to use the pKVM hypervisor symbols which has
 * the __pkvm suffix added by the Makefile. It is necessary to manually
 * add the suffix as the Makefile will not do this for symbols in the
 * host text.
 */
#define PKVM_DECLARE(type, f, params)	type f##__pkvm params
#define pkvm_sym(sym)			sym##__pkvm
#endif

#ifdef LINKER_SCRIPT

#define PKVM_SECTION_NAME(NAME)	.pkvm##NAME

#define BEGIN_PKVM_SECTION(NAME)			\
	PKVM_SECTION_NAME(NAME) : {

#define END_PKVM_SECTION				\
	}

#define PKVM_SECTION(NAME)			\
	BEGIN_PKVM_SECTION(NAME)		\
		*(NAME NAME##.*)		\
	END_PKVM_SECTION

#endif /* LINKER_SCRIPT */

#ifndef __ASSEMBLER__

#ifdef CONFIG_PKVM_X86
extern char pkvm_sym(text_start)[], pkvm_sym(text_end)[];
extern char pkvm_sym(rodata_start)[], pkvm_sym(rodata_end)[];
extern char pkvm_sym(data_start)[], pkvm_sym(data_end)[];
extern char pkvm_sym(bss_start)[], pkvm_sym(bss_end)[];
struct exception_table_entry;
extern struct exception_table_entry pkvm_sym(__start___ex_table)[];
extern struct exception_table_entry pkvm_sym(__stop___ex_table)[];
static inline bool is_pkvm_text(void *addr)
{
	return addr >= (void *)pkvm_sym(text_start) && addr < (void *)pkvm_sym(text_end);
}
#else
static inline bool is_pkvm_text(void *addr) { return false; }
#endif

#endif

#endif /* _ASM_X86_PKVM_IMAGE_H */
