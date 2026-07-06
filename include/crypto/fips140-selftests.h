/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CRYPTO_FIPS140_SELFTESTS_H
#define _CRYPTO_FIPS140_SELFTESTS_H

#include <linux/types.h>

#if defined(CONFIG_CRYPTO_FIPS140_MOD_EVAL_TESTING) && \
	(defined(BUILD_FIPS140_KO) || \
	 /* This handles fips140-eval-testing.c itself */ \
	 defined(NEED_FIPS140_EVAL_TESTING_DECLS))
void fips140_inject_selftest_failure(const char *impl, u8 *result);
#else
static inline void fips140_inject_selftest_failure(const char *impl, u8 *result)
{
}
#endif

#endif /* _CRYPTO_FIPS140_SELFTESTS_H */
