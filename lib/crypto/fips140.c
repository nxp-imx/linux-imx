// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2026 Google LLC */
#include <crypto/fips140-lib-overrides.h>

struct fips140_lib_funcs fips140_lib_funcs;
DEFINE_STATIC_KEY_FALSE(fips140_lib_funcs_loaded);

void register_fips140_lib_funcs(const struct fips140_lib_funcs *funcs)
{
	WARN_ON(static_key_enabled(&fips140_lib_funcs_loaded));
	fips140_lib_funcs = *funcs;
	static_branch_enable(&fips140_lib_funcs_loaded);
}
EXPORT_SYMBOL_GPL(register_fips140_lib_funcs);
