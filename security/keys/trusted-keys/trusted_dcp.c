// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 NXP
 */

#include <keys/trusted_dcp.h>
#include <keys/trusted-type.h>
#include <linux/build_bug.h>
#include <linux/key-type.h>
#include <soc/fsl/dcp-blob.h>

static int trusted_dcp_seal(struct trusted_key_payload *p, char *datablob)
{
	struct dcp_key_payload info = {
		.key  = p->key,  .blob   = p->blob,
		.key_len = p->key_len, .blob_len = p->blob_len,
	};
	int ret = 0;

	ret =  mxs_dcp_blob_to_key(&info);
	if (ret)
		pr_err("trusted DCP seal error :%d\n", ret);

	p->key_len = info.key_len;
	return ret;
}

static int trusted_dcp_unseal(struct trusted_key_payload *p, char *datablob)
{
	struct dcp_key_payload info = {
		.key  = p->key,  .blob   = p->blob,
		.key_len = p->key_len, .blob_len = p->blob_len,
	};
	int ret = 0;

	ret= mxs_dcp_blob_to_key(&info);
	if (ret)
		 pr_err("trusted DCP unseal error :%d\n", ret);
	p->key_len = info.key_len;
	return ret;
}

static int trusted_dcp_init(void)
{
	int ret;

	ret = register_key_type(&key_type_trusted);
	if (ret)
		printk(KERN_INFO "Trusted key not registered\n");

	return ret;
}

static void trusted_dcp_exit(void)
{
	unregister_key_type(&key_type_trusted);
}

struct trusted_key_ops trusted_key_dcp_ops = {
	.migratable = 0, /* non-migratable */
	.init = trusted_dcp_init,
	.seal = trusted_dcp_seal,
	.unseal = trusted_dcp_unseal,
	.exit = trusted_dcp_exit,
};
