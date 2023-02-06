/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 NXP
 */

#ifndef __DCP_BLOB_H__
#define __DCP_BLOB_H__

#define MAX_KEY_SIZE                    128
#define MAX_BLOB_SIZE                   512

struct dcp_key_payload {
	unsigned int key_len;
	unsigned int blob_len;
	unsigned char *key;
	unsigned char *blob;
 };

int mxs_dcp_blob_to_key(struct dcp_key_payload *p);

#endif
