/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __KVM_HYP_ERRNO_H
#define __KVM_HYP_ERRNO_H

#include <linux/errno.h>

enum {
	/* Recycle NFSv3 error codes */
	ENOMEMHYPALLOC = EBADHANDLE,
	ENOMEMHOSTS2,
};

#endif /* __KVM_HYP_ERRNO_H */
