/*
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 */

#ifndef CAAM_COMPAT_H
#define CAAM_COMPAT_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/crypto.h>
#include <linux/hw_random.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/rtnetlink.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/debugfs.h>
#include <linux/circ_buf.h>

#ifdef CONFIG_OF
#include <linux/of_platform.h>
#else
#include <linux/platform_device.h>
#endif

#ifdef CONFIG_ARM /* needs the clock control subsystem */
#include <linux/clk.h>
#include <asm/cacheflush.h>
#endif

#include <net/xfrm.h>

#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/sha.h>
#include <crypto/md5.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/skcipher.h>
#include <crypto/internal/hash.h>

#endif /* !defined(CAAM_COMPAT_H) */
