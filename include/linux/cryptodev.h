/*
 * Driver for /dev/crypto device (aka CryptoDev)
 *
 * Copyright (c) 2004 Michal Ludvig <mludvig@suse.cz>, SuSE Labs
 *
 * Structures and ioctl command names were taken from
 * OpenBSD to preserve compatibility with their API.
 *
 */

#ifndef _CRYPTODEV_H
#define _CRYPTODEV_H

#ifndef __KERNEL__
#include <inttypes.h>
#endif

#define	CRYPTODEV_MINOR	MISC_DYNAMIC_MINOR

#define CRYPTO_FLAG_ECB		0x0000
#define CRYPTO_FLAG_CBC		0x0001
#define CRYPTO_FLAG_CFB		0x0002
#define CRYPTO_FLAG_OFB		0x0003
#define CRYPTO_FLAG_CTR		0x0004
#define CRYPTO_FLAG_HMAC	0x0010
#define CRYPTO_FLAG_MASK	0x00FF

#define	CRYPTO_CIPHER_NAME	0x0100
#define	CRYPTO_CIPHER_NAME_CBC	(CRYPTO_CIPHER_NAME | CRYPTO_FLAG_CBC)
#define	CRYPTO_HASH_NAME	0x0200
#define	CRYPTO_HASH_NAME_HMAC	(CRYPTO_HASH_NAME | CRYPTO_FLAG_HMAC)

/* ioctl parameter to create a session */
struct session_op {
	unsigned int	cipher;		/* e.g. CRYPTO_DES_CBC */
	unsigned int	mac;		/* e.g. CRYPTO_MD5_HMAC */
	char		*alg_name;	/* set cipher=CRYPTO_CIPHER_NAME
					   or  mac=CRYPTO_HASH_NAME */
	#define	MAX_ALG_NAME_LEN 128
	size_t		alg_namelen;

	size_t		keylen;		/* cipher key */
	char		*key;
	int		mackeylen;	/* mac key */
	char		*mackey;

	/* Return values */
	unsigned int	blocksize;	/* selected algorithm's block size */
	uint32_t	ses;		/* session ID */
};

/* ioctl parameter to request a crypt/decrypt operation against a session  */
struct crypt_op {
	uint32_t	ses;		/* from session_op->ses */
	#define COP_DECRYPT	0
	#define COP_ENCRYPT	1
	uint32_t	op;		/* ie. COP_ENCRYPT */
	uint32_t	flags;		/* unused */

	size_t		len;
	char		*src, *dst;
	char		*mac;
	char		*iv;
};

/* clone original filedescriptor */
#define CRIOGET         _IOWR('c', 100, uint32_t)

/* create crypto session */
#define CIOCGSESSION    _IOWR('c', 101, struct session_op)

/* finish crypto session */
#define CIOCFSESSION    _IOW('c', 102, uint32_t)

/* request encryption/decryptions of a given buffer */
#define CIOCCRYPT       _IOWR('c', 103, struct crypt_op)

/* ioctl()s for asym-crypto. Not yet supported. */
#define CIOCKEY         _IOWR('c', 104, void *)
#define CIOCASYMFEAT    _IOR('c', 105, uint32_t)

#endif /* _CRYPTODEV_H */
