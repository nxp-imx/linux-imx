#ifndef _CRYPTO_FIPS140_LIB_OVERRIDES_H
#define _CRYPTO_FIPS140_LIB_OVERRIDES_H

#if !defined(__DISABLE_EXPORTS) || defined(BUILD_FIPS140_KO)

#include <crypto/aes.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <linux/jump_label.h>

/*
 * This struct contains a function pointer for each library function that can be
 * overridden by fips140.ko.  The name, parameter types, and return type of each
 * function pointer are the same as the corresponding library function.
 */
struct fips140_lib_funcs {
	/* lib/crypto/aes.c */
	int (*aes_expandkey)(struct crypto_aes_ctx *ctx, const u8 *in_key,
			     unsigned int key_len);
	void (*aes_encrypt)(const struct crypto_aes_ctx *ctx, u8 *out,
			    const u8 *in);
	void (*aes_decrypt)(const struct crypto_aes_ctx *ctx, u8 *out,
			    const u8 *in);

	/* lib/crypto/sha256.c */
	void (*sha224_init)(struct sha224_ctx *ctx);
	void (*sha256_init)(struct sha256_ctx *ctx);
	void (*__sha256_update)(struct __sha256_ctx *ctx, const u8 *data,
				size_t len);
	void (*sha224_final)(struct sha224_ctx *ctx,
			     u8 out[SHA224_DIGEST_SIZE]);
	void (*sha256_final)(struct sha256_ctx *ctx,
			     u8 out[SHA256_DIGEST_SIZE]);
	void (*sha224)(const u8 *data, size_t len, u8 out[SHA224_DIGEST_SIZE]);
	void (*sha256)(const u8 *data, size_t len, u8 out[SHA256_DIGEST_SIZE]);
	void (*sha256_finup_2x)(const struct sha256_ctx *ctx, const u8 *data1,
				const u8 *data2, size_t len,
				u8 out1[SHA256_DIGEST_SIZE],
				u8 out2[SHA256_DIGEST_SIZE]);
	bool (*sha256_finup_2x_is_optimized)(void);
	void (*hmac_sha224_preparekey)(struct hmac_sha224_key *key,
				       const u8 *raw_key, size_t raw_key_len);
	void (*hmac_sha256_preparekey)(struct hmac_sha256_key *key,
				       const u8 *raw_key, size_t raw_key_len);
	void (*__hmac_sha256_init)(struct __hmac_sha256_ctx *ctx,
				   const struct __hmac_sha256_key *key);
	void (*hmac_sha224_init_usingrawkey)(struct hmac_sha224_ctx *ctx,
					     const u8 *raw_key,
					     size_t raw_key_len);
	void (*hmac_sha256_init_usingrawkey)(struct hmac_sha256_ctx *ctx,
					     const u8 *raw_key,
					     size_t raw_key_len);
	void (*hmac_sha224_final)(struct hmac_sha224_ctx *ctx,
				  u8 out[SHA224_DIGEST_SIZE]);
	void (*hmac_sha256_final)(struct hmac_sha256_ctx *ctx,
				  u8 out[SHA256_DIGEST_SIZE]);
	void (*hmac_sha224)(const struct hmac_sha224_key *key, const u8 *data,
			    size_t data_len, u8 out[SHA224_DIGEST_SIZE]);
	void (*hmac_sha256)(const struct hmac_sha256_key *key, const u8 *data,
			    size_t data_len, u8 out[SHA256_DIGEST_SIZE]);
	void (*hmac_sha224_usingrawkey)(const u8 *raw_key, size_t raw_key_len,
					const u8 *data, size_t data_len,
					u8 out[SHA224_DIGEST_SIZE]);
	void (*hmac_sha256_usingrawkey)(const u8 *raw_key, size_t raw_key_len,
					const u8 *data, size_t data_len,
					u8 out[SHA256_DIGEST_SIZE]);

	/* lib/crypto/sha512.c */
	void (*sha384_init)(struct sha384_ctx *ctx);
	void (*sha512_init)(struct sha512_ctx *ctx);
	void (*__sha512_update)(struct __sha512_ctx *ctx, const u8 *data,
				size_t len);
	void (*sha384_final)(struct sha384_ctx *ctx,
			     u8 out[SHA384_DIGEST_SIZE]);
	void (*sha512_final)(struct sha512_ctx *ctx,
			     u8 out[SHA512_DIGEST_SIZE]);
	void (*sha384)(const u8 *data, size_t len, u8 out[SHA384_DIGEST_SIZE]);
	void (*sha512)(const u8 *data, size_t len, u8 out[SHA512_DIGEST_SIZE]);
	void (*hmac_sha384_preparekey)(struct hmac_sha384_key *key,
				       const u8 *raw_key, size_t raw_key_len);
	void (*hmac_sha512_preparekey)(struct hmac_sha512_key *key,
				       const u8 *raw_key, size_t raw_key_len);
	void (*__hmac_sha512_init)(struct __hmac_sha512_ctx *ctx,
				   const struct __hmac_sha512_key *key);
	void (*hmac_sha384_init_usingrawkey)(struct hmac_sha384_ctx *ctx,
					     const u8 *raw_key,
					     size_t raw_key_len);
	void (*hmac_sha512_init_usingrawkey)(struct hmac_sha512_ctx *ctx,
					     const u8 *raw_key,
					     size_t raw_key_len);
	void (*hmac_sha384_final)(struct hmac_sha384_ctx *ctx,
				  u8 out[SHA384_DIGEST_SIZE]);
	void (*hmac_sha512_final)(struct hmac_sha512_ctx *ctx,
				  u8 out[SHA512_DIGEST_SIZE]);
	void (*hmac_sha384)(const struct hmac_sha384_key *key, const u8 *data,
			    size_t data_len, u8 out[SHA384_DIGEST_SIZE]);
	void (*hmac_sha512)(const struct hmac_sha512_key *key, const u8 *data,
			    size_t data_len, u8 out[SHA512_DIGEST_SIZE]);
	void (*hmac_sha384_usingrawkey)(const u8 *raw_key, size_t raw_key_len,
					const u8 *data, size_t data_len,
					u8 out[SHA384_DIGEST_SIZE]);
	void (*hmac_sha512_usingrawkey)(const u8 *raw_key, size_t raw_key_len,
					const u8 *data, size_t data_len,
					u8 out[SHA512_DIGEST_SIZE]);
};

extern struct fips140_lib_funcs fips140_lib_funcs;
DECLARE_STATIC_KEY_FALSE(fips140_lib_funcs_loaded);

void register_fips140_lib_funcs(const struct fips140_lib_funcs *funcs);

/*
 * FIPS140_CALL() calls the fips140.ko version of the given function and returns
 * the result (if any), if it is called from code outside fips140.ko after
 * fips140.ko has been loaded.  Otherwise it does nothing.
 *
 * This must be invoked at the beginning of every function overridden by
 * fips140.ko, passing the name of the current function as 'func'.  (Note that
 * we can't just use __func__, since __func__ is a string.)
 */
#if defined(BUILD_FIPS140_KO) || \
	!defined(CONFIG_CRYPTO_FIPS140_LIB_OVERRIDE_SUPPORT)
#define FIPS140_CALL(func, ...)
#else
#define FIPS140_CALL(func, ...)                                        \
	({                                                             \
		if (static_branch_unlikely(&fips140_lib_funcs_loaded)) \
			return fips140_lib_funcs.func(__VA_ARGS__);    \
	})
#endif

#else
/* Building library file for pre-boot environment, e.g. purgatory */
#define FIPS140_CALL(func, ...)
#endif

#endif /* _CRYPTO_FIPS140_LIB_OVERRIDES_H */
