/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file fsl_shw_wrap.c
 *
 * This file implements Key-Wrap (Black Key) and Key Establishment functions of
 * the FSL SHW API for the SHW (non-SAHARA) driver.
 *
 * This is the Black Key information:
 *
 * <ul>
 * <li> Ownerid is an 8-byte, user-supplied, value to keep KEY
 *      confidential.</li>
 * <li> KEY is a 1-32 byte value which starts in SCC RED RAM before
 *     wrapping, and ends up there on unwrap.  Length is limited because of
 *    size of SCC1 RAM.</li>
 * <li> KEY' is the encrypted KEY</li>
 * <li> LEN is a 1-byte (for now) byte-length of KEY</li>
 * <li> ALG is a 1-byte value for the algorithm which which the key is
 *    associated.  Values are defined by the FSL SHW API</li>
 * <li> FLAGS is a 1-byte value contain information like "this key is for
 *    software" (TBD)</li>
 * <li> Ownerid, LEN, and ALG come from the user's "key_info" object, as does
 *    the slot number where KEY already is/will be.</li>
 * <li> T is a Nonce</li>
 * <li> T' is the encrypted T</li>
 * <li> KEK is a Key-Encryption Key for the user's Key</li>
 * <li> ICV is the "Integrity Check Value" for the wrapped key</li>
 * <li> Black Key is the string of bytes returned as the wrapped key</li>
 * <li> Wrap Key is the user's choice for encrypting the nonce.  One of
 *      the Fused Key, the Random Key, or the XOR of the two.
 * </ul>
<table border="0">
<tr><TD align="right">BLACK_KEY <TD width="3">=<TD>ICV | T' | LEN | ALG |
     FLAGS | KEY'</td></tr>
<tr><td>&nbsp;</td></tr>

<tr><th>To Wrap</th></tr>
<tr><TD align="right">T</td> <TD width="3">=</td> <TD>RND()<sub>16</sub>
    </td></tr>
<tr><TD align="right">KEK</td><TD width="3">=</td><TD>HASH<sub>sha256</sub>(T |
     Ownerid)<sub>16</sub></td></tr>
<tr><TD align="right">KEY'<TD width="3">=</td><TD>
     TDES<sub>cbc-enc</sub>(Key=KEK, Data=KEY, IV=Ownerid)</td></tr>
<tr><TD align="right">ICV</td><TD width="3">=</td><td>HMAC<sub>sha256</sub>
     (Key=T, Data=Ownerid | LEN | ALG | FLAGS | KEY')<sub>16</sub></td></tr>
<tr><TD align="right">T'</td><TD width="3">=</td><TD>TDES<sub>ecb-enc</sub>
     (Key=Wrap_Key, IV=Ownerid, Data=T)</td></tr>

<tr><td>&nbsp;</td></tr>

<tr><th>To Unwrap</th></tr>
<tr><TD align="right">T</td><TD width="3">=</td><TD>TDES<sub>ecb-dec</sub>
    (Key=Wrap_Key, IV=Ownerid, Data=T')</td></tr>
<tr><TD align="right">ICV</td><TD width="3">=</td><td>HMAC<sub>sha256</sub>
    (Key=T, Data=Ownerid | LEN | ALG | FLAGS | KEY')<sub>16</sub></td></tr>
<tr><TD align="right">KEK</td><TD width="3">=</td><td>HASH<sub>sha256</sub>
    (T | Ownerid)<sub>16</sub></td></tr>
<tr><TD align="right">KEY<TD width="3">=</td><TD>TDES<sub>cbc-dec</sub>
    (Key=KEK, Data=KEY', IV=Ownerid)</td></tr>
</table>

 * This code supports two types of keys: Software Keys and keys destined for
 * (or residing in) the DryIce Programmed Key Register.
 *
 * Software Keys go to / from the keystore.
 *
 * PK keys go to / from the DryIce Programmed Key Register.
 *
 * This code only works on a platform with DryIce.  "software" keys go into
 * the keystore.  "Program" keys go to the DryIce Programmed Key Register.
 * As far as this code is concerned, the size of that register is 21 bytes,
 * the size of a 3DES key with parity stripped.
 *
 * The maximum key size supported for wrapped/unwrapped keys depends upon
 * LENGTH_LENGTH.  Currently, it is one byte, so the maximum key size is
 * 255 bytes.  However, key objects cannot currently hold a key of this
 * length, so a smaller key size is the max.
 */

#include "fsl_platform.h"
	 
/* This code only works in kernel mode */

#include "shw_driver.h"
#ifdef DIAG_SECURITY_FUNC
#include "apihelp.h"
#endif

#if defined(__KERNEL__) && defined(FSL_HAVE_DRYICE)

#include "../dryice.h"
#include <linux/mxc_scc_driver.h>

#include "portable_os.h"
#include "fsl_shw_keystore.h"

#include <diagnostic.h>

#include "shw_hmac.h"
#include "shw_hash.h"

#define ICV_LENGTH 16
#define T_LENGTH 16
#define KEK_LENGTH 21
#define LENGTH_LENGTH 1
#define ALGORITHM_LENGTH 1
#define FLAGS_LENGTH 1

/* ICV | T' | LEN | ALG | FLAGS | KEY' */
#define ICV_OFFSET       0
#define T_PRIME_OFFSET   (ICV_OFFSET + ICV_LENGTH)
#define LENGTH_OFFSET    (T_PRIME_OFFSET + T_LENGTH)
#define ALGORITHM_OFFSET (LENGTH_OFFSET + LENGTH_LENGTH)
#define FLAGS_OFFSET     (ALGORITHM_OFFSET + ALGORITHM_LENGTH)
#define KEY_PRIME_OFFSET (FLAGS_OFFSET + FLAGS_LENGTH)

#define FLAGS_SW_KEY     0x01

#define LENGTH_PATCH  8
#define LENGTH_PATCH_MASK  (LENGTH_PATCH - 1)

/*! rounded up from 168 bits to the next word size */
#define HW_KEY_LEN_WORDS_BITS 192

/*!
 * Round a key length up to the TDES block size
 *
 * @param len         Length of key, in bytes
 *
 * @return Length rounded up, if necessary
 */
#define ROUND_LENGTH(len)                                                  \
({                                                                         \
    uint32_t orig_len = len;                                               \
    uint32_t new_len;                                                      \
                                                                           \
   if ((orig_len & LENGTH_PATCH_MASK) != 0) {                              \
       new_len = (orig_len + LENGTH_PATCH                                  \
            - (orig_len & LENGTH_PATCH_MASK));                             \
   }                                                                       \
   else {                                                                  \
       new_len = orig_len;                                                 \
   }                                                                       \
                                                                           \
   new_len;                                                                \
})

/* This is the system keystore object */
extern fsl_shw_kso_t system_keystore;

#ifdef DIAG_SECURITY_FUNC
static void dump(const char *name, const uint8_t * data, unsigned int len)
{
	os_printk("%s: ", name);
	while (len > 0) {
		os_printk("%02x ", (unsigned)*data++);
		len--;
	}
	os_printk("\n");
}
#endif

/*
 * For testing of the algorithm implementation,, the DO_REPEATABLE_WRAP flag
 * causes the T_block to go into the T field during a wrap operation.  This
 * will make the black key value repeatable (for a given SCC secret key, or
 * always if the default key is in use).
 *
 * Normally, a random sequence is used.
 */
#ifdef DO_REPEATABLE_WRAP
/*!
 * Block of zeroes which is maximum Symmetric block size, used for
 * initializing context register, etc.
 */
static uint8_t T_block[16] = {
	0x42, 0, 0, 0x42, 0x42, 0, 0, 0x42,
	0x42, 0, 0, 0x42, 0x42, 0, 0, 0x42
};
#endif

EXPORT_SYMBOL(fsl_shw_establish_key);
EXPORT_SYMBOL(fsl_shw_read_key);
EXPORT_SYMBOL(fsl_shw_extract_key);
EXPORT_SYMBOL(fsl_shw_release_key);

extern fsl_shw_return_t alloc_slot(fsl_shw_uco_t * user_ctx,
				   fsl_shw_sko_t * key_info);

extern fsl_shw_return_t load_slot(fsl_shw_uco_t * user_ctx,
				  fsl_shw_sko_t * key_info,
				  const uint8_t * key);

extern fsl_shw_return_t dealloc_slot(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info);

/*!
 * Initalialize SKO and SCCO used for T <==> T' cipher operation
 *
 * @param wrap_key  Which wrapping key user wants
 * @param key_info  Key object for selecting wrap key
 * @param wrap_ctx  Sym Context object for doing the cipher op
 */
static inline void init_wrap_key(fsl_shw_pf_key_t wrap_key,
				 fsl_shw_sko_t * key_info,
				 fsl_shw_scco_t * wrap_ctx)
{
	fsl_shw_sko_init_pf_key(key_info, FSL_KEY_ALG_TDES, wrap_key);
	fsl_shw_scco_init(wrap_ctx, FSL_KEY_ALG_TDES, FSL_SYM_MODE_ECB);
}

/*!
 * Insert descriptors to calculate ICV = HMAC(key=T, data=LEN|ALG|KEY')
 *
 * @param  user_ctx      A user context from #fsl_shw_register_user().
 * @param  T             Location of nonce (length is T_LENGTH bytes)
 * @param  userid        Location of userid/ownerid
 * @param  userid_len    Length, in bytes of @c userid
 * @param  black_key     Beginning of Black Key region
 * @param  key_length    Number of bytes of key' there are in @c black_key
 * @param[out] hmac      Location to store ICV.  Will be tagged "USES" so
 *                       sf routines will not try to free it.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
static fsl_shw_return_t calc_icv(const uint8_t * T,
				 const uint8_t * userid,
				 unsigned int userid_len,
				 const uint8_t * black_key,
				 uint32_t key_length, uint8_t * hmac)
{
	fsl_shw_return_t code;
	shw_hmac_state_t hmac_state;

	/* Load up T as key for the HMAC */
	code = shw_hmac_init(&hmac_state, T, T_LENGTH);
	if (code != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Previous step loaded key; Now set up to hash the data */

	/* Input - start with ownerid */
	code = shw_hmac_update(&hmac_state, userid, userid_len);
	if (code != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Still input  - Append black-key fields len, alg, key' */
	code = shw_hmac_update(&hmac_state,
			       (void *)black_key + LENGTH_OFFSET,
			       (LENGTH_LENGTH
				+ ALGORITHM_LENGTH
				+ FLAGS_LENGTH + key_length));
	if (code != FSL_RETURN_OK_S) {
		goto out;
	}

	/* Output - computed ICV/HMAC */
	code = shw_hmac_final(&hmac_state, hmac, ICV_LENGTH);
	if (code != FSL_RETURN_OK_S) {
		goto out;
	}

      out:

	return code;
}				/* calc_icv */

/*!
 * Compute and return the KEK (Key Encryption Key) from the inputs
 *
 * @param userid      The user's 'secret' for the key
 * @param userid_len  Length, in bytes of @c userid
 * @param T           The nonce
 * @param[out] kek    Location to store the computed KEK. It will
 *                    be 21 bytes long.
 *
 * @return the usual error code
 */
static fsl_shw_return_t calc_kek(const uint8_t * userid,
				 unsigned int userid_len,
				 const uint8_t * T, uint8_t * kek)
{
	fsl_shw_return_t code = FSL_RETURN_INTERNAL_ERROR_S;
	shw_hash_state_t hash_state;

	code = shw_hash_init(&hash_state, FSL_HASH_ALG_SHA256);
	if (code != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG_ARGS("Hash init failed: %s\n", fsl_error_string(code));
#endif
		goto out;
	}

	code = shw_hash_update(&hash_state, T, T_LENGTH);
	if (code != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG_ARGS("Hash for T failed: %s\n",
			      fsl_error_string(code));
#endif
		goto out;
	}

	code = shw_hash_update(&hash_state, userid, userid_len);
	if (code != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("Hash for userid failed: %s\n",
			      fsl_error_string(code));
#endif
		goto out;
	}

	code = shw_hash_final(&hash_state, kek, KEK_LENGTH);
	if (code != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("Could not extract kek: %s\n",
			      fsl_error_string(code));
#endif
		goto out;
	}

#if KEK_LENGTH != 21
	{
		uint8_t permuted_kek[21];

		fsl_shw_permute1_bytes(kek, permuted_kek, KEK_LENGTH / 8);
		memcpy(kek, permuted_kek, 21);
		memset(permuted_kek, 0, sizeof(permuted_kek));
	}
#endif

#ifdef DIAG_SECURITY_FUNC
	dump("kek", kek, 21);
#endif

      out:

	return code;
}				/* end fn calc_kek */

/*!
 * Validate user's wrap key selection
 *
 * @param wrap_key   The user's desired wrapping key
 */
static fsl_shw_return_t check_wrap_key(fsl_shw_pf_key_t wrap_key)
{
	/* unable to use desired key */
	fsl_shw_return_t ret = FSL_RETURN_NO_RESOURCE_S;

	if ((wrap_key != FSL_SHW_PF_KEY_IIM) &&
	    (wrap_key != FSL_SHW_PF_KEY_RND) &&
	    (wrap_key != FSL_SHW_PF_KEY_IIM_RND)) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Invalid wrap_key in key wrap/unwrap attempt");
#endif
		goto out;
	}
	ret = FSL_RETURN_OK_S;

      out:
	return ret;
}				/* end fn check_wrap_key */

/*!
 * Perform unwrapping of a black key into a RED slot
 *
 * @param         user_ctx      A user context from #fsl_shw_register_user().
 * @param[in,out] key_info      The information about the key to be which will
 *                              be unwrapped... key length, slot info, etc.
 * @param         black_key     Encrypted key
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
static fsl_shw_return_t unwrap(fsl_shw_uco_t * user_ctx,
			       fsl_shw_sko_t * key_info,
			       const uint8_t * black_key)
{
	fsl_shw_return_t ret;
	uint8_t hmac[ICV_LENGTH];
	uint8_t T[T_LENGTH];
	uint8_t kek[KEK_LENGTH + 20];
	int key_length = black_key[LENGTH_OFFSET];
	int rounded_key_length = ROUND_LENGTH(key_length);
	uint8_t key[rounded_key_length];
	fsl_shw_sko_t t_key_info;
	fsl_shw_scco_t t_key_ctx;
	fsl_shw_sko_t kek_key_info;
	fsl_shw_scco_t kek_ctx;
	int unwrapping_sw_key = key_info->flags & FSL_SKO_KEY_SW_KEY;
	int pk_needs_restoration = 0;	/* bool */
	unsigned original_key_length = key_info->key_length;
	int pk_was_held = 0;
	uint8_t current_pk[21];
	di_return_t di_code;

	ret = check_wrap_key(user_ctx->wrap_key);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	if (black_key == NULL) {
		ret = FSL_RETURN_ERROR_S;
		goto out;
	}
#ifdef DIAG_SECURITY_FUNC
	dump("black", black_key, KEY_PRIME_OFFSET + key_length);
#endif
	/* Validate SW flags to prevent misuse */
		if ((key_info->flags & FSL_SKO_KEY_SW_KEY)
			&& !(black_key[FLAGS_OFFSET] & FLAGS_SW_KEY)) {
			ret = FSL_RETURN_BAD_FLAG_S;
			goto out;
		}

	/* Compute T = 3des-dec-ecb(wrap_key, T') */
	init_wrap_key(user_ctx->wrap_key, &t_key_info, &t_key_ctx);
	ret = fsl_shw_symmetric_decrypt(user_ctx, &t_key_info, &t_key_ctx,
					T_LENGTH,
					black_key + T_PRIME_OFFSET, T);
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Recovery of nonce (T) failed");
#endif				/*DIAG_SECURITY_FUNC */
		goto out;
	}

	/* Compute ICV = HMAC(T, ownerid | len | alg | flags | key' */
	ret = calc_icv(T, (uint8_t *) & key_info->userid,
		       sizeof(key_info->userid),
		       black_key, original_key_length, hmac);

	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Calculation of ICV failed");
#endif				/*DIAG_SECURITY_FUNC */
		goto out;
	}
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG("Validating MAC of wrapped key");
#endif

	/* Check computed ICV against value in Black Key */
	if (memcmp(black_key + ICV_OFFSET, hmac, ICV_LENGTH) != 0) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Computed ICV fails validation\n");
#endif
		ret = FSL_RETURN_AUTH_FAILED_S;
		goto out;
	}

	/* Compute KEK = SHA256(T | ownerid). */
	ret = calc_kek((uint8_t *) & key_info->userid, sizeof(key_info->userid),
		       T, kek);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	if (unwrapping_sw_key) {
		di_code = dryice_get_programmed_key(current_pk, 8 * 21);
		if (di_code != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG_ARGS("Could not save current PK: %s\n",
				      di_error_string(di_code));
#endif
			ret = FSL_RETURN_ERROR_S;
			goto out;
		}
	}

	/*
	 * "Establish" the KEK in the PK.  If the PK was held and unwrapping a
	 * software key, then release it and try again, but remember that we need
	 * to leave it 'held' if we are unwrapping a software key.
	 *
	 * If the PK is held while we are unwrapping a key for the PK, then
	 * the user didn't call release, so gets an error.
	 */
	di_code = dryice_set_programmed_key(kek, 8 * 21, 0);
	if ((di_code == DI_ERR_INUSE) && unwrapping_sw_key) {
		/* Temporarily reprogram the PK out from under the user */
		pk_was_held = 1;
		dryice_release_programmed_key();
		di_code = dryice_set_programmed_key(kek, 8 * 21, 0);
	}
	if (di_code != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("Could not program KEK: %s\n",
			      di_error_string(di_code));
#endif
		ret = FSL_RETURN_ERROR_S;
		goto out;
	}

	if (unwrapping_sw_key) {
		pk_needs_restoration = 1;
	}
	dryice_release_programmed_key();	/* Because of previous 'set' */

	/* Compute KEY = TDES-decrypt(KEK, KEY') */
	fsl_shw_sko_init_pf_key(&kek_key_info, FSL_KEY_ALG_TDES,
				FSL_SHW_PF_KEY_PRG);
	fsl_shw_sko_set_key_length(&kek_key_info, KEK_LENGTH);

	fsl_shw_scco_init(&kek_ctx, FSL_KEY_ALG_TDES, FSL_SYM_MODE_CBC);
	fsl_shw_scco_set_flags(&kek_ctx, FSL_SYM_CTX_LOAD);
	fsl_shw_scco_set_context(&kek_ctx, (uint8_t *) & key_info->userid);
#ifdef DIAG_SECURITY_FUNC
	dump("KEY'", black_key + KEY_PRIME_OFFSET, rounded_key_length);
#endif
	ret = fsl_shw_symmetric_decrypt(user_ctx, &kek_key_info, &kek_ctx,
					rounded_key_length,
					black_key + KEY_PRIME_OFFSET, key);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}
#ifdef DIAG_SECURITY_FUNC
	dump("KEY", key, original_key_length);
#endif
	/* Now either put key into PK or into a slot */
	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
		ret = load_slot(user_ctx, key_info, key);
	} else {
		/*
		 * Since we have just unwrapped a program key, it had
		 * to have been wrapped as a program key, so it must
		 * be 168 bytes long and permuted ...
		 */
		ret = dryice_set_programmed_key(key, 8 * key_length, 0);
		if (ret != FSL_RETURN_OK_S) {
			goto out;
		}
	}

      out:
	key_info->key_length = original_key_length;

	if (pk_needs_restoration) {
		di_code = dryice_set_programmed_key(current_pk, 8 * 21, 0);
	}

	if (!pk_was_held) {
		dryice_release_programmed_key();
	}

	/* Erase tracks of confidential data */
	memset(T, 0, T_LENGTH);
	memset(key, 0, rounded_key_length);
	memset(current_pk, 0, sizeof(current_pk));
	memset(&t_key_info, 0, sizeof(t_key_info));
	memset(&t_key_ctx, 0, sizeof(t_key_ctx));
	memset(&kek_key_info, 0, sizeof(kek_key_info));
	memset(&kek_ctx, 0, sizeof(kek_ctx));
	memset(kek, 0, KEK_LENGTH);

	return ret;
}				/* unwrap */

/*!
 * Perform wrapping of a black key from a RED slot (or the PK register)
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param[in,out] key_info      The information about the key to be which will
 *                              be wrapped... key length, slot info, etc.
 * @param      black_key        Place to store encrypted key
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
static fsl_shw_return_t wrap(fsl_shw_uco_t * user_ctx,
			     fsl_shw_sko_t * key_info, uint8_t * black_key)
{
	fsl_shw_return_t ret = FSL_RETURN_OK_S;
	fsl_shw_sko_t t_key_info;	/* for holding T */
	fsl_shw_scco_t t_key_ctx;
	fsl_shw_sko_t kek_key_info;
	fsl_shw_scco_t kek_ctx;
	unsigned original_key_length = key_info->key_length;
	unsigned rounded_key_length;
	uint8_t T[T_LENGTH];
	uint8_t kek[KEK_LENGTH + 20];
	uint8_t *red_key = 0;
	int red_key_malloced = 0;	/* bool */
	int pk_was_held = 0;	/* bool */
	uint8_t saved_pk[21];
	uint8_t pk_needs_restoration;	/* bool */
	di_return_t di_code;

	ret = check_wrap_key(user_ctx->wrap_key);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	if (black_key == NULL) {
		ret = FSL_RETURN_ERROR_S;
		goto out;
	}

	if (key_info->flags & FSL_SKO_KEY_SELECT_PF_KEY) {
		if ((key_info->pf_key != FSL_SHW_PF_KEY_PRG)
		    && (key_info->pf_key != FSL_SHW_PF_KEY_IIM_PRG)) {
			ret = FSL_RETURN_ERROR_S;
			goto out;
		}
	} else {
		if (!(key_info->flags & FSL_SKO_KEY_ESTABLISHED)) {
			ret = FSL_RETURN_BAD_FLAG_S;	/* not established! */
			goto out;
		}
	}

	black_key[ALGORITHM_OFFSET] = key_info->algorithm;

#ifndef DO_REPEATABLE_WRAP
	/* Compute T = RND() */
	ret = fsl_shw_get_random(user_ctx, T_LENGTH, T);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}
#else
	memcpy(T, T_block, T_LENGTH);
#endif

	/* Compute KEK = SHA256(T | ownerid). */
	ret = calc_kek((uint8_t *) & key_info->userid, sizeof(key_info->userid),
		       T, kek);
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Calculation of KEK failed\n");
#endif				/*DIAG_SECURITY_FUNC */
		goto out;
	}

	rounded_key_length = ROUND_LENGTH(original_key_length);

	di_code = dryice_get_programmed_key(saved_pk, 8 * 21);
	if (di_code != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("Could not save current PK: %s\n",
			      di_error_string(di_code));
#endif
		ret = FSL_RETURN_ERROR_S;
		goto out;
	}

	/*
	 * Load KEK into DI PKR.  Note that we are NOT permuting it before loading,
	 * so we are using it as though it is a 168-bit key ready for the SCC.
	 */
	di_code = dryice_set_programmed_key(kek, 8 * 21, 0);
	if (di_code == DI_ERR_INUSE) {
		/* Temporarily reprogram the PK out from under the user */
		pk_was_held = 1;
		dryice_release_programmed_key();
		di_code = dryice_set_programmed_key(kek, 8 * 21, 0);
	}
	if (di_code != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("Could not program KEK: %s\n",
			      di_error_string(di_code));
#endif
		ret = FSL_RETURN_ERROR_S;
		goto out;
	}
	pk_needs_restoration = 1;
	dryice_release_programmed_key();

	/* Find red key */
	if (key_info->flags & FSL_SKO_KEY_SELECT_PF_KEY) {
		black_key[LENGTH_OFFSET] = 21;
		rounded_key_length = 24;

		red_key = saved_pk;
	} else {
		black_key[LENGTH_OFFSET] = key_info->key_length;

		red_key = os_alloc_memory(key_info->key_length, 0);
		if (red_key == NULL) {
			ret = FSL_RETURN_NO_RESOURCE_S;
			goto out;
		}
		red_key_malloced = 1;

		ret = fsl_shw_read_key(user_ctx, key_info, red_key);
		if (ret != FSL_RETURN_OK_S) {
			goto out;
		}
	}

#ifdef DIAG_SECURITY_FUNC
	dump("KEY", red_key, black_key[LENGTH_OFFSET]);
#endif
	/* Compute KEY' = TDES-encrypt(KEK, KEY) */
	fsl_shw_sko_init_pf_key(&kek_key_info, FSL_KEY_ALG_TDES,
				FSL_SHW_PF_KEY_PRG);
	fsl_shw_sko_set_key_length(&kek_key_info, KEK_LENGTH);

	fsl_shw_scco_init(&kek_ctx, FSL_KEY_ALG_TDES, FSL_SYM_MODE_CBC);
	fsl_shw_scco_set_flags(&kek_ctx, FSL_SYM_CTX_LOAD);
	fsl_shw_scco_set_context(&kek_ctx, (uint8_t *) & key_info->userid);
	ret = fsl_shw_symmetric_encrypt(user_ctx, &kek_key_info, &kek_ctx,
					rounded_key_length,
					red_key, black_key + KEY_PRIME_OFFSET);
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Encryption of KEY failed\n");
#endif				/*DIAG_SECURITY_FUNC */
		goto out;
	}

	/* Set up flags info */
	black_key[FLAGS_OFFSET] = 0;
	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
		black_key[FLAGS_OFFSET] |= FLAGS_SW_KEY;
	}
#ifdef DIAG_SECURITY_FUNC
	dump("KEY'", black_key + KEY_PRIME_OFFSET, rounded_key_length);
#endif
	/* Compute and store ICV into Black Key */
	ret = calc_icv(T,
		       (uint8_t *) & key_info->userid,
		       sizeof(key_info->userid),
		       black_key, original_key_length, black_key + ICV_OFFSET);
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Calculation of ICV failed\n");
#endif				/*DIAG_SECURITY_FUNC */
		goto out;
	}

	/* Compute T' = 3des-enc-ecb(wrap_key, T); Result goes to Black Key */
	init_wrap_key(user_ctx->wrap_key, &t_key_info, &t_key_ctx);
	ret = fsl_shw_symmetric_encrypt(user_ctx, &t_key_info, &t_key_ctx,
					T_LENGTH,
					T, black_key + T_PRIME_OFFSET);
	if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Encryption of nonce failed");
#endif
		goto out;
	}
#ifdef DIAG_SECURITY_FUNC
	dump("black", black_key, KEY_PRIME_OFFSET + black_key[LENGTH_OFFSET]);
#endif

      out:
	if (pk_needs_restoration) {
		dryice_set_programmed_key(saved_pk, 8 * 21, 0);
	}

	if (!pk_was_held) {
		dryice_release_programmed_key();
	}

	if (red_key_malloced) {
		memset(red_key, 0, key_info->key_length);
		os_free_memory(red_key);
	}

	key_info->key_length = original_key_length;

	/* Erase tracks of confidential data */
	memset(T, 0, T_LENGTH);
	memset(&t_key_info, 0, sizeof(t_key_info));
	memset(&t_key_ctx, 0, sizeof(t_key_ctx));
	memset(&kek_key_info, 0, sizeof(kek_key_info));
	memset(&kek_ctx, 0, sizeof(kek_ctx));
	memset(kek, 0, sizeof(kek));
	memset(saved_pk, 0, sizeof(saved_pk));

	return ret;
}				/* wrap */

static fsl_shw_return_t create(fsl_shw_uco_t * user_ctx,
			       fsl_shw_sko_t * key_info)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;
	unsigned key_length = key_info->key_length;
	di_return_t di_code;

	if (!(key_info->flags & FSL_SKO_KEY_SW_KEY)) {
		/* Must be creating key for PK */
		if ((key_info->algorithm != FSL_KEY_ALG_TDES) ||
		    ((key_info->key_length != 16)
		     && (key_info->key_length != 21)	/* permuted 168-bit key */
		     &&(key_info->key_length != 24))) {
			ret = FSL_RETURN_ERROR_S;
			goto out;
		}

		key_length = 21;	/* 168-bit PK */
	}

	/* operational block */
	{
		uint8_t key_value[key_length];

#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Creating random key\n");
#endif
		ret = fsl_shw_get_random(user_ctx, key_length, key_value);
		if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG("get_random for CREATE KEY failed\n");
#endif
			goto out;
		}

		if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
			ret = load_slot(user_ctx, key_info, key_value);
		} else {
			di_code =
			    dryice_set_programmed_key(key_value, 8 * key_length,
						      0);
			if (di_code != 0) {
#ifdef DIAG_SECURITY_FUNC
				LOG_DIAG_ARGS("di set_pk failed: %s\n",
					      di_error_string(di_code));
#endif
				ret = FSL_RETURN_ERROR_S;
				goto out;
			}
			ret = FSL_RETURN_OK_S;
		}
		memset(key_value, 0, key_length);
	}			/* end operational block */

#ifdef DIAG_SECURITY_FUNC
	if (ret != FSL_RETURN_OK_S) {
		LOG_DIAG("Loading random key failed");
	}
#endif

      out:

	return ret;
}				/* end fn create */

static fsl_shw_return_t accept(fsl_shw_uco_t * user_ctx,
			       fsl_shw_sko_t * key_info, const uint8_t * key)
{
	uint8_t permuted_key[21];
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

	if (key == NULL) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("ACCEPT:  Red Key is NULL");
#endif
		ret = FSL_RETURN_ERROR_S;
		goto out;
	}
#ifdef DIAG_SECURITY_FUNC
	dump("red", key, key_info->key_length);
#endif
	/* Only SW keys go into the keystore */
	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {

		/* Copy in safe number of bytes of Red key */
		ret = load_slot(user_ctx, key_info, key);
	} else {		/* not SW key */
		di_return_t di_ret;

		/* Only 3DES PGM key types can be established */
		if (((key_info->pf_key != FSL_SHW_PF_KEY_PRG)
		     && (key_info->pf_key != FSL_SHW_PF_KEY_IIM_PRG))
		    || (key_info->algorithm != FSL_KEY_ALG_TDES)) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG_ARGS
			    ("ACCEPT:  Failed trying to establish non-PRG"
			     " or invalid 3DES Key: iim%d, iim_prg%d, alg%d\n",
			     (key_info->pf_key != FSL_SHW_PF_KEY_PRG),
			     (key_info->pf_key != FSL_SHW_PF_KEY_IIM_PRG),
			     (key_info->algorithm != FSL_KEY_ALG_TDES));
#endif
			ret = FSL_RETURN_ERROR_S;
			goto out;
		}
		if ((key_info->key_length != 16)
		    && (key_info->key_length != 21)
		    && (key_info->key_length != 24)) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG_ARGS("ACCEPT:  Failed trying to establish"
				      " invalid 3DES Key: len=%d (%d)\n",
				      key_info->key_length,
				      ((key_info->key_length != 16)
				       && (key_info->key_length != 21)
				       && (key_info->key_length != 24)));
#endif
			ret = FSL_RETURN_BAD_KEY_LENGTH_S;
			goto out;
		}

		/* Convert key into 168-bit value and put it into PK */
		if (key_info->key_length != 21) {
			fsl_shw_permute1_bytes(key, permuted_key,
					       key_info->key_length / 8);
			di_ret =
			    dryice_set_programmed_key(permuted_key, 168, 0);
		} else {
			/* Already permuted ! */
			di_ret = dryice_set_programmed_key(key, 168, 0);
		}
		if (di_ret != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG_ARGS
			    ("ACCEPT:  DryIce error setting Program Key: %s",
			     di_error_string(di_ret));
#endif
			ret = FSL_RETURN_ERROR_S;
			goto out;
		}
	}

	ret = FSL_RETURN_OK_S;

      out:
	memset(permuted_key, 0, 21);

	return ret;
}				/* end fn accept */

/*!
 * Place a key into a protected location for use only by cryptographic
 * algorithms.
 *
 * This only needs to be used to a) unwrap a key, or b) set up a key which
 * could be wrapped by calling #fsl_shw_extract_key() at some later time).
 *
 * The protected key will not be available for use until this operation
 * successfully completes.
 *
 * @bug This whole discussion needs review.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param[in,out] key_info      The information about the key to be which will
 *                              be established.  In the create case, the key
 *                              length must be set.
 * @param      establish_type   How @a key will be interpreted to establish a
 *                              key for use.
 * @param key                   If @a establish_type is #FSL_KEY_WRAP_UNWRAP,
 *                              this is the location of a wrapped key.  If
 *                              @a establish_type is #FSL_KEY_WRAP_CREATE, this
 *                              parameter can be @a NULL.  If @a establish_type
 *                              is #FSL_KEY_WRAP_ACCEPT, this is the location
 *                              of a plaintext key.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_establish_key(fsl_shw_uco_t * user_ctx,
				       fsl_shw_sko_t * key_info,
				       fsl_shw_key_wrap_t establish_type,
				       const uint8_t * key)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;
	unsigned original_key_length = key_info->key_length;
	unsigned rounded_key_length;
	unsigned slot_allocated = 0;

	/* For now, only blocking mode calls are supported */
	if (!(user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("%s: Non-blocking call not supported\n",
			      __FUNCTION__);
#endif
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	/*
	   HW keys are always 'established', but otherwise do not allow user
	   * to establish over the top of an established key.
	 */
	if ((key_info->flags & FSL_SKO_KEY_ESTABLISHED)
	    && !(key_info->flags & FSL_SKO_KEY_SELECT_PF_KEY)) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("%s: Key already established\n", __FUNCTION__);
#endif
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	/* @bug VALIDATE KEY flags here -- SW or PRG/IIM_PRG */

	/* Write operations into SCC memory require word-multiple number of
	 * bytes.  For ACCEPT and CREATE functions, the key length may need
	 * to be rounded up.  Calculate. */
	if (LENGTH_PATCH && (original_key_length & LENGTH_PATCH_MASK) != 0) {
		rounded_key_length = original_key_length + LENGTH_PATCH
		    - (original_key_length & LENGTH_PATCH_MASK);
	} else {
		rounded_key_length = original_key_length;
	}

	/* SW keys need a place to live */
	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
		ret = alloc_slot(user_ctx, key_info);
		if (ret != FSL_RETURN_OK_S) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG("Slot allocation failed\n");
#endif
			goto out;
		}
		slot_allocated = 1;
	}

	switch (establish_type) {
	case FSL_KEY_WRAP_CREATE:
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Creating random key\n");
#endif
		ret = create(user_ctx, key_info);
		break;

	case FSL_KEY_WRAP_ACCEPT:
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Accepting plaintext key\n");
#endif
		ret = accept(user_ctx, key_info, key);
		break;

	case FSL_KEY_WRAP_UNWRAP:
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Unwrapping wrapped key\n");
#endif
		ret = unwrap(user_ctx, key_info, key);
		break;

	default:
		ret = FSL_RETURN_BAD_FLAG_S;
		break;
	}			/* switch */

      out:
	if (ret != FSL_RETURN_OK_S) {
		if (slot_allocated) {
			(void)dealloc_slot(user_ctx, key_info);
		}
		key_info->flags &= ~FSL_SKO_KEY_ESTABLISHED;
	} else {
		key_info->flags |= FSL_SKO_KEY_ESTABLISHED;
	}

	return ret;
}				/* end fn fsl_shw_establish_key */

/*!
 * Wrap a key and retrieve the wrapped value.
 *
 * A wrapped key is a key that has been cryptographically obscured.  It is
 * only able to be used with #fsl_shw_establish_key().
 *
 * This function will also release a software  key (see #fsl_shw_release_key())
 * so it must be re-established before reuse. This is not true of PGM keys.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      key_info         The information about the key to be deleted.
 * @param[out] covered_key      The location to store the 48-octet wrapped key.
 *                              (This size is based upon the maximum key size
 *                              of 32 octets).
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_extract_key(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info,
				     uint8_t * covered_key)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

	/* For now, only blocking mode calls are supported */
	if (!(user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG("Wrapping a key\n");
#endif

	if (!(key_info->flags & FSL_SKO_KEY_ESTABLISHED)) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG_ARGS("%s: Key not established\n", __FUNCTION__);
#endif
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}
	/* Verify that a SW key info really belongs to a SW key */
	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
	/*	  ret = FSL_RETURN_BAD_FLAG_S;
		  goto out;*/
	}

	ret = wrap(user_ctx, key_info, covered_key);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
		/* Need to deallocate on successful extraction */
		(void)dealloc_slot(user_ctx, key_info);
		/* Mark key not available in the flags */
		key_info->flags &=
		    ~(FSL_SKO_KEY_ESTABLISHED | FSL_SKO_KEY_PRESENT);
		memset(key_info->key, 0, sizeof(key_info->key));
	}

      out:
	return ret;
}				/* end fn fsl_shw_extract_key */

/*!
 * De-establish a key so that it can no longer be accessed.
 *
 * The key will need to be re-established before it can again be used.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      key_info         The information about the key to be deleted.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_release_key(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

	/* For now, only blocking mode calls are supported */
	if (!(user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Not in blocking mode\n");
#endif
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG("Releasing a key\n");
#endif

	if (!(key_info->flags & FSL_SKO_KEY_ESTABLISHED)) {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Key not established\n");
#endif
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	if (key_info->flags & FSL_SKO_KEY_SW_KEY) {
		(void)dealloc_slot(user_ctx, key_info);
		/* Turn off 'established' flag */
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("dealloc_slot() called\n");
#endif
		key_info->flags &= ~FSL_SKO_KEY_ESTABLISHED;
		ret = FSL_RETURN_OK_S;
		goto out;
	}

	if ((key_info->pf_key == FSL_SHW_PF_KEY_PRG)
	    || (key_info->pf_key == FSL_SHW_PF_KEY_IIM_PRG)) {
		di_return_t di_ret;

		di_ret = dryice_release_programmed_key();
		if (di_ret != DI_SUCCESS) {
#ifdef DIAG_SECURITY_FUNC
			LOG_DIAG_ARGS
			    ("dryice_release_programmed_key() failed: %d\n",
			     di_ret);
#endif
			ret = FSL_RETURN_ERROR_S;
			goto out;
		}
	} else {
#ifdef DIAG_SECURITY_FUNC
		LOG_DIAG("Neither SW nor HW key\n");
#endif
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}

	ret = FSL_RETURN_OK_S;

      out:
	return ret;
}				/* end fn fsl_shw_release_key */

fsl_shw_return_t fsl_shw_read_key(fsl_shw_uco_t * user_ctx,
				  fsl_shw_sko_t * key_info, uint8_t * key)
{
	fsl_shw_return_t ret = FSL_RETURN_INTERNAL_ERROR_S;

	/* Only blocking mode calls are supported */
	if (!(user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {
		ret = FSL_RETURN_BAD_FLAG_S;
		goto out;
	}
	printk("Reading a key\n");
#ifdef DIAG_SECURITY_FUNC
	LOG_DIAG("Reading a key");
#endif
	if (key_info->flags & FSL_SKO_KEY_PRESENT) {
		memcpy(key_info->key, key, key_info->key_length);
		ret = FSL_RETURN_OK_S;
	} else if (key_info->flags & FSL_SKO_KEY_ESTABLISHED) {
	printk("key established\n");
		if (key_info->keystore == NULL) {
			printk("keystore is null\n");
			/* First verify that the key access is valid */
			ret =
			    system_keystore.slot_verify_access(system_keystore.
							       user_data,
							       key_info->userid,
							       key_info->
							       handle);

			printk("key in system keystore\n");

			/* Key is in system keystore */
			ret = keystore_slot_read(&system_keystore,
						 key_info->userid,
						 key_info->handle,
						 key_info->key_length, key);
		} else {
		printk("key goes in user keystore.\n");
			/* Key goes in user keystore */
			ret = keystore_slot_read(key_info->keystore,
						 key_info->userid,
						 key_info->handle,
						 key_info->key_length, key);
		}
	}

      out:
	return ret;
}				/* end fn fsl_shw_read_key */

#else				/* __KERNEL__ && DRYICE */

/* User mode -- these functions are unsupported */

fsl_shw_return_t fsl_shw_establish_key(fsl_shw_uco_t * user_ctx,
				       fsl_shw_sko_t * key_info,
				       fsl_shw_key_wrap_t establish_type,
				       const uint8_t * key)
{
	(void)user_ctx;
	(void)key_info;
	(void)establish_type;
	(void)key;

	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t fsl_shw_extract_key(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info,
				     uint8_t * covered_key)
{
	(void)user_ctx;
	(void)key_info;
	(void)covered_key;

	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t fsl_shw_release_key(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info)
{
	(void)user_ctx;
	(void)key_info;

	return FSL_RETURN_NO_RESOURCE_S;
}

fsl_shw_return_t fsl_shw_read_key(fsl_shw_uco_t * user_ctx,
				  fsl_shw_sko_t * key_info, uint8_t * key)
{
	(void)user_ctx;
	(void)key_info;
	(void)key;

	return FSL_RETURN_NO_RESOURCE_S;
}

#endif				/* __KERNEL__ && DRYICE */
