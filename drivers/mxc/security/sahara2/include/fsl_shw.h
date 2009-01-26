/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * NOTE TO MAINTAINERS: Although this header file is *the* header file to be
 * #include'd by FSL SHW programs, it does not itself make any definitions for
 * the API.  Instead, it uses the fsl_platform.h file and / or compiler
 * environment variables to determine which actual driver header file to
 * include.  This allows different implementations to contain different
 * implementations of the various objects, macros, etc., or even to change
 * which functions are macros and which are not.
 */

/*!
 * @file fsl_shw.h
 *
 * @brief Definition of the Freescale Security Hardware API.
 *
 * See @ref index for an overview of the API.
 */

/*!
 * @if USE_MAINPAGE
 * @mainpage Common API for Freescale Security Hardware (FSL SHW API)
 * @endif
 *
 * @section intro_sec Introduction
 *
 * This is the interface definition for the Freescale Security Hardware API
 * (FSL SHW API) for User Mode and Kernel Mode to access Freescale Security
 * Hardware components for cryptographic acceleration. The API is intended to
 * provide cross-platform access to security hardware components of Freescale.
 *
 * This documentation has not been approved, and should not be taken to
 * mean anything definite about future direction.
 *
 * Some example code is provided to give some idea of usage of this API.
 *
 * Note: This first version has been defined around the capabilities of the
 * Sahara2 cryptographic accelerator, and may be expanded in the future to
 * provide support for other platforms.  The Platform Capabilities Object is
 * intended as a way to allow programs to adapt to different platforms.
 *
 * The i.MX25 is an example of a platform without a SAHARA but yet has
 * capabilities supported by this API.  These include #fsl_shw_get_random() and
 * #fsl_shw_add_entropy(), and the use of Triple-DES (TDEA) cipher algorithm
 * (with no checking of key parity supported) in ECB and CBC modes with @ref
 * sym_sec.  See also the @ref di_sec for information on key handling, and @ref
 * td_sec for detection of Tamper Events.  Only the random functions are
 * available from user space on this platform.
 *
 * @section usr_ctx The User Context
 *
 * The User Context Object (#fsl_shw_uco_t) controls the interaction between
 * the user program and the API.  It is initialized as part of user
 * registration (#fsl_shw_register_user()), and is part of every interaction
 * thereafter.
 *
 * @section pf_sec Platform Capabilities
 *
 * Since this API is not tied to one specific type of hardware or even one
 * given version of a given type of hardware, the platform capabilities object
 * could be used by a portable program to make choices about using software
 * instead of hardware for certain operations.
 *
 * See the #fsl_shw_pco_t, returned by #fsl_shw_get_capabilities().
 *
 * @ref pcoops are provided to query its contents.
 *
 *
 * @section sym_sec Symmetric-Key Encryption and Decryption
 *
 * Symmetric-Key encryption support is provided for the block cipher algorithms
 * AES, DES, and Triple DES.  Modes supported are #FSL_SYM_MODE_ECB,
 * #FSL_SYM_MODE_CBC, and #FSL_SYM_MODE_CTR, though not necessarily all modes
 * for all algorithms.  There is also support for the stream cipher algorithm
 * commonly known as ARC4.
 *
 * Encryption and decryption are performed by using the functions
 * #fsl_shw_symmetric_encrypt() and #fsl_shw_symmetric_decrypt(), respectively.
 * There are two objects which provide information about the operation of these
 * functions.  They are the #fsl_shw_sko_t, to provide key and algorithm
 * information; and the #fsl_shw_scco_t, to provide (and store) initial context
 * or counter value information.
 *
 * CCM is not supported by these functions.  For information CCM support, see
 * @ref cmb_sec.
 *
 *
 * @section hash_sec Cryptographic Hashing
 *
 * Hashing is performed by fsl_shw_hash(). Control of the function is through
 * flags in the #fsl_shw_hco_t.  The algorithms which are
 * supported are listed in #fsl_shw_hash_alg_t.
 *
 * The hashing function works on octet streams.  If a user application needs to
 * hash a bitstream, it will need to do its own padding of the last block.
 *
 *
 * @section hmac_sec Hashed Message Authentication Codes
 *
 * An HMAC is a method of combining a hash and a key so that a message cannot
 * be faked by a third party.
 *
 * The #fsl_shw_hmac() can be used by itself for one-shot or multi-step
 * operations, or in combination with #fsl_shw_hmac_precompute() to provide the
 * ability to compute and save the beginning hashes from a key one time, and
 * then use #fsl_shw_hmac() to calculate an HMAC on each message as it is
 * processed.
 *
 * The maximum key length which is directly supported by this API is 64 octets.
 * If a longer key size is needed for HMAC, the user will have to hash the key
 * and present the digest value as the key to be used by the HMAC functions.
 *
 *
 * @section rnd_sec Random Numbers
 *
 * Support is available for acquiring random values from a
 * cryptographically-strong random number generator.  See
 * #fsl_shw_get_random().  The function #fsl_shw_add_entropy() may be used to
 * add entropy to the random number generator.
 *
 *
 * @section cmb_sec Combined Cipher and Authentication
 *
 * Some schemes require that messages be encrypted and that they also have an
 * authentication code associated with the message. The function
 * #fsl_shw_gen_encrypt() will generate the authentication code and encrypt the
 * message.
 *
 * Upon receipt of such a message, the message must be decrypted and the
 * authentication code validated.  The function
 * #fsl_shw_auth_decrypt() will perform these steps.
 *
 * Only AES-CCM is supported.
 *
 *
 * @section wrap_sec Wrapped Keys
 *
 * On platforms with a Secure Memory, the function #fsl_shw_establish_key() can
 * be used to place a key into the System Keystore.  This key then can be used
 * directly by the cryptographic hardware.  It later then be wrapped
 * (cryptographically obscured) by #fsl_shw_extract_key() and stored for later
 * use.  If a software key (#FSL_SKO_KEY_SW_KEY) was established, then its
 * value can be retrieved with a call to #fsl_shw_read_key().
 *
 * The wrapping and unwrapping functions provide security against unauthorized
 * use and detection of tampering.
 *
 * The functions can also be used with a User Keystore.
 *
 * @section smalloc_sec Secure Memory Allocation
 *
 * On platforms with multiple partitions of Secure Memory, the function
 * #fsl_shw_smalloc() can be used to acquire a partition for private use.  The
 * function #fsl_shw_diminish_perms() can then be used to revoke specific
 * permissions on the partition, and #fsl_shw_sfree() can be used to release the
 * partition.
 *
 * @section keystore_sec User Keystore
 *
 * User Keystore functionality is defined in fsl_shw_keystore.h.  See @ref
 * user_keystore for details.  This is not supported on platforms without SCC2.
 *
 * @section di_sec Hardware key-select extensions - DryIce
 *
 * Some platforms have a component called DryIce which allows the software to
 * control which key will be used by the secure memory encryption hardware.
 * The choices are the secret per-chip Fused (IIM) Key, an unknown, hardware-
 * generated Random Key, a software-written Programmed Key, or the IIM Key in
 * combination with one of the others.  #fsl_shw_pco_check_pk_supported() can
 * be used to determine whether this feature is available on the platform.
 * The rest of this section will explain the symmetric ciphering and key
 * operations which are available on such a platform.
 *
 * The function #fsl_shw_sko_init_pf_key() will set up a Secret Key Object to
 * refer to one of the system's platform keys.  All keys which reference a
 * platform key must use this initialization function, including a user-
 * provided key value.  Keys which are intended for software encryption must
 * use #fsl_shw_sko_init().
 *
 * To change the setting of the Programmed Key of the DryIce module,
 * #fsl_shw_establish_key() must be called with a platform key object of type
 * #FSL_SHW_PF_KEY_PRG or #FSL_SHW_PF_KEY_IIM_PRG.  The key will be go
 * into the PK register of DryIce and not to the keystore.  Any symmetric
 * operation which references either #FSL_SHW_PF_KEY_PRG or
 * #FSL_SHW_PF_KEY_IIM_PRG will use the current PK value (possibly modified by
 * the secret fused IIM key).  Before the Flatform Key can be changed, a call to
 * #fsl_shw_release_key() or #fsl_shw_extract_key() must be made.  Neither
 * function will change the value in the PK registers, and further ciphering
 * can take place.
 *
 * When #fsl_shw_establish_key() is called to change the PK value, a plaintext
 * key can be passed in with the #FSL_KEY_WRAP_ACCEPT argument or a previously
 * wrapped key can be passed in with the #FSL_KEY_WRAP_UNWRAP argument.  If
 * #FSL_KEY_WRAP_CREATE is passed in, then a random value will be loaded into
 * the PK register.  The PK value can be wrapped by a call to
 * #fsl_shw_extract_key() for later use with the #FSL_KEY_WRAP_UNWRAP argument.
 *
 * As an alternative to using only the fused key for @ref wrap_sec,
 * #fsl_shw_uco_set_wrap_key() can be used to select either the random key or
 * the random key with the fused key as the key which will be used to protect
 * the one-time value used to wrap the key.  This allows for these
 * wrapped keys to be dependent upon and therefore unrecoverable after a tamper
 * event causes the erasure of the DryIce Random Key register.
 *
 * The software can request that the hardware generate a (new) Random Key for
 * DryIce by calling #fsl_shw_gen_random_pf_key().
 *
 *
 * @section td_sec Device Tamper-Detection
 *
 * Some platforms have a component which can detect certain types of tampering
 * with the hardware.  #fsl_shw_read_tamper_event() API will allow the
 * retrieval of the type of event which caused a tamper-detection failure.
 *
 */

/*! @defgroup glossary Glossary
 *
 * @li @b AES - Advanced Encryption Standard - An NIST-created block cipher
 *          originally knowns as Rijndael.
 * @li @b ARC4 - ARCFOUR - An S-Box-based OFB mode stream cipher.
 * @li @b CBC - Cipher-Block Chaining - Each encrypted block is XORed with the
 *          result of the previous block's encryption.
 * @li @b CCM - A way of combining CBC and CTR to perform cipher and
 *        authentication.
 * @li @b ciphertext - @a plaintext which has been encrypted in some fashion.
 * @li @b context - Information on the state of a cryptographic operation,
 *        excluding any key.  This could include IV, Counter Value, or SBox.
 * @li @b CTR - A mode where a counter value is encrypted and then XORed with
 *        the data.  After each block, the counter value is incremented.
 * @li @b DES - Data Encryption Standard - An 8-octet-block cipher.
 * @li @b ECB - Electronic Codebook - A straight encryption/decryption of the
 *        data.
 * @li @b hash - A cryptographically strong one-way function performed on data.
 * @li @b HMAC - Hashed Message Authentication Code - A key-dependent one-way
 *        hash result, used to verify authenticity of a message.  The equation
 *        for an HMAC is hash((K + A) || hash((K + B) || msg)), where K is the
 *        key, A is the constant for the outer hash, B is the constant for the
 *        inner hash, and hash is the hashing function (MD5, SHA256, etc).
 * @li @b IPAD - In an HMAC operation, the context generated by XORing the key
 *        with a constant and then hashing that value as the first block of the
 *        inner hash.
 * @li @b IV - An "Initial Vector" or @a context for modes like CBC.
 * @li @b MAC - A Message Authentication Code.  HMAC, hashing, and CCM all
 *        produce a MAC.
 * @li @b mode  - A way of using a cryptographic algorithm.  See ECB, CBC, etc.
 * @li @b MD5 - Message Digest 5 - A one-way hash function.
 * @li @b plaintext - Data which has not been encrypted, or has been decrypted
 *        from @a ciphertext.
 * @li @b OPAD - In an HMAC operation, the context generated by XORing the key
 *        with a constant and then hashing that value as the first block of the
 *        outer hash.
 * @li @b SHA - Secure Hash Algorithm - A one-way hash function.
 * @li @b TDES - AKA @b 3DES - Triple Data Encryption Standard - A method of
 *        using two or three keys and DES to perform three operations (encrypt
 *        decrypt encrypt) to create a new algorithm.
 * @li @b XOR - Exclusive-OR.  A Boolean arithmetic function.
 * @li @b Wrapped value - A (key) which has been encrypted into an opaque datum
 *        which cannot be unwrapped (decrypted) for use except by an authorized
 *        user.  Once created, the key is never visible, but may be used for
 *        other cryptographic operations.
 */

#ifndef FSL_SHW_H
#define FSL_SHW_H

/* Set FSL_HAVE_* flags */

#include "fsl_platform.h"

#ifndef API_DOC

#if defined(FSL_HAVE_SAHARA2) || defined(FSL_HAVE_SAHARA4)

#include "sahara.h"

#else

#if defined(FSL_HAVE_RNGA) || defined(FSL_HAVE_RNGB) || defined(FSL_HAVE_RNGC)

#include "rng_driver.h"

#else

#error FSL_SHW_API_platform_not_recognized

#endif

#endif				/* HAVE SAHARA */

#else				/* API_DOC */

#include <inttypes.h>		/* for uint32_t, etc. */
#include <stdio.h>		/* Mainly for definition of NULL !! */

/* These groups will appear in the order in which they are defined. */

/*!
 * @defgroup strgrp Objects
 *
 * These objects are used to pass information into and out of the API.  Through
 * flags and other settings, they control the behavior of the @ref opfuns.
 *
 * They are manipulated and queried by use of the various access functions.
 * There are different sets defined for each object.  See @ref objman.
 */

/*!
 * @defgroup consgrp Enumerations and other Constants
 *
 * This collection of symbols comprise the values which can be passed into
 * various functions to control how the API will work.
 */

/*! @defgroup opfuns Operational Functions
 *
 * These functions request that the underlying hardware perform cryptographic
 * operations.  They are the heart of the API.
 */

/******  Organization the Object Operations under one group ! **********/
/*! @defgroup objman Object-Manipulation Operations
 *
 */
/*! @addtogroup objman
    @{ */
/*!
 * @defgroup pcoops Platform Context Object Operations
 *
 * The Platform Context object is "read-only", so only query operations are
 * provided for it.  It is returned by the #fsl_shw_get_capabilities()
 * function.
 */

/*! @defgroup ucoops User Context Operations
 *
 * These operations should be the only access to the #fsl_shw_uco_t
 * type/struct, as the internal members of the object are subject to change.
 * The #fsl_shw_uco_init() function must be called before any other use of the
 * object.
 */

/*!
 * @defgroup rops Result Object Operations
 *
 * As the Result Object contains the result of one of the @ref opfuns.  The
 * manipulations provided are query-only.  No initialization is needed for this
 * object.
 */

/*!
 * @defgroup skoops Secret Key Object Operations
 *
 * These operations should be the only access to the #fsl_shw_sko_t
 * type/struct, as the internal members of that object are subject to change.
 */

/*!
 * @defgroup ksoops Keystore Object Operations
 *
 * These operations should be the only access to the #fsl_shw_kso_t
 * type/struct, as the internal members of that object are subject to change.
 */

/*!
 * @defgroup hcops Hash Context Object Operations
 *
 * These operations should be the only access to the #fsl_shw_hco_t
 * type/struct, as the internal members of that object are subject to change.
 */

/*!
 * @defgroup hmcops HMAC Context Object Operations
 *
 * These operations should be the only access to the #fsl_shw_hmco_t
 * type/struct, as the internal members of that object are subject to change.
 */

/*!
 * @defgroup sccops Symmetric Cipher Context Operations
 *
 * These operations should be the only access to the #fsl_shw_scco_t
 * type/struct, as the internal members of that object are subject to change
 */

/*! @defgroup accoops Authentication-Cipher Context Object Operations
 *
 * These functions operate on a #fsl_shw_acco_t.  Their purpose is to set
 * flags, fields, etc., in order to control the operation of
 * #fsl_shw_gen_encrypt() and #fsl_shw_auth_decrypt().
 */

	 /* @} *//************ END GROUPING of Object Manipulations *****************/

/*! @defgroup miscfuns Miscellaneous Functions
 *
 * These functions are neither @ref opfuns nor @ref objman.  Their behavior
 * does not depend upon the flags in the #fsl_shw_uco_t, yet they may involve
 * more interaction with the library and the kernel than simply querying an
 * object.
 */

/******************************************************************************
 * Enumerations
 *****************************************************************************/
/*! @addtogroup consgrp
    @{ */

/*!
 * Flags for the state of the User Context Object (#fsl_shw_uco_t).
 *
 * These flags describe how the @ref opfuns will operate.
 */
typedef enum fsl_shw_user_ctx_flags_t {
	/*!
	 * API will block the caller until operation completes.  The result will be
	 * available in the return code.  If this is not set, user will have to get
	 * results using #fsl_shw_get_results().
	 */
	FSL_UCO_BLOCKING_MODE,
	/*!
	 * User wants callback (at the function specified with
	 * #fsl_shw_uco_set_callback()) when the operation completes.  This flag is
	 * valid only if #FSL_UCO_BLOCKING_MODE is not set.
	 */
	FSL_UCO_CALLBACK_MODE,
	/*! Do not free descriptor chain after driver (adaptor) finishes */
	FSL_UCO_SAVE_DESC_CHAIN,
	/*!
	 * User has made at least one request with callbacks requested, so API is
	 * ready to handle others.
	 */
	FSL_UCO_CALLBACK_SETUP_COMPLETE,
	/*!
	 * (virtual) pointer to descriptor chain is completely linked with physical
	 * (DMA) addresses, ready for the hardware.  This flag should not be used
	 * by FSL SHW API programs.
	 */
	FSL_UCO_CHAIN_PREPHYSICALIZED,
	/*!
	 * The user has changed the context but the changes have not been copied to
	 * the kernel driver.
	 */
	FSL_UCO_CONTEXT_CHANGED,
	/*! Internal Use.  This context belongs to a user-mode API user. */
	FSL_UCO_USERMODE_USER,
} fsl_shw_user_ctx_flags_t;

/*!
 * Return code for FSL_SHW library.
 *
 * These codes may be returned from a function call.  In non-blocking mode,
 * they will appear as the status in a Result Object.
 */
typedef enum fsl_shw_return_t {
	/*!
	 * No error.  As a function return code in Non-blocking mode, this may
	 * simply mean that the operation was accepted for eventual execution.
	 */
	FSL_RETURN_OK_S = 0,
	/*! Failure for non-specific reason. */
	FSL_RETURN_ERROR_S,
	/*!
	 * Operation failed because some resource was not able to be allocated.
	 */
	FSL_RETURN_NO_RESOURCE_S,
	/*! Crypto algorithm unrecognized or improper. */
	FSL_RETURN_BAD_ALGORITHM_S,
	/*! Crypto mode unrecognized or improper. */
	FSL_RETURN_BAD_MODE_S,
	/*! Flag setting unrecognized or inconsistent. */
	FSL_RETURN_BAD_FLAG_S,
	/*! Improper or unsupported key length for algorithm. */
	FSL_RETURN_BAD_KEY_LENGTH_S,
	/*! Improper parity in a (DES, TDES) key. */
	FSL_RETURN_BAD_KEY_PARITY_S,
	/*!
	 * Improper or unsupported data length for algorithm or internal buffer.
	 */
	FSL_RETURN_BAD_DATA_LENGTH_S,
	/*! Authentication / Integrity Check code check failed. */
	FSL_RETURN_AUTH_FAILED_S,
	/*! A memory error occurred. */
	FSL_RETURN_MEMORY_ERROR_S,
	/*! An error internal to the hardware occurred. */
	FSL_RETURN_INTERNAL_ERROR_S,
	/*! ECC detected Point at Infinity */
	FSL_RETURN_POINT_AT_INFINITY_S,
	/*! ECC detected No Point at Infinity */
	FSL_RETURN_POINT_NOT_AT_INFINITY_S,
	/*! GCD is One */
	FSL_RETURN_GCD_IS_ONE_S,
	/*! GCD is not One */
	FSL_RETURN_GCD_IS_NOT_ONE_S,
	/*! Candidate is Prime */
	FSL_RETURN_PRIME_S,
	/*! Candidate is not Prime */
	FSL_RETURN_NOT_PRIME_S,
	/*! N register loaded improperly with even value */
	FSL_RETURN_EVEN_MODULUS_ERROR_S,
	/*! Divisor is zero. */
	FSL_RETURN_DIVIDE_BY_ZERO_ERROR_S,
	/*! Bad Exponent or Scalar value for Point Multiply */
	FSL_RETURN_BAD_EXPONENT_ERROR_S,
	/*! RNG hardware problem. */
	FSL_RETURN_OSCILLATOR_ERROR_S,
	/*! RNG hardware problem. */
	FSL_RETURN_STATISTICS_ERROR_S,
} fsl_shw_return_t;

/*!
 * Algorithm Identifier.
 *
 * Selection of algorithm will determine how large the block size of the
 * algorithm is.   Context size is the same length unless otherwise specified.
 * Selection of algorithm also affects the allowable key length.
 */
typedef enum fsl_shw_key_alg_t {
	FSL_KEY_ALG_HMAC,	/*!< Key will be used to perform an HMAC.  Key
				   size is 1 to 64 octets.  Block size is 64
				   octets. */
	FSL_KEY_ALG_AES,	/*!< Advanced Encryption Standard (Rijndael).
				   Block size is 16 octets.  Key size is 16
				   octets.  (The single choice of key size is a
				   Sahara platform limitation.) */
	FSL_KEY_ALG_DES,	/*!< Data Encryption Standard.  Block size is
				   8 octets.  Key size is 8 octets. */
	FSL_KEY_ALG_TDES,	/*!< 2- or 3-key Triple DES.  Block size is 8
				   octets.  Key size is 16 octets for 2-key
				   Triple DES, and 24 octets for 3-key. */
	FSL_KEY_ALG_ARC4	/*!< ARC4.  No block size.  Context size is 259
				   octets.  Allowed key size is 1-16 octets.
				   (The choices for key size are a Sahara
				   platform limitation.) */
} fsl_shw_key_alg_t;

/*!
 * Mode selector for Symmetric Ciphers.
 *
 * The selection of mode determines how a cryptographic algorithm will be
 * used to process the plaintext or ciphertext.
 *
 * For all modes which are run block-by-block (that is, all but
 * #FSL_SYM_MODE_STREAM), any partial operations must be performed on a text
 * length which is multiple of the block size.  Except for #FSL_SYM_MODE_CTR,
 * these block-by-block algorithms must also be passed a total number of octets
 * which is a multiple of the block size.
 *
 * In modes which require that the total number of octets of data be a multiple
 * of the block size (#FSL_SYM_MODE_ECB and #FSL_SYM_MODE_CBC), and the user
 * has a total number of octets which are not a multiple of the block size, the
 * user must perform any necessary padding to get to the correct data length.
 */
typedef enum fsl_shw_sym_mode_t {
	/*!
	 * Stream.  There is no associated block size.  Any request to process data
	 * may be of any length.  This mode is only for ARC4 operations, and is
	 * also the only mode used for ARC4.
	 */
	FSL_SYM_MODE_STREAM,

	/*!
	 * Electronic Codebook.  Each block of data is encrypted/decrypted.  The
	 * length of the data stream must be a multiple of the block size.  This
	 * mode may be used for DES, 3DES, and AES.  The block size is determined
	 * by the algorithm.
	 */
	FSL_SYM_MODE_ECB,
	/*!
	 * Cipher-Block Chaining.  Each block of data is encrypted/decrypted and
	 * then "chained" with the previous block by an XOR function.  Requires
	 * context to start the XOR (previous block).  This mode may be used for
	 * DES, 3DES, and AES.  The block size is determined by the algorithm.
	 */
	FSL_SYM_MODE_CBC,
	/*!
	 * Counter.  The counter is encrypted, then XORed with a block of data.
	 * The counter is then incremented (using modulus arithmetic) for the next
	 * block. The final operation may be non-multiple of block size.  This mode
	 * may be used for AES.  The block size is determined by the algorithm.
	 */
	FSL_SYM_MODE_CTR,
} fsl_shw_sym_mode_t;

/*!
 * Algorithm selector for Cryptographic Hash functions.
 *
 * Selection of algorithm determines how large the context and digest will be.
 * Context is the same size as the digest (resulting hash), unless otherwise
 * specified.
 */
typedef enum fsl_shw_hash_alg_t {
	FSL_HASH_ALG_MD5,	/*!< MD5 algorithm.  Digest is 16 octets. */
	FSL_HASH_ALG_SHA1,	/*!< SHA-1 (aka SHA or SHA-160) algorithm.
				   Digest is 20 octets. */
	FSL_HASH_ALG_SHA224,	/*!< SHA-224 algorithm.  Digest is 28 octets,
				   though context is 32 octets. */
	FSL_HASH_ALG_SHA256	/*!< SHA-256 algorithm.  Digest is 32
				   octets. */
} fsl_shw_hash_alg_t;

/*!
 * The type of Authentication-Cipher function which will be performed.
 */
typedef enum fsl_shw_acc_mode_t {
	/*!
	 * CBC-MAC for Counter.  Requires context and modulus.  Final operation may
	 * be non-multiple of block size.  This mode may be used for AES.
	 */
	FSL_ACC_MODE_CCM,
	/*!
	 * SSL mode.  Not supported.  Combines HMAC and encrypt (or decrypt).
	 * Needs one key object for encryption, another for the HMAC.  The usual
	 * hashing and symmetric encryption algorithms are supported.
	 */
	FSL_ACC_MODE_SSL,
} fsl_shw_acc_mode_t;

/*!
 * The operation which controls the behavior of #fsl_shw_establish_key().
 *
 * These values are passed to #fsl_shw_establish_key().
 */
typedef enum fsl_shw_key_wrap_t {
	FSL_KEY_WRAP_CREATE,	/*!< Generate a key from random values. */
	FSL_KEY_WRAP_ACCEPT,	/*!< Use the provided clear key. */
	FSL_KEY_WRAP_UNWRAP	/*!< Unwrap a previously wrapped key. */
} fsl_shw_key_wrap_t;

/* REQ-S2LRD-PINTFC-COA-HCO-001 */
/*!
 * Flags which control a Hash operation.
 *
 *  These may be combined by ORing them together.  See #fsl_shw_hco_set_flags()
 * and #fsl_shw_hco_clear_flags().
 */
typedef enum fsl_shw_hash_ctx_flags_t {
	FSL_HASH_FLAGS_INIT = 1,	/*!< Context is empty.  Hash is started
					   from scratch, with a message-processed
					   count of zero. */
	FSL_HASH_FLAGS_SAVE = 2,	/*!< Retrieve context from hardware after
					   hashing.  If used with the
					   #FSL_HASH_FLAGS_FINALIZE flag, the final
					   digest value will be saved in the
					   object. */
	FSL_HASH_FLAGS_LOAD = 4,	/*!< Place context into hardware before
					   hashing. */
	FSL_HASH_FLAGS_FINALIZE = 8,	/*!< PAD message and perform final digest
					   operation.  If user message is
					   pre-padded, this flag should not be
					   used. */
} fsl_shw_hash_ctx_flags_t;

/*!
 * Flags which control an HMAC operation.
 *
 * These may be combined by ORing them together.  See #fsl_shw_hmco_set_flags()
 * and #fsl_shw_hmco_clear_flags().
 */
typedef enum fsl_shw_hmac_ctx_flags_t {
	FSL_HMAC_FLAGS_INIT = 1,	/*!< Message context is empty.  HMAC is
					   started from scratch (with key) or from
					   precompute of inner hash, depending on
					   whether
					   #FSL_HMAC_FLAGS_PRECOMPUTES_PRESENT is
					   set. */
	FSL_HMAC_FLAGS_SAVE = 2,	/*!< Retrieve ongoing context from hardware
					   after hashing.  If used with the
					   #FSL_HMAC_FLAGS_FINALIZE flag, the final
					   digest value (HMAC) will be saved in the
					   object. */
	FSL_HMAC_FLAGS_LOAD = 4,	/*!< Place ongoing context into hardware
					   before hashing. */
	FSL_HMAC_FLAGS_FINALIZE = 8,	/*!< PAD message and perform final HMAC
					   operations of inner and outer hashes. */
	FSL_HMAC_FLAGS_PRECOMPUTES_PRESENT = 16	/*!< This means that the context
						   contains precomputed inner and outer
						   hash values. */
} fsl_shw_hmac_ctx_flags_t;

/*!
 * Flags to control use of the #fsl_shw_scco_t.
 *
 * These may be ORed together to get the desired effect.
 * See #fsl_shw_scco_set_flags() and #fsl_shw_scco_clear_flags()
 */
typedef enum fsl_shw_sym_ctx_flags_t {
	/*!
	 * Context is empty.  In ARC4, this means that the S-Box needs to be
	 * generated from the key.  In #FSL_SYM_MODE_CBC mode, this allows an IV of
	 * zero to be specified.  In #FSL_SYM_MODE_CTR mode, it means that an
	 * initial CTR value of zero is desired.
	 */
	FSL_SYM_CTX_INIT = 1,
	/*!
	 * Load context from object into hardware before running cipher.  In
	 * #FSL_SYM_MODE_CTR mode, this would refer to the Counter Value.
	 */
	FSL_SYM_CTX_LOAD = 2,
	/*!
	 * Save context from hardware into object after running cipher.  In
	 * #FSL_SYM_MODE_CTR mode, this would refer to the Counter Value.
	 */
	FSL_SYM_CTX_SAVE = 4,
	/*!
	 * Context (SBox) is to be unwrapped and wrapped on each use.
	 * This flag is unsupported.
	 * */
	FSL_SYM_CTX_PROTECT = 8,
} fsl_shw_sym_ctx_flags_t;

/*!
 * Flags which describe the state of the #fsl_shw_sko_t.
 *
 * These may be ORed together to get the desired effect.
 * See #fsl_shw_sko_set_flags() and #fsl_shw_sko_clear_flags()
 */
typedef enum fsl_shw_key_flags_t {
	FSL_SKO_KEY_IGNORE_PARITY = 1,	/*!< If algorithm is DES or 3DES, do not
					   validate the key parity bits. */
	FSL_SKO_KEY_PRESENT = 2,	/*!< Clear key is present in the object. */
	FSL_SKO_KEY_ESTABLISHED = 4,	/*!< Key has been established for use.  This
					   feature is not available for all
					   platforms, nor for all algorithms and
					   modes. */
	FSL_SKO_KEY_SW_KEY = 8,	/*!< This key is for software use, and can
				   be copied out of a keystore by its owner.
				   The default is that they key is available
				   only for hardware (or security driver)
				   use. */
} fsl_shw_key_flags_t;

/*!
 * Type of value which is associated with an established key.
 */
typedef uint64_t key_userid_t;

/*!
 * Flags which describe the state of the #fsl_shw_acco_t.
 *
 * The @a FSL_ACCO_CTX_INIT and @a FSL_ACCO_CTX_FINALIZE flags, when used
 * together, provide for a one-shot operation.
 */
typedef enum fsl_shw_auth_ctx_flags_t {
	FSL_ACCO_CTX_INIT = 1,	/*!< Initialize Context(s) */
	FSL_ACCO_CTX_LOAD = 2,	/*!< Load intermediate context(s).
				   This flag is unsupported. */
	FSL_ACCO_CTX_SAVE = 4,	/*!< Save intermediate context(s).
				   This flag is unsupported. */
	FSL_ACCO_CTX_FINALIZE = 8,	/*!< Create MAC during this operation. */
	FSL_ACCO_NIST_CCM = 16,	/*!< Formatting of CCM input data is
				   performed by calls to
				   #fsl_shw_ccm_nist_format_ctr_and_iv() and
				   #fsl_shw_ccm_nist_update_ctr_and_iv().  */
} fsl_shw_auth_ctx_flags_t;

/*!
 *  Modulus Selector for CTR modes.
 *
 * The incrementing of the Counter value may be modified by a modulus.  If no
 * modulus is needed or desired for AES, use #FSL_CTR_MOD_128.
 */
typedef enum fsl_shw_ctr_mod_t {
	FSL_CTR_MOD_8,		/*!< Run counter with modulus of 2^8. */
	FSL_CTR_MOD_16,		/*!< Run counter with modulus of 2^16. */
	FSL_CTR_MOD_24,		/*!< Run counter with modulus of 2^24. */
	FSL_CTR_MOD_32,		/*!< Run counter with modulus of 2^32. */
	FSL_CTR_MOD_40,		/*!< Run counter with modulus of 2^40. */
	FSL_CTR_MOD_48,		/*!< Run counter with modulus of 2^48. */
	FSL_CTR_MOD_56,		/*!< Run counter with modulus of 2^56. */
	FSL_CTR_MOD_64,		/*!< Run counter with modulus of 2^64. */
	FSL_CTR_MOD_72,		/*!< Run counter with modulus of 2^72. */
	FSL_CTR_MOD_80,		/*!< Run counter with modulus of 2^80. */
	FSL_CTR_MOD_88,		/*!< Run counter with modulus of 2^88. */
	FSL_CTR_MOD_96,		/*!< Run counter with modulus of 2^96. */
	FSL_CTR_MOD_104,	/*!< Run counter with modulus of 2^104. */
	FSL_CTR_MOD_112,	/*!< Run counter with modulus of 2^112. */
	FSL_CTR_MOD_120,	/*!< Run counter with modulus of 2^120. */
	FSL_CTR_MOD_128		/*!< Run counter with modulus of 2^128. */
} fsl_shw_ctr_mod_t;

/*!
 * Permissions flags for Secure Partitions
 *
 * They currently map directly to the SCC2 hardware values, but this is not
 * guarinteed behavior.
 */
typedef enum fsl_shw_permission_t {
/*! SCM Access Permission: Do not zeroize/deallocate partition on SMN Fail state */
	FSL_PERM_NO_ZEROIZE,
/*! SCM Access Permission: Enforce trusted key read in  */
	FSL_PERM_TRUSTED_KEY_READ,
/*! SCM Access Permission: Ignore Supervisor/User mode in permission determination */
	FSL_PERM_HD_S,
/*! SCM Access Permission: Allow Read Access to  Host Domain */
	FSL_PERM_HD_R,
/*! SCM Access Permission: Allow Write Access to  Host Domain */
	FSL_PERM_HD_W,
/*! SCM Access Permission: Allow Execute Access to  Host Domain */
	FSL_PERM_HD_X,
/*! SCM Access Permission: Allow Read Access to Trusted Host Domain */
	FSL_PERM_TH_R,
/*! SCM Access Permission: Allow Write Access to Trusted Host Domain */
	FSL_PERM_TH_W,
/*! SCM Access Permission: Allow Read Access to Other/World Domain */
	FSL_PERM_OT_R,
/*! SCM Access Permission: Allow Write Access to Other/World Domain */
	FSL_PERM_OT_W,
/*! SCM Access Permission: Allow Execute Access to Other/World Domain */
	FSL_PERM_OT_X,
} fsl_shw_permission_t;

/*!
 * Select the cypher mode to use for partition cover/uncover operations.
 *
 * They currently map directly to the values used in the SCC2 driver, but this
 * is not guarinteed behavior.
 */
typedef enum fsl_shw_cypher_mode_t {
	FSL_SHW_CYPHER_MODE_ECB,	/*!< ECB mode */
	FSL_SHW_CYPHER_MODE_CBC,	/*!< CBC mode */
} fsl_shw_cypher_mode_t;

/*!
 * Which platform key should be presented for cryptographic use.
 */
typedef enum fsl_shw_pf_key_t {
	FSL_SHW_PF_KEY_IIM,	/*!< Present fused IIM key */
	FSL_SHW_PF_KEY_PRG,	/*!< Present Program key */
	FSL_SHW_PF_KEY_IIM_PRG,	/*!< Present IIM ^ Program key */
	FSL_SHW_PF_KEY_IIM_RND,	/*!< Present Random key */
	FSL_SHW_PF_KEY_RND,	/*!< Present IIM ^ Random key */
} fsl_shw_pf_key_t;

/*!
 * The various security tamper events
 */
typedef enum fsl_shw_tamper_t {
	FSL_SHW_TAMPER_NONE,	/*!< No error detected */
	FSL_SHW_TAMPER_WTD,	/*!< wire-mesh tampering det */
	FSL_SHW_TAMPER_ETBD,	/*!< ext tampering det: input B */
	FSL_SHW_TAMPER_ETAD,	/*!< ext tampering det: input A */
	FSL_SHW_TAMPER_EBD,	/*!< external boot detected */
	FSL_SHW_TAMPER_SAD,	/*!< security alarm detected */
	FSL_SHW_TAMPER_TTD,	/*!< temperature tampering det */
	FSL_SHW_TAMPER_CTD,	/*!< clock tampering det */
	FSL_SHW_TAMPER_VTD,	/*!< voltage tampering det */
	FSL_SHW_TAMPER_MCO,	/*!< monotonic counter overflow */
	FSL_SHW_TAMPER_TCO,	/*!< time counter overflow */
} fsl_shw_tamper_t;

/*! @} *//* consgrp */

/******************************************************************************
 * Data Structures
 *****************************************************************************/
/*! @addtogroup strgrp
    @{ */

/* REQ-S2LRD-PINTFC-COA-IBO-001 */
/*!
 * Application Initialization Object
 *
 * This object, the operations on it, and its interaction with the driver are
 * TBD.
 */
typedef struct fsl_sho_ibo_t {
} fsl_sho_ibo_t;

/* REQ-S2LRD-PINTFC-COA-UCO-001 */
/*!
 * User Context Object
 *
 * This object must be initialized by a call to #fsl_shw_uco_init().  It must
 * then be passed to #fsl_shw_register_user() before it can be used in any
 * calls besides those in @ref ucoops.
 *
 * It contains the user's configuration for the API, for instance whether an
 * operation should block, or instead should call back the user upon completion
 * of the operation.
 *
 * See @ref ucoops for further information.
 */
typedef struct fsl_shw_uco_t {	/* fsl_shw_user_context_object */
} fsl_shw_uco_t;

/* REQ-S2LRD-PINTFC-API-GEN-006  ??  */
/*!
 * Result Object
 *
 * This object will contain success and failure information about a specific
 * cryptographic request which has been made.
 *
 * No direct access to its members should be made by programs.  Instead, the
 * object should be manipulated using the provided functions.  See @ref rops.
 */
typedef struct fsl_shw_result_t {	/* fsl_shw_result */
} fsl_shw_result_t;

/*!
 * Keystore Object
 *
 * This object holds the context of a user keystore, including the functions
 * that define the interface and pointers to where the key data is stored.  The
 * user must supply a set of functions to handle keystore management, including
 * slot allocation, deallocation, etc.  A default keystore manager is provided
 * as part of the API.
 * 
 * No direct access to its members should be made by programs.  Instead, the
 * object should be manipulated using the provided functions.  See @ref ksoops.
 */
typedef struct fsl_shw_kso_t {	/* fsl_shw_keystore_object */
} fsl_shw_kso_t;

/* REQ-S2LRD-PINTFC-COA-SKO-001 */
/*!
 * Secret Key Object
 *
 * This object contains a key for a cryptographic operation, and information
 * about its current state, its intended usage, etc.  It may instead contain
 * information about a protected key, or an indication to use a platform-
 * specific secret key.
 *
 * No direct access to its members should be made by programs.  Instead, the
 * object should be manipulated using the provided functions.  See @ref skoops.
 */
typedef struct fsl_shw_sko_t {	/* fsl_shw_secret_key_object */
} fsl_shw_sko_t;

/* REQ-S2LRD-PINTFC-COA-CO-001 */
/*!
 * Platform Capabilities Object
 *
 * This object will contain information about the cryptographic features of the
 * platform which the program is running on.
 *
 * No direct access to its members should be made by programs.  Instead, the
 * object should be manipulated using the provided functions.
 *
 * See @ref pcoops.
 */
typedef struct fsl_shw_pco_t {	/* fsl_shw_platform_capabilities_object */
} fsl_shw_pco_t;

/* REQ-S2LRD-PINTFC-COA-HCO-001 */
/*!
 * Hash Context Object
 *
 * This object contains information to control hashing functions.

 * No direct access to its members should be made by programs.  Instead, the
 * object should be manipulated using the provided functions.  See @ref hcops.
 */
typedef struct fsl_shw_hco_t {	/* fsl_shw_hash_context_object */
} fsl_shw_hco_t;

/*!
 * HMAC Context Object
 *
 * This object contains information to control HMAC functions.

 * No direct access to its members should be made by programs.  Instead, the
 * object should be manipulated using the provided functions.  See @ref hmcops.
 */
typedef struct fsl_shw_hmco_t {	/* fsl_shw_hmac_context_object */
} fsl_shw_hmco_t;

/* REQ-S2LRD-PINTFC-COA-SCCO-001 */
/*!
 * Symmetric Cipher Context Object
 *
 * This object contains information to control Symmetric Ciphering encrypt and
 * decrypt functions in #FSL_SYM_MODE_STREAM (ARC4), #FSL_SYM_MODE_ECB,
 * #FSL_SYM_MODE_CBC, and #FSL_SYM_MODE_CTR modes and the
 * #fsl_shw_symmetric_encrypt() and #fsl_shw_symmetric_decrypt() functions.
 * CCM mode is controlled with the #fsl_shw_acco_t object.
 *
 * No direct access to its members should be made by programs.  Instead, the
 * object should be manipulated using the provided functions.  See @ref sccops.
 */
typedef struct fsl_shw_scco_t {	/* fsl_shw_symmetric_cipher_context_object */
} fsl_shw_scco_t;

/*!
 * Authenticate-Cipher Context Object

 * An object for controlling the function of, and holding information about,
 * data for the authenticate-cipher functions, #fsl_shw_gen_encrypt() and
 * #fsl_shw_auth_decrypt().
 *
 * No direct access to its members should be made by programs.  Instead, the
 * object should be manipulated using the provided functions.  See @ref
 * accoops.
 */
typedef struct fsl_shw_acco_t {	/* fsl_shw_authenticate_cipher_context_object */
} fsl_shw_acco_t;
	  /*! @} *//* strgrp */

/******************************************************************************
 * Access Macros for Objects
 *****************************************************************************/
/*! @addtogroup pcoops
    @{ */

/*!
 * Get FSL SHW API version
 *
 * @param      pc_info   The Platform Capabilities Object to query.
 * @param[out] major     A pointer to where the major version
 *                       of the API is to be stored.
 * @param[out] minor     A pointer to where the minor version
 *                       of the API is to be stored.
 */
void fsl_shw_pco_get_version(const fsl_shw_pco_t * pc_info,
			     uint32_t * major, uint32_t * minor);

/*!
 * Get underlying driver version.
 *
 * @param      pc_info   The Platform Capabilities Object to query.
 * @param[out] major     A pointer to where the major version
 *                       of the driver is to be stored.
 * @param[out] minor     A pointer to where the minor version
 *                       of the driver is to be stored.
 */
void fsl_shw_pco_get_driver_version(const fsl_shw_pco_t * pc_info,
				    uint32_t * major, uint32_t * minor);

/*!
 * Get list of symmetric algorithms supported.
 *
 * @param pc_info   The Platform Capabilities Object to query.
 * @param[out] algorithms A pointer to where to store the location of
 *                        the list of algorithms.
 * @param[out] algorithm_count A pointer to where to store the number of
 *                             algorithms in the list at @a algorithms.
 */
void fsl_shw_pco_get_sym_algorithms(const fsl_shw_pco_t * pc_info,
				    fsl_shw_key_alg_t * algorithms[],
				    uint8_t * algorithm_count);

/*!
 * Get list of symmetric modes supported.
 *
 * @param pc_info         The Platform Capabilities Object to query.
 * @param[out] modes      A pointer to where to store the location of
 *                        the list of modes.
 * @param[out] mode_count A pointer to where to store the number of
 *                        algorithms in the list at @a modes.
 */
void fsl_shw_pco_get_sym_modes(const fsl_shw_pco_t * pc_info,
			       fsl_shw_sym_mode_t * modes[],
			       uint8_t * mode_count);

/*!
 * Get list of hash algorithms supported.
 *
 * @param pc_info         The Platform Capabilities Object to query.
 * @param[out] algorithms A pointer which will be set to the list of
 *                        algorithms.
 * @param[out] algorithm_count The number of algorithms in the list at @a
 *                             algorithms.
 */
void fsl_shw_pco_get_hash_algorithms(const fsl_shw_pco_t * pc_info,
				     fsl_shw_hash_alg_t * algorithms[],
				     uint8_t * algorithm_count);

/*!
 * Determine whether the combination of a given symmetric algorithm and a given
 * mode is supported.
 *
 * @param pc_info    The Platform Capabilities Object to query.
 * @param algorithm  A Symmetric Cipher algorithm.
 * @param mode       A Symmetric Cipher mode.
 *
 * @return 0 if combination is not supported, non-zero if supported.
 */
int fsl_shw_pco_check_sym_supported(const fsl_shw_pco_t * pc_info,
				    fsl_shw_key_alg_t algorithm,
				    fsl_shw_sym_mode_t mode);

/*!
 * Determine whether a given Encryption-Authentication mode is supported.
 *
 * @param pc_info   The Platform Capabilities Object to query.
 * @param mode       The Authentication mode.
 *
 * @return 0 if mode is not supported, non-zero if supported.
 */
int fsl_shw_pco_check_auth_supported(const fsl_shw_pco_t * pc_info,
				     fsl_shw_acc_mode_t mode);

/*!
 * Determine whether Black Keys (key establishment / wrapping) is supported.
 *
 * @param pc_info  The Platform Capabilities Object to query.
 *
 * @return 0 if wrapping is not supported, non-zero if supported.
 */
int fsl_shw_pco_check_black_key_supported(const fsl_shw_pco_t * pc_info);

/*!
 * Get FSL SHW SCC driver version
 *
 * @param      pc_info   The Platform Capabilities Object to query.
 * @param[out] major     A pointer to where the major version
 *                       of the SCC driver is to be stored.
 * @param[out] minor     A pointer to where the minor version
 *                       of the SCC driver is to be stored.
 */
void fsl_shw_pco_get_scc_driver_version(const fsl_shw_pco_t * pc_info,
					uint32_t * major, uint32_t * minor);

/*!
 * Get SCM hardware version
 *
 * @param      pc_info   The Platform Capabilities Object to query.
 * @return               The SCM hardware version
 */
uint32_t fsl_shw_pco_get_scm_version(const fsl_shw_pco_t * pc_info);

/*!
 * Get SMN hardware version
 *
 * @param      pc_info   The Platform Capabilities Object to query.
 * @return               The SMN hardware version
 */
uint32_t fsl_shw_pco_get_smn_version(const fsl_shw_pco_t * pc_info);

/*!
 * Get the size of an SCM block, in bytes
 *
 * @param      pc_info   The Platform Capabilities Object to query.
 * @return               The size of an SCM block, in bytes.
 */
uint32_t fsl_shw_pco_get_scm_block_size(const fsl_shw_pco_t * pc_info);

/*!
 * Get size of Black and Red RAM memory
 *
 * @param      pc_info     The Platform Capabilities Object to query.
 * @param[out] black_size  A pointer to where the size of the Black RAM, in
 *                         blocks, is to be placed.
 * @param[out] red_size    A pointer to where the size of the Red RAM, in 
 *                         blocks, is to be placed.
 */
void fsl_shw_pco_get_smn_size(const fsl_shw_pco_t * pc_info,
			      uint32_t * black_size, uint32_t * red_size);

/*!
 * Determine whether Secure Partitions are supported
 *
 * @param pc_info          The Platform Capabilities Object to query.
 *
 * @return 0 if secure partitions are not supported, non-zero if supported.
 */
int fsl_shw_pco_check_spo_supported(const fsl_shw_pco_t * pc_info);

/*!
 * Get the size of a Secure Partitions
 *
 * @param pc_info          The Platform Capabilities Object to query.
 * 
 * @return Partition size, in bytes.  0 if Secure Partitions not supported.
 */
uint32_t fsl_shw_pco_get_spo_size_bytes(const fsl_shw_pco_t * pc_info);

/*!
 * Get the number of Secure Partitions on this platform
 *
 * @param pc_info          The Platform Capabilities Object to query.
 * 
 * @return Number of partitions. 0 if Secure Partitions not supported.  Note
 *         that this returns the total number of partitions, though
 *         not all may be available to the user.
 */
uint32_t fsl_shw_pco_get_spo_count(const fsl_shw_pco_t * pc_info);

/*!
 * Determine whether Platform Key features are available
 *
 * @param pc_info          The Platform Capabilities Object to query.
 *
 * @return  1 if Programmed Key features are available, otherwise zero.
 */
int fsl_shw_pco_check_pk_supported(const fsl_shw_pco_t * pc_info);

/*!
  * Determine whether Software Key features are available
  *
  * @param pc_info 		 The Platform Capabilities Object to query.
  *
  * @return  1 if Software key features are available, otherwise zero.
  */
int fsl_shw_pco_check_sw_keys_supported(const fsl_shw_pco_t * pc_info);

/*! @} *//* pcoops */

/*! @addtogroup ucoops
    @{ */

/*!
 * Initialize a User Context Object.
 *
 * This function must be called before performing any other operation with the
 * Object.  It sets the User Context Object to initial values, and set the size
 * of the results pool.  The mode will be set to a default of
 * #FSL_UCO_BLOCKING_MODE.
 *
 * When using non-blocking operations, this sets the maximum number of
 * operations which can be outstanding.  This number includes the counts of
 * operations waiting to start, operation(s) being performed, and results which
 * have not been retrieved.
 *
 * Changes to this value are ignored once user registration has completed.  It
 * should be set to 1 if only blocking operations will ever be performed.
 *
 * @param user_ctx     The User Context object to operate on.
 * @param pool_size    The maximum number of operations which can be
 *                     outstanding.
 */
void fsl_shw_uco_init(fsl_shw_uco_t * user_ctx, uint16_t pool_size);

/*!
 * Set the User Reference for the User Context.
 *
 * @param user_ctx     The User Context object to operate on.
 * @param reference    A value which will be passed back with a result.
 */
void fsl_shw_uco_set_reference(fsl_shw_uco_t * user_ctx, uint32_t reference);

/*!
 * Set the callback routine for the User Context.
 *
 * Note that the callback routine may be called when no results are available,
 * and possibly even when no requests are outstanding.
 *
 *
 * @param user_ctx     The User Context object to operate on.
 * @param callback_fn  The function the API will invoke when an operation
 *                     completes.
 */
void fsl_shw_uco_set_callback(fsl_shw_uco_t * user_ctx,
			      void (*callback_fn) (fsl_shw_uco_t * uco));

/*!
 * Set flags in the User Context.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param user_ctx     The User Context object to operate on.
 * @param flags        ORed values from #fsl_shw_user_ctx_flags_t.
 */
void fsl_shw_uco_set_flags(fsl_shw_uco_t * user_ctx, uint32_t flags);

/*!
 * Clear flags in the User Context.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param user_ctx     The User Context object to operate on.
 * @param flags        ORed values from #fsl_shw_user_ctx_flags_t.
 */
void fsl_shw_uco_clear_flags(fsl_shw_uco_t * user_ctx, uint32_t flags);

/*!
 * Select a key for the key-wrap key for key wrapping/unwrapping
 *
 * Without a call to this function, default is FSL_SHW_PF_KEY_IIM.  The wrap
 * key is used to encrypt and decrypt the per-key random secret which is used
 * to calculate the key which will encrypt/decrypt the user's key.
 *
 * @param user_ctx     The User Context object to operate on.
 * @param pf_key       Which key to use.  Valid choices are
 *                     #FSL_SHW_PF_KEY_IIM, #FSL_SHW_PF_KEY_RND, and
 *                     #FSL_SHW_PF_KEY_IIM_RND.
 */
void fsl_shw_uco_set_wrap_key(fsl_shw_uco_t * user_ctx,
			      fsl_shw_pf_key_t pf_key);

	  /*! @} *//* ucoops */

/*! @addtogroup rops
    @{ */

/*!
 * Retrieve the status code from a Result Object.
 *
 * @param result   The result object to query.
 *
 * @return The status of the request.
 */
fsl_shw_return_t fsl_shw_ro_get_status(fsl_shw_result_t * result);

/*!
 * Retrieve the reference value from a Result Object.
 *
 * @param result   The result object to query.
 *
 * @return The reference associated with the request.
 */
uint32_t fsl_shw_ro_get_reference(fsl_shw_result_t * result);

	 /* @} *//* rops */

/*! @addtogroup skoops
    @{ */

/*!
 * Initialize a Secret Key Object.
 *
 * This function or #fsl_shw_sko_init_pf_key() must be called before performing
 * any other operation with the Object.
 *
 * @param key_info  The Secret Key Object to be initialized.
 * @param algorithm DES, AES, etc.
 *
 */
void fsl_shw_sko_init(fsl_shw_sko_t * key_info, fsl_shw_key_alg_t algorithm);

/*!
 * Initialize a Secret Key Object to use a Platform Key register.
 *
 * This function or #fsl_shw_sko_init() must be called before performing any
 * other operation with the Object.  #fsl_shw_sko_set_key() does not work on
 * a key object initialized in this way.
 *
 * If this function is used to initialize the key object, but no key is
 * established with the key object, then the object will refer strictly to the
 * key value specified by the @c pf_key selection.
 *
 * If the pf key is #FSL_SHW_PF_KEY_PRG or #FSL_SHW_PF_KEY_IIM_PRG, then the
 * key object may be used with #fsl_shw_establish_key() to change the Program
 * Key value.  When the pf key is neither #FSL_SHW_PF_KEY_PRG nor
 * #FSL_SHW_PF_KEY_IIM_PRG, it is an error to call #fsl_shw_establish_key().
 *
 * @param key_info     The Secret Key Object to be initialized.
 * @param algorithm    DES, AES, etc.
 * @param pf_key       Which platform key is referenced.
 */
void fsl_shw_sko_init_pf_key(fsl_shw_sko_t * key_info,
			     fsl_shw_key_alg_t algorithm,
			     fsl_shw_pf_key_t pf_key);

/*!
 * Store a cleartext key in the key object.
 *
 * This has the side effect of setting the #FSL_SKO_KEY_PRESENT flag. It should
 * not be used if there is a key established with the key object.  If there is,
 * a call to #fsl_shw_release_key() should be made first.
 *
 * @param key_object   A variable of type #fsl_shw_sko_t.
 * @param key          A pointer to the beginning of the key.
 * @param key_length   The length, in octets, of the key.  The value should be
 *                     appropriate to the key size supported by the algorithm.
 *                     64 octets is the absolute maximum value allowed for this
 *                     call.
 */
void fsl_shw_sko_set_key(fsl_shw_sko_t * key_object,
			 const uint8_t * key, uint16_t key_length);

/*!
 * Set a size for the key.
 *
 * This function would normally be used when the user wants the key to be
 * generated from a random source.
 *
 * @param key_object   A variable of type #fsl_shw_sko_t.
 * @param key_length   The length, in octets, of the key.  The value should be
 *                     appropriate to the key size supported by the algorithm.
 *                     64 octets is the absolute maximum value allowed for this
 *                     call.
 */
void fsl_shw_sko_set_key_length(fsl_shw_sko_t * key_object,
				uint16_t key_length);

/*!
 * Set the User ID associated with the key.
 *
 * @param key_object   A variable of type #fsl_shw_sko_t.
 * @param userid       The User ID to identify authorized users of the key.
 */
void fsl_shw_sko_set_user_id(fsl_shw_sko_t * key_object, key_userid_t userid);

/*!
 * Set the keystore that the key will be stored in.
 *
 * @param key_object    A variable of type #fsl_shw_sko_t.
 * @param keystore      The keystore to place the key in.  This is a variable of
 *                      type #fsl_shw_kso_t.
 */
void fsl_shw_sko_set_keystore(fsl_shw_sko_t * key_object,
			      fsl_shw_kso_t * keystore);

/*!
 * Set the establish key handle into a key object.
 *
 * The @a userid field will be used to validate the access to the unwrapped
 * key.  This feature is not available for all platforms, nor for all
 * algorithms and modes.
 *
 * The #FSL_SKO_KEY_ESTABLISHED will be set (and the #FSL_SKO_KEY_PRESENT
 * flag will be cleared).
 *
 * @param key_object   A variable of type #fsl_shw_sko_t.
 * @param userid       The User ID to verify this user is an authorized user of
 *                     the key.
 * @param handle       A @a handle from #fsl_shw_sko_get_established_info.
 */
void fsl_shw_sko_set_established_info(fsl_shw_sko_t * key_object,
				      key_userid_t userid, uint32_t handle);

/*!
 * Extract the algorithm from a key object.
 *
 * @param      key_info  The Key Object to be queried.
 * @param[out] algorithm A pointer to the location to store the algorithm.
 */
void fsl_shw_sko_get_algorithm(const fsl_shw_sko_t * key_info,
			       fsl_shw_key_alg_t * algorithm);

/*!
 * Retrieve the cleartext key from a key object that is stored in a user
 * keystore.
 *
 * @param      skobject     The Key Object to be queried.
 * @param[out] skkey        A pointer to the location to store the key.  NULL
 *                          if the key is not stored in a user keystore.
 */
void fsl_shw_sko_get_key(const fsl_shw_sko_t * skobject, void *skkey);

/*!
 * Retrieve the established-key handle from a key object.
 *
 * @param key_object   A variable of type #fsl_shw_sko_t.
 * @param handle       The location to store the @a handle of the unwrapped
 *                     key.
 */
void fsl_shw_sko_get_established_info(fsl_shw_sko_t * key_object,
				      uint32_t * handle);

/*!
 * Determine the size of a wrapped key based upon the cleartext key's length.
 *
 * This function can be used to calculate the number of octets that
 * #fsl_shw_extract_key() will write into the location at @a covered_key.
 *
 * If zero is returned at @a length, this means that the key length in
 * @a key_info is not supported.
 *
 * @param      key_info         Information about a key to be wrapped.
 * @param      length           Location to store the length of a wrapped
 *                              version of the key in @a key_info.
 */
void fsl_shw_sko_calculate_wrapped_size(const fsl_shw_sko_t * key_info,
					uint32_t * length);

/*!
 * Set some flags in the key object.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param key_object   A variable of type #fsl_shw_sko_t.
 * @param flags        (One or more) ORed members of #fsl_shw_key_flags_t which
 *                     are to be set.
 */
void fsl_shw_sko_set_flags(fsl_shw_sko_t * key_object, uint32_t flags);

/*!
 * Clear some flags in the key object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param key_object   A variable of type #fsl_shw_sko_t.
 * @param flags        (One or more) ORed members of #fsl_shw_key_flags_t which
 *                      are to be reset.
 */
void fsl_shw_sko_clear_flags(fsl_shw_sko_t * key_object, uint32_t flags);

	  /*! @} *//* end skoops */

/*****************************************************************************/

/*! @addtogroup hcops
    @{ */

/*****************************************************************************/
/* REQ-S2LRD-PINTFC-API-BASIC-HASH-004 - partially */
/*!
 * Initialize a Hash Context Object.
 *
 * This function must be called before performing any other operation with the
 * Object.  It sets the current message length and hash algorithm in the hash
 * context object.
 *
 * @param      hash_ctx  The hash context to operate upon.
 * @param      algorithm The hash algorithm to be used (#FSL_HASH_ALG_MD5,
 *                       #FSL_HASH_ALG_SHA256, etc).
 *
 */
void fsl_shw_hco_init(fsl_shw_hco_t * hash_ctx, fsl_shw_hash_alg_t algorithm);

/*****************************************************************************/
/* REQ-S2LRD-PINTFC-API-BASIC-HASH-001 */
/* REQ-S2LRD-PINTFC-API-BASIC-HASH-002 */
/*!
 * Get the current hash value and message length from the hash context object.
 *
 * The algorithm must have already been specified.  See #fsl_shw_hco_init().
 *
 * @param      hash_ctx  The hash context to query.
 * @param[out] digest    Pointer to the location of @a length octets where to
 *                       store a copy of the current value of the digest.
 * @param      length    Number of octets of hash value to copy.
 * @param[out] msg_length Pointer to the location to store the number of octets
 *                        already hashed.
 */
void fsl_shw_hco_get_digest(const fsl_shw_hco_t * hash_ctx, uint8_t * digest,
			    uint8_t length, uint32_t * msg_length);

/*****************************************************************************/
/* REQ-S2LRD-PINTFC-API-BASIC-HASH-002 - partially */
/*!
 * Get the hash algorithm from the hash context object.
 *
 * @param      hash_ctx  The hash context to query.
 * @param[out] algorithm Pointer to where the algorithm is to be stored.
 */
void fsl_shw_hco_get_info(const fsl_shw_hco_t * hash_ctx,
			  fsl_shw_hash_alg_t * algorithm);

/*****************************************************************************/
/* REQ-S2LRD-PINTFC-API-BASIC-HASH-003 */
/* REQ-S2LRD-PINTFC-API-BASIC-HASH-004 */
/*!
 * Set the current hash value and message length in the hash context object.
 *
 * The algorithm must have already been specified.  See #fsl_shw_hco_init().
 *
 * @param      hash_ctx  The hash context to operate upon.
 * @param      context   Pointer to buffer of appropriate length to copy into
 *                       the hash context object.
 * @param      msg_length The number of octets of the message which have
 *                        already been hashed.
 *
 */
void fsl_shw_hco_set_digest(fsl_shw_hco_t * hash_ctx, const uint8_t * context,
			    uint32_t msg_length);

/*!
 * Set flags in a Hash Context Object.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param hash_ctx   The hash context to be operated on.
 * @param flags      The flags to be set in the context.  These can be ORed
 *                   members of #fsl_shw_hash_ctx_flags_t.
 */
void fsl_shw_hco_set_flags(fsl_shw_hco_t * hash_ctx, uint32_t flags);

/*!
 * Clear flags in a Hash Context Object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param hash_ctx   The hash context to be operated on.
 * @param flags      The flags to be reset in the context.  These can be ORed
 *                   members of #fsl_shw_hash_ctx_flags_t.
 */
void fsl_shw_hco_clear_flags(fsl_shw_hco_t * hash_ctx, uint32_t flags);

	  /*! @} *//* end hcops */

/*****************************************************************************/

/*! @addtogroup hmcops
    @{ */

/*!
 * Initialize an HMAC Context Object.
 *
 * This function must be called before performing any other operation with the
 * Object.  It sets the current message length and hash algorithm in the HMAC
 * context object.
 *
 * @param      hmac_ctx  The HMAC context to operate upon.
 * @param      algorithm The hash algorithm to be used (#FSL_HASH_ALG_MD5,
 *                       #FSL_HASH_ALG_SHA256, etc).
 *
 */
void fsl_shw_hmco_init(fsl_shw_hmco_t * hmac_ctx, fsl_shw_hash_alg_t algorithm);

/*!
 * Set flags in an HMAC Context Object.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param hmac_ctx   The HMAC context to be operated on.
 * @param flags      The flags to be set in the context.  These can be ORed
 *                   members of #fsl_shw_hmac_ctx_flags_t.
 */
void fsl_shw_hmco_set_flags(fsl_shw_hmco_t * hmac_ctx, uint32_t flags);

/*!
 * Clear flags in an HMAC Context Object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param hmac_ctx   The HMAC context to be operated on.
 * @param flags      The flags to be reset in the context.  These can be ORed
 *                   members of #fsl_shw_hmac_ctx_flags_t.
 */
void fsl_shw_hmco_clear_flags(fsl_shw_hmco_t * hmac_ctx, uint32_t flags);

/*! @} */

/*****************************************************************************/

/*! @addtogroup sccops
    @{ */

/*!
 * Initialize a Symmetric Cipher Context Object.
 *
 * This function must be called before performing any other operation with the
 * Object.  This will set the @a mode and @a algorithm and initialize the
 * Object.
 *
 * @param sym_ctx   The context object to operate on.
 * @param algorithm The cipher algorithm this context will be used with.
 * @param mode      #FSL_SYM_MODE_CBC, #FSL_SYM_MODE_ECB, etc.
 *
 */
void fsl_shw_scco_init(fsl_shw_scco_t * sym_ctx,
		       fsl_shw_key_alg_t algorithm, fsl_shw_sym_mode_t mode);

/*!
 * Set the flags for a Symmetric Cipher Context.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param sym_ctx  The context object to operate on.
 * @param flags    The flags to reset (one or more values from
 *                 #fsl_shw_sym_ctx_flags_t ORed together).
 *
 */
void fsl_shw_scco_set_flags(fsl_shw_scco_t * sym_ctx, uint32_t flags);

/*!
 * Clear some flags in a Symmetric Cipher Context Object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param sym_ctx  The context object to operate on.
 * @param flags    The flags to reset (one or more values from
 *                 #fsl_shw_sym_ctx_flags_t ORed together).
 *
 */
void fsl_shw_scco_clear_flags(fsl_shw_scco_t * sym_ctx, uint32_t flags);

/*!
 * Set the Context (IV) for a Symmetric Cipher Context.
 *
 * This is to set the context/IV for #FSL_SYM_MODE_CBC mode, or to set the
 * context (the S-Box and pointers) for ARC4.  The full context size will
 * be copied.
 *
 * @param sym_ctx  The context object to operate on.
 * @param context  A pointer to the buffer which contains the context.
 *
 */
void fsl_shw_scco_set_context(fsl_shw_scco_t * sym_ctx, uint8_t * context);

/*!
 * Get the Context for a Symmetric Cipher Context.
 *
 * This is to retrieve the context/IV for #FSL_SYM_MODE_CBC mode, or to
 * retrieve context (the S-Box and pointers) for ARC4.  The full context
 * will be copied.
 *
 * @param      sym_ctx  The context object to operate on.
 * @param[out] context  Pointer to location where context will be stored.
 */
void fsl_shw_scco_get_context(const fsl_shw_scco_t * sym_ctx,
			      uint8_t * context);

/*!
 * Set the Counter Value for a Symmetric Cipher Context.
 *
 * This will set the Counter Value for CTR mode.
 *
 * @param sym_ctx  The context object to operate on.
 * @param counter  The starting counter value.  The number of octets.
 *                 copied will be the block size for the algorithm.
 * @param modulus  The modulus for controlling the incrementing of the counter.
 *
 */
void fsl_shw_scco_set_counter_info(fsl_shw_scco_t * sym_ctx,
				   const uint8_t * counter,
				   fsl_shw_ctr_mod_t modulus);

/*!
 * Get the Counter Value for a Symmetric Cipher Context.
 *
 * This will retrieve the Counter Value is for CTR mode.
 *
 * @param sym_ctx         The context object to query.
 * @param[out] counter    Pointer to location to store the current counter
 *                        value.  The number of octets copied will be the
 *                        block size for the algorithm.
 * @param[out] modulus    Pointer to location to store the modulus.
 *
 */
void fsl_shw_scco_get_counter_info(const fsl_shw_scco_t * sym_ctx,
				   uint8_t * counter,
				   fsl_shw_ctr_mod_t * modulus);

	  /*! @} *//* end sccops */

/*****************************************************************************/

/*! @addtogroup accoops
    @{ */

/*!
 * Initialize a Authentication-Cipher Context.
 *
 * @param auth_object  Pointer to object to operate on.
 * @param mode         The mode for this object (only #FSL_ACC_MODE_CCM
 *                     supported).
 */
void fsl_shw_acco_init(fsl_shw_acco_t * auth_object, fsl_shw_acc_mode_t mode);

/*!
 * Set the flags for a Authentication-Cipher Context.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param auth_object  Pointer to object to operate on.
 * @param flags        The flags to set (one or more from
 *                     #fsl_shw_auth_ctx_flags_t ORed together).
 *
 */
void fsl_shw_acco_set_flags(fsl_shw_acco_t * auth_object, uint32_t flags);

/*!
 * Clear some flags in a Authentication-Cipher Context Object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param auth_object  Pointer to object to operate on.
 * @param flags        The flags to reset (one or more from
 *                     #fsl_shw_auth_ctx_flags_t ORed together).
 *
 */
void fsl_shw_acco_clear_flags(fsl_shw_acco_t * auth_object, uint32_t flags);

/*!
 * Set up the Authentication-Cipher Object for CCM mode.
 *
 * This will set the @a auth_object for CCM mode and save the @a ctr,
 * and @a mac_length.  This function can be called instead of
 * #fsl_shw_acco_init().
 *
 * The parameter @a ctr is Counter Block 0, (counter value 0), which is for the
 * MAC.
 *
 * @param auth_object  Pointer to object to operate on.
 * @param algorithm    Cipher algorithm.  Only AES is supported.
 * @param ctr          The initial counter value.
 * @param mac_length   The number of octets used for the MAC.  Valid values are
 *                     4, 6, 8, 10, 12, 14, and 16.
 */
void fsl_shw_acco_set_ccm(fsl_shw_acco_t * auth_object,
			  fsl_shw_key_alg_t algorithm,
			  const uint8_t * ctr, uint8_t mac_length);

/*!
 * Format the First Block (IV) & Initial Counter Value per NIST CCM.
 *
 * This function will also set the IV and CTR values per Appendix A of NIST
 * Special Publication 800-38C (May 2004).  It will also perform the
 * #fsl_shw_acco_set_ccm() operation with information derived from this set of
 * parameters.
 *
 * Note this function assumes the algorithm is AES.  It initializes the
 * @a auth_object by setting the mode to #FSL_ACC_MODE_CCM and setting the
 * flags to be #FSL_ACCO_NIST_CCM.
 *
 * @param auth_object  Pointer to object to operate on.
 * @param t_length     The number of octets used for the MAC.  Valid values are
 *                     4, 6, 8, 10, 12, 14, and 16.
 * @param ad_length    Number of octets of Associated Data (may be zero).
 * @param q_length     A value for the size of the length of @a q field.  Valid
 *                     values are 1-8.
 * @param n            The Nonce (packet number or other changing value). Must
 *                     be (15 - @a q_length) octets long.
 * @param q            The value of Q (size of the payload in octets).
 *
 */
void fsl_shw_ccm_nist_format_ctr_and_iv(fsl_shw_acco_t * auth_object,
					uint8_t t_length,
					uint32_t ad_length,
					uint8_t q_length,
					const uint8_t * n, uint32_t q);

/*!
 * Update the First Block (IV) & Initial Counter Value per NIST CCM.
 *
 * This function will set the IV and CTR values per Appendix A of NIST Special
 * Publication 800-38C (May 2004).
 *
 * Note this function assumes that #fsl_shw_ccm_nist_format_ctr_and_iv() has
 * previously been called on the @a auth_object.
 *
 * @param auth_object  Pointer to object to operate on.
 * @param n             The Nonce (packet number or other changing value). Must
 *                      be (15 - @a q_length) octets long.
 * @param q             The value of Q (size of the payload in octets).
 *
 */
void fsl_shw_ccm_nist_update_ctr_and_iv(fsl_shw_acco_t * auth_object,
					const uint8_t * n, uint32_t q);

	 /* @} *//* accoops */

/******************************************************************************
 * Library functions
 *****************************************************************************/

/*! @addtogroup miscfuns
    @{ */

/* REQ-S2LRD-PINTFC-API-GEN-003 */
/*!
 * Determine the hardware security capabilities of this platform.
 *
 * Though a user context object is passed into this function, it will always
 * act in a non-blocking manner.
 *
 * @param  user_ctx   The user context which will be used for the query.
 *
 * @return  A pointer to the capabilities object.
 */
extern fsl_shw_pco_t *fsl_shw_get_capabilities(fsl_shw_uco_t * user_ctx);

/* REQ-S2LRD-PINTFC-API-GEN-004 */
/*!
 * Create an association between the user and the provider of the API.
 *
 * @param  user_ctx   The user context which will be used for this association.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_register_user(fsl_shw_uco_t * user_ctx);

/* REQ-S2LRD-PINTFC-API-GEN-005 */
/*!
 * Destroy the association between the user and the provider of the API.
 *
 * @param  user_ctx   The user context which is no longer needed.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_deregister_user(fsl_shw_uco_t * user_ctx);

/* REQ-S2LRD-PINTFC-API-GEN-006 */
/*!
 * Retrieve results from earlier operations.
 *
 * @param         user_ctx     The user's context.
 * @param         result_size  The number of array elements of @a results.
 * @param[in,out] results      Pointer to first of the (array of) locations to
 *                             store results.
 * @param[out]    result_count Pointer to store the number of results which
 *                             were returned.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_get_results(fsl_shw_uco_t * user_ctx,
					    uint16_t result_size,
					    fsl_shw_result_t results[],
					    uint16_t * result_count);

/*!
 * Allocate a block of secure memory
 *
 * @param       user_ctx        User context
 * @param       size            Memory size (octets).  Note: currently only
 *                              supports only single-partition sized blocks.
 * @param       UMID            User Mode ID to use when registering the
 *                              partition.
 * @param       permissions     Permissions to initialize the partition with.
 *                              Can be made by ORing flags from the
 *                              #fsl_shw_permission_t.
 *
 * @return                      Address of the allocated memory.  NULL if the
 *                              call was not successful.
 */
extern void *fsl_shw_smalloc(fsl_shw_uco_t * user_ctx,
			     uint32_t size,
			     const uint8_t * UMID, uint32_t permissions);

/*!
 * Free a block of secure memory that was allocated with #fsl_shw_smalloc
 *
 * @param       user_ctx        User context
 * @param       address         Address of the block of secure memory to be
 *                              released.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_sfree(fsl_shw_uco_t * user_ctx, void *address);

/*!
 * Diminish the permissions of a block of secure memory.  Note that permissions
 * can only be revoked.
 *
 * @param       user_ctx        User context
 * @param       address         Base address of the secure memory to work with
 * @param       permissions     Permissions to initialize the partition with.
 *                              Can be made by ORing flags from the
 *                              #fsl_shw_permission_t.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_diminish_perms(fsl_shw_uco_t * user_ctx,
					       void *address,
					       uint32_t permissions);

/*!
 * @brief   Encrypt a region of secure memory using the hardware secret key
 *
 * @param       user_ctx        User context
 * @param       partition_base  Base address of the partition
 * @param       offset_bytes    Offset of data from the partition base
 * @param       byte_count      Length of the data to encrypt
 * @param       black_data      Location to store the encrypted data
 * @param       IV              IV to use for the encryption routine
 * @param       cypher_mode     Cyphering mode to use, specified by type
 *                              #fsl_shw_cypher_mode_t
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t
do_scc_encrypt_region(fsl_shw_uco_t * user_ctx,
		      void *partition_base, uint32_t offset_bytes,
		      uint32_t byte_count, uint8_t * black_data,
		      uint32_t * IV, fsl_shw_cypher_mode_t cypher_mode);

/*!
 * @brief   Decrypt a region of secure memory using the hardware secret key
 *
 * @param       user_ctx        User context
 * @param       partition_base  Base address of the partition
 * @param       offset_bytes    Offset of data from the partition base
 * @param       byte_count      Length of the data to encrypt
 * @param       black_data      Location to store the encrypted data
 * @param       IV              IV to use for the encryption routine
 * @param       cypher_mode     Cyphering mode to use, specified by type
 *                              #fsl_shw_cypher_mode_t
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t
do_scc_decrypt_region(fsl_shw_uco_t * user_ctx,
		      void *partition_base, uint32_t offset_bytes,
		      uint32_t byte_count, const uint8_t * black_data,
		      uint32_t * IV, fsl_shw_cypher_mode_t cypher_mode);

	  /*! @} *//* miscfuns */

/*! @addtogroup opfuns
    @{ */

/* REQ-S2LRD-PINTFC-API-BASIC-SYM-002 */
/* PINTFC-API-BASIC-SYM-ARC4-001 */
/* PINTFC-API-BASIC-SYM-ARC4-002 */
/*!
 * Encrypt a stream of data with a symmetric-key algorithm.
 *
 * In ARC4, and also in #FSL_SYM_MODE_CBC and #FSL_SYM_MODE_CTR modes, the
 * flags of the @a sym_ctx object will control part of the operation of this
 * function.  The #FSL_SYM_CTX_INIT flag means that there is no context info in
 * the object.  The #FSL_SYM_CTX_LOAD means to use information in the
 * @a sym_ctx at the start of the operation, and the #FSL_SYM_CTX_SAVE flag
 * means to update the object's context information after the operation has
 * been performed.
 *
 * All of the data for an operation can be run through at once using the
 * #FSL_SYM_CTX_INIT or #FSL_SYM_CTX_LOAD flags, as appropriate, and then using
 * a @a length for the whole of the data.
 *
 * If a #FSL_SYM_CTX_SAVE flag were added, an additional call to the function
 * would "pick up" where the previous call left off, allowing the user to
 * perform the larger function in smaller steps.
 *
 * In #FSL_SYM_MODE_CBC and #FSL_SYM_MODE_ECB modes, the @a length must always
 * be a multiple of the block size for the algorithm being used.  For proper
 * operation in #FSL_SYM_MODE_CTR mode, the @a length must be a multiple of the
 * block size until the last operation on the total octet stream.
 *
 * Some users of ARC4 may want to compute the context (S-Box and pointers) from
 * the key before any data is available.  This may be done by running this
 * function with a @a length of zero, with the init & save flags flags on in
 * the @a sym_ctx.  Subsequent operations would then run as normal with the
 * load and save flags.  Note that they key object is still required.
 *
 * @param         user_ctx  A user context from #fsl_shw_register_user().
 * @param         key_info  Key and algorithm  being used for this operation.
 * @param[in,out] sym_ctx   Info on cipher mode, state of the cipher.
 * @param         length   Length, in octets, of the pt (and ct).
 * @param         pt       pointer to plaintext to be encrypted.
 * @param[out]    ct       pointer to where to store the resulting ciphertext.
 *
 * @return    A return code of type #fsl_shw_return_t.
 *
 */
extern fsl_shw_return_t fsl_shw_symmetric_encrypt(fsl_shw_uco_t * user_ctx,
						  fsl_shw_sko_t * key_info,
						  fsl_shw_scco_t * sym_ctx,
						  uint32_t length,
						  const uint8_t * pt,
						  uint8_t * ct);

/* PINTFC-API-BASIC-SYM-002 */
/* PINTFC-API-BASIC-SYM-ARC4-001 */
/* PINTFC-API-BASIC-SYM-ARC4-002 */
/*!
 * Decrypt a stream of data with a symmetric-key algorithm.
 *
 * In ARC4, and also in #FSL_SYM_MODE_CBC and #FSL_SYM_MODE_CTR modes, the
 * flags of the @a sym_ctx object will control part of the operation of this
 * function.  The #FSL_SYM_CTX_INIT flag means that there is no context info in
 * the object.  The #FSL_SYM_CTX_LOAD means to use information in the
 * @a sym_ctx at the start of the operation, and the #FSL_SYM_CTX_SAVE flag
 * means to update the object's context information after the operation has
 * been performed.
 *
 * All of the data for an operation can be run through at once using the
 * #FSL_SYM_CTX_INIT or #FSL_SYM_CTX_LOAD flags, as appropriate, and then using
 * a @a length for the whole of the data.
 *
 * If a #FSL_SYM_CTX_SAVE flag were added, an additional call to the function
 * would "pick up" where the previous call left off, allowing the user to
 * perform the larger function in smaller steps.
 *
 * In #FSL_SYM_MODE_CBC and #FSL_SYM_MODE_ECB modes, the @a length must always
 * be a multiple of the block size for the algorithm being used.  For proper
 * operation in #FSL_SYM_MODE_CTR mode, the @a length must be a multiple of the
 * block size until the last operation on the total octet stream.
 *
 * Some users of ARC4 may want to compute the context (S-Box and pointers) from
 * the key before any data is available.  This may be done by running this
 * function with a @a length of zero, with the #FSL_SYM_CTX_INIT &
 * #FSL_SYM_CTX_SAVE flags on in the @a sym_ctx.  Subsequent operations would
 * then run as normal with the load & save flags.  Note that they key object is
 * still required.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param      key_info The key and algorithm being used in this operation.
 * @param[in,out] sym_ctx Info on cipher mode, state of the cipher.
 * @param      length   Length, in octets, of the ct (and pt).
 * @param      ct       pointer to ciphertext to be decrypted.
 * @param[out] pt       pointer to where to store the resulting plaintext.
 *
 * @return    A return code of type #fsl_shw_return_t
 *
 */
extern fsl_shw_return_t fsl_shw_symmetric_decrypt(fsl_shw_uco_t * user_ctx,
						  fsl_shw_sko_t * key_info,
						  fsl_shw_scco_t * sym_ctx,
						  uint32_t length,
						  const uint8_t * ct,
						  uint8_t * pt);

/* REQ-S2LRD-PINTFC-API-BASIC-HASH-005 */
/*!
 * Hash a stream of data with a cryptographic hash algorithm.
 *
 * The flags in the @a hash_ctx control the operation of this function.
 *
 * Hashing functions work on 64 octets of message at a time.  Therefore, when
 * any partial hashing of a long message is performed, the message @a length of
 * each segment must be a multiple of 64.  When ready to
 * #FSL_HASH_FLAGS_FINALIZE the hash, the @a length may be any value.
 *
 * With the #FSL_HASH_FLAGS_INIT and #FSL_HASH_FLAGS_FINALIZE flags on, a
 * one-shot complete hash, including padding, will be performed.  The @a length
 * may be any value.
 *
 * The first octets of a data stream can be hashed by setting the
 * #FSL_HASH_FLAGS_INIT and #FSL_HASH_FLAGS_SAVE flags.  The @a length must be
 * a multiple of 64.
 *
 * The flag #FSL_HASH_FLAGS_LOAD is used to load a context previously saved by
 * #FSL_HASH_FLAGS_SAVE.  The two in combination will allow a (multiple-of-64
 * octets) 'middle sequence' of the data stream to be hashed with the
 * beginning.  The @a length must again be a multiple of 64.
 *
 * Since the flag #FSL_HASH_FLAGS_LOAD is used to load a context previously
 * saved by #FSL_HASH_FLAGS_SAVE, the #FSL_HASH_FLAGS_LOAD and
 * #FSL_HASH_FLAGS_FINALIZE flags, used together, can be used to finish the
 * stream.  The @a length may be any value.
 *
 * If the user program wants to do the padding for the hash, it can leave off
 * the #FSL_HASH_FLAGS_FINALIZE flag.  The @a length must then be a multiple of
 * 64 octets.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param[in,out] hash_ctx Hashing algorithm and state of the cipher.
 * @param      msg       Pointer to the data to be hashed.
 * @param      length    Length, in octets, of the @a msg.
 * @param[out] result    If not null, pointer to where to store the hash
 *                       digest.
 * @param      result_len Number of octets to store in @a result.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_hash(fsl_shw_uco_t * user_ctx,
				     fsl_shw_hco_t * hash_ctx,
				     const uint8_t * msg,
				     uint32_t length,
				     uint8_t * result, uint32_t result_len);

/* REQ-S2LRD-PINTFC-API-BASIC-HMAC-001 */
/*!
 * Precompute the Key hashes for an HMAC operation.
 *
 * This function may be used to calculate the inner and outer precomputes,
 * which are the hash contexts resulting from hashing the XORed key for the
 * 'inner hash' and the 'outer hash', respectively, of the HMAC function.
 *
 * After execution of this function, the @a hmac_ctx will contain the
 * precomputed inner and outer contexts, so that they may be used by
 * #fsl_shw_hmac().  The flags of @a hmac_ctx will be updated with
 * #FSL_HMAC_FLAGS_PRECOMPUTES_PRESENT to mark their presence.  In addition, the
 * #FSL_HMAC_FLAGS_INIT flag will be set.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param      key_info  The key being used in this operation.  Key must be
 *                       1 to 64 octets long.
 * @param[in,out] hmac_ctx The context which controls, by its flags and
 *                         algorithm, the operation of this function.
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_hmac_precompute(fsl_shw_uco_t * user_ctx,
						fsl_shw_sko_t * key_info,
						fsl_shw_hmco_t * hmac_ctx);

/* REQ-S2LRD-PINTFC-API-BASIC-HMAC-002 */
/*!
 * Continue, finalize, or one-shot an HMAC operation.
 *
 * There are a number of ways to use this function.  The flags in the
 * @a hmac_ctx object will determine what operations occur.
 *
 * If #FSL_HMAC_FLAGS_INIT is set, then the hash will be started either from
 * the @a key_info, or from the precomputed inner hash value in the
 * @a hmac_ctx, depending on the value of #FSL_HMAC_FLAGS_PRECOMPUTES_PRESENT.
 *
 * If, instead, #FSL_HMAC_FLAGS_LOAD is set, then the hash will be continued
 * from the ongoing inner hash computation in the @a hmac_ctx.
 *
 * If #FSL_HMAC_FLAGS_FINALIZE are set, then the @a msg will be padded, hashed,
 * the outer hash will be performed, and the @a result will be generated.
 *
 * If the #FSL_HMAC_FLAGS_SAVE flag is set, then the (ongoing or final) digest
 * value will be stored in the ongoing inner hash computation field of the @a
 * hmac_ctx.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param key_info       If #FSL_HMAC_FLAGS_INIT is set in the @a hmac_ctx,
 *                       this is the key being used in this operation, and the
 *                       IPAD.  If #FSL_HMAC_FLAGS_INIT is set in the @a
 *                       hmac_ctx and @a key_info is NULL, then
 *                       #fsl_shw_hmac_precompute() has been used to populate
 *                       the @a inner_precompute and @a outer_precompute
 *                       contexts.  If #FSL_HMAC_FLAGS_INIT is not set, this
 *                       parameter is ignored.

 * @param[in,out] hmac_ctx The context which controls, by its flags and
 *                       algorithm, the operation of this function.
 * @param      msg               Pointer to the message to be hashed.
 * @param      length            Length, in octets, of the @a msg.
 * @param[out] result            Pointer, of @a result_len octets, to where to
 *                               store the HMAC.
 * @param      result_len        Length of @a result buffer.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_hmac(fsl_shw_uco_t * user_ctx,
				     fsl_shw_sko_t * key_info,
				     fsl_shw_hmco_t * hmac_ctx,
				     const uint8_t * msg,
				     uint32_t length,
				     uint8_t * result, uint32_t result_len);

/* REQ-S2LRD-PINTFC-API-BASIC-RNG-002 */
/*!
 * Get random data.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param      length    The number of octets of @a data being requested.
 * @param[out] data      A pointer to a location of @a length octets to where
 *                       random data will be returned.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_get_random(fsl_shw_uco_t * user_ctx,
					   uint32_t length, uint8_t * data);

/* REQ-S2LRD-PINTFC-API-BASIC-RNG-002 */
/*!
 * Add entropy to random number generator.
 *
 * @param      user_ctx  A user context from #fsl_shw_register_user().
 * @param      length    Number of bytes at @a data.
 * @param      data      Entropy to add to random number generator.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_add_entropy(fsl_shw_uco_t * user_ctx,
					    uint32_t length, uint8_t * data);

/*!
 * Perform Generation-Encryption by doing a Cipher and a Hash.
 *
 * Generate the authentication value @a auth_value as well as encrypt the @a
 * payload into @a ct (the ciphertext).  This is a one-shot function, so all of
 * the @a auth_data and the total message @a payload must passed in one call.
 * This also means that the flags in the @a auth_ctx must be #FSL_ACCO_CTX_INIT
 * and #FSL_ACCO_CTX_FINALIZE.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      auth_ctx         Controlling object for Authenticate-decrypt.
 * @param      cipher_key_info  The key being used for the cipher part of this
 *                              operation.  In CCM mode, this key is used for
 *                              both parts.
 * @param      auth_key_info    The key being used for the authentication part
 *                              of this operation.  In CCM mode, this key is
 *                              ignored and may be NULL.
 * @param      auth_data_length Length, in octets, of @a auth_data.
 * @param      auth_data        Data to be authenticated but not encrypted.
 * @param      payload_length   Length, in octets, of @a payload.
 * @param      payload          Pointer to the plaintext to be encrypted.
 * @param[out] ct               Pointer to the where the encrypted @a payload
 *                              will be stored.  Must be @a payload_length
 *                              octets long.
 * @param[out] auth_value       Pointer to where the generated authentication
 *                              field will be stored. Must be as many octets as
 *                              indicated by MAC length in the @a function_ctx.
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_gen_encrypt(fsl_shw_uco_t * user_ctx,
					    fsl_shw_acco_t * auth_ctx,
					    fsl_shw_sko_t * cipher_key_info,
					    fsl_shw_sko_t * auth_key_info,
					    uint32_t auth_data_length,
					    const uint8_t * auth_data,
					    uint32_t payload_length,
					    const uint8_t * payload,
					    uint8_t * ct, uint8_t * auth_value);

/*!
 * Perform Authentication-Decryption in Cipher + Hash.
 *
 * This function will perform a one-shot decryption of a data stream as well as
 * authenticate the authentication value.  This is a one-shot function, so all
 * of the @a auth_data and the total message @a payload must passed in one
 * call.  This also means that the flags in the @a auth_ctx must be
 * #FSL_ACCO_CTX_INIT and #FSL_ACCO_CTX_FINALIZE.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      auth_ctx         Controlling object for Authenticate-decrypt.
 * @param      cipher_key_info  The key being used for the cipher part of this
 *                              operation.  In CCM mode, this key is used for
 *                              both parts.
 * @param      auth_key_info    The key being used for the authentication part
 *                              of this operation.  In CCM mode, this key is
 *                              ignored and may be NULL.
 * @param      auth_data_length Length, in octets, of @a auth_data.
 * @param      auth_data        Data to be authenticated but not decrypted.
 * @param      payload_length   Length, in octets, of @a ct and @a pt.
 * @param      ct               Pointer to the encrypted input stream.
 * @param      auth_value       The (encrypted) authentication value which will
 *                              be authenticated.  This is the same data as the
 *                              (output) @a auth_value argument to
 *                              #fsl_shw_gen_encrypt().
 * @param[out] payload          Pointer to where the plaintext resulting from
 *                              the decryption will be stored.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_auth_decrypt(fsl_shw_uco_t * user_ctx,
					     fsl_shw_acco_t * auth_ctx,
					     fsl_shw_sko_t * cipher_key_info,
					     fsl_shw_sko_t * auth_key_info,
					     uint32_t auth_data_length,
					     const uint8_t * auth_data,
					     uint32_t payload_length,
					     const uint8_t * ct,
					     const uint8_t * auth_value,
					     uint8_t * payload);

/*!
 * Establish the key in a protected location, which can be the system keystore,
 * user keystore, or (on platforms that support it) as a Platform Key.
 *
 * By default, keys initialized with #fsl_shw_sko_init() will be placed into
 * the system keystore.  The user can cause the key to be established in a
 * user keystore by first calling #fsl_shw_sko_set_keystore() on the key.
 * Normally, keys in the system keystore can only be used for hardware
 * encrypt or decrypt operations, however if the #FSL_SKO_KEY_SW_KEY flag is
 * applied using #fsl_shw_sko_set_flags(), the key will be established as a
 * software key, which can then be read out using #fsl_shw_read_key().
 *
 * Keys initialized with #fsl_shw_sko_init_pf_key() are established as a
 * Platform Key.  Their use is covered in @ref di_sec.
 *
 * This function only needs to be used when unwrapping a key, setting up a key
 * which could be wrapped with a later call to #fsl_shw_extract_key(), or
 * setting up a key as a Platform Key.  Normal cleartext keys can simply be
 * placed into #fsl_shw_sko_t key objects with #fsl_shw_sko_set_key() and used
 * directly.
 *
 * The maximum key size supported for wrapped/unwrapped keys is 32 octets.
 * (This is the maximum reasonable key length on Sahara - 32 octets for an HMAC
 * key based on SHA-256.)  The key size is determined by the @a key_info.  The
 * expected length of @a key can be determined by
 * #fsl_shw_sko_calculate_wrapped_size()
 *
 * The protected key will not be available for use until this operation
 * successfully completes.
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
 */
extern fsl_shw_return_t fsl_shw_establish_key(fsl_shw_uco_t * user_ctx,
					      fsl_shw_sko_t * key_info,
					      fsl_shw_key_wrap_t establish_type,
					      const uint8_t * key);

/*!
 * Read the key value from a key object.
 *
 * Only a key marked as a software key (#FSL_SKO_KEY_SW_KEY) can be read with
 * this call.  It has no effect on the status of the key store.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      key_info         The referenced key.
 * @param[out] key              The location to store the key value.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_read_key(fsl_shw_uco_t * user_ctx,
					 fsl_shw_sko_t * key_info,
					 uint8_t * key);

/*!
 * Wrap a key and retrieve the wrapped value.
 *
 * A wrapped key is a key that has been cryptographically obscured.  It is
 * only able to be used with keys that have been established by 
 * #fsl_shw_establish_key().
 *
 * For keys established in the system or user keystore, this function will
 * also release the key (see #fsl_shw_release_key()) so that it must be re-
 * established before reuse.  This function will not release keys that are
 * established as a Platform Key, so a call to #fsl_shw_release_key() is
 * necessary to release those keys.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      key_info         The information about the key to be deleted.
 * @param[out] covered_key      The location to store the wrapped key.
 *                              (This size is based upon the maximum key size
 *                              of 32 octets).
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_extract_key(fsl_shw_uco_t * user_ctx,
					    fsl_shw_sko_t * key_info,
					    uint8_t * covered_key);

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
extern fsl_shw_return_t fsl_shw_release_key(fsl_shw_uco_t * user_ctx,
					    fsl_shw_sko_t * key_info);

/*!
 * Cause the hardware to create a new random key for use by the secure memory
 * encryption hardware.
 *
 * Have the hardware use the secure hardware random number generator to load a
 * new secret key into the system's Random Key register.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
extern fsl_shw_return_t fsl_shw_gen_random_pf_key(fsl_shw_uco_t * user_ctx);

/*!
 * Retrieve the detected tamper event.
 *
 * Note that if more than one event was detected, this routine will only ever
 * return one of them.
 *
 * @param[in]  user_ctx         A user context from #fsl_shw_register_user().
 * @param[out] tamperp          Location to store the tamper information.
 * @param[out] timestampp       Locate to store timestamp from hardwhare when
 *                              an event was detected.
 *
 *
 * @return    A return code of type #fsl_shw_return_t (for instance, if the platform
 *            is not in a fail state.
 */
extern fsl_shw_return_t fsl_shw_read_tamper_event(fsl_shw_uco_t * user_ctx,
						  fsl_shw_tamper_t * tamperp,
						  uint64_t * timestampp);

/*! @} *//* opfuns */

/* Insert example code into the API documentation. */

/*!
 * @example apitest.c
 */

/*!
 * @example sym.c
 */

/*!
 * @example rand.c
 */

/*!
 * @example hash.c
 */

/*!
 * @example hmac1.c
 */

/*!
 * @example hmac2.c
 */

/*!
 * @example gen_encrypt.c
 */

/*!
 * @example auth_decrypt.c
 */

/*!
 * @example wrapped_key.c
 */

/*!
 * @example smalloc.c
 */

/*!
 * @example user_keystore.c
 */

/*!
 * @example dryice.c
 */

#endif				/* API_DOC */

#endif				/* FSL_SHW_H */
