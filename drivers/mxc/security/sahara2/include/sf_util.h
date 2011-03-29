/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sf_util.h
*
* @brief Header for Sahara Descriptor-chain building Functions
*/
#ifndef SF_UTIL_H
#define SF_UTIL_H

#include <fsl_platform.h>
#include <sahara.h>

/*! Header value for Sahara Descriptor  1 */
#define SAH_HDR_SKHA_SET_MODE_IV_KEY  0x10880000
/*! Header value for Sahara Descriptor  2 */
#define SAH_HDR_SKHA_SET_MODE_ENC_DEC 0x108D0000
/*! Header value for Sahara Descriptor  4 */
#define SAH_HDR_SKHA_ENC_DEC          0x90850000
/*! Header value for Sahara Descriptor  5 */
#define SAH_HDR_SKHA_READ_CONTEXT_IV  0x10820000
/*! Header value for Sahara Descriptor  6 */
#define SAH_HDR_MDHA_SET_MODE_MD_KEY  0x20880000
/*! Header value for Sahara Descriptor  8 */
#define SAH_HDR_MDHA_SET_MODE_HASH    0x208D0000
/*! Header value for Sahara Descriptor  10 */
#define SAH_HDR_MDHA_HASH             0xA0850000
/*! Header value for Sahara Descriptor  11 */
#define SAH_HDR_MDHA_STORE_DIGEST     0x20820000
/*! Header value for Sahara Descriptor  18 */
#define SAH_HDR_RNG_GENERATE          0x308C0000
/*! Header value for Sahara Descriptor 19 */
#define SAH_HDR_PKHA_LD_N_E           0xC0800000
/*! Header value for Sahara Descriptor 20 */
#define SAH_HDR_PKHA_LD_A_EX_ST_B     0x408D0000
/*! Header value for Sahara Descriptor 21 */
#define SAH_HDR_PKHA_LD_N_EX_ST_B     0x408E0000
/*! Header value for Sahara Descriptor 22 */
#define SAH_HDR_PKHA_LD_A_B           0xC0830000
/*! Header value for Sahara Descriptor 23 */
#define SAH_HDR_PKHA_LD_A0_A1         0x40840000
/*! Header value for Sahara Descriptor 24 */
#define SAH_HDR_PKHA_LD_A2_A3         0xC0850000
/*! Header value for Sahara Descriptor 25 */
#define SAH_HDR_PKHA_LD_B0_B1         0xC0860000
/*! Header value for Sahara Descriptor 26 */
#define SAH_HDR_PKHA_LD_B2_B3         0x40870000
/*! Header value for Sahara Descriptor 27 */
#define SAH_HDR_PKHA_ST_A_B           0x40820000
/*! Header value for Sahara Descriptor 28 */
#define SAH_HDR_PKHA_ST_A0_A1         0x40880000
/*! Header value for Sahara Descriptor 29 */
#define SAH_HDR_PKHA_ST_A2_A3         0xC0890000
/*! Header value for Sahara Descriptor 30 */
#define SAH_HDR_PKHA_ST_B0_B1         0xC08A0000
/*! Header value for Sahara Descriptor 31 */
#define SAH_HDR_PKHA_ST_B2_B3         0x408B0000
/*! Header value for Sahara Descriptor 32 */
#define SAH_HDR_PKHA_EX_ST_B1         0xC08C0000
/*! Header value for Sahara Descriptor 33 */
#define SAH_HDR_ARC4_SET_MODE_SBOX    0x90890000
/*! Header value for Sahara Descriptor 34 */
#define SAH_HDR_ARC4_READ_SBOX        0x90860000
/*! Header value for Sahara Descriptor 35 */
#define SAH_HDR_ARC4_SET_MODE_KEY     0x90830000
/*! Header value for Sahara Descriptor 36 */
#define SAH_HDR_PKHA_LD_A3_B0         0x40810000
/*! Header value for Sahara Descriptor 37 */
#define SAH_HDR_PKHA_ST_B1_B2         0xC08F0000
/*! Header value for Sahara Descriptor 38 */
#define SAH_HDR_SKHA_CBC_ICV          0x10840000
/*! Header value for Sahara Descriptor 39 */
#define SAH_HDR_MDHA_ICV_CHECK        0xA08A0000

/*! Header bit indicating "Link-List optimization" */
#define SAH_HDR_LLO                   0x01000000

#define SAH_SF_DCLS                                                         \
    fsl_shw_return_t ret;                                                   \
    unsigned sf_executed = 0;                                               \
    sah_Head_Desc* desc_chain = NULL;                                       \
    uint32_t header

#define SAH_SF_USER_CHECK()                                                 \
do {                                                                        \
    ret = sah_validate_uco(user_ctx);                                       \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
} while (0)

#define SAH_SF_EXECUTE()                                                    \
do {                                                                        \
    sf_executed = 1;                                                        \
    ret = sah_Descriptor_Chain_Execute(desc_chain, user_ctx);               \
} while (0)

#define SAH_SF_DESC_CLEAN()                                                 \
do {                                                                        \
    if (!sf_executed || (user_ctx->flags & FSL_UCO_BLOCKING_MODE)) {        \
        sah_Descriptor_Chain_Destroy(user_ctx->mem_util, &desc_chain);      \
    }                                                                       \
    (void) header;                                                          \
} while (0)

/*! Add Descriptor with two inputs */
#define DESC_IN_IN(hdr, len1, ptr1, len2, ptr2)                             \
{                                                                           \
    ret = sah_add_two_in_desc(hdr, ptr1, len1, ptr2, len2,                  \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}

/*! Add Descriptor with two vectors */
#define DESC_D_D(hdr, len1, ptr1, len2, ptr2)                               \
{                                                                           \
    ret = sah_add_two_d_desc(hdr, ptr1, len1, ptr2, len2,                   \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}

/*! Add Descriptor with input and a key */
#define DESC_IN_KEY(hdr, len1, ptr1, key2)                                  \
{                                                                           \
    ret = sah_add_in_key_desc(hdr, ptr1, len1, key2,                        \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}

/*! Add Descriptor with input and an output */
#define DESC_IN_OUT(hdr, len1, ptr1, len2, ptr2)                            \
{                                                                           \
    ret = sah_add_in_out_desc(hdr, ptr1, len1, ptr2, len2,                  \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}

/*! Add Descriptor with input and a key output */
#define DESC_IN_KEYOUT(hdr, len1, ptr1, key2)                               \
{                                                                           \
    ret = sah_add_in_keyout_desc(hdr, ptr1, len1, key2,                     \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}

/*! Add Descriptor with a key and an output */
#define DESC_KEY_OUT(hdr, key1, len2, ptr2)                                 \
{                                                                           \
    ret = sah_add_key_out_desc(hdr, key1, ptr2, len2,                       \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}

/*! Add Descriptor with two outputs */
#define DESC_OUT_OUT(hdr, len1, ptr1, len2, ptr2)                           \
{                                                                           \
    ret = sah_add_two_out_desc(hdr, ptr1, len1, ptr2, len2,                 \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}

/*! Add Descriptor with output then input pointers */
#define DESC_OUT_IN(hdr, len1, ptr1, len2, ptr2)                            \
{                                                                           \
    ret = sah_add_out_in_desc(hdr, ptr1, len1, ptr2, len2,                  \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}

#ifdef SAH_SF_DEBUG
/*! Add Descriptor with two outputs */
#define DBG_DESC(hdr, len1, ptr1, len2, ptr2)                               \
{                                                                           \
    ret = sah_add_two_out_desc(hdr, ptr1, len1, ptr2, len2,                 \
                              user_ctx->mem_util, &desc_chain);             \
    if (ret != FSL_RETURN_OK_S) {                                           \
        goto out;                                                           \
    }                                                                       \
}
#else
#define DBG_DESC(hdr, len1, ptr1, len2, ptr2)
#endif

#ifdef __KERNEL__
#define DESC_DBG_ON  ({console_loglevel = 8;})
#define DESC_DBG_OFF ({console_loglevel = 7;})
#else
#define DESC_DBG_ON  system("echo 8 > /proc/sys/kernel/printk")
#define DESC_DBG_OFF system("echo 7 > /proc/sys/kernel/printk")
#endif

#define DESC_TEMP_ALLOC(size)                                               \
({                                                                          \
    uint8_t* ptr;                                                           \
    ptr = user_ctx->mem_util->mu_malloc(user_ctx->mem_util->mu_ref,         \
                                                   size);                   \
    if (ptr == NULL) {                                                      \
        ret = FSL_RETURN_NO_RESOURCE_S;                                     \
        goto out;                                                           \
    }                                                                       \
                                                                            \
    ptr;                                                                    \
})

#define DESC_TEMP_FREE(ptr)                                                 \
({                                                                          \
    if ((ptr != NULL) &&                                                    \
        (!sf_executed || (user_ctx->flags & FSL_UCO_BLOCKING_MODE))) {      \
            user_ctx->mem_util->                                            \
                mu_free(user_ctx->mem_util->mu_ref, ptr);                   \
            ptr = NULL;                                                     \
    }                                                                       \
})

/* Temporary implementation.  This needs to be in internal/secure RAM */
#define DESC_TEMP_SECURE_ALLOC(size)                                        \
({                                                                          \
    uint8_t* ptr;                                                           \
    ptr = user_ctx->mem_util->mu_malloc(user_ctx->mem_util->mu_ref,         \
                                                   size);                   \
    if (ptr == NULL) {                                                      \
        ret = FSL_RETURN_NO_RESOURCE_S;                                     \
        goto out;                                                           \
    }                                                                       \
                                                                            \
    ptr;                                                                    \
})

#define DESC_TEMP_SECURE_FREE(ptr, size)                                    \
({                                                                          \
    if ((ptr != NULL) &&                                                    \
        (!sf_executed || (user_ctx->flags & FSL_UCO_BLOCKING_MODE))) {      \
            user_ctx->mem_util->mu_memset(user_ctx->mem_util->mu_ref,       \
                                          ptr, 0, size);                    \
                                                                            \
            user_ctx->mem_util->                                            \
                mu_free(user_ctx->mem_util->mu_ref, ptr);                   \
            ptr = NULL;                                                     \
    }                                                                       \
})

extern const uint32_t sah_insert_mdha_algorithm[];

/*! @defgroup mdhaflags MDHA Mode Register Values
 *
 * These are bit fields and combinations of bit fields for setting the Mode
 * Register portion of a Sahara Descriptor Header field.
 *
 * The parity bit has been set to ensure that these values have even parity,
 * therefore using an Exclusive-OR operation against an existing header will
 * preserve its parity.
 *
 * @addtogroup mdhaflags
   @{
 */
#define sah_insert_mdha_icv_check 0x80001000
#define sah_insert_mdha_ssl       0x80000400
#define sah_insert_mdha_mac_full  0x80000200
#define sah_insert_mdha_opad      0x80000080
#define sah_insert_mdha_ipad      0x80000040
#define sah_insert_mdha_init      0x80000020
#define sah_insert_mdha_hmac      0x80000008
#define sah_insert_mdha_pdata     0x80000004
#define sah_insert_mdha_algorithm_sha224  0x00000003
#define sah_insert_mdha_algorithm_sha256  0x80000002
#define sah_insert_mdha_algorithm_md5     0x80000001
#define sah_insert_mdha_algorithm_sha1    0x00000000
/*! @} */

extern const uint32_t sah_insert_skha_algorithm[];
extern const uint32_t sah_insert_skha_mode[];
extern const uint32_t sah_insert_skha_modulus[];

/*! @defgroup skhaflags SKHA Mode Register Values
 *
 * These are bit fields and combinations of bit fields for setting the Mode
 * Register portion of a Sahara Descriptor Header field.
 *
 * The parity bit has been set to ensure that these values have even parity,
 * therefore using an Exclusive-OR operation against an existing header will
 * preserve its parity.
 *
 * @addtogroup skhaflags
   @{
 */
/*! */
#define sah_insert_skha_modulus_128    0x00001e00
#define sah_insert_skha_no_key_parity  0x80000100
#define sah_insert_skha_ctr_last_block 0x80000020
#define sah_insert_skha_suppress_cbc   0x80000020
#define sah_insert_skha_no_permute     0x80000020
#define sah_insert_skha_algorithm_arc4 0x00000003
#define sah_insert_skha_algorithm_tdes 0x80000002
#define sah_insert_skha_algorithm_des  0x80000001
#define sah_insert_skha_algorithm_aes  0x00000000
#define sah_insert_skha_aux0           0x80000020
#define sah_insert_skha_mode_ctr       0x00000018
#define sah_insert_skha_mode_ccm       0x80000010
#define sah_insert_skha_mode_cbc       0x80000008
#define sah_insert_skha_mode_ecb       0x00000000
#define sah_insert_skha_encrypt        0x80000004
#define sah_insert_skha_decrypt        0x00000000
/*! @} */

/*! @defgroup rngflags RNG Mode Register Values
 *
 */
/*! */
#define sah_insert_rng_gen_seed        0x80000001

/*! @} */

/*! @defgroup pkhaflags PKHA Mode Register Values
 *
 */
/*! */
#define sah_insert_pkha_soft_err_false         0x80000200
#define sah_insert_pkha_soft_err_true          0x80000100

#define sah_insert_pkha_rtn_clr_mem            0x80000001
#define sah_insert_pkha_rtn_clr_eram           0x80000002
#define sah_insert_pkha_rtn_mod_exp            0x00000003
#define sah_insert_pkha_rtn_mod_r2modn         0x80000004
#define sah_insert_pkha_rtn_mod_rrmodp         0x00000005
#define sah_insert_pkha_rtn_ec_fp_aff_ptmul    0x00000006
#define sah_insert_pkha_rtn_ec_f2m_aff_ptmul   0x80000007
#define sah_insert_pkha_rtn_ec_fp_proj_ptmul   0x80000008
#define sah_insert_pkha_rtn_ec_f2m_proj_ptmul  0x00000009
#define sah_insert_pkha_rtn_ec_fp_add          0x0000000A
#define sah_insert_pkha_rtn_ec_fp_double       0x8000000B
#define sah_insert_pkha_rtn_ec_f2m_add         0x0000000C
#define sah_insert_pkha_rtn_ec_f2m_double      0x8000000D
#define sah_insert_pkha_rtn_f2m_r2modn         0x8000000E
#define sah_insert_pkha_rtn_f2m_inv            0x0000000F
#define sah_insert_pkha_rtn_mod_inv            0x80000010
#define sah_insert_pkha_rtn_rsa_sstep          0x00000011
#define sah_insert_pkha_rtn_mod_emodn          0x00000012
#define sah_insert_pkha_rtn_f2m_emodn          0x80000013
#define sah_insert_pkha_rtn_ec_fp_ptmul        0x00000014
#define sah_insert_pkha_rtn_ec_f2m_ptmul       0x80000015
#define sah_insert_pkha_rtn_f2m_gcd            0x80000016
#define sah_insert_pkha_rtn_mod_gcd            0x00000017
#define sah_insert_pkha_rtn_f2m_dbl_aff        0x00000018
#define sah_insert_pkha_rtn_fp_dbl_aff         0x80000019
#define sah_insert_pkha_rtn_f2m_add_aff        0x8000001A
#define sah_insert_pkha_rtn_fp_add_aff         0x0000001B
#define sah_insert_pkha_rtn_f2m_exp            0x8000001C
#define sah_insert_pkha_rtn_mod_exp_teq        0x0000001D
#define sah_insert_pkha_rtn_rsa_sstep_teq      0x0000001E
#define sah_insert_pkha_rtn_f2m_multn          0x8000001F
#define sah_insert_pkha_rtn_mod_multn          0x80000020
#define sah_insert_pkha_rtn_mod_add            0x00000021
#define sah_insert_pkha_rtn_mod_sub            0x00000022
#define sah_insert_pkha_rtn_mod_mult1_mont     0x80000023
#define sah_insert_pkha_rtn_mod_mult2_deconv   0x00000024
#define sah_insert_pkha_rtn_f2m_add            0x80000025
#define sah_insert_pkha_rtn_f2m_mult1_mont     0x80000026
#define sah_insert_pkha_rtn_f2m_mult2_deconv   0x00000027
#define sah_insert_pkha_rtn_miller_rabin       0x00000028
#define sah_insert_pkha_rtn_mod_amodn          0x00000029
#define sah_insert_pkha_rtn_f2m_amodn          0x8000002A
/*! @} */

/*! Add a descriptor with two input pointers */
fsl_shw_return_t sah_add_two_in_desc(uint32_t header,
				     const uint8_t * in1,
				     uint32_t in1_length,
				     const uint8_t * in2,
				     uint32_t in2_length,
				     const sah_Mem_Util * mu,
				     sah_Head_Desc ** desc_chain);

/*! Add a descriptor with two 'data' pointers */
fsl_shw_return_t sah_add_two_d_desc(uint32_t header,
				    const uint8_t * in1,
				    uint32_t in1_length,
				    const uint8_t * in2,
				    uint32_t in2_length,
				    const sah_Mem_Util * mu,
				    sah_Head_Desc ** desc_chain);

/*! Add a descriptor with an input and key pointer */
fsl_shw_return_t sah_add_in_key_desc(uint32_t header,
				     const uint8_t * in1,
				     uint32_t in1_length,
				     fsl_shw_sko_t * key_info,
				     const sah_Mem_Util * mu,
				     sah_Head_Desc ** desc_chain);

/*! Add a descriptor with two key pointers */
fsl_shw_return_t sah_add_key_key_desc(uint32_t header,
				      fsl_shw_sko_t * key_info1,
				      fsl_shw_sko_t * key_info2,
				      const sah_Mem_Util * mu,
				      sah_Head_Desc ** desc_chain);

/*! Add a descriptor with two output pointers */
fsl_shw_return_t sah_add_two_out_desc(uint32_t header,
				      uint8_t * out1,
				      uint32_t out1_length,
				      uint8_t * out2,
				      uint32_t out2_length,
				      const sah_Mem_Util * mu,
				      sah_Head_Desc ** desc_chain);

/*! Add a descriptor with an input and output pointer */
fsl_shw_return_t sah_add_in_out_desc(uint32_t header,
				     const uint8_t * in, uint32_t in_length,
				     uint8_t * out, uint32_t out_length,
				     const sah_Mem_Util * mu,
				     sah_Head_Desc ** desc_chain);

/*! Add a descriptor with an input and key output pointer */
fsl_shw_return_t sah_add_in_keyout_desc(uint32_t header,
					const uint8_t * in, uint32_t in_length,
					fsl_shw_sko_t * key_info,
					const sah_Mem_Util * mu,
					sah_Head_Desc ** desc_chain);

/*! Add a descriptor with a key and an output pointer */
fsl_shw_return_t sah_add_key_out_desc(uint32_t header,
					  const fsl_shw_sko_t * key_info,
				      uint8_t * out, uint32_t out_length,
				      const sah_Mem_Util * mu,
				      sah_Head_Desc ** desc_chain);

/*! Add a descriptor with an output and input pointer */
fsl_shw_return_t sah_add_out_in_desc(uint32_t header,
				     uint8_t * out, uint32_t out_length,
				     const uint8_t * in, uint32_t in_length,
				     const sah_Mem_Util * mu,
				     sah_Head_Desc ** desc_chain);

/*! Verify that supplied User Context Object is valid */
fsl_shw_return_t sah_validate_uco(fsl_shw_uco_t * uco);

#endif				/* SF_UTIL_H */

/* End of sf_util.h */
