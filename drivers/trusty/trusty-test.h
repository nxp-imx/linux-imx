/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 Google, Inc.
 */
#ifndef _TRUSTY_TEST_H
#define _TRUSTY_TEST_H

#define SMC_SC_TEST_VERSION SMC_STDCALL_NR(SMC_ENTITY_TEST, 0)
#define SMC_SC_TEST_SHARED_MEM_RW SMC_STDCALL_NR(SMC_ENTITY_TEST, 1)

/**
 * SMC_SC_TEST_CLOBBER_FPSIMD_CLOBBER - Test save and clobber of FP/SIMD
 * registers during an NS <-> TF-A <-> Trusty roundtrip.
 *
 * Return: 0 on success, or one of the libsm errors otherwise:
 * * %SM_ERR_NOT_ALLOWED: Not allowed to enable the FPU in Trusty.
 * * %SM_ERR_INTERNAL_FAILURE: The test failed to load random values
 *                             into the FP registers.
 *
 * Set all of the secure-side FP registers to random values.
 */
#define SMC_FC_TEST_CLOBBER_FPSIMD_CLOBBER SMC_FASTCALL_NR(SMC_ENTITY_TEST, 0)

/**
 * SMC_SC_TEST_CLOBBER_FPSIMD_CHECK - Check and restore FP/SIMD
 * registers after an NS <-> TF-A <-> Trusty roundtrip.
 *
 * Return: 0 on success, or one of the libsm errors otherwise:
 * * %SM_ERR_NOT_ALLOWED: Not allowed to enable the FPU in Trusty.
 * * %SM_ERR_BUSY: Another thread clobbered our registers.
 * * %SM_ERR_INTERNAL_FAILURE: The FP registers did not match
 *                             the expected values.
 *
 * The call should immediately follow a corresponding clobber,
 * since the latter stores some internal state in Trusty.
 *
 * The caller should disable interrupts before
 * &SMC_FC_TEST_CLOBBER_FPSIMD_CLOBBER and don't re-enable before
 * &SMC_FC_TEST_CLOBBER_FPSIMD_CHECK returns to avoid the %SM_ERR_BUSY error.
 */
#define SMC_FC_TEST_CLOBBER_FPSIMD_CHECK SMC_FASTCALL_NR(SMC_ENTITY_TEST, 1)

/**
 * SMC_NC_TEST_CLOBBER_FPSIMD_TIMER - Trigger the FP/SIMD test timer.
 *
 * Return: 1 on success, or one of the libsm errors otherwise.
 *
 * Trigger a secure timer that runs periodically a fixed number of
 * times, then automatically disables itself.
 *
 * The timer is not strictly required for the test, so failing to
 * start or stop the timer is not an error per se.
 */
#define SMC_NC_TEST_CLOBBER_FPSIMD_TIMER SMC_STDCALL_NR(SMC_ENTITY_TEST, 0)

#define TRUSTY_STDCALLTEST_API_VERSION 1

void trusty_fpsimd_save_state(struct user_fpsimd_state *fp_regs);
void trusty_fpsimd_load_state(struct user_fpsimd_state *fp_regs);

#endif /* _TRUSTY_TEST_H */
