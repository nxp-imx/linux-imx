// SPDX-License-Identifier: GPL-2.0
#include <linux/sched.h>
#include <asm/cpufeature.h>
#include <asm/current.h>
#include "internal.h"
#include "fpu.h"
#include "xstate.h"

/*
 * percpu_fpstate is used to reset the FPU hardware to its initial state
 * when switching from pVM to host, preventing the host from accessing
 * pVM's FPU contents.
 *
 * To achieve this, it sets percpu_fpstate.regs.xsave.header.xfeatures to 0
 * (which represents the XSTATE_BV field in the XSAVE header), ensuring the
 * XRSTOR(S) instruction loads the default FPU state. Since it doesn't need
 * full-sized memory pages for all FPU registers, a static allocation of
 * percpu_fpstate is sufficient.
 */
static DEFINE_PER_CPU(struct fpstate, percpu_fpstate);

int pkvm_init_percpu_fpu(void)
{
	struct fpu *fpu = x86_task_fpu(current);

	if (boot_cpu_has(X86_FEATURE_XSAVE)) {
		/*
		 * According to SDM Vol. 1 Operation Of XSAVE/XRSTOR:
		 * 1) If the XSAVE feature set is not enabled (CR4.OSXSAVE = 0),
		 * an invalid-opcode exception (#UD) occurs.
		 * 2) If CR0.TS[bit 3] is 1, a device-not-available exception
		 * (#NM) occurs.
		 * The pKVM's CR4/CR0 are configured with the same value as the
		 * host before deprivileging, so most likely CR4.OSXSAVE is
		 * already set and CR0.TS is clear. But still check the CR4/CR0
		 * in case the host didn't.
		 */
		if (!(native_read_cr4() & X86_CR4_OSXSAVE))
			return -EINVAL;

		if (native_read_cr0() & X86_CR0_TS)
			return -EINVAL;
	}

	/*
	 * Set the current fpstate pointer to the percpu_fpstate, which is used
	 * to restore the FPU to the initial state before switching from a pVM
	 * to the host.
	 */
	fpu->fpstate = this_cpu_ptr(&percpu_fpstate);
	fpstate_init_user(fpu->fpstate);

	/* The perm is initialized with the maximum features */
	fpu->perm.__state_perm		= fpu_kernel_cfg.max_features;
	fpu->perm.__state_size		= fpu_kernel_cfg.max_size;

	fpu->guest_perm = fpu->perm;

	/*
	 * As this function is used during pKVM initialization, there is no
	 * guest FPU state on hardware yet, thus set TIF_NEED_FPU_LOAD to
	 * indicate this.
	 */
	set_thread_flag(TIF_NEED_FPU_LOAD);

	return 0;
}

void pkvm_init_guest_fpu(struct fpu_guest *gfpu)
{
	u64 permitted = xstate_get_group_perm(true);
	struct fpstate *fpstate = gfpu->fpstate;

	fpstate->xfeatures	= fpu_kernel_cfg.default_features & permitted;
	fpstate->user_xfeatures	= fpu_user_cfg.default_features & permitted;
	fpstate->xfd		= 0;

	fpstate->in_use		= false;

	fpstate_init_user(fpstate);

	gfpu->xfeatures		= fpstate->user_xfeatures;
}
