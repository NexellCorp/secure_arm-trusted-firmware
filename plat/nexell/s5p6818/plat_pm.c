/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <cci.h>
#include <debug.h>
#include <cci.h>
#include <errno.h>
#include <gic_v2.h>
#include <s5p6818.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <s5p6818_timer.h>
#include <psci.h>
#include <s5p6818_def.h>
#include <s5p6818_private.h>

static void s5p6818_power_on_cpu(int cluster, int cpu, int linear_id)
{
	unsigned int ctrl_addr;
	unsigned int data;

	/* Set arm64 mode */
	ctrl_addr = NXP_CPU_CLUSTERx_CTRL(linear_id);

	data  = mmio_read_32(ctrl_addr);
	data |= (1 << NXP_CPU_CLUSTERx_AARCH64_SHIFT(linear_id));
	mmio_write_32(ctrl_addr, data);

	mmio_write_32(NXP_CPU_RESET_ENB_CTRL, 1);
	mmio_write_32(NXP_CPU_PWRDOWN_REQ_CTRL, (1 << linear_id));
	mmio_write_32(NXP_CPU_PWRUP_REQ_CTRL,   (1 << linear_id));

	/* Checking WFI of cpus */
	if (cluster)
		while (mmio_read_32(NXP_TIEOFF_REG(107)) & (1 << cpu))
			;
	else
		while (mmio_read_32(NXP_TIEOFF_REG(90)) & (1 << cpu))
			;

	mmio_write_32(NXP_CPU_PWRUP_REQ_CTRL, 0);
}

/*******************************************************************************
 * Nexell handler called when an affinity instance is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
int32_t s5p6818_affinst_on(uint64_t mpidr,
			 uint64_t sec_entrypoint,
			 uint32_t afflvl,
			 uint32_t state)
{
	int cpu, cluster;
	uint32_t linear_id;
	uint32_t temp;

	linear_id = platform_get_core_pos(mpidr);
	cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFF1_SHIFT;
	cpu = mpidr & MPIDR_CPU_MASK;

	VERBOSE("#%s, mpidr:%llx, afflvl:%x, state:%x\n",
			__func__, mpidr, afflvl, state);

	/* directly return for power on */
	if (state == PSCI_STATE_ON)
		return PSCI_E_SUCCESS;

	switch (afflvl) {
	case MPIDR_AFFLVL0:
		/* Setup cpu entrypoint when it next powers up */
		temp = (unsigned int)(sec_entrypoint >> 2);

		do {
			mmio_write_32(NXP_CPUx_RVBARADDR(linear_id), temp);
		} while (mmio_read_32(NXP_CPUx_RVBARADDR(linear_id)) != temp);

		s5p6818_power_on_cpu(cluster, cpu, linear_id);
		break;
	}

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Nexell handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 ******************************************************************************/
void s5p6818_affinst_on_finish(uint32_t afflvl, uint32_t state)
{
	uint32_t linear_id;

	/* Get the mpidr for this cpu */
	linear_id = plat_my_core_pos();

	/*
	 * Perform the common cluster specific operations i.e enable coherency
	 * if this cluster was off.
	 */
	if (afflvl > MPIDR_AFFLVL0) {
		/* Enable coherency if this cluster was off */
		plat_cci_enable();
	}

	/* Cleanup cpu entry point */
	mmio_write_32(NXP_CPUx_RVBARADDR(linear_id), 0x0);

	/* Enable the gic cpu interface */
	arm_gic_cpuif_setup();

	/* TODO: if GIC in AON, then just need init for cold boot */
	arm_gic_pcpu_distif_setup();
}

static int32_t s5p6818_do_plat_actions(uint32_t afflvl, uint32_t state)
{
	uint32_t max_phys_off_afflvl;

	assert(afflvl <= MPIDR_AFFLVL1);

	if (state != PSCI_STATE_OFF)
		return -EAGAIN;

	/*
	 * Find the highest affinity level which will be suspended and postpone
	 * all the platform specific actions until that level is hit.
	 */
	max_phys_off_afflvl = psci_get_max_phys_off_afflvl();
	assert(max_phys_off_afflvl != PSCI_INVALID_DATA);
	assert(psci_get_suspend_afflvl() >= max_phys_off_afflvl);
	if (afflvl != max_phys_off_afflvl)
		return -EAGAIN;

	return 0;
}

static void s5p6818_affinst_off(uint32_t afflvl, uint32_t state)
{
	if (s5p6818_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/* Prevent interrupts from spuriously waking up this cpu */
	arm_gic_cpuif_deactivate();

	if (afflvl != MPIDR_AFFLVL0) {
		/* Disable coherency if this cluster is to be turned off */
		plat_cci_disable();
	}
}

static void s5p6818_affinst_suspend(uint64_t sec_entrypoint,
				  uint32_t afflvl,
				  uint32_t state)
{
	uint32_t linear_id;

	/* Get the mpidr for this cpu */
	linear_id = plat_my_core_pos();

	/* Determine if any platform actions need to be executed */
	if (s5p6818_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/* Set cpu entry point */
	mmio_write_32(NXP_CPUx_RVBARADDR(linear_id),
			(unsigned int)(sec_entrypoint >> 2));

	/* Cluster is to be turned off, so disable coherency */
	if (afflvl > MPIDR_AFFLVL0)
		plat_cci_disable();

	if (afflvl > MPIDR_AFFLVL1)
		/* Prevent interrupts from spuriously waking up this cpu */
		arm_gic_cpuif_deactivate();
}

static void s5p6818_affinst_suspend_finish(uint32_t afflvl,
					 uint32_t state)
{
	uint32_t linear_id;

	/* Get the mpidr for this cpu */
	linear_id = plat_my_core_pos();

	if (afflvl > MPIDR_AFFLVL1) {
		/* Enable the gic cpu interface */
		arm_gic_setup();
		arm_gic_cpuif_setup();
	}

	if (afflvl > MPIDR_AFFLVL0) {
		/* Enable coherency if this cluster was off */
		plat_cci_enable();
	}

	/* TODO: This setup is needed only after a cold boot */
	arm_gic_pcpu_distif_setup();

	/* cleanup cpu entry point */
	mmio_write_32(NXP_CPUx_RVBARADDR(linear_id), 0x0);
}

static void __dead2 s5p6818_system_reset(void)
{
	VERBOSE("%s: reset system\n", __func__);

	wfi();
	panic();
}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_pm_ops_t s5p6818_ops = {
	.affinst_on		= s5p6818_affinst_on,
	.affinst_on_finish	= s5p6818_affinst_on_finish,
	.affinst_off		= s5p6818_affinst_off,
	.affinst_standby	= NULL,
	.affinst_suspend	= s5p6818_affinst_suspend,
	.affinst_suspend_finish	= s5p6818_affinst_suspend_finish,
	.system_off		= NULL,
	.system_reset		= s5p6818_system_reset,
};

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int32_t platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &s5p6818_ops;
	return 0;
}
