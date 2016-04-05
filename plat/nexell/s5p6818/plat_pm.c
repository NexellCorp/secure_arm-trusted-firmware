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
	unsigned int data, regdata;

	/* Set arm64 mode */
	ctrl_addr = NXP_CPU_CLUSTERx_CTRL(linear_id);

	data  = mmio_read_32(ctrl_addr);
	data |= (1 << NXP_CPU_CLUSTERx_AARCH64_SHIFT(linear_id));
	mmio_write_32(ctrl_addr, data);

	if (cluster)
		while (!(mmio_read_32(NXP_TIEOFF_REG(107)) & (1 << cpu)))
			;
	else
		while (!(mmio_read_32(NXP_TIEOFF_REG(90)) & (1 << cpu)))
			;

	regdata = mmio_read_32(NXP_CPU_PWRUP_REQ_CTRL);
	mmio_write_32(NXP_CPU_PWRUP_REQ_CTRL, regdata & ~(1 << linear_id));
	mmio_write_32(NXP_CPU_PWRDOWN_REQ_CTRL, (1 << linear_id));
	__asm__ __volatile__ ("dmb sy");
	__asm__ __volatile__ ("isb");
	/* wait update of NXP_CPU_PWR_STATUS register: 10000 is loop count */
	plat_reg_delay(10000);
	while (!(mmio_read_32(NXP_CPU_PWR_STATUS) & (1 << (linear_id+8))))
		;

	regdata = mmio_read_32(NXP_CPU_PWRDOWN_REQ_CTRL);
	mmio_write_32(NXP_CPU_PWRDOWN_REQ_CTRL, regdata & ~(1 << linear_id));
	mmio_write_32(NXP_CPU_PWRUP_REQ_CTRL, (1 << linear_id));
	__asm__ __volatile__ ("dmb sy");
	__asm__ __volatile__ ("isb");
	plat_reg_delay(10000);	/* if bus is busy, status is not updated */
	while (mmio_read_32(NXP_CPU_PWR_STATUS) & (1 << (linear_id+8)))
		;

	regdata = mmio_read_32(NXP_CPU_PWRUP_REQ_CTRL);
	mmio_write_32(NXP_CPU_PWRUP_REQ_CTRL, regdata & ~(1 << linear_id));

	/* TODO: need workaround code.
	 * Current code has implicit problem when rich os is busy
	 * and status of new added cpu is not wfi.
	 */
	if (cluster)
		while (mmio_read_32(NXP_TIEOFF_REG(107)) & (1 << cpu))
			;
	else
		while (mmio_read_32(NXP_TIEOFF_REG(90)) & (1 << cpu))
			;
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
			__func__, (unsigned long long)mpidr, afflvl, state);

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
static void s5p6818_cpu_off(uint32_t afflvl)
{
	uint32_t regdata, linear_id;

	linear_id = platform_get_core_pos(plat_my_core_pos());

	regdata = mmio_read_32(NXP_CPU_PWRUP_REQ_CTRL);
	mmio_write_32(NXP_CPU_PWRUP_REQ_CTRL,  regdata & ~(1<< linear_id));
	/* 
	 * TODO afflvl 1 is need more power function. but currently skiped
	 * if all cpu is off in same cluster, system will be corrupted
	 */
	if(linear_id & 0x3)
		mmio_write_32(NXP_CPU_PWRDOWN_REQ_CTRL, (1 << linear_id));
	__asm__ __volatile__ ("dmb sy");
	__asm__ __volatile__ ("isb");
	__asm__ __volatile__ ("dsb sy");
	__asm__ __volatile__ ("wfi");
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
	}else
		s5p6818_cpu_off(afflvl);
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

/* 
 * send nxe2000 pmic power off command through i2c port 2
 * device id:0x32, register number 0xE, register value 0x1
 */
struct nxi2c {
	uint32_t iccr;
	uint32_t icsr;
	uint32_t icar;
	uint32_t idsr;
	uint32_t lr;
};
#define NUMOFI2CPORT
#define NXI2C_PORT      2
#define PMIC_ID         (0x32<<1)
#define PMIC_PWROFF     0x0E
static uintptr_t i2cbase [NUMOFI2CPORT] =
{
	0xc00a4000,
	0xc00a5000,
	0xC00A6000
};
static uintptr_t i2cclkgen [NUMOFI2CPORT] =
{
	0xc00ae000,
	0xc00af000,
	0xC00B0000
};
static uint8_t i2crst[NUMOFI2CPORT] =
{ 20, 21, 22};

static void __dead2 s5p6818_system_off(void)
{
	uint32_t regvalue;
	register struct nxi2c *I2C_BASE = (struct nxi2c *)i2cbase[NXI2C_PORT];
	uintptr_t clkgen = i2cclkgen[NXI2C_PORT];

	regvalue = mmio_read_32(clkgen);
	mmio_write_32(clkgen,
			regvalue | 1<<3 );      /* set i2c pclk mode*/

	regvalue = mmio_read_32((uintptr_t)(GPIO_BASE+GPIO_OFFSET*3+0x20));
	mmio_write_32((uintptr_t)(GPIO_BASE+GPIO_OFFSET*3+0x20),
			(regvalue & ~0xF<<(6*2)) | 5<<(6*2));   /* gpio alt1 */

	regvalue = mmio_read_32((uintptr_t)DEVICE_RESET_BASE);
	mmio_write_32((uintptr_t)DEVICE_RESET_BASE,
			regvalue | 1<<(i2crst[NXI2C_PORT]));    /* reset off */

	mmio_write_32((uintptr_t)&I2C_BASE->iccr,
			0xF<<5 | 0xF<<0);       /* pclk/256/(15+1) */
	mmio_write_32((uintptr_t)&I2C_BASE->lr,
			1<<2 | 3<<0);   /* filter enable, 15 clocks delayed */

	mmio_write_32((uintptr_t)&I2C_BASE->icsr,
			3<<6 | 1<<5 | 1<<4);    /* enable tx rx */

	mmio_write_32((uintptr_t)&I2C_BASE->idsr, PMIC_ID & 0xFE);
	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->icsr);
	mmio_write_32((uintptr_t)&I2C_BASE->icsr, regvalue | 1<<5); /* start */

	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->iccr);
	mmio_write_32((uintptr_t)&I2C_BASE->iccr,
			regvalue & ~(1<<4));    /* clr int pend(nxt start)*/
	/* wait int pending */
	while(!(mmio_read_32((uintptr_t)&I2C_BASE->iccr) & 1<<4))
		;

	if(mmio_read_32((uintptr_t)&I2C_BASE->icsr) & 1<<0)
	{
		VERBOSE("no device ack\r\n");
		goto i2cexit;
	}

	mmio_write_32((uintptr_t)&I2C_BASE->idsr, PMIC_PWROFF);
	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->iccr);
	mmio_write_32((uintptr_t)&I2C_BASE->iccr,
			regvalue & ~(1<<4));    /* clr int pend(nxt start)*/
	/* wait int pending */
	while(!(mmio_read_32((uintptr_t)&I2C_BASE->iccr) & 1<<4))
		;

	mmio_write_32((uintptr_t)&I2C_BASE->idsr, 1);
	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->iccr);
	mmio_write_32((uintptr_t)&I2C_BASE->iccr, regvalue & ~(1<<4));
	/* wait int pending */
	while(!(mmio_read_32((uintptr_t)&I2C_BASE->iccr) & 1<<4))
		;

	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->iccr);
	mmio_write_32((uintptr_t)&I2C_BASE->iccr,regvalue | (1<<4));

	if(mmio_read_32((uintptr_t)&I2C_BASE->icsr) & 1<<0)
	{
		VERBOSE("no data ack\r\n");
		goto i2cexit;
	}

	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->iccr);
	mmio_write_32((uintptr_t)&I2C_BASE->iccr,
			(regvalue & ~(1<<4)) | (1<<8));

	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->icsr);
	mmio_write_32((uintptr_t)&I2C_BASE->icsr,
			regvalue & ~(1<<5)); /* stop */

	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->icsr);
	mmio_write_32((uintptr_t)&I2C_BASE->icsr,
			regvalue & ~(1<<4));    /* enable tx rx */

i2cexit:
	regvalue = mmio_read_32((uintptr_t)&I2C_BASE->icsr);
	mmio_write_32((uintptr_t)&I2C_BASE->icsr,
			regvalue & ~(1<<4));    /* bus disable */
	VERBOSE("i2c stop\r\n");

	wfi();
	panic();
}

static void __dead2 s5p6818_system_reset(void)
{
        uint32_t regvalue;
	VERBOSE("%s: reset system\n", __func__);

	regvalue = mmio_read_32(DEVICE_RESET_BASE+((DEVICE_RESET_WDT0>>5)<<2));
	mmio_write_32(DEVICE_RESET_BASE+((DEVICE_RESET_WDT0>>5)<<2), 
			regvalue | 3<<(DEVICE_RESET_WDT0 & (32-1)));

	regvalue =      WDT_PRE_CLK<<8 |       /* prescaler */
		WDT_DIVFACTOR128<<3  |
		0x1<<2;         /* watchdog reset enable */

	mmio_write_32(WDT_BASE, regvalue);
	mmio_write_32(WDT_BASE+8, 0x1); /* reset count */
	regvalue |= 1<<5;       /* watchdog timer enable */
	mmio_write_32(WDT_BASE, regvalue);

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
	.system_off		= s5p6818_system_off,
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
