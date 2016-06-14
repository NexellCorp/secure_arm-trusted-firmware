/*
 * Copyright (c) 2014-2015, Linaro Ltd and Contributors. All rights reserved.
 * Copyright (c) 2014-2015, Nexell Ltd and Contributors. All rights reserved.
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
#include <cci.h>
#include <arm_gic.h>
#include <assert.h>
#include <bl_common.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <xlat_tables.h>
#include <s5p6818_def.h>

#define MAP_LOADER_FLASH	MAP_REGION_FLAT(FLASH_LOADER_BASE,	\
					FLASH_LOADER_SIZE,		\
					MT_MEMORY | MT_RO | MT_SECURE)

#define MAP_SECURE_FLASH	MAP_REGION_FLAT(FLASH_SECURE_BASE,	\
					FLASH_SECURE_SIZE,		\
					MT_MEMORY | MT_RW | MT_SECURE)

#define MAP_NONSECURE_FLASH	MAP_REGION_FLAT(FLASH_NONSECURE_BASE,	\
					FLASH_NONSECURE_SIZE,		\
					MT_MEMORY | MT_RW | MT_NS)

#define MAP_DEVICE	MAP_REGION_FLAT(DEVICE_BASE,			\
					DEVICE_SIZE,			\
					MT_DEVICE | MT_RW | MT_SECURE)

#define MAP_NS_DRAM	MAP_REGION_FLAT(DRAM_NS_BASE,			\
					DRAM_NS_SIZE,			\
					MT_DEVICE | MT_RW | MT_NS)

#define MAP_SEC_MEM	MAP_REGION_FLAT(DRAM_SEC_BASE,			\
					DRAM_SEC_SIZE,			\
					MT_MEMORY | MT_RW | MT_SECURE)

#define MAP_TSP_MEM	MAP_REGION_FLAT(TSP_SEC_MEM_BASE,		\
					TSP_SEC_MEM_SIZE,		\
					MT_MEMORY | MT_RW | MT_SECURE)

#define MAP_ROM_PARAM	MAP_REGION_FLAT(ATFRAM0_BASE,			\
					0x200000,			\
					MT_MEMORY | MT_RW | MT_SECURE)

#define MAP_SRAM	MAP_REGION_FLAT(SRAM_BASE,			\
					SRAM_SIZE,			\
					MT_MEMORY | MT_RW | MT_SECURE)

#ifdef BL31_ON_SRAM
#define MAP_BL31_DRAM	MAP_REGION_FLAT(BL31_DRAM_BASE,			\
					BL31_DRAM_SIZE,			\
					MT_MEMORY | MT_RW | MT_SECURE)
#endif

/*
 * Table of regions for different BL stages to map using the MMU.
 * This doesn't include Trusted RAM as the 'mem_layout' argument passed to
 * configure_mmu_elx() will give the available subset of that,
 */
#if IMAGE_BL1
static const mmap_region_t s5p6818_mmap[] = {
	MAP_LOADER_FLASH,
	MAP_DEVICE,
	MAP_NS_DRAM,
	MAP_SEC_MEM,
	MAP_ROM_PARAM,
	MAP_SRAM,
	{0}
};
#endif
#if IMAGE_BL2
static const mmap_region_t s5p6818_mmap[] = {
	MAP_LOADER_FLASH,
	MAP_SECURE_FLASH,
	MAP_NONSECURE_FLASH,
	MAP_DEVICE,
	MAP_NS_DRAM,
	MAP_SEC_MEM,
	MAP_SRAM,
#ifdef BL31_ON_SRAM
	MAP_BL31_DRAM,
#endif
	{0}
};
#endif
#if IMAGE_BL31
static const mmap_region_t s5p6818_mmap[] = {
	MAP_DEVICE,
	MAP_NS_DRAM,
	MAP_TSP_MEM,
#ifdef BL31_ON_SRAM
	MAP_SEC_MEM,
	MAP_SRAM,
	MAP_BL31_DRAM,
#endif
	{0}
};
#endif
#if IMAGE_BL32
static const mmap_region_t s5p6818_mmap[] = {
	MAP_DEVICE,
	MAP_NS_DRAM,
	MAP_TSP_MEM,
	{0}
};
#endif

/* Array of secure interrupts to be configured by the gic driver */
const unsigned int irq_sec_array[] = {
	IRQ_SEC_PHY_TIMER,
	IRQ_SEC_SGI_0,
	IRQ_SEC_SGI_1,
	IRQ_SEC_SGI_2,
	IRQ_SEC_SGI_3,
	IRQ_SEC_SGI_4,
	IRQ_SEC_SGI_5,
	IRQ_SEC_SGI_6,
	IRQ_SEC_SGI_7,
	IRQ_SEC_SGI_8
};

const unsigned int num_sec_irqs = sizeof(irq_sec_array) /
	sizeof(irq_sec_array[0]);

static const int s5p6818_cci_map[] = {
	CCI_CLUSTER0_SL_IFACE_IX,
	CCI_CLUSTER1_SL_IFACE_IX
};

/*******************************************************************************
 * Macro generating the code for the function setting up the pagetables as per
 * the platform memory map & initialize the mmu, for the given exception level
 ******************************************************************************/
#if USE_COHERENT_MEM
#define DEFINE_CONFIGURE_MMU_EL(_el)					\
	void plat_configure_mmu_el ## _el(unsigned long total_base,	\
					  unsigned long total_size,	\
					  unsigned long ro_start,	\
					  unsigned long ro_limit,	\
					  unsigned long coh_start,	\
					  unsigned long coh_limit)	\
	{								\
		mmap_add_region(total_base, total_base,			\
				total_size,				\
				MT_MEMORY | MT_RW | MT_SECURE);		\
		mmap_add_region(ro_start, ro_start,			\
				ro_limit - ro_start,			\
				MT_MEMORY | MT_RO | MT_SECURE);		\
		mmap_add_region(coh_start, coh_start,			\
				coh_limit - coh_start,			\
				MT_DEVICE | MT_RW | MT_SECURE);		\
		mmap_add(s5p6818_mmap);					\
		init_xlat_tables();					\
									\
		enable_mmu_el##_el(0);					\
	}
#else
#define DEFINE_CONFIGURE_MMU_EL(_el)					\
	void plat_configure_mmu_el ## _el(unsigned long total_base,	\
					  unsigned long total_size,	\
					  unsigned long ro_start,	\
					  unsigned long ro_limit)	\
	{								\
		mmap_add(s5p6818_mmap);					\
		mmap_add_region(total_base, total_base,			\
				total_size,				\
				MT_MEMORY | MT_RW | MT_SECURE);		\
		mmap_add_region(ro_start, ro_start,			\
				ro_limit - ro_start,			\
				MT_MEMORY | MT_RO | MT_SECURE);		\
		init_xlat_tables();					\
									\
		enable_mmu_el##_el(0);					\
	}
#endif

/* Define EL1 and EL3 variants of the function initialising the MMU */
DEFINE_CONFIGURE_MMU_EL(1)
DEFINE_CONFIGURE_MMU_EL(3)

unsigned long plat_get_ns_image_entrypoint(void)
{
	return NS_IMAGE_OFFSET;
}

/*******************************************************************************
 * Gets SPSR for BL32 entry
 ******************************************************************************/
uint32_t plat_get_spsr_for_bl32_entry(void)
{
	/*
	 * The Secure Payload Dispatcher service is responsible for
	 * setting the SPSR prior to entry into the BL3-2 image.
	 */
	return 0;
}

/*******************************************************************************
 * Gets SPSR for BL33 entry
 ******************************************************************************/
uint32_t plat_get_spsr_for_bl33_entry(void)
{
	unsigned long el_status;
	unsigned int mode;
	uint32_t spsr;

	/* Figure out what mode we enter the non-secure world in */
	el_status = read_id_aa64pfr0_el1() >> ID_AA64PFR0_EL2_SHIFT;
	el_status &= ID_AA64PFR0_ELX_MASK;

	mode = (el_status) ? MODE_EL2 : MODE_EL1;

	/*
	 * TODO: Consider the possibility of specifying the SPSR in
	 * the FIP ToC and allowing the platform to have a say as
	 * well.
	 */
	spsr = SPSR_64(mode, MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
	return spsr;
}

uint64_t plat_get_syscnt_freq(void)
{
	return SYS_COUNTER_FREQ_IN_TICKS;
}

void plat_gic_init(void)
{
	arm_gic_init(GICC_BASE, GICD_BASE, 0, irq_sec_array, num_sec_irqs);
}

void plat_cci_init(void)
{
	cci_init(CCI400_BASE, s5p6818_cci_map, ARRAY_SIZE(s5p6818_cci_map));
}

void plat_cci_enable(void)
{
	/*
	 * Enable CCI coherency for this cluster.
	 * No need for locks as no other cpu is active at the moment.
	 */
	cci_enable_snoop_dvm_reqs(MPIDR_AFFLVL1_VAL(read_mpidr()));
}

void plat_cci_disable(void)
{
	cci_disable_snoop_dvm_reqs(MPIDR_AFFLVL1_VAL(read_mpidr()));
}
