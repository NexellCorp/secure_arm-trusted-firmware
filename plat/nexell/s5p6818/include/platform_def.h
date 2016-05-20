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

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

#include <arch.h>
#include <s5p6818_def.h>
#include <common_def.h>

#define DEBUG_XLAT_TABLE 0

/*******************************************************************************
 * Platform binary types for linking
 ******************************************************************************/
#define PLATFORM_LINKER_FORMAT          "elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH            aarch64

/*******************************************************************************
 * Generic platform constants
 ******************************************************************************/

/* Size of cacheable stacks */
#if DEBUG_XLAT_TABLE
#define PLATFORM_STACK_SIZE 0x800
#elif IMAGE_BL1
#define PLATFORM_STACK_SIZE 0x1440
#elif IMAGE_BL2
#define PLATFORM_STACK_SIZE 0x2000
#elif IMAGE_BL31
#define PLATFORM_STACK_SIZE 0x800
#elif IMAGE_BL32
#define PLATFORM_STACK_SIZE 0x440
#endif

#define FIRMWARE_WELCOME_STR		"Booting Trusted Firmware\n"

#define LOADER_MEM_NAME			"loader_mem"

#define BOOT_EMMC_NAME			"l-loader.bin"

#define NORMAL_EMMC_NAME		"normal emmc"

/* Firmware Image Package */
#define FIP_IMAGE_ID			0

#define FIP_LOADER_IMAGE_ID		1
#define FIP_SECURE_IMAGE_ID		2
#define FIP_NONSECURE_IMAGE_ID		3
/* Trusted Boot Firmware BL2 */
#define BL2_IMAGE_ID			4

/* SCP Firmware BL3-0 */
#define BL30_IMAGE_ID			5

/* EL3 Runtime Firmware BL31 */
#define BL31_IMAGE_ID			6

/* Secure Payload BL32 (Trusted OS) */
#define BL32_IMAGE_ID			7

/* Non-Trusted Firmware BL33 */
#define BL33_IMAGE_ID			8

/* Certificates */
#define BL2_CERT_ID			9
#define TRUSTED_KEY_CERT_ID		10

#define BL30_KEY_CERT_ID		11
#define BL31_KEY_CERT_ID		12
#define BL32_KEY_CERT_ID		13
#define BL33_KEY_CERT_ID		14

#define BL30_CERT_ID			15
#define BL31_CERT_ID			16
#define BL32_CERT_ID			17
#define BL33_CERT_ID			18

#define BOOT_EMMC_ID			(BL33_CERT_ID + 1)
#define NORMAL_EMMC_ID			(BL33_CERT_ID + 2)

#define PLATFORM_MAX_AFFLVL		MPIDR_AFFLVL1
#define PLATFORM_CACHE_LINE_SIZE	64
#define PLATFORM_CLUSTER_COUNT		2
#define PLATFORM_CLUSTER0_CORE_COUNT	4
#define PLATFORM_CLUSTER1_CORE_COUNT	4
#define PLATFORM_CORE_COUNT		(PLATFORM_CLUSTER1_CORE_COUNT +	\
					 PLATFORM_CLUSTER0_CORE_COUNT)
#define PLATFORM_MAX_CPUS_PER_CLUSTER	4
#define PLATFORM_NUM_AFFS		(PLATFORM_CLUSTER_COUNT + \
					 PLATFORM_CORE_COUNT)
#define MAX_IO_DEVICES			3
#define MAX_IO_HANDLES			4

/*******************************************************************************
 * Platform memory map related constants
 ******************************************************************************/

#define MMC_BASE			0x00000000
#define MMC_SIZE			0x80000000
#define MMC_BLOCK_SIZE			0x200		/* 512 bytes */

#define MMC_LOADER_BASE			(MMC_BASE + (0x10200))
#define MMC_LOADER_SIZE			(0x40000-0x1000)

#define MMC_SECURE_BASE			(MMC_BASE + (0x50200))
#define MMC_SECURE_SIZE			(0xC0000-0x1000)

#define MMC_NONSECURE_BASE		(MMC_BASE + (0x110200))
#define MMC_NONSECURE_SIZE		(0x200000-0x1000)

#define BL1_ATFRAM0_OFFSET		0x1000

#define DDR_BASE			0x40000000

#define MMC_DESC_BASE			(DDR_BASE + 0x0080000)
#define MMC_DESC_SIZE			0x00020000
#define MMC_DATA_BASE			(MMC_DESC_BASE + MMC_DESC_SIZE)
#define MMC_DATA_SIZE			0x00800000

/*******************************************************************************
 * BL1 specific defines.
 * BL1 RW data is relocated from ROM to RAM at runtime so we need 2 base
 * addresses.
 ******************************************************************************/
#define BL1_RO_BASE			(ATFRAM0_BASE + BL1_ATFRAM0_OFFSET)
#define BL1_RO_LIMIT			(ATFRAM0_BASE + 0x10000)
#define BL1_RW_BASE			(BL1_RO_LIMIT)
#define BL1_RW_SIZE			(BL1_LIMIT - BL1_RW_BASE)
#define BL1_RW_LIMIT			(BL1_LIMIT)

/*******************************************************************************
 * BL2 specific defines.
 ******************************************************************************/
/* Set it in DDR first. If necessary, we can set them into SRAM again. */
#define BL2_BASE			(ATFRAM0_BASE + 0x100000)
#define BL2_LIMIT			(BL2_BASE + 0x80000)

/*******************************************************************************
 * BL3-1 specific defines.
 ******************************************************************************/
#ifdef BL31_ON_SRAM
#define BL31_BASE			(0xFFFF0000)
#define BL31_LIMIT			(0x100000000)

#define BL31_PT_BASE			(BL1_LIMIT)
#define BL31_PT_SIZE			0x6800
#else
#define BL31_BASE			(BL2_LIMIT)
#define BL31_LIMIT			(BL31_BASE + 0x40000)
#endif

/*******************************************************************************
 * BL3-2 specific defines.
 ******************************************************************************/

/*
 * The TSP can execute either from Trusted SRAM or Trusted DRAM.
 */
#define BL32_SRAM_BASE			BL31_LIMIT
#define BL32_SRAM_LIMIT			0 /*(BL32_SRAM_BASE + 0x0040000)*/

#define BL32_DRAM_BASE			DRAM_SEC_BASE
#define BL32_DRAM_LIMIT			(DRAM_SEC_BASE + DRAM_SEC_SIZE)

#if (PLAT_TSP_LOCATION_ID == PLAT_TRUSTED_SRAM_ID)
#define TSP_SEC_MEM_BASE		BL32_SRAM_BASE
#define TSP_SEC_MEM_SIZE		(BL32_SRAM_LIMIT - BL32_SRAM_BASE)
#define BL32_BASE			BL32_SRAM_BASE
#define BL32_LIMIT			BL32_SRAM_LIMIT
#elif (PLAT_TSP_LOCATION_ID == PLAT_TRUSTED_DRAM_ID)
#define TSP_SEC_MEM_BASE		BL32_DRAM_BASE
#define TSP_SEC_MEM_SIZE		(BL32_DRAM_LIMIT - BL32_DRAM_BASE)
#define BL32_BASE			BL32_DRAM_BASE
#define BL32_LIMIT			BL32_DRAM_LIMIT
#else
#error "Unsupported PLAT_TSP_LOCATION_ID value"
#endif


/*******************************************************************************
 * Load address of BL3-3
 ******************************************************************************/
#define NS_IMAGE_OFFSET			(DRAM_BASE + 0x3C00000)

/*******************************************************************************
 * Platform specific page table and MMU setup constants
 ******************************************************************************/
#define ADDR_SPACE_SIZE			(1ull << 32)

#if IMAGE_BL1 || IMAGE_BL31 || IMAGE_BL32
# define MAX_XLAT_TABLES		3
#endif

#if IMAGE_BL2
# define MAX_XLAT_TABLES		4
#endif

#define MAX_MMAP_REGIONS		16

/*******************************************************************************
 * Declarations and constants to access the mailboxes safely. Each mailbox is
 * aligned on the biggest cache line size in the platform. This is known only
 * to the platform as it might have a combination of integrated and external
 * caches. Such alignment ensures that two maiboxes do not sit on the same cache
 * line at any cache level. They could belong to different cpus/clusters &
 * get written while being protected by different locks causing corruption of
 * a valid mailbox address.
 ******************************************************************************/
#define CACHE_WRITEBACK_SHIFT   6
#define CACHE_WRITEBACK_GRANULE (1 << CACHE_WRITEBACK_SHIFT)

#endif /* __PLATFORM_DEF_H__ */
