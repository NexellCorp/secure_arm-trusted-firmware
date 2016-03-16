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
#define PLATFORM_STACK_SIZE 0x440
#elif IMAGE_BL2
#define PLATFORM_STACK_SIZE 0x400
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

/* Trusted Boot Firmware BL2 */
#define BL2_IMAGE_ID			1

/* SCP Firmware BL3-0 */
#define BL30_IMAGE_ID			2

/* EL3 Runtime Firmware BL31 */
#define BL31_IMAGE_ID			3

/* Secure Payload BL32 (Trusted OS) */
#define BL32_IMAGE_ID			4

/* Non-Trusted Firmware BL33 */
#define BL33_IMAGE_ID			5

/* Certificates */
#define BL2_CERT_ID			6
#define TRUSTED_KEY_CERT_ID		7

#define BL30_KEY_CERT_ID		8
#define BL31_KEY_CERT_ID		9
#define BL32_KEY_CERT_ID		10
#define BL33_KEY_CERT_ID		11

#define BL30_CERT_ID			12
#define BL31_CERT_ID			13
#define BL32_CERT_ID			14
#define BL33_CERT_ID			15

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

/*******************************************************************************
 * BL1 is stored in XG2RAM0_HIRQ that is 784KB large. Could we use 8MB size?
 * The first part is BL1_RAM, and the second part is TZRAM. The name isn't good
 * enough. We need to update it later.
 ******************************************************************************/
#define MMC_BASE			0x00000000
#define MMC_SIZE			0x80000000
#define MMC_LOADER_BASE			MMC_BASE		/* boot */
#define MMC_BL1_SIZE			0x00200000

#define ONCHIPROM_PARAM_BASE		(XG2RAM0_BASE + 0x700)
#define LOADER_RAM_BASE			(XG2RAM0_BASE + 0x800)
#define BL1_XG2RAM0_OFFSET		0x1000

#define DDR_BASE			0x41800000

#define MMC_DESC_BASE			(DDR_BASE + 0x0080000)
#define MMC_DESC_SIZE			0x00020000
#define MMC_DATA_BASE			(MMC_DESC_BASE + MMC_DESC_SIZE)
#define MMC_DATA_SIZE			0x00800000

/*******************************************************************************
 * BL1 specific defines.
 * BL1 RW data is relocated from ROM to RAM at runtime so we need 2 base
 * addresses.
 ******************************************************************************/
#define BL1_RO_BASE			(XG2RAM0_BASE + BL1_XG2RAM0_OFFSET)
#define BL1_RO_LIMIT			(XG2RAM0_BASE + 0x10000)
#define BL1_RW_BASE			(BL1_RO_LIMIT)
#define BL1_RW_SIZE			(BL31_LIMIT - BL1_RW_BASE)
#define BL1_RW_LIMIT			(BL31_LIMIT)

/*******************************************************************************
 * BL2 specific defines.
 ******************************************************************************/
/* Set it in DDR first. If necessary, we can set them into SRAM again. */
#define BL2_BASE			(BL1_RW_BASE + 0x8000)
#define BL2_LIMIT			(BL2_BASE + 0x40000)

/*******************************************************************************
 * BL3-1 specific defines.
 ******************************************************************************/
#define BL31_BASE			(BL2_LIMIT)
#define BL31_LIMIT			(BL31_BASE + 0x40000)

/*******************************************************************************
 * BL3-2 specific defines.
 ******************************************************************************/

/*
 * The TSP can execute either from Trusted SRAM or Trusted DRAM.
 */
#define BL32_SRAM_BASE			BL31_LIMIT
#define BL32_SRAM_LIMIT			(BL31_LIMIT+0x00080000)

#define BL32_DRAM_BASE			DRAM_SEC_BASE
#define BL32_DRAM_LIMIT			(DRAM_SEC_BASE+DRAM_SEC_SIZE)

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
 * BL3-0 specific defines:
 *
 * BL3-0 is loaded for mcu firmware, firstly load it into temperary buffer
 * into 0x0100_0000; then BL2 will parse the sections and load then into
 * separated buffers as needed.
 *
 ******************************************************************************/
#if 0
#define BL30_BASE			(DRAM_NS_BASE + 0x01000000)
#define BL30_LIMIT			(DRAM_NS_BASE + 0x01100000)
#define BL30_SIZE			(BL30_LIMIT - BL30_BASE)
#endif

/*******************************************************************************
 * Load address of BL3-3
 ******************************************************************************/
#define NS_IMAGE_OFFSET			(DRAM_BASE + 0x2400000)

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
