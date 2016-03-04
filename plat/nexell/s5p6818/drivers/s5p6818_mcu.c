/*
 * Copyright (c) 2014-2015, Linaro Ltd and Contributors. All rights reserved.
 * Copyright (c) 2014-2015, Hisilicon Ltd and Contributors. All rights reserved.
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
#include <assert.h>
#include <bl_common.h>
#include <console.h>
#include <debug.h>
#include <partitions.h>
#include <platform.h>
#include <platform_def.h>
#include <string.h>
#include <mmio.h>

#define MCU_SECTION_MAX		30

enum MCU_IMAGE_SEC_TYPE_ENUM {
	MCU_IMAGE_SEC_TYPE_TEXT = 0,	/* text section */
	MCU_IMAGE_SEC_TYPE_DATA,	/* data section */
	MCU_IMAGE_SEC_TYPE_BUTT
};

enum MCU_IMAGE_SEC_LOAD_ENUM {
	MCU_IMAGE_SEC_LOAD_STATIC = 0,
	MCU_IMAGE_SEC_LOAD_DYNAMIC,
	MCU_IMAGE_SEC_LOAD_BUFFER,
	MCU_IMAGE_SEC_LOAD_MODEM_ENTRY,
	MCU_IMAGE_SEC_LOAD_BUTT
};

struct mcu_image_sec {
	unsigned short serial;
	char type;
	char load_attr;
	uint32_t src_offset;		/* offset in image */
	uint32_t dst_offset;		/* offset in memory */
	uint32_t size;
};

struct mcu_image_head {
	char time_stamp[24];
	uint32_t image_size;
	uint32_t secs_num;
	struct mcu_image_sec secs[MCU_SECTION_MAX];
};

#define SOC_SRAM_M3_BASE_ADDR		(0xF6000000)

#define MCU_SRAM_SIZE			(0x0000C000)
#define MCU_CACHE_SIZE			(0x00004000)
#define MCU_CODE_SIZE			(MCU_SRAM_SIZE - MCU_CACHE_SIZE)

#define MCU_SYS_MEM_ADDR		(0x05E00000)
#define MCU_SYS_MEM_SIZE		(0x00100000)

static uint32_t mcu2ap_addr(uint32_t mcu_addr)
{
	if (mcu_addr < MCU_CODE_SIZE)
		return (mcu_addr + SOC_SRAM_M3_BASE_ADDR);
	else if ((mcu_addr >= MCU_SRAM_SIZE) &&
		 (mcu_addr < MCU_SRAM_SIZE + MCU_SYS_MEM_SIZE))
		return mcu_addr - MCU_SRAM_SIZE + MCU_SYS_MEM_ADDR;
	else
		return mcu_addr;
}

static int is_binary_header_invalid(struct mcu_image_head *head,
				    unsigned length)
{
	/* invalid cases */
	if ((head->image_size == 0) ||
	    (head->image_size > length) ||
	    (head->secs_num > MCU_SECTION_MAX) ||
	    (head->secs_num == 0))
		return 1;

	return 0;
}

static int is_binary_section_invalid(struct mcu_image_sec *sec,
				     struct mcu_image_head *head)
{
	unsigned long ap_dst_offset = 0;

	if ((sec->serial >= head->secs_num) ||
	    (sec->src_offset + sec->size > head->image_size))
		return 1;

	if ((sec->type >= MCU_IMAGE_SEC_TYPE_BUTT) ||
	    (sec->load_attr >= MCU_IMAGE_SEC_LOAD_BUTT))
		return 1;

	ap_dst_offset = mcu2ap_addr(sec->dst_offset);
	if ((ap_dst_offset >= SOC_SRAM_M3_BASE_ADDR) &&
	    (ap_dst_offset < SOC_SRAM_M3_BASE_ADDR + 0x20000 - sec->size))
		return 0;
	else if ((ap_dst_offset >= MCU_SYS_MEM_ADDR) &&
	    (ap_dst_offset < MCU_SYS_MEM_ADDR + MCU_SYS_MEM_SIZE - sec->size))
		return 0;
	else if ((ap_dst_offset >= 0xfff8e000) &&
			(ap_dst_offset < 0xfff91c00 - sec->size))
		return 0;

	ERROR("%s: mcu destination address invalid.\n", __func__);
	ERROR("%s: number=%d, dst offset=%d size=%d\n",
		__func__, sec->serial, sec->dst_offset, sec->size);
	return 1;
}
