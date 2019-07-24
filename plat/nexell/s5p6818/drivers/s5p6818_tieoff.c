/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: Youngbok, Park <ybparkle@nexell.co.kr>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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
#include <s5p6818_tieoff.h>
#include <io.h>

#define	NX_PIN_FN_SIZE	4
#define TIEOFF_REG_NUM 33

struct	nx_tieoff_registerset {
	uint32_t	tieoffreg[TIEOFF_REG_NUM];
};

static struct nx_tieoff_registerset *nx_tieoff = (void *)0xC0011000;


void nx_tieoff_set(uint32_t tieoff_index, uint32_t tieoff_value)
{
	uint32_t regindex, mask;
	uint32_t lsb, msb;
	uint32_t regval;

	uint32_t position;
	uint32_t bitwidth;

	position = tieoff_index & 0xffff;
	bitwidth = (tieoff_index>>16) & 0xffff;

	regindex	= position>>5;

	lsb = position & 0x1F;
	msb = lsb+bitwidth;

	if (msb > 32) {
		msb &= 0x1F;
		mask   = ~(0xffffffff<<lsb);
		regval = read32((vaddr_t)&nx_tieoff->tieoffreg[regindex])
			& mask;
		regval |= ((tieoff_value & ((1UL<<bitwidth)-1))<<lsb);
		write32(regval, (vaddr_t)&nx_tieoff->tieoffreg[regindex]);

		mask   = (0xffffffff<<msb);
		regval = read32((vaddr_t)&nx_tieoff->tieoffreg[regindex+1])
			& mask;
		regval |= ((tieoff_value & ((1UL<<bitwidth)-1))>>msb);
		write32(regval, (vaddr_t)&nx_tieoff->tieoffreg[regindex+1]);
	} else	{
		mask	= (0xffffffff<<msb) | (~(0xffffffff<<lsb));
		regval	= read32((vaddr_t)&nx_tieoff->tieoffreg[regindex])
			& mask;
		regval	|= ((tieoff_value & ((1UL<<bitwidth)-1))<<lsb);
		write32(regval, (vaddr_t)&nx_tieoff->tieoffreg[regindex]);
	}
}

uint32_t nx_tieoff_get(uint32_t tieoff_index)
{
	uint32_t regindex, mask;
	uint32_t lsb, msb;
	uint32_t regval;

	uint32_t position;
	uint32_t bitwidth;

	position = tieoff_index & 0xffff;
	bitwidth = (tieoff_index>>16) & 0xffff;

	regindex = position/32;
	lsb = position % 32;
	msb = lsb+bitwidth;

	if (msb > 32) {
		msb &= 0x1F;
		mask   = 0xffffffff<<lsb;
		regval = read32((vaddr_t)&nx_tieoff->tieoffreg[regindex])
			& mask;
		regval >>= lsb;

		mask   = ~(0xffffffff<<msb);
		regval |= ((read32((vaddr_t)&nx_tieoff->tieoffreg[regindex+1])
			    & mask)
			  << (32-lsb));
	} else	{
		mask   = ~(0xffffffff<<msb) & (0xffffffff<<lsb);
		regval = read32((vaddr_t)&nx_tieoff->tieoffreg[regindex])
			& mask;
		regval >>= lsb;
	}
	return regval;
}
