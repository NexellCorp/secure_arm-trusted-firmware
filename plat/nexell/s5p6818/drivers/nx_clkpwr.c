/*
 * Copyright (C) 2012 Nexell Co., All Rights Reserved
 * Nexell Co. Proprietary & Confidential
 *
 * NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 * AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 * FOR A PARTICULAR PURPOSE.
 *
 * Module          : Clock & Power Manager
 * File            : nx_clkpwr.c
 * Description     :
 * Author          : hans
 * History         : 2016.05.16 hans create
 */

#include <nx_s5p6818.h>
#include <nx_clkpwr.h>

static const unsigned int __osc_khz = 24000;

unsigned int nx_clkpwr_getpllfreq(unsigned int pllnumber)
{
	const unsigned int pll_pdiv_bit_pos =    18;
	const unsigned int pll_mdiv_bit_pos =    8;
	const unsigned int pll_sdiv_bit_pos =    0;
	const unsigned int pll_kdiv_bit_pos =    16;
	register unsigned int regvalue, regvalue1, np, nm, ns, nk;
	unsigned int temp = 0;
	volatile struct nx_clkpwr_registerset *pclkpwr =
		(struct nx_clkpwr_registerset *)PHY_BASEADDR_CLKPWR_MODULE;

	regvalue = pclkpwr->pllsetreg[pllnumber];
	np = (regvalue >> pll_pdiv_bit_pos) & 0x3f;
	nm = (regvalue >> pll_mdiv_bit_pos) & 0x3ff;
	ns = (regvalue >> pll_sdiv_bit_pos) & 0xff;
	regvalue1 = pclkpwr->pllsetreg_sscg[pllnumber];
	nk = (regvalue1 >> pll_kdiv_bit_pos) & 0xffff;

	if ((pllnumber > 1) && nk) {
		temp = (unsigned int)(((((nk * 1000) / 65536) *
					__osc_khz) / np) >> ns);
	}

	temp = (unsigned int)((((( nm * __osc_khz)/np)>>ns) * 1000) + temp);
	return temp;
}
