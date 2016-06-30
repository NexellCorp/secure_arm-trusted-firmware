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

#include <mmio.h>
#include <nx_s5p6818.h>
#include <nx_clkpwr.h>
#include <nx_alive.h>

inline void DMC_Delay(int milisecond)
{
	int count, temp;

	for (count = 0; count < milisecond; count++) {
		temp ^= count;
	}
}

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

void vdd_power_off(void)
{
	volatile struct nx_clkpwr_registerset *pclkpwr =
		(struct nx_clkpwr_registerset *)PHY_BASEADDR_CLKPWR_MODULE;

	volatile struct nx_alive_registerset *palive =
		(struct nx_alive_registerset *)PHY_BASEADDR_ALIVE_MODULE;

	/* clear for reset issue. */
	mmio_write_32((uintptr_t)&pclkpwr->cpuwarmresetreq, 0x0);

	/* Clear USE_WFI & USE_WFE bits for STOP mode. */
	/* mmio_clrbits_32((uintptr_t) &pclkpwr->PWRCONT, (0x3FFFF << 8)); */
	/* stop mode needs all cpu wfi */
	mmio_setbits_32((uintptr_t)&pclkpwr->pwrcont, 0xF << 12 | 0xF << 20);
	/* stop mode does not need all cpu wfe */
	mmio_clrbits_32((uintptr_t)&pclkpwr->pwrcont, 0xF << 8 | 0xF << 16);

#if (MULTICORE_SLEEP_CONTROL == 1)
	/* alive power gate open */
	mmio_write_32((uintptr_t)&palive->alivepwrgatereg, 0x00000001);

	/*----------------------------------
	 * Save leveling & training values.
	 */
#if 1
	/* clear - ctrl_shiftc */
	mmio_write_32((uintptr_t)&palive->alivescratchrst5, 0xFFFFFFFF);
	/* clear - ctrl_offsetC */
	mmio_write_32((uintptr_t)&palive->alivescratchrst6, 0xFFFFFFFF);
	/* clear - ctrl_offsetr */
	mmio_write_32((uintptr_t)&palive->alivescratchrst7, 0xFFFFFFFF);
	/* clear - ctrl_offsetw */
	mmio_write_32((uintptr_t)&palive->alivescratchrst8, 0xFFFFFFFF);

	/* store - ctrl_shiftc */
	mmio_write_32((uintptr_t)&palive->alivescratchset5, g_GT_cycle);
	/* store - ctrl_offsetc */
	mmio_write_32((uintptr_t)&palive->alivescratchset6, g_GT_code);
	/* store - ctrl_offsetr */
	mmio_write_32((uintptr_t)&palive->alivescratchset7, g_RD_vwmc);
	/* store - ctrl_offsetw */
	mmio_write_32((uintptr_t)&palive->alivescratchset8, g_WR_vwmc);
#endif

	/* clear delay counter, reference rtc clock */
	mmio_write_32((uintptr_t)&palive->vddoffcntvaluerst, 0xFFFFFFFF);
	/* set minimum delay time for VDDPWRON pin.
	 * 1cycle per 32.768Kh (about 30us)
	 */
	mmio_write_32((uintptr_t)&palive->vddoffcntvalueset, 0x00000001);

	if (mmio_read_32((uintptr_t)&palive->alivegpiodetectpendreg)) {
		/* now real entering point to stop mode. */
		__asm__ __volatile__("wfi");
	} else {
		/* Retention off (Pad hold off) */
		mmio_write_32((uintptr_t)&palive->vddctrlsetreg, 0x000003FC);
		/* vddpoweron off, start counting down. */
		mmio_write_32((uintptr_t)&palive->vddctrlrstreg, 0x00000001);

		/* all alive pend pending clear until power down. */
		mmio_write_32((uintptr_t)&palive->alivegpiodetectpendreg, 0xFF);
		/* 600 : 110us, Delay for Pending Clear. When
		 * CPU clock is 400MHz, this value is minimum delay value.
		 */
		DMC_Delay(600);

		/* alive power gate close */
		/* mmio_write_32((uintptr_t)&palive->ALIVEPWRGATEREG, 0x00000000); */

		while (1) {
			/* enter STOP mode. */
			mmio_write_32((uintptr_t)&pclkpwr->pwrmode, (0x1 << 1));
			/* now real entering point to stop mode. */
			__asm__ __volatile__("wfi");
		}
	}

#else  /* #if (MULTICORE_SLEEP_CONTROL == 1) */

	/* alive power gate open */
	mmio_write_32((uintptr_t)&palive->alivepwrgatereg, 0x00000001);

	/* clear delay counter, reference rtc clock */
	mmio_write_32((uintptr_t)&palive->vddoffcntvaluerst, 0xFFFFFFFF);
	/* set minimum delay time for VDDPWRON pin.
	 * 1cycle per 32.768Kh (about 30us)
	 */
	mmio_write_32((uintptr_t)&palive->vddoffcntvalueset, 0x00000001);

	/* vddpoweron off, start counting down. */
	mmio_write_32((uintptr_t)&palive->vddctrlrstreg, 0x00000001);
	DMC_Delay(220);

	/* all alive pend pending clear until power down. */
	mmio_write_32((uintptr_t)&palive->alivegpiodetectpendreg, 0xFF);
	/* alive power gate close */
	mmio_write_32((uintptr_t)&palive->alivepwrgatereg, 0x00000000);

	/* clear for reset issue. */
	mmio_write_32((uintptr_t)&pclkpwr->cpuwarmresetreq, 0x0);

	while (1) {
		/* enter STOP mode. */
		mmio_write_32((uintptr_t)&pclkpwr->pwrmode, (0x1 << 1));
		/* now real entering point to stop mode. */
		__asm__ __volatile__("wfi");
	}
#endif /* #if (MULTICORE_SLEEP_CONTROL == 1) */
}
