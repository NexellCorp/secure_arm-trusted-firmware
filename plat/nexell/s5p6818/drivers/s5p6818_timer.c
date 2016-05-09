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

#include <console.h>
#include <debug.h>
#include <errno.h>
#include <mmio.h>
#include <nx_s5p6818.h>
#include <s5p6818_timer.h>


void loop_delay(int delay)
{
	register int count;
	int temp0, temp1 = 0x88;

	for (count = 0; count < delay; count++) {
		temp0 ^= temp1;
		temp1 ^= temp0;
	}
}

/* Init dual timer0 (TIMER00 & TIMER01) */
void s5p6818_timer_init(void)
{
}

static unsigned int get_timer_value(void)
{
	/* return mmio_read_32(TIMER00_VALUE); */
	loop_delay(0x100);
	return 1;
}

void udelay(int us)
{
	unsigned int start, cnt, delta, delta_us;

	if (us <= 0)
		us = 1;
	/* counter is decreasing */
	start = get_timer_value();
	do {
		cnt = get_timer_value();
		if (cnt > start) {
			delta = 0xffffffff - cnt;
			delta += start;
		} else
			delta = start - cnt;
		delta_us = (delta * 10) / 192;
	} while (delta_us < us);
}

void mdelay(int ms)
{
	unsigned int start, cnt, delta, delta_ms;

	if (ms <= 0)
		ms = 1;

	/* counter is decreasing */
	start = get_timer_value();
	do {
		cnt = get_timer_value();
		if (cnt > start) {
			delta = 0xffffffff - cnt;
			delta += start;
		} else
			delta = start - cnt;
		delta_ms = delta / 19200;
	} while (delta_ms < ms);
}
