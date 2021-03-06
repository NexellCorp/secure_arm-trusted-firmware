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

#include <arch.h>
#include <asm_macros.S>
#include <bl_common.h>
#include <cortex_a53.h>
#include <cpu_macros.S>
#include <platform_def.h>
#include <s5p6818_def.h>

	/* Global functions */
	.weak	plat_calc_core_pos
	.weak	plat_my_core_pos
	.globl	plat_crash_console_init
	.globl	plat_crash_console_putc
	.globl	plat_report_exception
	.globl	plat_reset_handler
	.globl	platform_get_core_pos
	.globl	platform_mem_init

	/* Global variables */
	.globl	sec_entry_point

	/* Define a crash console for the plaform */
#define S5P6818_CRASH_CONSOLE_BASE		CONSOLE_UART_BASE

	/* -----------------------------------------------------
	 *  unsigned int plat_my_core_pos(void)
	 *  This function uses the plat_arm_calc_core_pos()
	 *  definition to get the index of the calling CPU.
	 * -----------------------------------------------------
	 */
func plat_my_core_pos
	mrs	x0, mpidr_el1
	b	plat_calc_core_pos
endfunc plat_my_core_pos

.global plat_reg_delay
func plat_reg_delay
1:
	sub	x0, x0, #1
	cmp x0, #0
	bne	1b
	ret
endfunc plat_reg_delay

	/* -----------------------------------------------------
	 *  unsigned int plat_arm_calc_core_pos(uint64_t mpidr)
	 *  Helper function to calculate the core position.
	 *  With this function: CorePos = (ClusterId * 4) +
	 *  				  CoreId
	 * -----------------------------------------------------
	 */
func plat_calc_core_pos
	and	x1, x0, #MPIDR_CPU_MASK
	and	x0, x0, #MPIDR_CLUSTER_MASK
	add	x0, x1, x0, LSR #6
	ret
endfunc plat_calc_core_pos

	/* ---------------------------------------------
	 * int plat_crash_console_init(void)
	 * Function to initialize the crash console
	 * without a C Runtime to print crash report.
	 * Clobber list : x0, x1, x2
	 * ---------------------------------------------
	 */
func plat_crash_console_init
	mov_imm	x0, S5P6818_CRASH_CONSOLE_BASE
	mov_imm	x1, NXP_UART_CLK_IN_HZ
	mov_imm	x2, NXP_BAUDRATE
	b	console_core_init
endfunc plat_crash_console_init

	/* ---------------------------------------------
	 * int plat_crash_console_putc(int c)
	 * Function to print a character on the crash
	 * console without a C Runtime.
	 * Clobber list : x1, x2
	 * ---------------------------------------------
	 */
func plat_crash_console_putc
	mov_imm	x1, S5P6818_CRASH_CONSOLE_BASE
	b	console_core_putc
endfunc plat_crash_console_putc

	/* ---------------------------------------------
	 * void plat_report_exception(unsigned int type)
	 * Function to report an unhandled exception
	 * with platform-specific means.
	 * On NEXELL platform, it updates the LEDs
	 * to indicate where we are
	 * ---------------------------------------------
	 */
func plat_report_exception
	mov	x8, x30

	/* Turn on LED according to x0 (0 -- f) */
#if 0
	ldr	x2, =0xf7020000
	and	x1, x0, #1
	str	w1, [x2, #4]
	and	x1, x0, #2
	str	w1, [x2, #8]
	and	x1, x0, #4
	str	w1, [x2, #16]
	and	x1, x0, #8
	str	w1, [x2, #32]
#endif

	adr	x4, plat_err_str
	bl	asm_print_str

	adr	x4, esr_el3_str
	bl	asm_print_str

	mrs	x4, esr_el3
	bl	asm_print_hex

	adr	x4, elr_el3_str
	bl	asm_print_str

	mrs	x4, elr_el3
	bl	asm_print_hex

	mov	x30, x8
	ret
endfunc plat_report_exception

	/* -----------------------------------------------------
	 * void plat_reset_handler(void);
	 *
	 * Implement workaround for defect id 831273 by enabling
	 * an event stream every 65536 cycles and set the L2 RAM
	 * latencies for Cortex-A57.
	 * -----------------------------------------------------
	 */
func plat_reset_handler
	/* In juno, it sets the latency of L2 Data and Tag. How about nexell? */
	/* Do anything just after reset. At here, do we need? */
	ret
endfunc plat_reset_handler

	/*
	 * Return 0 to 7
	 */
func platform_get_core_pos
	and	x1, x0, #MPIDR_CPU_MASK
	and	x0, x0, #MPIDR_CLUSTER_MASK
	add	x0, x1, x0, LSR #6
	ret
endfunc platform_get_core_pos

	/* -----------------------------------------------------
	 * void platform_mem_init(void);
	 *
	 * We don't need to carry out any memory initialization
	 * on NEXELL. The Secure RAM is accessible straight away.
	 * -----------------------------------------------------
	 */
func platform_mem_init
	ret
endfunc platform_mem_init

.section .rodata.rev_err_str, "aS"
plat_err_str:
	.asciz "\nPlatform exception reporting:"
esr_el3_str:
	.asciz "\nESR_EL3: "
elr_el3_str:
	.asciz "\nELR_EL3: "
