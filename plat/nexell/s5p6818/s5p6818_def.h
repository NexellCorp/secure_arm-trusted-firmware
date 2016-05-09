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

#ifndef __S5P6818_DEF_H__
#define __S5P6818_DEF_H__

#define DEVICE_BASE			0xC0000000	/* Peripheral base */
#define DEVICE_SIZE			0x3F000000

/*******************************************************************************
 * Nexell memory map related constants
 ******************************************************************************/
/*
 * S5P6818 DRAM memory layout
 * +-----------------------------+ 0x40000000 + DRAM_SIZE
 * | ARM Trusted Firmware        |
 * +----------------------(24MB)-+
 * |        FIP save image       |
 * +-----------------------------+ 0x40000000 + DRAM_SIZE - 2MB
 * |                             |
 * |   Normal Memory             |
 * |                             |
 * +-----------------------------+ 0x40000000
 *
 *  * FIP will be removed as the loader reads direct from storage.
 */
#define FLASH_BASE			0x40000000
#define FLASH_SIZE			0x01800000

/* The size of DRAM is 1GB as default. */
#define XG2RAM0_SIZE			0x200000
#define DRAM_BASE			0x41800000

#if defined(PLAT_DRAM_SIZE) && PLAT_DRAM_SIZE == 2048
#define XG2RAM0_BASE			0xbfe00000
#define DRAM_SIZE			0x7e600000UL
#define BL1_LIMIT			0xbfe98000
#elif defined(PLAT_DRAM_SIZE) && PLAT_DRAM_SIZE == 512
#define XG2RAM0_BASE			0x5fe00000
#define DRAM_SIZE			0x1e600000UL
#define BL1_LIMIT			0x5fe98000
#else
#define XG2RAM0_BASE			0x7fe00000
#define DRAM_SIZE			0x3e600000UL
#define BL1_LIMIT			0x7fe98000
#endif /* PLAT_DRAM_SIZE */

#define PLAT_TRUSTED_SRAM_ID		0
#define PLAT_TRUSTED_DRAM_ID		1

/*
 * DRAM at 0x4000_0000 is divided in two regions:
 *   - Secure DRAM (default is the top 16MB)
 *   - Non-Secure DRAM (remaining DRAM starting at DRAM_BASE)
 */
#define DRAM_SEC_SIZE			0x01000000
#define DRAM_SEC_BASE			(DRAM_BASE + DRAM_SIZE - DRAM_SEC_SIZE)

#define DRAM_NS_BASE			DRAM_BASE
#define DRAM_NS_SIZE			(DRAM_SIZE - DRAM_SEC_SIZE)

#define SRAM_BASE			0xFFFF0000
#define SRAM_SIZE			0x00010000

/*******************************************************************************
 * System counter frequency related constants
 ******************************************************************************/
#define SYS_COUNTER_FREQ_IN_TICKS	12000000
#define SYS_COUNTER_FREQ_IN_MHZ		12

/*******************************************************************************
 * GIC-400 & interrupt handling related constants
 ******************************************************************************/
#define GICD_BASE			0xC0009000
#define GICC_BASE			0xC000A000

#define IRQ_SEC_PHY_TIMER		29
#define IRQ_SEC_SGI_0			8
#define IRQ_SEC_SGI_1			9
#define IRQ_SEC_SGI_2			10
#define IRQ_SEC_SGI_3			11
#define IRQ_SEC_SGI_4			12
#define IRQ_SEC_SGI_5			13
#define IRQ_SEC_SGI_6			14
#define IRQ_SEC_SGI_7			15
#define IRQ_SEC_SGI_8			16

/*******************************************************************************
 * NXP_UART related constants
 ******************************************************************************/
#define UART0_BASE			0xC00A1000
#define UART1_BASE			0xC00A0000
#define UART2_BASE			0xC00A2000
#define UART3_BASE			0xC00A3000
#define UART4_BASE			0xC006D000
#define UART5_BASE			0xC006F000

#ifdef PLAT_UART_BASE
#define CONSOLE_UART_BASE		(PLAT_UART_BASE)
#else
#define CONSOLE_UART_BASE		(UART3_BASE)
#endif

#define NXP_BAUDRATE			115200
#define NXP_UART_CLK_IN_HZ		19200000

/*******************************************************************************
 * CCI-400 related constants
 ******************************************************************************/
#define CCI400_BASE				0xE0090000
#if 0
#define CCI400_SL_IFACE3_CLUSTER1_IX	1
#define CCI400_SL_IFACE4_CLUSTER0_IX	0
#else
#define CCI_CLUSTER0_SL_IFACE_IX	3
#define CCI_CLUSTER1_SL_IFACE_IX	4
#endif
#endif /* __S5P6818_DEF_H__ */
