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
/* hardware dependency */
#define DEVICE_SD	0
#define DEVICE_MMC	2

/* The size of DRAM is 1GB as default. */
/*unused: #define ATFRAM0_SIZE			0x200000*/
#define DRAM_BASE			0x40000000

#if defined(PLAT_DRAM_SIZE) && PLAT_DRAM_SIZE == 2048
#define ATFRAM0_BASE			0xbfe00000
#define DRAM_SIZE			0x7fb00000UL
#define BL1_LIMIT			0xbffe8000
#define OFFSET_FIXUP			0x80000000
#elif defined(PLAT_DRAM_SIZE) && PLAT_DRAM_SIZE == 512
#define ATFRAM0_BASE			0x5fe00000
#define DRAM_SIZE			0x1fb00000UL
#define BL1_LIMIT			0x5ffe8000
#define OFFSET_FIXUP			0x20000000
#else
#define ATFRAM0_BASE			0x7fe00000
#define DRAM_SIZE			0x3fb00000UL
#define BL1_LIMIT			0x7ffe8000
#define OFFSET_FIXUP			0x40000000
#endif /* PLAT_DRAM_SIZE */

#define FLASH_LOADER_BASE		(OFFSET_FIXUP + 0x3fdce000)
#define FLASH_LOADER_SIZE		0x00042000

#define FLASH_SECURE_BASE		(OFFSET_FIXUP + 0x3fd00000)
#define FLASH_SECURE_SIZE		0x000c0000

#define FLASH_NONSECURE_BASE		(OFFSET_FIXUP + 0x3fb00000)
#define FLASH_NONSECURE_SIZE		0x00200000



#define PLAT_TRUSTED_SRAM_ID		0
#define PLAT_TRUSTED_DRAM_ID		1

/*
 * DRAM at 0x4000_0000 is divided in two regions:
 *   - Secure DRAM (default is the top 16MB)
 *   - Non-Secure DRAM (remaining DRAM starting at DRAM_BASE)
 */
#define DRAM_SEC_SIZE			0x02000000
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
/* arm secure policy assign to non-secure this sgi interrupt,
 * but linux does not use this interrupt number (use 0~3).
 * so I'll use this sgi for user.
 */
#define IRQ_SEC_SGI_8			7

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
#define CCI400_BASE			0xE0090000
#define CCI_CLUSTER0_SL_IFACE_IX	4
#define CCI_CLUSTER1_SL_IFACE_IX	3

/*******************************************************************************
 * ALIVE SCRATCH
 ******************************************************************************/
#define PHY_BASEADDR_ALIVE		(0xC0010800)

#define	SCR_ALIVE_BASE			(PHY_BASEADDR_ALIVE)
#define	SCR_SIGNATURE_RESET		(SCR_ALIVE_BASE + 0x068)
#define	SCR_SIGNATURE_SET		(SCR_ALIVE_BASE + 0x06C)
#define	SCR_SIGNATURE_READ		(SCR_ALIVE_BASE + 0x070)
/* ALIVESCRATCHRST1 */
#define	SCR_WAKE_FN_RESET		(SCR_ALIVE_BASE + 0x0AC)
#define	SCR_WAKE_FN_SET			(SCR_ALIVE_BASE + 0x0B0)
#define	SCR_WAKE_FN_READ		(SCR_ALIVE_BASE + 0x0B4)
/* ALIVESCRATCHRST2 */
#define	SCR_NEXT_HDR_RESET		(SCR_ALIVE_BASE + 0x0B8)
#define	SCR_NEXT_HDR_SET		(SCR_ALIVE_BASE + 0x0BC)
#define	SCR_NEXT_HDR_READ		(SCR_ALIVE_BASE + 0x0C0)
/* ALIVESCRATCHRST3 */
#define	SCR_CRC_PHY_RESET		(SCR_ALIVE_BASE + 0x0C4)
#define	SCR_CRC_PHY_SET			(SCR_ALIVE_BASE + 0x0C8)
#define	SCR_CRC_PHY_READ		(SCR_ALIVE_BASE + 0x0CC)
/* ALIVESCRATCHRST4 */
#define	SCR_CRC_LEN_RESET		(SCR_ALIVE_BASE + 0x0D0)
#define	SCR_CRC_LEN_SET			(SCR_ALIVE_BASE + 0x0D4)
#define	SCR_CRC_LEN_READ		(SCR_ALIVE_BASE + 0x0D8)
/* ALIVESCRATCHRST5 */
#define	SCR_RESET_SIG_RESET		(SCR_ALIVE_BASE + 0x0DC)
#define	SCR_RESET_SIG_SET		(SCR_ALIVE_BASE + 0x0E0)
#define	SCR_RESET_SIG_READ		(SCR_ALIVE_BASE + 0x0E4)
/* ALIVESCRATCHRST6 */
#define	SCR_USER_SIG6_RESET		(SCR_ALIVE_BASE + 0x0E8)
#define	SCR_USER_SIG6_SET		(SCR_ALIVE_BASE + 0x0EC)
#define	SCR_USER_SIG6_READ		(SCR_ALIVE_BASE + 0x0F0)
/* ALIVESCRATCHRST7 */
#define	SCR_USER_SIG7_RESET		(SCR_ALIVE_BASE + 0x0F4)
#define	SCR_USER_SIG7_SET		(SCR_ALIVE_BASE + 0x0F8)
#define	SCR_USER_SIG7_READ		(SCR_ALIVE_BASE + 0x0FC)
/* ALIVESCRATCHRST8 */
#define	SCR_USER_SIG8_RESET		(SCR_ALIVE_BASE + 0x100)
#define	SCR_USER_SIG8_SET		(SCR_ALIVE_BASE + 0x104)
#define	SCR_USER_SIG8_READ		(SCR_ALIVE_BASE + 0x108)

#define SUSPEND_SIGNATURE		(0x41544600)	/* (ASCII) : "ATF" */
#define	SUSPEND_SAVE_SIZE		(128*1024)	/* (_etext - _stext) */
#define RECOVERY_SIGNATURE		(0x52455343)	/* (ASCII) : R.E.S.C */
#define USBBOOT_SIGNATURE		(0x85836666)	/* (ASCII) : U.S.B.B */


#endif /* __S5P6818_DEF_H__ */
