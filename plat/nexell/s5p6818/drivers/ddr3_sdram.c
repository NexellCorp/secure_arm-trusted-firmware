/*
 *	Copyright (C) 2012 Nexell Co., All Rights Reserved
 * 	Nexell Co. Proprietary & Confidential
 *
 *	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 *	AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 * 	BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR
 *	FITNESS
 *	FOR A PARTICULAR PURPOSE.
 *
 *	Module		: (DDR3) Memory Controller
 *	File		: ddr3_sdram.c
 *	Description	:
 *	Author		: Deoks
 *	History		:
 */

#include <string.h>
#include <mmio.h>
#include <platform.h>
#include <nx_s5p6818.h>
#include <drex.h>
#include <ddrphy.h>
#include <ddr3_sdram.h>

#include <secondboot.h>

#include <debug.h>


struct NX_SecondBootInfo *const pSBI =
    (struct NX_SecondBootInfo * const)HEADER_BASE;

struct s5p6818_drex_sdram_reg *const g_drex_reg =
    (struct s5p6818_drex_sdram_reg * const)PHY_BASEADDR_DREX_MODULE_CH0_APB;
struct s5p6818_ddrphy_reg *const g_ddrphy_reg =
    (struct s5p6818_ddrphy_reg * const)PHY_BASEADDR_DREX_MODULE_CH1_APB;
struct s5p6818_drex_tz_reg *const g_drextz_reg =
    (struct s5p6818_drex_tz_reg * const)PHY_BASEADDR_DREX_TZ_MODULE;

#define nop() __asm__ __volatile__("mov\tx0,x0\t\r\n nop\n\t");

extern void DMC_Delay(int milisecond);


void send_directcmd(SDRAM_CMD cmd, unsigned char chipnum, SDRAM_MODE_REG mrx, unsigned short value)
{
	mmio_write_32((uintptr_t)&g_drex_reg->DIRECTCMD, (unsigned int)((cmd << 24) |
		((chipnum & 1) << 20) | (mrx << 16) | value));
}

void enter_self_refresh(void)
{
	union SDRAM_MR MR;
	unsigned int nTemp;
	unsigned int nChips = 0;
	struct NX_SecondBootInfo SBI;

	int DDR3_CS_NUM, MR1_ODS, MR1_Rtt_Nom, MR1_nAL, nCWL;

	/* Alive power gate open */
	mmio_write_32(SCR_ALIVE_BASE, 1);

	/* Read to Memroy Information (Scratch)*/
	DDR3_CS_NUM = (mmio_read_32(SCR_USER_SIG6_READ) >> 0) & 0x3;
	MR1_ODS = (mmio_read_32(SCR_USER_SIG6_READ) >> 2) & 0x1;
	MR1_Rtt_Nom = (mmio_read_32(SCR_USER_SIG6_READ) >> 4) & 0x7;
	MR1_nAL	= (mmio_read_32(SCR_USER_SIG6_READ) >> 8 ) & 0xF;
	nCWL	= (mmio_read_32(SCR_USER_SIG6_READ) >> 12) & 0xF;

	memcpy(&SBI, pSBI, sizeof(struct NX_SecondBootInfo));

	if (DDR3_CS_NUM > 1)
		nChips = 0x3;
	else
		nChips = 0x1;

	while (mmio_read_32((uintptr_t)&g_drex_reg->CHIPSTATUS) & 0xF)
		nop();

	/* Step 01. Send PALL Command */
	send_directcmd(SDRAM_CMD_PALL, 0, (SDRAM_MODE_REG)0, 0);
	if (DDR3_CS_NUM > 1)
		send_directcmd(SDRAM_CMD_PALL, 1, (SDRAM_MODE_REG)0, 0);

	DMC_Delay(100);

	/* Step 02. (DRAM) ODT OFF */
	MR.Reg = 0;
	MR.MR2.RTT_WR = 0;							// 0: disable, 1: RZQ/4 (60ohm), 2: RZQ/2 (120ohm)
	MR.MR2.SRT = 0;								// self refresh normal range, if (ASR == 1) SRT = 0;
	MR.MR2.ASR = 1;								// auto self refresh enable
	MR.MR2.CWL = (nCWL - 5);
	send_directcmd(SDRAM_CMD_MRS, 0, SDRAM_MODE_REG_MR2, MR.Reg);
	if (DDR3_CS_NUM > 1)
		send_directcmd(SDRAM_CMD_MRS, 1, SDRAM_MODE_REG_MR2, MR.Reg);

	/* Step 03. (DRAM) Set the Drive Strength */
	MR.Reg = 0;
	MR.MR1.DLL = 1;								// 0: Enable, 1 : Disable
	MR.MR1.AL = MR1_nAL;
	MR.MR1.ODS1 = (MR1_ODS >> 1) & 1;
	MR.MR1.ODS0 = (MR1_ODS >> 0) & 1;
	MR.MR1.RTT_Nom2 = (MR1_Rtt_Nom >> 2) & 1;
	MR.MR1.RTT_Nom1 = (MR1_Rtt_Nom >> 1) & 1;
	MR.MR1.RTT_Nom0 = (MR1_Rtt_Nom >> 0) & 1;
	MR.MR1.QOff = 0;
	MR.MR1.WL = 0;

#if 0
	MR.MR1.TDQS     = (_DDR3_BUS_WIDTH>>3) & 1;
#endif

	send_directcmd(SDRAM_CMD_MRS, 0, SDRAM_MODE_REG_MR1, MR.Reg);
	if (DDR3_CS_NUM > 1)
		send_directcmd(SDRAM_CMD_MRS, 1, SDRAM_MODE_REG_MR1, MR.Reg);

	/* Step 04. Enter Self-Refresh Command */
	send_directcmd(SDRAM_CMD_REFS, 0, (SDRAM_MODE_REG)0, 0);
	if (DDR3_CS_NUM > 1)
		send_directcmd(SDRAM_CMD_REFS, 1, (SDRAM_MODE_REG)0, 0);

	/*  Step 05. Check the Busy State */
	do {
		nTemp = (mmio_read_32((uintptr_t)&g_drex_reg->CHIPSTATUS) & nChips);
	} while (nTemp);

	/* Step 06. Check the Sel-Refresh State (FSM) */
	do {
		nTemp = ((mmio_read_32((uintptr_t)&g_drex_reg->CHIPSTATUS) >> 8) & nChips);
	} while (nTemp != nChips);
#if 0
	/* Step 05. Disable the Auto refresh Counter */
	mmio_clrbits_32((uintptr_t)&g_drex_reg->CONCONTROL, (0x1 << 5));			// afre_en[5]. Auto Refresh Counter. Disable:0, Enable:1
#endif
	/* Step  06. Disable the Dynamic Clock */
	mmio_setbits_32((uintptr_t)&g_drex_reg->MEMCONTROL, (0x1 << 0));			// clk_stop_en[0] : Dynamic Clock Control   :: 1'b0 - Always running
}
