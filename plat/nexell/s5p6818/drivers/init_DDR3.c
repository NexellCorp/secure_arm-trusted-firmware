/*
 *      Copyright (C) 2012 Nexell Co., All Rights Reserved
 *      Nexell Co. Proprietary & Confidential
 *
 *      NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 *      AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 *      BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR
 *FITNESS
 *      FOR A PARTICULAR PURPOSE.
 *
 *      Module          : DDR3
 *      File            : init_DDR3.c
 *      Description     :
 *      Author          : Kook
 *      History         :
 */

#include <string.h>
#include <mmio.h>
#include <platform.h>
#include <nx_s5p6818.h>
#include <nx_drex.h>
#include <nx_ddrphy.h>
#include <nx_bootheader.h>
#include "ddr3_sdram.h"
#include "secondboot.h"

#include <debug.h>

#define MEMMSG INFO
#define MEM_TYPE_DDR3

#define CFG_ODT_OFF 			(0)
#define CFG_FLY_BY 			(0)
#define CFG_8BIT_DESKEW 		(0)
#define CFG_UPDATE_DREX_SIDE 		(1)	/* 0 : PHY side,  1: Drex side */
#define SKIP_LEVELING_TRAINING 		(0)

#define DDR_RW_CAL 			(0)

#define DDR_CA_SWAP_MODE 		(0)	/* for LPDDR3 */

#define DDR_WRITE_LEVELING_EN 		(0)	/* for fly-by */
#define DDR_CA_CALIB_EN 		(0)	/* for LPDDR3 */
#define DDR_GATE_LEVELING_EN 		(1)	/* for DDR3 great then 800MHz */
#define DDR_READ_DQ_CALIB_EN 		(1)
#define DDR_WRITE_DQ_CALIB_EN 		(1)

#define DDR_RESET_GATE_LVL 		(1)
#define DDR_RESET_READ_DQ 		(1)
#define DDR_RESET_WRITE_DQ 		(1)

#define DDR_MEMINFO_SHOWLOCK		(0)	/* DDR Memory Show Lock Value */


struct NX_SecondBootInfo *const pSBI =
    (struct NX_SecondBootInfo * const)HEADER_BASE;

struct NX_DREXSDRAM_RegisterSet *const pReg_Drex =
    (struct NX_DREXSDRAM_RegisterSet * const)PHY_BASEADDR_DREX_MODULE_CH0_APB;
struct NX_DDRPHY_RegisterSet *const pReg_DDRPHY =
    (struct NX_DDRPHY_RegisterSet * const)PHY_BASEADDR_DREX_MODULE_CH1_APB;



union DWtoB
{
	unsigned int dw;
	unsigned char b[4];
};


#if (CFG_NSIH_EN == 0)
#include "DDR3_K4B8G1646B_MCK0.h"
#endif

#if PLAT == s5p6818
#define nop() __asm__ __volatile__("mov\tx0,x0\t\r\n nop\n\t");
#else
#define nop() __asm__ __volatile__("mov\tr0,r0\t@ nop\n\t");
#endif

struct phy_lock_info {
	unsigned int val;
	unsigned int count;
	unsigned int lock_count[5];
};

unsigned int g_Lock_Val;
unsigned int g_WR_lvl;
unsigned int g_GT_cycle;
unsigned int g_GT_code;
unsigned int g_RD_vwmc;
unsigned int g_WR_vwmc;


static inline void DMC_Delay(int milisecond)
{
	int count, temp;

	for (count = 0; count < milisecond; count++)
		temp ^= count;
}

#if DDR_MEMINFO_SHOWLOCK
void showLockValue(void)
{
	struct phy_lock_info lock_info[20];
	unsigned int fFound = 0;
	unsigned int lock_status, lock_val;
	unsigned int temp, i, j;

	INFO("[msg] waiting for ddr3 lock value calibration! \r\n");

	for (i = 0; i < 20; i++) {
		lock_info[i].val        = 0;
		lock_info[i].count      = 0;

		for (j = 0; j < 5; j++)
			lock_info[i].lock_count[j]  = 0;
	}

	for (i = 0; i < 1000000; i++) {
		temp        = mmio_read_32((uintptr_t) &pReg_DDRPHY->MDLL_CON[1]);
		lock_status = temp & 0x7;
		lock_val    = (temp >> 8) & 0x1FF;         /* read lock value */

		fFound = 0;

		for (j = 0; lock_info[j].val != 0; j++) {
			if (lock_info[j].val == lock_val) {
				fFound = 1;
				lock_info[j].count++;
				if (lock_status)
					lock_info[j].lock_count[(lock_status>>1)]++;
				else
					lock_info[j].lock_count[4]++;
			}
		}

		if (j == 20)
			break;

		if (fFound == 0)
		{
			lock_info[j].val   = lock_val;
			lock_info[j].count = 1;
			if (lock_status)
				lock_info[j].lock_count[(lock_status>>1)] = 1;
			else
				lock_info[j].lock_count[4]  = 1;
		}

		DMC_Delay(10);
	}

	INFO("\r\n");
	INFO("--------------------------------------\r\n");
	INFO(" Show lock values : %d\r\n", g_Lock_Val);
	INFO("--------------------------------------\r\n");

	INFO("lock_val,   hit       bad, not bad,   good, better,   best\r\n");

	for (i = 0; lock_info[i].val; i++) {
		INFO("[%6d, %6d] - [%6d", lock_info[i].val, lock_info[i].count,
				lock_info[i].lock_count[4]);

		for (j = 0; j < 4; j++)
			INFO(", %6d", lock_info[i].lock_count[j]);
		INFO("]\r\n");
	}
}
#endif

#if defined(MEM_TYPE_DDR3)
// inline
void SendDirectCommand(SDRAM_CMD cmd, unsigned char chipnum, SDRAM_MODE_REG mrx, unsigned short value)
{
	mmio_write_32((uintptr_t)
	    &pReg_Drex->DIRECTCMD,
	    (unsigned int)((cmd << 24) | ((chipnum & 1) << 20) | (mrx << 16) | value));
}
#endif
#if defined(MEM_TYPE_LPDDR23)
void SendDirectCommand(SDRAM_CMD cmd, unsigned char chipnum, SDRAM_MODE_REG mrx, unsigned short value)
{
	mmio_write_32((uintptr_t)&pReg_Drex->DIRECTCMD,
			(unsigned int)((cmd << 24) | ((chipnum & 1) << 20) |
			(((mrx >> 3) & 0x7) << 16) | ((mrx & 0x7) << 10) |
			((value & 0xFF) << 2) | ((mrx >> 6) & 0x3)));
}
#endif

void enterSelfRefresh(void)
{
	union SDRAM_MR MR;
	unsigned int nTemp;
	unsigned int nChips = 0;

	struct NX_SecondBootInfo SBI;

	memcpy(&SBI, pSBI, sizeof(struct NX_SecondBootInfo));

#if (CFG_NSIH_EN == 0)
#if (_DDR_CS_NUM > 1)
	nChips = 0x3;
#else
	nChips = 0x1;
#endif
#else
	if (SBI.DII.ChipNum > 1)
		nChips = 0x3;
	else
		nChips = 0x1;
#endif

	while (mmio_read_32((uintptr_t)&pReg_Drex->CHIPSTATUS) & 0xF)
		nop();

	/* Send PALL command */
	SendDirectCommand(SDRAM_CMD_PALL, 0, (SDRAM_MODE_REG)0, 0);
#if (CFG_NSIH_EN == 0)
#if (_DDR_CS_NUM > 1)
	SendDirectCommand(SDRAM_CMD_PALL, 1, (SDRAM_MODE_REG)0, 0);
#endif
#else
	if (SBI.DII.ChipNum > 1)
		SendDirectCommand(SDRAM_CMD_PALL, 1, (SDRAM_MODE_REG)0, 0);
#endif
	DMC_Delay(100);

	/* odt off */
	MR.Reg = 0;
	MR.MR2.RTT_WR = 0; /* 0: disable, 1: RZQ/4 (60ohm), 2: RZQ/2 (120ohm) */
	MR.MR2.SRT = 0; /* self refresh normal range, if (ASR == 1) SRT = 0; */
	MR.MR2.ASR = 1; /* auto self refresh enable */
#if (CFG_NSIH_EN == 0)
	MR.MR2.CWL = (nCWL - 5);
#else
	MR.MR2.CWL = (SBI.DII.CWL - 5);
#endif

	SendDirectCommand(SDRAM_CMD_MRS, 0, SDRAM_MODE_REG_MR2, MR.Reg);
#if (CFG_NSIH_EN == 0)
#if (_DDR_CS_NUM > 1)
	SendDirectCommand(SDRAM_CMD_MRS, 1, SDRAM_MODE_REG_MR2, MR.Reg);
#endif
#else
	if (SBI.DII.ChipNum > 1)
		SendDirectCommand(SDRAM_CMD_MRS, 1, SDRAM_MODE_REG_MR2, MR.Reg);
#endif

	MR.Reg = 0;
	MR.MR1.DLL = 1; /* 0: Enable, 1 : Disable */
#if (CFG_NSIH_EN == 0)
	MR.MR1.AL = MR1_nAL;
#else
	MR.MR1.AL = SBI.DII.MR1_AL;
#endif
	MR.MR1.ODS1 = SBI.DDR3_DSInfo.MR1_ODS & (1 << 1);
	MR.MR1.ODS0 = SBI.DDR3_DSInfo.MR1_ODS & (1 << 0);
	MR.MR1.RTT_Nom2 = SBI.DDR3_DSInfo.MR1_RTT_Nom & (1 << 2);
	MR.MR1.RTT_Nom1 = SBI.DDR3_DSInfo.MR1_RTT_Nom & (1 << 1);
	MR.MR1.RTT_Nom0 = SBI.DDR3_DSInfo.MR1_RTT_Nom & (1 << 0);
	MR.MR1.QOff = 0;
	MR.MR1.WL = 0;
#if 0
#if (CFG_NSIH_EN == 0)
    MR.MR1.TDQS     = (_DDR_BUS_WIDTH>>3) & 1;
#else
    MR.MR1.TDQS     = (SBI.DII.BusWidth>>3) & 1;
#endif
#endif

	SendDirectCommand(SDRAM_CMD_MRS, 0, SDRAM_MODE_REG_MR1, MR.Reg);
#if (CFG_NSIH_EN == 0)
#if (_DDR_CS_NUM > 1)
	SendDirectCommand(SDRAM_CMD_MRS, 1, SDRAM_MODE_REG_MR1, MR.Reg);
#endif
#else
	if (SBI.DII.ChipNum > 1)
		SendDirectCommand(SDRAM_CMD_MRS, 1, SDRAM_MODE_REG_MR1, MR.Reg);
#endif

	/* Enter self-refresh command */
	SendDirectCommand(SDRAM_CMD_REFS, 0, (SDRAM_MODE_REG)0, 0);
#if (CFG_NSIH_EN == 0)
#if (_DDR_CS_NUM > 1)
	SendDirectCommand(SDRAM_CMD_REFS, 1, (SDRAM_MODE_REG)0, 0);
#endif
#else
	if (SBI.DII.ChipNum > 1)
		SendDirectCommand(SDRAM_CMD_REFS, 1, (SDRAM_MODE_REG)0, 0);
#endif

	do {
		nTemp = (mmio_read_32((uintptr_t)&pReg_Drex->CHIPSTATUS) & nChips);
	} while (nTemp);

	do {
		nTemp = ((mmio_read_32((uintptr_t)&pReg_Drex->CHIPSTATUS) >> 8) & nChips);
	} while (nTemp != nChips);

#if 0
	/*
	 * Step 52 Auto refresh counter disable
	 *
	 * afre_en[5]. Auto Refresh Counter. Disable:0, Enable:1
	 */
	mmio_clrbits_32((uintptr_t)&pReg_Drex->CONCONTROL, (0x1 << 5));
#endif

	/*
	 * Step 10  ACK, ACKB off
	 *
	 * clk_stop_en[0] : Dynamic Clock Control
	 *   1'b0 - Always running
	 */
	mmio_setbits_32((uintptr_t)&pReg_Drex->MEMCONTROL, (0x1 << 0));

	/* DMC_Delay(1000 * 3); */
}

unsigned int getVWMC_Offset(unsigned int code, unsigned int lock_div4)
{
	unsigned int i, ret_val;
	unsigned char vwmc[4];
	int offset[4];

	for (i = 0; i < 4; i++)
		vwmc[i] = ((code >> (8 * i)) & 0xFF);

	for (i = 0; i < 4; i++) {
		offset[i] = (int)(vwmc[i] - lock_div4);
		if (offset[i] < 0) {
			offset[i] *= -1;
			offset[i] |= 0x80;
		}
	}

	ret_val = (((unsigned char)offset[3] << 24) |
			((unsigned char)offset[2] << 16) |
			((unsigned char)offset[1] << 8) |
			(unsigned char)offset[0]);

	return ret_val;
}
