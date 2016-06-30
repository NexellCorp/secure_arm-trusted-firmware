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
 *      Module          : second boot
 *      File            : secondboot.h
 *      Description     : This must be synchronized with NSIH.txt
 *      Author          : Firmware Team
 *      History         : 2013.06.23 create
 */

#ifndef __NX_SECONDBOOT_H__
#define __NX_SECONDBOOT_H__

//#include "cfgBootDefine.h"
/* FIXME: */
#define MEM_TYPE_DDR3

#define HEADER_ID                                                              \
	((((unsigned int)'N') << 0) | (((unsigned int)'S') << 8) | (((unsigned int)'I') << 16) |          \
	 (((unsigned int)'H') << 24))

#define LVLTR_WR_LVL (1 << 0)
#define LVLTR_CA_CAL (1 << 1)
#define LVLTR_GT_LVL (1 << 2)
#define LVLTR_RD_CAL (1 << 3)
#define LVLTR_WR_CAL (1 << 4)

enum { BOOT_FROM_USB = 0UL,
       BOOT_FROM_SPI = 1UL,
       BOOT_FROM_NAND = 2UL,
       BOOT_FROM_SDMMC = 3UL,
       BOOT_FROM_SDFS = 4UL,
       BOOT_FROM_UART = 5UL };

struct NX_NANDBootInfo {
	unsigned char AddrStep;
	unsigned char tCOS;
	unsigned char tACC;
	unsigned char tOCH;
#if 0
	unsigned int PageSize    :24;    // 1 byte unit
	unsigned int LoadDevice  :8;
#else
	unsigned char PageSize;   // 512bytes unit
	unsigned char TIOffset;   // 3rd boot Image copy Offset. 1MB unit.
	unsigned char CopyCount;  // 3rd boot image copy count
	unsigned char LoadDevice; // device chip select number
#endif
	unsigned int CRC32;
};

struct NX_SPIBootInfo {
	unsigned char AddrStep;
	unsigned char _Reserved0[2];
	unsigned char PortNumber;

	unsigned int _Reserved1 : 24;
	unsigned int LoadDevice : 8;

	unsigned int CRC32;
};

struct NX_UARTBootInfo {
	unsigned int _Reserved0;

	unsigned int _Reserved1 : 24;
	unsigned int LoadDevice : 8;

	unsigned int CRC32;
};

struct NX_SDMMCBootInfo {
#if 1
	unsigned char PortNumber;
	unsigned char _Reserved0[3];
#else
	unsigned char _Reserved0[3];
	unsigned char PortNumber;
#endif

	unsigned int _Reserved1 : 24;
	unsigned int LoadDevice : 8;

	unsigned int CRC32;
};

struct NX_DDR3DEV_DRVDSInfo {
	unsigned char MR2_RTT_WR;
	unsigned char MR1_ODS;
	unsigned char MR1_RTT_Nom;
	unsigned char _Reserved0;
};

struct NX_LPDDR3DEV_DRVDSInfo {
	unsigned char MR3_DS : 4;
	unsigned char MR11_DQ_ODT : 2;
	unsigned char MR11_PD_CON : 1;
	unsigned char _Reserved0 : 1;

	unsigned char _Reserved1;
	unsigned char _Reserved2;
	unsigned char _Reserved3;
};

struct NX_DDRPHY_DRVDSInfo {
	unsigned char DRVDS_Byte0; // Data Slice 0
	unsigned char DRVDS_Byte1; // Data Slice 1
	unsigned char DRVDS_Byte2; // Data Slice 2
	unsigned char DRVDS_Byte3; // Data Slice 3

	unsigned char DRVDS_CK;  // CK
	unsigned char DRVDS_CKE; // CKE
	unsigned char DRVDS_CS;  // CS
	unsigned char DRVDS_CA;  // CA[9:0], RAS, CAS, WEN, ODT[1:0], RESET, BANK[2:0]

	unsigned char ZQ_DDS; // zq mode driver strength selection.
	unsigned char ZQ_ODT;
	unsigned char _Reserved0[2];
};

struct NX_SDFSBootInfo {
	char BootFileName[12]; // 8.3 format ex)"NXDATA.TBL"
};

union NX_DeviceBootInfo {
	struct NX_NANDBootInfo NANDBI;
	struct NX_SPIBootInfo SPIBI;
	struct NX_SDMMCBootInfo SDMMCBI;
	struct NX_SDFSBootInfo SDFSBI;
	struct NX_UARTBootInfo UARTBI;
};

struct NX_DDRInitInfo {
	unsigned char ChipNum;  // 0x88
	unsigned char ChipRow;  // 0x89
	unsigned char BusWidth; // 0x8A
	unsigned char ChipCol;  // 0x8B

	unsigned short ChipMask; // 0x8C
	unsigned short ChipSize; // 0x8E

#if 0
	unsigned char  CWL;            // 0x90
	unsigned char  WL;             // 0x91
	unsigned char  RL;             // 0x92
	unsigned char  DDRRL;          // 0x93
#else
	unsigned char CWL;    // 0x90
	unsigned char CL;     // 0x91
	unsigned char MR1_AL; // 0x92, MR2_RLWL (LPDDR3)
	unsigned char MR0_WR; // 0x93, MR1_nWR (LPDDR3)
#endif

	unsigned int READDELAY;  // 0x94
	unsigned int WRITEDELAY; // 0x98

	unsigned int TIMINGPZQ;   // 0x9C
	unsigned int TIMINGAREF;  // 0xA0
	unsigned int TIMINGROW;   // 0xA4
	unsigned int TIMINGDATA;  // 0xA8
	unsigned int TIMINGPOWER; // 0xAC
};

struct NX_SecondBootInfo {
	unsigned int VECTOR[8];     // 0x000 ~ 0x01C
	unsigned int VECTOR_Rel[8]; // 0x020 ~ 0x03C

	unsigned int DEVICEADDR; // 0x040

	unsigned int LOADSIZE;		     // 0x044
	unsigned int LOADADDR;		     // 0x048
	unsigned int LAUNCHADDR;		     // 0x04C
	union NX_DeviceBootInfo DBI; // 0x050~0x058

	unsigned int PLL[4];       // 0x05C ~ 0x068
	unsigned int PLLSPREAD[2]; // 0x06C ~ 0x070

#if defined(ARCH_NXP4330) || defined(ARCH_S5P4418)
	unsigned int DVO[5]; // 0x074 ~ 0x084

	struct NX_DDRInitInfo DII; // 0x088 ~ 0x0AC

#if defined(MEM_TYPE_DDR3)
	struct NX_DDR3DEV_DRVDSInfo DDR3_DSInfo; // 0x0B0
#endif
#if defined(MEM_TYPE_LPDDR23)
	struct NX_LPDDR3DEV_DRVDSInfo LPDDR3_DSInfo; // 0x0B0
#endif
	struct NX_DDRPHY_DRVDSInfo PHY_DSInfo; // 0x0B4 ~ 0x0BC

	unsigned short LvlTr_Mode; // 0x0C0 ~ 0x0C1
	unsigned short FlyBy_Mode; // 0x0C2 ~ 0x0C3

	unsigned int Stub[(0x1EC - 0x0C4) / 4]; // 0x0C4 ~ 0x1EC
#endif
#if PLAT == s5p6818
	unsigned int DVO[9]; // 0x074 ~ 0x094

	struct NX_DDRInitInfo DII; // 0x098 ~ 0x0BC

#if defined(MEM_TYPE_DDR3)
	struct NX_DDR3DEV_DRVDSInfo DDR3_DSInfo; // 0x0C0
#endif
#if defined(MEM_TYPE_LPDDR23)
	struct NX_LPDDR3DEV_DRVDSInfo LPDDR3_DSInfo; // 0x0C0
#endif
	struct NX_DDRPHY_DRVDSInfo PHY_DSInfo; // 0x0C4 ~ 0x0CC

	unsigned short LvlTr_Mode; // 0x0D0 ~ 0x0D1
	unsigned short FlyBy_Mode; // 0x0D2 ~ 0x0D3

#if (BOOTCOUNT == 1)
	unsigned int Stub[(0x1E4 - 0x0D4) / 4]; // 0x0D4 ~ 0x1E4
	unsigned int ResetCount;		       // 0x1E4
	unsigned int BootCount;		       // 0x1E8
#else
	unsigned int Stub[(0x1EC - 0x0D4) / 4]; // 0x0D4 ~ 0x1EC
#endif
#endif

	unsigned int MemTestAddr;     // 0x1EC
	unsigned int MemTestSize;     // 0x1F0
	unsigned int MemTestTryCount; // 0x1F4

	unsigned int BuildInfo; // 0x1F8

	unsigned int SIGNATURE; // 0x1FC    "NSIH"
} __attribute__((packed, aligned(4)));

// [0] : Use ICache
// [1] : Change PLL
// [2] : Decrypt
// [3] : Suspend Check

#endif //__NX_SECONDBOOT_H__
