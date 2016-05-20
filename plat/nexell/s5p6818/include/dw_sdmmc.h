/*
 * Copyright (C) 2012 Nexell Co., All Rights Reserved
 * Nexell Co. Proprietary & Confidential
 *
 * NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 * AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 * FOR A PARTICULAR PURPOSE.
 *
 * Module          : SDMMC
 * File            : nx_sdmmc.h
 * Description     :
 * Author          : Firware Team
 * History         :
 */

#ifndef __NX_SDMMC_H__
#define __NX_SDMMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* @brief  SDMMC Module's Register List */
struct  nx_sdmmc_registerset {
	unsigned int	ctrl;			/* 0x000 : Control Register */
	unsigned int	pwren;			/* 0x004 : Power Enable Register */
	unsigned int	clkdiv;			/* 0x008 : Clock Divider Register */
	unsigned int	clksrc;			/* 0x00C : Clock Source Register */
	unsigned int	clkena;			/* 0x010 : Clock Enable Register */
	unsigned int	tmout;			/* 0x014 : Time-Out Register */
	unsigned int	ctype;			/* 0x018 : Card Type Register */
	unsigned int	blksiz;			/* 0x01C : Block Size Register */
	unsigned int	bytcnt;			/* 0x020 : Byte Count Register */
	unsigned int	intmask;		/* 0x024 : Interrupt Mask Register */
	unsigned int	cmdarg;			/* 0x028 : Command Argument Register */
	unsigned int	cmd;			/* 0x02C : Command Register */
	unsigned int	resp0;			/* 0x030 : Response 0 Register */
	unsigned int	resp1;			/* 0x034 : Response 1 Register */
	unsigned int	resp2;			/* 0x038 : Response 2 Register */
	unsigned int	resp3;			/* 0x03C : Response 3 Register */
	unsigned int	mintsts;		/* 0x040 : Masked Interrupt Status Register */
	unsigned int	rintsts;		/* 0x044 : Raw Interrupt Status Register */
	unsigned int	status;			/* 0x048 : Status Register - Mainly for Debug Purpose */
	unsigned int	fifoth;			/* 0x04C : FIFO Threshold Register */
	unsigned int	cdetect;		/* 0x050 : Card Detect Register */
	unsigned int	wrtprt;			/* 0x054 : Write Protect Register */
	unsigned int	gpio;			/* 0x058 : General Purpose Input/Output Register */
	unsigned int	tcbcnt;			/* 0x05C : Transferred CIU card byte count */
	unsigned int	tbbcnt;			/* 0x060 : Transferred Host Byte Count */
	unsigned int	debnce;			/* 0x064 : Card Detect Debounce Register */
	unsigned int	usrid;			/* 0x068 : User ID Register */
	unsigned int	verid;			/* 0x06C : Version ID Register */
	unsigned int	hcon;			/* 0x070 : Hardware Configuration Register */
	unsigned int	uhs_reg;		/* 0x074 : UHS_REG register */
	unsigned int	rstn;			/* 0x078 : Hardware reset register */
	unsigned int	_reserved0;		/* 0x07C */
	unsigned int	bmod;			/* 0x080 : Bus Mode Register */
	unsigned int	pldmnd;			/* 0x084 : Poll Demand Register */
	unsigned int	dbaddr;			/* 0x088 : Descriptor List Base Address Register */
	unsigned int	idsts;			/* 0x08C : Internal DMAC Status Register */
	unsigned int	idinten;		/* 0x090 : Internal DMAC Interrupt Enable Register */
	unsigned int	dscaddr;		/* 0x094 : Current Host Descriptor Address Register */
	unsigned int	bufaddr;		/* 0x098 : Current Buffer Descriptor Address Register */
	unsigned char	_reserved1[0x64];	/* 0x09C ~ 0x100 reserved area */
	unsigned int	cardthrctl;		/* 0x100 : Card Read Threshold Enable */
	unsigned int	backend_power;		/* 0x104 : Back-end Power */
	unsigned int	uhs_reg_ext;		/* 0x108 : eMMC 4.5 1.2V Register */
	unsigned int	emmc_ddr_reg;		/* 0x10C : eMMC DDR START bit detection control register */
	unsigned int	enable_shift;		/* 0x110 : Phase shift control register */
	unsigned int	clkctrl;		/* 0x114 : External clock phase & delay control register */
	unsigned char	_reserved2[0xe8];	/* 0x118 ~ 0x200 */
	unsigned int	data;			/* 0x200 : Data */
};

struct strnxsdmmcbiu {
	unsigned int	ctrl;
	unsigned int	bsize;
	unsigned int	baddr;
	struct strnxsdmmcbiu *next;
};

/* SDMMC Interrupts for Interrupt Interface */
enum    NX_SDMMC_INT {
	NX_SDMMC_INT_SDIO	= 16,
	NX_SDMMC_INT_EBE	= 15,
	NX_SDMMC_INT_ACD	= 14,
	NX_SDMMC_INT_SBE	= 13,
	NX_SDMMC_INT_HLE	= 12,
	NX_SDMMC_INT_FRUN	= 11,
	NX_SDMMC_INT_HTO	= 10,
	NX_SDMMC_INT_DRTO	= 9,
	NX_SDMMC_INT_RTO	= 8,
	NX_SDMMC_INT_DCRC	= 7,
	NX_SDMMC_INT_RCRC	= 6,
	NX_SDMMC_INT_RXDR	= 5,
	NX_SDMMC_INT_TXDR	= 4,
	NX_SDMMC_INT_DTO	= 3,
	NX_SDMMC_INT_CD		= 2,
	NX_SDMMC_INT_RE		= 1,
	NX_SDMMC_INT_CDET	= 0
};

enum    NX_SDMMC_CMDFLAG {
#if 1
	NX_SDMMC_CMDFLAG_STARTCMD		= -2147483648, /* enum is int */
#else
	NX_SDMMC_CMDFLAG_STARTCMD		= 1UL<<31,
#endif
	NX_SDMMC_CMDFLAG_USE_HOLD		= 1UL<<29,
	NX_SDMMC_CMDFLAG_VOLT_SWITCH		= 1UL<<28,
	NX_SDMMC_CMDFLAG_BOOT_MODE		= 1UL<<27,
	NX_SDMMC_CMDFLAG_DISABLE_BOOT		= 1UL<<26,
	NX_SDMMC_CMDFLAG_EXPECT_BOOT_ACK	= 1UL<<25,
	NX_SDMMC_CMDFLAG_ENABLE_BOOT		= 1UL<<24,
	NX_SDMMC_CMDFLAG_CCS_EXPECTED		= 1UL<<23,
	NX_SDMMC_CMDFLAG_READ_CEATA		= 1UL<<22,
	NX_SDMMC_CMDFLAG_UPDATECLKONLY		= 1UL<<21,
	NX_SDMMC_CMDFLAG_CARD_NUMBER		= 1UL<<16,
	NX_SDMMC_CMDFLAG_SENDINIT		= 1UL<<15,
	NX_SDMMC_CMDFLAG_STOPABORT		= 1UL<<14,
	NX_SDMMC_CMDFLAG_WAITPRVDAT		= 1UL<<13,
	NX_SDMMC_CMDFLAG_SENDAUTOSTOP		= 1UL<<12,
	NX_SDMMC_CMDFLAG_BLOCK			= 0UL<<11,
	NX_SDMMC_CMDFLAG_STREAM			= 1UL<<11,
	NX_SDMMC_CMDFLAG_TXDATA			= 3UL<<9,
	NX_SDMMC_CMDFLAG_RXDATA			= 1UL<<9,
	NX_SDMMC_CMDFLAG_CHKRSPCRC		= 1UL<<8,
	NX_SDMMC_CMDFLAG_SHORTRSP		= 1UL<<6,
	NX_SDMMC_CMDFLAG_LONGRSP		= 3UL<<6
};

enum	NX_SDMMC_CMDFSM {
	NX_SDMMC_CMDFSM_IDLE		= 0,	/* Idle. */
	NX_SDMMC_CMDFSM_SENDINIT	= 1,	/* Send init sequence. */
	NX_SDMMC_CMDFSM_TXCMDSTART	= 2,	/* Tx command start bit */
	NX_SDMMC_CMDFSM_TXCMDTX		= 3,	/* Tx command tx bit. */
	NX_SDMMC_CMDFSM_TXCMDINDEXARG	= 4,	/* Tx command index + arg. */
	NX_SDMMC_CMDFSM_TXCMDCRC7	= 5,	/* Tx command CRC7. */
	NX_SDMMC_CMDFSM_TXCMDEND	= 6,	/* Tx command end bit. */
	NX_SDMMC_CMDFSM_RXRSPSTART	= 7,	/* Rx response start bit. */
	NX_SDMMC_CMDFSM_RXRSPIRQ	= 8,	/* Rx response IRQ response. */
	NX_SDMMC_CMDFSM_RXRSPTX		= 9,	/* Rx response tx bit. */
	NX_SDMMC_CMDFSM_TXRSPCMDIDX	= 10,	/* Rx response command index. */
	NX_SDMMC_CMDFSM_RXRSPDATA	= 11,	/* Rx response data. */
	NX_SDMMC_CMDFSM_RXRSPCRC7	= 12,	/* Rx response CRC7. */
	NX_SDMMC_CMDFSM_RXRSPEND	= 13,	/* Rx response end bit. */
	NX_SDMMC_CMDFSM_CMDWAITNCC	= 14,   /* Command path wait NCC. */
	NX_SDMMC_CMDFSM_WAIT		= 15	/* Wait; Comamnd to Response turnaround. */
};

enum	NX_SDMMC_CLOCK_SHIFT {
	NX_SDMMC_CLOCK_SHIFT_0		= 0,
	NX_SDMMC_CLOCK_SHIFT_90		= 1,
	NX_SDMMC_CLOCK_SHIFT_180	= 2,
	NX_SDMMC_CLOCK_SHIFT_270	= 3
};

enum	NX_SDMMC_CLOCK_SOURCE {
	NX_SDMMC_CLOCK_SOURCE_0		= 0,
	NX_SDMMC_CLOCK_SOURCE_1		= 1,
	NX_SDMMC_CLOCK_SOURCE_2		= 2,
	NX_SDMMC_CLOCK_SOURCE_3		= 3,
};

#ifdef __cplusplus
}
#endif

#endif
