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
 * File            : nx_clkpwr.h
 * Description     :
 * Author          : Firware Team
 * History         :
 */
#ifndef __NX_CLKPWR_H__
#define __NX_CLKPWR_H__


#define NUMBER_OF_PMU_POWER_DOMAIN      2


#ifdef	__cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------*/
/*/ @defgroup	CLKPWR Clock & Power Manager */
/*----------------------------------------------------------------------------*/


/* @brief	Clock & Power Manager Module's Register List */
struct	nx_clkpwr_registerset {
	unsigned int clkmodereg0;		/* 0x000 : Clock Mode Register 0 */
	unsigned int __reserved0;		/* 0x004 */
	unsigned int pllsetreg[4];		/* 0x008 ~ 0x014 : PLL Setting Register */
	unsigned int __reserved1[2];		/* 0x018 ~ 0x01C */
	unsigned int dvoreg[9];			/* 0x020 ~ 0x040 : Divider Setting Register */
	unsigned int __reserved2;		/* 0x044 */
	unsigned int pllsetreg_sscg[6];		/* 0x048 ~ 0x05C */
	unsigned int __reserved3[8];		/* 0x060 ~ 0x07C */
	unsigned char __reserved4[0x200-0x80];	/* padding (0x80 ~ 0x1FF) */
	unsigned int gpiowakeupriseenb;		/* 0x200 : GPIO Rising Edge Detect Enable Register */
	unsigned int gpiowakeupfallenb;		/* 0x204 : GPIO Falling Edge Detect Enable Register */
	unsigned int gpiorstenb;		/* 0x208 : GPIO Reset Enable Register */
	unsigned int gpiowakeupenb;		/* 0x20C : GPIO Wakeup Source Enable */
	unsigned int gpiointenb;		/* 0x210 : Interrupt Enable Register */
	unsigned int gpiointpend;		/* 0x214 : Interrupt Pend Register */
	unsigned int resetstatus;		/* 0x218 : Reset Status Register */
	unsigned int intenable;			/* 0x21C : Interrupt Enable Register */
	unsigned int intpend;			/* 0x220 : Interrupt Pend Register */
	unsigned int pwrcont;			/* 0x224 : Power Control Register */
	unsigned int pwrmode;			/* 0x228 : Power Mode Register */
	unsigned int __reserved5;		/* 0x22C : Reserved Region */
	unsigned int scratch[3];		/* 0x230 ~ 0x238	: Scratch Register */
	unsigned int sysrstconfig;		/* 0x23C : System Reset Configuration Register. */
	unsigned char  __reserved6[0x2a0-0x240];/* padding (0x240 ~ 0x29F) */
	unsigned int cpupowerdownreq;		/* 0x2A0 : CPU Power Down Request Register */
	unsigned int cpupoweronreq;		/* 0x2A4 : CPU Power On Request Register */
	unsigned int cpuresetmode;		/* 0x2A8 : CPU Reset Mode Register */
	unsigned int cpuwarmresetreq;		/* 0x2AC : CPU Warm Reset Request Register */
	unsigned int __reserved7;		/* 0x2B0 */
	unsigned int cpustatus;			/* 0x2B4 : CPU Status Register */
	unsigned char  __reserved8[0x400-0x2b8];/* padding (0x2B8 ~ 0x3FF) */
};

/* @brief	CLKPWR Interrupts for interrupt interfaces */
enum NX_CLKPWR_INT {
	NX_CLKPWR_INT_ALIVEGPIO0	=  0,	/* ALIVE GPIO 0 int */
	NX_CLKPWR_INT_ALIVEGPIO1	=  1,	/* ALIVE GPIO 1 int */
	NX_CLKPWR_INT_ALIVEGPIO2	=  2,	/* ALIVE GPIO 2 int */
	NX_CLKPWR_INT_ALIVEGPIO3	=  3,	/* ALIVE GPIO 3 int */
	NX_CLKPWR_INT_ALIVEGPIO4	=  4,	/* ALIVE GPIO 4 int */
	NX_CLKPWR_INT_ALIVEGPIO5	=  5,	/* ALIVE GPIO 5 int */
	NX_CLKPWR_INT_VDDTOGGLE		=  6,	/* VDD Power Toggle int */
	NX_CLKPWR_INT_UART0		=  7,	/* UART RX 0 int */
	NX_CLKPWR_INT_UART1		=  8,	/* UART RX 1 int */
	NX_CLKPWR_INT_UART2		=  9,	/* UART RX 2 int */
	NX_CLKPWR_INT_UART3		= 10,	/* UART RX 3 int */
	NX_CLKPWR_INT_UART4		= 11,	/* UART RX 4 int */
	NX_CLKPWR_INT_UART5		= 12,	/* UART RX 5 int */
	NX_CLKPWR_INT_USB0		= 13,	/* USB  0 int */
	NX_CLKPWR_INT_USB1		= 14,	/* USB  1 int */
	NX_CLKPWR_INT_RTC		= 15,	/* RTC int */
	NX_CLKPWR_INT_BATF		= 16,	/* Battery fault event int */
	NX_CLKPWR_INT_MAX		= 17
};

/* @brief	Reset status */
enum NX_CLKPWR_RESETSTATUS {
	NX_CLKPWR_RESETSTATUS_POWERON		= 1<<0,	/* Power on reset */
	NX_CLKPWR_RESETSTATUS_GPIO		= 1<<1,	/* GPIO reset */
	NX_CLKPWR_RESETSTATUS_WATCHDOG		= 1<<2,	/* Watchdog Reset */
	NX_CLKPWR_RESETSTATUS_SOFTWARE		= 1<<3,	/* Software reset */
	NX_CLKPWR_RESETSTATUS_DSLEEPRESET	= 1<<4,	/* Deep Sleep reset */
	NX_CLKPWR_RESETSTATUS_SLEEPRESET	= 1<<5	/* Sleep reset */
};

/* @brief	Power mode */
enum NX_CLKPWR_POWERMODE {
	NX_CLKPWR_POWERMODE_STOP	= 1<<5,	/* Stop mode */
	NX_CLKPWR_POWERMODE_IDLE	= 1<<4	/* Idle mode */
};

/* @brief	CPU Reset mode */
enum NX_CLKPWR_CPU_RESETMODE {
	NX_CLKPWR_CPU_RESETMODE_NONE	= 0UL,	/*  */
	NX_CLKPWR_CPU_RESETMODE_SAFE	= 1UL,	/* Safe reset mode */
	NX_CLKPWR_CPU_RESETMODE_DIRECT	= 2UL	/* Direct reset mode */
};

/* @brief	pll number */
enum NX_CLKPWR_PLL {
	NX_CLKPWR_PLL_0			= 0UL,	/* pll number 0 */
	NX_CLKPWR_PLL_1			= 1UL,	/* pll number 1 */
	NX_CLKPWR_PLL_2			= 2UL,	/* pll number 2 */
	NX_CLKPWR_PLL_3 		= 3UL	/* pll number 3 */
};

/* @brief	clock divider select */
enum NX_CLKPWR_CLOCK {
	NX_CLKPWR_CLOCK_FCLK		= 0UL,	/* clock divider FCLK */
	NX_CLKPWR_CLOCK_BCLK		= 1UL,	/* clock divider BCLK */
	NX_CLKPWR_CLOCK_MCLK		= 2UL,	/* clock divider MCLK */
	NX_CLKPWR_CLOCK_GR3DBCLK	= 3UL,	/* clock divider GR3DBCLK */
	NX_CLKPWR_CLOCK_MPEGBCLK	= 4UL	/* clock divider MPEGBCLK */
};

/* @brief	Glitchless mux select */
enum NX_CLKPWR_GMUX {
	NX_CLKPWR_GMUX_XTAL		= 0UL,	/* PLL's out-mux to Xrystal */
	NX_CLKPWR_GMUX_PLL		= 1UL	/* PLL's out-mux to PLL */
};

/* @brief	power switch number */
enum NX_CLKPWR_POWER_SWITCH {
	NX_CLKPWR_POWER_SWITCH_PRECHARGE	= 0UL,	/* pre discharge power switch */
	NX_CLKPWR_POWER_SWITCH_ALL		= 1UL	/* all power switch */
};

enum NX_CLKPWR_POWER_DOMAIN {
	NX_CLKPWR_POWER_DOMAIN_0	= 0UL,	/* power domain 0 */
	NX_CLKPWR_POWER_DOMAIN_1	= 1UL	/* power domain 1 */
};

enum NX_CLKPWR_PLL_POWER {
	NX_CLKPWR_PLL_POWER_UP  	= 0UL,	/* power up */
	NX_CLKPWR_PLL_POWER_DOWN	= 1UL	/* power down */
};


unsigned int nx_clkpwr_getpllfreq(unsigned int pllnumber);

#ifdef	__cplusplus
}
#endif


#endif
