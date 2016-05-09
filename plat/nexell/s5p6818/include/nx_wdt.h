/*------------------------------------------------------------------------------
 *
 * Copyright (C) Nexell Co. 2012
 *
 * This confidential and proprietary software may be used only as authorized by
 * a licensing agreement from Nexell Co.
 * The entire notice above must be reproduced on all authorized copies and
 * copies may only be made to the extent permitted by a licensing agreement
 * from Nexell Co.
 *
 * Module	: I2C
 * File		: nx_i2c.c
 * Description	:
 * Author	: Jonghyuk Park(charles@nexell.co.kr)
 * History	: 20120814 first implementation
 *----------------------------------------------------------------------------*/
#ifndef _NX_WDT_H
#define _NX_WDT_H


#ifdef	__cplusplus
extern "C"
{
#endif

struct nx_wdt_registerset {
	unsigned int wtcon;	/* 0x00	R/W control register */
	unsigned int wtdat;	/* 0x04	R/W data register */
	unsigned int wtcnt;	/* 0x08	R/W count register */
	unsigned int wtclrint;	/* 0x0C	W   interrupt clear register */
};

enum NX_WDT_CLOCK_DIV {
	WDT_CLOCK_DIV16	 = 0,
	WDT_CLOCK_DIV32  = 1,
	WDT_CLOCK_DIV64  = 2,
	WDT_CLOCK_DIV128 = 3
};


#ifdef	__cplusplus
}
#endif

#endif
