/*
 * Copyright (C) 2012 Nexell Co., All Rights Reserved
 * Nexell Co. Proprietary & Confidential
 *
 * NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 * AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 * FOR A PARTICULAR PURPOSE.
 *
 * Module	: reset control
 * File		: rstcon.c
 * Description	:
 * Author	: Hans
 * History	: 2016.05.12 First implementation
 */

#include <nx_s5p6818.h>
#include <nx_rstcon.h>

void nx_setresetcon(unsigned int resetnum, int benb)
{
	unsigned int regval;
	volatile struct nx_rstcon_registerset * prstcon =
		(struct nx_rstcon_registerset *)PHY_BASEADDR_RSTCON_MODULE;

	regval = prstcon->regrst[resetnum>>5];
	regval &= ~(1<<(resetnum & 0x1f));

	if(0 == benb)	/* reset negate */
		regval |= 1<<(resetnum & 0x1f);

	prstcon->regrst[resetnum>>5] = regval;
}

int nx_getresetcon(unsigned int resetnum)
{
	unsigned int regval;

	volatile struct nx_rstcon_registerset * prstcon =
		(struct nx_rstcon_registerset *)PHY_BASEADDR_RSTCON_MODULE;

	regval = prstcon->regrst[resetnum>>5];

	return (regval >> (resetnum & 0x1f)) & 1;
}
