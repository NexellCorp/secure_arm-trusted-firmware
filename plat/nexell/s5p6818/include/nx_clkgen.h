/*------------------------------------------------------------------------------
 *
 *	Copyright (C) Nexell Co. 2012
 *
 * This confidential and proprietary software may be used only as authorized by
 * a licensing agreement from Nexell Co.
 * The entire notice above must be reproduced on all authorized copies and
 * copies may only be made to the extent permitted by a licensing agreement
 * from Nexell Co.
 *
 *	Module		: CLKGEN Ctrl
 *	File		: nx_CLKGEN.h
 *	Description	:
 *	Author		: SOC Team
 *	History		:
 *
 *----------------------------------------------------------------------------*/
#ifndef __NX_CLKGEN_H__
#define __NX_CLKGEN_H__


#ifdef __cplusplus
extern "C"
{
#endif

struct	nx_clkgen_registerset {
	unsigned int	clkenb;		/* 0x00 : Clock Enable */
	unsigned int	clkgen[4];	/* 0x04 : Clock Generate */
};
enum	NX_PCLKMODE {
	NX_PCLKMODE_DYNAMIC	= 0UL,	/* PCLK is provided only when CPU has access to registers of this module. */
	NX_PCLKMODE_ALWAYS	= 1UL	/* PCLK is always provided for this module. */
};

/*	@brief type for BCLK control mode */
enum	NX_BCLKMODE {
	NX_BCLKMODE_DISABLE	= 0UL,	/* BCLK is disabled. */
	NX_BCLKMODE_DYNAMIC	= 2UL,	/* BCLK is provided only when this module requests it. */
	NX_BCLKMODE_ALWAYS	= 3UL	/* BCLK is always provided for this module. */
};


#ifdef __cplusplus
}
#endif

#endif
