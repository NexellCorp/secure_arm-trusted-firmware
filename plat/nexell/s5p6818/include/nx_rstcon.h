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
 * Author	: Firmware Team
 * History	:
 *----------------------------------------------------------------------------*/
#ifndef __NX_RSTCON_H__
#define __NX_RSTCON_H__

#include "nx_s5p6818.h"

#ifdef	__cplusplus
extern "C"
{
#endif

struct	nx_rstcon_registerset {
	unsigned int	regrst[(NUMBER_OF_RESET_MODULE_PIN+31)>>5];
};

enum RSTCON {
	RSTCON_ASSERT	= 0UL,
	RSTCON_NEGATE	= 1UL
};


#ifdef	__cplusplus
}
#endif

#endif
