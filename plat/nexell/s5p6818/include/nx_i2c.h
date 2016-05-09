/*------------------------------------------------------------------------------
 *
 *  Copyright (C) Nexell Co. 2012
 *
 *  This confidential and proprietary software may be used only as authorized by
 *  a licensing agreement from Nexell Co.
 *  The entire notice above must be reproduced on all authorized copies and
 *  copies may only be made to the extent permitted by a licensing agreement
 *  from Nexell Co.
 *
 *  Module	: I2C
 *  File	: nx_i2c.c
 *  Description	:
 *  Author	: Firmware Team
 *  History	: 2014.10.01	Revision Comment & Function Define. (Deoks)
 *----------------------------------------------------------------------------*/
#ifndef __NX_I2C_H__
#define __NX_I2C_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*------------------------------------------------------------------------------
 * @defgroup   I2C I2C
 *----------------------------------------------------------------------------*/


/* @brief  I2C Module's Register List */
struct  nx_i2c_registerset {
	unsigned int iccr;	/* 0x00 : I2C Control Register */
	unsigned int icsr;	/* 0x04 : I2C Status Register */
	unsigned int iar; 	/* 0x08 : I2C Address Register */
	unsigned int idsr;	/* 0x0C : I2C Data Register */
	unsigned int iclc;	/* 0x10 : I2C Line Control Register */
};

/* @brief Select I2C Mode */
enum NX_I2C_TXRXMODE {
	NX_I2C_TXRXMODE_SLAVE_RX    = 0,	/* Slave Receive Mode */
	NX_I2C_TXRXMODE_SLAVE_TX    = 1,	/* Slave Transmit Mode */
	NX_I2C_TXRXMODE_MASTER_RX   = 2,	/* Master Receive Mode */
	NX_I2C_TXRXMODE_MASTER_TX   = 3		/* Master Transmit Mode */
};

/* @brief  Start/Stop signal */
enum NX_I2C_SIGNAL {
	NX_I2C_SIGNAL_STOP  = 0,		/* Stop signal generation */
	NX_I2C_SIGNAL_START = 1			/* Start signal generation */
};

enum NX_I2C_SDA_DELAY {
	NX_I2C_SDA_DELAY_0CLOCK  = 0,
	NX_I2C_SDA_DELAY_5CLOCK  = 1,
	NX_I2C_SDA_DELAY_10CLOCK = 2,
	NX_I2C_SDA_DELAY_15CLOCK = 3,
};

#ifdef __cplusplus
}
#endif

#endif
