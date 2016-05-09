/*------------------------------------------------------------------------------
 *
 * Copyright (C) 2009 Nexell Co., All Rights Reserved
 * Nexell Co. Proprietary & Confidential
 *
 * NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 * AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 * FOR A PARTICULAR PURPOSE.
 *
 * Module	: GPIO
 * File		: nx_gpio.h
 * Description	:
 * Author	: Firmware Team
 * History	:
 *
 *----------------------------------------------------------------------------*/
#ifndef __NX_GPIO_H__
#define __NX_GPIO_H__


#ifdef	__cplusplus
extern "C"
{
#endif

/* @brief	GPIO Module's Register List */
struct	nx_gpio_registerset {
	unsigned int gpioxout;				/* 0x00 : Output Register */
	unsigned int gpioxoutenb;			/* 0x04 : Output Enable Register */
	unsigned int gpioxdetmode[2];			/* 0x08 : Event Detect Mode Register */
	unsigned int gpioxintenb;			/* 0x10 : Interrupt Enable Register */
	unsigned int gpioxdet;				/* 0x14 : Event Detect Register */
	unsigned int gpioxpad;				/* 0x18 : PAD Status Register */
	unsigned int gpioxpuenb;			/* 0x1C : Pull Up Enable Register */
	unsigned int gpioxaltfn[2];			/* 0x20 : Alternate Function Select Register */
	unsigned int gpioxdetmodeex;			/* 0x28 : Event Detect Mode extended Register */
	unsigned int __reserved[4];			/* 0x2B : */
	unsigned int gpioxdetenb;			/* 0x3C : IntPend Detect Enable Register */
	unsigned int gpiox_slew;			/* 0x40 : Slew Register */
	unsigned int gpiox_slew_disable_default;	/* 0x44 : Slew set On/Off Register */
	unsigned int gpiox_drv1;			/* 0x48 : drive strength LSB Register */
	unsigned int gpiox_drv1_disable_default;	/* 0x4C : drive strength LSB set On/Off Register */
	unsigned int gpiox_drv0;			/* 0x50 : drive strength MSB Register */
	unsigned int gpiox_drv0_disable_default;	/* 0x54 : drive strength MSB set On/Off Register */
	unsigned int gpiox_pullsel;			/* 0x58 : Pull UP/DOWN Selection Register */
	unsigned int gpiox_pullsel_disable_default;	/* 0x5C : Pull UP/DOWN Selection On/Off Register */
	unsigned int gpiox_pullenb;			/* 0x60 : Pull Enable/Disable Register */
	unsigned int gpiox_pullenb_disable_default;	/* 0x64 : Pull Enable/Disable selection On/Off Register */
	unsigned int gpiox_inputmuxselect0;		/* 0x68 */
	unsigned int gpiox_inputmuxselect1;		/* 0x6C */
	unsigned char __reserved1[0x1000-0x70];
};

/* @brief	GPIO Interrupts for interrupt interface */
enum NX_GPIO_INT {
	NX_GPIO_INT_GPIO0	= 0,
	NX_GPIO_INT_GPIO1	= 1,
	NX_GPIO_INT_GPIO2	= 2,
	NX_GPIO_INT_GPIO3	= 3,
	NX_GPIO_INT_GPIO4	= 4,
	NX_GPIO_INT_GPIO5	= 5,
	NX_GPIO_INT_GPIO6	= 6,
	NX_GPIO_INT_GPIO7	= 7,
	NX_GPIO_INT_GPIO8	= 8,
	NX_GPIO_INT_GPIO9	= 9,
	NX_GPIO_INT_GPIO10	= 10,
	NX_GPIO_INT_GPIO11	= 11,
	NX_GPIO_INT_GPIO12	= 12,
	NX_GPIO_INT_GPIO13	= 13,
	NX_GPIO_INT_GPIO14	= 14,
	NX_GPIO_INT_GPIO15	= 15,
	NX_GPIO_INT_GPIO16	= 16,
	NX_GPIO_INT_GPIO17	= 17,
	NX_GPIO_INT_GPIO18	= 18,
	NX_GPIO_INT_GPIO19	= 19,
	NX_GPIO_INT_GPIO20	= 20,
	NX_GPIO_INT_GPIO21	= 21,
	NX_GPIO_INT_GPIO22	= 22,
	NX_GPIO_INT_GPIO23	= 23,
	NX_GPIO_INT_GPIO24	= 24,
	NX_GPIO_INT_GPIO25	= 25,
	NX_GPIO_INT_GPIO26	= 26,
	NX_GPIO_INT_GPIO27	= 27,
	NX_GPIO_INT_GPIO28	= 28,
	NX_GPIO_INT_GPIO29	= 29,
	NX_GPIO_INT_GPIO30	= 30,
	NX_GPIO_INT_GPIO31	= 31
};

enum NX_GPIO_GROUP {
	NX_GPIO_GROUP_A		= 0,
	NX_GPIO_GROUP_B		= 1,
	NX_GPIO_GROUP_C		= 2,
	NX_GPIO_GROUP_D		= 3,
	NX_GPIO_GROUP_E		= 4
};

/* @brief	GPIO interrupt mode */
enum NX_GPIO_INTMODE {
	NX_GPIO_INTMODE_LOWLEVEL	= 0UL,	/* Low level detect */
	NX_GPIO_INTMODE_HIGHLEVEL	= 1UL,	/* High level detect */
	NX_GPIO_INTMODE_FALLINGEDGE	= 2UL,	/* Falling edge detect */
	NX_GPIO_INTMODE_RISINGEDGE	= 3UL,	/* Rising edge detect */
	NX_GPIO_INTMODE_BOTHEDGE	= 4UL  	/* both (rise and falling) edge detect */
};

/* @brief	I/O mode */
enum NX_GPIO_PADFUNC {
	NX_GPIO_PADFUNC_0		= 0UL,	/* Alternate function 0 */
	NX_GPIO_PADFUNC_1		= 1UL,	/* Alternate function 1 */
	NX_GPIO_PADFUNC_2		= 2UL,	/* Alternate function 2 */
	NX_GPIO_PADFUNC_3		= 3UL	/* Alternate function 3 */
};

enum NX_GPIO_DRVSTRENGTH {
	NX_GPIO_DRVSTRENGTH_0		= 0UL,
	NX_GPIO_DRVSTRENGTH_1		= 1UL,
	NX_GPIO_DRVSTRENGTH_2		= 2UL,
	NX_GPIO_DRVSTRENGTH_3		= 3UL
};

enum NX_GPIO_PULL {
	NX_GPIO_PULL_DOWN		= 0UL,
	NX_GPIO_PULL_UP			= 1UL,
	NX_GPIO_PULL_OFF		= 2UL
};


struct NX_PAD {
	unsigned int	alt	: 3;	/* pad function number */
	unsigned int	pin	: 5;	/* pad number at any gpio group */
	unsigned int	group	: 8;	/* gpio group number */
	unsigned int	can_gpio:16;	/* weather can use gpio mode or special function use only pad. */
};
#ifdef	__cplusplus
}
#endif

#endif
