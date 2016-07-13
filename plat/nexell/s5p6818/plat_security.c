/*
 * Copyright (c) 2015, Linaro Ltd and Contributors. All rights reserved.
 * Copyright (c) 2015, Hisilicon Ltd and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <stdint.h>
#include <strings.h>
#include <platform_def.h>
#include <nx_s5p6818.h>

#define PORTNUM_MAX		1

#define TZPC_GROUPNUM_MAX	7
#define TZPC_PORTNUM_MAX	4

#define DREX_SECURITY_BASE	PHY_BASEADDR_DREX_TZ_MODULE
#define TZPC_BASE		(PHY_BASEADDR_DREXTZASC_MODULE + 0x1000)
#define TZPC_OFFSET		0x1000


#define TZPC_S_MODE		0
#define TZPC_NS_MODE		1

#define TZPC_UNLOCK		0
#define TZPC_LOCK		1

#define TZPC_R0SIZE_NONE	0
#define TZPC_R0SIZE_4KB		1
#define TZPC_R0SIZE_8KB		2
#define TZPC_R0SIZE_12KB	3
#define TZPC_R0SIZE_16KB	4
#define TZPC_R0SIZE_20KB	5
#define TZPC_R0SIZE_24KB	6
#define TZPC_R0SIZE_28KB	7
#define TZPC_R0SIZE_32KB	8
#define TZPC_R0SIZE_36KB	9
#define TZPC_R0SIZE_40KB	10
#define TZPC_R0SIZE_44KB	11
#define TZPC_R0SIZE_48KB	12
#define TZPC_R0SIZE_52KB	13
#define TZPC_R0SIZE_56KB	14
#define TZPC_R0SIZE_60KB	15
#define TZPC_R0SIZE_64KB	16
#define TZPC_R0SIZE_2MB		0x1FF
#define TZPC_R0SIZE_ALL		0x200

struct tzpc_decprot_reg {
	unsigned status;			/* Decode Protection Status */
	unsigned set;				/* Decode Protect Set */
	unsigned clear;				/* Decode Protect Clear */
};

struct tzpc_prot_reg {
	unsigned r0size;			/* 0x000 RAM Region Size */
	unsigned reserved_0[0x1FF];		/* 0x004~0x7FC */
	struct tzpc_decprot_reg decprot[4];	/* 0x800~0x82C */
	unsigned reserved_1[0x1EC];		/* 0x830~0xFDC */
	unsigned periphid[4];			/* 0xFE0~0xFEC */
	unsigned pcellid[4];			/* 0xFF0~0xFFC */
};

struct rgn_map_reg {
	unsigned long reserved_0	:16;
	unsigned long rgn_base_addr	:19;
};

struct rgn_attr_reg {
	unsigned rgn_en			: 1;
	unsigned reserved_0		: 3;
	unsigned rgn_size		:19;
	unsigned reserved_1		: 5;
	unsigned sp				: 4;
};

const unsigned short tzpc_r0size[TZPC_GROUPNUM_MAX] = {
	TZPC_R0SIZE_ALL,  /* size of secure mode management area of sram */
	TZPC_R0SIZE_4KB,  /* GIC400, */
	TZPC_R0SIZE_NONE,
	TZPC_R0SIZE_NONE,
	TZPC_R0SIZE_NONE,
	TZPC_R0SIZE_NONE,
	TZPC_R0SIZE_NONE
};

struct tzpc_prot_bit {
	unsigned char	port0	:1;
	unsigned char	port1	:1;
	unsigned char	port2	:1;
	unsigned char	port3	:1;
	unsigned char	port4	:1;
	unsigned char	port5	:1;
	unsigned char	port6	:1;
	unsigned char	port7	:1;
};

const struct tzpc_prot_bit tzpc_prot_bit[TZPC_GROUPNUM_MAX*TZPC_PORTNUM_MAX] = {
	/*=============TZPC0================= */
	{
		TZPC_NS_MODE,	/* ioperi_bus_m0 */
		TZPC_NS_MODE,	/* ioperi_bus_m1 */
		TZPC_NS_MODE,	/* ioperi_bus_m2 */
		TZPC_NS_MODE,	/* top_bus m0 */
		TZPC_NS_MODE,	/* static_bus m0 mcus address area */
		TZPC_NS_MODE,	/* static_bus m1 sram address area */
		TZPC_NS_MODE,	/* display_bus m0 */
		TZPC_NS_MODE	/* bot_bus m0 */
	},
	{
		TZPC_NS_MODE,	/* sfr_bus m0 */
		TZPC_NS_MODE,	/* sfr_bus m1 */
		TZPC_NS_MODE,	/* sfr_bus m2 */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* drex_0_secure_boot_lock */
		TZPC_UNLOCK	/* GIC400 */
	},
	{
		TZPC_NS_MODE,	/* sfr0_bus m0 */
		TZPC_NS_MODE,	/* sfr0_bus m1 */
		TZPC_NS_MODE,	/* sfr0_bus m2 */
		TZPC_NS_MODE,	/* sfr0_bus m3 */
		TZPC_NS_MODE,	/* sfr0_bus m4 */
		TZPC_NS_MODE,	/* sfr0_bus m5 */
		TZPC_NS_MODE,	/* sfr0_bus m6 */
		TZPC_S_MODE	/* NC */
	},
	{
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	},
	/*=============TZPC1================= */
	{
		TZPC_NS_MODE,	/* sfr1_bus m0 */
		TZPC_NS_MODE,	/* sfr1_bus m1 */
		TZPC_NS_MODE,	/* sfr1_bus m2 */
		TZPC_NS_MODE,	/* sfr1_bus m3 */
		TZPC_NS_MODE,	/* sfr1_bus m4 */
		TZPC_NS_MODE,	/* sfr1_bus m5 */
		TZPC_NS_MODE,	/* sfr1_bus m6 */
		TZPC_NS_MODE	/* mali400 XPROT */
	},
	{
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	},
	{
		TZPC_NS_MODE,	/* sfr2_bus m0 */
		TZPC_NS_MODE,	/* sfr2_bus m1 */
		TZPC_NS_MODE,	/* sfr2_bus m2 */
		TZPC_NS_MODE,	/* sfr2_bus m3 */
		TZPC_NS_MODE,	/* sfr2_bus m4 */
		TZPC_NS_MODE,	/* sfr2_bus m5 */
		TZPC_NS_MODE,	/* display s0 XPROT */
		TZPC_NS_MODE	/* display s0 XPROT */
	},
	{
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	},
	/*=============TZPC2================= */
	{
		TZPC_NS_MODE,	/* sysctrltop	-- finally need to set secure */
		TZPC_NS_MODE,	/* tieoff	-- finally need to set secure */
		TZPC_NS_MODE,	/* rstcon	-- finally need to set secure */
		TZPC_S_MODE,	/* NC */
		TZPC_NS_MODE,	/* pdm */
		TZPC_NS_MODE,	/* crypto */
		TZPC_S_MODE,	/* NC */
		TZPC_NS_MODE	/* pwmtimer0 */
	},
	{
		TZPC_NS_MODE,	/* pwmtimer1 */
		TZPC_NS_MODE,	/* wdt		---finally need to set secure */
		TZPC_NS_MODE,	/* gpioA	---finally need to set secure */
		TZPC_NS_MODE,	/* gpioB	---finally need to set secure */
		TZPC_NS_MODE,	/* gpioC	---finally need to set secure */
		TZPC_NS_MODE,	/* gpioD	---finally need to set secure */
		TZPC_NS_MODE,	/* gpioE	---finally need to set secure */
		TZPC_S_MODE	/* NC */
	},
	{
		TZPC_S_MODE,	/* NC */
		TZPC_NS_MODE,	/* mcustop control interface */
		TZPC_S_MODE,	/* NC */
		TZPC_NS_MODE,	/* adc */
		TZPC_NS_MODE,	/* ppm */
		TZPC_NS_MODE,	/* i2s0 */
		TZPC_NS_MODE,	/* i2s1 */
		TZPC_NS_MODE	/* i2s2 */
	},
	{
		TZPC_NS_MODE,	/* ac97 */
		TZPC_NS_MODE,	/* spdiftx */
		TZPC_NS_MODE,	/* spdifrx */
		TZPC_NS_MODE,	/* ssp0 */
		TZPC_NS_MODE,	/* ssp1 */
		TZPC_NS_MODE,	/* mpegtsi */
		TZPC_S_MODE,	/* NC */
		TZPC_NS_MODE	/* ssp2 */
	},
	/*=============TZPC3================= */
	{
		TZPC_NS_MODE,	/* gmac gmac0 XPROT */
		TZPC_NS_MODE,	/* gmac */
		TZPC_NS_MODE,	/* sdmmc0 */
		TZPC_NS_MODE,	/* vip1 VIP001 XPROT */
		TZPC_NS_MODE,	/* vip0 VIP000 XPROT */
		TZPC_NS_MODE,	/* deinterlace DEINTERLACE XPROT */
		TZPC_NS_MODE,	/* scaler SCALER XPROT */
		TZPC_NS_MODE	/* ecid		---finally need to set secure */
	},
	{
		TZPC_NS_MODE,	/* sdmmc1 */
		TZPC_NS_MODE,	/* sdmmc2 */
		TZPC_NS_MODE,	/* usbhost_phy */
		TZPC_NS_MODE,	/* usbhostclk */
		TZPC_NS_MODE,	/* usbotg_phy */
		TZPC_NS_MODE,	/* uart04 */
		TZPC_NS_MODE,	/* uart04clk */
		TZPC_NS_MODE	/* uart05 */
	},
	{
		TZPC_NS_MODE,	/* coda0 */
		TZPC_NS_MODE,	/* coda1 */
		TZPC_NS_MODE,	/* coda2 */
		TZPC_NS_MODE,	/* coda3 */
		TZPC_NS_MODE,	/* coda to bottom bus xiu */
		TZPC_NS_MODE,	/* coda to ccibus xiu */
		TZPC_S_MODE,	/* NC */
		TZPC_NS_MODE	/* CODA960 XPROT */
	},
	{
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	},
	/*=============TZPC4================= */
	{
		TZPC_NS_MODE,	/* ioperi_bus */
		TZPC_NS_MODE,	/* bot_bus */
		TZPC_NS_MODE,	/* top_bus */
		TZPC_NS_MODE,	/* disp_bus */
		TZPC_NS_MODE,	/* sfr_bus */
		TZPC_NS_MODE,	/* sfr0_bus */
		TZPC_NS_MODE,	/* sfr1_bus */
		TZPC_NS_MODE	/* sfr2_bus */
	},
	{
		TZPC_NS_MODE,	/* static_bus */
		TZPC_NS_MODE,	/* vip2 VIP002 XPROT */
		TZPC_NS_MODE,	/* vip2clk */
		TZPC_S_MODE,	/* tmu */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	},
	{
		TZPC_NS_MODE,	/* uart01 */
		TZPC_NS_MODE,	/* uart00 */
		TZPC_NS_MODE,	/* uart02 */
		TZPC_NS_MODE,	/* uart03 */
		TZPC_NS_MODE,	/* i2c0 */
		TZPC_NS_MODE,	/* i2c1 */
		TZPC_NS_MODE,	/* i2c2 */
		TZPC_NS_MODE	/* ssp2clk */
	},
	{
		TZPC_NS_MODE,	/* uart01clk */
		TZPC_NS_MODE,	/* uart00clk */
		TZPC_NS_MODE,	/* uart02clk */
		TZPC_NS_MODE,	/* uart03clk */
		TZPC_NS_MODE,	/* ssp0clk */
		TZPC_NS_MODE,	/* ssp1clk */
		TZPC_NS_MODE,	/* i2c0clk */
		TZPC_NS_MODE	/* i2c1clk */
	},
	/*=============TZPC5================= */
	{
		TZPC_NS_MODE,	/* i2c2clk */
		TZPC_NS_MODE,	/* uart05clk */
		TZPC_NS_MODE,	/* i2s0clk */
		TZPC_NS_MODE,	/* i2s1clk */
		TZPC_NS_MODE,	/* i2s2clk */
		TZPC_NS_MODE,	/* deinterclk */
		TZPC_NS_MODE,	/* scalerclk */
		TZPC_NS_MODE	/* mpegtsiclk */
	},
	{
		TZPC_NS_MODE,	/* spdiftxclk */
		TZPC_NS_MODE,	/* pwmt0clk */
		TZPC_NS_MODE,	/* pwmt1clk */
		TZPC_NS_MODE,	/* timer01 */
		TZPC_NS_MODE,	/* timer02 */
		TZPC_NS_MODE,	/* timer03 */
		TZPC_NS_MODE,	/* pwm01 */
		TZPC_NS_MODE	/* pwm02 */
	},
	{
		TZPC_NS_MODE,	/* pwm03 */
		TZPC_NS_MODE,	/* vip1clk */
		TZPC_NS_MODE,	/* vip0clk */
		TZPC_NS_MODE,	/* maliclk */
		TZPC_NS_MODE,	/* ppmclk */
		TZPC_NS_MODE,	/* sdmmc0clk */
		TZPC_NS_MODE,	/* cryptoclk */
		TZPC_NS_MODE	/* codaclk */
	},
	{
		TZPC_NS_MODE,	/* gmacclk */
		TZPC_NS_MODE,	/* reserved */
		TZPC_NS_MODE,	/* mipitop clk */
		TZPC_NS_MODE,	/* pdmclk */
		TZPC_NS_MODE,	/* sdmmc1clk */
		TZPC_NS_MODE,	/* sdmmc2clk */
		TZPC_NS_MODE,	/* can0 */
		TZPC_NS_MODE	/* can1 */
	},
	/*=============TZPC6================= */
	{
		TZPC_S_MODE,	/* DREX */
		TZPC_S_MODE,	/* DDRPHY */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	},
	{
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	},
	{
		TZPC_NS_MODE,	/* HDMI_PHY */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	},
	{
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE,	/* NC */
		TZPC_S_MODE	/* NC */
	}
};

union tzpc_prot_attr {
	unsigned char		prot_data;
	struct tzpc_prot_bit	prot_bit;
};

static struct tzpc_prot_reg *get_tzpc_prot_reg(uint32_t base, int grp)
{
	uint64_t addr = base + 0x1000 * grp;

	return (struct tzpc_prot_reg *)addr;
}

static void set_TZPC(uint32_t tzpc_base)
{
	unsigned int i, j;
	struct tzpc_prot_reg *tzpc_prot_reg;
	union tzpc_prot_attr *tzpc_prot_attr =
		(union tzpc_prot_attr *)tzpc_prot_bit;

	for (i = 0; i < TZPC_GROUPNUM_MAX; i++) {
		tzpc_prot_reg = get_tzpc_prot_reg(tzpc_base, i);

		/* printf("i:%d, base:%X, size:%x\r\n",
		 * i, tzpc_prot_reg->r0size, sizeof(struct tzpc_prot_reg)); */
		tzpc_prot_reg->r0size = tzpc_r0size[i];
		for (j = 0; j < TZPC_PORTNUM_MAX; j++) {
			tzpc_prot_reg->decprot[j].clear	=
			  ~(tzpc_prot_attr[i*TZPC_PORTNUM_MAX+j].prot_data);
			tzpc_prot_reg->decprot[j].set	=
			  tzpc_prot_attr[i*TZPC_PORTNUM_MAX+j].prot_data;
		}
	}
}

static struct rgn_map_reg *get_rgn_map_reg(uint32_t base, int region,
						int port)
{
	uint64_t addr = base + 0x100 + 0x10 * region + 0x400 * port;

	return (struct rgn_map_reg *)addr;
}

static struct rgn_attr_reg *get_rgn_attr_reg(uint32_t base, int region,
						int port)
{
	uint64_t addr = base + 0x108 + 0x10 * region + 0x400 * port;

	return (struct rgn_attr_reg *)addr;
}

static int is_power_of_two(uint32_t x)
{
	return ((x != 0) && !(x & (x - 1)));
}

/*
 * Configure secure memory region
 * region_size must be a power of 2 and at least 64KB
 * region_base must be region_size aligned
 */
static void sec_protect(uint32_t region_base, uint32_t region_size)
{
	struct rgn_map_reg *rgn_map_reg;
	struct rgn_attr_reg *rgn_attr_reg;
	uint32_t i = 0;

	if (!is_power_of_two(region_size) || region_size < 0x10000) {
		ERROR("Secure region size is not a power of 2 >= 64KB\n");
		return;
	}
#if 0
	if (region_base & (region_size - 1)) {
		ERROR("Secure region address is not aligned to region size\n");
		return;
	}
#endif

	INFO("BL2: TrustZone: protecting %u bytes of memory at 0x%x\n",
			region_size, region_base);

	for (i = 0; i < PORTNUM_MAX; i++) {
		rgn_map_reg = get_rgn_map_reg(DREX_SECURITY_BASE, 1, i);
		rgn_attr_reg = get_rgn_attr_reg(DREX_SECURITY_BASE, 1, i);

		rgn_map_reg->rgn_base_addr = region_base >> 16;
		/* rgn_map_reg->rgn_base_addr =
		 *	(region_base - 0x40000000) >> 16; */
		/* rgn_attr_reg->sp = (i == 3) ? 0xC : 0x0; */
		/* rgn_attr_reg->sp = 0xF; */
		/* rgn_attr_reg->sp = 0x0; */
		rgn_attr_reg->sp = 0xC;
		rgn_attr_reg->rgn_size = region_size >> 16;
					/* (region_size / 64KB) */
		/* rgn_attr_reg->rgn_size = 0x7FFFF; */
		rgn_attr_reg->rgn_en = 0x1;
	}

	/* while(1); */
}

/*******************************************************************************
 * Initialize the secure environment.
 ******************************************************************************/
void plat_security_setup(void)
{
	set_TZPC(TZPC_BASE);
	sec_protect(DRAM_SEC_BASE, DRAM_SEC_SIZE);
}
