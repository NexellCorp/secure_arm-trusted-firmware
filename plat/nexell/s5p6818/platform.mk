#
# Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# On s5p6818, the TSP can execute either from Trusted SRAM or Trusted DRAM.
# Trusted DRAM is the default.
#
PLAT_TSP_LOCATION	:=	tdram
ifeq (${PLAT_TSP_LOCATION}, tsram)
  PLAT_TSP_LOCATION_ID := PLAT_TRUSTED_SRAM_ID
else ifeq (${PLAT_TSP_LOCATION}, tdram)
  PLAT_TSP_LOCATION_ID := PLAT_TRUSTED_DRAM_ID
else
  $(error "Unsupported PLAT_TSP_LOCATION value")
endif

# Process flags
$(eval $(call add_define,PLAT_TSP_LOCATION_ID))


NEXELL_PLAT		:=	plat/nexell
NEXELL_PLAT_SOC		:=	${NEXELL_PLAT}/${PLAT}

PLAT_INCLUDES		:=	-I${NEXELL_PLAT_SOC}/include/			\
				-I${NEXELL_PLAT_SOC}/drivers/			\
				-I${NEXELL_PLAT_SOC}/

PLAT_BL_COMMON_SOURCES	:=	lib/aarch64/xlat_tables.c			\
				plat/common/aarch64/plat_common.c		\
				plat/common/plat_gic.c				\
				drivers/io/io_fip.c				\
				drivers/io/io_memmap.c				\
				drivers/io/io_storage.c				\
				${NEXELL_PLAT_SOC}/aarch64/platform_common.c	\
				${NEXELL_PLAT_SOC}/aarch64/plat_helpers.S	\
				${NEXELL_PLAT_SOC}/drivers/uart_console.S	\
				${NEXELL_PLAT_SOC}/plat_io_storage.c
#				drivers/io/io_block.c				\

BL1_SOURCES		+=	drivers/arm/cci/cci.c				\
				drivers/console/console.S			\
				lib/cpus/aarch64/cortex_a53.S			\
				plat/common/aarch64/platform_up_stack.S		\
				${NEXELL_PLAT_SOC}/aarch64/bl1_plat_helpers.S	\
				${NEXELL_PLAT_SOC}/drivers/s5p6818_timer.c	\
				${NEXELL_PLAT_SOC}/bl1_plat_setup.c
#				${NEXELL_PLAT_SOC}/usb.c			\
#				${NEXELL_PLAT_SOC}/drivers/dw_mmc.c		\
#				${NEXELL_PLAT_SOC}/pll.c			\
#				drivers/arm/gpio/gpio.c				\
#				${NEXELL_PLAT_SOC}/partitions.c			\

BL2_SOURCES		+=	plat/common/aarch64/platform_up_stack.S		\
				drivers/console/console.S			\
				${NEXELL_PLAT_SOC}/bl2_plat_setup.c		\
				${NEXELL_PLAT_SOC}/plat_security.c		\
				${NEXELL_PLAT_SOC}/drivers/nx_rstcon.c		\
				${NEXELL_PLAT_SOC}/drivers/nx_clkpwr.c		\
				${NEXELL_PLAT_SOC}/drivers/dw_sd.c		\
				${NEXELL_PLAT_SOC}/sign/bignum.c		\
				${NEXELL_PLAT_SOC}/sign/sha256.c		\
				${NEXELL_PLAT_SOC}/sign/rsa_verify_pss.c
#				${NEXELL_PLAT_SOC}/drivers/dw_mmc.c		\
				${NEXELL_PLAT_SOC}/drivers/s5p6818_timer.c
#				${NEXELL_PLAT_SOC}/partitions.c

BL31_SOURCES		+=	drivers/arm/cci/cci.c				\
				drivers/arm/gic/arm_gic.c			\
				drivers/arm/gic/gic_v2.c			\
				drivers/arm/gic/gic_v3.c			\
				drivers/console/console.S			\
				lib/cpus/aarch64/cortex_a53.S			\
				plat/common/aarch64/platform_mp_stack.S		\
				${NEXELL_PLAT_SOC}/bl31_plat_setup.c		\
				${NEXELL_PLAT_SOC}/drivers/s5p6818_timer.c	\
				${NEXELL_PLAT_SOC}/plat_pm.c			\
				${NEXELL_PLAT_SOC}/plat_sip.c			\
				${NEXELL_PLAT_SOC}/plat_topology.c		\
				${NEXELL_PLAT_SOC}/drivers/nx_clkpwr.c		\
				${NEXELL_PLAT_SOC}/drivers/init_DDR3.c

# Flag used by the S5P6818_platform port to determine the version of ARM GIC
# architecture to use for interrupt management in EL3.
ARM_GIC_ARCH		:=	2
$(eval $(call add_define,ARM_GIC_ARCH))

# Enable workarounds for selected Cortex-A53 erratas.
ERRATA_A53_836870	:=	1

# indicate the reset vector address can be programmed
PROGRAMMABLE_RESET_ADDRESS	:=	1
