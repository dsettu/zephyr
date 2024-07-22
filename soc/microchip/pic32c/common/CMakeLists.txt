# Makefile - Atmel SAM0 MCU family
#
# Copyright (c) 2019 ML!PA Consulting GmbH
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(soc_port.c)


zephyr_sources_ifdef(CONFIG_SOC_SERIES_PIC32CM_MC00 soc_pic32cm_mc00.c)
zephyr_include_directories(.)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
