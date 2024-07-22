/*
# Copyright (c) 2024 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: Apache-2.0
*/

#ifndef _PIC32CM_CM00_SOC_H_
#define _PIC32CM_CM00_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>
#include "../common/soc_dt.h"

#include <pic32cm1216mc00048.h>

#endif /* _ASMLANGUAGE */

#define SOC_ADC_REFERENCE_ENABLE_PROTECTED

//#include "adc_fixup_sam0.h"
#include "../common/soc_port.h"
#include "../common/soc_dt.h"

#define SOC_OSC32K_FREQ_HZ 32768
#define SOC_OSC48M_FREQ_HZ 48000000

/** Processor Clock (HCLK) Frequency */
#define SOC_HCLK_FREQ_HZ CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
/** Master Clock (MCK) Frequency */
#define SOC_MCK_FREQ_HZ SOC_HCLK_FREQ_HZ
#define SOC_GCLK0_FREQ_HZ SOC_MCK_FREQ_HZ
#define SOC_UART_CLOCK_FREQ_HZ	SOC_GCLK0_FREQ_HZ
#endif /* _PIC32CM_CM00_SOC_H_ */
