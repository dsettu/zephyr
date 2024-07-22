/*
 * Copyright (c) 2016 Piotr Mienkowski
 * Copyright (c) 2018 Google LLC.
 * Copyright (c) 2021 Gerson Fernando Budke
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Atmel SAM0 MCU family I/O Pin Controller (PORT)
 */

#include <stdbool.h>

#include "soc_port.h"

int soc_port_pinmux_set(port_group_registers_t *pg, uint32_t pin, uint32_t func)
{
	bool is_odd = pin & 1;
	int idx = pin / 2U;

	/* Each pinmux register holds the config for two pins.  The
	 * even numbered pin goes in the bits 0..3 and the odd
	 * numbered pin in bits 4..7.
	 */
	if (is_odd) {
		pg->PORT_PMUX[idx] = (pg->PORT_PMUX[idx] & ~PORT_PMUX_PMUXO_Msk) | PORT_PMUX_PMUXO(func);
	} else {
		pg->PORT_PMUX[idx] = (pg->PORT_PMUX[idx] & ~PORT_PMUX_PMUXE_Msk) | PORT_PMUX_PMUXE(func);
	}
	pg->PORT_PINCFG[pin] |= PORT_PINCFG_PMUXEN_Msk;

	return 0;
}

void soc_port_configure(const struct soc_port_pin *pin)
{
	port_group_registers_t *pg = pin->regs;
	uint32_t flags = pin->flags;
	uint32_t func = (pin->flags & SOC_PORT_FUNC_MASK) >> SOC_PORT_FUNC_POS;
	uint8_t pincfg =0;

	/* Reset or analog I/O: all digital disabled */
	pg->PORT_PINCFG[pin->pinum] = pincfg;
	pg->PORT_DIRCLR = (1 << pin->pinum);
	pg->PORT_OUTCLR = (1 << pin->pinum);

	if (flags & SOC_PORT_PMUXEN_ENABLE) {
		soc_port_pinmux_set(pg, pin->pinum, func);
		return;
	}

	if (flags & (SOC_PORT_PULLUP | SOC_PORT_PULLDOWN)) {
		if (flags & SOC_PORT_PULLUP) {
			pg->PORT_OUTSET = (1 << pin->pinum);
		}

		pincfg |= PORT_PINCFG_PULLEN_Msk;
	}

	if (flags & SOC_PORT_INPUT_ENABLE) {
		pincfg |= PORT_PINCFG_INEN_Msk;
	}

	if (flags & SOC_PORT_OUTPUT_ENABLE) {
		pg->PORT_DIRSET = (1 << pin->pinum);
	}

	if (flags & SOC_PORT_STRENGTH_STRONGER) {
		pincfg |= PORT_PINCFG_DRVSTR_Msk;
	}

	pg->PORT_PINCFG[pin->pinum] = pincfg;
}

void soc_port_list_configure(const struct soc_port_pin pins[],
			     unsigned int size)
{
	for (int i = 0; i < size; i++) {
		soc_port_configure(&pins[i]);
	}
}
