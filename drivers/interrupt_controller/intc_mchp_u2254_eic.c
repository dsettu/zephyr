/*
 * Copyright (c) 2024 Microchip Technology Inc. and its subsidiaries.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_eic_u2254

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <zephyr/drivers/interrupt_controller/pic32c_eic.h>
#include "intc_mchp_u2254_eic_priv.h"

struct pic32c_eic_line_assignment {
	uint8_t pin : 5;
	uint8_t port : 2;
	uint8_t enabled : 1;
};

struct pic32c_eic_port_data {
	pic32c_eic_callback_t cb;
	void *data;
};

struct pic32c_eic_data {
	struct pic32c_eic_port_data ports[PORT_GROUPS];
	struct pic32c_eic_line_assignment lines[EIC_EXTINT_NUM];
};

static void wait_synchronization(void)
{
#ifdef EIC_SYNCBUSY_Msk
	while (EIC_REGS->EIC_SYNCBUSY) {
	}
#else
	while (EIC_REGS->EIC_STATUS & (uint8_t)EIC_STATUS_SYNCBUSY_Msk) == (uint8_t)EIC_STATUS_SYNCBUSY_Msk) {
	}
#endif
}

static inline void set_eic_enable(bool on)
{
#ifdef EIC_CTRLA_Msk
	EIC_REGS->EIC_CTRLA = (EIC_REGS->EIC_CTRLA & ~EIC_CTRLA_ENABLE_Msk) | EIC_CTRLA_ENABLE(on);
#else
	EIC_REGS->EIC_CTRL = (EIC_REGS->EIC_CTRL & ~EIC_CTRL_ENABLE_Msk) | EIC_CTRL_ENABLE(on);
#endif
}

static void pic32c_eic_isr(const struct device *dev)
{
	struct pic32c_eic_data *const dev_data = dev->data;
	uint16_t bits = EIC_REGS->EIC_INTFLAG;
	uint32_t line_index;

	/* Acknowledge all interrupts */
	EIC_REGS->EIC_INTFLAG = bits;

	/* No clz on M0, so just do a quick test */
#if __CORTEX_M >= 3
	line_index = __CLZ(__RBIT(bits));
	bits >>= line_index;
#else
	if (bits & 0xFF) {
		line_index = 0;
	} else {
		line_index = 8;
		bits >>= 8;
	}
#endif

	/*
	 * Map the EIC lines to the port pin masks based on which port is
	 * selected in the line data.
	 */
	for (; bits; bits >>= 1, line_index++) {
		if (!(bits & 1)) {
			continue;
		}

		/*
		 * These could be aggregated together into one call, but
		 * usually on a single one will be set, so just call them
		 * one by one.
		 */
		struct pic32c_eic_line_assignment *line_assignment =
			&dev_data->lines[line_index];
		struct pic32c_eic_port_data *port_data =
			&dev_data->ports[line_assignment->port];

		port_data->cb(BIT(line_assignment->pin), port_data->data);
	}
}

int pic32c_eic_acquire(int port, int pin, enum pic32c_eic_trigger trigger,
		     bool filter, pic32c_eic_callback_t cb, void *data)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct pic32c_eic_data *dev_data = dev->data;
	struct pic32c_eic_port_data *port_data;
	struct pic32c_eic_line_assignment *line_assignment;
	uint32_t mask;
	int line_index;
	int config_index;
	int config_shift;
	unsigned int key;
	uint32_t config;

	line_index = pic32c_eic_map_to_line(port, pin);
	if (line_index < 0) {
		return line_index;
	}

	mask = BIT(line_index);
	config_index = line_index / 8;
	config_shift = (line_index % 8) * 4;

	/* Lock everything so it's safe to reconfigure */
	key = irq_lock();
	/* Disable the EIC for reconfiguration */
	set_eic_enable(0);

	line_assignment = &dev_data->lines[line_index];

	/* Check that the required line is available */
	if (line_assignment->enabled) {
		if (line_assignment->port != port ||
		    line_assignment->pin != pin) {
			goto err_in_use;
		}
	}

	/* Set the EIC configuration data */
	port_data = &dev_data->ports[port];
	port_data->cb = cb;
	port_data->data = data;
	line_assignment->pin = pin;
	line_assignment->port = port;
	line_assignment->enabled = 1;
	config = 0;

#if EIC_CONFIG_Msk
	config = EIC_REGS->EIC_CONFIG[config_index];
#else
	if (config_index == 0)
		config = EIC_REGS->EIC_CONFIG0;
	else if (config_index == 1)
		config = EIC_REGS->EIC_CONFIG1;
#endif
	config &= ~(0xF << config_shift);
	switch (trigger) {
	case PIC32C_EIC_RISING:
		config |= EIC_CONFIG0_SENSE0_RISE << config_shift;
		break;
	case PIC32C_EIC_FALLING:
		config |= EIC_CONFIG0_SENSE0_FALL << config_shift;
		break;
	case PIC32C_EIC_BOTH:
		config |= EIC_CONFIG0_SENSE0_BOTH << config_shift;
		break;
	case PIC32C_EIC_HIGH:
		config |= EIC_CONFIG0_SENSE0_HIGH << config_shift;
		break;
	case PIC32C_EIC_LOW:
		config |= EIC_CONFIG0_SENSE0_LOW << config_shift;
		break;
	}

	if (filter) {
		config |= EIC_CONFIG0_FILTEN0_Msk << config_shift;
	}

#if EIC_CONFIG_Msk
	/* Apply the config to the EIC itself */
	EIC_REGS->EIC_CONFIG[config_index] = config;
#else
	if (config_index == 0)
		EIC_REGS->EIC_CONFIG0 = config ;
	else if (config_index == 1)
		EIC_REGS->EIC_CONFIG1 = config ;
#endif	

	set_eic_enable(1);
	wait_synchronization();
	/*
	 * Errata: The EIC generates a spurious interrupt for the newly
	 * enabled pin after being enabled, so clear it before re-enabling
	 * the IRQ.
	 */
	EIC_REGS->EIC_INTFLAG = mask;
	irq_unlock(key);
	return 0;

err_in_use:
	set_eic_enable(1);
	wait_synchronization();
	irq_unlock(key);
	return -EBUSY;
}

static bool pic32c_eic_check_ownership(int port, int pin, int line_index)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct pic32c_eic_data *dev_data = dev->data;
	struct pic32c_eic_line_assignment *line_assignment =
		&dev_data->lines[line_index];

	if (!line_assignment->enabled) {
		return false;
	}

	if (line_assignment->port != port ||
	    line_assignment->pin != pin) {
		return false;
	}

	return true;
}

int pic32c_eic_release(int port, int pin)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct pic32c_eic_data *dev_data = dev->data;
	uint32_t mask;
	int line_index;
	int config_index;
	int config_shift;
	unsigned int key;

	line_index = pic32c_eic_map_to_line(port, pin);
	if (line_index < 0) {
		return line_index;
	}

	mask = BIT(line_index);
	config_index = line_index / 8;
	config_shift = (line_index % 8) * 4;

	/* Lock everything so it's safe to reconfigure */
	key = irq_lock();
	/* Disable the EIC */
	set_eic_enable(0);
	wait_synchronization();

	/*
	 * Check to make sure the requesting actually owns the line and do
	 * nothing if it does not.
	 */
	if (!pic32c_eic_check_ownership(port, pin, line_index)) {
		goto done;
	}

	dev_data->lines[line_index].enabled = 0;


#if EIC_CONFIG_Msk
	/* Clear the EIC config, including the trigger condition */
	EIC_REGS->EIC_CONFIG[config_index] &= ~(0xF << config_shift);	
#else
	if (config_index == 0)
		EIC_REGS->EIC_CONFIG0 &= ~(0xF << config_shift);	
	else if (config_index == 1)
		EIC_REGS->EIC_CONFIG1 &= ~(0xF << config_shift);	
#endif	



	/* Clear any pending interrupt for it */
	EIC_REGS->EIC_INTENCLR = mask;
	EIC_REGS->EIC_INTENCLR = mask;

done:
	set_eic_enable(1);
	wait_synchronization();
	irq_unlock(key);
	return 0;
}

int pic32c_eic_enable_interrupt(int port, int pin)
{
	uint32_t mask;
	int line_index;

	line_index = pic32c_eic_map_to_line(port, pin);
	if (line_index < 0) {
		return line_index;
	}

	if (!pic32c_eic_check_ownership(port, pin, line_index)) {
		return -EBUSY;
	}

	mask = BIT(line_index);
	EIC_REGS->EIC_INTFLAG = mask;
	EIC_REGS->EIC_INTENSET = mask;

	return 0;
}

int pic32c_eic_disable_interrupt(int port, int pin)
{
	uint32_t mask;
	int line_index;

	line_index = pic32c_eic_map_to_line(port, pin);
	if (line_index < 0) {
		return line_index;
	}

	if (!pic32c_eic_check_ownership(port, pin, line_index)) {
		return -EBUSY;
	}

	mask = BIT(line_index);
	EIC_REGS->EIC_INTENCLR = mask;
	EIC_REGS->EIC_INTFLAG = mask;

	return 0;
}

uint32_t pic32c_eic_interrupt_pending(int port)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct pic32c_eic_data *dev_data = dev->data;
	struct pic32c_eic_line_assignment *line_assignment;
	uint32_t set = EIC_REGS->EIC_INTFLAG;
	uint32_t mask = 0;

	for (int line_index = 0; line_index < EIC_EXTINT_NUM; line_index++) {
		line_assignment = &dev_data->lines[line_index];

		if (!line_assignment->enabled) {
			continue;
		}

		if (line_assignment->port != port) {
			continue;
		}

		if (!(set & BIT(line_index))) {
			continue;
		}

		mask |= BIT(line_assignment->pin);
	}

	return mask;
}


#define PIC32C_EIC_IRQ_CONNECT(n)						\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, n, irq),		\
			    DT_INST_IRQ_BY_IDX(0, n, priority),		\
			    pic32c_eic_isr, DEVICE_DT_INST_GET(0), 0);	\
		irq_enable(DT_INST_IRQ_BY_IDX(0, n, irq));		\
	} while (false)

static int pic32c_eic_init(const struct device *dev)
{
	ARG_UNUSED(dev);

#ifdef MCLK_REGS
	/* Enable the EIC clock in APBAMASK */
	MCLK_REGS->MCLK_APBAMASK |= MCLK_APBAMASK_EIC_Msk;

	/* Enable the GCLK */
	GCLK_REGS->GCLK_PCHCTRL[EIC_GCLK_ID] = GCLK_PCHCTRL_GEN_GCLK0 |
					 GCLK_PCHCTRL_CHEN_Msk;
#else
	/* Enable the EIC clock in PM */
	PM_REGS->PM_APBAMASK | = PM_APBAMASK_EIC_Msk;

	/* Enable the GCLK */
	GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID_EIC | GCLK_CLKCTRL_GEN_GCLK0 |
			    GCLK_CLKCTRL_CLKEN_Msk;
#endif

#if DT_INST_IRQ_HAS_CELL(0, irq)
	PIC32C_EIC_IRQ_CONNECT(0);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 1)
	PIC32C_EIC_IRQ_CONNECT(1);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 2)
	PIC32C_EIC_IRQ_CONNECT(2);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 3)
	PIC32C_EIC_IRQ_CONNECT(3);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 4)
	PIC32C_EIC_IRQ_CONNECT(4);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 5)
	PIC32C_EIC_IRQ_CONNECT(5);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 6)
	PIC32C_EIC_IRQ_CONNECT(6);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 7)
	PIC32C_EIC_IRQ_CONNECT(7);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 8)
	PIC32C_EIC_IRQ_CONNECT(8);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 9)
	PIC32C_EIC_IRQ_CONNECT(9);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 10)
	PIC32C_EIC_IRQ_CONNECT(10);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 11)
	PIC32C_EIC_IRQ_CONNECT(11);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 12)
	PIC32C_EIC_IRQ_CONNECT(12);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 13)
	PIC32C_EIC_IRQ_CONNECT(13);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 14)
	PIC32C_EIC_IRQ_CONNECT(14);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 15)
	PIC32C_EIC_IRQ_CONNECT(15);
#endif

	set_eic_enable(1);
	wait_synchronization();

	return 0;
}

static struct pic32c_eic_data eic_data;
DEVICE_DT_INST_DEFINE(0, pic32c_eic_init,
	      NULL, &eic_data, NULL,
	      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,
	      NULL);
