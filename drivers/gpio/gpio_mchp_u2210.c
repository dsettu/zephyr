/*
 * Copyright (c) 2024 Microchip Technology Inc. and its subsidiaries.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_port_u2210

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/microchip-pic32c-gpio.h>
#include <soc.h>
#include <zephyr/drivers/interrupt_controller/pic32c_eic.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#ifndef PORT_PMUX_PMUXE_A_Val
#define PORT_PMUX_PMUXE_A_Val (0)
#endif

struct gpio_u22100_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	port_group_registers_t *regs;
#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254
	uint8_t id;
#endif
};

struct gpio_u22100_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	const struct device *dev;
	gpio_port_pins_t debounce;
#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254
	sys_slist_t callbacks;
#endif
};

#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254
static void gpio_u22100_isr(uint32_t pins, void *arg)
{
	struct gpio_u22100_data *const data = (struct gpio_u22100_data *)arg;

	gpio_fire_callbacks(&data->callbacks, data->dev, pins);
}
#endif

static int gpio_u22100_config(const struct device *dev, gpio_pin_t pin,
			    gpio_flags_t flags)
{
	const struct gpio_u22100_config *config = dev->config;
	struct gpio_u22100_data *data = dev->data;
	port_group_registers_t *regs = config->regs;
	uint8_t pincfg = 0;

	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		return -ENOTSUP;
	}

	/* Supports disconnected, input, output, or bidirectional */
	if ((flags & GPIO_INPUT) != 0) {
		pincfg |= PORT_PINCFG_INEN_Msk;
	}
	if ((flags & GPIO_OUTPUT) != 0) {
		/* Output is incompatible with pull */
		if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
			return -ENOTSUP;
		}
		/* Bidirectional is supported */
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			regs->PORT_OUTCLR = BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			regs->PORT_OUTSET = BIT(pin);
		}
		regs->PORT_DIRSET = BIT(pin);
	} else {
		/* Not output, may be input */
		regs->PORT_DIRCLR = BIT(pin);

		/* Pull configuration is supported if not output */
		if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
			pincfg |= PORT_PINCFG_PULLEN_Msk;
			if ((flags & GPIO_PULL_UP) != 0) {
				regs->PORT_OUTSET = BIT(pin);
			} else {
				regs->PORT_OUTCLR = BIT(pin);
			}
		}
	}

	/* Preserve debounce flag for interrupt configuration. */
	WRITE_BIT(data->debounce, pin,
		  ((flags & PIC32C_GPIO_DEBOUNCE) != 0)
		  && ((pincfg & PORT_PINCFG_INEN_Msk) != 0));

	/* Write the now-built pin configuration */
	regs->PORT_PINCFG[pin] = pincfg;

	return 0;
}

static int gpio_u22100_port_get_raw(const struct device *dev,
				  gpio_port_value_t *value)
{
	const struct gpio_u22100_config *config = dev->config;

	*value = config->regs->PORT_IN;

	return 0;
}

static int gpio_u22100_port_set_masked_raw(const struct device *dev,
					 gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_u22100_config *config = dev->config;
	uint32_t out = config->regs->PORT_OUT;

	config->regs->PORT_OUT = (out & ~mask) | (value & mask);

	return 0;
}

static int gpio_u22100_port_set_bits_raw(const struct device *dev,
				       gpio_port_pins_t pins)
{
	const struct gpio_u22100_config *config = dev->config;

	config->regs->PORT_OUTSET = pins;

	return 0;
}

static int gpio_u22100_port_clear_bits_raw(const struct device *dev,
					 gpio_port_pins_t pins)
{
	const struct gpio_u22100_config *config = dev->config;

	config->regs->PORT_OUTCLR = pins;

	return 0;
}

static int gpio_u22100_port_toggle_bits(const struct device *dev,
				      gpio_port_pins_t pins)
{
	const struct gpio_u22100_config *config = dev->config;

	config->regs->PORT_OUTTGL = pins;

	return 0;
}

#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254

static int gpio_u22100_pin_interrupt_configure(const struct device *dev,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	const struct gpio_u22100_config *config = dev->config;
	struct gpio_u22100_data *const data = dev->data;
	port_group_registers_t *regs = config->regs;
	uint8_t pincfg = regs->PORT_PINCFG[pin];

	enum pic32c_eic_trigger trigger;
	int rc = 0;

	data->dev = dev;

	switch (mode) {
	case GPIO_INT_MODE_DISABLED:
		pincfg &= ~PORT_PINCFG_PMUXEN_Msk;
		rc = pic32c_eic_disable_interrupt(config->id, pin);
		if (rc == -EBUSY) {
			/* Ignore diagnostic disabling disabled */
			rc = 0;
		}
		if (rc == 0) {
			rc = pic32c_eic_release(config->id, pin);
		}
		break;
	case GPIO_INT_MODE_LEVEL:
	case GPIO_INT_MODE_EDGE:
		/* Enabling interrupts on a pin requires disconnecting
		 * the pin from the I/O pin controller (PORT) module
		 * and connecting it to the External Interrupt
		 * Controller (EIC).  This would prevent using the pin
		 * as an output, so interrupts are only supported if
		 * the pin is configured as input-only.
		 */
		if (((pincfg & PORT_PINCFG_INEN_Msk) == 0)
		    || ((regs->PORT_DIR & BIT(pin)) != 0)) {
			rc = -ENOTSUP;
			break;
		}

		/* Transfer control to EIC */
		pincfg |= PORT_PINCFG_PMUXEN_Msk;

		if ((pin & 1U) != 0) {
			regs->PORT_PMUX[pin / 2U] = (regs->PORT_PMUX[pin / 2U] & ~PORT_PMUX_PMUXO_Msk) | PORT_PMUX_PMUXO_A;
		} else {
			regs->PORT_PMUX[pin / 2U] = (regs->PORT_PMUX[pin / 2U] & ~PORT_PMUX_PMUXE_Msk) | PORT_PMUX_PMUXE_A;
		}

		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			trigger = (mode == GPIO_INT_MODE_LEVEL)
				? PIC32C_EIC_LOW
				: PIC32C_EIC_FALLING;
			break;
		case GPIO_INT_TRIG_HIGH:
			trigger = (mode == GPIO_INT_MODE_LEVEL)
				? PIC32C_EIC_HIGH
				: PIC32C_EIC_RISING;
			break;
		case GPIO_INT_TRIG_BOTH:
			trigger = PIC32C_EIC_BOTH;
			break;
		default:
			rc = -EINVAL;
			break;
		}

		if (rc == 0) {
			rc = pic32c_eic_acquire(config->id, pin, trigger,
					      (data->debounce & BIT(pin)) != 0,
					      gpio_u22100_isr, data);
		}
		if (rc == 0) {
			rc = pic32c_eic_enable_interrupt(config->id, pin);
		}

		break;
	default:
		rc = -EINVAL;
		break;
	}

	if (rc == 0) {
		/* Update the pin configuration */
		regs->PORT_PINCFG[pin] = pincfg;
	}

	return rc;
}


static int gpio_u22100_manage_callback(const struct device *dev,
				     struct gpio_callback *callback, bool set)
{
	struct gpio_u22100_data *const data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static uint32_t gpio_u22100_get_pending_int(const struct device *dev)
{
	const struct gpio_u22100_config *config = dev->config;

	return pic32c_eic_interrupt_pending(config->id);
}

#endif

static const struct gpio_driver_api gpio_u22100_api = {
	.pin_configure = gpio_u22100_config,
	.port_get_raw = gpio_u22100_port_get_raw,
	.port_set_masked_raw = gpio_u22100_port_set_masked_raw,
	.port_set_bits_raw = gpio_u22100_port_set_bits_raw,
	.port_clear_bits_raw = gpio_u22100_port_clear_bits_raw,
	.port_toggle_bits = gpio_u22100_port_toggle_bits,
#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254
	.pin_interrupt_configure = gpio_u22100_pin_interrupt_configure,
	.manage_callback = gpio_u22100_manage_callback,
	.get_pending_int = gpio_u22100_get_pending_int,
#endif
};

static int gpio_u22100_init(const struct device *dev) { return 0; }

/* Port A */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porta), okay)

static const struct gpio_u22100_config gpio_u22100_config_0 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0),
	},
	.regs = (port_group_registers_t *)DT_REG_ADDR(DT_NODELABEL(porta)),
#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254
	.id = 0,
#endif
};

static struct gpio_u22100_data gpio_u22100_data_0;

DEVICE_DT_DEFINE(DT_NODELABEL(porta),
		    gpio_u22100_init, NULL,
		    &gpio_u22100_data_0, &gpio_u22100_config_0,
		    PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,
		    &gpio_u22100_api);
#endif

/* Port B */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portb), okay)

static const struct gpio_u22100_config gpio_u22100_config_1 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(1),
	},
	.regs = (port_group_registers_t *)DT_REG_ADDR(DT_NODELABEL(portb)),
#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254
	.id = 1,
#endif
};

static struct gpio_u22100_data gpio_u22100_data_1;

DEVICE_DT_DEFINE(DT_NODELABEL(portb),
		    gpio_u22100_init, NULL,
		    &gpio_u22100_data_1, &gpio_u22100_config_1,
		    PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,
		    &gpio_u22100_api);
#endif

/* Port C */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portc), okay)

static const struct gpio_u22100_config gpio_u22100_config_2 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(2),
	},
	.regs = (port_group_registers_t *)DT_REG_ADDR(DT_NODELABEL(portc)),
#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254
	.id = 2,
#endif
};

static struct gpio_u22100_data gpio_u22100_data_2;

DEVICE_DT_DEFINE(DT_NODELABEL(portc),
		    gpio_u22100_init, NULL,
		    &gpio_u22100_data_2, &gpio_u22100_config_2,
		    PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,
		    &gpio_u22100_api);
#endif

/* Port D */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portd), okay)

static const struct gpio_u22100_config gpio_u22100_config_3 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(3),
	},
	.regs = (port_group_registers_t *)DT_REG_ADDR(DT_NODELABEL(portd)),
#ifdef CONFIG_INTC_MICROCHIP_EIC_U2254
	.id = 3,
#endif
};

static struct gpio_u22100_data gpio_u22100_data_3;

DEVICE_DT_DEFINE(DT_NODELABEL(portd),
		    gpio_u22100_init, NULL,
		    &gpio_u22100_data_3, &gpio_u22100_config_3,
		    PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,
		    &gpio_u22100_api);
#endif
