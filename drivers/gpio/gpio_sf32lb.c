/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bf0_hal_gpio.h"

LOG_MODULE_REGISTER(gpio_sf32lb, CONFIG_GPIO_LOG_LEVEL);

struct gpio_sf32lb_config {
	struct gpio_driver_config common;
	GPIO_TypeDef *reg_base;
	uint8_t irq_num;
	uint8_t irq_priority;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	const int port;
	void (*irq_cfg_func)(void);
};

struct gpio_sf32lb_data {
	struct k_mutex lock;
	uint32_t pin_state;
	/* user ISR cb */
	sys_slist_t cb;
	/* pin callback routine enable flags, by pin number */
	uint32_t pin_callback_enables;
};

static void gpio_sf32lb_isr(const struct device *dev)
{
	const struct gpio_sf32lb_config *config = dev->config;
	struct gpio_sf32lb_data *data = dev->data;
	GPIO_TypeDef *gpiox = config->reg_base + config->port;
	const int port_pin_mask = config->common.port_pin_mask;
	uint32_t int_status;

    if (!(config->reg_base == hwp_gpio1)) {
        int_status = gpiox->ISR_EXT;

        gpiox->ISR_EXT |= int_status;
    } else {
		int_status = gpiox->ISR;

		gpiox->ISR |= int_status;
	}

	/* Call the user callback */
	gpio_fire_callbacks(&data->cb, dev, int_status);
}

static int gpio_sf32lb_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_sf32lb_config *config = dev->config;
	GPIO_InitTypeDef GPIO_Init;
	uint32_t io_pin = pin + ((config->port == 1 && pin < 32) ? 32 : 0);
	GPIO_TypeDef *gpiox = config->reg_base + config->port;
	const int port_pin_mask = config->common.port_pin_mask;
	GPIO_Init.Pin = io_pin;
	GPIO_Init.Pull = GPIO_NOPULL;

	if ((port_pin_mask & BIT(pin)) == 0) {
		return -EINVAL;
	}

	HAL_PIN_Set(PAD_PA00 + io_pin, GPIO_A0 + io_pin, PIN_NOPULL, 1);

	if ((flags & GPIO_OUTPUT) && !(flags & GPIO_INPUT)) {
		GPIO_Init.Mode = GPIO_MODE_OUTPUT;
		if ((flags & GPIO_PULL_UP) != 0) {
			GPIO_Init.Pull = GPIO_PULLUP;
		} else if ((flags & GPIO_PULL_DOWN) != 0) {
			GPIO_Init.Pull = GPIO_PULLDOWN;
		} else {
			GPIO_Init.Pull = GPIO_NOPULL;
		}
		HAL_GPIO_Init(gpiox, &GPIO_Init);
	} else if ((flags & GPIO_INPUT) && !(flags & GPIO_OUTPUT)) {
		GPIO_Init.Mode = GPIO_MODE_INPUT;

		if (flags & GPIO_PULL_UP) {
			GPIO_Init.Pull = GPIO_PULLUP;
		} else if (flags & GPIO_PULL_DOWN) {
			GPIO_Init.Pull = GPIO_PULLDOWN;
		} else {
			GPIO_Init.Pull = GPIO_NOPULL;
		}
		HAL_GPIO_Init(gpiox, &GPIO_Init);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_sf32lb_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_sf32lb_config *config = dev->config;
	GPIO_TypeDef *gpiox = config->reg_base + config->port;

	*value = gpiox->DOR;

	return 0;
}

static int gpio_sf32lb_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
	const struct gpio_sf32lb_config *config = dev->config;
	struct gpio_sf32lb_data *data = dev->data;
	GPIO_TypeDef *gpiox = config->reg_base + config->port;

	k_mutex_lock(&data->lock, K_FOREVER);

	data->pin_state = (data->pin_state & ~mask) | (value & mask);
	gpiox->DOR = data->pin_state;

	k_mutex_unlock(&data->lock);

	return 0;
}

static int gpio_sf32lb_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_sf32lb_config *config = dev->config;
	GPIO_TypeDef *gpiox = config->reg_base + config->port;

	gpiox->DOSR = mask;

	return 0;
}

static int gpio_sf32lb_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_sf32lb_config *config = dev->config;
	GPIO_TypeDef *gpiox = config->reg_base + config->port;

	gpiox->DOCR = mask;

	return 0;
}

static int gpio_sf32lb_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	const struct gpio_sf32lb_config *config = dev->config;
	struct gpio_sf32lb_data *data = dev->data;
	GPIO_TypeDef *gpiox = config->reg_base + config->port;
	const int port_pin_mask = config->common.port_pin_mask;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->pin_state ^= mask;//TODO: remove
	gpiox->DOR = mask;
	k_mutex_unlock(&data->lock);

	return 0;
}

static int gpio_sf32lb_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_sf32lb_config *config = dev->config;
	struct gpio_sf32lb_data *data = dev->data;
	uint32_t io_pin = pin + ((config->port == 1 && pin < 32) ? 32 : 0);
	GPIO_TypeDef *gpiox = config->reg_base + config->port;
	const int port_pin_mask = config->common.port_pin_mask;
	GPIO_InitTypeDef GPIO_Init;
	GPIO_Init.Pin = io_pin;
	GPIO_Init.Pull = GPIO_NOPULL;

	if ((port_pin_mask & BIT(pin)) == 0) {
		return -EINVAL;
	}

	/* Disable interrupt for the pin first */
	if (config->reg_base == hwp_gpio1) {
		gpiox->IECR = BIT(pin);
	} else {
		gpiox->IECR_EXT = BIT(pin);
	}
	data->pin_callback_enables &= ~BIT(pin);
	switch (mode) {
	case GPIO_INT_MODE_DISABLED:
		break;

	case GPIO_INT_MODE_LEVEL:
		if (trig == GPIO_INT_TRIG_LOW) {
			GPIO_Init.Mode = GPIO_MODE_IT_LOW_LEVEL;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			GPIO_Init.Mode = GPIO_MODE_IT_HIGH_LEVEL;
		}
		HAL_GPIO_Init(gpiox, &GPIO_Init);

		if (config->reg_base == hwp_gpio1) {
			gpiox->IESR = BIT(pin);
		} else {
			gpiox->IESR_EXT = BIT(pin);
		}
		data->pin_callback_enables |= BIT(pin);
		break;

	case GPIO_INT_MODE_EDGE:
		if (trig == GPIO_INT_TRIG_LOW) {
			GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			GPIO_Init.Mode = GPIO_MODE_IT_RISING;
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			GPIO_Init.Mode = GPIO_MODE_IT_RISING_FALLING;
		}
		HAL_GPIO_Init(gpiox, &GPIO_Init);

		if (config->reg_base == hwp_gpio1) {
			gpiox->IESR = BIT(pin);
		} else {
			gpiox->IESR_EXT = BIT(pin);
		}
		data->pin_callback_enables |= BIT(pin);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_sf32lb_manage_callback(const struct device *dev, struct gpio_callback *cb, bool set)
{
	struct gpio_sf32lb_data *data = dev->data;

	return gpio_manage_callback(&data->cb, cb, set);
}

static const struct gpio_driver_api gpio_sf32lb_driver_api = {
	.pin_configure = gpio_sf32lb_configure,
	.port_get_raw = gpio_sf32lb_port_get_raw,
	.port_set_masked_raw = gpio_sf32lb_port_set_masked_raw,
	.port_set_bits_raw = gpio_sf32lb_port_set_bits_raw,
	.port_clear_bits_raw = gpio_sf32lb_port_clear_bits_raw,
	.port_toggle_bits = gpio_sf32lb_port_toggle_bits,
	.pin_interrupt_configure = gpio_sf32lb_pin_interrupt_configure,
	.manage_callback = gpio_sf32lb_manage_callback,
};

static int gpio_sf32lb_init(const struct device *dev)
{
	int ret = 0;
	const struct gpio_sf32lb_config *config = dev->config;
	ret = clock_control_on(config->clock_dev, config->clock_subsys);

	config->irq_cfg_func();

	return ret;
}

#define SF32LB_GPIO_INIT(n)                                                                        \
	static struct gpio_sf32lb_data gpio_sf32lb_data_##n = {                                    \
		.lock = Z_MUTEX_INITIALIZER(gpio_sf32lb_data_##n.lock),                            \
		.pin_state = 0,                                                                    \
		.pin_callback_enables = 0,                                                         \
	};                                                                                         \
                                                                                                   \
	static void gpio_sf32lb_irq_config_##n(void);                                              \
                                                                                                   \
	static const struct gpio_sf32lb_config gpio_sf32lb_config_##n = {                          \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.reg_base = (GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpioa)),                      \
		.irq_num = DT_INST_IRQN(n),                                                        \
		.irq_priority = DT_INST_IRQ(n, priority),                                          \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                \
		.port = n,                                                                         \
		.irq_cfg_func = gpio_sf32lb_irq_config_##n,                                        \
	};                                                                                         \
	static void gpio_sf32lb_irq_config_##n(void)                                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), gpio_sf32lb_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(n, gpio_sf32lb_init, NULL, &gpio_sf32lb_data_##n,                    \
			      &gpio_sf32lb_config_##n, POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,     \
			      &gpio_sf32lb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SF32LB_GPIO_INIT)
