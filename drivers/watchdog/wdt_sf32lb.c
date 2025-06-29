/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_watchdog

#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/clock_control.h>

#include "bf0_hal_wdt.h"

LOG_MODULE_REGISTER(wdt_sf32lb, CONFIG_WDT_LOG_LEVEL);

struct wdt_sf32lb_config {
	WDT_TypeDef *reg;
};

struct wdt_sf32lb_data {
	uint32_t timeout;
	bool window_enabled;
	bool reset_enabled;
	WDT_HandleTypeDef hwdt;
};

static int wdt_sf32lb_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_sf32lb_config *config = dev->config;
	struct wdt_sf32lb_data *data = dev->data;
	WDT_HandleTypeDef *hwdt = &data->hwdt;

	hwdt->Init.Reload = data->timeout;
	hwdt->Instance = config->reg;
	if (HAL_WDT_Init(hwdt) != HAL_OK) {
		LOG_ERR("HAL_WDT_Init failed");
		return -EIO;
	}
	return 0;
}

static int wdt_sf32lb_disable(const struct device *dev)
{
	struct wdt_sf32lb_data *data = dev->data;
	__HAL_WDT_STOP(&data->hwdt);
	return 0;
}

static int wdt_sf32lb_install_timeout(const struct device *dev,
					const struct wdt_timeout_cfg *cfg)
{
	struct wdt_sf32lb_data *data = dev->data;
	if (cfg->window.min != 0) {
		return -ENOTSUP;
	}
	if (cfg->window.max > 0x0FFFFFF) {
		return -EINVAL;
	}
	data->timeout = cfg->window.max;
	data->window_enabled = (cfg->window.min != 0);
	data->reset_enabled = (cfg->flags & WDT_FLAG_RESET_SOC) != 0;
	return 0;
}

static int wdt_sf32lb_feed(const struct device *dev, int channel_id)
{
	struct wdt_sf32lb_data *data = dev->data;
	if (HAL_WDT_Refresh(&data->hwdt) != HAL_OK) {
		LOG_ERR("HAL_WDT_Refresh failed");
		return -EIO;
	}
	return 0;
}

static const struct wdt_driver_api wdt_sf32lb_api = {
	.setup = wdt_sf32lb_setup,
	.disable = wdt_sf32lb_disable,
	.install_timeout = wdt_sf32lb_install_timeout,
	.feed = wdt_sf32lb_feed,
};

static int wdt_sf32lb_init(const struct device *dev)
{
	const struct wdt_sf32lb_config *config = dev->config;
	struct wdt_sf32lb_data *data = dev->data;
	WDT_HandleTypeDef *hwdt = &data->hwdt;

	data->timeout = 0x0FFFFFF;  // Default max
	data->window_enabled = false;
	data->reset_enabled = true;
	hwdt->Instance = config->reg;
	hwdt->Init.Reload = data->timeout;
	if (HAL_WDT_Init(hwdt) != HAL_OK) {
		LOG_ERR("HAL_WDT_Init failed");
		return -EIO;
	}
	LOG_INF("Watchdog initialized with timeout %u", data->timeout);
	return 0;
}

#define WDT_SF32LB_INIT(n) \
	static struct wdt_sf32lb_data wdt_sf32lb_data_##n; \
	static const struct wdt_sf32lb_config wdt_sf32lb_config_##n = { \
		.reg = (WDT_TypeDef *)DT_INST_REG_ADDR(n), \
	}; \
	DEVICE_DT_INST_DEFINE(n, \
				wdt_sf32lb_init, \
				NULL, \
				&wdt_sf32lb_data_##n, \
				&wdt_sf32lb_config_##n, \
				POST_KERNEL, \
				CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
				&wdt_sf32lb_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_SF32LB_INIT)
