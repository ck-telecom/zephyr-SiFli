#
# SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0


add_subdirectory_ifdef(CONFIG_CLOCK_CONTROL clock_control)
add_subdirectory_ifdef(CONFIG_ENTROPY_GENERATOR entropy)
add_subdirectory_ifdef(CONFIG_FLASH flash)
add_subdirectory_ifdef(CONFIG_GPIO gpio)
add_subdirectory_ifdef(CONFIG_INPUT input)
add_subdirectory_ifdef(CONFIG_I2C i2c)
add_subdirectory_ifdef(CONFIG_PINCTRL pinctrl)
add_subdirectory_ifdef(CONFIG_RTC rtc)
add_subdirectory_ifdef(CONFIG_SERIAL serial)
add_subdirectory_ifdef(CONFIG_WATCHDOG watchdog)