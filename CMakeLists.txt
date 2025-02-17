# Makefile - STM32Cube SDK
#
# Copyright (c) 2020 Linaro Limited
#
# SPDX-License-Identifier: Apache-2.0
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/zephyr)
set(SOC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/zephyr)
set(EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/zephyr)
add_subdirectory_ifdef(CONFIG_HAS_SF32LB boards)
add_subdirectory_ifdef(CONFIG_HAS_SF32LB drivers)
add_subdirectory_ifdef(CONFIG_HAS_SF32LB zephyr/drivers)
