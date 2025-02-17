# Copyright (c) 2024 SiFli
# SPDX-License-Identifier: Apache-2.0

file(GLOB SF32LB_SOURCES "../../drivers/hal/*.c")
foreach(FILE ${SF32LB_SOURCES})
    string(FIND "${FILE}" "bf0_hal_audcodec_m.c" POSITION) 
    if(POSITION GREATER -1 AND CONFIG_SOC_SERIES_SF32LB56X)
        message(${FILE})
    else()
        zephyr_library_sources(${FILE})
    endif()
endforeach()

zephyr_library_sources_ifdef(CONFIG_GPIO sflb32_gpio.c)
zephyr_library_sources_ifdef(CONFIG_CONSOLE sflb32_console.c)
zephyr_library_sources_ifdef(CONFIG_SERIAL sflb32_serial.c)

