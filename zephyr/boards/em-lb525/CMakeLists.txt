# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../SifliSDK/customer/boards/include/)
file(GLOB SF32LB52X_SOURCES "../../../SifliSDK/customer/boards/eh-lb52xu/*.c")
foreach(FILE ${SF32LB52X_SOURCES})
    zephyr_library_sources(${FILE})
endforeach()
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../SifliSDK/customer/boards/include/config/sf32lb52x)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../SifliSDK/customer/boards/eh-lb52xu)

file(GLOB SF32LB_SOURCES "../../../SifliSDK/customer/boards/common/*.c")
foreach(FILE ${SF32LB_SOURCES})
    zephyr_sources(${FILE})
endforeach()


zephyr_code_relocate(FILES ../../../SifliSDK/customer/boards/eh-lb52xu/bsp_init.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../../SifliSDK/customer/boards/eh-lb52xu/bsp_lcd_tp.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../../SifliSDK/customer/boards/eh-lb52xu/bsp_pinmux.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../../SifliSDK/customer/boards/eh-lb52xu/bsp_power.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../../SifliSDK/customer/boards/common/bsp_common.c ".*\\HAL_MspInit" LOCATION RAM NOKEEP)

