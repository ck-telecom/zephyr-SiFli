# Copyright (c) 2024 SiFli
# SPDX-License-Identifier: Apache-2.0

file(GLOB SF32LB_SOURCES "../../SifliSDK/drivers/hal/*.c")
foreach(FILE ${SF32LB_SOURCES})
    string(FIND "${FILE}" "bf0_hal_audcodec_m.c" POSITION) 
    if(POSITION GREATER -1 AND CONFIG_SOC_SERIES_SF32LB56X)
        message("Remove ${FILE} from compiling.")
    else()
        string(FIND "${FILE}" "bf0_hal_audcodec.c" POSITION)         
        if(POSITION GREATER -1 AND CONFIG_SOC_SERIES_SF32LB52X)
            message("Remove ${FILE} from compiling.")
        else()
            zephyr_library_sources(${FILE})
        endif()
    endif()
endforeach()

zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal_mpi.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal_mpi_ex.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal_mpi_psram.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal_mpi_psram.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal_rcc.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal_hpaon.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal_pinmux.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal_gpio.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/flash_table.c LOCATION RAM NOKEEP)
zephyr_code_relocate(FILES ../../SifliSDK/drivers/hal/bf0_hal.c FILTER ".*\\.HAL_Init|.*\\.HAL_Delay_us|.*\\.HAL_Delay_us_|.*\\.HAL_MspInit" LOCATION 
RAM NOKEEP)

if (CONFIG_SOC_SERIES_SF32LB56X)
	zephyr_library_sources("../../SifliSDK/drivers/cmsis/sf32lb56x/Templates/system_bf0_ap.c")
	zephyr_library_sources("../../SifliSDK/drivers/cmsis/sf32lb56x/bt_rf_fulcal.c")
	zephyr_library_sources("../../SifliSDK/drivers/cmsis/sf32lb56x/bf0_pin_const.c")
    zephyr_code_relocate(FILES ../../SifliSDK/drivers/cmsis/sf32lb56x/bf0_pin_const.c LOCATION RAM NOKEEP)
endif()

if (CONFIG_SOC_SERIES_SF32LB52X)
	zephyr_library_sources("../../SifliSDK/drivers/cmsis/sf32lb52x/Templates/system_bf0_ap.c")
	zephyr_library_sources("../../SifliSDK/drivers/cmsis/sf32lb52x/bt_rf_fulcal.c")
	zephyr_library_sources("../../SifliSDK/drivers/cmsis/sf32lb52x/bf0_pin_const.c")
    zephyr_code_relocate(FILES ../../SifliSDK/drivers/cmsis/sf32lb52x/bf0_pin_const.c LOCATION RAM NOKEEP)
endif()

zephyr_library_sources_ifdef(CONFIG_GPIO sflb32_gpio.c)
zephyr_library_sources_ifdef(CONFIG_CONSOLE sflb32_console.c)
zephyr_library_sources_ifdef(CONFIG_SERIAL sflb32_serial.c)

