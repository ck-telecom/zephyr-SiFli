# SPDX-License-Identifier: Apache-2.0

zephyr_compile_definitions(
  -DSF32LB56X
  -DSOC_SF32LB56X
)

zephyr_include_directories(${ZEPHYR_BASE}/drivers)

zephyr_sources(
  soc.c
  )

zephyr_sources_ifdef(CONFIG_PM
  power.c
  )

zephyr_sources_ifdef(CONFIG_POWEROFF poweroff.c)


file(GLOB SF32LB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../../../boards/common/*.c")
foreach(FILE ${SF32LB_SOURCES})
    zephyr_sources(${FILE})
endforeach()

file(GLOB SF32LB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../../../boards/eh-lb56xu/*.c")
foreach(FILE ${SF32LB_SOURCES})
    zephyr_sources(${FILE})
endforeach()


zephyr_include_directories(.)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/cmsis/sf32lb56x)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../boards/include/config/sf32lb56x)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../boards/eh-lb56xu)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
