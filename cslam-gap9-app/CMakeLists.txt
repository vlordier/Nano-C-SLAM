cmake_minimum_required(VERSION 3.19)

if(NANO_C_SLAM_GVSOC_MAPPING_SMOKE)
    set(TARGET_NAME nano_c_slam_gvsoc_mapping_smoke)
elseif(NANO_C_SLAM_GVSOC_SMOKE)
    set(TARGET_NAME nano_c_slam_gvsoc_smoke)
else()
    set(TARGET_NAME nano_c_slam_gap9)
endif()
include($ENV{GAP_SDK_HOME}/utils/cmake/setup.cmake)
project(${TARGET_NAME} C ASM)

option(NANO_C_SLAM_ICP_DIAGNOSTICS "Enable test-only ICP stage diagnostics" OFF)

set_source_files_properties(
    ../tests/gap9_sdk_gvsoc_mapping_smoke.c
    PROPERTIES COMPILE_OPTIONS "-std=gnu11"
)

if(NANO_C_SLAM_GVSOC_MAPPING_SMOKE)
    file(GLOB GAP9_SLAM_SOURCES CONFIGURE_DEPENDS "slam/*.c" "slam/rcm/*.c")
    file(GLOB GAP9_POINT_CLOUD_SOURCES CONFIGURE_DEPENDS "point-cloud/*.c")
    add_executable(
        ${TARGET_NAME}
        ../tests/gap9_sdk_gvsoc_mapping_smoke.c
        dma/dma_transfers.c
        icp/icp.c
        ${GAP9_SLAM_SOURCES}
        ${GAP9_POINT_CLOUD_SOURCES}
    )
    target_include_directories(
        ${TARGET_NAME}
        PRIVATE slam slam/rcm dma icp point-cloud
    )
    target_compile_definitions(${TARGET_NAME} PRIVATE NANO_C_SLAM_TARGET_GVSOC=1)
    set(APP_OPT_LEVEL -O2)
elseif(NANO_C_SLAM_GVSOC_SMOKE)
    add_executable(${TARGET_NAME} ../tests/gap9_sdk_gvsoc_smoke.c icp/icp.c dma/dma_transfers.c)
    target_include_directories(${TARGET_NAME} PRIVATE icp dma slam)
    set(APP_OPT_LEVEL -O2)
else()
    file(GLOB GAP9_SPI_SOURCES CONFIGURE_DEPENDS "spi/*.c")
    file(GLOB GAP9_SLAM_SOURCES CONFIGURE_DEPENDS "slam/*.c" "slam/rcm/*.c")
    file(GLOB GAP9_POINT_CLOUD_SOURCES CONFIGURE_DEPENDS "point-cloud/*.c")
    add_executable(
        ${TARGET_NAME}
        main.c
        dma/dma_transfers.c
        icp/icp.c
        gpio/gpio_config.c
        ${GAP9_SPI_SOURCES}
        ${GAP9_SLAM_SOURCES}
        ${GAP9_POINT_CLOUD_SOURCES}
    )
    target_include_directories(
        ${TARGET_NAME}
        PRIVATE
        slam
        slam/rcm
        dma
        icp
        spi
        point-cloud
        gpio
    )
    set(APP_OPT_LEVEL -O3)
endif()

target_include_directories(${TARGET_NAME} PRIVATE $ENV{GAP_SDK_HOME}/rtos/pmsis/tools/assert/include)
target_compile_options(
    ${TARGET_NAME}
    PRIVATE
    ${APP_OPT_LEVEL}
    -Wno-error=implicit-function-declaration
    -Wno-error=deprecated-declarations
    -include
    $ENV{GAP_SDK_HOME}/rtos/pmsis/tools/assert/include/assert.h
)
target_compile_definitions(${TARGET_NAME} PRIVATE CONFIG_DISABLE_WERROR=1)
if(NANO_C_SLAM_ICP_DIAGNOSTICS)
    target_compile_definitions(${TARGET_NAME} PRIVATE NANO_C_SLAM_ICP_DIAGNOSTICS=1)
endif()
target_link_options(${TARGET_NAME} PRIVATE -Wl,--print-memory-usage)
setupos(${TARGET_NAME})
target_link_libraries(${TARGET_NAME} PRIVATE m)
