cmake_minimum_required(VERSION 3.19)

set(TARGET_NAME nano_c_slam_gap9)
include($ENV{GAP_SDK_HOME}/utils/cmake/setup.cmake)
project(${TARGET_NAME} C ASM)

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
    $ENV{GAP_SDK_HOME}/rtos/pmsis/tools/assert/include
)
target_compile_options(
    ${TARGET_NAME}
    PRIVATE
    -O3
    -Wno-error=implicit-function-declaration
    -Wno-error=deprecated-declarations
    -include
    $ENV{GAP_SDK_HOME}/rtos/pmsis/tools/assert/include/assert.h
)
target_compile_definitions(${TARGET_NAME} PRIVATE CONFIG_DISABLE_WERROR=1)
target_link_options(${TARGET_NAME} PRIVATE -Wl,--print-memory-usage)
target_link_libraries(${TARGET_NAME} PRIVATE m)
setupos(${TARGET_NAME})
