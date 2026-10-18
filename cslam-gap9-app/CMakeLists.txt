cmake_minimum_required(VERSION 3.19)

if(NANO_C_SLAM_GVSOC_SPI_TRACE)
    set(TARGET_NAME nano_c_slam_gvsoc_spi_trace)
elseif(NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE)
    set(TARGET_NAME nano_c_slam_gvsoc_spi_main_probe)
elseif(NANO_C_SLAM_GVSOC_SPI_SLAVE_PROBE)
    set(TARGET_NAME nano_c_slam_gvsoc_spi_slave_probe)
elseif(NANO_C_SLAM_GVSOC_DMA_SMOKE)
    set(TARGET_NAME nano_c_slam_gvsoc_dma_smoke)
elseif(NANO_C_SLAM_GVSOC_MAPPING_SMOKE)
    set(TARGET_NAME nano_c_slam_gvsoc_mapping_smoke)
elseif(NANO_C_SLAM_GVSOC_SMOKE)
    set(TARGET_NAME nano_c_slam_gvsoc_smoke)
else()
    set(TARGET_NAME nano_c_slam_gap9)
endif()
include($ENV{GAP_SDK_HOME}/utils/cmake/setup.cmake)
project(${TARGET_NAME} C ASM)

option(NANO_C_SLAM_ICP_DIAGNOSTICS "Enable test-only ICP stage diagnostics" OFF)
set(NANO_C_SLAM_POSE_GRAPH_FC_THRESHOLD "64" CACHE STRING
    "Largest pose-graph dimension solved serially on the FC")
set(NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT "2" CACHE STRING
    "Finite command count for the test-only production SPI probe (2 or 66)")
option(NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_SKIP_SPI_TX
    "Skip optimizer SPI TX only for an isolated GVSOC ingress run" OFF)
if(NOT NANO_C_SLAM_POSE_GRAPH_FC_THRESHOLD MATCHES "^[0-9]+$")
    message(FATAL_ERROR "NANO_C_SLAM_POSE_GRAPH_FC_THRESHOLD must be a nonnegative integer")
endif()
if(NOT NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT MATCHES "^(2|66)$")
    message(FATAL_ERROR "SPI main probe packet count must be 2 or 66")
endif()

set_source_files_properties(
    ../tests/gap9_sdk_gvsoc_mapping_smoke.c
    ../tests/gap9_sdk_gvsoc_dma_smoke.c
    PROPERTIES COMPILE_OPTIONS "-std=gnu11"
)

if(NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE)
    file(GLOB GAP9_SLAM_SOURCES CONFIGURE_DEPENDS "slam/*.c" "slam/rcm/*.c")
    file(GLOB GAP9_POINT_CLOUD_SOURCES CONFIGURE_DEPENDS "point-cloud/*.c")
    file(GLOB GAP9_SPI_SOURCES CONFIGURE_DEPENDS "spi/*.c")
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
        PRIVATE slam slam/rcm dma icp spi point-cloud gpio
    )
    target_compile_definitions(
        ${TARGET_NAME}
        PRIVATE
        NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE=1
        NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT=${NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT}
        NANO_C_SLAM_TARGET_GVSOC=1
    )
    if(NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_SKIP_SPI_TX)
        target_compile_definitions(
            ${TARGET_NAME} PRIVATE NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_SKIP_SPI_TX=1
        )
    endif()
    set(APP_OPT_LEVEL -O2)
elseif(NANO_C_SLAM_GVSOC_SPI_SLAVE_PROBE)
    add_executable(
        ${TARGET_NAME}
        ../tests/gap9_gvsoc_spi_slave_probe.c
        spi/spi_driver.c
    )
    target_include_directories(${TARGET_NAME} PRIVATE spi)
    target_compile_definitions(${TARGET_NAME} PRIVATE NANO_C_SLAM_TARGET_GVSOC=1)
    set(APP_OPT_LEVEL -O2)
elseif(NANO_C_SLAM_GVSOC_SPI_TRACE)
    file(GLOB GAP9_SLAM_SOURCES CONFIGURE_DEPENDS "slam/*.c" "slam/rcm/*.c")
    file(GLOB GAP9_POINT_CLOUD_SOURCES CONFIGURE_DEPENDS "point-cloud/*.c")
    add_executable(
        ${TARGET_NAME}
        main.c
        ../tests/gap9_gvsoc_spi_trace_transport.c
        spi/spi_buffers.c
        spi/spi_cmds_decode.c
        gpio/gpio_config.c
        dma/dma_transfers.c
        icp/icp.c
        ${GAP9_SLAM_SOURCES}
        ${GAP9_POINT_CLOUD_SOURCES}
    )
    target_include_directories(
        ${TARGET_NAME}
        PRIVATE ../tests slam slam/rcm dma icp spi point-cloud gpio
    )
    target_compile_definitions(
        ${TARGET_NAME}
        PRIVATE NANO_C_SLAM_GVSOC_SPI_TRACE=1 NANO_C_SLAM_TARGET_GVSOC=1
    )
    set(APP_OPT_LEVEL -O2)
elseif(NANO_C_SLAM_GVSOC_DMA_SMOKE)
    add_executable(
        ${TARGET_NAME}
        ../tests/gap9_sdk_gvsoc_dma_smoke.c
        dma/dma_transfers.c
    )
    target_include_directories(${TARGET_NAME} PRIVATE dma)
    set(APP_OPT_LEVEL -O2)
elseif(NANO_C_SLAM_GVSOC_MAPPING_SMOKE)
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
target_compile_definitions(
    ${TARGET_NAME}
    PRIVATE NANO_C_SLAM_POSE_GRAPH_FC_THRESHOLD=${NANO_C_SLAM_POSE_GRAPH_FC_THRESHOLD}
)
if(NANO_C_SLAM_ICP_DIAGNOSTICS)
    target_compile_definitions(${TARGET_NAME} PRIVATE NANO_C_SLAM_ICP_DIAGNOSTICS=1)
endif()
target_link_options(${TARGET_NAME} PRIVATE -Wl,--print-memory-usage)
setupos(${TARGET_NAME})
target_link_libraries(${TARGET_NAME} PRIVATE m)
