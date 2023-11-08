# this is copy of "lib/pico_dvi/software/CMakeLists.txt" with some modifications

cmake_minimum_required(VERSION 3.12)

# Copy code to ram on startup
set(PICO_COPY_TO_RAM 1)
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/pico_sdk")
set(PICO_DVI_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/pico_dvi/software")
include("${PICO_DVI_PATH}/pico_sdk_import.cmake")

project(picodvi C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_compile_options(-Wall)


set(FREERTOS_CFG_DIRECTORY "${CMAKE_SOURCE_DIR}/config")
set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/lib/freertos_kernel")

# Add FreeRTOS as a library
add_library(FreeRTOS STATIC
    ${FREERTOS_SRC_DIRECTORY}/event_groups.c
    ${FREERTOS_SRC_DIRECTORY}/list.c
    ${FREERTOS_SRC_DIRECTORY}/queue.c
    ${FREERTOS_SRC_DIRECTORY}/stream_buffer.c
    ${FREERTOS_SRC_DIRECTORY}/tasks.c
    ${FREERTOS_SRC_DIRECTORY}/timers.c
    ${FREERTOS_SRC_DIRECTORY}/portable/MemMang/heap_3.c
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0/port.c
)

# Build FreeRTOS
target_include_directories(FreeRTOS PUBLIC
    ${FREERTOS_CFG_DIRECTORY}/
    ${FREERTOS_SRC_DIRECTORY}/include
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0
)

include_directories(
	"${PICO_DVI_PATH}/assets"
	"${PICO_DVI_PATH}/include"
)

add_subdirectory("${PICO_DVI_PATH}/libdvi")
add_subdirectory("${PICO_DVI_PATH}/libsprite")

add_subdirectory(app)
