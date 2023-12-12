# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB APP_SOURCES
    "*.c*"
	"cli/*.c*"
	"imu/*.c*"
)

add_executable(flipper_dvi
	${APP_SOURCES}
)

target_compile_definitions(flipper_dvi PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
)

target_link_libraries(flipper_dvi
	pico_stdlib
	pico_multicore
	pico_util
	hardware_spi
	libdvi
	libsprite
	FreeRTOS
)

# create map/bin/hex file etc.
pico_add_extra_outputs(flipper_dvi)

# disable stdio for uart
pico_enable_stdio_uart(flipper_dvi 0)
pico_enable_stdio_usb(flipper_dvi 1)

# uf2 to board via cp
add_custom_target(flash_uf2
	COMMAND cp "${CMAKE_CURRENT_BINARY_DIR}/flipper_dvi.uf2" "/Volumes/RPI-RP2/flipper_dvi.uf2"
	DEPENDS flipper_dvi
)

# flash board via dap-link
add_custom_target(flash_dap
	COMMAND openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ${CMAKE_CURRENT_BINARY_DIR}/flipper_dvi.elf verify reset exit"
	DEPENDS flipper_dvi
)

# Build date
string(TIMESTAMP FW_BUILD_DATE "%Y-%m-%d")

# Commit
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE FW_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE)

# Branch
execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE FW_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE)

set(INFO_FLAGS
    -DFW_BUILD_DATE="${FW_BUILD_DATE}"
    -DFW_GIT_COMMIT="${FW_GIT_COMMIT}"
    -DFW_GIT_BRANCH="${FW_GIT_BRANCH}"
)

message(STATUS "Build date: ${FW_BUILD_DATE}")
message(STATUS "FW commit: ${FW_GIT_COMMIT}")
message(STATUS "FW branch: ${FW_GIT_BRANCH}")

set_property(SOURCE "cli/cli_command_device_info.cpp" APPEND PROPERTY COMPILE_OPTIONS ${INFO_FLAGS})
