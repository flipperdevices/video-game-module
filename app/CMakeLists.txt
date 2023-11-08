# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB APP_SOURCES
    "*.c*"
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
	libdvi
	libsprite
	FreeRTOS
)

# create map/bin/hex file etc.
pico_add_extra_outputs(flipper_dvi)

# add command to flash uf2 to board via cp
add_custom_target(flash_uf2
	COMMAND cp "${CMAKE_CURRENT_BINARY_DIR}/flipper_dvi.uf2" "/Volumes/RPI-RP2/flipper_dvi.uf2"
	DEPENDS flipper_dvi
)

# add command to flash board via dap-link
add_custom_target(flash_dap
	COMMAND openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ${CMAKE_CURRENT_BINARY_DIR}/flipper_dvi.elf verify reset exit"
	DEPENDS flipper_dvi
)
