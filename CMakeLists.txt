# this is copy of "lib/pico_dvi/software/CMakeLists.txt" with some modifications

cmake_minimum_required(VERSION 3.12)

set(PICO_COPY_TO_RAM 1)
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/pico_sdk")
set(PICO_DVI_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/pico_dvi/software")
include("${PICO_DVI_PATH}/pico_sdk_import.cmake")

project(picodvi C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_compile_options(-Wall)

include_directories(
	"${PICO_DVI_PATH}/assets"
	"${PICO_DVI_PATH}/include"
)

add_subdirectory("${PICO_DVI_PATH}/libdvi")
add_subdirectory("${PICO_DVI_PATH}/libsprite")

add_subdirectory(app)
