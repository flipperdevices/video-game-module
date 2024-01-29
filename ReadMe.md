# Flipper Video Game Module (Powered by Raspberry Pi)
![Flipper Zero Video Game Module Powered by Raspberry Pi](https://cdn.flipper.net/video_game_module_preview_for_github.png)

## Building

Requirements: 

- arm-none-eabi-gcc
- CMake
- Protobuf

## Getting Source Code

	git clone --recursive https://github.com/flipperdevices/video-game-module.git

Make sure that all git sub-modules was recursively cloned.

## Compiling

	# In project folder
	( cd build && cmake .. && make )

Compiled firmware can be found in `app` folder.

## Flashing

- Press and hold boot button, plug VGM into your computer USB
- Copy `vgm-fw-*.uf2` from `build/app` folder to newly appeared drive
