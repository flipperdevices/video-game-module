#pragma once

#include <stddef.h>
#include <stdint.h>

#define FLIPPER_SCREEN_WIDTH (128)
#define FLIPPER_SCREEN_HEIGHT (64)

#define FLIPPER_BITMAP_SIZE (FLIPPER_SCREEN_WIDTH * FLIPPER_SCREEN_HEIGHT / 8)

extern const uint8_t bitmap_splash_screen[FLIPPER_BITMAP_SIZE];
