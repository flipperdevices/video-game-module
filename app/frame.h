#pragma once
#include <stdint.h>

typedef struct {
    const uint8_t data[1024];
} frame_t;

void frame_init();

uint32_t frame_get_clock();

uint32_t frame_get_voltage();

void frame_parse_data(const frame_t* frame);
