#include <dvi.h>
#include <dvi_serialiser.h>
#include <common_dvi_pin_configs.h>
#include <sprite.h>
#include <pico/multicore.h>
#include <hardware/vreg.h>
#include <string.h>
#include "frame.h"

#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240

static struct dvi_inst dvi0;
static uint16_t framebuf[FRAME_WIDTH * 2];

#define COLOR_FG 0xFC00
#define COLOR_BG 0x0000

// TMDS bit clock 252 MHz
// DVDD 1.2V (1.1V seems ok too)
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

static frame_t frame_current = {0};
static frame_t frame_next = {0};
auto_init_mutex(my_mutex);

static void core1_main() {
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    dvi_start(&dvi0);
    dvi_scanbuf_main_16bpp(&dvi0);
    __builtin_unreachable();
}

static inline bool is_pixel_set(const frame_t* frame, uint8_t x, uint8_t y) {
    size_t i = (y / 8) * 128;
    y &= 7;
    i += x;
    return (frame->data[i] & (1 << y)) != 0;
}

static void buf_set_color(uint16_t* buf, size_t x, uint16_t color) {
    buf[32 + x * 2 + 0] = color;
    buf[32 + x * 2 + 1] = color;
}

static void fill_scanline(uint16_t* buf, uint scanline) {
    int32_t frame_y = scanline / 3 - 8;

    for(size_t frame_x = 0; frame_x < 128; frame_x++) {
        uint16_t color = COLOR_BG;

        if(frame_y >= 0 && frame_y < 64) {
            if(is_pixel_set(&frame_current, frame_x, frame_y)) {
                color = COLOR_FG;
            }
        }

        buf_set_color(buf, frame_x, color);
    }
}

static void core1_scanline_callback() {
    // Note first two scanlines are pushed before DVI start
    static uint scanline = 2;

    // Discard any scanline pointers passed back
    uint16_t* bufptr;
    while(queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
        ;

    // Get a pointer to the next scanline
    bufptr = framebuf;

    // Fill the next scanline with the frame buffer
    fill_scanline(bufptr, scanline);

    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
    scanline = (scanline + 1) % FRAME_HEIGHT;

    if(scanline == 0) {
        if(mutex_try_enter(&my_mutex, NULL)) {
            memcpy(&frame_current, &frame_next, sizeof(frame_t));
            mutex_exit(&my_mutex);
        }
    }
}

void frame_init() {
    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = picodvi_dvi_cfg;
    dvi0.scanline_callback = core1_scanline_callback;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    // add two scanlines to the scanlines queue
    queue_add_blocking_u32(&dvi0.q_colour_valid, &framebuf[0]);
    queue_add_blocking_u32(&dvi0.q_colour_valid, &framebuf[FRAME_WIDTH]);

    multicore_launch_core1(core1_main);
}

uint32_t frame_get_clock() {
    return DVI_TIMING.bit_clk_khz;
}

uint32_t frame_get_voltage() {
    return VREG_VSEL;
}

void frame_parse_data(const frame_t* frame) {
    mutex_enter_blocking(&my_mutex);
    memcpy(&frame_next, frame, sizeof(frame_t));
    mutex_exit(&my_mutex);
}