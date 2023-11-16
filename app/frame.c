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

#define COLOR_BG 0xFC00
#define COLOR_FG 0x0000

#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F

bool orientation_enable = false;
uint16_t color_bg = COLOR_BG;
uint16_t color_fg = COLOR_FG;

// TMDS bit clock 252 MHz
// DVDD 1.2V (1.1V seems ok too)
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

typedef struct {
    frame_t frame;
    uint8_t orientation;
} frame_data_t;

static frame_data_t current = {0};
static frame_data_t next = {0};
auto_init_mutex(data_mutex);

static __not_in_flash("core1_main") void core1_main() {
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    dvi_start(&dvi0);
    dvi_scanbuf_main_16bpp(&dvi0);
    __builtin_unreachable();
}

static inline bool __not_in_flash("is_pixel_set")
    is_pixel_set(const frame_t* frame, uint8_t x, uint8_t y) {
    size_t i = (y / 8) * 128;
    y &= 7;
    i += x;
    return (frame->data[i] & (1 << y)) != 0;
}

static void __not_in_flash("buf_set_color_h")
    buf_set_color_h(uint16_t* buf, size_t x, uint16_t color) {
    buf[32 + x * 2 + 0] = color;
    buf[32 + x * 2 + 1] = color;
}

static void __not_in_flash("buf_set_color_v")
    buf_set_color_v(uint16_t* buf, size_t x, uint16_t color) {
    buf[32 + 96 + x + 0] = color;
}

static void __not_in_flash("fill_scanline_h") fill_scanline_h(uint16_t* buf, uint scanline) {
    int32_t frame_y = scanline / 3 - 8;

    for(size_t frame_x = 0; frame_x < 128; frame_x++) {
        uint16_t color = color_bg;

        if(frame_y >= 0 && frame_y < 64) {
            if(is_pixel_set(&current.frame, frame_x, frame_y)) {
                color = color_fg;
            }
        }

        buf_set_color_h(buf, frame_x, color);
    }
}

static void __not_in_flash("fill_scanline_v") fill_scanline_v(uint16_t* buf, uint scanline) {
    int32_t frame_x = scanline - 50;

    for(size_t frame_y = 0; frame_y < 64; frame_y++) {
        uint16_t color = color_bg;

        if(frame_x >= 0 && frame_x < 128) {
            if(is_pixel_set(&current.frame, frame_x, 64 - 1 - frame_y)) {
                color = color_fg;
            }
        }

        buf_set_color_v(buf, frame_y, color);
    }
}

static void __not_in_flash("core1_scanline_callback") core1_scanline_callback() {
    static uint scanline = 0;

    if(scanline == 0) {
        if(mutex_try_enter(&data_mutex, NULL)) {
            memcpy(&current, &next, sizeof(frame_data_t));
            current.orientation = next.orientation;
            mutex_exit(&data_mutex);
        }
    }

    // Discard any scanline pointers passed back
    uint16_t* bufptr;
    while(queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
        ;

    // Get a pointer to the next scanline
    bufptr = framebuf;

    for(size_t i = 0; i <= 32; i++) {
        framebuf[i] = color_bg;
        framebuf[FRAME_WIDTH - i] = color_bg;
    }

    if(orientation_enable && (current.orientation == OrientationVertical ||
                              current.orientation == OrientationVerticalFlip)) {
        fill_scanline_v(bufptr, scanline);
    } else {
        fill_scanline_h(bufptr, scanline);
    }

    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);

    scanline = (scanline + 1) % FRAME_HEIGHT;
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

void frame_parse_data(uint8_t orientation, const frame_t* frame, uint32_t timeout_ms) {
    if(mutex_enter_timeout_ms(&data_mutex, timeout_ms)) {
        memcpy(&next.frame, frame, sizeof(frame_t));
        next.orientation = orientation;
        mutex_exit(&data_mutex);
    }
}

void frame_set_color(uint16_t bg, uint16_t fg) {
    color_bg = bg;
    color_fg = fg;
}