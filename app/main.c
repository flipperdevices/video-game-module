#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <hardware/vreg.h>
#include <pico/sem.h>
#include <FreeRTOS.h>
#include <task.h>
#include "frame.h"
#include "led.h"
#include "led_state.h"
#include "uart.h"
#include "usb.h"
#include "bitmaps.h"

static void init() {
    led_init();

    sleep_ms(10);
    vreg_set_voltage(frame_get_voltage());

    sleep_ms(10);
    set_sys_clock_khz(frame_get_clock(), true);

    stdio_init_all();
    frame_init();
    usb_init();
    uart_protocol_init();

    led_red(false);
}

static void show_defaul_screen() {
    uint8_t* frame_buffer = calloc(FLIPPER_BITMAP_SIZE, sizeof(uint8_t));

    bitmap_xbm_to_screen_frame(
        frame_buffer, bitmap_default_screen, FLIPPER_SCREEN_WIDTH, FLIPPER_SCREEN_HEIGHT);
    frame_parse_data(OrientationHorizontal, (const frame_t*)frame_buffer, pdMS_TO_TICKS(1000));

    free(frame_buffer);
}

int main() {
    init();

    show_defaul_screen();

    // init the led task
    led_state_init();

    vTaskStartScheduler();

    while(1) __wfe();
    __builtin_unreachable();
}
