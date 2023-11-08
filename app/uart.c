#include "uart.h"
#include <pico/stdlib.h>

// #include <stdio.h>
// #include <stdlib.h>
// #include <pico/multicore.h>
// #include <hardware/clocks.h>
// #include <hardware/gpio.h>
// #include <hardware/irq.h>
// #include <hardware/sync.h>
// #include <hardware/vreg.h>
// #include <pico/sem.h>

#include <hardware/gpio.h>

#define UART_ID uart0
#define UART_IRQ UART0_IRQ
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_BAUD_RATE 230400

static uart_rx_callback_t callback;
static void* context;

// RX interrupt handler
static void uart_on_rx() {
    while(uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        if(callback) {
            callback(&ch, 1, context);
        }
    }
}

void uart_protocol_send(uint8_t* data, size_t length) {
    uart_write_blocking(UART_ID, data, length);
}

void uart_protocol_init(uart_rx_callback_t callback, void* context) {
    // set callback
    callback = callback;
    context = context;

    // init uart 0
    uart_init(uart0, UART_BAUD_RATE);

    // init uart gpio
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // disable uart fifo
    uart_set_fifo_enabled(UART_ID, false);

    // add uart irqs
    irq_set_exclusive_handler(UART_IRQ, uart_on_rx);
    irq_set_enabled(UART_IRQ, true);

    // enable rx irq
    uart_set_irq_enables(UART_ID, true, false);
}