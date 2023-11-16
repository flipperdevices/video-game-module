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
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdlib.h>
#include "frame.h"
#include "led.h"

#define UART_ID uart0
#define UART_IRQ UART0_IRQ
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_BAUD_RATE 921600

const uint8_t magic[8] = {'D', 'A', 'T', 'A', '1', '3', '3', '7'};

static QueueHandle_t queue;

// RX interrupt handler
static void uart_on_rx() {
    while(uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        BaseType_t pxHigherPriorityTaskWoken;
        xQueueSendFromISR(queue, &ch, &pxHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}

uint8_t protocol_read() {
    uint8_t ch = 0;
    xQueueReceive(queue, &ch, portMAX_DELAY);
    return ch;
}

static void protocol_wait_for_magic() {
    for(int i = 0; i < 8; i++) {
        uint8_t ch = protocol_read();
        if(ch != magic[i]) {
            i = -1;
        }
    }
}

static size_t protocol_read_size() {
    union {
        uint8_t size8[4];
        size_t size;
    } size;
    for(size_t i = 0; i < sizeof(size_t); i++) {
        size.size8[i] = protocol_read();
    }
    return size.size;
}

static void protocol_read_data(void* data, size_t size) {
    uint8_t* data8 = (uint8_t*)data;
    for(size_t i = 0; i < size; i++) {
        data8[i] = protocol_read();
    }
}

static void protocol_resp(void* data, size_t size) {
    uint8_t* size8 = (uint8_t*)&size;
    led_blue(true);
    uart_write_blocking(UART_ID, magic, 8);
    uart_write_blocking(UART_ID, size8, sizeof(size_t));
    uart_write_blocking(UART_ID, data, size);
    led_blue(false);
}

typedef enum {
    CMD_FRAME = 0xAB,
    CMD_FRAME_COLOR = 0xBC,
} protocol_cmd_t;

static void protocol_parse_data(void* data, size_t size) {
    uint8_t* data8 = (uint8_t*)data;

    switch(data8[0]) {
    case CMD_FRAME:
        frame_parse_data(data8[1], (frame_t*)&data8[2], 1000);
        protocol_resp("OK", 2);
        break;
    case CMD_FRAME_COLOR:
        frame_set_color(data8[2] << 8 | data8[1], data8[4] << 8 | data8[3]);
        protocol_resp("OK", 2);
        break;

    default:
        break;
    }
}

static void uart_task(void* unused_arg) {
    // init queue
    queue = xQueueCreate(1024, sizeof(uint8_t));
    assert(queue != NULL);

    // init uart 0
    uart_init(uart0, UART_BAUD_RATE);

    // init uart gpio
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    gpio_set_pulls(UART_RX_PIN, true, false);
    gpio_set_pulls(UART_TX_PIN, true, false);

    // disable uart fifo
    uart_set_fifo_enabled(UART_ID, false);

    // config uart for 8N1 transmission
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    // disable hardware flow control
    uart_set_hw_flow(UART_ID, false, false);

    // add uart irqs
    irq_set_exclusive_handler(UART_IRQ, uart_on_rx);
    irq_set_enabled(UART_IRQ, true);

    // enable rx irq
    uart_set_irq_enables(UART_ID, true, false);

    while(true) {
        protocol_wait_for_magic();

        size_t size = protocol_read_size();
        if(size <= 4096) {
            void* data = malloc(size);
            assert(data != NULL);
            protocol_read_data(data, size);
            protocol_parse_data(data, size);
            protocol_resp(data, size);
            free(data);
        } else {
            protocol_resp("SNAK", 4);
        }
    }
}

void uart_protocol_init(void) {
    TaskHandle_t uart_task_handle = NULL;
    BaseType_t status = xTaskCreate(uart_task, "uart_task", 1024, NULL, 1, &uart_task_handle);
    assert(status == pdPASS);
}