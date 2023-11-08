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

#define UART_ID uart0
#define UART_IRQ UART0_IRQ
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_BAUD_RATE 115200

QueueHandle_t queue;

// RX interrupt handler
static void uart_on_rx() {
    while(uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        BaseType_t pxHigherPriorityTaskWoken;
        xQueueSendFromISR(queue, &ch, &pxHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
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
    uart_set_format(UART_ID, 8, UART_PARITY_NONE, 1);

    // disable hardware flow control
    uart_set_hw_flow(UART_ID, false, false);

    // add uart irqs
    irq_set_exclusive_handler(UART_IRQ, uart_on_rx);
    irq_set_enabled(UART_IRQ, true);

    // enable rx irq
    uart_set_irq_enables(UART_ID, true, false);

    while(true) {
        uint8_t ch = 0;
        if(xQueueReceive(queue, &ch, portMAX_DELAY)) {
            uart_write_blocking(UART_ID, &ch, 1);
        }
    }
}

void uart_protocol_init() {
    TaskHandle_t uart_task_handle = NULL;
    BaseType_t status = xTaskCreate(uart_task, "uart_task", 128, NULL, 1, &uart_task_handle);
    assert(status == pdPASS);
}