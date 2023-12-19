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
#include <stream_buffer.h>
#include <stdlib.h>
// #include "frame.h"
// #include "led.h"

#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <flipperzero-protobuf/flipper.pb.h>

#include "expansion_protocol.h"

#define UART_ID uart0
#define UART_IRQ UART0_IRQ
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_INIT_BAUD_RATE (9600UL)
#define UART_BAUD_RATE (230400UL)
#define EXPANSION_TIMEOUT_MS (250UL)

// const uint8_t magic[8] = {'D', 'A', 'T', 'A', '1', '3', '3', '7'};

static StreamBufferHandle_t stream;
// static PB_Main message;

// RX interrupt handler
static void uart_on_rx() {
    while(uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        BaseType_t pxHigherPriorityTaskWoken;
        xStreamBufferSendFromISR(stream, &ch, sizeof(ch), &pxHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}

// uint8_t protocol_read() {
//     uint8_t ch = 0;
//     xQueueReceive(queue, &ch, portMAX_DELAY);
//     return ch;
// }

// static void protocol_wait_for_magic() {
//     for(int i = 0; i < 8; i++) {
//         uint8_t ch = protocol_read();
//         if(ch != magic[i]) {
//             i = -1;
//         }
//     }
// }
//
// static size_t protocol_read_size() {
//     union {
//         uint8_t size8[4];
//         size_t size;
//     } size;
//     for(size_t i = 0; i < sizeof(size_t); i++) {
//         size.size8[i] = protocol_read();
//     }
//     return size.size;
// }
//
// static void protocol_read_data(void* data, size_t size) {
//     uint8_t* data8 = (uint8_t*)data;
//     for(size_t i = 0; i < size; i++) {
//         data8[i] = protocol_read();
//     }
// }
//
// static void protocol_resp(void* data, size_t size) {
//     uint8_t* size8 = (uint8_t*)&size;
//     led_blue(true);
//     uart_write_blocking(UART_ID, magic, 8);
//     uart_write_blocking(UART_ID, size8, sizeof(size_t));
//     uart_write_blocking(UART_ID, data, size);
//     led_blue(false);
// }
//
// typedef enum {
//     CMD_FRAME = 0xAB,
//     CMD_FRAME_COLOR = 0xBC,
// } protocol_cmd_t;
//
// static void protocol_parse_data(void* data, size_t size) {
//     uint8_t* data8 = (uint8_t*)data;
//
//     switch(data8[0]) {
//     case CMD_FRAME:
//         frame_parse_data(data8[1], (frame_t*)&data8[2], 1000);
//         protocol_resp("OK", 2);
//         break;
//     case CMD_FRAME_COLOR:
//         frame_set_color(data8[2] << 8 | data8[1], data8[4] << 8 | data8[3]);
//         protocol_resp("OK", 2);
//         break;
//
//     default:
//         break;
//     }
// }

// Receive frames
static size_t expansion_receive_callback(uint8_t* data, size_t data_size, void* context) {
    size_t received_size = 0;

    while(true) {
        const size_t received_size_cur = xStreamBufferReceive(stream, data + received_size, data_size - received_size, EXPANSION_TIMEOUT_MS);
        if(received_size_cur == 0) break;
        received_size += received_size_cur;
        if(received_size == data_size) break;
    }

    return received_size;
}

static inline bool expansion_receive_frame(ExpansionFrame* frame) {
    return expansion_frame_decode(frame, expansion_receive_callback, NULL);
}

static inline bool expansion_is_heartbeat_frame(const ExpansionFrame* frame) {
    return frame->header.type == ExpansionFrameTypeHeartbeat;
}

static inline bool expansion_is_success_frame(const ExpansionFrame* frame) {
    return frame->header.type == ExpansionFrameTypeStatus &&
           frame->content.status.error == ExpansionFrameErrorNone;
}

// Send frames
static size_t expansion_send_callback(const uint8_t* data, size_t data_size, void* context) {
    (void)context;

    uart_write_blocking(UART_ID, data, data_size);
    return data_size;
}

static inline bool expansion_send_frame(const ExpansionFrame* frame) {
    return expansion_frame_encode(frame, expansion_send_callback, NULL);
}

static inline bool expansion_send_heartbeat() {
    ExpansionFrame frame = {
        .header.type = ExpansionFrameTypeHeartbeat,
        .content.heartbeat = {},
    };

    return expansion_send_frame(&frame);
}

static bool expansion_send_baud_rate_request(uint32_t baud_rate) {
    ExpansionFrame frame = {
        .header.type = ExpansionFrameTypeBaudRate,
        .content.baud_rate.baud = baud_rate,
    };

    return expansion_send_frame(&frame);
}

static bool expansion_send_control_request(ExpansionFrameControlCommand command) {
    ExpansionFrame frame = {
        .header.type = ExpansionFrameTypeControl,
        .content.control.command = command,
    };

    return expansion_send_frame(&frame);
}

static bool expansion_wait_ready() {
    bool success = false;

    do {
        ExpansionFrame frame;
        if(!expansion_receive_frame(&frame)) break;
        if(!expansion_is_heartbeat_frame(&frame)) break;
        success = true;
    } while(false);

    return success;
}

static bool expansion_handshake() {
    bool success = false;

    do {
        if(!expansion_send_baud_rate_request(UART_BAUD_RATE)) break;
        ExpansionFrame frame;
        if(!expansion_receive_frame(&frame)) break;
        if(!expansion_is_success_frame(&frame)) break;
        uart_set_baudrate(UART_ID, UART_BAUD_RATE);
        success = true;
    } while(false);

    return success;
}

static bool expansion_start_rpc() {
    bool success = false;

    do {
        if(!expansion_send_control_request(ExpansionFrameControlCommandStartRpc)) break;
        ExpansionFrame frame;
        if(!expansion_receive_frame(&frame)) break;
        if(!expansion_is_success_frame(&frame)) break;
        success = true;
    } while(false);

    return success;
}

static bool expansion_idle() {
    while(true) {
        if(!expansion_send_heartbeat()) break;
        ExpansionFrame frame;
        if(!expansion_receive_frame(&frame)) break;
        if(!expansion_is_heartbeat_frame(&frame)) break;
        vTaskDelay(EXPANSION_TIMEOUT_MS - 50);
    }

    return false;
}

static void uart_task(void* unused_arg) {
    // init stream buffer
    stream = xStreamBufferCreate(sizeof(ExpansionFrame), 1);
    assert(buf != NULL);

    // init uart 0
    uart_init(uart0, UART_INIT_BAUD_RATE);

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
        // arbitrary delay just in case
        vTaskDelay(50);
        // reset baud rate to initial value
        uart_set_baudrate(UART_ID, UART_INIT_BAUD_RATE);
        // announce presence
        uart_putc_raw(UART_ID, 0xaa);

        // wait for flipper response
        if(!expansion_wait_ready()) continue;
        // negotiate baud rate
        if(!expansion_handshake()) continue;
        // start rpc
        if(!expansion_start_rpc()) continue;
        // test: maintain connection
        if(!expansion_idle()) continue;
    }

    // while(true) {
    //     protocol_wait_for_magic();
    //
    //     size_t size = protocol_read_size();
    //     if(size <= 4096) {
    //         void* data = malloc(size);
    //         assert(data != NULL);
    //         protocol_read_data(data, size);
    //         protocol_parse_data(data, size);
    //         protocol_resp(data, size);
    //         free(data);
    //     } else {
    //         protocol_resp("SNAK", 4);
    //     }
    // }
}

void uart_protocol_init(void) {
    TaskHandle_t uart_task_handle = NULL;
    BaseType_t status = xTaskCreate(uart_task, "uart_task", 1024, NULL, 1, &uart_task_handle);
    assert(status == pdPASS);
    (void)status;
}
