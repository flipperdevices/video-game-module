#include "uart.h"
#include <pico/stdlib.h>

#include <hardware/gpio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stream_buffer.h>
#include <stdlib.h>

#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <flipperzero-protobuf/flipper.pb.h>

#include "frame.h"
#include "led_state.h"
#include "bitmaps.h"
#include "expansion_protocol.h"

#define UART_ID uart0
#define UART_IRQ UART0_IRQ
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_INIT_BAUD_RATE (9600UL)
#define UART_BAUD_RATE (1843200UL)

#define EXPANSION_MODULE_TIMEOUT_MS (EXPANSION_PROTOCOL_TIMEOUT_MS - 50UL)
#define EXPANSION_MODULE_STARTUP_DELAY_MS (250UL)
#define EXPANSION_MODULE_BUFFER_SIZE (sizeof(ExpansionFrame) + sizeof(ExpansionFrameChecksum))

static StreamBufferHandle_t stream;
static PB_Main rpc_message;

// RX interrupt handler
static void uart_on_rx() {
    BaseType_t higher_priority_task_woken = pdFALSE;

    while(uart_is_readable(UART_ID)) {
        const uint8_t ch = uart_getc(UART_ID);
        xStreamBufferSendFromISR(stream, &ch, sizeof(ch), &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

// Receive frames
static size_t expansion_receive_callback(uint8_t* data, size_t data_size, void* context) {
    size_t received_size = 0;

    while(true) {
        const size_t received_size_cur = xStreamBufferReceive(
            stream,
            data + received_size,
            data_size - received_size,
            pdMS_TO_TICKS(EXPANSION_MODULE_TIMEOUT_MS));
        if(received_size_cur == 0) break;
        received_size += received_size_cur;
        if(received_size == data_size) break;
    }

    return received_size;
}

static inline bool expansion_receive_frame(ExpansionFrame* frame) {
    return expansion_protocol_decode(frame, expansion_receive_callback, NULL) ==
           ExpansionProtocolStatusOk;
}

static inline bool expansion_is_heartbeat_frame(const ExpansionFrame* frame) {
    return frame->header.type == ExpansionFrameTypeHeartbeat;
}

static inline bool expansion_is_data_frame(const ExpansionFrame* frame) {
    return frame->header.type == ExpansionFrameTypeData;
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
    return expansion_protocol_encode(frame, expansion_send_callback, NULL) ==
           ExpansionProtocolStatusOk;
}

static inline bool expansion_send_heartbeat() {
    ExpansionFrame frame = {
        .header.type = ExpansionFrameTypeHeartbeat,
        .content.heartbeat = {},
    };

    return expansion_send_frame(&frame);
}

static inline bool expansion_send_status(ExpansionFrameError error) {
    ExpansionFrame frame = {
        .header.type = ExpansionFrameTypeStatus,
        .content.status.error = error,
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

static bool expansion_send_data_request(const uint8_t* data, size_t data_size) {
    ExpansionFrame frame = {
        .header.type = ExpansionFrameTypeData,
        .content.data.size = data_size,
    };

    memcpy(frame.content.data.bytes, data, data_size);
    return expansion_send_frame(&frame);
}

// Rpc functions

typedef struct {
    ExpansionFrame frame;
    size_t read_size; // Number of bytes already read from the data frame
} ExpansionRpcContext;

static inline bool expansion_rpc_is_read_complete(const ExpansionRpcContext* ctx) {
    return ctx->frame.content.data.size == ctx->read_size;
}

// Receive a frame while maintaining idle connection
static inline bool expansion_receive_frame_rpc_mode(ExpansionFrame* frame) {
    bool heartbeat_pending = false;

    while(true) {
        const ExpansionProtocolStatus status =
            expansion_protocol_decode(frame, expansion_receive_callback, NULL);

        if(status == ExpansionProtocolStatusErrorCommunication) {
            if(!heartbeat_pending && expansion_send_heartbeat()) {
                heartbeat_pending = true;
                continue;
            } else {
                return false;
            }

        } else if(status != ExpansionProtocolStatusOk) {
            return false;
        }

        if(expansion_is_heartbeat_frame(frame)) {
            heartbeat_pending = false;
        } else {
            return true;
        }
    }
}

static bool
    expansion_rpc_decode_callback(pb_istream_t* stream, pb_byte_t* data, size_t data_size) {
    ExpansionRpcContext* ctx = stream->state;
    size_t received_size = 0;

    while(received_size != data_size) {
        if(expansion_rpc_is_read_complete(ctx)) {
            // Read next frame
            if(!expansion_receive_frame_rpc_mode(&ctx->frame)) break;
            if(!expansion_is_data_frame(&ctx->frame)) break;

            ctx->read_size = 0;
        }

        const size_t current_size =
            MIN(data_size - received_size, ctx->frame.content.data.size - ctx->read_size);
        memcpy(data + received_size, ctx->frame.content.data.bytes + ctx->read_size, current_size);

        ctx->read_size += current_size;

        if(expansion_rpc_is_read_complete(ctx)) {
            // Confirm the frame
            if(!expansion_send_status(ExpansionFrameErrorNone)) break;
        }

        received_size += current_size;
    }

    return (received_size == data_size);
}

static bool expansion_receive_rpc_message(PB_Main* message) {
    ExpansionRpcContext ctx = {};

    pb_istream_t is = {
        .callback = expansion_rpc_decode_callback,
        .state = &ctx,
        .bytes_left = SIZE_MAX,
        .errmsg = NULL,
    };

    return pb_decode_ex(&is, &PB_Main_msg, message, PB_DECODE_DELIMITED);
}

static bool
    expansion_rpc_encode_callback(pb_ostream_t* stream, const pb_byte_t* data, size_t data_size) {
    size_t sent_size = 0;

    while(sent_size != data_size) {
        const size_t current_size = MIN(data_size - sent_size, EXPANSION_PROTOCOL_MAX_DATA_SIZE);
        if(!expansion_send_data_request(data + sent_size, current_size)) break;

        ExpansionFrame rx_frame;
        if(!expansion_receive_frame(&rx_frame)) break;
        if(!expansion_is_success_frame(&rx_frame)) break;

        sent_size += current_size;
    }

    return (sent_size == data_size);
}

static bool expansion_send_rpc_message(PB_Main* message) {
    pb_ostream_t os = {
        .callback = expansion_rpc_encode_callback,
        .state = NULL,
        .max_size = SIZE_MAX,
        .bytes_written = 0,
        .errmsg = NULL,
    };

    const bool success = pb_encode_ex(&os, &PB_Main_msg, message, PB_ENCODE_DELIMITED);
    pb_release(&PB_Main_msg, message);
    return success;
}

static inline bool expansion_is_success_rpc_response(const PB_Main* message) {
    return message->command_status == PB_CommandStatus_OK &&
           message->which_content == PB_Main_empty_tag;
}

static inline bool expansion_is_input_rpc_response(const PB_Main* message) {
    return message->command_id == 0 && message->command_status == PB_CommandStatus_OK &&
           message->which_content == PB_Main_gui_send_input_event_request_tag;
}

static inline bool expansion_is_screen_frame_rpc_response(const PB_Main* message) {
    return message->command_id == 0 && message->command_status == PB_CommandStatus_OK &&
           message->which_content == PB_Main_gui_screen_frame_tag;
}

// Main states

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
        vTaskDelay(pdMS_TO_TICKS(EXPANSION_PROTOCOL_BAUD_CHANGE_DT_MS));
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

static uint32_t expansion_get_next_command_id() {
    static uint32_t command_id;
    while(++command_id == 0)
        ;
    return command_id;
}

static bool expansion_start_virtual_display() {
    bool success = false;

    rpc_message.command_id = expansion_get_next_command_id();
    rpc_message.command_status = PB_CommandStatus_OK;
    rpc_message.which_content = PB_Main_gui_start_virtual_display_request_tag;
    rpc_message.has_next = false;

    PB_Gui_StartVirtualDisplayRequest* request =
        &rpc_message.content.gui_start_virtual_display_request;

    request->send_input = true;
    request->has_first_frame = true;
    request->first_frame.orientation = PB_Gui_ScreenOrientation_HORIZONTAL;
    request->first_frame.data = malloc(PB_BYTES_ARRAY_T_ALLOCSIZE(FLIPPER_BITMAP_SIZE));
    request->first_frame.data->size = FLIPPER_BITMAP_SIZE;

    memcpy(request->first_frame.data->bytes, bitmap_splash_screen, FLIPPER_BITMAP_SIZE);

    do {
        if(!expansion_send_rpc_message(&rpc_message)) break;
        if(!expansion_receive_rpc_message(&rpc_message)) break;
        if(!expansion_is_success_rpc_response(&rpc_message)) break;

        // Show the same picture on display
        uint8_t* frame_buffer = calloc(FLIPPER_BITMAP_SIZE, sizeof(uint8_t));

        bitmap_xbm_to_screen_frame(
            frame_buffer, bitmap_splash_screen, FLIPPER_SCREEN_WIDTH, FLIPPER_SCREEN_HEIGHT);
        frame_parse_data(
            OrientationHorizontal,
            (const frame_t*)frame_buffer,
            pdMS_TO_TICKS(EXPANSION_MODULE_TIMEOUT_MS));

        free(frame_buffer);
        success = true;
    } while(false);

    pb_release(&PB_Main_msg, &rpc_message);
    return success;
}

static bool expansion_wait_input() {
    bool success = false;

    while(!success) {
        if(!expansion_receive_rpc_message(&rpc_message)) break;
        if(!expansion_is_input_rpc_response(&rpc_message)) break;

        const PB_Gui_SendInputEventRequest* request =
            &rpc_message.content.gui_send_input_event_request;
        success = (request->type == PB_Gui_InputType_RELEASE);
        pb_release(&PB_Main_msg, &rpc_message);
    };

    pb_release(&PB_Main_msg, &rpc_message);
    return success;
}

static bool expansion_stop_virtual_display() {
    bool success = false;

    rpc_message.command_id = expansion_get_next_command_id();
    rpc_message.command_status = PB_CommandStatus_OK;
    rpc_message.which_content = PB_Main_gui_stop_virtual_display_request_tag;
    rpc_message.has_next = false;

    if(expansion_send_rpc_message(&rpc_message)) {
        while(true) {
            if(!expansion_receive_rpc_message(&rpc_message)) break;
            if(!expansion_is_input_rpc_response(&rpc_message)) {
                success = expansion_is_success_rpc_response(&rpc_message);
                break;
            }

            pb_release(&PB_Main_msg, &rpc_message);
        }
    }

    pb_release(&PB_Main_msg, &rpc_message);
    return success;
}

static bool expansion_start_screen_streaming() {
    bool success = false;

    rpc_message.command_id = expansion_get_next_command_id();
    rpc_message.command_status = PB_CommandStatus_OK;
    rpc_message.which_content = PB_Main_gui_start_screen_stream_request_tag;
    rpc_message.has_next = false;

    do {
        if(!expansion_send_rpc_message(&rpc_message)) break;
        if(!expansion_receive_rpc_message(&rpc_message)) break;
        if(!expansion_is_success_rpc_response(&rpc_message)) break;
        success = true;
    } while(false);

    pb_release(&PB_Main_msg, &rpc_message);
    return success;
}

static void expansion_process_screen_streaming() {
    while(true) {
        if(!expansion_receive_rpc_message(&rpc_message)) break;
        if(!expansion_is_screen_frame_rpc_response(&rpc_message)) break;

        // Display frame
        const PB_Gui_ScreenOrientation orientation =
            rpc_message.content.gui_screen_frame.orientation;
        const pb_byte_t* data = rpc_message.content.gui_screen_frame.data->bytes;

        frame_parse_data(
            orientation, (const frame_t*)data, pdMS_TO_TICKS(EXPANSION_MODULE_TIMEOUT_MS));

        pb_release(&PB_Main_msg, &rpc_message);
    }

    pb_release(&PB_Main_msg, &rpc_message);
}

static void uart_task(void* unused_arg) {
    // startup delay (skip potential module insertion interference)
    vTaskDelay(pdMS_TO_TICKS(EXPANSION_MODULE_STARTUP_DELAY_MS));
    // init stream buffer
    stream = xStreamBufferCreate(EXPANSION_MODULE_BUFFER_SIZE, 1);
    assert(stream != NULL);

    // init uart 0
    uart_init(uart0, UART_INIT_BAUD_RATE);

    // init uart gpio
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    gpio_set_pulls(UART_RX_PIN, true, false);
    gpio_set_pulls(UART_TX_PIN, true, false);

    // enable uart fifo
    uart_set_fifo_enabled(UART_ID, true);

    // config uart for 8N1 transmission
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    // disable hardware flow control
    uart_set_hw_flow(UART_ID, false, false);

    // add uart irqs
    irq_set_exclusive_handler(UART_IRQ, uart_on_rx);
    irq_set_enabled(UART_IRQ, true);

    // enable rx irq
    uart_set_irq_enables(UART_ID, true, false);

    // show splash screen only once per power-up
    bool splash_screen_shown = false;

    while(true) {
        // leds: activate waiting state
        led_state_wait();

        // reset baud rate to initial value
        uart_set_baudrate(UART_ID, UART_INIT_BAUD_RATE);
        // announce presence (one pulse high -> low)
        uart_putc_raw(UART_ID, 0xF0);

        // wait for host response
        if(!expansion_wait_ready()) continue;
        // negotiate baud rate
        if(!expansion_handshake()) continue;
        // start rpc
        if(!expansion_start_rpc()) continue;

        // leds: activate active state
        led_state_active();

        if(!splash_screen_shown) {
            // start virtual display
            if(!expansion_start_virtual_display()) continue;
            // wait for button press
            if(!expansion_wait_input()) continue;
            // stop virtual display
            if(!expansion_stop_virtual_display()) continue;

            splash_screen_shown = true;
        }
        // start screen streaming
        if(!expansion_start_screen_streaming()) continue;
        // process screen frame messages - returns only on error
        expansion_process_screen_streaming();
    }
}

void uart_protocol_init(void) {
    TaskHandle_t uart_task_handle = NULL;
    BaseType_t status = xTaskCreate(uart_task, "uart_task", 4 * 1024, NULL, 1, &uart_task_handle);
    assert(status == pdPASS);
    (void)status;
}
