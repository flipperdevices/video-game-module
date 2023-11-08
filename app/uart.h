#pragma once
#include <stdint.h>
#include <stdlib.h>

typedef void (*uart_rx_callback_t)(uint8_t* data, size_t length, void* context);

void uart_protocol_init(uart_rx_callback_t callback, void* context);

void uart_protocol_send(uint8_t* data, size_t length);