#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void led_state_init(void);

void led_state_wait(void);

void led_state_active(void);

#ifdef __cplusplus
}
#endif