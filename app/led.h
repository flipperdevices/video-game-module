#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void led_init(void);

void led_blue(bool on);

void led_red(bool on);

void led_disable(void);

#ifdef __cplusplus
}
#endif