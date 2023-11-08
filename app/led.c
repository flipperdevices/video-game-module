#include "led.h"
#include <hardware/gpio.h>

#define LED_PIN 25

void led_init() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
}

void led_set(bool on) {
    gpio_put(LED_PIN, on ? 0 : 1);
}