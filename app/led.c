#include "led.h"
#include <hardware/gpio.h>

#define BLUE_PIN 25
#define RED_PIN 24

static bool leds_active = true;

void led_init(void) {
    gpio_init(BLUE_PIN);
    gpio_set_dir(BLUE_PIN, GPIO_OUT);
    gpio_put(BLUE_PIN, 0);

    gpio_init(RED_PIN);
    gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_put(RED_PIN, 0);
}

void led_blue(bool on) {
    if(!leds_active) return;
    gpio_put(BLUE_PIN, on ? 1 : 0);
}

void led_red(bool on) {
    if(!leds_active) return;
    gpio_put(RED_PIN, on ? 1 : 0);
}

void led_disable(void) {
    leds_active = false;
}