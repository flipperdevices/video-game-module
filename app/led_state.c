#include "led.h"
#include "led_state.h"
#include <FreeRTOS.h>
#include <task.h>
#include <assert.h>

typedef enum {
    LED_STATE_WAIT,
    LED_STATE_ACTIVE,
} led_state_t;

volatile static led_state_t led_state = LED_STATE_WAIT;

static void leds(bool red, bool green, bool blue) {
    led_red(red);
    led_green(green);
    led_blue(blue);
}

static void led_task(void* unused_arg) {
    while(1) {
        switch(led_state) {
        case LED_STATE_WAIT:
            leds(false, false, true);
            vTaskDelay(pdMS_TO_TICKS(250));
            leds(false, false, false);
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        case LED_STATE_ACTIVE:
            leds(false, true, false);
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        default:
            break;
        }
    }
}

void led_state_init(void) {
    BaseType_t status;

    TaskHandle_t led_task_handle = NULL;
    status = xTaskCreate(led_task, "led_task", 128, NULL, 1, &led_task_handle);
    assert(status == pdPASS);
    (void)status;
}

void led_state_wait(void) {
    led_state = LED_STATE_WAIT;
}

void led_state_active(void) {
    led_state = LED_STATE_ACTIVE;
}