#include "usb.h"
#include <stdio.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "cli/cli.h"

static void usb_task(void* unused_arg) {
    while(true) {
        cli_work();
    }
}

void usb_init(void) {
    TaskHandle_t task_handle = NULL;
    BaseType_t status = xTaskCreate(usb_task, "usb_task", 1024, NULL, 1, &task_handle);
    assert(status == pdPASS);
}