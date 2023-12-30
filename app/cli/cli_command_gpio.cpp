#include "cli_commands.h"
#include "../led.h"
#include <vector>
#include <hardware/gpio.h>
#include <FreeRTOS.h>
#include <task.h>

typedef struct {
    uint8_t pin;
    bool is_led = false;
    bool open_drain_only = false;
    bool danger = false;
} GPIOItem;

static const GPIOItem gpios[] = {
    {.pin = 7, .danger = true}, // accelerometer, int2 pin
    {.pin = 16}, // external gpio header
    {.pin = 17}, // external gpio header
    {.pin = 21, .is_led = true, .danger = true}, // external gpio header, green led
    {.pin = 22}, // external gpio header
    {.pin = 23}, // external gpio header
    {.pin = 26}, // external gpio header
    {.pin = 27}, // external gpio header
    {.pin = 28}, // external gpio header
    {.pin = 29}, // external gpio header
    {.pin = 24, .is_led = true, .danger = true}, // external gpio header, red led
    {.pin = 25, .is_led = true, .danger = true}, // external gpio header, blue led
    {.pin = 18, .open_drain_only = true, .danger = true}, // dvi sda
    {.pin = 19, .open_drain_only = true, .danger = true}, // dvi scl
    {.pin = 20, .open_drain_only = true, .danger = true}, // dvi cec
};

typedef struct {
    void (*const fn)(Cli* cli, std::vector<std::string>& argv);
    const char* name;
    const size_t argc;
} CliGPIOCommand;

static const size_t gpios_count = sizeof(gpios) / sizeof(gpios[0]);
static bool danger = false;

static bool str_to_int(const char* str, uint8_t* value) {
    char* end;
    long int val = strtol(str, &end, 10);
    if(val > UINT8_MAX || val < 0 || *end != '\0') {
        return false;
    }

    *value = (uint8_t)val;
    return true;
}

static const GPIOItem* gpio_get_and_prepare(Cli* cli, std::string& arg) {
    const GPIOItem* gpio = NULL;
    uint8_t pin;

    if(!str_to_int(arg.c_str(), &pin)) {
        cli_printf(cli, "Invalid pin: %s" EOL, arg.c_str());
        return NULL;
    }

    for(size_t i = 0; i < gpios_count; i++) {
        if(gpios[i].pin == pin) {
            gpio = &gpios[i];
        }
    }

    if(gpio == NULL) {
        cli_printf(cli, "Pin not found: %d" EOL, pin);
        return NULL;
    }

    if(!danger && gpio->danger) {
        cli_printf(
            cli,
            "This pin is dangerous, use \"gpio i_know_what_i'm_doing\" to enable danger mode" EOL);
        return NULL;
    }

    if(gpio->is_led) {
        led_disable();
    }

    return gpio;
}

static void gpio_input(Cli* cli, const GPIOItem* gpio) {
    gpio_init(gpio->pin);
    gpio_set_dir(gpio->pin, GPIO_IN);
    gpio_pull_up(gpio->pin);
    cli_printf(cli, "Value: ");
    vTaskDelay(pdMS_TO_TICKS(25));
    cli_printf(cli, "%d", gpio_get(gpio->pin));
    cli_write_eol(cli);
}

static void gpio_output(Cli* cli, const GPIOItem* gpio, uint8_t value) {
    if(gpio->open_drain_only && value == 1) {
        cli_printf(cli, "This pin is open drain only, value must be 0");
        cli_write_eol(cli);
        return;
    }

    gpio_init(gpio->pin);
    gpio_set_dir(gpio->pin, GPIO_OUT);
    gpio_put(gpio->pin, value);
    cli_printf(cli, "Value set to: %d", value);
    cli_write_eol(cli);
}

static void cli_gpio_help(Cli* cli) {
    cli_printf(cli, "Usage: " EOL);
    cli_printf(cli, "\tgpio list                  - list all gpio pins" EOL);
    cli_printf(cli, "\tgpio out <pin> <0|1>       - set gpio pin to output and set value" EOL);
    cli_printf(cli, "\tgpio in <pin>              - set gpio pin to input and read value" EOL);
    cli_printf(
        cli,
        "\tgpio i_know_what_i'm_doing - enable danger mode, may brick your device if you don't know what you're doing" EOL);
}

static void cli_gpio_list(Cli* cli, std::vector<std::string>& argv) {
    cli_printf(cli, "GPIO pins:" EOL);
    for(size_t i = 0; i < gpios_count; i++) {
        const GPIOItem* gpio = &gpios[i];
        cli_printf(cli, "\t%d", gpio->pin);
        if(gpio->is_led) {
            cli_printf(cli, " (led)");
        }
        if(gpio->open_drain_only) {
            cli_printf(cli, " (open drain only)");
        }
        if(gpio->danger) {
            cli_printf(cli, " (danger)");
        }
        cli_write_eol(cli);
    }
    return;
}

static void cli_gpio_out(Cli* cli, std::vector<std::string>& argv) {
    uint8_t value;
    if(!str_to_int(argv[2].c_str(), &value)) {
        cli_printf(cli, "Invalid value: %s", argv[2].c_str());
        cli_write_eol(cli);
        return;
    }

    if(value != 0 && value != 1) {
        cli_printf(cli, "Invalid value: %s", argv[2].c_str());
        cli_write_eol(cli);
        return;
    }

    const GPIOItem* gpio = gpio_get_and_prepare(cli, argv[1]);
    if(gpio) {
        gpio_output(cli, gpio, value);
    }

    return;
}

static void cli_gpio_in(Cli* cli, std::vector<std::string>& argv) {
    const GPIOItem* gpio = gpio_get_and_prepare(cli, argv[1]);
    if(gpio) {
        gpio_input(cli, gpio);
    }

    return;
}

static void cli_gpio_i_know(Cli* cli, std::vector<std::string>& argv) {
    danger = true;
    cli_printf(
        cli, "Danger mode enabled. Boad may be bricked if you don't know what you're doing!");
    cli_write_eol(cli);
    return;
}

static const CliGPIOCommand cli_gpio_commands[] = {
    {.fn = cli_gpio_list, .name = "list", .argc = 0},
    {.fn = cli_gpio_out, .name = "out", .argc = 2},
    {.fn = cli_gpio_in, .name = "in", .argc = 1},
    {.fn = cli_gpio_i_know, .name = "i_know_what_i'm_doing", .argc = 0},
};

static const size_t cli_gpio_commands_count =
    sizeof(cli_gpio_commands) / sizeof(cli_gpio_commands[0]);

void cli_gpio(Cli* cli, std::string& args) {
    std::vector<std::string> argv = cli_split_args(args);
    if(argv.size() < 1) {
        cli_gpio_help(cli);
        return;
    }

    for(size_t i = 0; i < cli_gpio_commands_count; i++) {
        const CliGPIOCommand* cmd = &cli_gpio_commands[i];
        if(cmd->argc == argv.size() - 1 && cmd->name == argv[0]) {
            cmd->fn(cli, argv);
            return;
        }
    }

    cli_gpio_help(cli);
}