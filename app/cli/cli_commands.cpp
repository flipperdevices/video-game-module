#include "cli_commands.h"
#include <string.h>

void cli_gpio(Cli* cli, std::string& args);
void cli_device_info(Cli* cli, std::string& args);
void cli_imu_test(Cli* cli, std::string& args);

void cli_help(Cli* cli, std::string& args) {
    size_t max_len = 0;
    for(size_t i = 0; i < cli_items_count; i++) {
        if(strlen(cli_items[i].name) > max_len) {
            max_len = strlen(cli_items[i].name);
        }
    }

    max_len += 1;

    for(size_t i = 0; i < cli_items_count; i++) {
        cli_write_str(cli, cli_items[i].name);

        if(cli_items[i].desc != NULL && strlen(cli_items[i].desc)) {
            for(size_t s = 0; s < (max_len - strlen(cli_items[i].name)); s++) {
                cli_write_str(cli, " ");
            }
            cli_write_str(cli, "- ");
            cli_write_str(cli, cli_items[i].desc);
        }

        if((i + 1) < cli_items_count) {
            cli_write_eol(cli);
        }
    }

    cli_write_eol(cli);
}

const CliItem cli_items[] = {
    {
        .name = "!",
        .desc = "alias for device_info",
        .callback = cli_device_info,
    },
    {
        .name = "?",
        .desc = "alias for help",
        .callback = cli_help,
    },
    {
        .name = "help",
        .desc = "show this help",
        .callback = cli_help,
    },
    {
        .name = "device_info",
        .desc = "show device info",
        .callback = cli_device_info,
    },
    {
        .name = "gpio",
        .desc = "gpio control",
        .callback = cli_gpio,
    },
    {
        .name = "imu_test",
        .desc = "test the IMU",
        .callback = cli_imu_test,
    },
};

size_t cli_items_count = sizeof(cli_items) / sizeof(CliItem);