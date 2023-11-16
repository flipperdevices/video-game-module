#include "cli_commands.h"
#include <FreeRTOS.h>
#include <task.h>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <stdarg.h>

struct Cli {
    std::string line;
    std::string prev_line;
    size_t cursor_position;
    bool esc_mode;
};

typedef enum {
    SOH = 0x01,
    ETX = 0x03,
    EOT = 0x04,
    Bell = 0x07,
    Backspace = 0x08,
    Tab = 0x09,
    CR = 0x0D,
    Esc = 0x1B,
    US = 0x1F,
    Space = 0x20,
    Del = 0x7F,
} CliSymbol;

static void cli_reset(Cli* cli) {
    cli->line.clear();
    cli->cursor_position = 0;
}

// trim from start
static inline std::string& ltrim(std::string& s) {
    s.erase(
        s.begin(),
        std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
static inline std::string& rtrim(std::string& s) {
    s.erase(
        std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(),
        s.end());
    return s;
}

// trim from both ends
static inline std::string& trim(std::string& s) {
    return ltrim(rtrim(s));
}

static const CliItem* cli_search_item(Cli* cli, const std::string& name) {
    for(size_t i = 0; i < cli_items_count; i++) {
        if(name == cli_items[i].name) {
            return &cli_items[i];
        }
    }
    return NULL;
}

static void cli_handle_enter(Cli* cli) {
    std::string command = "";
    std::string args = "";
    const CliItem* item = NULL;

    cli->prev_line = cli->line;
    cli->line = trim(cli->line);

    size_t ws = cli->line.find_first_of(" ");
    if(ws == std::string::npos) {
        command = cli->line;
    } else {
        command = cli->line.substr(0, ws);
        args = cli->line.substr(ws + 1);
        args = trim(args);
    }

    item = cli_search_item(cli, command);
    if(item != NULL) {
        item->callback(cli, args);
    } else {
        cli_write_str(cli, "Not found");
    }
}

static void cli_handle_backspace(Cli* cli) {
    if(cli->line.size() > 0 && cli->cursor_position > 0) {
        // Other side
        cli_write_str(cli, "\e[D\e[1P");
        // Our side
        cli->line.erase(cli->cursor_position - 1, 1);
        cli->cursor_position--;
    } else {
        cli_write_char(cli, CliSymbol::Bell);
    }
}

static void cli_force_motd(Cli* cli) {
    // cli_write_motd(cli);
    cli_write_eol(cli);
    cli_write_prompt(cli);
    cli_flush(cli);
}

static void cli_handle_char(Cli* cli, uint8_t c) {
    if(cli->esc_mode) {
        switch(c) {
        case '[':
            cli->esc_mode = true;
            break;
        case 'A': // Up
            if(cli->line.size() == 0 && cli->line != cli->prev_line) {
                // Set line buffer and cursor position
                cli->line = cli->prev_line;
                cli->cursor_position = cli->line.size();
                // Show new line to user
                cli_write_str(cli, cli->line.c_str());
            }
            cli->esc_mode = false;
            break;
        case 'B': // Down
            cli->esc_mode = false;
            break;
        case 'C': // Right
            if(cli->cursor_position < cli->line.size()) {
                cli_write_str(cli, "\e[C");
                cli->cursor_position++;
            }
            cli->esc_mode = false;
            break;
        case 'D': // Left
            if(cli->cursor_position > 0) {
                cli_write_str(cli, "\e[D");
                cli->cursor_position--;
            }
            cli->esc_mode = false;
            break;
        default:
            cli->esc_mode = false;
            break;
        }
    } else {
        switch(c) {
        case CliSymbol::Esc:
            cli->esc_mode = true;
            break;
        case CliSymbol::CR:
            if(cli->line.size() == 0) {
                cli_write_eol(cli);
            } else {
                cli_write_eol(cli);
                cli_handle_enter(cli);
                cli_reset(cli);
                cli_write_eol(cli);
            }
            cli_write_prompt(cli);
            break;
        case CliSymbol::Del:
        case CliSymbol::Backspace:
            cli_handle_backspace(cli);
            break;
        case CliSymbol::SOH:
            vTaskDelay(pdMS_TO_TICKS(33));
            cli_force_motd(cli);
            break;
        case CliSymbol::ETX:
            cli_reset(cli);
            cli_write_eol(cli);
            cli_write_prompt(cli);
            break;
        case CliSymbol::EOT:
            cli_reset(cli);
            break;
        case ' ' ... '~':
            if(cli->cursor_position == cli->line.size()) {
                cli->line.push_back(c);
                cli_write_char(cli, c);
                cli->cursor_position++;
            } else {
                cli->line.insert(cli->cursor_position, 1, c);
                cli_write_str(cli, "\e[1@");
                cli_write_str(cli, cli->line.substr(cli->cursor_position).c_str());
                cli->cursor_position++;
                cli_write_str(cli, "\e[D");
            }
            break;
        default:
            break;
        }
    }

    cli_flush(cli);
}

void cli_write_prompt(Cli* cli) {
    cli_write_str(cli, ">: ");
}

void cli_write_str(Cli* cli, const char* str) {
    fputs(str, stdout);
}

void cli_write_char(Cli* cli, char c) {
    putchar(c);
}

void cli_write_eol(Cli* cli) {
    cli_write_str(cli, EOL);
}

void cli_flush(Cli* cli) {
    fflush(stdout);
}

void cli_printf(Cli* cli, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

std::vector<std::string> cli_split_args(std::string& args) {
    std::vector<std::string> argv;
    std::string::size_type pos = 0;
    std::string::size_type prev = 0;
    while((pos = args.find_first_of(" ", prev)) != std::string::npos) {
        if(pos > prev) {
            argv.push_back(args.substr(prev, pos - prev));
        }
        prev = pos + 1;
    }
    if(prev < args.size()) {
        argv.push_back(args.substr(prev));
    }
    return argv;
}

extern "C" void cli_work(void) {
    Cli cli = {
        .line = "",
        .prev_line = "",
        .cursor_position = 0,
        .esc_mode = false,
    };

    cli_force_motd(&cli);

    while(true) {
        char c = getchar();
        cli_handle_char(&cli, c);
    }
}