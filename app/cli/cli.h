#pragma once
#define EOL "\n"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Cli Cli;

void cli_work(void);

void cli_write_prompt(Cli* cli);
void cli_write_str(Cli* cli, const char* str);
void cli_write_char(Cli* cli, char ch);
void cli_write_eol(Cli* cli);
void cli_flush(Cli* cli);
void cli_printf(Cli* cli, const char* format, ...);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <string>
#include <vector>

std::vector<std::string> cli_split_args(std::string& args);
#endif