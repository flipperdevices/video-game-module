#include "cli_commands.h"
#include <pico/unique_id.h>

void cli_device_info(Cli* cli, std::string& args) {
    const size_t len = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
    char id[len];
    pico_get_unique_board_id_string(id, len);
    cli_printf(cli, "flash_id: %s" EOL, id);
    cli_printf(cli, "fw_build_date: %s" EOL, FW_BUILD_DATE);
    cli_printf(cli, "fw_git_commit: %s" EOL, FW_GIT_COMMIT);
    cli_printf(cli, "fw_git_branch: %s" EOL, FW_GIT_BRANCH);
    cli_printf(cli, "fw_git_tag: %s" EOL, FW_GIT_TAG);
}