/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>

#include "sc_sysreg.h"

static int cmd_cfgmem_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Boot bank    : %d", sc_get_boot_cfgmem());
	shell_print(sh, "Current bank : %d", sc_get_cfgmem());

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(cfgmem_cmds,
			       SHELL_CMD(show, NULL,
					 "Show the boot and current Configuration Memory banks",
					 cmd_cfgmem_show),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(cfgmem, &cfgmem_cmds, "Configuration Memory commands", NULL);

int main(void)
{
	return 0;
}
