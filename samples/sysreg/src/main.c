/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "sc_sysreg.h"

#define POWER_CYCLE_TIMEOUT   K_SECONDS(1)
#define CONSOLE_MESSAGE_DELAY K_MSEC(100)

static int cmd_cfgmem_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Boot bank    : %d", sc_get_boot_cfgmem());
	shell_print(sh, "Current bank : %d", sc_get_cfgmem());

	return 0;
}

static int cmd_cfgmem_select(const struct shell *sh, size_t argc, char **argv)
{
	int err = 0;
	int ret;
	unsigned long bank;

	ARG_UNUSED(argc);

	bank = shell_strtoul(argv[1], 10, &err);
	if (err != 0 || bank > SC_CFG_MEM_1) {
		shell_error(sh, "Bank must be 0 or 1");
		return -EINVAL;
	}

	ret = sc_select_cfgmem((enum sc_cfgmem)bank);
	if (ret < 0) {
		shell_error(sh, "Failed to select Configuration Memory bank %lu: %d", bank, ret);
		return ret;
	}

	shell_print(sh, "Current bank : %d", sc_get_cfgmem());

	return 0;
}

static int cmd_power_cycle(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Requesting TRCH to perform a power cycle via FPGA");

	/* Sleep to allow the message to appear on the console. */
	k_sleep(CONSOLE_MESSAGE_DELAY);

	sc_power_cycle();

	/*
	 * Print an error to the console if the power cycle does not complete
	 * within POWER_CYCLE_TIMEOUT.
	 */
	k_sleep(POWER_CYCLE_TIMEOUT);

	shell_error(sh, "Power cycle request timed out");

	return -ETIMEDOUT;
}

SHELL_STATIC_SUBCMD_SET_CREATE(cfgmem_cmds,
			       SHELL_CMD_ARG(select, NULL,
					     "Select Configuration Memory bank 0 or 1",
					     cmd_cfgmem_select, 2, 0),
			       SHELL_CMD(show, NULL,
					 "Show the boot and current Configuration Memory banks",
					 cmd_cfgmem_show),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(cfgmem, &cfgmem_cmds, "Configuration Memory commands", NULL);
SHELL_CMD_REGISTER(pwrcycle, NULL, "Request TRCH to perform a power cycle via FPGA",
		   cmd_power_cycle);

int main(void)
{
	return 0;
}
