/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "sc_sysmon.h"

int main(void)
{
	int ret;
	float temp;
	int32_t bus;

	ret = sc_bhm_enable();
	if (ret < 0) {
		printf("Failed to enable the Board Health Monitor: %d\n", ret);
		goto end;
	}

	/* Wait for the first monitoring to finish */
	k_sleep(K_SECONDS(1));

	while (true) {
		ret = sc_bhm_get_obc_temp(SCOBC_A1_TEMP_1, &temp);
		if (ret < 0) {
			printf("Failed to get the on Board Temperature 1: %d\n", ret);
			goto end;
		}
		printf("On Board Tempareture 1 : %.4f [deg]\n", (double)temp);

		ret = sc_bhm_get_obc_cv(SCOBC_A1_3V3_BUS, &bus);
		if (ret < 0) {
			printf("Failed to get the 3V3SYS Bus voltage: %d\n", ret);
			goto end;
		}
		printf("3V3SYS Bus voltage     : %d [mv]\n", bus);

		k_sleep(K_SECONDS(1));
	}

end:
	sc_bhm_disable();

	return ret;
}
