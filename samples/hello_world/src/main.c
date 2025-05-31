/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

int main(void)
{
	printf("Hello World! %s Rev %s\n", CONFIG_BOARD, CONFIG_BOARD_REVISION);

	return 0;
}
