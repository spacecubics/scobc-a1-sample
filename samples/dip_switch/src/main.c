/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>

int main(void)
{
	int ret;
	int curr_state;
	int last_state;
	const struct gpio_dt_spec sw1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

	if (!gpio_is_ready_dt(&sw1)) {
		printf("Switch GPIO device not ready\n");
		ret = -ENODEV;
		goto end;
	}

	ret = gpio_pin_configure_dt(&sw1, GPIO_INPUT);
	if (ret < 0) {
		printf("Failed to configure switch pins (%d)\n", ret);
		goto end;
	}

	last_state = gpio_pin_get_dt(&sw1);
	if (last_state < 0) {
		printf("Failed to read initial switch state (%d)\n", last_state);
		ret = last_state;
		goto end;
	}

	printf("Switch SW1[9] initial state: %s\n", last_state ? "ON" : "OFF");

	while (true) {
		curr_state = gpio_pin_get_dt(&sw1);
		if (curr_state < 0) {
			printf("Failed to read switch state (%d)\n", curr_state);
			ret = curr_state;
			goto end;
		}

		if (curr_state != last_state) {
			printf("Switch SW1[9] state changed: %s\n", curr_state ? "ON" : "OFF");
			last_state = curr_state;
		}

		k_sleep(K_MSEC(200));
	}

end:
	return ret;
}
