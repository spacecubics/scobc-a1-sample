/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>

int main(void)
{
	int ret;
	const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		printf("GPIO device is not ready\n");
		ret = -ENODEV;
		goto end;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("Failed to configure a GPIO pin (%d)\n", ret);
		goto end;
	}

	while (true) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			printf("Failed to toggle a GPIO pin (%d)\n", ret);
			goto end;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_sleep(K_SECONDS(1));
	}

end:
	return ret;
}
