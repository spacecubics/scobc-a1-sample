/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>

#define TMP175_I2C_ADDR (0x4B)
#define TMP175_TEMP_REG (0x00)

int main(void)
{
	int ret;
	uint8_t data[2];
	float temp;

	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c)) {
		printf("I2C device is not ready\n");
		ret = -1;
		goto end;
	}

	ret = i2c_burst_read(i2c, TMP175_I2C_ADDR, TMP175_TEMP_REG, data, ARRAY_SIZE(data));
	if (ret < 0) {
		printf("Failed to read from Temperature Sensor (%d)\n", ret);
		goto end;
	}

	/*
	 * The TMP175 temperature sensor provides temperature data with a 12-bit, and resolution
	 * is 0.0625°C. Since the data is transmitted over I2C as 16 bits, the lower 4 bits of
	 * the second byte should be discarded. For more details, please refer to the TMP175
	 * datasheet.
	 */
	data[1] = data[1] >> 4;
	temp = (int8_t)data[0] + (float)data[1] * 0.0625f;

	printf("Temperature: %.4f [deg]\n", (double)temp);

end:
	return ret;
}
