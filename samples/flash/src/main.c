/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>

#include "../../../drivers/flash/flash_sc_qspi_a1.h"

/*
 * Register Addresses Map (S25FL256L / S25FL128L)
 * Based on Table 42 of the Datasheet.
 */

/* --- Non-Volatile Registers (Stored in Flash memory) --- */
#define ADDR_CR1NV 0x000002 /* Configuration Register 1 NV */
#define ADDR_CR2NV 0x000003 /* Configuration Register 2 NV */
#define ADDR_CR3NV 0x000004 /* Configuration Register 3 NV */

/* --- Volatile Registers (Shadow registers in RAM, active settings) --- */
#define ADDR_SR1V 0x800000 /* Status Register 1 Volatile */
#define ADDR_SR2V 0x800001 /* Status Register 2 Volatile */
#define ADDR_CR1V 0x800002 /* Configuration Register 1 Volatile */
#define ADDR_CR2V 0x800003 /* Configuration Register 2 Volatile */
#define ADDR_CR3V 0x800004 /* Configuration Register 3 Volatile */

/* Target: data_flash0 as defined in the Device Tree */
#define FLASH_DEVICE_NODE DT_NODELABEL(data_flash0)

#define TEST_CHUNK_SIZE 256
#define SECTOR_SIZE     4096
#define TEST_OFFSET     0x00000000 /* Test at the beginning of flash */

static uint8_t wr_buf[TEST_CHUNK_SIZE];
static uint8_t rd_buf[TEST_CHUNK_SIZE];

/* Helper function to read and log a register */
static void read_and_log_reg(const struct device *dev, const char *reg_name, uint32_t addr)
{
	uint8_t val;
	int ret;

	ret = sc_qspi_nor_a1_read_any_reg(dev, addr, &val);

	if (ret == 0) {
		printf("%-6s (Addr: 0x%06x) = 0x%02x\n", reg_name, addr, val);
	} else {
		printf("Failed to read %s (Addr: 0x%06x), Error: %d\n", reg_name, addr, ret);
	}
}

static int verify_erased(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (buf[i] != 0xFF) {
			printf("Erase verification failed, aborting at offset %zu: 0x%02x\n", i,
				buf[i]);
			return -1;
		}
	}
	return 0;
}

static int verify_data(const uint8_t *expected, const uint8_t *read, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (expected[i] != read[i]) {
			printf("Data mismatch, aborting at offset %zu: expected 0x%02x, got "
				"0x%02x\n",
				i, expected[i], read[i]);
			return -1;
		}
	}
	return 0;
}

int main(void)
{
	const struct device *flash_dev = DEVICE_DT_GET(FLASH_DEVICE_NODE);
	int ret;

	printf("=== TEST DATA_FLASH0 ===\n");

	if (!device_is_ready(flash_dev)) {
		printf("Error: Device data_flash0 is not ready or not found!\n");
		return -1;
	}

	printf("Device found: %s\n", flash_dev->name);

	printf("1. Erasing sector at 0x%x...\n", TEST_OFFSET);
	ret = flash_erase(flash_dev, TEST_OFFSET, SECTOR_SIZE);
	if (ret != 0) {
		printf("Erase failed: %d\n", ret);
		return -1;
	}

	printf("2. Erased sector verification at 0x%x...\n", TEST_OFFSET);
	memset(rd_buf, 0, TEST_CHUNK_SIZE);
	ret = flash_read(flash_dev, TEST_OFFSET, rd_buf, TEST_CHUNK_SIZE);
	if (ret != 0) {
		printf("Read after erase failed: %d\n", ret);
		return -1;
	}

	ret = verify_erased(rd_buf, sizeof(rd_buf));
	if (ret) {
		printf("Erase verification FAILED\n");
		return -1;
	}

	printf("Erase verification OK\n");

	for (size_t i = 0; i < TEST_CHUNK_SIZE; i++) {
		wr_buf[i] = (uint8_t)(i + 0x55); /* Simple incrementing pattern */
	}

	printf("3. Writing %d bytes...\n", TEST_CHUNK_SIZE);
	ret = flash_write(flash_dev, TEST_OFFSET, wr_buf, TEST_CHUNK_SIZE);
	if (ret != 0) {
		printf("Write failed: %d\n", ret);
		return -1;
	}

	printf("4. Wrote data readback and verification at 0x%x...\n", TEST_OFFSET);
	memset(rd_buf, 0, TEST_CHUNK_SIZE);
	ret = flash_read(flash_dev, TEST_OFFSET, rd_buf, TEST_CHUNK_SIZE);
	if (ret != 0) {
		printf("Read failed: %d\n", ret);
		return -1;
	}

	ret = verify_data(wr_buf, rd_buf, TEST_CHUNK_SIZE);
	if (ret) {
		printf("Write/Read verification FAILED\n");
		return -1;
	}

	printf("Write verification OK\n");

	printf("==========================================\n");
	printf("   S25FL256L Register Dump (RDAR 0x65)    \n");
	printf("==========================================\n");

	/* 1. Read Non-Volatile Registers (Factory/Stored Settings) */
	printf("--- Non-Volatile Registers (NV) ---\n");
	read_and_log_reg(flash_dev, "CR1NV", ADDR_CR1NV);
	read_and_log_reg(flash_dev, "CR2NV", ADDR_CR2NV);
	read_and_log_reg(flash_dev, "CR3NV", ADDR_CR3NV);

	/* 2. Read Volatile Registers (Current Active Settings) */
	printf("--- Volatile Registers (V) ---\n");
	read_and_log_reg(flash_dev, "SR1V", ADDR_SR1V);
	read_and_log_reg(flash_dev, "SR2V", ADDR_SR2V);
	read_and_log_reg(flash_dev, "CR1V", ADDR_CR1V);
	read_and_log_reg(flash_dev, "CR2V", ADDR_CR2V);
	read_and_log_reg(flash_dev, "CR3V", ADDR_CR3V);

	return 0;
}
