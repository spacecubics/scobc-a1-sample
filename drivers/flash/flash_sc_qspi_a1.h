/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_FLASH_SC_QSPI_A1_H_
#define ZEPHYR_DRIVERS_FLASH_SC_QSPI_A1_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read any register from the SC QSPI NOR Flash of the SC-OBC Module A1 using RDAR (0x65)
 * command.
 *
 * This function allows reading Volatile and Non-Volatile registers directly
 * by their memory address (Refer to the Datasheet).
 *
 * @param dev      Pointer to the device structure.
 * @param reg_addr The register address (e.g., 0x800000 for SR1V for S25FL256L).
 * @param val      Pointer to a 8-bit variable to store the read value.
 *
 * @return 0 on success, negative errno code on failure.
 */
int sc_qspi_nor_a1_read_any_reg(const struct device *dev, uint32_t reg_addr, uint8_t *val);

/**
 * @brief Write to any register in the SC QSPI NOR Flash of the SC-OBC Module A1 using WRAR (0x71)
 * command.
 *
 * This function allows writing to Volatile and Non-Volatile registers directly
 * by their memory address (Refer to the Datasheet).
 *
 * @param dev      Pointer to the device structure.
 * @param reg_addr The register address (e.g., 0x800000 for SR1V for S25FL256L).
 * @param val      The 8-bit value to write to the register.
 *
 * @return 0 on success, negative errno code on failure.
 */
int sc_qspi_nor_a1_write_any_reg(const struct device *dev, uint32_t reg_addr, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_FLASH_SC_QSPI_A1_H_ */
