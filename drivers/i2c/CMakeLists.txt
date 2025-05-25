# Copyright (c) 2025 Space Cubics Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_I2C_SC i2c_sc.c)
