# Copyright (c) 2025 Space Cubics Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_FLASH)
  zephyr_library_amend()
  zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/flash)
  zephyr_library_sources_ifdef(CONFIG_SC_QSPI_NOR_A1 flash_sc_qspi_a1.c)
endif()
