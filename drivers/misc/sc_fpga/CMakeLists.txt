# Copyright (c) 2025 Space Cubics Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SC_SYSMON sc_sysmon.c)

zephyr_include_directories(.)
