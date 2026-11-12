# Copyright (c) 2025 Space Cubics Inc.
# SPDX-License-Identifier: Apache-2.0

menuconfig SC_QSPI_NOR_A1
	bool "Space Cubics SC-OBC Module A1 QSPI NOR flash driver"
	default y
	depends on DT_HAS_SC_QSPI_NOR_A1_ENABLED
	select FLASH_HAS_DRIVER_ENABLED
	select FLASH_HAS_EXPLICIT_ERASE
	select FLASH_HAS_PAGE_LAYOUT
	select FLASH_JESD216
	help
	  Enable support for Space Cubics SC-OBC Module A1 QSPI NOR flash driver.

if SC_QSPI_NOR_A1

config SC_QSPI_NOR_A1_IDLE_TIMEOUT_MS
	int "Timeout duration for waiting until the QSPI Bus becomes idle.[ms]"
	default 100
	help
	  Timeout duration for waiting until the QSPI Bus becomes idle. [ms]

config SC_QSPI_NOR_A1_SLEEP_ERASE_MS
	int "Poll interval for erase [ms]"
	default 50
	range 1 800
	help
	  How long to sleep between status register polls during erase.
	  Typical time for 4KB erase command for the NOR flash S25FL256L is 50ms.

config SC_QSPI_NOR_A1_ERASE_TIMEOUT_MS
	int "Total timeout for erase operation [ms]"
	default 1000
	range 1 2000
	help
	  Maximum time to wait for an erase operation to complete while polling WIP.
	  The required time depends on the flash device and erase command (sector/block).
	  Typical time is 270ms for 64KB erase command, max 725ms for the NOR flash S25FL256L.
	  Set this higher than the datasheet maximum to account for system jitter and margins.

config SC_QSPI_NOR_A1_SLEEP_WRITE_US
	int "Poll interval for erase [us]"
	default 300
	range 1 1200
	help
	  How long to sleep between status register polls during page programming.

config SC_QSPI_NOR_A1_WRITE_TIMEOUT_US
	int "Total timeout for write (page program) [us]"
	default 2000
	range 1 3000
	help
	  Maximum time to wait for a page program to complete.
	  Typical time is 300us, max 1.2ms for the NOR flash S25FL256L.

endif
