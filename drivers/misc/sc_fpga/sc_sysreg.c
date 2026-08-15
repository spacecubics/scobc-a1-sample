/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_sysreg

#include "sc_sysreg.h"

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(sc_sysreg, CONFIG_SC_FPGASYS_LOG_LEVEL);

#define SCOBCA1_SYSREG_BASE DT_INST_REG_ADDR(0)

#define SCOBCA1_SYSREG_CODEMSEL  (SCOBCA1_SYSREG_BASE + 0x0000)
#define SCOBCA1_SYSREG_CFGMEMCTL (SCOBCA1_SYSREG_BASE + 0x0010)
#define SCOBCA1_SYSREG_BOOTSTS   (SCOBCA1_SYSREG_BASE + 0x0014)
#define SCOBCA1_SYSREG_VER       (SCOBCA1_SYSREG_BASE + 0xF000)
#define SCOBCA1_SYSREG_BUILDINFO (SCOBCA1_SYSREG_BASE + 0xFF00)
#define SCOBCA1_SYSREG_DNA1      (SCOBCA1_SYSREG_BASE + 0xFF10)
#define SCOBCA1_SYSREG_DNA2      (SCOBCA1_SYSREG_BASE + 0xFF14)

#define SCOBCA1_SYSREG_CFGBOOTMEM(x) (((x) & BIT(12)) >> 12)
#define SCOBCA1_SYSREG_CFGMEMSEL(x)  (((x) & BIT(0)) << 4)
#define SCOBCA1_SYSREG_CFGMEMMON(x)  (((x) & BIT(5)) >> 5)

#define SCOBCA1_SYSREG_MAGIC (0x5A5A0000)

#define CFG_MEMSEL_RETRY_COUNT    (100U)
#define CFG_MEMSEL_RETRY_INTERVAL K_MSEC(1)

#define BOOTSTS_FALLBACK BIT(1)

enum sc_cfgmem sc_get_boot_cfgmem(void)
{
	return SCOBCA1_SYSREG_CFGBOOTMEM(sys_read32(SCOBCA1_SYSREG_CFGMEMCTL));
}

enum sc_cfgmem sc_get_cfgmem(void)
{
	return SCOBCA1_SYSREG_CFGMEMMON(sys_read32(SCOBCA1_SYSREG_CFGMEMCTL));
}

int sc_select_cfgmem(enum sc_cfgmem mem)
{
	uint16_t retry = CFG_MEMSEL_RETRY_COUNT;

	if (mem != SC_CFG_MEM_0 && mem != SC_CFG_MEM_1) {
		return -EINVAL;
	}

	sys_write32(SCOBCA1_SYSREG_CFGMEMSEL(mem), SCOBCA1_SYSREG_CFGMEMCTL);

	while (true) {
		if (sc_get_cfgmem() == mem) {
			return 0;
		}

		if (retry-- == 0) {
			break;
		}

		k_sleep(CFG_MEMSEL_RETRY_INTERVAL);
	}

	return -ETIMEDOUT;
}

void sc_select_codemem(enum sc_codemem mem)
{
	sys_write32(SCOBCA1_SYSREG_MAGIC | mem, SCOBCA1_SYSREG_CODEMSEL);
}

uint32_t sc_get_fpga_ip_ver(void)
{
	return sys_read32(SCOBCA1_SYSREG_VER);
}

uint32_t sc_get_fpga_build_hash(void)
{
	return sys_read32(SCOBCA1_SYSREG_BUILDINFO);
}

uint32_t sc_get_fpga_dna_1(void)
{
	return sys_read32(SCOBCA1_SYSREG_DNA1);
}

uint32_t sc_get_fpga_dna_2(void)
{
	return sys_read32(SCOBCA1_SYSREG_DNA2);
}

uint32_t sc_get_bootsts(void)
{
	return sys_read32(SCOBCA1_SYSREG_BOOTSTS);
}

bool sc_is_fallback(void)
{
	return (sc_get_bootsts() & BOOTSTS_FALLBACK) != 0;
}

static int sc_sysreg_init(void)
{
	return 0;
}

SYS_INIT(sc_sysreg_init, POST_KERNEL, CONFIG_SC_FPGASYS_INIT_PRIORITY);
