/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_sysmon

#include "sc_sysmon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sc_sysmon, CONFIG_SC_SYSMON_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>

#define SCOBC_A1_SYSMON_BASE DT_REG_ADDR_BY_IDX(DT_INST(0, sc_sysmon), 0)
#define SCOBC_A1_GPTMR_BASE  (0x4F050000)

#define SCOBC_A1_SYSMON_WDOG_CTRL     (SCOBC_A1_SYSMON_BASE + 0x0000)
#define SCOBC_A1_SYSMON_SEM_ECCOUNT   (SCOBC_A1_SYSMON_BASE + 0x0044)
#define SCOBC_A1_SYSMON_XADC_TEMP     (SCOBC_A1_SYSMON_BASE + 0x1000)
#define SCOBC_A1_SYSMON_XADC_VCCINT   (SCOBC_A1_SYSMON_BASE + 0x1010)
#define SCOBC_A1_SYSMON_XADC_VCCAUX   (SCOBC_A1_SYSMON_BASE + 0x1020)
#define SCOBC_A1_SYSMON_XADC_VCCBRAM  (SCOBC_A1_SYSMON_BASE + 0x1060)
#define SCOBC_A1_SYSMON_INICTLR       (SCOBC_A1_SYSMON_BASE + 0x2000)
#define SCOBC_A1_SYSMON_ACCCTLR       (SCOBC_A1_SYSMON_BASE + 0x2004)
#define SCOBC_A1_SYSMON_ISR           (SCOBC_A1_SYSMON_BASE + 0x2010)
#define SCOBC_A1_SYSMON_1V0_SNTVR     (SCOBC_A1_SYSMON_BASE + 0x2020)
#define SCOBC_A1_SYSMON_1V0_BUSVR     (SCOBC_A1_SYSMON_BASE + 0x2024)
#define SCOBC_A1_SYSMON_1V8_SNTVR     (SCOBC_A1_SYSMON_BASE + 0x2028)
#define SCOBC_A1_SYSMON_1V8_BUSVR     (SCOBC_A1_SYSMON_BASE + 0x202C)
#define SCOBC_A1_SYSMON_3V3_SNTVR     (SCOBC_A1_SYSMON_BASE + 0x2030)
#define SCOBC_A1_SYSMON_3V3_BUSVR     (SCOBC_A1_SYSMON_BASE + 0x2034)
#define SCOBC_A1_SYSMON_3V3SYSA_SNTVR (SCOBC_A1_SYSMON_BASE + 0x2038)
#define SCOBC_A1_SYSMON_3V3SYSA_BUSVR (SCOBC_A1_SYSMON_BASE + 0x203C)
#define SCOBC_A1_SYSMON_3V3SYSB_SNTVR (SCOBC_A1_SYSMON_BASE + 0x2040)
#define SCOBC_A1_SYSMON_3V3SYSB_BUSVR (SCOBC_A1_SYSMON_BASE + 0x2044)
#define SCOBC_A1_SYSMON_3V3IO_SNTVR   (SCOBC_A1_SYSMON_BASE + 0x2048)
#define SCOBC_A1_SYSMON_3V3IO_BUSVR   (SCOBC_A1_SYSMON_BASE + 0x204C)
#define SCOBC_A1_SYSMON_TEMP1R        (SCOBC_A1_SYSMON_BASE + 0x2050)
#define SCOBC_A1_SYSMON_TEMP2R        (SCOBC_A1_SYSMON_BASE + 0x2054)
#define SCOBC_A1_SYSMON_TEMP3R        (SCOBC_A1_SYSMON_BASE + 0x2058)

#define SCOBC_A1_SYSMON_CFGMEMCTL_OFFSET (0x0010)
#define SCOBC_A1_SYSMON_VER_OFFSET       (0xF000)
#define SCOBC_A1_SYSMON_BUILDINFO_OFFSET (0xFF00)
#define SCOBC_A1_SYSMON_DNA1_OFFSET      (0xFF10)
#define SCOBC_A1_SYSMON_DNA2_OFFSET      (0xFF14)

#define SCOBC_A1_GPTMR_TECR    (SCOBC_A1_GPTMR_BASE + 0x0004)
#define SCOBC_A1_GPTMR_HITCR   (SCOBC_A1_GPTMR_BASE + 0x0200)
#define SCOBC_A1_GPTMR_HITPR   (SCOBC_A1_GPTMR_BASE + 0x0204)
#define SCOBC_A1_GPTMR_HITOCR1 (SCOBC_A1_GPTMR_BASE + 0x0210)
#define SCOBC_A1_GPTMR_HITOCR2 (SCOBC_A1_GPTMR_BASE + 0x0214)
#define SCOBC_A1_GPTMR_HITOCR3 (SCOBC_A1_GPTMR_BASE + 0x0218)

/* SEM Error Correction Count Register */
#define SCOBC_A1_SYSMON_SEMCCOUNT(x) (((x) & GENMASK(15, 0)))

/* BHM Initialization Access Control Register */
#define SCOBC_A1_SYSMON_INIT_REQ     BIT(16)
#define SCOBC_A1_SYSMON_TEMP3_INITEN BIT(4)
#define SCOBC_A1_SYSMON_TEMP2_INITEN BIT(3)
#define SCOBC_A1_SYSMON_TEMP1_INITEN BIT(2)
#define SCOBC_A1_SYSMON_CVM2_INITEN  BIT(1)
#define SCOBC_A1_SYSMON_CVM1_INITEN  BIT(0)
#define SCOBC_A1_SYSMON_INIT_ALL                                                                   \
	(SCOBC_A1_SYSMON_INIT_REQ | SCOBC_A1_SYSMON_TEMP3_INITEN | SCOBC_A1_SYSMON_TEMP2_INITEN |  \
	 SCOBC_A1_SYSMON_TEMP1_INITEN | SCOBC_A1_SYSMON_CVM2_INITEN | SCOBC_A1_SYSMON_CVM1_INITEN)

/* BHM Access Control Register */
#define SCOBC_A1_SYSMON_TEMP3_MONIEN BIT(4)
#define SCOBC_A1_SYSMON_TEMP2_MONIEN BIT(3)
#define SCOBC_A1_SYSMON_TEMP1_MONIEN BIT(2)
#define SCOBC_A1_SYSMON_CVM2_MONIEN  BIT(1)
#define SCOBC_A1_SYSMON_CVM1_MONIEN  BIT(0)
#define SCOBC_A1_SYSMON_MONIEN_ALL                                                                 \
	(SCOBC_A1_SYSMON_TEMP3_MONIEN | SCOBC_A1_SYSMON_TEMP2_MONIEN |                             \
	 SCOBC_A1_SYSMON_TEMP1_MONIEN | SCOBC_A1_SYSMON_CVM2_MONIEN | SCOBC_A1_SYSMON_CVM1_MONIEN)

/* BHM Interrupt Status Register */
#define SCOBC_A1_SYSMON_INIT_ACCEND BIT(0)

/* BHM Monitor Register */
#define SCOBC_A1_SYSMON_BHM_NUPD BIT(31)

/* Timer Enable Control Register */
#define SCOBC_A1_SYSMON_GPTMR_HITEN BIT(1)

#define SYSMON_RETRY_COUNT (100U)

static float convert_obc_temp(uint32_t raw)
{
	int8_t int_tmp = (raw & 0x0000FF00) >> 8;
	float point = ((raw & 0x000000F0) >> 4) * 0.0625;

	return int_tmp + point;
}

static float convert_xadc_temp(uint32_t raw)
{
	float conv_tmp;

	conv_tmp = ((((raw & 0x0000FFF0) >> 4) * 503.975) / 4096) - 273.15;

	return conv_tmp;
}

static uint32_t convert_cv_shunt(int16_t raw)
{
	return ((raw >> 3) * 40);
}

static uint32_t convert_cv_bus(int16_t raw)
{
	return ((raw >> 3) * 8);
}

static float convert_cv_xadc(uint32_t raw)
{
	float conv_cv;

	conv_cv = (raw & 0x0000FFF0) >> 4;

	return conv_cv / 4096 * 3;
}

static int convert_obc_cv(enum scobc_a1_cv_pos pos, uint32_t raw, int32_t *cv)
{
	int ret = 0;
	int16_t rawval = raw & 0x0000FFFF;

	switch (pos) {
	case SCOBC_A1_1V0_SHUNT:
	case SCOBC_A1_1V8_SHUNT:
	case SCOBC_A1_3V3_SHUNT:
	case SCOBC_A1_3V3_SYSA_SHUNT:
	case SCOBC_A1_3V3_SYSB_SHUNT:
	case SCOBC_A1_3V3_IO_SHUNT:
		*cv = convert_cv_shunt(rawval);
		break;
	case SCOBC_A1_1V0_BUS:
	case SCOBC_A1_1V8_BUS:
	case SCOBC_A1_3V3_BUS:
	case SCOBC_A1_3V3_SYSA_BUS:
	case SCOBC_A1_3V3_SYSB_BUS:
	case SCOBC_A1_3V3_IO_BUS:
		*cv = convert_cv_bus(rawval);
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int get_obc_temp_register_addr(enum scobc_a1_temp_pos pos, uint32_t *addr)
{
	int ret = 0;

	switch (pos) {
	case SCOBC_A1_TEMP_1:
		*addr = SCOBC_A1_SYSMON_TEMP1R;
		break;
	case SCOBC_A1_TEMP_2:
		*addr = SCOBC_A1_SYSMON_TEMP2R;
		break;
	case SCOBC_A1_TEMP_3:
		*addr = SCOBC_A1_SYSMON_TEMP3R;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int get_obc_cv_register_addr(enum scobc_a1_cv_pos pos, uint32_t *addr)
{
	int ret = 0;

	switch (pos) {
	case SCOBC_A1_1V0_SHUNT:
		*addr = SCOBC_A1_SYSMON_1V0_SNTVR;
		break;
	case SCOBC_A1_1V0_BUS:
		*addr = SCOBC_A1_SYSMON_1V0_BUSVR;
		break;
	case SCOBC_A1_1V8_SHUNT:
		*addr = SCOBC_A1_SYSMON_1V8_SNTVR;
		break;
	case SCOBC_A1_1V8_BUS:
		*addr = SCOBC_A1_SYSMON_1V8_BUSVR;
		break;
	case SCOBC_A1_3V3_SHUNT:
		*addr = SCOBC_A1_SYSMON_3V3_SNTVR;
		break;
	case SCOBC_A1_3V3_BUS:
		*addr = SCOBC_A1_SYSMON_3V3_BUSVR;
		break;
	case SCOBC_A1_3V3_SYSA_SHUNT:
		*addr = SCOBC_A1_SYSMON_3V3SYSA_SNTVR;
		break;
	case SCOBC_A1_3V3_SYSA_BUS:
		*addr = SCOBC_A1_SYSMON_3V3SYSA_BUSVR;
		break;
	case SCOBC_A1_3V3_SYSB_SHUNT:
		*addr = SCOBC_A1_SYSMON_3V3SYSB_SNTVR;
		break;
	case SCOBC_A1_3V3_SYSB_BUS:
		*addr = SCOBC_A1_SYSMON_3V3SYSB_BUSVR;
		break;
	case SCOBC_A1_3V3_IO_SHUNT:
		*addr = SCOBC_A1_SYSMON_3V3IO_SNTVR;
		break;
	case SCOBC_A1_3V3_IO_BUS:
		*addr = SCOBC_A1_SYSMON_3V3IO_BUSVR;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int get_xadc_register_addr(enum scobc_a1_xadc_pos pos, uint32_t *addr)
{
	int ret = 0;

	switch (pos) {
	case SCOBC_A1_XADC_VCCINT:
		*addr = SCOBC_A1_SYSMON_XADC_VCCINT;
		break;
	case SCOBC_A1_XADC_VCCAUX:
		*addr = SCOBC_A1_SYSMON_XADC_VCCAUX;
		break;
	case SCOBC_A1_XADC_VCCBRAM:
		*addr = SCOBC_A1_SYSMON_XADC_VCCBRAM;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int wait_for_bhm_complete(uint32_t target_bit)
{
	int ret = 0;
	int i;
	uint32_t reg;

	/* Wait for any processing to be complete */
	for (i = 0; i < SYSMON_RETRY_COUNT; i++) {
		reg = sys_read32(SCOBC_A1_SYSMON_ISR);
		if ((reg & target_bit) == target_bit) {
			sys_write32(target_bit, SCOBC_A1_SYSMON_ISR);
			break;
		}
		k_sleep(K_MSEC(1));
	}

	if (i == SYSMON_RETRY_COUNT) {
		ret = -ETIMEDOUT;
	}

	return ret;
}

static int sc_bhm_init(void)
{
	sys_set_bits(SCOBC_A1_SYSMON_INICTLR, SCOBC_A1_SYSMON_INIT_ALL);

	return wait_for_bhm_complete(SCOBC_A1_SYSMON_INIT_ACCEND);
}

static void sc_bhm_set_timer(void)
{
	/*
	 * This value is using the recommended value from the FPGA technical
	 * reference manual.
	 * Monitoring inteval: 1 seconds
	 * TODO: Extend API to allow users to configure parameters.
	 */
	sys_write32(0x00280000, SCOBC_A1_GPTMR_HITCR);
	sys_write32(0x095F, SCOBC_A1_GPTMR_HITPR);
	sys_write32(0x0050, SCOBC_A1_GPTMR_HITOCR1);
	sys_write32(0x0001, SCOBC_A1_GPTMR_HITOCR2);
	sys_write32(0x0040, SCOBC_A1_GPTMR_HITOCR3);
}

static inline void sc_bhm_monitor_enable(void)
{
	sys_set_bits(SCOBC_A1_SYSMON_ACCCTLR, SCOBC_A1_SYSMON_MONIEN_ALL);
}

static inline void sc_bhm_monitor_disable(void)
{
	sys_clear_bits(SCOBC_A1_SYSMON_ACCCTLR, SCOBC_A1_SYSMON_MONIEN_ALL);
}

static inline void sc_bhm_timer_enable(void)
{
	sys_set_bits(SCOBC_A1_GPTMR_TECR, SCOBC_A1_SYSMON_GPTMR_HITEN);
}

static inline void sc_bhm_timer_disable(void)
{
	sys_clear_bits(SCOBC_A1_GPTMR_TECR, SCOBC_A1_SYSMON_GPTMR_HITEN);
}

static int sc_bhm_get_sensor_data(uint32_t addr, uint32_t *raw)
{
	int ret = 0;

	*raw = sys_read32(addr);
	if (*raw & SCOBC_A1_SYSMON_BHM_NUPD) {
		ret = -EAGAIN;
	}

	return ret;
}

void sc_kick_wdt_timer(void)
{
	uint32_t reg;

	reg = sys_read32(SCOBC_A1_SYSMON_WDOG_CTRL);
	sys_write32(reg, SCOBC_A1_SYSMON_WDOG_CTRL);
}

int sc_bhm_enable(void)
{
	int ret;

	ret = sc_bhm_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize the BHM. (%d)", ret);
		goto end;
	}

	sc_bhm_set_timer();

	sc_bhm_monitor_enable();

	sc_bhm_timer_enable();
end:
	return ret;
}

int sc_bhm_disable(void)
{
	sc_bhm_timer_disable();

	sc_bhm_monitor_disable();

	return 0;
}

int sc_bhm_get_obc_temp(enum scobc_a1_temp_pos pos, float *temp)
{
	int ret;
	uint32_t addr;
	uint32_t raw;

	ret = get_obc_temp_register_addr(pos, &addr);
	if (ret < 0) {
		goto end;
	}

	ret = sc_bhm_get_sensor_data(addr, &raw);
	if (ret < 0) {
		goto end;
	}

	*temp = convert_obc_temp(raw);

end:
	return ret;
}

int sc_bhm_get_obc_cv(enum scobc_a1_cv_pos pos, int32_t *cv)
{
	int ret;
	uint32_t addr;
	uint32_t raw;

	ret = get_obc_cv_register_addr(pos, &addr);
	if (ret < 0) {
		goto end;
	}

	ret = sc_bhm_get_sensor_data(addr, &raw);
	if (ret < 0) {
		goto end;
	}

	ret = convert_obc_cv(pos, raw, cv);
end:
	return ret;
}

int sc_bhm_get_xadc_temp(float *temp)
{
	int ret;
	uint32_t raw;

	ret = sc_bhm_get_sensor_data(SCOBC_A1_SYSMON_XADC_TEMP, &raw);
	if (ret < 0) {
		goto end;
	}

	*temp = convert_xadc_temp(raw);

end:
	return ret;
}

int sc_bhm_get_xadc_cv(enum scobc_a1_xadc_pos pos, float *cv)
{
	int ret;
	uint32_t addr;
	uint32_t raw;

	ret = get_xadc_register_addr(pos, &addr);
	if (ret < 0) {
		goto end;
	}

	ret = sc_bhm_get_sensor_data(addr, &raw);
	if (ret < 0) {
		goto end;
	}

	*cv = convert_cv_xadc(raw);
end:
	return ret;
}

uint16_t sc_sem_get_error_count(void)
{
	uint32_t val;

	val = sys_read32(SCOBC_A1_SYSMON_SEM_ECCOUNT);

	return SCOBC_A1_SYSMON_SEMCCOUNT(val);
}

static int sc_sysmon_init(void)
{
	return 0;
}

SYS_INIT(sc_sysmon_init, POST_KERNEL, CONFIG_SC_SYSMON_INIT_PRIORITY);
