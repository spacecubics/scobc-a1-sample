/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum scobc_a1_temp_pos {
	SCOBC_A1_TEMP_1 = 0,
	SCOBC_A1_TEMP_2,
	SCOBC_A1_TEMP_3,
	SCOBC_A1_TEMP_POS_NUM,
};

enum scobc_a1_cv_pos {
	SCOBC_A1_1V0_SHUNT = 0,
	SCOBC_A1_1V0_BUS,
	SCOBC_A1_1V8_SHUNT,
	SCOBC_A1_1V8_BUS,
	SCOBC_A1_3V3_SHUNT,
	SCOBC_A1_3V3_BUS,
	SCOBC_A1_3V3_SYSA_SHUNT,
	SCOBC_A1_3V3_SYSA_BUS,
	SCOBC_A1_3V3_SYSB_SHUNT,
	SCOBC_A1_3V3_SYSB_BUS,
	SCOBC_A1_3V3_IO_SHUNT,
	SCOBC_A1_3V3_IO_BUS,
	SCOBC_A1_CV_POS_NUM
};

enum scobc_a1_xadc_pos {
	SCOBC_A1_XADC_VCCINT = 0,
	SCOBC_A1_XADC_VCCAUX,
	SCOBC_A1_XADC_VCCBRAM,
	SCOBC_A1_XADC_CV_POS_NUM,
};

void sc_kick_wdt_timer(void);
int sc_bhm_enable(void);
int sc_bhm_disable(void);
int sc_bhm_get_obc_temp(enum scobc_a1_temp_pos pos, float *temp);
int sc_bhm_get_xadc_temp(float *temp);
int sc_bhm_get_obc_cv(enum scobc_a1_cv_pos pos, int32_t *cv);
int sc_bhm_get_xadc_cv(enum scobc_a1_xadc_pos pos, float *cv);
uint16_t sc_sem_get_error_count(void);
