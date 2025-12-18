/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "flash_sc_qspi_a1.h"
#include "spi_nor.h"
#include "jesd216.h"

LOG_MODULE_REGISTER(flash_sc_qspi_a1, CONFIG_FLASH_LOG_LEVEL);

/* Offsets */
#define SCQSPI_A1_ACR_OFFSET    0x0000 /* Access Control */
#define SCQSPI_A1_TDR_OFFSET    0x0004 /* TX Data */
#define SCQSPI_A1_RDR_OFFSET    0x0008 /* RX Data */
#define SCQSPI_A1_ASR_OFFSET    0x000C /* Access Status */
#define SCQSPI_A1_FIFOSR_OFFSET 0x0010 /* FIFO Status */
#define SCQSPI_A1_FIFORR_OFFSET 0x0014 /* FIFO Reset */
#define SCQSPI_A1_ISR_OFFSET    0x0020 /* Interrupt Status */
#define SCQSPI_A1_IER_OFFSET    0x0024 /* Interrupt Enable */
#define SCQSPI_A1_CCR_OFFSET    0x0030 /* Clock Control */

/* Access Control Register (ACR) Bits */
#define SCQSPI_A1_ACR_SS_OFF    0x00
#define SCQSPI_A1_ACR_SS1       BIT(0)
#define SCQSPI_A1_ACR_SS2       BIT(1)
#define SCQSPI_A1_ACR_MODE_QUAD BIT(17)

/* QSPI Interrupt Status Register */
#define SCQSPI_A1_SPICTRLDN BIT(0)

/* QSPI Interrupt Enable Register */
#define SCQSPI_A1_TXFIFOUTHEMB BIT(26)
#define SCQSPI_A1_TXFIFOOVFEMB BIT(25)
#define SCQSPI_A1_TXFIFOUDFEMB BIT(24)
#define SCQSPI_A1_RXFIFOOTHEMB BIT(18)
#define SCQSPI_A1_RXFIFOOVFEMB BIT(17)
#define SCQSPI_A1_RXFIFOUDFEMB BIT(16)
#define SCQSPI_A1_SPIBUSYDNEMB BIT(0)
#define SCQSPI_A1_IER_ALL                                                                          \
	(SCQSPI_A1_TXFIFOUTHEMB | SCQSPI_A1_TXFIFOOVFEMB | SCQSPI_A1_TXFIFOUDFEMB |                \
	 SCQSPI_A1_RXFIFOOTHEMB | SCQSPI_A1_RXFIFOOVFEMB | SCQSPI_A1_RXFIFOUDFEMB |                \
	 SCQSPI_A1_SPIBUSYDNEMB)

/* Clock Control Register (CCR) Shifts */
#define SCQSPI_A1_CCR_SCKPOL_SHIFT 20U
#define SCQSPI_A1_CCR_SCKPHA_SHIFT 16U
#define SCQSPI_A1_CCR_SCKDIV_SHIFT 0U

/* Driver Constants */
#define SCQSPI_A1_RX_FIFO_MAX_BYTE      16U
#define SCQSPI_A1_PAGE_BUFFER_BYTE      256U
/*
 * Since the current driver implementation only supports a fixed latency of
 * 8 dummy cycles, this value is hardcoded via #define. It is not exposed via
 * Device Tree (DTS) to avoid implying flexibility/configurability that is
 * not yet supported by the underlying code.
 * Dummy byte count mapping:
 * - Single SPI: 1 dummy byte  = 8 dummy clock cycles (1 bit per clock)
 * - QSPI:       4 dummy bytes = 8 dummy clock cycles (4 bits per clock)
 */
#define SCQSPI_A1_DUMMY_BYTE_COUNT      1U
#define SCQSPI_A1_QSPI_DUMMY_BYTE_COUNT 4U

/* Address Lengths */
#define SCQSPI_A1_ADDR_LEN_3B 3U
#define SCQSPI_A1_ADDR_LEN_4B 4U

/* FPGA System Controller Configuration Memory Control Register Offsets */
#define SCOBCA1_REG_CFGMEMCTL_OFFSET 0x0010

/* FPGA System Controller Configuration Memory Control Register Bits */
#define SCOBCA1_CFGMEMCTL_SEL_VAL(x) (((x) & BIT(0)) << 4)
#define SCOBCA1_CFGMEMCTL_MON_GET(x) (((x) & BIT(5)) >> 5)

/* FPGA Memory Selection Timing */
#define SC_FPGA_MEM_SEL_RETRY_CNT 100
#define SC_FPGA_MEM_SEL_DELAY_US  100

/* Access Control Register (ACR) Masks */
#define SCQSPI_A1_ACR_SPISSCTL_MASK GENMASK(1, 0)

/* Read Any Register Commands */
#define SCQSPI_A1_RDAR_CMD 0x65
/* Write Any Register Commands */
#define SCQSPI_A1_WRAR_CMD 0x71

/*
 * ===========================================================================
 * QSPI Controller Driver - sc,qspi-a1
 * ===========================================================================
 */
#define DT_DRV_COMPAT sc_qspi_a1

typedef void (*irq_config_func_t)(const struct device *dev);

struct sc_qspi_a1_ctrl_config {
	uint32_t base_address;
	irq_config_func_t irq_config;

	uint8_t cpol;
	uint8_t cpha;
	uint16_t spiclk_div;
};

struct sc_qspi_a1_ctrl_data {
	struct k_mutex bus_lock;
	struct k_sem isr_sem;
};

static void sc_qspi_a1_ctrl_isr(const struct device *dev)
{
	const struct sc_qspi_a1_ctrl_config *cfg = dev->config;
	struct sc_qspi_a1_ctrl_data *data = dev->data;
	uint32_t isr;

	isr = sys_read32(cfg->base_address + SCQSPI_A1_ISR_OFFSET);
	sys_write32(isr, cfg->base_address + SCQSPI_A1_ISR_OFFSET);

	if (isr & SCQSPI_A1_SPICTRLDN) {
		k_sem_give(&data->isr_sem);
	}
}

static void sc_qspi_a1_ctrl_clock_configure(const struct sc_qspi_a1_ctrl_config *cfg)
{
	uint32_t clock;

	clock = cfg->cpol << SCQSPI_A1_CCR_SCKPOL_SHIFT | cfg->cpha << SCQSPI_A1_CCR_SCKPHA_SHIFT |
		cfg->spiclk_div << SCQSPI_A1_CCR_SCKDIV_SHIFT;

	sys_write32(clock, cfg->base_address + SCQSPI_A1_CCR_OFFSET);
}

static void sc_qspi_a1_ctrl_enable_irq(const struct device *dev)
{
	const struct sc_qspi_a1_ctrl_config *cfg = dev->config;

	sys_set_bits(cfg->base_address + SCQSPI_A1_IER_OFFSET, SCQSPI_A1_IER_ALL);
	cfg->irq_config(dev);
}

static int sc_qspi_a1_ctrl_init(const struct device *dev)
{
	const struct sc_qspi_a1_ctrl_config *cfg = dev->config;
	struct sc_qspi_a1_ctrl_data *data = dev->data;

	k_mutex_init(&data->bus_lock);
	k_sem_init(&data->isr_sem, 0, 1);

	sc_qspi_a1_ctrl_clock_configure(cfg);

	sc_qspi_a1_ctrl_enable_irq(dev);

	return 0;
}

#define SC_QSPI_A1_CTRL_INIT(inst)                                                                 \
	static void sc_qspi_a1_ctrl_irq_config_##inst(const struct device *dev)                    \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), sc_qspi_a1_ctrl_isr,  \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
                                                                                                   \
	static const struct sc_qspi_a1_ctrl_config ctrl_cfg_##inst = {                             \
		.base_address = DT_INST_REG_ADDR(inst),                                            \
		.irq_config = sc_qspi_a1_ctrl_irq_config_##inst,                                   \
		.cpol = DT_INST_PROP(inst, cpol),                                                  \
		.cpha = DT_INST_PROP(inst, cpha),                                                  \
		.spiclk_div = DT_INST_PROP(inst, spiclk_div),                                      \
	};                                                                                         \
                                                                                                   \
	static struct sc_qspi_a1_ctrl_data ctrl_data_##inst;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &sc_qspi_a1_ctrl_init, NULL, &ctrl_data_##inst,                \
			      &ctrl_cfg_##inst, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,   \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(SC_QSPI_A1_CTRL_INIT)

/*
 * ===========================================================================
 * QSPI Flash NOR Driver - sc,qspi-nor
 * ===========================================================================
 */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT sc_qspi_nor_a1

/* Build-time data associated with the device. */
struct sc_qspi_nor_a1_cfg {
	const struct device *parent;

	/* Size of device in bytes, from size property */
	uint32_t flash_size;

	/* Memory selected number */
	uint8_t mem_no;

	/* Expected JEDEC ID, from jedec-id property */
	uint8_t jedec_id[SPI_NOR_MAX_ID_LEN];

	/* Base address of FPGA system controller, if used */
	uint32_t sysreg_base_addr;
};

struct sc_qspi_nor_a1_data {
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	struct flash_pages_layout layout;
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
};

static const struct flash_parameters flash_nor_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

static int sc_qspi_nor_a1_select_cfg_mem(const struct device *dev)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	uint32_t reg_addr = cfg->sysreg_base_addr + SCOBCA1_REG_CFGMEMCTL_OFFSET;
	uint32_t reg_val;
	int retry = SC_FPGA_MEM_SEL_RETRY_CNT;

	sys_write32(SCOBCA1_CFGMEMCTL_SEL_VAL(cfg->mem_no), reg_addr);

	/* Check we actually change the selected memory */
	while (retry--) {
		reg_val = sys_read32(reg_addr);
		if (SCOBCA1_CFGMEMCTL_MON_GET(reg_val) == cfg->mem_no) {
			return 0;
		}
		k_busy_wait(SC_FPGA_MEM_SEL_DELAY_US);
	}

	LOG_ERR("Failed to select Config Memory %d via fpga system register", cfg->mem_no);
	return -ETIMEDOUT;
}

static void acquire_device(const struct device *dev)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	struct sc_qspi_a1_ctrl_data *parent_data = cfg->parent->data;

	k_mutex_lock(&parent_data->bus_lock, K_FOREVER);
}

static void release_device(const struct device *dev)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	struct sc_qspi_a1_ctrl_data *parent_data = cfg->parent->data;

	k_mutex_unlock(&parent_data->bus_lock);
}

static uint32_t get_base_addr(const struct device *dev)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	const struct sc_qspi_a1_ctrl_config *parent_cfg = cfg->parent->config;

	return parent_cfg->base_address;
}

static int sc_qspi_a1_wait_for_idle(const struct device *dev)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	struct sc_qspi_a1_ctrl_data *parent_data = cfg->parent->data;
	int ret;

	/* Waiting for the 'SPICTRLDN' interrupt from the FPGA. */
	ret = k_sem_take(&parent_data->isr_sem, K_MSEC(CONFIG_SC_QSPI_NOR_A1_IDLE_TIMEOUT_MS));
	if (ret < 0) {
		LOG_ERR("Timeout waiting for idle interrupt (%d)", ret);
	}

	return ret;
}

static int sc_qspi_a1_cs_control(const struct device *dev, bool quad_mode)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	const uint32_t acr_reg_addr = get_base_addr(dev) + SCQSPI_A1_ACR_OFFSET;
	uint32_t target_acr;
	uint32_t current_acr;

	if (cfg->sysreg_base_addr != 0) {
		int ret = sc_qspi_nor_a1_select_cfg_mem(dev);
		if (ret < 0) {
			return ret;
		}
		/* With the flash configuration memory we always use SCQSPI_A1_ACR_SS1 because we
		 * change selected memory from the fpga system register */
		target_acr = SCQSPI_A1_ACR_SS1;
	} else {
		target_acr = BIT(cfg->mem_no);
	}

	current_acr = sys_read32(acr_reg_addr);

	if (quad_mode) {
		target_acr |= SCQSPI_A1_ACR_MODE_QUAD;
	}
	sys_write32(target_acr, acr_reg_addr);

	/* Check if Chip Select control bits have changed, if the CS signal hasn't physically
	 * changed, we don't need to wait for idl. */
	if ((current_acr & SCQSPI_A1_ACR_SPISSCTL_MASK) ==
	    (target_acr & SCQSPI_A1_ACR_SPISSCTL_MASK)) {
		return 0;
	}

	/* The CS has changed (e.g. from Negated to Asserted), so we must wait for the controller to
	 * become idle. */
	return sc_qspi_a1_wait_for_idle(dev);
}

static int sc_qspi_a1_cs_activate(const struct device *dev)
{
	return sc_qspi_a1_cs_control(dev, false);
}

static int sc_qspi_a1_cs_activate_quad(const struct device *dev)
{
	return sc_qspi_a1_cs_control(dev, true);
}

static int sc_qspi_a1_cs_inactivate(const struct device *dev)
{
	sys_write32(SCQSPI_A1_ACR_SS_OFF, get_base_addr(dev) + SCQSPI_A1_ACR_OFFSET);

	return sc_qspi_a1_wait_for_idle(dev);
}

static int sc_qspi_nor_a1_write_mem_addr(const struct device *dev, uint32_t mem_addr, uint8_t len)
{
	const uint32_t addr_tdr = get_base_addr(dev) + SCQSPI_A1_TDR_OFFSET;
	int ret;
	uint8_t byte;

	for (int i = len - 1; i >= 0; --i) {
		byte = (mem_addr >> (i * 8)) & 0xFF;
		sys_write32(byte, addr_tdr);

		ret = sc_qspi_a1_wait_for_idle(dev);
		if (ret < 0) {
			LOG_ERR("Failed to write addr byte %d (0x%08x)", i, mem_addr);
			return ret;
		}
	}

	return ret;
}

static int sc_qspi_nor_a1_send_cmd(const struct device *dev, uint8_t cmd)
{
	const uint32_t addr_tdr = get_base_addr(dev) + SCQSPI_A1_TDR_OFFSET;
	int ret;
	int ret_inact;

	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		return ret;
	}

	sys_write32(cmd, addr_tdr);

	ret = sc_qspi_a1_wait_for_idle(dev);

	ret_inact = sc_qspi_a1_cs_inactivate(dev);

	return (ret < 0) ? ret : ret_inact;
}

static int sc_qspi_nor_a1_write_disable(const struct device *dev)
{
	return sc_qspi_nor_a1_send_cmd(dev, SPI_NOR_CMD_WRDI);
}

static int sc_qspi_nor_a1_write_enable(const struct device *dev)
{
	return sc_qspi_nor_a1_send_cmd(dev, SPI_NOR_CMD_WREN);
}

static int sc_qspi_nor_a1_request_rx_data(const struct device *dev, size_t size)
{
	const uint32_t base = get_base_addr(dev);
	const uint32_t addr_rdr = base + SCQSPI_A1_RDR_OFFSET;

	for (int i = 0; i < size; i++) {
		sys_write32(0x00, addr_rdr);
	}

	return sc_qspi_a1_wait_for_idle(dev);
}

/*
 * Read RX data from QSPI NOR flash into buffer.
 * If 'buf' is NULL, data is read and discarded. (Dummy Cycle)
 */
static int sc_qspi_nor_a1_read_rx_data(const struct device *dev, uint8_t *buf, size_t size)
{
	const uint32_t base = get_base_addr(dev);
	const uint32_t addr_rdr = base + SCQSPI_A1_RDR_OFFSET;
	int ret = 0;
	size_t bytes = 0;

	while (bytes < size) {
		size_t chunk = MIN(SCQSPI_A1_RX_FIFO_MAX_BYTE, size - bytes);

		ret = sc_qspi_nor_a1_request_rx_data(dev, chunk);
		if (ret < 0) {
			LOG_ERR("Timeout while requesting RX data (%d)", ret);
			return ret;
		}

		for (size_t i = 0; i < chunk; i++) {
			if (buf) {
				uint32_t reg_val = sys_read32(addr_rdr);
				buf[bytes + i] = (uint8_t)reg_val;
			} else {
				(void)sys_read32(addr_rdr);
			}
		}

		bytes += chunk;
	}

	return ret;
}

static int sc_qspi_nor_a1_send_dummy_cycle(const struct device *dev, uint8_t count)
{
	int ret;

	ret = sc_qspi_nor_a1_read_rx_data(dev, NULL, count);
	if (ret < 0) {
		LOG_ERR("Timeout while sending dummy cycles (%d)", ret);
		return ret;
	}

	return ret;
}

static int sc_qspi_nor_a1_read_status(const struct device *dev, uint8_t *status_reg)
{
	const uint32_t addr_tdr = get_base_addr(dev) + SCQSPI_A1_TDR_OFFSET;
	int ret;

	if (!status_reg) {
		return -EINVAL;
	}

	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		return ret;
	}

	sys_write32(SPI_NOR_CMD_RDSR, addr_tdr);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_read_rx_data(dev, status_reg, 1);
	if (ret < 0) {
		goto out_cs;
	}

out_cs:
	int ret_inact = sc_qspi_a1_cs_inactivate(dev);

	return (ret < 0) ? ret : ret_inact;
}

static int sc_qspi_nor_a1_wait_ready(const struct device *dev, uint32_t poll_delay_us,
				     uint32_t timeout_us)
{
	uint8_t status_reg;
	int ret;
	uint32_t max_loops = timeout_us / poll_delay_us;

	/*
	 * The 'max_loops' calculation differs significantly between operations:
	 * - Page Program (Write): Very fast (~300us typical (S25FL256L)). Short sleep, few loops.
	 * - Block Erase: Slow (~50ms to 725ms typical (S25FL256L)). Longer sleep to save CPU, many
	 * loops.
	 */
	for (uint32_t i = 0; i < max_loops; i++) {
		k_usleep(poll_delay_us);
		status_reg = 0;
		ret = sc_qspi_nor_a1_read_status(dev, &status_reg);
		if (ret < 0) {
			return ret;
		}

		if (!(status_reg & SPI_NOR_WIP_BIT) && !(status_reg & SPI_NOR_WEL_BIT)) {
			return 0;
		}
	}

	/* Timeout diagnostics */
	if (status_reg & SPI_NOR_WIP_BIT) {
		LOG_ERR("Timeout waiting for Write In Progress (WIP) to clear (SR=0x%02x)",
			(uint8_t)status_reg);
	}

	if (status_reg & SPI_NOR_WEL_BIT) {
		LOG_ERR("Timeout waiting for Write Enable Latch (WEL) to clear (SR=0x%02x)",
			(uint8_t)status_reg);
		/* Try to recover by explicitly clearing WEL */
		(void)sc_qspi_nor_a1_write_disable(dev);
	}

	return -ETIMEDOUT;
}

/*
 * Write to any internal register using WRAR (0x71).
 * Requires WREN before sending the command.
 */
int sc_qspi_nor_a1_write_any_reg(const struct device *dev, uint32_t reg_addr, uint8_t val)
{
	const uint32_t addr_tdr = get_base_addr(dev) + SCQSPI_A1_TDR_OFFSET;
	int ret;
	uint32_t timeout = CONFIG_SC_QSPI_NOR_A1_WRITE_TIMEOUT_US;
	uint32_t poll_delay = CONFIG_SC_QSPI_NOR_A1_SLEEP_WRITE_US;

	/*
	 * Si l'adresse est < 0x800000, c'est un registre Non-Volatile (NV).
	 * L'écriture prend beaucoup plus de temps (similaire à un Erase ou Write long).
	 * On utilise le timeout d'Erase pour être sûr (~100ms+).
	 */
	if (reg_addr < 0x800000) {
		/* MS to US */
		timeout = CONFIG_SC_QSPI_NOR_A1_ERASE_TIMEOUT_MS * 1000U;
		poll_delay = CONFIG_SC_QSPI_NOR_A1_SLEEP_ERASE_MS * 1000U;
	}

	acquire_device(dev);

	ret = sc_qspi_nor_a1_write_enable(dev);
	if (ret < 0) {
		release_device(dev);
		return ret;
	}

	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		release_device(dev);
		return ret;
	}

	sys_write32(SCQSPI_A1_WRAR_CMD, addr_tdr);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_write_mem_addr(dev, reg_addr, SCQSPI_A1_ADDR_LEN_3B);
	if (ret < 0) {
		goto out_cs;
	}

	sys_write32(val, addr_tdr);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

out_cs:
	int ret_inact = sc_qspi_a1_cs_inactivate(dev);

	if (ret < 0) {
		return ret;
	}

	if (ret_inact < 0) {
		return ret_inact;
	}

	ret = sc_qspi_nor_a1_wait_ready(dev, CONFIG_SC_QSPI_NOR_A1_SLEEP_WRITE_US,
					CONFIG_SC_QSPI_NOR_A1_WRITE_TIMEOUT_US);

	release_device(dev);

	return (ret < 0) ? ret : ret_inact;
}

int sc_qspi_nor_a1_read_any_reg(const struct device *dev, uint32_t reg_addr, uint8_t *val)
{
	const uint32_t addr_tdr = get_base_addr(dev) + SCQSPI_A1_TDR_OFFSET;
	int ret;

	if (!val) {
		return -EINVAL;
	}

	acquire_device(dev);

	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		release_device(dev);
		return ret;
	}

	sys_write32(SCQSPI_A1_RDAR_CMD, addr_tdr);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_write_mem_addr(dev, reg_addr, SCQSPI_A1_ADDR_LEN_3B);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_send_dummy_cycle(dev, SCQSPI_A1_DUMMY_BYTE_COUNT);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_read_rx_data(dev, val, 1);
	if (ret < 0) {
		goto out_cs;
	}

out_cs:
	int ret_inact = sc_qspi_a1_cs_inactivate(dev);
	release_device(dev);

	return (ret < 0) ? ret : ret_inact;
}

static int sc_qspi_nor_a1_quad_read_internal(const struct device *dev, off_t addr, uint8_t *buf,
					     size_t size)
{
	uint32_t mem_addr = (uint32_t)addr;
	const uint32_t base = get_base_addr(dev);
	const uint32_t addr_tdr = base + SCQSPI_A1_TDR_OFFSET;
	int ret;

	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		return ret;
	}

	sys_write32(SPI_NOR_CMD_4READ_4B, addr_tdr);

	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		LOG_ERR("Failed to send 4READ_4B opcode (%d)", ret);
		goto out_cs;
	}

	ret = sc_qspi_a1_cs_activate_quad(dev);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_write_mem_addr(dev, mem_addr, SCQSPI_A1_ADDR_LEN_4B);
	if (ret < 0) {
		goto out_cs;
	}

	sys_write32(0x00, addr_tdr);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		LOG_ERR("Failed to send mode byte (%d)", ret);
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_send_dummy_cycle(dev, SCQSPI_A1_QSPI_DUMMY_BYTE_COUNT);
	if (ret < 0) {
		LOG_ERR("Failed to send dummy cycles (%d)", ret);
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_read_rx_data(dev, buf, size);
	if (ret < 0) {
		LOG_ERR("Failed to read RX data (%d)", ret);
		goto out_cs;
	}

out_cs:
	int ret_inact = sc_qspi_a1_cs_inactivate(dev);

	return (ret < 0) ? ret : ret_inact;
}

static int sc_qspi_nor_a1_read(const struct device *dev, off_t addr, void *dest, size_t size)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	int ret;

	if (!dest) {
		return -EINVAL;
	}

	if (addr < 0 || (addr + size) > cfg->flash_size) {
		return -EINVAL;
	}

	acquire_device(dev);
	ret = sc_qspi_nor_a1_quad_read_internal(dev, addr, (uint8_t *)dest, size);
	release_device(dev);

	return ret;
}

static int sc_qspi_nor_a1_page_program_internal(const struct device *dev, uint32_t addr,
						const uint8_t *src, size_t len)
{
	const uint32_t addr_tdr = get_base_addr(dev) + SCQSPI_A1_TDR_OFFSET;
	int ret;

	if (!src || len == 0 || len > SCQSPI_A1_PAGE_BUFFER_BYTE) {
		return -EINVAL;
	}

	ret = sc_qspi_nor_a1_write_enable(dev);
	if (ret < 0) {
		return ret;
	}

	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		return ret;
	}

	sys_write32(SPI_NOR_CMD_PP_4B, addr_tdr);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_write_mem_addr(dev, addr, SCQSPI_A1_ADDR_LEN_4B);
	if (ret < 0) {
		goto out_cs;
	}

	for (size_t i = 0; i < len; i++) {
		sys_write32(src[i], addr_tdr);
	}

	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

out_cs:
	int ret_inact = sc_qspi_a1_cs_inactivate(dev);

	if (ret < 0) {
		return ret;
	}

	if (ret_inact < 0) {
		return ret_inact;
	}

	return sc_qspi_nor_a1_wait_ready(dev, CONFIG_SC_QSPI_NOR_A1_SLEEP_WRITE_US,
					 CONFIG_SC_QSPI_NOR_A1_WRITE_TIMEOUT_US);
}

static int sc_qspi_nor_a1_write(const struct device *dev, off_t addr, const void *src, size_t size)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	const uint8_t *src_ptr = src;
	uint32_t curr_addr = (uint32_t)addr;
	int ret = 0;

	if (!src) {
		return -EINVAL;
	}

	if ((addr < 0) || ((addr + size) > cfg->flash_size)) {
		return -EINVAL;
	}

	acquire_device(dev);

	while (size > 0) {
		size_t page_off = curr_addr % SCQSPI_A1_PAGE_BUFFER_BYTE;
		size_t space_in_page = SCQSPI_A1_PAGE_BUFFER_BYTE - page_off;
		size_t chunk = (size < space_in_page) ? size : space_in_page;

		ret = sc_qspi_nor_a1_page_program_internal(dev, curr_addr, src_ptr, chunk);
		if (ret < 0) {
			break;
		}

		curr_addr += chunk;
		src_ptr += chunk;
		size -= chunk;
	}

	release_device(dev);
	return ret;
}

static int sc_qspi_nor_a1_block_erase_internal(const struct device *dev, uint8_t cmd, uint32_t addr)
{
	int ret;

	ret = sc_qspi_nor_a1_write_enable(dev);
	if (ret < 0) {
		return ret;
	}

	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		return ret;
	}

	sys_write32(cmd, get_base_addr(dev) + SCQSPI_A1_TDR_OFFSET);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_write_mem_addr(dev, addr, SCQSPI_A1_ADDR_LEN_4B);
	if (ret < 0) {
		goto out_cs;
	}

out_cs:
	int ret_inact = sc_qspi_a1_cs_inactivate(dev);

	if (ret < 0) {
		return ret;
	}

	if (ret_inact < 0) {
		return ret_inact;
	}

	return sc_qspi_nor_a1_wait_ready(dev, (CONFIG_SC_QSPI_NOR_A1_SLEEP_ERASE_MS * 1000U),
					 (CONFIG_SC_QSPI_NOR_A1_ERASE_TIMEOUT_MS * 1000U));
}

static int sc_qspi_nor_a1_erase(const struct device *dev, off_t addr, size_t size)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	uint32_t curr_addr = (uint32_t)addr;
	int ret = 0;

	if ((addr < 0) || ((addr + size) > cfg->flash_size)) {
		return -EINVAL;
	}

	if (!SPI_NOR_IS_SECTOR_ALIGNED(curr_addr) || !SPI_NOR_IS_SECTOR_ALIGNED(size)) {
		return -EINVAL;
	}

	acquire_device(dev);

	while (size > 0) {
		uint8_t cmd;
		size_t chunk;

		/* Smart Erase: Prefer 64KB Block Erase if aligned and big enough */
		if ((curr_addr % SPI_NOR_BLOCK_SIZE == 0) && (size >= SPI_NOR_BLOCK_SIZE)) {
			cmd = SPI_NOR_CMD_BE_4B;
			chunk = SPI_NOR_BLOCK_SIZE;
			LOG_DBG("Block Erase (64KB) @ 0x%08x", curr_addr);
		} else {
			cmd = SPI_NOR_CMD_SE_4B;
			chunk = SPI_NOR_SECTOR_SIZE;
			LOG_DBG("Sector Erase (4KB) @ 0x%08x", curr_addr);
		}

		ret = sc_qspi_nor_a1_block_erase_internal(dev, cmd, curr_addr);
		if (ret < 0) {
			break;
		}

		curr_addr += chunk;
		size -= chunk;
	}

	release_device(dev);
	return ret;
}

/*
 * Read Serial Flash Discovery Parameter
 */
static int sc_qspi_nor_a1_read_sfdp(const struct device *dev, off_t addr, void *data, size_t size)
{
	const uint32_t base = get_base_addr(dev);
	const uint32_t addr_tdr = base + SCQSPI_A1_TDR_OFFSET;
	uint8_t *buf = (uint8_t *)data;
	int ret;

	if (!data) {
		return -EINVAL;
	}

	acquire_device(dev);
	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		release_device(dev);
		return ret;
	}

	sys_write32(JESD216_CMD_READ_SFDP, addr_tdr);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_write_mem_addr(dev, addr, SCQSPI_A1_ADDR_LEN_3B);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_send_dummy_cycle(dev, SCQSPI_A1_DUMMY_BYTE_COUNT);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_read_rx_data(dev, buf, size);
	if (ret < 0) {
		LOG_ERR("Failed to read RX data (%d)", ret);
		goto out_cs;
	}

out_cs:
	int ret_inact = sc_qspi_a1_cs_inactivate(dev);

	release_device(dev);
	return (ret < 0) ? ret : ret_inact;
}

static int sc_qspi_nor_a1_read_jedec_id(const struct device *dev, uint8_t *id_buf)
{
	const uint32_t base = get_base_addr(dev);
	const uint32_t addr_tdr = base + SCQSPI_A1_TDR_OFFSET;
	int ret;

	if (!id_buf) {
		return -EINVAL;
	}

	acquire_device(dev);
	ret = sc_qspi_a1_cs_activate(dev);
	if (ret < 0) {
		release_device(dev);
		return ret;
	}

	sys_write32(SPI_NOR_CMD_RDID, addr_tdr);
	ret = sc_qspi_a1_wait_for_idle(dev);
	if (ret < 0) {
		goto out_cs;
	}

	ret = sc_qspi_nor_a1_read_rx_data(dev, id_buf, SPI_NOR_MAX_ID_LEN);
	if (ret < 0) {
		LOG_ERR("Failed to read RX data (%d)", ret);
		goto out_cs;
	}

out_cs:
	int ret_inact = sc_qspi_a1_cs_inactivate(dev);

	release_device(dev);
	return (ret < 0) ? ret : ret_inact;
}

static int sc_qspi_nor_a1_init(const struct device *dev)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;
	struct sc_qspi_nor_a1_data *data = dev->data;
	int ret;

	/* Check JEDEC ID  */
	uint8_t jedec_id_read[SPI_NOR_MAX_ID_LEN];

	ret = sc_qspi_nor_a1_read_jedec_id(dev, jedec_id_read);
	if (ret < 0) {
		LOG_ERR("Failed to read JEDEC ID (%d)", ret);
		return ret;
	}

	if (memcmp(jedec_id_read, cfg->jedec_id, SPI_NOR_MAX_ID_LEN) != 0) {
		LOG_ERR("JEDEC ID mismatch: read %02x %02x %02x, expected %02x %02x %02x",
			jedec_id_read[0], jedec_id_read[1], jedec_id_read[2], cfg->jedec_id[0],
			cfg->jedec_id[1], cfg->jedec_id[2]);
		return -ENODEV;
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	data->layout.pages_count = cfg->flash_size / SPI_NOR_SECTOR_SIZE;
	data->layout.pages_size = SPI_NOR_SECTOR_SIZE;
#endif

	return ret;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

static void sc_qspi_nor_a1_pages_layout(const struct device *dev,
					const struct flash_pages_layout **layout,
					size_t *layout_size)
{
	const struct sc_qspi_nor_a1_data *data = dev->data;

	*layout = &data->layout;

	*layout_size = 1;
}

#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *sc_qspi_nor_a1_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_nor_parameters;
}

static int sc_qspi_nor_a1_get_size(const struct device *dev, uint64_t *size)
{
	const struct sc_qspi_nor_a1_cfg *cfg = dev->config;

	*size = (uint64_t)cfg->flash_size;

	return 0;
}

static DEVICE_API(flash, sc_qspi_nor_a1_api) = {
	.read = sc_qspi_nor_a1_read,
	.write = sc_qspi_nor_a1_write,
	.erase = sc_qspi_nor_a1_erase,
	.get_parameters = sc_qspi_nor_a1_get_parameters,
	.get_size = sc_qspi_nor_a1_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = sc_qspi_nor_a1_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = sc_qspi_nor_a1_read_sfdp,
	.read_jedec_id = sc_qspi_nor_a1_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
};

#define SC_QSPI_NOR_A1_INST(inst)                                                                  \
	static const struct sc_qspi_nor_a1_cfg sc_qspi_nor_a1_cfg_##inst = {                       \
		.mem_no = DT_INST_PROP(inst, mem_no),                                              \
		.flash_size = DT_INST_PROP(inst, size),                                            \
		.jedec_id = DT_INST_PROP(inst, jedec_id),                                          \
		.parent = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                     \
		.sysreg_base_addr = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, sc_sysreg),         \
                                  (DT_REG_ADDR(DT_INST_PHANDLE(inst, sc_sysreg))), \
                                  (0)),                       \
	};                                                                                         \
                                                                                                   \
	static struct sc_qspi_nor_a1_data sc_qspi_nor_a1_data_##inst;                              \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, sc_qspi_nor_a1_init, NULL, &sc_qspi_nor_a1_data_##inst,        \
			      &sc_qspi_nor_a1_cfg_##inst, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, \
			      &sc_qspi_nor_a1_api);

DT_INST_FOREACH_STATUS_OKAY(SC_QSPI_NOR_A1_INST)
