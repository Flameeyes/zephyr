/*
 * Copyright (c) 2023, Meta Platforms, Inc. and its affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mfd/upd350.h>

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mfd_upd350);

#define UPD350_RESET_TIME_US 800

#define FASTREAD_CMD 0x0B
#define WRITE_CMD    0x02

#define FASTREAD_DIR_INC    0x00
#define FASTREAD_DIR_DEC    0x80
#define FASTREAD_DIR_STATIC 0xC0

#define SPI_TEST_REG 0x0E

#if CONFIG_MFD_UPD350_SPI
static int upd350_spi_read(const struct device *dev, uint16_t reg, void *buf, size_t len)
{
	const struct mfd_upd350_config *config = dev->config;
	int ret;

	uint8_t cmd[4];
	cmd[0] = FASTREAD_CMD;
	cmd[1] = ((reg >> 8) & 0x3F) | FASTREAD_DIR_INC;
	cmd[2] = reg & 0xFF;

	struct spi_buf buffers[] = {{.buf = cmd, .len = 4}, {.buf = buf, .len = len}};

	const struct spi_buf_set tx = {.buffers = buffers, .count = 1};
	const struct spi_buf_set rx = {.buffers = buffers, .count = 2};

	ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("spi_transceive FAIL %d", ret);
		return ret;
	}

	return 0;
}

static int upd350_spi_write(const struct device *dev, uint16_t reg, void *buf, size_t len)
{
	return 0;
}

static int upd350_spi_bus_is_ready(const struct device *dev)
{
	const struct mfd_upd350_config *config = dev->config;

	if (!spi_is_ready_dt(&config->bus.spi)) {
		LOG_ERR("SPI bus %s not ready", config->bus.spi.bus->name);
		return -ENODEV;
	}

	return 0;
}

static int upd350_spi_wakeup(const struct device *dev)
{
	uint8_t spi_test = 0xFF;

	/* Wait for the SPI communication to be available, but try only 4 times. */
	for (int i = 0; i < 4; i++) {
		// This can only be done over SPI.
		upd350_spi_read(dev, SPI_TEST_REG, &spi_test, 1);
		if (spi_test == 0x02) {
			return 0;
		}
		k_usleep(2000);
	}

	LOG_ERR("UPD350 device didn't respond to initialization.");
	return -ENODEV;
}
#endif

static int upd350_init(const struct device *dev)
{
	const struct mfd_upd350_config *config = dev->config;
	struct mfd_upd350_drv_data *drv_data = dev->data;
	int err;

	err = config->bus_fn(dev);
	if (err < 0) {
		return err;
	}

	/* If the RESET line is available, pulse it. */
	if (config->gpio_reset.port) {
		err = gpio_pin_configure_dt(&config->gpio_reset, GPIO_OUTPUT_ACTIVE);
		if (err != 0) {
			LOG_ERR("Failed to configure RESET line: %d", err);
			return -EIO;
		}

		err = gpio_pin_set_dt(&config->gpio_reset, 1);
		if (err != 0) {
			LOG_ERR("Failed to activate RESET line: %d", err);
			return -EIO;
		}

		k_usleep(UPD350_RESET_TIME_US);

		err = gpio_pin_set_dt(&config->gpio_reset, 0);
		if (err != 0) {
			LOG_ERR("Failed to deactivate RESET line: %d", err);
			return -EIO;
		}
	}

	err = config->wakeup_fn(dev);
	if (err < 0) {
		return err;
	}

	drv_data->dev = dev;

	return 0;
}

int upd350_identify(const struct device *dev)
{
	const struct mfd_upd350_config *config = dev->config;

	int err;
	struct {
		struct {
			uint16_t rev;
			uint16_t id;
		} id_rev;
		uint16_t vid;
		uint16_t pid;
		uint16_t pd_rev;
		uint16_t c_rev;
	} chip_id;

	// The chip identification registers are all adjacent, and both
	// SPI and I2C transports allow multiple register reads.
	err = config->read_fn(dev, UPD350_ID_REV_REG, &chip_id, sizeof(chip_id));
	if (err < 0) {
		LOG_ERR("unable to fetch chip identification: %d", err);
		return err;
	}

	printf("UPD350-compatible chip identified.\n"
	       "  Chip ID: %04x Rev: %04x\n"
	       "  USB ID %04x:%04x\n"
	       "  PD_REV: %04x C_REV: %04x\n",
	       chip_id.id_rev.id, chip_id.id_rev.rev, chip_id.vid, chip_id.pid, chip_id.pd_rev,
	       chip_id.c_rev);

	return 0;
}

#if CONFIG_MFD_UPD350_SPI
#define DT_DRV_COMPAT microchip_upd350

#define MFD_UPD350_SPI_DEVICE(inst)                                                                \
	static struct mfd_upd350_drv_data upd350_##inst##_drvdata = {};                            \
	static struct mfd_upd350_config upd350_##inst##_config = {                                 \
		.bus = {.spi = SPI_DT_SPEC_INST_GET(inst,                                          \
						    SPI_OP_MODE_MASTER | SPI_MODE_CPOL |           \
							    SPI_MODE_CPHA | SPI_WORD_SET(8),       \
						    0)},                                           \
		.gpio_reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
		.read_fn = upd350_spi_read,                                                        \
		.write_fn = upd350_spi_write,                                                      \
		.bus_fn = upd350_spi_bus_is_ready,                                                 \
		.wakeup_fn = upd350_spi_wakeup};                                                   \
	DEVICE_DT_INST_DEFINE(inst, upd350_init, NULL, &upd350_##inst##_drvdata,                   \
			      &upd350_##inst##_config, POST_KERNEL,                                \
			      CONFIG_MFD_UPD350_SPI_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MFD_UPD350_SPI_DEVICE)
#endif
