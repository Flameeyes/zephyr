/*
 * Copyright (c) 2023, Meta Platforms, Inc. and its affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MFD_UPD350_H_
#define ZEPHYR_DRIVERS_MFD_UPD350_H_

#include <zephyr/drivers/spi.h>

#define UPD350_ID_REV_REG 0x00
#define UPD350_VID_REG    0x04
#define UPD350_PID_REG    0x06
#define UPD350_PD_REV_REG 0x08
#define UPD350_C_REV_REG  0x0A

typedef int (*upd350_read_reg)(const struct device *dev, uint16_t reg, void *buf, size_t len);
typedef int (*upd350_write_reg)(const struct device *dev, uint16_t reg, void *buf, size_t len);
typedef int (*upd350_bus_is_ready)(const struct device *dev);
typedef int (*upd350_wakeup)(const struct device *dev);

struct mfd_upd350_config {
	union {
		// UPD350A/C use i2c.
		struct spi_dt_spec spi;
	} bus;

	struct gpio_dt_spec gpio_reset;

	upd350_read_reg read_fn;
	upd350_write_reg write_fn;
	upd350_bus_is_ready bus_fn;
	upd350_wakeup wakeup_fn;
};

struct mfd_upd350_drv_data {
	const struct device *dev;

	struct {
	} reg_cache;
};

int upd350_identify(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_MFD_UPD350_H_ */
