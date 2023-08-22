/*
 * Copyright 2022 The Chromium OS Authors
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB-C VBUS device APIs
 *
 * This file contains the USB-C VBUS device APIs.
 * All USB-C VBUS measurment and control device drivers should
 * implement the APIs described in this file.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USBC_VBUS_H_
#define ZEPHYR_INCLUDE_DRIVERS_USBC_VBUS_H_

/**
 * @brief USB-C VBUS API
 * @defgroup usbc_vbus_api USB-C VBUS API
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb_c/usbc_tc.h>

#ifdef __cplusplus
extern "C" {
#endif

struct usbc_vbus_driver_api {
	bool (*check_level)(const struct device *dev, enum tc_vbus_level level);
	int (*measure)(const struct device *dev, int *vbus_meas);
	int (*discharge)(const struct device *dev, bool enable);
	int (*enable)(const struct device *dev, bool enable);
};

/**
 * @brief Checks if VBUS is at a particular level
 *
 * @param dev    Runtime device structure
 * @param level  The level voltage to check against
 *
 * @retval true if VBUS is at the level voltage
 * @retval false if VBUS is not at that level voltage
 */
static inline bool usbc_vbus_check_level(const struct device *dev, enum tc_vbus_level level)
{
	const struct usbc_vbus_driver_api *api = (const struct usbc_vbus_driver_api *)dev->api;

	return api->check_level(dev, level);
}

/**
 * @brief Reads and returns VBUS measured in mV
 *
 * @param dev        Runtime device structure
 * @param meas       pointer where the measured VBUS voltage is stored
 *
 * @retval 0 on success
 * @retval -EIO on failure
 * @retval -ENOTSUP if the driver doesn't implement the request
 */
static inline int usbc_vbus_measure(const struct device *dev, int *meas)
{
	const struct usbc_vbus_driver_api *api = (const struct usbc_vbus_driver_api *)dev->api;

	if (api->measure == NULL) {
		return -ENOTSUP;
	}

	return api->measure(dev, meas);
}

/**
 * @brief Controls a pin that discharges VBUS
 *
 * @param dev        Runtime device structure
 * @param enable     Discharge VBUS when true
 *
 * @retval 0 on success
 * @retval -EIO on failure
 * @retval -ENOENT if discharge pin isn't defined
 * @retval -ENOTSUP if the driver doesn't implement the request
  */
static inline int usbc_vbus_discharge(const struct device *dev, bool enable)
{
	const struct usbc_vbus_driver_api *api = (const struct usbc_vbus_driver_api *)dev->api;

	if (api->discharge == NULL) {
		return -ENOTSUP;
	}

	return api->discharge(dev, enable);
}

/**
 * @brief Controls a pin that enables VBUS measurments
 *
 * @param dev     Runtime device structure
 * @param enable  enable VBUS measurments when true
 *
 * @retval 0 on success
 * @retval -EIO on failure
 * @retval -ENOENT if enable pin isn't defined
 * @retval -ENOTSUP if the driver doesn't implement the request
 */
static inline int usbc_vbus_enable(const struct device *dev, bool enable)
{
	const struct usbc_vbus_driver_api *api = (const struct usbc_vbus_driver_api *)dev->api;

	if (api->enable == NULL) {
		return -ENOTSUP;
	}

	return api->enable(dev, enable);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_USBC_VBUS_H_ */
