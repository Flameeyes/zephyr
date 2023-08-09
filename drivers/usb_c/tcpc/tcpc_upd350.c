/*
 * Copyright (c) 2023, Meta Platforms, Inc. and its affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/mfd/upd350.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>

#define LOG_LEVEL CONFIG_USBC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tcpc_upd350);

struct tcpc_upd350_drv_data {
};

struct tcpc_upd350_config {
	const struct device *mfd_dev;
};

static int ucpd_get_cc(const struct device *dev, enum tc_cc_voltage_state *cc1,
		       enum tc_cc_voltage_state *cc2)
{
	return -ENOTSUP;
}

static int ucpd_set_vconn(const struct device *dev, bool enable)
{
	return -ENOTSUP;
}

static int ucpd_vconn_discharge(const struct device *dev, bool enable)
{
	return -ENOTSUP;
}

static int ucpd_select_rp_value(const struct device *dev, enum tc_rp_value rp)
{
	return -ENOTSUP;
}

static int ucpd_get_rp_value(const struct device *dev, enum tc_rp_value *rp)
{
	return -ENOTSUP;
}

static int ucpd_set_cc(const struct device *dev, enum tc_cc_pull cc_pull)
{
	return -ENOTSUP;
}

static int ucpd_cc_set_polarity(const struct device *dev, enum tc_cc_polarity polarity)
{
	return -ENOTSUP;
}

static int ucpd_set_rx_enable(const struct device *dev, bool enable)
{
	return -ENOTSUP;
}

static int ucpd_set_roles(const struct device *dev, enum tc_power_role power_role,
			  enum tc_data_role data_role)
{
	return -ENOTSUP;
}

static int ucpd_sop_prime_enable(const struct device *dev, bool enable)
{
	return -ENOTSUP;
}

static int ucpd_transmit_data(const struct device *dev, struct pd_msg *msg)
{
	return -ENOTSUP;
}

static bool ucpd_is_rx_pending_msg(const struct device *dev, enum pd_packet_type *type)
{
	return false;
}

static int ucpd_receive_data(const struct device *dev, struct pd_msg *msg)
{
	return -ENOTSUP;
}

static int ucpd_set_bist_test_mode(const struct device *dev, bool enable)
{
	return -ENOTSUP;
}

static int ucpd_dump_std_reg(const struct device *dev)
{
	return -ENOTSUP;
}

static int ucpd_set_alert_handler_cb(const struct device *dev, tcpc_alert_handler_cb_t handler,
				     void *alert_data)
{
	return -ENOTSUP;
}

static void ucpd_set_vconn_cb(const struct device *dev, tcpc_vconn_control_cb_t vconn_cb)
{
	return;
}

static void ucpd_set_vconn_discharge_cb(const struct device *dev, tcpc_vconn_discharge_cb_t cb)
{
	return;
}

static int tcpc_upd350_init(const struct device *dev)
{
	const struct tcpc_upd350_config *config = dev->config;

	int err;

	if (!device_is_ready(config->mfd_dev)) {
		LOG_ERR("MFD device %s is not ready", config->mfd_dev->name);
		return -ENODEV;
	}

	err = upd350_identify(config->mfd_dev);
	if (err < 0) {
		return -ENODEV;
	}

	return 0;
}

static const struct tcpc_driver_api driver_api = {
	.init = tcpc_upd350_init,
	.set_alert_handler_cb = ucpd_set_alert_handler_cb,
	.get_cc = ucpd_get_cc,
	.set_rx_enable = ucpd_set_rx_enable,
	.is_rx_pending_msg = ucpd_is_rx_pending_msg,
	.receive_data = ucpd_receive_data,
	.transmit_data = ucpd_transmit_data,
	.select_rp_value = ucpd_select_rp_value,
	.get_rp_value = ucpd_get_rp_value,
	.set_cc = ucpd_set_cc,
	.set_roles = ucpd_set_roles,
	.set_vconn_cb = ucpd_set_vconn_cb,
	.set_vconn_discharge_cb = ucpd_set_vconn_discharge_cb,
	.set_vconn = ucpd_set_vconn,
	.vconn_discharge = ucpd_vconn_discharge,
	.set_cc_polarity = ucpd_cc_set_polarity,
	.dump_std_reg = ucpd_dump_std_reg,
	.set_bist_test_mode = ucpd_set_bist_test_mode,
	.sop_prime_enable = ucpd_sop_prime_enable,
};

#define DT_DRV_COMPAT microchip_upd350_tcpc

#define TCPC_UPD350_DEVICE(inst)                                                                   \
	static struct tcpc_upd350_drv_data upd350_##inst##_drvdata = {};                           \
	static struct tcpc_upd350_config upd350_##inst##_config = {                                \
		.mfd_dev = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tcpc_upd350_init, NULL, &upd350_##inst##_drvdata,              \
			      &upd350_##inst##_config, POST_KERNEL,                                \
			      CONFIG_USBC_TCPC_UPD350_INIT_PRIORITY, &driver_api);

DT_INST_FOREACH_STATUS_OKAY(TCPC_UPD350_DEVICE)
