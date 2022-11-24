/** @file
 *  @brief GATT Battery Service
 */

/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <stdbool.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/adc.h>

#define LOG_LEVEL CONFIG_BT_BAS_LOG_LEVEL
#include <zephyr/logging/log.h>

#include "main.h"

LOG_MODULE_REGISTER(voltage);

int32_t bt_voltage_get_power(void);

static const struct adc_dt_spec power_adc_spec =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int properly_setup = 0;

static ssize_t read_power(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	int32_t lvl = bt_voltage_get_power();

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &lvl,
				 sizeof(lvl));
}

BT_GATT_SERVICE_DEFINE(voltage,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_GATT),
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_POWER,
			       BT_GATT_CHRC_READ/* | BT_GATT_CHRC_NOTIFY*/,
			       BT_GATT_PERM_READ, read_power, NULL,
			       NULL),
);

static int voltage_init(const struct device *dev)
{
    int res;

    res = adc_channel_setup_dt(&power_adc_spec);

    if (res != 0)
    {
        LOG_ERR("ADC device not found");
        return -1;
    }
    LOG_INF("ADC device initialised");

    if (res != 0)
    {
        LOG_ERR("ADC channel setup failed");
        return -1;
    }

    properly_setup = 1;

	return 0;
}

int32_t bt_voltage_get_power(void)
{
    int16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };
    int err;

    if (!properly_setup)
    {
        return -1;
    }

    adc_sequence_init_dt(&power_adc_spec, &sequence);

    err = adc_read(power_adc_spec.dev, &sequence);
    if (err < 0)
    {
        LOG_ERR("ADC reading failed");
        return -1;
    }
    LOG_INF("ADC raw read: %hd", buf);

    int32_t val_mv = buf;
    err = adc_raw_to_millivolts_dt(&power_adc_spec, &val_mv);
    if (err < 0)
    {
        LOG_ERR("Conv mv failed");
        return -1;
    }

    LOG_INF("ADC mV read: %d", val_mv);

    return val_mv;
}

SYS_INIT(voltage_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
