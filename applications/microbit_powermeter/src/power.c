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
#include <math.h>

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

static int32_t mean = 0;

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
    // Scan 100 samples per 20ms cycle. 20 cycles, 2000 samples
    // 0.2ms per sample, 200us.
#define NSAMPLES 2000
    static int16_t buf[NSAMPLES];
    const struct adc_sequence_options options = {
        .interval_us = 200,
        .extra_samplings = NSAMPLES-1,
    };
    struct adc_sequence sequence = {
        .options = &options,
        .buffer = buf,
        .channels = 1,
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
    LOG_INF("ADC raw[0] read: %hd", buf[0]);
    LOG_INF("ADC raw[NSAMPLES-1] read: %hd", buf[NSAMPLES-1]);

    const float factor = 0.6 * 6 / 1024 / 220 * 2000 * 230;

    int32_t meansum = 0;
    int32_t sum = 0;

    for(int i=0; i < NSAMPLES; ++i)
    {
        meansum += buf[i];
        int32_t filtered = buf[i] - mean;
        sum += filtered * filtered;
    }

    mean = meansum / NSAMPLES;
    int32_t power = sqrtf((float)sum / NSAMPLES) * factor;

    LOG_INF("Voltage raw mean: %d", mean);
    LOG_INF("ADC power calculation: %d", power);

    return power;
}

SYS_INIT(voltage_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
