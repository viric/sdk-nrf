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
#include <zephyr/drivers/sensor.h>

#define LOG_LEVEL CONFIG_BT_BAS_LOG_LEVEL
#include <zephyr/logging/log.h>

#include "main.h"

LOG_MODULE_REGISTER(temp);

uint8_t bt_temp_get_temperature(void);

const struct device *temp_device;

static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				       uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("TEMP Notifications %s", notif_enabled ? "enabled" : "disabled");
}

static ssize_t read_temp(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	uint8_t lvl8 = bt_temp_get_temperature();

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &lvl8,
				 sizeof(lvl8));
}

BT_GATT_SERVICE_DEFINE(temp,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ENVSENS),
	BT_GATT_CHARACTERISTIC(BT_UUID_ENVSENS_TEMPERATURE,
			       BT_GATT_CHRC_READ/* | BT_GATT_CHRC_NOTIFY*/,
			       BT_GATT_PERM_READ, read_temp, NULL,
			       &temp_device),
	BT_GATT_CCC(temp_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static int temp_init(const struct device *dev)
{
	ARG_UNUSED(dev);

    temp_device = DEVICE_DT_GET(DT_NODELABEL(temp));

    if (!device_is_ready(temp_device))
    {
        temp_device = 0;
        LOG_ERR("TEMP device not found");
    }
    LOG_INF("TEMP device initialised");

	return 0;
}

uint8_t bt_temp_get_temperature(void)
{
    if (temp_device)
    {
        sensor_sample_fetch(temp_device);
        struct sensor_value temp;
        sensor_channel_get(temp_device, SENSOR_CHAN_DIE_TEMP, &temp);

        return temp.val1;
    }
    return 0;
}

SYS_INIT(temp_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
