/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zephyr.h>

#include <zephyr/settings/settings.h>

#include <zephyr/display/mb_display.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/drivers/gpio.h>

#include "main.h"

#define SCROLL_SPEED 400

/* BLE */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_ENVSENS_TEMPERATURE_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

/* BUTTONS */

#define SWITCH_PIN 12
static const struct gpio_dt_spec sw0_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec sw1_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

static int64_t a_timestamp;
static int64_t b_timestamp;

static const struct device *gpio_dev;

void menu_engegar()
{
    gpio_pin_set(gpio_dev, SWITCH_PIN, 1);
}

void menu_apagar()
{
    gpio_pin_set(gpio_dev, SWITCH_PIN, 0);
}

void menu_boot()
{
    struct mb_display *disp = mb_display_get();

    mb_display_print(disp, MB_DISPLAY_MODE_DEFAULT | MB_DISPLAY_FLAG_LOOP,
            SCROLL_SPEED, "termo");
}

struct {
    const char *text;
    void (*callback)();
} options[] = {
    { "engegar", menu_engegar },
    { "apagar", menu_apagar },
    { "inici", menu_boot },
};

static int option;

#define NELEMS(a) sizeof(a)/sizeof(a[0])

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	/* Filter out spurious presses */
	if (pins & BIT(sw0_gpio.pin)) {
		printk("A pressed\n");
		if (k_uptime_delta(&a_timestamp) < 100) {
			printk("Too quick A presses\n");
			return;
		}
	} else {
		printk("B pressed\n");
		if (k_uptime_delta(&b_timestamp) < 100) {
			printk("Too quick B presses\n");
			return;
		}
	}

	if (pins & BIT(sw0_gpio.pin)) {
        ++option;
        if (option >= NELEMS(options)) {
            option = 0;
        }
        struct mb_display *disp = mb_display_get();
        mb_display_print(disp, MB_DISPLAY_MODE_DEFAULT | MB_DISPLAY_FLAG_LOOP,
                SCROLL_SPEED, "%d %s", option, options[option].text);
	} else {
        options[option].callback();
	}
}

static void configure_buttons(void)
{
    static struct gpio_callback button_cb_data;

    /* since sw0_gpio.port == sw1_gpio.port, we only need to check ready once */
    if (!device_is_ready(sw0_gpio.port)) {
        printk("%s: device not ready.\n", sw0_gpio.port->name);
        return;
    }

    gpio_pin_configure_dt(&sw0_gpio, GPIO_INPUT);
    gpio_pin_configure_dt(&sw1_gpio, GPIO_INPUT);

    gpio_pin_configure_dt(&sw1_gpio, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&sw0_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&sw1_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_cb_data, button_pressed,
               BIT(sw0_gpio.pin) | BIT(sw1_gpio.pin));

    gpio_add_callback(sw0_gpio.port, &button_cb_data);

    /* Switch */
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(sw0_gpio.port)) {
        printk("%s: device not ready.\n", gpio_dev->name);
        return;
    }

    gpio_pin_configure(gpio_dev, SWITCH_PIN, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
}

void main(void)
{
	int err;

    configure_buttons();

    menu_boot();

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_auth_cb_register(&auth_cb_display);

	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	while (1) {
		k_sleep(K_SECONDS(1));
	}
}
