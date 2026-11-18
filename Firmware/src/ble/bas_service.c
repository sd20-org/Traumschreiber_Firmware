/*
 * Battery Service (BAS) - Implementation
 * Standard Bluetooth SIG Battery Service (UUID: 0x180F)
 *
 * Provides battery level information to connected BLE clients.
 *
 * @author ISDT
 * @date 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "bas_service.h"

LOG_MODULE_REGISTER(bas_service, LOG_LEVEL_INF);

/* Current battery level (0-100%) */
static uint8_t battery_level = 100;

/* BAS CCC (Client Characteristic Configuration) descriptor state */
static bool notify_enabled = false;

/**
 * @brief Battery Level characteristic read callback
 */
static ssize_t read_battery_level(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *value = attr->user_data;

    LOG_DBG("Battery level read: %u%%", *value);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(*value));
}

/**
 * @brief Battery Level CCC write callback
 */
static void battery_level_ccc_changed(const struct bt_gatt_attr *attr,
                                      uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);

    LOG_INF("Battery level notifications %s",
            notify_enabled ? "enabled" : "disabled");
}

/* Battery Service Declaration (UUID: 0x180F) */
BT_GATT_SERVICE_DEFINE(bas_svc,
                       /* Primary Service: Battery Service */
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),

                       /* Battery Level Characteristic (UUID: 0x2A19) */
                       BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_battery_level, NULL, &battery_level),

                       /* Client Characteristic Configuration Descriptor */
                       BT_GATT_CCC(battery_level_ccc_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

int bas_init(void)
{
    LOG_INF("Battery Service initialized");
    LOG_INF("  Service UUID: 0x180F");
    LOG_INF("  Battery Level UUID: 0x2A19");
    LOG_INF("  Initial level: %u%%", battery_level);

    return 0;
}

int bas_battery_level_update(uint8_t level_percent)
{
    if (level_percent > 100)
    {
        LOG_WRN("Invalid battery level: %u%% (clamping to 100%%)",
                level_percent);
        level_percent = 100;
    }

    battery_level = level_percent;

    LOG_DBG("Battery level updated to: %u%%", battery_level);

    /* Send notification if enabled */
    if (notify_enabled)
    {
        int err = bt_gatt_notify(NULL, &bas_svc.attrs[1],
                                 &battery_level, sizeof(battery_level));
        if (err)
        {
            LOG_ERR("Failed to send battery notification: %d", err);
            return err;
        }

        LOG_DBG("Battery notification sent: %u%%", battery_level);
    }

    return 0;
}

uint8_t bas_battery_level_get(void)
{
    return battery_level;
}
