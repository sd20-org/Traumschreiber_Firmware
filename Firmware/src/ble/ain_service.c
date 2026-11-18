/*
 * AIN BLE Service - Implementation
 *
 * Streams Nordic SAADC AIN0-AIN7 raw ADC values via BLE notifications every
 * 5 seconds when notifications are enabled.
 *
 * Notification payload layout (little-endian):
 * [0]    : header = 0xA0
 * [1..4] : timestamp_ms (uint32_t)
 * [5..20]: channels[0..7] raw uint16_t values (16 bytes)
 * Total length: 21 bytes
 *
 * @author ISDT
 * @date 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#include "ain_service.h"
#include "../drivers/nordic_adc.h"

LOG_MODULE_REGISTER(ain_service, LOG_LEVEL_INF);

/* Notify interval (ms) */
#define AIN_NOTIFY_INTERVAL_MS (5 * 1000)

static bool notify_enabled = false;
static struct k_work_delayable ain_monitor_work;
static nordic_adc_sample_t g_last_sample;
static bool g_last_sample_valid = false;

/* Forward declarations */
static void ain_monitor_work_handler(struct k_work *work);
static void ain_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t ain_data_read(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset);

/* AIN Service Definition */
BT_GATT_SERVICE_DEFINE(ain_service,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_AIN_SERVICE),

                       /* AIN Data Characteristic */
                       BT_GATT_CHARACTERISTIC(BT_UUID_AIN_DATA,
                                              BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              ain_data_read, NULL, NULL),
                       BT_GATT_CCC(ain_data_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

static void ain_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool was = notify_enabled;
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);

    LOG_INF("AIN notifications %s",
            notify_enabled ? "enabled" : "disabled");

    if (notify_enabled && !was)
    {
        /* schedule immediate work */
        k_work_schedule(&ain_monitor_work, K_NO_WAIT);
    }
    else if (!notify_enabled && was)
    {
        /* cancel pending work */
        k_work_cancel_delayable(&ain_monitor_work);
    }
}

static ssize_t ain_data_read(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    uint8_t payload[21];

    if (!g_last_sample_valid)
    {
        return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, 0);
    }

    payload[0] = 0xA0;
    uint32_t ts = (uint32_t)g_last_sample.timestamp_ms;
    memcpy(&payload[1], &ts, 4);

    for (int i = 0; i < NORDIC_ADC_NUM_CHANNELS; i++)
    {
        uint16_t raw = g_last_sample.channels[i];
        memcpy(&payload[5 + i * 2], &raw, 2);
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, sizeof(payload));
}

static void ain_monitor_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!notify_enabled)
    {
        /* not subscribed; reschedule to check again later */
        k_work_schedule(&ain_monitor_work, K_MSEC(AIN_NOTIFY_INTERVAL_MS));
        return;
    }

    nordic_adc_sample_t sample;
    if (nordic_adc_sample_once(&sample) == NORDIC_ADC_OK)
    {
        g_last_sample = sample;
        g_last_sample_valid = true;

        uint8_t payload[21];
        payload[0] = 0xA0;
        uint32_t ts = (uint32_t)sample.timestamp_ms;
        memcpy(&payload[1], &ts, 4);
        for (int i = 0; i < NORDIC_ADC_NUM_CHANNELS; i++)
        {
            uint16_t raw = sample.channels[i];
            memcpy(&payload[5 + i * 2], &raw, 2);
        }

        int err = bt_gatt_notify(NULL, &ain_service.attrs[1], payload, sizeof(payload));
        if (err)
        {
            LOG_WRN("AIN notification failed: %d", err);
        }
        else
        {
            LOG_DBG("AIN notification sent: ts=%u ms", ts);
        }
    }
    else
    {
        LOG_WRN("AIN sampling failed; not sending notification");
    }

    /* Reschedule after interval */
    k_work_schedule(&ain_monitor_work, K_MSEC(AIN_NOTIFY_INTERVAL_MS));
}

int ain_service_init(void)
{
    LOG_INF("=== Initializing AIN BLE Service ===");

    /* Init work item */
    k_work_init_delayable(&ain_monitor_work, ain_monitor_work_handler);

    /* Initially not scheduled; it will start when client enables notifications */
    g_last_sample_valid = false;

    LOG_INF("AIN BLE service initialized (notify interval: %d ms)", AIN_NOTIFY_INTERVAL_MS);

    return 0;
}