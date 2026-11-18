/*
 * LED Button Service (LBS) - Implementation
 * Based on Nordic Semiconductor's peripheral_lbs sample
 *
 * Implements a custom BLE GATT service with:
 * - Button state characteristic (notify)
 * - LED control characteristic (write)
 * - RGB LED control characteristic (write)
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

#include "lbs_service.h"
#include <string.h>
#include "subsystems/audio_player.h"

LOG_MODULE_REGISTER(lbs_service, LOG_LEVEL_INF);

static bool notify_enabled;
static bool led_notify_enabled;
static bool playback_notify_enabled;
static struct lbs_cb lbs_callbacks;

/* Current button state for read operations */
static uint8_t button_state;
/* Current LED state for read operations: 0 = off, 1 = on */
/* Current LED RGB state for read operations: 3 bytes [R,G,B] */
static uint8_t led_state[3];

/* Playback state (single byte) */
static uint8_t playback_state = LBS_PLAYBACK_STATE_STOPPED;

/* Pointer to LED and playback state attributes (resolved at init) */
static const struct bt_gatt_attr *led_state_attr = NULL;
static const struct bt_gatt_attr *playback_state_attr = NULL;

/* Sound entry notification state and attribute */
static bool sound_notify_enabled;
static const struct bt_gatt_attr *sound_entry_attr = NULL;
static char sound_entry[64];

/* Pending sound list request and worker */
static bool sound_list_request_pending = false;
static struct k_work sound_list_work;
static struct k_work_delayable sound_list_timeout_work;
static size_t sound_list_count = 0;

/**
 * @brief LED characteristic write callback
 *
 * Called when the smartphone writes to the LED characteristic.
 * Controls a single on/off LED.
 */
static ssize_t write_led(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset,
                         uint8_t flags)
{
    LOG_DBG("LED write received, len: %u", len);

    if (len != 1U)
    {
        LOG_WRN("Invalid LED write length: %u", len);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (offset != 0)
    {
        LOG_WRN("Invalid LED write offset: %u", offset);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (lbs_callbacks.led_cb)
    {
        uint8_t val = *((uint8_t *)buf);
        lbs_callbacks.led_cb(val ? true : false);
        LOG_INF("LED state changed to: %s", val ? "ON" : "OFF");
    }

    return len;
}

/**
 * @brief RGB LED characteristic write callback
 *
 * Called when the smartphone writes to the RGB LED characteristic.
 * Controls the RGB LED color with 3 bytes [R, G, B].
 */
static ssize_t write_rgb_led(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset,
                             uint8_t flags)
{
    LOG_DBG("RGB LED write received, len: %u", len);

    if (len != 3U)
    {
        LOG_WRN("Invalid RGB LED write length: %u (expected 3)", len);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (offset != 0)
    {
        LOG_WRN("Invalid RGB LED write offset: %u", offset);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (lbs_callbacks.rgb_led_cb)
    {
        const uint8_t *rgb = (const uint8_t *)buf;
        lbs_callbacks.rgb_led_cb(rgb[0], rgb[1], rgb[2]);
        LOG_INF("RGB LED color changed to: R=%u G=%u B=%u",
                rgb[0], rgb[1], rgb[2]);
    }

    return len;
}

/**
 * @brief Button characteristic read callback
 *
 * Called when the smartphone reads the button characteristic.
 */
static ssize_t read_button(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *value = attr->user_data;

    LOG_DBG("Button read, state: %u", *value);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(*value));
}

/**
 * @brief Button characteristic CCC write callback
 *
 * Called when the smartphone enables/disables notifications.
 */
static void button_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                   uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);

    LOG_INF("Button notifications %s",
            notify_enabled ? "enabled" : "disabled");
}

static void led_state_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    led_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("LED state notifications %s", led_notify_enabled ? "enabled" : "disabled");
}

static ssize_t read_led_state(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *value = attr->user_data;

    LOG_DBG("LED state read, R=%u G=%u B=%u", value[0], value[1], value[2]);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(led_state));
}

/**
 * @brief Write-only characteristic handler that allows a connected client
 * to request the current LED RGB state be sent as a notification.
 * Any write payload is accepted; the act of writing triggers the notify
 * (if notifications are enabled).
 */
static ssize_t write_led_state_request(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len, uint16_t offset,
                                       uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(buf);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    LOG_DBG("LED state request received from client (len=%u)", len);

    if (!led_notify_enabled)
    {
        LOG_DBG("LED notifications not enabled, cannot satisfy request");
        return -EACCES;
    }

    if (led_state_attr == NULL)
    {
        LOG_WRN("LED state attribute not found, cannot notify");
        return -EINVAL;
    }

    int rc = bt_gatt_notify(NULL, led_state_attr, &led_state, sizeof(led_state));
    if (rc < 0)
    {
        LOG_WRN("Failed to notify LED state on request: %d", rc);
        /* Still return the written length so the write itself succeeds */
    }

    return len;
}

/* Playback: request to send current playback state as notification (write-only) */
static ssize_t write_playback_state_request(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            const void *buf, uint16_t len, uint16_t offset,
                                            uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(buf);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    LOG_DBG("Playback state request received (len=%u)", len);

    if (!playback_notify_enabled)
    {
        LOG_DBG("Playback notifications not enabled, cannot satisfy request");
        return -EACCES;
    }

    if (playback_state_attr == NULL)
    {
        LOG_WRN("Playback state attribute not found, cannot notify");
        return -EINVAL;
    }

    int rc = bt_gatt_notify(NULL, playback_state_attr, &playback_state, sizeof(playback_state));
    if (rc < 0)
    {
        LOG_WRN("Failed to notify playback state on request: %d", rc);
    }

    return len;
}

/**
 * @brief Write callback to request playback of a file from SD card
 *
 * Payload: UTF-8 path (max 64 bytes), e.g. "SOUNDS/BELL1.WAV"
 */
static ssize_t write_play_file(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len, uint16_t offset,
                               uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);

    LOG_DBG("Play file write received, len: %u", len);

    if (offset != 0)
    {
        LOG_WRN("Invalid play file write offset: %u", offset);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len == 0 || len > 64)
    {
        LOG_WRN("Invalid play file write length: %u", len);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    char path[65];
    memcpy(path, buf, len);
    path[len] = '\0';

    /* Basic sanitization: no parent directory references and must be under SOUNDS/ */
    if (strstr(path, "..") != NULL)
    {
        LOG_WRN("Rejected play path with parent reference: %s", path);
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);
    }

    if (strncmp(path, "SOUNDS/", 7) != 0)
    {
        LOG_WRN("Rejected play path outside SOUNDS/: %s", path);
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);
    }

    if (lbs_callbacks.play_file_cb)
    {
        lbs_callbacks.play_file_cb(path);
        LOG_INF("Play request: %s", path);
    }

    return len;
}

/**
 * @brief Playback control (1-byte opcode)
 * Opcodes: 0 = STOP, 1 = PAUSE (not implemented), 2 = RESUME (not implemented)
 */
static ssize_t write_playback_control(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);

    LOG_DBG("Playback control write received, len: %u", len);

    if (offset != 0)
    {
        LOG_WRN("Invalid playback control write offset: %u", offset);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != 1)
    {
        LOG_WRN("Invalid playback control write length: %u", len);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t opcode = *((const uint8_t *)buf);

    if (lbs_callbacks.playback_ctrl_cb)
    {
        lbs_callbacks.playback_ctrl_cb(opcode);
        LOG_INF("Playback control opcode received: %u", opcode);
    }

    return len;
}

static void playback_state_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    playback_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Playback state notifications %s", playback_notify_enabled ? "enabled" : "disabled");
}

static ssize_t read_playback_state(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *value = attr->user_data;

    LOG_DBG("Playback state read: %u", *value);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(playback_state));
}

static void sound_entry_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    sound_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Sound entry notifications %s", sound_notify_enabled ? "enabled" : "disabled");

    /* If a list request was pending, trigger it now */
    if (sound_notify_enabled && sound_list_request_pending)
    {
        LOG_INF("Pending SOUND_LIST_REQ detected and notifications are now enabled - scheduling listing");
        sound_list_request_pending = false;
        k_work_cancel_delayable(&sound_list_timeout_work);
        k_work_submit(&sound_list_work);
    }
}

static ssize_t read_sound_entry(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    const char *value = (const char *)attr->user_data;

    LOG_DBG("Sound entry read: %s", value);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

/* Forwarder used by audio_player_list_wav_files to send notifications and count them */
static void audio_player_counting_forward_cb(const char *filename, void *ctx)
{
    ARG_UNUSED(ctx);
    sound_list_count++;
    int rc = lbs_send_sound_entry(filename);
    if (rc < 0)
    {
        LOG_WRN("Failed to send SOUND_ENTRY: %d", rc);
    }
    /* Small delay to avoid overrunning notification queue */
    k_sleep(K_MSEC(10));
}

/**
 * @brief Write-only characteristic handler that requests a listing of WAV files
 * in the SOUNDS/ directory. Triggers SOUND_ENTRY notifications for each file.
 */
static void perform_sound_list_work(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!sound_notify_enabled)
    {
        LOG_WRN("Cannot perform sound list: notifications not enabled");
        return;
    }

    LOG_INF("Performing SOUND_LIST enumeration...");
    sound_list_count = 0;

    int ret = audio_player_list_wav_files(audio_player_counting_forward_cb, NULL);

    /* Send end-of-list marker */
    (void)lbs_send_sound_entry("[END]");

    if (ret == AUDIO_PLAYER_OK)
    {
        LOG_INF("SOUND_LIST enumeration complete: %zu entries sent", sound_list_count);
    }
    else
    {
        LOG_WRN("SOUND_LIST enumeration failed: %d", ret);
    }
}

static void sound_list_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (sound_list_request_pending)
    {
        LOG_WRN("SOUND_LIST_REQ pending timed out (no CCC); dropping request");
        sound_list_request_pending = false;
    }
}

static ssize_t write_sound_list_request(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        const void *buf, uint16_t len, uint16_t offset,
                                        uint8_t flags)
{
    char addr[BT_ADDR_LE_STR_LEN] = "(unknown)";

    if (conn)
    {
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    }

    LOG_INF("SOUND_LIST_REQ from %s, len=%u", addr, len);
    if (len > 0 && buf)
    {
        const uint8_t *b = (const uint8_t *)buf;
        LOG_DBG("SOUND_LIST_REQ payload[0]=0x%02X", b[0]);
    }

    if (sound_entry_attr == NULL)
    {
        LOG_WRN("Sound entry attribute not found, cannot notify");
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    if (!sound_notify_enabled)
    {
        /* Defer listing until client enables notifications */
        LOG_INF("Notifications not enabled - deferring SOUND_LIST_REQ (will wait up to 5s)");
        sound_list_request_pending = true;
        k_work_schedule(&sound_list_timeout_work, K_SECONDS(5));
        return len;
    }

    /* Notifications are enabled - perform listing on worker to avoid blocking GATT context */
    k_work_submit(&sound_list_work);

    return len;
}

/* LBS Service Declaration */
BT_GATT_SERVICE_DEFINE(lbs_svc,
                       /* Primary Service Declaration */
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS),

                       /* Button Characteristic (Read + Notify) */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_BUTTON,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_button, NULL, &button_state),
                       BT_GATT_CCC(button_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* LED State Characteristic (Read + Notify) */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED_STATE,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_led_state, NULL, led_state),
                       BT_GATT_CCC(led_state_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* LED STATE REQUEST Characteristic (Write-only). Writing any payload
                        * triggers the device to send the current LED_STATE as a notification
                        * (if notifications are enabled).
                        */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED_STATE_REQ,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_led_state_request, NULL),

                       /* LED Characteristic (Write) */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_led, NULL),

                       /* RGB LED Characteristic (Write) */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_RGB_LED,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_rgb_led, NULL),

                       /* Playback: Play File (Write) */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PLAY_FILE,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_play_file, NULL),

                       /* Playback Control (Write: 1 byte opcode) */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PLAYBACK_CONTROL,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_playback_control, NULL),

                       /* Playback State (Read + Notify): 1 byte */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PLAYBACK_STATE,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_playback_state, NULL, &playback_state),
                       BT_GATT_CCC(playback_state_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* Playback STATE REQUEST (Write-only): triggers notify if enabled */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PLAYBACK_STATE_REQ,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_playback_state_request, NULL),

                       /* Sound Entry Characteristic (Read + Notify): single filename */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_SOUND_ENTRY,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_sound_entry, NULL, sound_entry),
                       BT_GATT_CCC(sound_entry_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* Sound List Request (Write-only): requests device to enumerate SOUNDS/ */
                       BT_GATT_CHARACTERISTIC(BT_UUID_LBS_SOUND_LIST_REQ,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_sound_list_request, NULL), );

int lbs_init(struct lbs_cb *callbacks)
{
    if (callbacks == NULL)
    {
        LOG_ERR("Callbacks cannot be NULL");
        return -EINVAL;
    }

    lbs_callbacks.led_cb = callbacks->led_cb;
    lbs_callbacks.rgb_led_cb = callbacks->rgb_led_cb;
    lbs_callbacks.button_cb = callbacks->button_cb;
    lbs_callbacks.play_file_cb = callbacks->play_file_cb;
    lbs_callbacks.playback_ctrl_cb = callbacks->playback_ctrl_cb;

    /* Initialize work items */
    k_work_init(&sound_list_work, perform_sound_list_work);
    k_work_init_delayable(&sound_list_timeout_work, sound_list_timeout_handler);

    /* Locate LED, Playback and Sound Entry attributes in the generated service attrs array so we can notify them later */
    for (size_t i = 0; i < lbs_svc.attr_count; i++)
    {
        const struct bt_gatt_attr *a = &lbs_svc.attrs[i];
        if (a->uuid)
        {
            if (bt_uuid_cmp(a->uuid, BT_UUID_LBS_LED_STATE) == 0)
            {
                led_state_attr = a;
                LOG_DBG("Found LED state attribute at index %u", (unsigned)i);
            }

            if (bt_uuid_cmp(a->uuid, BT_UUID_LBS_PLAYBACK_STATE) == 0)
            {
                playback_state_attr = a;
                LOG_DBG("Found playback state attribute at index %u", (unsigned)i);
            }

            if (bt_uuid_cmp(a->uuid, BT_UUID_LBS_SOUND_ENTRY) == 0)
            {
                sound_entry_attr = a;
                LOG_DBG("Found sound entry attribute at index %u", (unsigned)i);
            }

            if (led_state_attr && playback_state_attr && sound_entry_attr)
            {
                break; /* found all */
            }
        }
    }

    /* Initialize led_state to off (R,G,B = 0) */
    led_state[0] = 0;
    led_state[1] = 0;
    led_state[2] = 0;

    /* Initialize playback_state to STOPPED */
    playback_state = LBS_PLAYBACK_STATE_STOPPED;

    LOG_INF("LBS service initialized");
    return 0;
}

int lbs_send_button_state(bool state)
{
    if (!notify_enabled)
    {
        LOG_DBG("Notifications not enabled, skipping button update");
        return -EACCES;
    }

    button_state = state ? 1 : 0;

    LOG_INF("Sending button notification: %s", state ? "PRESSED" : "RELEASED");

    return bt_gatt_notify(NULL, &lbs_svc.attrs[1],
                          &button_state, sizeof(button_state));
}

int lbs_send_led_state_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    /* Detect whether the RGB actually changed; if not, skip notifying */
    bool changed = (led_state[0] != r) || (led_state[1] != g) || (led_state[2] != b);

    if (!changed)
    {
        LOG_DBG("LED RGB unchanged (R=%u G=%u B=%u), skipping notify", (unsigned)r, (unsigned)g, (unsigned)b);
        return 0;
    }

    /* Update stored state so reads reflect the latest color even if notifications are disabled */
    led_state[0] = r;
    led_state[1] = g;
    led_state[2] = b;

    if (led_state_attr == NULL)
    {
        LOG_WRN("LED state attribute not found, cannot notify");
        return -EINVAL;
    }

    if (!led_notify_enabled)
    {
        LOG_DBG("LED notifications not enabled, skipping LED update notify");
        return -EACCES;
    }

    LOG_INF("Sending LED RGB notification: R=%u G=%u B=%u", (unsigned)r, (unsigned)g, (unsigned)b);

    return bt_gatt_notify(NULL, led_state_attr, &led_state, sizeof(led_state));
}

/**
 * @brief Send playback state notification to connected client
 */
int lbs_send_playback_state(uint8_t state)
{
    bool changed = (playback_state != state);

    if (!changed)
    {
        LOG_DBG("Playback state unchanged (%u), skipping notify", (unsigned)state);
        return 0;
    }

    playback_state = state;

    if (playback_state_attr == NULL)
    {
        LOG_WRN("Playback state attribute not found, cannot notify");
        return -EINVAL;
    }

    if (!playback_notify_enabled)
    {
        LOG_DBG("Playback notifications not enabled, skipping playback update notify");
        return -EACCES;
    }

    LOG_INF("Sending playback state notification: %u", (unsigned)state);

    return bt_gatt_notify(NULL, playback_state_attr, &playback_state, sizeof(playback_state));
}

/**
 * @brief Send a single filename as SOUND_ENTRY notification
 */
int lbs_send_sound_entry(const char *name)
{
    if (!name)
    {
        name = "";
    }

    /* Copy into local buffer for read operations */
    size_t len = strlen(name);
    if (len >= sizeof(sound_entry))
    {
        len = sizeof(sound_entry) - 1;
    }
    memcpy(sound_entry, name, len);
    sound_entry[len] = '\0';

    if (sound_entry_attr == NULL)
    {
        LOG_WRN("Sound entry attribute not found, cannot notify");
        return -EINVAL;
    }

    if (!sound_notify_enabled)
    {
        LOG_DBG("Sound notifications not enabled, skipping notify: %s", sound_entry);
        return -EACCES;
    }

    LOG_INF("Sending sound entry notification: %s", sound_entry);
    return bt_gatt_notify(NULL, sound_entry_attr, sound_entry, strlen(sound_entry));
}

bool lbs_notify_enabled(void)
{
    return notify_enabled;
}
