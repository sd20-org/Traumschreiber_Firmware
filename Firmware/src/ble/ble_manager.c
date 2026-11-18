/*
 * BLE Manager - Implementation
 *
 * Handles Bluetooth Low Energy initialization, advertising,
 * and connection management for Traumschreiber2509
 *
 * @author ISDT
 * @date 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/sys/byteorder.h>

#include "ble_manager.h"
#include "lbs_service.h"
#include "bas_service.h"
#include "ads_service.h"
#include "ain_service.h"
#include "hw_abstraction/rgb_led.h"
#include "hw_abstraction/button.h"
#include "subsystems/system_config.h"
#include "subsystems/audio_player.h"
#include "subsystems/led_animation.h"
#include "ble/lbs_service.h"

LOG_MODULE_REGISTER(ble_manager, LOG_LEVEL_INF);

/* BLE state */
static bool initialized = false;
static bool connected = false;
static struct bt_conn *current_conn = NULL;
static bool radio_enabled = true;
static ble_manager_adv_mode_t current_adv_mode = BLE_MANAGER_ADV_MODE_NORMAL;

/* Advertising restart retry work - retries advertising start if initial attempt fails */
static void advertising_restart_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(advertising_restart_work, advertising_restart_work_handler);

static void advertising_restart_work_handler(struct k_work *work)
{
    int err = ble_manager_start_advertising();
    if (err != BLE_MANAGER_OK)
    {
        LOG_WRN("Retrying advertising start in 1000 ms");
        k_work_schedule(&advertising_restart_work, K_MSEC(1000));
    }
}

/* Device name - compile-time default (used if no runtime override provided) */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Advertising flags and service UUID in advertising payload (keep scan response small for device name) */
static const struct bt_data ad_flags[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

/* Scan response data - only runtime device name (filled at init) */
static uint8_t runtime_name_buf[32]; /* will hold nul-terminated name; size must be sufficient */
static struct bt_data sd[1] = {
    {.type = BT_DATA_NAME_COMPLETE, .data_len = 0, .data = (const uint8_t *)runtime_name_buf}};

/* Compile-time check for default name */
BUILD_ASSERT(DEVICE_NAME_LEN > 0, "Device name cannot be empty");

/* Note: runtime-provided device name will be validated/truncated at runtime to fit advertising limits */

/**
 * @brief Validate and print advertising/scan response data for debugging
 * @param ad Pointer to advertising array
 * @param ad_count Number of advertising fields
 * @param sd Pointer to scan response array
 * @param sd_count Number of scan response fields
 */
static void debug_print_adv_data(const struct bt_data *ad, size_t ad_count,
                                 const struct bt_data *sd, size_t sd_count)
{
    size_t total_ad_len = 0;
    size_t total_sd_len = 0;

    LOG_INF("=== Advertising Data Validation ===");

    LOG_INF("Advertising Data (AD):");
    for (int i = 0; i < (int)ad_count; i++)
    {
        size_t field_len = 2 + ad[i].data_len;
        total_ad_len += field_len;

        LOG_INF("  Field %d:", i);
        LOG_INF("    Type: 0x%02x", ad[i].type);
        LOG_INF("    Data length: %u bytes", ad[i].data_len);
        LOG_INF("    Field size: %u bytes (including header)", field_len);

        if (ad[i].type == BT_DATA_NAME_COMPLETE)
        {
            LOG_INF("    Device name: '%.*s'", ad[i].data_len, (const char *)ad[i].data);
        }
    }
    LOG_INF("  Total AD packet size: %u bytes", total_ad_len);
    if (total_ad_len > 31)
    {
        LOG_ERR("  !!! AD too large! Max is 31 bytes !!!");
    }

    LOG_INF("Scan Response Data (SD):");
    for (int i = 0; i < (int)sd_count; i++)
    {
        size_t field_len = 2 + sd[i].data_len;
        total_sd_len += field_len;

        LOG_INF("  Field %d:", i);
        LOG_INF("    Type: 0x%02x", sd[i].type);
        LOG_INF("    Data length: %u bytes", sd[i].data_len);
        LOG_INF("    Field size: %u bytes (including header)", field_len);

        if (sd[i].type == BT_DATA_UUID128_ALL)
        {
            LOG_INF("    UUID (hex):");
            const uint8_t *uuid = sd[i].data;
            LOG_HEXDUMP_INF(uuid, sd[i].data_len, "      ");
        }
    }
    LOG_INF("  Total SD packet size: %u bytes", total_sd_len);
    if (total_sd_len > 31)
    {
        LOG_ERR("  !!! SD too large! Max is 31 bytes !!!");
    }

    LOG_INF("=== Validation Complete ===");
}

/**
 * @brief Connection callback - called when device connects
 */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err)
    {
        LOG_ERR("Connection failed: %s (err 0x%02x)", addr, err);
        return;
    }

    LOG_INF("Connected: %s", addr);
    connected = true;
    current_conn = bt_conn_ref(conn);

    /* Visual feedback: animate LED blue → green on connection */
    led_animation_on_connected();

    /* Notify ADS service about connection (will trigger MTU/PHY/Data Length updates) */
    ads_service_on_connected(conn);
}

/**
 * @brief Disconnection callback - called when device disconnects
 */
static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

    connected = false;
    if (current_conn)
    {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    /* Notify ADS service about disconnection */
    ads_service_on_disconnected(conn);

    /* Visual feedback: resume advertising pulse */
    led_animation_on_disconnected();

    /* Restart advertising once the connection object is recycled (recycled_cb).
     * Starting it right here always failed with -ENOMEM (the conn object is
     * still referenced inside this callback) and only the 1 s retry worked —
     * every reconnect lost a second (RTT 2026-10-03). The 1 s work stays as a
     * fallback in case recycled never fires. */
    if (radio_enabled)
    {
        k_work_schedule(&advertising_restart_work, K_MSEC(1000));
    }
}

/**
 * @brief Connection recycled callback - advertising can restart
 */
static void recycled_cb(void)
{
    LOG_DBG("Connection object recycled, ready for new connection");
    if (radio_enabled && !connected)
    {
        k_work_reschedule(&advertising_restart_work, K_NO_WAIT);
    }
}

/* BLE connection callbacks */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .recycled = recycled_cb,
};

/**
 * @brief LBS LED callback - called when smartphone writes to LED characteristic
 */
static void lbs_led_callback(bool led_state)
{
    LOG_INF("BLE LED control: %s", led_state ? "ON" : "OFF");

    /* Simple on/off control - use white for on, off for off */
    if (led_state)
    {
        rgb_led_set_color(&RGB_COLOR_WHITE);
    }
    else
    {
        rgb_led_off();
    }
}

/**
 * @brief LBS RGB LED callback - called when smartphone writes RGB color
 */
static void lbs_rgb_led_callback(uint8_t r, uint8_t g, uint8_t b)
{
    LOG_INF("BLE RGB LED control: R=%u G=%u B=%u", r, g, b);

    /* Create RGB color and set it (RGB values are 0-255, need to convert to 0-100%) */
    rgb_color_t color = {
        .red = (r * 100) / 255,
        .green = (g * 100) / 255,
        .blue = (b * 100) / 255};
    rgb_led_set_color(&color);
}

/**
 * @brief LBS play file callback - called when smartphone writes file path
 */
static void lbs_play_file_callback(const char *path)
{
    LOG_INF("BLE Play file request: %s", path);

    int res = audio_player_play_wav_from_sd_async(path);
    if (res == AUDIO_PLAYER_OK)
    {
        int rc = lbs_send_playback_state(LBS_PLAYBACK_STATE_PLAYING);
        if (rc)
        {
            LOG_WRN("Failed to send playback PLAYING notification: %d", rc);
        }
    }
    else
    {
        LOG_WRN("Failed to start playback: %d", res);
        lbs_send_playback_state(LBS_PLAYBACK_STATE_ERROR);
    }
}

/**
 * @brief LBS playback control callback - handles 1-byte opcodes
 */
static void lbs_playback_control_callback(uint8_t opcode)
{
    LOG_INF("BLE Playback control opcode: %u", opcode);
    switch (opcode)
    {
    case 0: /* STOP */
    {
        int err = audio_player_stop();
        if (err == AUDIO_PLAYER_OK)
        {
            lbs_send_playback_state(LBS_PLAYBACK_STATE_STOPPED);
        }
        else
        {
            LOG_WRN("audio_player_stop failed: %d", err);
        }
        break;
    }
    default:
        LOG_WRN("Playback opcode not implemented: %u", opcode);
        break;
    }
}

/**
 * @brief LBS button callback - returns current button state
 */
static bool lbs_button_callback(void)
{
    return button_is_pressed();
}

/* Forward declaration of button event handler */
static void ble_button_event_handler(button_event_t event);

/*
 * Helper: set TX power using HCI vendor-specific write command
 * Mirrors the pattern of Nordic's hci_pwr_ctrl sample
 *
 * NOTE: bt_hci_cmd_alloc() is not available in this NCS/Zephyr build configuration.
 * Runtime TX power control via HCI VS is disabled until proper API access is configured.
 * TX power can still be set via Kconfig (CONFIG_BT_CTLR_TX_PWR_*) at build time.
 */
#if 0 /* Disabled: bt_hci_cmd_alloc not available in link */
static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
    struct bt_hci_cp_vs_write_tx_power_level *cp;
    struct bt_hci_rp_vs_write_tx_power_level *rp;
    struct net_buf *buf, *rsp = NULL;
    int err;

    buf = bt_hci_cmd_alloc(K_FOREVER);
    if (!buf)
    {
        LOG_WRN("Unable to allocate HCI command buffer for TX power set");
        return;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);
    cp->handle_type = handle_type;
    cp->tx_power_level = tx_pwr_lvl;

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &rsp);
    if (err)
    {
        LOG_WRN("Set Tx power err: %d", err);
        return;
    }

    rp = (void *)rsp->data;
    LOG_INF("Controller selected TX power = %d dBm", rp->selected_tx_power);

    net_buf_unref(rsp);
}

/*
 * Helper: read TX power using HCI vendor-specific read command
 */
static void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl)
{
    struct bt_hci_cp_vs_read_tx_power_level *cp;
    struct bt_hci_rp_vs_read_tx_power_level *rp;
    struct net_buf *buf, *rsp = NULL;
    int err;

    *tx_pwr_lvl = 0xFF;
    buf = bt_hci_cmd_alloc(K_FOREVER);
    if (!buf)
    {
        LOG_WRN("Unable to allocate HCI command buffer for TX power read");
        return;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);
    cp->handle_type = handle_type;

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL, buf, &rsp);
    if (err)
    {
        LOG_WRN("Read Tx power err: %d", err);
        return;
    }

    rp = (void *)rsp->data;
    *tx_pwr_lvl = rp->tx_power_level;

    net_buf_unref(rsp);
}
#else
/* Stub implementations - HCI command allocation not available */
static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
    ARG_UNUSED(handle_type);
    ARG_UNUSED(handle);
    LOG_INF("Runtime TX power control not available (bt_hci_cmd_alloc missing in build)");
    LOG_INF("  Requested: %d dBm (will use Kconfig default instead)", tx_pwr_lvl);
    LOG_INF("  To set TX power, configure CONFIG_BT_CTLR_TX_PWR_* in prj.conf");
}

static void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl)
{
    ARG_UNUSED(handle_type);
    ARG_UNUSED(handle);
    if (tx_pwr_lvl)
    {
        *tx_pwr_lvl = 0xFF;
    }
    LOG_DBG("Runtime TX power read not available (bt_hci_cmd_alloc missing in build)");
}
#endif

#if 0 /* Disabled: periodic monitor also needs HCI cmd alloc */
static struct k_thread tx_power_thread_data;
K_THREAD_STACK_DEFINE(tx_power_thread_stack, 512);

static void tx_power_monitor_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1)
    {
        int8_t txp = 0xFF;
        get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, &txp);
        if (txp == 0xFF)
        {
            LOG_WRN("Periodic TX power read: not available");
        }
        else
        {
            LOG_INF("Periodic TX power read: %d dBm", txp);
        }
        k_sleep(K_SECONDS(10));
    }
}
#endif

ble_manager_result_t ble_manager_init(void)
{
    int err;

    if (initialized)
    {
        LOG_WRN("BLE manager already initialized");
        return BLE_MANAGER_ALREADY_INITIALIZED;
    }

    LOG_INF("=== Initializing BLE Manager - Debug Info ===");

    /* Enable Bluetooth */
    LOG_INF("Step 1: Enabling Bluetooth stack...");
    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("!!! Bluetooth init failed !!!");
        LOG_ERR("  Error code: %d", err);
        return BLE_MANAGER_ERROR;
    }

    LOG_INF("✓ Bluetooth stack enabled");

    /* Check BT ready state */
    if (bt_is_ready())
    {
        LOG_INF("✓ BT stack is ready");
    }
    else
    {
        LOG_WRN("⚠ BT stack reports not ready (unexpected)");
    }

    /* Apply runtime device name (use config override if present). Must be done after bt_enable(). */
#ifdef CONFIG_BT
    {
        /* Immer den build-time Namen "Traumschreiber" advertisen — config.txt
         * BLE_DEVICE_NAME wird bewusst ignoriert, damit die App den TS zuverlässig
         * per Namen findet (der Scan-Filter matcht "traumschreiber"). */
        const char *desired = DEVICE_NAME;
        size_t max_name = (size_t)31; /* safe advertising limit for a field */
#ifdef CONFIG_BT_DEVICE_NAME_MAX
        if (CONFIG_BT_DEVICE_NAME_MAX > 1)
        {
            max_name = (size_t)(CONFIG_BT_DEVICE_NAME_MAX - 1);
        }
#endif
        /* Truncate to fit our runtime_name_buf */
        size_t copy_len = strlen(desired);
        if (copy_len > sizeof(runtime_name_buf) - 1)
            copy_len = sizeof(runtime_name_buf) - 1;
        if (copy_len > max_name)
            copy_len = max_name;
        memcpy(runtime_name_buf, desired, copy_len);
        runtime_name_buf[copy_len] = '\0';

        /* Update bt_data entry length to match name */
        sd[0].data_len = (uint8_t)copy_len;
        sd[0].data = runtime_name_buf;

        LOG_INF("Applying runtime BLE device name after bt_enable(): '%s' (len=%u)", runtime_name_buf, sd[0].data_len);

        int name_err = bt_set_name(runtime_name_buf);
        if (name_err)
        {
            LOG_WRN("bt_set_name failed (%d) for '%s' - will use build-time name", name_err, runtime_name_buf);
        }
        else
        {
            LOG_INF("Runtime device name applied: %s", runtime_name_buf);
        }

        /* Apply runtime TX power if configured using vendor HCI (recommended approach) */
#if defined(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL) && defined(BT_HCI_OP_VS_READ_TX_POWER_LEVEL)
        {
            /* Read current controller TX power (optional) and apply requested level */
            int8_t current_tx = 0xFF;
            get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, &current_tx);
            LOG_INF("Controller default TX power = %d dBm", current_tx);

            set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, g_config.tx_power_dbm);
        }
#else
        LOG_DBG("Runtime TX power set requested: %d dBm — HCI VS TX power ops not available in this build", g_config.tx_power_dbm);
#endif
    }
#endif

    /* Debug: Print device info */
    LOG_INF("Device information:");
#ifdef CONFIG_BT
    LOG_INF("  Device name: %s", (sd[0].data_len > 0) ? (const char *)sd[0].data : DEVICE_NAME);
#else
    LOG_INF("  Device name: %s (build-time)", DEVICE_NAME);
#endif

    /* Print BT address */
    bt_addr_le_t addr;
    size_t count = 1;
    bt_id_get(&addr, &count);
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
    LOG_INF("  BT Address: %s", addr_str);

    /* Initialize LBS service */
    LOG_INF("Step 2: Initializing LBS service...");
    struct lbs_cb lbs_callbacks = {
        .led_cb = lbs_led_callback,
        .rgb_led_cb = lbs_rgb_led_callback,
        .button_cb = lbs_button_callback,
        .play_file_cb = lbs_play_file_callback,
        .playback_ctrl_cb = lbs_playback_control_callback,
    };

    err = lbs_init(&lbs_callbacks);
    if (err)
    {
        LOG_ERR("!!! Failed to initialize LBS service !!!");
        LOG_ERR("  Error code: %d", err);
        return BLE_MANAGER_ERROR;
    }

    /* Initialize Battery Service (BAS) */
    LOG_INF("Step 3: Initializing Battery Service (BAS)...");
    err = bas_init();
    if (err)
    {
        LOG_ERR("!!! Failed to initialize BAS service !!!");
        LOG_ERR("  Error code: %d", err);
        return BLE_MANAGER_ERROR;
    }

    /* Initialize ADS Streaming Service */
    LOG_INF("Step 4: Initializing ADS Streaming Service...");
    err = ads_service_init();
    if (err)
    {
        LOG_ERR("!!! Failed to initialize ADS service !!!");
        LOG_ERR("  Error code: %d", err);
        return BLE_MANAGER_ERROR;
    }

    /* Initialize AIN BLE Service (periodic AIN notifications) */
    LOG_INF("Step 5: Initializing AIN BLE Service...");
    err = ain_service_init();
    if (err)
    {
        LOG_ERR("!!! Failed to initialize AIN service !!!");
        LOG_ERR("  Error code: %d", err);
        return BLE_MANAGER_ERROR;
    }

    /* Register button event handler for BLE notifications */
    LOG_INF("Step 5: Registering button callback...");
    button_register_secondary_callback(ble_button_event_handler);

    /* Note: Periodic TX power monitor disabled - bt_hci_cmd_alloc not available in this build */

    LOG_INF("✓ BLE services initialized successfully");
    LOG_INF("=== BLE Manager Ready ===");

    initialized = true;
    return BLE_MANAGER_OK;
}

ble_manager_result_t ble_manager_start_advertising(void)
{
    int err;

    if (!initialized)
    {
        LOG_ERR("BLE manager not initialized");
        return BLE_MANAGER_NOT_INITIALIZED;
    }

    if (!radio_enabled)
    {
        LOG_WRN("BLE radio disabled; advertising start skipped");
        return BLE_MANAGER_ERROR;
    }

    LOG_INF("=== Starting BLE Advertising - Debug Info ===");

    /* Validate and print advertising data structure */
    debug_print_adv_data(ad_flags, ARRAY_SIZE(ad_flags), sd, ARRAY_SIZE(sd));

    /* Debug: Print advertising data */
    LOG_INF("Advertising data array size: %d", ARRAY_SIZE(ad_flags));
    for (int i = 0; i < ARRAY_SIZE(ad_flags); i++)
    {
        LOG_INF("  AD[%d]: type=0x%02x, data_len=%d",
                i, ad_flags[i].type, ad_flags[i].data_len);
        if (ad_flags[i].type == BT_DATA_NAME_COMPLETE)
        {
            LOG_INF("    Device name: %.*s", ad_flags[i].data_len, ad_flags[i].data);
        }
    }

    /* Debug: Print scan response data */
    LOG_INF("Scan response data array size: %d", ARRAY_SIZE(sd));
    for (int i = 0; i < ARRAY_SIZE(sd); i++)
    {
        LOG_INF("  SD[%d]: type=0x%02x, data_len=%d",
                i, sd[i].type, sd[i].data_len);
    }

    /* Debug: Print advertising parameters */
    LOG_INF("Advertising parameters:");
    const uint16_t adv_int_min = (current_adv_mode == BLE_MANAGER_ADV_MODE_WORST_CASE) ? 0x0020 : BT_GAP_ADV_FAST_INT_MIN_2;
    const uint16_t adv_int_max = (current_adv_mode == BLE_MANAGER_ADV_MODE_WORST_CASE) ? 0x0020 : BT_GAP_ADV_FAST_INT_MAX_2;
    LOG_INF("  Interval min: %d (0x%04x)", adv_int_min, adv_int_min);
    LOG_INF("  Interval max: %d (0x%04x)", adv_int_max, adv_int_max);

    LOG_INF("Calling bt_le_adv_start()...");
    LOG_INF("  ad: %p, count: %d", ad_flags, ARRAY_SIZE(ad_flags));
    LOG_INF("  sd: %p, count: %d", sd, ARRAY_SIZE(sd));

    /* Use connectable advertising parameters (new API) */
    struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .secondary_max_skip = 0,
        .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min = adv_int_min,
        .interval_max = adv_int_max,
        .peer = NULL,
    };

    err = bt_le_adv_start(&adv_param, ad_flags, ARRAY_SIZE(ad_flags),
                          sd, ARRAY_SIZE(sd));
    if (err)
    {
        LOG_ERR("!!! Advertising failed to start !!!");
        LOG_ERR("  Error code: %d", err);
        /* detailed switch omitted for brevity in logging */

        /* Additional debug: check BT state */
        if (bt_is_ready())
        {
            LOG_INF("  BT stack reports: READY");
        }
        else
        {
            LOG_ERR("  BT stack reports: NOT READY!");
        }

        return BLE_MANAGER_ERROR;
    }

    LOG_INF("✓ Advertising started successfully!");
    LOG_INF("  Device name: %s", DEVICE_NAME);
    LOG_INF("  Service UUID: 00001523-1212-efde-1523-785feabcd123");

    /* Visual feedback: blink green while advertising */
    rgb_led_set_color(&RGB_COLOR_GREEN);

    /* Cancel any pending advertising restart retries */
    k_work_cancel_delayable(&advertising_restart_work);

    return BLE_MANAGER_OK;
}

ble_manager_result_t ble_manager_set_radio_enabled(bool enable)
{
    radio_enabled = enable;

    if (!initialized)
    {
        return BLE_MANAGER_NOT_INITIALIZED;
    }

    if (!enable)
    {
        k_work_cancel_delayable(&advertising_restart_work);
        if (current_conn)
        {
            (void)bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
        (void)ble_manager_stop_advertising();
        return BLE_MANAGER_OK;
    }

    if (!connected)
    {
        return ble_manager_start_advertising();
    }

    return BLE_MANAGER_OK;
}

bool ble_manager_is_radio_enabled(void)
{
    return radio_enabled;
}

ble_manager_result_t ble_manager_set_adv_mode(ble_manager_adv_mode_t mode)
{
    current_adv_mode = mode;

    if (!initialized)
    {
        return BLE_MANAGER_NOT_INITIALIZED;
    }

    if (!connected && radio_enabled)
    {
        (void)ble_manager_stop_advertising();
        return ble_manager_start_advertising();
    }

    return BLE_MANAGER_OK;
}

ble_manager_adv_mode_t ble_manager_get_adv_mode(void)
{
    return current_adv_mode;
}

ble_manager_result_t ble_manager_stop_advertising(void)
{
    int err;

    if (!initialized)
    {
        LOG_ERR("BLE manager not initialized");
        return BLE_MANAGER_NOT_INITIALIZED;
    }

    LOG_INF("Stopping advertising...");

    err = bt_le_adv_stop();
    if (err)
    {
        LOG_ERR("Advertising failed to stop (err %d)", err);
        return BLE_MANAGER_ERROR;
    }

    LOG_INF("Advertising stopped");
    return BLE_MANAGER_OK;
}

bool ble_manager_is_connected(void)
{
    return connected;
}

uint16_t ble_manager_get_mtu_size(void)
{
    if (!connected || current_conn == NULL)
    {
        return -1;
    }

    uint16_t mtu = bt_gatt_get_mtu(current_conn);
    if (mtu == 0)
        return -2;
    return mtu;
}

/**
 * @brief Button event handler - sends BLE notifications on button events
 */
static void ble_button_event_handler(button_event_t event)
{
    bool button_state = false;

    switch (event)
    {
    case BUTTON_EVENT_PRESSED:
        button_state = true;
        LOG_INF("Button pressed - sending BLE notification");
        break;

    case BUTTON_EVENT_RELEASED:
        button_state = false;
        LOG_INF("Button released - sending BLE notification");
        break;

    default:
        /* Don't send notifications for clicks, only press/release */
        return;
    }

    /* Send button state notification if connected */
    if (ble_manager_is_connected() && lbs_notify_enabled())
    {
        int err = lbs_send_button_state(button_state);
        if (err)
        {
            LOG_WRN("Failed to send button notification: %d", err);
        }
        else
        {
            LOG_DBG("Button notification sent: %s",
                    button_state ? "PRESSED" : "RELEASED");
        }
    }
}
