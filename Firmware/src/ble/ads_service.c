/*
 * ADS Streaming Service - Implementation
 *
 * High-speed BLE service for streaming ADS131M08 ADC data at 250-500 Hz.
 * Based on Nordic's Throughput sample best practices.
 *
 * @author ISDT
 * @date 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include "ads_service.h"
#include "subsystems/system_config.h"
#include "firmware_version.h"
#include "drivers/ads131m08_ti.h"
/* Required for writing uploaded config to SD */
#include "subsystems/data_logger.h"
#include "subsystems/noise_test.h"
#include <errno.h>
/* sys_reboot() and SYS_REBOOT_COLD */
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(ads_service, LOG_LEVEL_INF);

/* Buffering and batching configuration */
#define ADS_BUFFER_SECONDS 1
#define ADS_SAMPLE_RATE 250
#define ADS_QUEUE_SIZE (ADS_BUFFER_SECONDS * ADS_SAMPLE_RATE)

/* Timer period for checking/sending batches (milliseconds)
 * Lower values = more responsive to queue filling, higher CPU overhead
 * Recommended: 50-200ms for good balance
 */
#define ADS_BATCH_TIMER_MS 800

/* Live-notification batching latency bound (ms). A live sample enqueued while
 * the batch is still partial waits at most this long before being flushed, so
 * live latency is bounded even if the sample stream stalls mid-batch. 40ms =
 * 10 samples @250Hz = one full MTU-498 packet. The full-batch fast path (queue
 * reached the MTU-derived samples-per-packet) flushes earlier than this.
 *
 * WHY THIS EXISTS: the producer used to kick the notify work on EVERY sample
 * (K_NO_WAIT), so the work drained 1 sample/run → ~250 notifications/s (one
 * sample per BLE notification). At that rate a single dropped notification =
 * exactly one missing sample-number, and an overnight recording accumulated
 * ~33k such 1-sample gaps; recovering each via its own backfill request both
 * fragmented backfill (~6 samples/s drain) and, by hammering the GATT with
 * ~1 write/s for hours, eventually wedged the phone BLE stack (~8h in). Batching
 * to one packet per MTU-worth of samples cuts the notification rate ~10× and
 * makes any dropped-notification gap CONTIGUOUS (one backfill range, not ten
 * scattered 1-sample gaps). The packing machinery already existed in the work
 * handler; only the ACCUMULATION POLICY below changed. */
#define ADS_LIVE_FLUSH_MS 40

/* Maximum element size used for temporary buffers and drains.
 * Compression support is disabled, so the queue only needs full samples.
 */
#define ADS_RUNTIME_ELEM_MAX sizeof(ads_sample_data_t)

/* File-scope reusable temporary buffers to avoid large stack usage (declare early!).
 * Sized for the largest possible single-notification batch:
 *   max samples per packet = (MTU-1)/sample_size.
 * 10 samples/packet at 250 Hz = 25 notifications/s, well within BLE's
 * 60–130 packets/s capacity, so this never bottlenecks throughput.
 * (Reduced from 16 to reclaim RAM for the 24-bit BDF record buffers in
 * data_logger.c; any shortfall under congestion is recovered by SD backfill.) */
static ads_sample_data_t g_uncompressed_batch[10];
/* MTU=498 → max payload 495 B + 1 header = 496. 512 covers it with room to grow. */
static uint8_t g_notification_payload[512];

/* Bytes per channel sample sent over BLE. Switched from 4 (int32) to 3
 * (int24, two's-complement little-endian) to match the BDF on-disk format and
 * save ~25% bandwidth — the ADS131M08 only produces 24 significant bits, so
 * the top byte of the old int32 was always sign-extension. */
#define BLE_CH_BYTES 3

/* Write the low 24 bits of a channel value (int32 sign-extended 24-bit) as
 * 3 bytes little-endian into buf[off]. */
static inline void ble_put_i24(uint8_t *buf, size_t off, int32_t v)
{
    uint32_t u = (uint32_t)v;
    buf[off + 0] = (uint8_t)(u & 0xFF);
    buf[off + 1] = (uint8_t)((u >> 8) & 0xFF);
    buf[off + 2] = (uint8_t)((u >> 16) & 0xFF);
}

/* Config upload buffer (file-scope) */
#define CONFIG_UPLOAD_MAX_SIZE 2048
static uint8_t config_upload_buffer[CONFIG_UPLOAD_MAX_SIZE];
static size_t config_upload_len = 0;
static bool config_upload_active = false;

/* ── Backfill state ───────────────────────────────────────────────────────── */

/* Maximum samples packed into one backfill notification.
 * With 8 channels: one sample = 4+8+32+4 = 48 bytes.
 * At MTU=498 (payload 495) we fit 10 samples (10*48+1 = 481 bytes).
 * At MTU=252 (payload 249) we fit 5 samples — runtime spp cap handles that.
 * Raised 8 → 10 so high-MTU peers (the common case) get fewer packets per gap. */
#define BACKFILL_STAGING_SAMPLES 10

/* NEAR-HEAD GUARD (0.0.70): while logging, refuse/clamp backfill ranges
 * within this many samples (2 s @ 250 Hz) of the live sample counter — a
 * read at the write head wedged the whole SD subsystem in the field
 * (2026-07-16 08:02). See the guard in backfill_worker_thread_entry. */
#define BACKFILL_HEAD_GUARD_SAMPLES 500

/* Staging buffer: 1 header byte + up to BACKFILL_STAGING_SAMPLES full uncompressed records */
/* Sized to the BDF wire layout (not the in-RAM struct): header + N samples of
 * (sample_num 4 + ts 8 + 8×int24 channels + status 4) = 1 + N×40 bytes. */
static uint8_t g_backfill_staging_buf[1 + BACKFILL_STAGING_SAMPLES * (4 + 8 + 8 * BLE_CH_BYTES + 4)];

static bool backfill_notifications_enabled = false;
static volatile bool g_backfill_cancel = false;
static bool g_backfill_in_progress = false;

/* File-service request, dispatched on the SAME backfill worker (no new thread —
 * RAM is at ~100%). 0 = none (a plain EEG backfill), else one of the file ops.
 * The path lives in config_upload_buffer (idle during a browse); offset/len
 * carry the READ window. ~9 bytes of new static RAM total. */
enum { FILE_OP_NONE = 0, FILE_OP_LIST = 1, FILE_OP_READ = 2 };
static volatile uint8_t g_file_op = FILE_OP_NONE;
static uint32_t g_file_offset = 0;
static uint32_t g_file_len = 0;
static volatile bool g_file_abort = false;

/* Wall-time sync (0x40, fw 0.0.64): the clock reference is stored directly in
 * ads_control_write (cheap, no I/O); this flag defers the SD-side work (file
 * rename + BDF header patch) to the backfill worker, which serializes it
 * against range reads and file ops — FatFS must never rename a file another
 * handle has open. 1 byte of new static RAM. */
static volatile bool g_time_sync_pending = false;

/* Phase 3 (firmware-v0.0.48): when set, the live-notification work handler
 * skips its fetch/notify cycle so the backfill worker gets the full BLE TX
 * queue. ADS sampling and SD writes are unaffected; the app can later
 * backfill the pause window from SD. Toggled by control bytes 0x22/0x23. */
static volatile bool g_live_notifications_paused = false;

/* v0.0.51 retry stash: when bt_gatt_notify() returns -ENOMEM, the live worker
 * used to DROP the fetched batch (see comment around line 797). Empirical
 * measurement on the bench DK showed that under steady background load — no
 * Faraday cage, no out-of-range — a small ENOMEM burst every 5–60 s caused
 * 1–7 samples to be lost at the producer→notify boundary; the app then saw a
 * gap and asked for backfill, and backfill notifications themselves filled
 * the TX queue enough to trigger MORE ENOMEM in the live worker, kicking off
 * a feedback cascade (worst observed: 31 lost samples in one second).
 *
 * Fix: cache the failed batch and replay it on the next worker run before
 * fetching from the producer queue. Stash holds exactly one batch; if the
 * NEXT call also fails, fall back to the old drop behaviour so an indefinite
 * BLE link breakage can't wedge the worker. Order is preserved because the
 * stash always replays before any new fetch. */
static ads_sample_data_t g_live_retry_stash[ARRAY_SIZE(g_uncompressed_batch)];
static size_t g_live_retry_stash_count = 0;

/* Sample count at which we last synced SD before a backfill range.
 * If the live sample count hasn't advanced since, we can skip the 300ms sync. */
static uint32_t g_backfill_last_synced_sample = 0;

/* ── DEBUG instrumentation ────────────────────────────────────────────────
 * Per-second STATS aggregation for diagnosing -ENOMEM bursts and producer
 * stalls. All fields are updated in the BLE notification work handler or the
 * producer-side ads_service_send_sample(); both run in cooperative contexts
 * (work / driver callback) on this MCU, so non-atomic uint32_t is fine.
 */
static uint32_t g_dbg_last_samples_sent_snapshot = 0;
static uint32_t g_dbg_last_samples_dropped_snapshot = 0;
static uint32_t g_dbg_notifs_sent_1s = 0;
static uint32_t g_dbg_enomem_events_1s = 0;
static uint32_t g_dbg_max_consecutive_enomem_1s = 0;
static uint32_t g_dbg_queue_max_1s = 0;
static uint32_t g_dbg_max_enqueue_gap_ms_1s = 0;
static int64_t  g_dbg_last_enqueue_ms = 0;
static int64_t  g_dbg_last_stats_emit_ms = 0;

typedef struct
{
    uint32_t start_sample_num;
    uint32_t end_sample_num;
    uint32_t conn_epoch; /* g_conn_epoch when the request arrived */
} backfill_req_t;

static backfill_req_t g_backfill_req;
static struct k_sem g_backfill_sem;

/* Connection epoch (0.0.92): bumped on every connect AND disconnect. A backfill
 * request belongs to the connection it arrived on. Before this, a large range
 * running when the link dropped kept reading SD (cancel only muted the sends),
 * and the next request's handler cleared g_backfill_cancel — so the OLD range
 * resumed streaming on the NEW connection and its terminal answered the new
 * request; every later terminal was off by one (RTT + PC client 2026-10-03:
 * 7902 samples of a stale range after reconnect, app saw no answer to its own
 * request = field 28.09. 04:57). The worker now drops requests and aborts reads
 * whose epoch is no longer current, and never sends their terminal. */
static volatile uint32_t g_conn_epoch = 0;
static uint32_t g_backfill_run_epoch = 0;

/* Worker thread for sending backfill data to the BLE client */
#define BACKFILL_THREAD_STACK_SIZE 3072
#define BACKFILL_THREAD_PRIORITY   15
K_THREAD_STACK_DEFINE(backfill_thread_stack, BACKFILL_THREAD_STACK_SIZE);
static struct k_thread backfill_thread;

/* Forward declarations for backfill implementation */
static void backfill_worker_thread_entry(void *p1, void *p2, void *p3);
static void ads_backfill_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

/* Runtime queue backing buffer and helpers (use static BSS buffer to avoid
 * runtime heap allocations which may fail on constrained systems). */
static struct k_msgq ads_runtime_queue;
static size_t ads_runtime_element_size = 0;
static bool ads_runtime_queue_initialized = false; /* new flag */

/* Static backing buffer for the message queue. Size is ADS_QUEUE_SIZE * ADS_RUNTIME_ELEM_MAX */
/* Note: Put the buffer into .noinit or normal BSS depending on link; default BSS is fine. */
enum
{
    ADS_RUNTIME_BUFFER_BYTES = (ADS_QUEUE_SIZE * ADS_RUNTIME_ELEM_MAX)
};
static uint8_t ads_runtime_queue_buffer_static[ADS_RUNTIME_BUFFER_BYTES];

static int ads_runtime_queue_init(void)
{
    ads_runtime_element_size = ADS_RUNTIME_ELEM_MAX;

    /* Sanity check: ensure element size not zero and buffer large enough */
    if (ads_runtime_element_size == 0 || (sizeof(ads_runtime_queue_buffer_static) < (ADS_QUEUE_SIZE * ads_runtime_element_size)))
    {
        LOG_ERR("ads_runtime_queue_init: static buffer size mismatch (%u bytes)", (unsigned)sizeof(ads_runtime_queue_buffer_static));
        return -ENOMEM;
    }

    k_msgq_init(&ads_runtime_queue, ads_runtime_queue_buffer_static, ads_runtime_element_size, ADS_QUEUE_SIZE);
    ads_runtime_queue_initialized = true;
    return 0;
}

static void ads_runtime_queue_deinit(void)
{
    /* Static buffer: nothing to free */
    ads_runtime_queue_initialized = false;
}

/* Work item for sending notifications (use delayable work for backoff on -ENOMEM)
 * This existing work handler is reused as the batching sender. It will be scheduled
 * periodically by a k_timer (e.g., every 1 second) instead of on every sample enqueue.
 */
static void ads_notification_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(ads_notification_work, ads_notification_work_handler);

/* Delayed BLE parameter update work — forward declaration (defined near on_connected) */
static void ble_param_update_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ble_param_update_work, ble_param_update_work_handler);

/* Timer that triggers periodic batching send (ADS_BATCH_TIMER_MS period). The timer's expiry
 * handler simply schedules the ads_notification_work. Timer is started when streaming
 * is active and stopped on STOP_STREAMING or disconnect.
 */
static struct k_timer ads_batch_timer;

/* Work item for handling config upload finalization (runs in application context) */
static void config_upload_work_handler(struct k_work *work);
K_WORK_DEFINE(config_upload_work, config_upload_work_handler);

static void ads_batch_timer_expiry(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    /* Schedule the notification work to run immediately */
    k_work_schedule(&ads_notification_work, K_NO_WAIT);
}

/* Current connection and streaming state */
static struct bt_conn *current_conn = NULL;
static bool notifications_enabled = false;
static bool streaming_active = false;

/* Drop everything the live path still holds from an earlier stream: the
 * producer queue AND the -ENOMEM retry stash (0.0.92). The stash used to
 * survive a disconnect — on 2026-09-28 it delivered a batch from 07:30 right
 * after a START at 09:28 and set the new session's time base ~2 h wrong. */
static void ads_live_discard_pending(void)
{
    g_live_retry_stash_count = 0;
    if (ads_runtime_queue_initialized)
    {
        uint8_t temp[ADS_RUNTIME_ELEM_MAX];
        while (k_msgq_get(&ads_runtime_queue, temp, K_NO_WAIT) == 0)
        {
            /* drain */
        }
    }
}
static bool forced_streaming = false;

/* Statistics */
static ads_status_t current_status = {
    .is_streaming = false,
    .samples_sent = 0,
    .samples_dropped = 0,
    .notification_errors = 0};

static void ads_update_streaming_timer_state(void)
{
    const bool should_run = notifications_enabled && current_conn != NULL && (streaming_active || forced_streaming);

    current_status.is_streaming = should_run;

    if (should_run)
    {
        if (!ads_runtime_queue_initialized)
        {
            (void)ads_runtime_queue_init();
        }
        k_timer_start(&ads_batch_timer, K_MSEC(ADS_BATCH_TIMER_MS), K_MSEC(ADS_BATCH_TIMER_MS));
        return;
    }

    k_timer_stop(&ads_batch_timer);
    k_work_cancel_delayable(&ads_notification_work);
}

/* MTU exchange parameters */
#if IS_ENABLED(CONFIG_BT_GATT_EXCHANGE)
static struct bt_gatt_exchange_params mtu_exchange_params;

/**
 * @brief MTU exchange completed callback
 */
static void mtu_exchange_callback(struct bt_conn *conn, uint8_t err,
                                  struct bt_gatt_exchange_params *params)
{
    if (err)
    {
        LOG_ERR("MTU exchange failed (err %u)", err);
        return;
    }

    uint16_t mtu = bt_gatt_get_mtu(conn);
    LOG_INF("MTU exchange successful: %u bytes", mtu);
}
#endif

/* Data length parameters (from Throughput sample) */
static const struct bt_conn_le_data_len_param data_len_params = {
    .tx_max_len = 251,  /* Maximum data length */
    .tx_max_time = 2120 /* Maximum time in microseconds */
};

/* PHY parameters (from Throughput sample) - prefer 2M for double speed */
static const struct bt_conn_le_phy_param phy_params = {
    .options = BT_CONN_LE_PHY_OPT_NONE,
    .pref_tx_phy = BT_GAP_LE_PHY_2M,
    .pref_rx_phy = BT_GAP_LE_PHY_2M};

/* Connection parameters optimized for high-speed streaming
 * For 250 Hz @ 8-sample batching = 31 notifications/sec
 * Need short interval for frequent transmission opportunities
 */
static const struct bt_le_conn_param conn_params = {
    .interval_min = 40, /* 40 * 1.25ms = 50ms - energy-optimized */
    .interval_max = 40, /* 40 * 1.25ms = 50ms */
    .latency = 3,       /* Allow skipping up to 3 events -> sleep up to 150ms */
    .timeout = 400      /* 4000ms */
};

/* Forward declarations */
static ssize_t ads_control_write(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len,
                                 uint16_t offset, uint8_t flags);

static ssize_t ads_status_read(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset);

static ssize_t ads_fw_version_read(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);

static void ads_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void ads_backfill_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

/* Forward prototypes for runtime queue helpers (avoid implicit decl warnings) */
static int ads_runtime_queue_init(void);
static void ads_runtime_queue_deinit(void);

/* ADS Service Definition
 * Attribute index map (used by bt_gatt_notify):
 *   [0]  Primary Service declaration
 *   [1]  ADS Data char declaration   ← live stream uses attrs[1] (Zephyr auto-adjusts to value)
 *   [2]  ADS Data char value
 *   [3]  ADS Data CCC
 *   [4]  ADS Control char declaration
 *   [5]  ADS Control char value
 *   [6]  ADS Status char declaration
 *   [7]  ADS Status char value
 *   [8]  ADS Backfill char declaration ← backfill uses attrs[8]
 *   [9]  ADS Backfill char value
 *   [10] ADS Backfill CCC
 */
BT_GATT_SERVICE_DEFINE(ads_service,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_ADS_SERVICE),

                       /* ADS Data Characteristic - for streaming samples */
                       BT_GATT_CHARACTERISTIC(BT_UUID_ADS_DATA,
                                              BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE,
                                              NULL, NULL, NULL),
                       BT_GATT_CCC(ads_data_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* ADS Control Characteristic - for start/stop commands */
                       BT_GATT_CHARACTERISTIC(BT_UUID_ADS_CONTROL,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, ads_control_write, NULL),

                       /* ADS Status Characteristic - for reading statistics */
                       BT_GATT_CHARACTERISTIC(BT_UUID_ADS_STATUS,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              ads_status_read, NULL, NULL),

                       /* ADS Backfill Characteristic - for SD gap-fill retransmission */
                       BT_GATT_CHARACTERISTIC(BT_UUID_ADS_BACKFILL,
                                              BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE,
                                              NULL, NULL, NULL),
                       BT_GATT_CCC(ads_backfill_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* Firmware Version Characteristic - read-only, ASCII */
                       BT_GATT_CHARACTERISTIC(BT_UUID_ADS_FW_VERSION,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              ads_fw_version_read, NULL, NULL), );

/**
 * @brief CCC descriptor changed callback
 *
 * Called when client enables/disables notifications.
 */
static void ads_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notifications_enabled = (value == BT_GATT_CCC_NOTIFY);

    LOG_INF("ADS Data notifications %s",
            notifications_enabled ? "enabled" : "disabled");

    if (!notifications_enabled)
    {
        streaming_active = false;
        current_status.is_streaming = false;

        /* Clear queue + retry stash when notifications are disabled */
        ads_live_discard_pending();
    }
}

static void ads_backfill_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    backfill_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("ADS Backfill notifications %s",
            backfill_notifications_enabled ? "enabled" : "disabled");

    if (!backfill_notifications_enabled)
    {
        g_backfill_cancel = true;
        g_backfill_in_progress = false;
    }
}

/**
 * @brief Control characteristic write callback
 *
 * Handles start/stop streaming commands.
 */
static ssize_t ads_control_write(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len,
                                 uint16_t offset, uint8_t flags)
{
    if (offset != 0)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len < 1)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = buf;
    uint8_t cmd = data[0];

    LOG_INF("ADS Control command received: 0x%02X", cmd);

    switch (cmd)
    {
    case ADS_CONTROL_START_STREAMING:
        if (!notifications_enabled)
        {
            LOG_WRN("Cannot start streaming: notifications not enabled");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }

        if (!streaming_active)
        {
            /* Fresh stream: nothing queued before this START may go out
             * (START itself stays idempotent while already streaming — the
             * app re-sends it as a nudge). */
            ads_live_discard_pending();
        }
        streaming_active = true;
        LOG_INF("ADS streaming started");
        ads_update_streaming_timer_state();
        break;

    case ADS_CONTROL_STOP_STREAMING:
        streaming_active = false;
        LOG_INF("ADS streaming stopped");
        ads_update_streaming_timer_state();
        break;

    case ADS_CONTROL_UPLOAD_BEGIN:
        /* Start a new config upload session */
        config_upload_len = 0;
        config_upload_active = true;
        LOG_INF("Config upload begun via BLE (max %u bytes)", (unsigned)CONFIG_UPLOAD_MAX_SIZE);
        break;

    case ADS_CONTROL_UPLOAD_CHUNK:
        /* Append chunk bytes (data[1..len-1]) to upload buffer */
        if (!config_upload_active)
        {
            LOG_WRN("Upload chunk received without BEGIN");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }
        if (len <= 1)
        {
            LOG_DBG("Empty upload chunk received");
            break;
        }

        if (config_upload_len + (len - 1) > CONFIG_UPLOAD_MAX_SIZE)
        {
            LOG_ERR("Config upload too large: %u + %u > %u", (unsigned)config_upload_len, (unsigned)(len - 1), (unsigned)CONFIG_UPLOAD_MAX_SIZE);
            config_upload_active = false;
            return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
        }

        memcpy(&config_upload_buffer[config_upload_len], &data[1], len - 1);
        config_upload_len += (len - 1);
        LOG_DBG("Received upload chunk: %u bytes (total %u)", (unsigned)(len - 1), (unsigned)config_upload_len);
        break;

    case ADS_CONTROL_UPLOAD_END:
        if (!config_upload_active)
        {
            LOG_WRN("Upload END received without BEGIN");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }

        if (config_upload_len == 0)
        {
            LOG_WRN("Upload END received but no data was provided");
            config_upload_active = false;
            break;
        }

        if (config_upload_len >= CONFIG_UPLOAD_MAX_SIZE)
        {
            LOG_ERR("Upload size invalid or overflow");
            config_upload_active = false;
            return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
        }

        /* Defer the actual SD write and reboot to a work handler running in application context */
        LOG_INF("Upload END: scheduling config write work for %u bytes", (unsigned)config_upload_len);
        config_upload_active = false;
        k_work_submit(&config_upload_work);
        break;

    case ADS_CONTROL_GET_STATUS:
        /* Status is read via Status characteristic, not control */
        LOG_DBG("Get status command (use Status characteristic instead)");
        break;

    case ADS_CONTROL_BACKFILL_REQUEST:
        /* Payload: [0]=0x20, [1..4]=start_sample_num (LE), [5..8]=end_sample_num (LE) */
        if (len < 9)
        {
            LOG_WRN("BACKFILL_REQUEST too short: %u bytes (need 9)", len);
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }
        if (!backfill_notifications_enabled)
        {
            LOG_WRN("Backfill requested but client has not subscribed to Backfill notifications");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }
        memcpy(&g_backfill_req.start_sample_num, &data[1], 4);
        memcpy(&g_backfill_req.end_sample_num, &data[5], 4);
        g_backfill_req.conn_epoch = g_conn_epoch;
        g_backfill_cancel = false;
        LOG_INF("Backfill request: samples [%u .. %u]",
                g_backfill_req.start_sample_num, g_backfill_req.end_sample_num);
        /* NOTE: do NOT request a tighter CI here — fragmenting radio time
         * starves the live stream notification path, which then drops samples
         * and creates *more* gaps than backfill can clear. Keep 50ms CI. */
        k_sem_give(&g_backfill_sem);
        break;

    case ADS_CONTROL_BACKFILL_CANCEL:
        g_backfill_cancel = true;
        LOG_INF("Backfill cancel requested");
        break;

    case ADS_CONTROL_PAUSE_LIVE:
        /* Pause live notifications so backfill gets the whole BLE pipe.
         * ADS sampling and SD writes continue; only the live-stream
         * notification work is gated. Effective immediately on the next
         * work-handler tick. */
        g_live_notifications_paused = true;
        LOG_INF("Live notifications PAUSED (backfill gets full BLE link)");
        break;

    case ADS_CONTROL_RESUME_LIVE:
        g_live_notifications_paused = false;
        /* Kick the work handler immediately so the resume isn't delayed by
         * the 50ms reschedule the pause path uses. */
        k_work_reschedule(&ads_notification_work, K_NO_WAIT);
        LOG_INF("Live notifications RESUMED");
        break;

    case ADS_CONTROL_FILE_LIST:
        /* [0]=0x30, [1..]=path (ASCII). Results stream on the Backfill char. */
        if (!backfill_notifications_enabled)
        {
            LOG_WRN("FILE_LIST but Backfill notifications not subscribed");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }
        if (g_file_op != FILE_OP_NONE || g_backfill_in_progress)
        {
            LOG_WRN("FILE_LIST rejected: file/backfill op already in progress");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }
        {
            size_t plen = (len > 1) ? (size_t)(len - 1) : 0;
            if (plen >= CONFIG_UPLOAD_MAX_SIZE)
            {
                return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
            }
            if (plen > 0)
            {
                memcpy(config_upload_buffer, &data[1], plen);
            }
            config_upload_buffer[plen] = '\0';
        }
        g_file_abort = false;
        g_file_op = FILE_OP_LIST;
        k_sem_give(&g_backfill_sem);
        LOG_INF("FILE_LIST queued: '%s'", (const char *)config_upload_buffer);
        break;

    case ADS_CONTROL_FILE_READ:
        /* [0]=0x31, [1..4]=offset LE, [5..8]=len LE, [9..]=path (ASCII). */
        if (len < 9)
        {
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }
        if (!backfill_notifications_enabled)
        {
            LOG_WRN("FILE_READ but Backfill notifications not subscribed");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }
        if (g_file_op != FILE_OP_NONE || g_backfill_in_progress)
        {
            LOG_WRN("FILE_READ rejected: file/backfill op already in progress");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }
        {
            size_t plen = (size_t)(len - 9);
            if (plen >= CONFIG_UPLOAD_MAX_SIZE)
            {
                return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
            }
            memcpy(&g_file_offset, &data[1], 4);
            memcpy(&g_file_len, &data[5], 4);
            if (plen > 0)
            {
                memcpy(config_upload_buffer, &data[9], plen);
            }
            config_upload_buffer[plen] = '\0';
        }
        g_file_abort = false;
        g_file_op = FILE_OP_READ;
        k_sem_give(&g_backfill_sem);
        LOG_INF("FILE_READ queued: '%s' off=%u len=%u",
                (const char *)config_upload_buffer, g_file_offset, g_file_len);
        break;

    case ADS_CONTROL_FILE_ABORT:
        /* Cancel an in-progress list/read (worker cbs poll g_file_abort). */
        g_file_abort = true;
        LOG_INF("FILE_ABORT requested");
        break;

    case ADS_CONTROL_TIME_SYNC:
        /* [0]=0x40, [1]=yy (year-2000), [2]=MM, [3]=dd, [4]=hh, [5]=mm,
         * [6]=ss — the phone's LOCAL wall time. Store the clock reference
         * immediately; the rename/header work runs on the backfill worker.
         * NOTE: g_backfill_sem coalesces (max count 1) — the app sends 0x40
         * BEFORE its first 0x20 of a connection, and its idle watchdog
         * re-requests any chunk whose trigger were ever swallowed. */
        if (len < 7)
        {
            LOG_WRN("TIME_SYNC too short: %u bytes (need 7)", len);
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }
        {
            data_logger_wall_time_t wt = {
                .year   = (uint16_t)(2000U + data[1]),
                .month  = data[2],
                .day    = data[3],
                .hour   = data[4],
                .minute = data[5],
                .second = data[6],
            };
            data_logger_set_wall_time(&wt);
        }
        g_time_sync_pending = true;
        k_sem_give(&g_backfill_sem);
        break;

    case ADS_CONTROL_NOISE_TEST:
    {
        uint16_t secs = 0;
        if (len >= 3)
        {
            secs = (uint16_t)(data[1] | ((uint16_t)data[2] << 8));
        }
        if (secs == 0)
        {
            secs = g_config.noise_test_secs ? g_config.noise_test_secs : 10;
        }
        int nt = noise_test_start(secs, "ble");
        if (nt == -EBUSY)
        {
            LOG_WRN("Noise test already running");
            return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
        }
        if (nt != 0)
        {
            LOG_WRN("Noise test start failed: %d", nt);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }
        break;
    }

    default:
        LOG_WRN("Unknown ADS control command: 0x%02X", cmd);
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
    }

    return len;
}

static void config_upload_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (config_upload_len == 0)
    {
        LOG_WRN("Config upload work ran with empty buffer");
        return;
    }

    /* Falls gerade aufgenommen wird: BDF-Writer pausieren, damit der config.txt-
     * Write die SD sauber bekommt. Danach folgt ohnehin der Reboot. */
    data_logger_pause_ads131_logging(true);

    data_logger_result_t res = data_logger_create_file("config.txt",
                                                       (const char *)config_upload_buffer,
                                                       config_upload_len);
    if (res != DATA_LOGGER_OK)
    {
        LOG_ERR("Failed to write uploaded config.txt: %d", res);
        return;
    }

    LOG_INF("Uploaded config.txt written successfully (%u bytes), rebooting",
            (unsigned)config_upload_len);
    sys_reboot(SYS_REBOOT_COLD);
}

/**
 * @brief Status characteristic read callback
 *
 * Returns current streaming statistics.
 */
/* Read handler for the Firmware Version characteristic. Returns the version
 * string (no null terminator on the wire); the client treats it as UTF-8. */
static ssize_t ads_fw_version_read(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset)
{
    ARG_UNUSED(conn);
    const char *v = FIRMWARE_VERSION_STR;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, v, strlen(v));
}

static ssize_t ads_status_read(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    /* Pack status into buffer */
    /* Compose extended status: base(13 bytes) + mask(1) + count(1) + channels[count] + compression_off(1)
     * Layout:
     * [0] is_streaming (1)
     * [1..4] samples_sent (4)
     * [5..8] samples_dropped (4)
     * [9..12] notification_errors (4)
     * [13] ble_transfer_channel_mask (1)
     * [14] ble_transfer_channel_count (1)
     * [15..15+count-1] channel indices (count bytes)
    * [15+count] compression_enabled flag (always 0)
     */
    uint8_t status_buf[64];
    size_t pos = 0;
    status_buf[pos++] = current_status.is_streaming ? 1 : 0;
    memcpy(&status_buf[pos], &current_status.samples_sent, 4);
    pos += 4;
    memcpy(&status_buf[pos], &current_status.samples_dropped, 4);
    pos += 4;
    memcpy(&status_buf[pos], &current_status.notification_errors, 4);
    pos += 4;

    /* Append channel configuration from g_config */
    status_buf[pos++] = g_config.ble_transfer_channel_mask;
    status_buf[pos++] = g_config.ble_transfer_channel_count;
    for (int i = 0; i < g_config.ble_transfer_channel_count && pos < (int)sizeof(status_buf); i++)
    {
        status_buf[pos++] = g_config.ble_transfer_channels[i];
    }

    /* Compression is disabled in this build; keep a stable status byte for clients. */
    status_buf[pos++] = 0;

    /* Keep delta_max field at zero for backwards-compatible status payload size. */
    if ((pos + 4) <= (int)sizeof(status_buf))
    {
        int32_t delta_max_le = 0;
        memcpy(&status_buf[pos], &delta_max_le, 4);
        pos += 4;
    }

    /* Append hardware ADC sample counter — keeps incrementing even during BLE disconnect,
     * so the app can detect the exact gap and request the right backfill range. */
    if ((pos + 4) <= (int)sizeof(status_buf))
    {
        uint32_t sample_count = ads131m08_ti_get_sample_count();
        memcpy(&status_buf[pos], &sample_count, 4);
        pos += 4;
    }

    /* v0.0.53: SD-writer health summary so the app can warn the user mid-
     * recording if writes have started to fail (the overnight-7.25h-then-
     * silently-dropped-2.5h symptom). 4 bytes, only emitted if the status
     * buffer has room — older clients ignoring the tail still work. */
    if ((pos + 4) <= (int)sizeof(status_buf))
    {
        uint32_t consec = 0, remounts_a = 0, remounts_s = 0, drops = 0,
                 last_ms = 0;
        bool unhealthy = false;
        data_logger_get_sd_writer_health(&last_ms, &consec, &remounts_a,
                                         &remounts_s, &drops, &unhealthy);
        status_buf[pos++] = unhealthy ? 1 : 0;
        status_buf[pos++] = (uint8_t)(consec > 255 ? 255 : consec);
        uint16_t remounts_attempted_clipped =
            (uint16_t)(remounts_a > 0xFFFF ? 0xFFFF : remounts_a);
        memcpy(&status_buf[pos], &remounts_attempted_clipped, 2);
        pos += 2;
    }

    /* Active ADS PGA gain (×N multiplier) + BLE channel sample width in bytes,
     * so the app can convert raw counts to µV correctly and parse the
     * 24-bit sample stream. Appended at the tail → older clients ignore it. */
    if ((pos + 2) <= (int)sizeof(status_buf))
    {
        status_buf[pos++] = g_config.ads_gain;   /* 1,2,4,...,128 */
        status_buf[pos++] = BLE_CH_BYTES;          /* 3 (int24) */
    }

    /* fw 0.0.91: per-channel gain (ADS_GAIN_CH0..7 overrides ADS_GAIN) — 8
     * bytes ×N — then the noise-test state (0 idle, 1 running, 2 done,
     * 3 failed) and its sequence counter (+1 per finished measurement). Tail
     * again, so older clients ignore it; they fall back to the global gain
     * byte above, which is correct as long as no per-channel gain is set. */
    if ((pos + 10) <= (int)sizeof(status_buf))
    {
        for (int ch = 0; ch < 8; ch++)
        {
            status_buf[pos++] = system_config_ads_gain_for_channel(ch);
        }
        status_buf[pos++] = (uint8_t)noise_test_get_state();
        status_buf[pos++] = noise_test_get_seq();
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             status_buf, pos);
}

/**
 * @brief PHY update completed callback
 */
static void phy_update_callback(struct bt_conn *conn,
                                struct bt_conn_le_phy_info *param)
{
    LOG_INF("PHY updated: TX=%u, RX=%u", param->tx_phy, param->rx_phy);
}

/**
 * @brief Data length update completed callback
 */
static void data_len_update_callback(struct bt_conn *conn,
                                     struct bt_conn_le_data_len_info *info)
{
    LOG_INF("Data length updated: TX len=%u time=%u, RX len=%u time=%u",
            info->tx_max_len, info->tx_max_time,
            info->rx_max_len, info->rx_max_time);
}

/**
 * @brief Connection parameters updated callback
 */
static void conn_param_update_callback(struct bt_conn *conn,
                                       uint16_t interval,
                                       uint16_t latency,
                                       uint16_t timeout)
{
    /* Avoid floating point formatting in LOG macros. Compute ms as fixed-point (interval * 1.25 ms)
     * Represent as whole_ms.msec (two decimal digits) using integer arithmetic.
     */
    uint32_t ms_times_100 = (uint32_t)interval * 125U; /* interval * 1.25 * 100 */
    uint32_t ms_whole = ms_times_100 / 100U;
    uint32_t ms_frac = ms_times_100 % 100U;

    LOG_INF("Connection parameters updated: interval=%u (%u.%02u ms), latency=%u, timeout=%u",
            interval, ms_whole, ms_frac, latency, timeout);
}

static void ads_notification_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Phase 3 pause-live gate. When the app sends 0x22 we silently skip
     * the fetch/notify cycle and reschedule cheaply, freeing the entire BLE
     * TX queue for the backfill worker. Producer (ADC ISR) keeps enqueuing
     * into ads_runtime_queue; backpressure drains the queue if it overflows
     * but the SD path is the durable record either way.
     *
     * Heartbeat (v0.0.50): emit a 1-byte 0xC0 packet on the live attr every
     * ~1 s so that flutter_reactive_ble on the phone sees per-characteristic
     * notification activity. Without this, the previous pause-live attempt
     * triggered Android-side disconnects when the live characteristic stayed
     * silent for > ~5 s while only the backfill characteristic was active.
     * The app's existing 0xC0 parser treats packets shorter than one sample
     * worth as a noop (sample loop bounded by `pos + block <= len`), so a
     * 1-byte heartbeat is consumed without producing spurious data. */
    if (g_live_notifications_paused)
    {
        static int64_t last_heartbeat_ms;
        const int64_t now_ms = k_uptime_get();
        if (current_conn && notifications_enabled &&
            (last_heartbeat_ms == 0 || (now_ms - last_heartbeat_ms) >= 1000))
        {
            uint8_t hb = 0xC0;
            int err = bt_gatt_notify(current_conn, &ads_service.attrs[1], &hb, 1);
            if (err == 0 || err == -ENOMEM)
            {
                /* Even on -ENOMEM, mark the attempt — we'll retry next tick. */
                last_heartbeat_ms = now_ms;
            }
        }
        k_work_reschedule(&ads_notification_work, K_MSEC(100));
        return;
    }

    /* Was 3 — at 1ms reschedule × 3 notifs per run we yielded back to the
     * workqueue 4× per BLE-buffer-fill cycle, which left the LL drain time
     * underutilised. Raised to 12 so one invocation can flush a full burst.
     * Bounded by ADS_QUEUE_SIZE/sample_size so it can never exceed the buffer
     * count; backpressure still works via -ENOMEM. */
    const int max_notifications_per_run = 12;
    int notifications_sent = 0;
    static uint8_t consecutive_enomem = 0; /* track consecutive -ENOMEM errors */

    while (notifications_sent < max_notifications_per_run)
    {
        if (!current_conn || !notifications_enabled || !streaming_active)
        {
            /* Stop processing if not ready */
            consecutive_enomem = 0;
            break;
        }

        int result = 0;

        /* Process uncompressed items from unified runtime queue */
        size_t sel_count = g_config.ble_transfer_channel_count;
        size_t sample_size = 16 + sel_count * BLE_CH_BYTES; /* sample_num(4)+timestamp(8)+status(4) + N*3 */

        uint16_t mtu = bt_gatt_get_mtu(current_conn);
        size_t payload_max = (mtu > 3) ? (size_t)(mtu - 3) : 20;
        size_t avail_for_samples = (payload_max > 1) ? (payload_max - 1) : 0;
        size_t max_samples_per_packet = (avail_for_samples > 0) ? (avail_for_samples / sample_size) : 0;
        if (max_samples_per_packet == 0)
            max_samples_per_packet = 1;
        size_t batch_cap = max_samples_per_packet > ARRAY_SIZE(g_uncompressed_batch) ? ARRAY_SIZE(g_uncompressed_batch) : max_samples_per_packet;

        size_t fetched = 0;
        bool from_stash = false;
        if (g_live_retry_stash_count > 0)
        {
            /* Backstop: never replay a stash that is far behind the ADC
             * counter (> 10 s) — it belongs to an earlier stream. */
            const uint32_t head = ads131m08_ti_get_sample_count();
            if (head - g_live_retry_stash[0].sample_num > 2500u)
            {
                LOG_WRN("Live: discarding stale retry stash (sn %u, head %u)",
                        g_live_retry_stash[0].sample_num, head);
                g_live_retry_stash_count = 0;
                continue;
            }
        }
        if (g_live_retry_stash_count > 0)
        {
            /* Replay the previous batch that hit -ENOMEM. Preserves sample
             * order: stash is always drained before any new queue read. */
            fetched = g_live_retry_stash_count;
            if (fetched > batch_cap) fetched = batch_cap;
            memcpy(g_uncompressed_batch, g_live_retry_stash,
                   fetched * sizeof(g_uncompressed_batch[0]));
            from_stash = true;
        }
        else
        {
            for (size_t i = 0; i < batch_cap; i++)
            {
                if (k_msgq_get(&ads_runtime_queue, &g_uncompressed_batch[i], K_NO_WAIT) != 0)
                    break;
                fetched++;
            }
        }

        if (fetched == 0)
        {
            break; /* queue empty */
        }

        /* DEBUG: track peak queue depth in this 1s window. Sampled AFTER fetch,
         * so this is the "remaining" depth — add `fetched` mentally for total. */
        {
            uint32_t qu = (uint32_t)k_msgq_num_used_get(&ads_runtime_queue);
            if (qu > g_dbg_queue_max_1s) g_dbg_queue_max_1s = qu;
        }

        size_t total_bytes = 1 + fetched * sample_size;
        if (total_bytes > sizeof(g_notification_payload))
        {
            LOG_ERR("Uncompressed payload too large (%u bytes)", (unsigned)total_bytes);
            for (size_t i = 0; i < fetched; i++)
            {
                if (k_msgq_put(&ads_runtime_queue, &g_uncompressed_batch[i], K_NO_WAIT) != 0)
                    current_status.samples_dropped++;
            }
            /* Already re-queued to the tail. Bypass the post-loop -ENOMEM
             * handler (which would re-stash and double-send) by clearing
             * stash and exiting this iteration cleanly. */
            g_live_retry_stash_count = 0;
            break;
        }
        else
        {
            uint8_t *payload = g_notification_payload;
            payload[0] = 0xC0;
            size_t out_off = 1;

            for (size_t i = 0; i < fetched; i++)
            {
                memcpy(&payload[out_off], &g_uncompressed_batch[i].sample_num, 4);
                out_off += 4;
                memcpy(&payload[out_off], &g_uncompressed_batch[i].timestamp_us, 8);
                out_off += 8;
                for (size_t c = 0; c < sel_count; c++)
                {
                    uint8_t chidx = g_config.ble_transfer_channels[c];
                    ble_put_i24(payload, out_off,
                                g_uncompressed_batch[i].channels[chidx]);
                    out_off += BLE_CH_BYTES;
                }
                memcpy(&payload[out_off], &g_uncompressed_batch[i].status, 4);
                out_off += 4;
            }

            int err = bt_gatt_notify(current_conn, &ads_service.attrs[1], payload, total_bytes);
            if (err == -ENOMEM)
            {
                result = -ENOMEM;
            }
            else if (err)
            {
                LOG_ERR("Uncompressed notification failed: %d", err);
                current_status.notification_errors++;
                result = err;
            }
            else
            {
                current_status.samples_sent += fetched;
                notifications_sent++;
                g_dbg_notifs_sent_1s++;
                consecutive_enomem = 0;
                /* Notify succeeded — discard the retry stash (whether it
                 * was just sent or never used) so the next iteration reads
                 * fresh samples from the queue. */
                g_live_retry_stash_count = 0;
                result = 1;
            }
        }

        if (result == -ENOMEM)
        {
            /* TX buffer full. Save the fetched batch into the retry stash
             * (only if it's not already a stash replay that failed) and
             * back off. The next worker run replays the stash before any
             * new queue read, preserving sample order. If the SAME batch
             * fails again (already stashed), fall back to drop-and-clear
             * so a permanently-broken BLE link can't wedge the worker.
             *
             * Pre-v0.0.51 behaviour was: drop on first -ENOMEM, hope the
             * app re-asks via backfill. That cost ~5–30 lost samples per
             * minute on the bench DK *with no backfill running and no
             * cage* — pure ENOMEM bursts from TX-buffer-pool contention. */
            consecutive_enomem++;
            g_dbg_enomem_events_1s++;
            if (consecutive_enomem > g_dbg_max_consecutive_enomem_1s) {
                g_dbg_max_consecutive_enomem_1s = consecutive_enomem;
            }
            if (from_stash)
            {
                /* Already a retry; give up on this batch. */
                current_status.samples_dropped += fetched;
                g_live_retry_stash_count = 0;
                LOG_DBG("ENOMEM-on-retry: dropping stashed %u samples, consec=%u",
                        (unsigned)fetched,
                        (unsigned)consecutive_enomem);
            }
            else
            {
                /* First-time -ENOMEM: stash for retry next tick. */
                memcpy(g_live_retry_stash, g_uncompressed_batch,
                       fetched * sizeof(g_uncompressed_batch[0]));
                g_live_retry_stash_count = fetched;
                LOG_DBG("ENOMEM-stash %u samples queue_used=%u consec=%u",
                        (unsigned)fetched,
                        (unsigned)k_msgq_num_used_get(&ads_runtime_queue),
                        (unsigned)consecutive_enomem);
            }
            uint32_t backoff_ms = MIN(consecutive_enomem * 25, 200);
            k_work_schedule(&ads_notification_work, K_MSEC(backoff_ms));
            break;
        }
    }

    /* DEBUG: emit per-second STATS line. Counters reset after emit. */
    {
        int64_t now = k_uptime_get();
        if (g_dbg_last_stats_emit_ms == 0) {
            g_dbg_last_stats_emit_ms = now;
            g_dbg_last_samples_sent_snapshot = current_status.samples_sent;
            g_dbg_last_samples_dropped_snapshot = current_status.samples_dropped;
        } else if (now - g_dbg_last_stats_emit_ms >= 1000) {
            uint32_t samples_sent_1s = current_status.samples_sent - g_dbg_last_samples_sent_snapshot;
            uint32_t samples_dropped_1s = current_status.samples_dropped - g_dbg_last_samples_dropped_snapshot;
            uint16_t mtu = current_conn ? bt_gatt_get_mtu(current_conn) : 0;
            LOG_INF("STATS notifs=%u sent=%u dropped=%u enomem=%u maxconsec=%u qmax=%u maxgap_ms=%u mtu=%u",
                    g_dbg_notifs_sent_1s,
                    samples_sent_1s,
                    samples_dropped_1s,
                    g_dbg_enomem_events_1s,
                    g_dbg_max_consecutive_enomem_1s,
                    g_dbg_queue_max_1s,
                    g_dbg_max_enqueue_gap_ms_1s,
                    (unsigned)mtu);
            g_dbg_notifs_sent_1s = 0;
            g_dbg_enomem_events_1s = 0;
            g_dbg_max_consecutive_enomem_1s = 0;
            g_dbg_queue_max_1s = 0;
            g_dbg_max_enqueue_gap_ms_1s = 0;
            g_dbg_last_samples_sent_snapshot = current_status.samples_sent;
            g_dbg_last_samples_dropped_snapshot = current_status.samples_dropped;
            g_dbg_last_stats_emit_ms = now;
        }
    }

    bool has_more_samples = (ads_runtime_queue_initialized && (k_msgq_num_used_get(&ads_runtime_queue) > 0));

    if (has_more_samples)
    {
        k_work_schedule(&ads_notification_work, K_MSEC(1));
    }
}

/* True when the running backfill/file op must stop: explicit cancel, Backfill
 * CCC off, or its connection is gone (epoch changed). */
static bool backfill_should_stop(void)
{
    return g_backfill_cancel || !current_conn || !backfill_notifications_enabled ||
           g_backfill_run_epoch != g_conn_epoch;
}

/* Non-blocking Backfill notify (0.0.92). Zephyr's ATT layer allocates the TX
 * buffer with K_FOREVER for any thread that is not the system workqueue
 * (att.c bt_att_chan_create_pdu), so bt_gatt_notify() from the backfill worker
 * BLOCKED instead of returning -ENOMEM: all the -ENOMEM retry/cancel loops
 * below were dead code, and while the worker waited, every freed TX buffer was
 * handed straight to it — the live path (sysworkq, K_NO_WAIT) got -ENOMEM and
 * dropped samples whenever the link was slow. Sending from the system
 * workqueue makes the allocation K_NO_WAIT: both paths now compete fairly and
 * the backfill worker sees -ENOMEM again (and can be cancelled). */
static struct
{
    const uint8_t *buf;
    uint16_t len;
    int err;
} g_bf_tx;
static K_SEM_DEFINE(g_bf_tx_done, 0, 1);

static void bf_tx_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    struct bt_conn *conn = current_conn;
    g_bf_tx.err = conn ? bt_gatt_notify(conn, &ads_service.attrs[8],
                                        g_bf_tx.buf, g_bf_tx.len)
                       : -ENOTCONN;
    k_sem_give(&g_bf_tx_done);
}
static K_WORK_DEFINE(g_bf_tx_work, bf_tx_work_handler);

/* Only ever called from the single backfill worker thread. The work item never
 * blocks (K_NO_WAIT allocation), so waiting for it is bounded. */
static int backfill_notify(const uint8_t *buf, size_t len)
{
    if (!current_conn)
    {
        return -ENOTCONN;
    }
    g_bf_tx.buf = buf;
    g_bf_tx.len = (uint16_t)len;
    g_bf_tx.err = -EIO;
    k_sem_reset(&g_bf_tx_done);
    k_work_submit(&g_bf_tx_work);
    (void)k_sem_take(&g_bf_tx_done, K_FOREVER);
    return g_bf_tx.err;
}

typedef struct
{
    size_t staged;
    size_t sample_size;
    size_t samples_per_pkt;
    uint32_t packets_sent;
    uint32_t samples_sent;
    /* R5 self-describing terminal: the ACTUAL delivered sample-number range for
     * this request. `first_sn`/`last_sn` are valid iff `samples_sent > 0`. The
     * terminal reports (first_sn, last_sn, samples_sent) so the app can tell a
     * PERMANENT SD gap (the reader found nothing for the range → count 0) apart
     * from BLE-transit loss (count > 0 but the phone received fewer 0xBF), and
     * stop burning retries on genuinely-absent data. */
    uint32_t first_sn;
    uint32_t last_sn;
} backfill_cb_ctx_t;

/* Per-record slow-channel forwarder: sends one small 0xBC notification per
 * BDF record carrying the record's battery/brightness/button slow samples so
 * the app can restore brightness+battery history across a disconnect
 * (they are otherwise live-only). Wire format (10 bytes, little-endian):
 *   [0]    0xBC
 *   [1..4] rec_first_sn (u32 LE)
 *   [5..6] battery_mv   (i16 LE)
 *   [7..8] brightness_mv(i16 LE)
 *   [9]    button code  (u8)
 * Loss-tolerant: fewer retries than the 0xBF data path — the app deduplicates
 * and a re-request re-delivers. Old apps ignore the unknown header. */
static void backfill_slow_cb(uint32_t rec_first_sn,
                             int32_t battery_mv,
                             int32_t brightness_mv,
                             int32_t button_code,
                             void *ctx_ptr)
{
    ARG_UNUSED(ctx_ptr);
    if (backfill_should_stop())
    {
        return;
    }
    uint8_t pkt[10];
    pkt[0] = 0xBC;
    pkt[1] = (uint8_t)(rec_first_sn & 0xFF);
    pkt[2] = (uint8_t)((rec_first_sn >> 8) & 0xFF);
    pkt[3] = (uint8_t)((rec_first_sn >> 16) & 0xFF);
    pkt[4] = (uint8_t)((rec_first_sn >> 24) & 0xFF);
    int16_t bat = (int16_t)battery_mv;
    int16_t bri = (int16_t)brightness_mv;
    pkt[5] = (uint8_t)(bat & 0xFF);
    pkt[6] = (uint8_t)((bat >> 8) & 0xFF);
    pkt[7] = (uint8_t)(bri & 0xFF);
    pkt[8] = (uint8_t)((bri >> 8) & 0xFF);
    pkt[9] = (uint8_t)(button_code & 0xFF);

    int err = -ENOMEM;
    int retries = 0;
    while (err == -ENOMEM && retries < 5 && !backfill_should_stop())
    {
        err = backfill_notify(pkt, sizeof(pkt));
        if (err == -ENOMEM)
        {
            k_sleep(K_MSEC(20));
            retries++;
        }
    }
    /* Silent on failure — see loss-tolerance note above. */
}

static int backfill_sample_cb(const void *record_48b, void *ctx_ptr)
{
    const ads_sample_data_t *s = (const ads_sample_data_t *)record_48b;
    backfill_cb_ctx_t *ctx = (backfill_cb_ctx_t *)ctx_ptr;

    if (!s || !ctx)
    {
        return 1;
    }
    if (backfill_should_stop())
    {
        return 1; /* stop the SD read — nobody will receive the rest */
    }

    size_t off = 1 + ctx->staged * ctx->sample_size;
    memcpy(&g_backfill_staging_buf[off], &s->sample_num, 4);
    off += 4;
    memcpy(&g_backfill_staging_buf[off], &s->timestamp_us, 8);
    off += 8;
    for (int c = 0; c < g_config.ble_transfer_channel_count; c++)
    {
        uint8_t chidx = g_config.ble_transfer_channels[c];
        ble_put_i24(g_backfill_staging_buf, off, s->channels[chidx]);
        off += BLE_CH_BYTES;
    }
    memcpy(&g_backfill_staging_buf[off], &s->status, 4);
    ctx->staged++;
    ctx->samples_sent++;
    /* R5: track the real delivered sn range for the self-describing terminal. */
    if (ctx->samples_sent == 1)
    {
        ctx->first_sn = s->sample_num;
    }
    ctx->last_sn = s->sample_num;

    if (ctx->staged >= ctx->samples_per_pkt)
    {
        g_backfill_staging_buf[0] = 0xBF; /* backfill data header */
        size_t pkt_len = 1 + ctx->staged * ctx->sample_size;
        int err = -ENOMEM;
        int retries = 0;
        /* Retry on -ENOMEM with a long-ish sleep so the live notification
         * work (which backs off 25-200ms on its own -ENOMEM) gets a chance
         * to drain the TX buffer. Short sleeps would starve live. */
        /* 40 × 30 ms: with the non-blocking send a congested link now
         * returns -ENOMEM here; give it ~1.2 s before the packet is lost (the
         * app then re-requests the shortfall from the terminal's count). */
        while (err == -ENOMEM && retries < 40 && !backfill_should_stop())
        {
            err = backfill_notify(g_backfill_staging_buf, pkt_len);
            if (err == -ENOMEM)
            {
                k_sleep(K_MSEC(30));
                retries++;
            }
        }
        if (err == 0)
        {
            ctx->packets_sent++;
        }
        else if (err != -ENOTCONN)
        {
            LOG_WRN("Backfill notify err after %d retries: %d", retries, err);
        }
        ctx->staged = 0;
        /* Inter-packet pace: hold the next backfill notify for one connection
         * interval (~50ms with CI=40) so the BLE TX queue has time to drain.
         * Without this, the worker enqueues 0xBF packets as fast as possible,
         * the controller's host buffers stay saturated, and the live
         * ads_notification_work_handler keeps getting -ENOMEM from
         * bt_gatt_notify(). The live path then drops samples (see comment
         * around line 797). Empirically with k_yield() alone, ~5-6 live
         * samples were dropped per 10-sample backfill burst; with a 15 ms
         * forced gap they drop to <1. The throughput cost is small: at most
         * one packet (10 samples) per 15 ms = 667 sps backfill, which still
         * drains a 5-min disconnect in <2 minutes wall-clock. */
        if (!g_live_notifications_paused)
        {
            k_sleep(K_MSEC(15));
        }
        else
        {
            /* Live is paused (app explicitly handed the link to backfill) —
             * no need to throttle, just yield to give workqueue a tick. */
            k_yield();
        }
    }
    return 0;
}

/**
/* ── File-service helpers (run on the backfill worker; stream on attrs[8]) ──
 * These reuse the Backfill characteristic + staging buffer; only the single
 * backfill worker drives them, so they never interleave with an EEG backfill. */

/* Per-read state passed through data_logger_read_file_stream's ctx. */
typedef struct
{
    bool notify_error;
} file_read_ctx_t;

/* Notify on the Backfill characteristic, retrying on -ENOMEM (same backpressure
 * discipline as the EEG-backfill path). Returns 0 on success, <0 otherwise. */
static int file_notify(const uint8_t *buf, size_t len)
{
    if (!current_conn || !backfill_notifications_enabled)
    {
        return -ENOTCONN;
    }
    int err = -ENOMEM;
    int retries = 0;
    while (err == -ENOMEM && retries < 100 && !g_file_abort)
    {
        err = backfill_notify(buf, len);
        if (err == -ENOMEM)
        {
            k_sleep(K_MSEC(20));
            retries++;
        }
    }
    return err;
}

/* One Backfill notification per directory entry:
 * [0xF0][nameLen:1][name][isDir:1][size:4 LE] */
static void file_list_entry_cb(const char *name, bool is_dir, uint32_t size,
                               void *ctx)
{
    ARG_UNUSED(ctx);
    if (g_file_abort)
    {
        return;
    }
    size_t nlen = strlen(name);
    if (nlen > 200)
    {
        nlen = 200; /* names are short; keep well inside the staging buffer */
    }
    uint8_t *p = g_backfill_staging_buf;
    size_t pos = 0;
    p[pos++] = ADS_FILE_HDR_DIR_ENTRY;
    p[pos++] = (uint8_t)nlen;
    memcpy(&p[pos], name, nlen);
    pos += nlen;
    p[pos++] = is_dir ? 1 : 0;
    memcpy(&p[pos], &size, 4);
    pos += 4;
    (void)file_notify(p, pos);
}

static void handle_file_list(void)
{
    const char *path = (const char *)config_upload_buffer;
    int n = data_logger_list_dir_meta(path, file_list_entry_cb, NULL);
    if (n < 0)
    {
        uint8_t pkt[2] = {ADS_FILE_HDR_DIR_ERR, (uint8_t)(-n)};
        (void)file_notify(pkt, 2);
        LOG_WRN("FILE_LIST '%s' error %d", path, n);
    }
    else
    {
        uint8_t done = ADS_FILE_HDR_DIR_DONE;
        (void)file_notify(&done, 1);
        LOG_INF("FILE_LIST '%s' done: %d entries (abort=%d)", path, n,
                (int)g_file_abort);
    }
}

/* Stream one file-read chunk as 0xF3 data packets, MTU-sized. Returns non-zero
 * to abort the stream (on abort request or a hard notify failure). */
static int file_read_chunk_cb(const uint8_t *buf, size_t len, void *ctx)
{
    file_read_ctx_t *c = (file_read_ctx_t *)ctx;
    if (g_file_abort)
    {
        return 1;
    }
    uint16_t mtu = bt_gatt_get_mtu(current_conn);
    size_t payload_max = (mtu > 3) ? (size_t)(mtu - 3) : 20;
    size_t chunk_max = (payload_max > 1) ? (payload_max - 1) : 1; /* minus header */
    if (chunk_max > sizeof(g_backfill_staging_buf) - 1)
    {
        chunk_max = sizeof(g_backfill_staging_buf) - 1;
    }
    size_t off = 0;
    while (off < len)
    {
        if (g_file_abort)
        {
            return 1;
        }
        size_t n = (len - off < chunk_max) ? (len - off) : chunk_max;
        g_backfill_staging_buf[0] = ADS_FILE_HDR_DATA;
        memcpy(&g_backfill_staging_buf[1], &buf[off], n);
        int err = file_notify(g_backfill_staging_buf, 1 + n);
        if (err)
        {
            if (c)
            {
                c->notify_error = true;
            }
            return 1; /* hard failure → abort; host resumes from its offset */
        }
        off += n;
    }
    return 0;
}

/* Signierschluessel einer Bastelversion (*.pem auf der Karte) nie ueber Bluetooth
 * herausgeben: ohne Pairing koennte sonst jeder in Funkreichweite den Schluessel
 * lesen und eigene Firmware aufspielen. Prueft auch FAT-Kurznamen (XXXXXX~1.PEM). */
static bool file_read_forbidden(const char *path)
{
    size_t n = strlen(path);
    for (size_t i = 0; i + 4 <= n; i++)
    {
        if (path[i] == '.' &&
            (path[i + 1] | 0x20) == 'p' && (path[i + 2] | 0x20) == 'e' &&
            (path[i + 3] | 0x20) == 'm')
        {
            return true;
        }
    }
    return false;
}

static void handle_file_read(void)
{
    const char *path = (const char *)config_upload_buffer;
    if (file_read_forbidden(path))
    {
        uint8_t pkt[2] = {ADS_FILE_HDR_DATA_ERR, (uint8_t)EACCES};
        (void)file_notify(pkt, 2);
        LOG_WRN("FILE_READ '%s' refused (key file)", path);
        return;
    }
    /* config.txt-Read waehrend einer laufenden Aufnahme: den BDF-Writer kurz
     * pausieren, damit der SD-Read nicht mit ihm um die Karte konkurriert (sonst
     * stockt/scheitert der Read). config.txt ist winzig -> vernachlaessigbare
     * Aufnahme-Luecke. Grosse Datei-Downloads (Recordings) bleiben unberuehrt. */
    bool pause_sd = (strstr(path, "config.txt") != NULL) ||
                    (strstr(path, "CONFIG.TXT") != NULL);
    if (pause_sd)
    {
        data_logger_pause_ads131_logging(true);
    }
    file_read_ctx_t c = {.notify_error = false};
    data_logger_result_t res = data_logger_read_file_stream(
        path, g_file_offset, g_file_len, file_read_chunk_cb, &c);
    if (pause_sd)
    {
        data_logger_pause_ads131_logging(false);
    }

    if (res == DATA_LOGGER_OK && !g_file_abort && !c.notify_error)
    {
        uint8_t done = ADS_FILE_HDR_DATA_DONE;
        (void)file_notify(&done, 1);
        LOG_INF("FILE_READ '%s' done", path);
    }
    else
    {
        uint8_t pkt[2] = {ADS_FILE_HDR_DATA_ERR,
                          (uint8_t)(g_file_abort ? 0 : (-res))};
        (void)file_notify(pkt, 2);
        LOG_WRN("FILE_READ '%s' ended res=%d abort=%d notify_err=%d", path,
                (int)res, (int)g_file_abort, (int)c.notify_error);
    }
}

/**
 * @brief Backfill worker thread
 *
 * Waits for a trigger from ads_control_write (0x20 command), then reads the
 * requested sample range from SD and sends it via Backfill notifications.
 * Lower priority (15) than the live streaming path so it never starves samples.
 * Also dispatches the file-service ops (LIST/READ) so they don't need their
 * own thread (RAM is at ~100%).
 */
static void backfill_worker_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true)
    {
        k_sem_take(&g_backfill_sem, K_FOREVER);

        /* Wall-time sync? Apply the recording rename + BDF header patch and
         * loop. Running it here (not in the BT RX context) guarantees it can
         * never race a range read's open file handle. */
        if (g_time_sync_pending)
        {
            g_time_sync_pending = false;
            (void)data_logger_apply_wall_time_to_recording();
            continue;
        }

        /* File-service op? Handle it and loop — don't fall into EEG backfill. */
        if (g_file_op != FILE_OP_NONE)
        {
            uint8_t op = g_file_op;
            g_file_op = FILE_OP_NONE;
            if (!current_conn || !backfill_notifications_enabled)
            {
                LOG_WRN("File worker: no connection, dropping op %u", op);
                continue;
            }
            if (op == FILE_OP_LIST)
            {
                handle_file_list();
            }
            else if (op == FILE_OP_READ)
            {
                handle_file_read();
            }
            continue;
        }

        if (!current_conn || !backfill_notifications_enabled)
        {
            LOG_WRN("Backfill worker: no connection / notifications not enabled, skipping");
            continue;
        }

        uint32_t start = g_backfill_req.start_sample_num;
        uint32_t end = g_backfill_req.end_sample_num;
        g_backfill_run_epoch = g_backfill_req.conn_epoch;

        if (g_backfill_run_epoch != g_conn_epoch)
        {
            LOG_WRN("Backfill worker: dropping request [%u .. %u] from a previous connection",
                    start, end);
            continue;
        }

        if (start > end)
        {
            LOG_WRN("Backfill worker: invalid range [%u > %u]", start, end);
            continue;
        }

        LOG_INF("Backfill worker: sending range [%u .. %u]", start, end);
        g_backfill_in_progress = true;

        /* Compute per-sample payload size (same layout as 0xC0 live stream) */
        uint8_t sel_count = g_config.ble_transfer_channel_count;
        size_t sample_size = 4 + 8 + (size_t)sel_count * BLE_CH_BYTES + 4; /* sample_num + ts + channels(3B) + status */

        /* How many samples fit in one MTU-sized notification (minus 1 header byte) */
        uint16_t mtu = bt_gatt_get_mtu(current_conn);
        size_t payload_max = (mtu > 3) ? (size_t)(mtu - 3) : 20;
        size_t avail = (payload_max > 1) ? (payload_max - 1) : 0;
        size_t spp = (sample_size > 0) ? (avail / sample_size) : 1; /* samples per packet */
        if (spp == 0)
        {
            spp = 1; /* always send at least one per packet */
        }
        if (spp > BACKFILL_STAGING_SAMPLES)
        {
            spp = BACKFILL_STAGING_SAMPLES;
        }

        /* Guard: staging buffer must be large enough */
        size_t needed = 1 + spp * sample_size;
        if (needed > sizeof(g_backfill_staging_buf))
        {
            spp = (sizeof(g_backfill_staging_buf) - 1) / sample_size;
            if (spp == 0)
            {
                LOG_ERR("Backfill: staging buffer too small for even one sample, aborting");
                g_backfill_in_progress = false;
                continue;
            }
            LOG_WRN("Backfill: capped spp to %u to fit staging buffer", (unsigned)spp);
        }

        backfill_cb_ctx_t ctx = {
            .staged = 0,
            .sample_size = sample_size,
            .samples_per_pkt = spp,
            .packets_sent = 0,
            .samples_sent = 0,
            .first_sn = 0,
            .last_sn = 0,
        };

        /* NEAR-HEAD GUARD (0.0.70): while logging is active, never serve a
         * range that touches the record currently being written. Field
         * 2026-07-16: a request for a 64 ms old gap forced a pre-read fs_sync
         * + read AT the live write head — the exact moment the entire SD
         * subsystem wedged (BDF writer stopped at record 28228/08:02:16, the
         * backfill responder answered ~once in 14 min). The app (>= tonight's
         * build) holds near-head gaps back for 60 s itself; this is the
         * device-side backstop (2 s). Too-fresh ranges answer a clean 0xBE
         * with delivered=0 WITHOUT touching the SD (the new app treats
         * near-head delivered-0 as retryable, not permanent); a range merely
         * ENDING too fresh is clamped and its aged part served. */
        uint32_t current_sample = ads131m08_ti_get_sample_count();
        bool skip_sd_read = false;
        if (data_logger_is_ads131_logging_initialized())
        {
            uint32_t head_guard =
                (current_sample > BACKFILL_HEAD_GUARD_SAMPLES)
                    ? current_sample - BACKFILL_HEAD_GUARD_SAMPLES
                    : 0;
            if (start >= head_guard)
            {
                LOG_WRN("Backfill: range [%u..%u] within %u samples of live "
                        "head %u — deferring (delivered 0, no SD access)",
                        start, end, BACKFILL_HEAD_GUARD_SAMPLES,
                        current_sample);
                skip_sd_read = true;
            }
            else if (end >= head_guard)
            {
                LOG_INF("Backfill: clamping end %u -> %u (live head %u)",
                        end, head_guard - 1, current_sample);
                end = head_guard - 1;
            }
        }

        /* Sync SD before reading — but only when the requested range extends
         * past what we've previously sync'd.  For typical backfill workloads
         * (processing many small historical gaps), the first sync covers the
         * whole queue and subsequent ranges hit the fast path. */
        data_logger_result_t res = DATA_LOGGER_OK;
        if (!skip_sd_read)
        {
            if (current_sample < g_backfill_last_synced_sample) {
                /* counter reset (new recording) — invalidate cache */
                g_backfill_last_synced_sample = 0;
            }
            if (end > g_backfill_last_synced_sample) {
                data_logger_sync_ads131_data();
                g_backfill_last_synced_sample = current_sample;
            } else {
                k_sleep(K_MSEC(2)); /* range already covered by previous sync */
            }

            res = data_logger_read_ads131_range(start, end,
                                                backfill_sample_cb, &ctx,
                                                backfill_slow_cb, NULL);
        }

        /* Flush any partially-filled packet */
        if (ctx.staged > 0 && !backfill_should_stop())
        {
            g_backfill_staging_buf[0] = 0xBF;
            size_t pkt_len = 1 + ctx.staged * ctx.sample_size;
            int err = -ENOMEM;
            for (int r = 0; r < 40 && err == -ENOMEM && !backfill_should_stop(); r++)
            {
                err = backfill_notify(g_backfill_staging_buf, pkt_len);
                if (err == -ENOMEM)
                {
                    k_sleep(K_MSEC(30));
                }
            }
            if (err && err != -ENOTCONN)
            {
                LOG_WRN("Backfill tail notify err: %d", err);
            }
            else if (!err)
            {
                ctx.packets_sent++;
            }
        }

        /* Send terminal marker: 0xBE = done cleanly, 0xBD = cancelled / error.
         *
         * R5 SELF-DESCRIBING TERMINAL (13 bytes, little-endian):
         *   [0]     0xBE (clean done) | 0xBD (cancel/error)
         *   [1..4]  first delivered sample-number (0 if none delivered)
         *   [5..8]  last  delivered sample-number (0 if none delivered)
         *   [9..12] delivered sample count
         * This lets the app distinguish a PERMANENT SD gap (count==0 for a
         * requested range → the samples were never written; do NOT retry
         * forever) from BLE-transit loss (count>0 but the phone received fewer
         * 0xBF samples → re-request the shortfall). Backward-compatible: old
         * apps read byte[0] only and ignore the trailing payload.
         *
         * Retry on -ENOMEM with a sleep, same pattern as the 0xBF data path
         * above. Without this retry, if live BLE traffic happens to fill the
         * TX buffer right when the worker emits the terminal, it is silently
         * dropped and the app waits forever for it: its 90s watchdog then
         * re-queues the same range, the firmware processes the new request and
         * possibly loses the terminal again — the queue head wedges
         * indefinitely and blocks every other gap behind it. */
        /* Never answer on a different connection than the request came on. */
        if (current_conn && backfill_notifications_enabled &&
            g_backfill_run_epoch == g_conn_epoch)
        {
            uint8_t term[13];
            term[0] = (res == DATA_LOGGER_OK && !g_backfill_cancel) ? 0xBE : 0xBD;
            uint32_t t_first = (ctx.samples_sent > 0) ? ctx.first_sn : 0;
            uint32_t t_last = (ctx.samples_sent > 0) ? ctx.last_sn : 0;
            uint32_t t_count = ctx.samples_sent;
            memcpy(&term[1], &t_first, 4);
            memcpy(&term[5], &t_last, 4);
            memcpy(&term[9], &t_count, 4);
            int err = -ENOMEM;
            int retries = 0;
            while (err == -ENOMEM && retries < 60)
            {
                err = backfill_notify(term, sizeof(term));
                if (err == -ENOMEM)
                {
                    k_sleep(K_MSEC(30));
                    retries++;
                }
            }
            if (err && err != -ENOTCONN)
            {
                LOG_WRN("Backfill terminal notify err after %d retries: %d", retries, err);
            }
        }

        g_backfill_in_progress = false;
        unsigned backfill_samples_sent = (unsigned)ctx.samples_sent;
        unsigned backfill_packets_sent = (unsigned)ctx.packets_sent;
        LOG_INF("Backfill worker done: res=%d cancel=%d stale=%d samples_sent=%u packets_sent=%u",
            (int)res, (int)g_backfill_cancel,
            (int)(g_backfill_run_epoch != g_conn_epoch),
            backfill_samples_sent, backfill_packets_sent);
    }
}

/**
 * @brief Initialization of the ADS Service
 */
int ads_service_init(void)
{
    LOG_INF("=== Initializing ADS Streaming Service ===");

    /* Avoid floating-point formatting in LOG_* macros (cbprintf lacks float support).
     * Report integer KB values instead.
     */
    unsigned uncompressed_kb = (unsigned)((ADS_QUEUE_SIZE * sizeof(ads_sample_data_t) + 1023) / 1024);

    LOG_INF("  Queue size: %d samples (%u KB raw)",
            ADS_QUEUE_SIZE,
            uncompressed_kb);
    LOG_INF("  Compression: DISABLED (raw 32-bit samples only)");
    LOG_INF("  Service UUID: 0483DDDA-6C9D-6CA9-5D41-03AD4FFF4ABB");

    /* Initialize batch timer */
    k_timer_init(&ads_batch_timer, ads_batch_timer_expiry, NULL);

    /* Initialize backfill semaphore and spawn worker thread */
    k_sem_init(&g_backfill_sem, 0, 1);
    k_thread_create(&backfill_thread, backfill_thread_stack,
                    K_THREAD_STACK_SIZEOF(backfill_thread_stack),
                    backfill_worker_thread_entry, NULL, NULL, NULL,
                    BACKFILL_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&backfill_thread, "ble_backfill");

    /* Reset statistics */
    memset(&current_status, 0, sizeof(current_status));

    /* Initialize runtime queue according to current config */
    int err = ads_runtime_queue_init();
    if (err)
    {
        LOG_ERR("Failed to initialize ADS runtime queue: %d", err);
        return err;
    }

    LOG_INF("ADS Streaming Service initialized");
    return 0;
}

void ads_service_on_disconnected(struct bt_conn *conn)
{
    LOG_INF("=== ADS Service: Connection lost ===");

    if (current_conn)
    {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    notifications_enabled = false;
    streaming_active = false;
    current_status.is_streaming = false;

    /* Clear the pause-live flag so a fresh session doesn't inherit a stale
     * paused state from the previous connection. The app re-establishes
     * pause/resume policy on a per-session basis. */
    g_live_notifications_paused = false;

    /* Cancel any pending notification work so no background attempts to notify occur */
    k_work_cancel_delayable(&ads_notification_work);

    /* Cancel pending PHY/DLE update if disconnect happened before it fired */
    k_work_cancel_delayable(&ble_param_update_work);

    /* Stop batching timer */
    k_timer_stop(&ads_batch_timer);

    /* Clear queue + retry stash so nothing from this stream can surface on
     * the next connection. */
    ads_live_discard_pending();

    /* Any backfill/request of this connection is now stale (see g_conn_epoch). */
    g_conn_epoch++;
    g_backfill_cancel = true;

    /* Deinit runtime queue to free memory */
    ads_runtime_queue_deinit();

    LOG_INF("ADS streaming stopped, scheduled work canceled, queue cleared");
}

/* Register callbacks for connection parameter updates */
static struct bt_conn_cb ads_conn_callbacks = {
    .le_phy_updated = phy_update_callback,
    .le_data_len_updated = data_len_update_callback,
    .le_param_updated = conn_param_update_callback,
};

/* Register callbacks at module initialization */
static int ads_service_conn_cb_init(void)
{
    bt_conn_cb_register(&ads_conn_callbacks);
    return 0;
}

SYS_INIT(ads_service_conn_cb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* Number of live samples that fit in ONE notification at the CURRENTLY
 * negotiated ATT MTU, clamped to the batch buffer. The ATT MTU is negotiated
 * per-connection and varies by central (23 default … 498 on this app), so this
 * is computed from the live MTU rather than a fixed constant — the producer
 * flushes as soon as one full packet's worth is queued, adapting when a
 * different phone/tablet (or a re-negotiation) changes the MTU. Mirrors the
 * work handler's `batch_cap` (see ads_notification_work_handler) so the flush
 * threshold and the drain size stay in lockstep. Returns the full buffer size
 * when no connection/MTU is available yet (harmless: the latency timer still
 * bounds the flush). */
static size_t ads_samples_per_packet(void)
{
    if (!current_conn)
        return ARRAY_SIZE(g_uncompressed_batch);
    const size_t sample_size = 16 + g_config.ble_transfer_channel_count * BLE_CH_BYTES;
    const uint16_t mtu = bt_gatt_get_mtu(current_conn);
    const size_t payload_max = (mtu > 3) ? (size_t)(mtu - 3) : 20;
    const size_t avail_for_samples = (payload_max > 1) ? (payload_max - 1) : 0;
    size_t spp = (avail_for_samples > 0 && sample_size > 0)
                     ? (avail_for_samples / sample_size)
                     : 0;
    if (spp == 0)
        spp = 1;
    if (spp > ARRAY_SIZE(g_uncompressed_batch))
        spp = ARRAY_SIZE(g_uncompressed_batch);
    return spp;
}

int ads_service_send_sample(const ads_sample_data_t *sample)
{
    if (!sample)
    {
        return -EINVAL;
    }

    /* Only queue samples when streaming is active (notifications enabled + started by client) */
    if (!streaming_active && !forced_streaming)
    {
        return 0; /* Silently ignore samples when not streaming */
    }

    if (!ads_runtime_queue_initialized)
    {
        /* Try to initialize runtime queue on-demand (defensive) */
        const int r = ads_runtime_queue_init();
        if (r)
            return r;
    }

    /* Try to queue the sample (non-blocking for ISR safety) */
    int err = k_msgq_put(&ads_runtime_queue, sample, K_NO_WAIT);

    if (err)
    {
        current_status.samples_dropped++;
        return -ENOMEM;
    }

    /* Kick the BLE notification work — batched. Flush IMMEDIATELY once a full
     * MTU-sized packet's worth of samples is queued; otherwise arm a bounded
     * latency timer so a partial batch still flushes within ADS_LIVE_FLUSH_MS.
     * This turns ~250 one-sample notifications/s into ~25 ten-sample ones,
     * killing the single-sample-gap fragmentation (see ADS_LIVE_FLUSH_MS).
     *
     * Semantics are load-bearing:
     *  - FULL batch → k_work_RESCHEDULE(K_NO_WAIT): must bring the deadline
     *    FORWARD (cancel any pending latency timer and run now).
     *  - partial   → k_work_SCHEDULE(K_MSEC(flush)): must NOT push the deadline
     *    out — k_work_schedule is a no-op if already pending, so the deadline
     *    stays anchored to the FIRST sample of the batch (latency stays
     *    bounded, never reset by later samples in the same batch).
     * The threshold is the MTU-derived samples-per-packet (adapts to the
     * negotiated MTU); the work handler drains exactly that many per notify. */
    const size_t queued = k_msgq_num_used_get(&ads_runtime_queue);
    if (queued >= ads_samples_per_packet())
    {
        k_work_reschedule(&ads_notification_work, K_NO_WAIT);
    }
    else
    {
        k_work_schedule(&ads_notification_work, K_MSEC(ADS_LIVE_FLUSH_MS));
    }

    /* DEBUG: track inter-enqueue gap. At 250Hz steady state this is ~4ms;
     * a value >8ms means the producer (ADC callback) was preempted/blocked. */
    int64_t now_ms = k_uptime_get();
    if (g_dbg_last_enqueue_ms != 0) {
        int64_t gap = now_ms - g_dbg_last_enqueue_ms;
        if (gap > 0 && (uint32_t)gap > g_dbg_max_enqueue_gap_ms_1s) {
            g_dbg_max_enqueue_gap_ms_1s = (uint32_t)gap;
        }
    }
    g_dbg_last_enqueue_ms = now_ms;

    return 0;
}

int ads_service_send_compressed_sample(const ads_compressed_sample_t *sample)
{
    ARG_UNUSED(sample);
    return -ENOTSUP;
}

int ads_service_set_forced_streaming(bool enable)
{
    forced_streaming = enable;
    ads_update_streaming_timer_state();
    return 0;
}

bool ads_service_is_forced_streaming(void)
{
    return forced_streaming;
}

/* Public API wrappers expected by other modules */
int ads_service_get_status(ads_status_t *status)
{
    if (!status)
        return -EINVAL;
    memcpy(status, &current_status, sizeof(ads_status_t));
    return 0;
}

/* BLE parameter update handler (work item defined near file top) */
static void ble_param_update_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!current_conn)
    {
        LOG_WRN("BLE param update: no connection");
        return;
    }

    int err;

    err = bt_conn_le_phy_update(current_conn, &phy_params);
    if (err)
    {
        LOG_ERR("PHY update failed: %d", err);
    }
    else
    {
        LOG_INF("PHY update initiated (requesting 2M PHY)");
    }

    err = bt_conn_le_data_len_update(current_conn, &data_len_params);
    if (err)
    {
        LOG_ERR("Data length update failed: %d", err);
    }
    else
    {
        LOG_INF("Data length update initiated");
    }
}

void ads_service_on_connected(struct bt_conn *conn)
{
    if (!conn)
        return;

    LOG_INF("=== ADS Service: Connection established ===");

    g_conn_epoch++;
    ads_live_discard_pending();
    current_conn = bt_conn_ref(conn);

#if IS_ENABLED(CONFIG_BT_GATT_EXCHANGE)
    int err;
    mtu_exchange_params.func = mtu_exchange_callback;
    err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    if (err)
    {
        LOG_ERR("MTU exchange failed: %d", err);
    }
    else
    {
        LOG_INF("MTU exchange initiated");
    }
#else
    LOG_DBG("MTU exchange API not enabled; skipping MTU exchange");
#endif

    /* Delay PHY and Data Length updates by 5 seconds so the central can
     * complete GATT service discovery first.  Firing these LL control
     * procedures immediately races with Android's discoverServices() and
     * triggers GATT_CONN_TERMINATE_LOCAL_HOST (status 0x16 / 22). */
    k_work_schedule(&ble_param_update_work, K_SECONDS(5));

    LOG_INF("ADS Service connection setup scheduled (PHY/DLE in 5 s)");
}

