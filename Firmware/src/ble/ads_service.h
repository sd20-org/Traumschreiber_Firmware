/*
 * ADS Streaming Service - Header
 *
 * High-speed BLE service for streaming ADS131M08 ADC data.
 * Optimized for 250-500 Hz data rates using techniques from
 * Nordic's Throughput sample.
 *
 * Features:
 * - Efficient notification-based streaming
 * - MTU exchange for larger packets
 * - PHY update to 2M for double speed
 * - Data length extension for reduced overhead
 * - Queue-based data buffering from interrupt context
 *
 * @author ISDT
 * @date 2025
 */

#ifndef ADS_SERVICE_H_
#define ADS_SERVICE_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ADS Service UUID: 0483DDDA-6C9D-6CA9-5D41-03AD-4FFF4ABB (custom UUID) */
#define BT_UUID_ADS_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x0483DDDA, 0x6C9D, 0x6CA9, 0x5D41, 0x03AD4FFF4ABB)

#define BT_UUID_ADS_SERVICE BT_UUID_DECLARE_128(BT_UUID_ADS_SERVICE_VAL)

/* ADS Data Characteristic UUID */
#define BT_UUID_ADS_DATA_VAL \
    BT_UUID_128_ENCODE(0x0483DDDB, 0x6C9D, 0x6CA9, 0x5D41, 0x03AD4FFF4ABB)

#define BT_UUID_ADS_DATA BT_UUID_DECLARE_128(BT_UUID_ADS_DATA_VAL)

/* ADS Control Characteristic UUID */
#define BT_UUID_ADS_CONTROL_VAL \
    BT_UUID_128_ENCODE(0x0483DDDC, 0x6C9D, 0x6CA9, 0x5D41, 0x03AD4FFF4ABB)

#define BT_UUID_ADS_CONTROL BT_UUID_DECLARE_128(BT_UUID_ADS_CONTROL_VAL)

/* ADS Status Characteristic UUID */
#define BT_UUID_ADS_STATUS_VAL \
    BT_UUID_128_ENCODE(0x0483DDDD, 0x6C9D, 0x6CA9, 0x5D41, 0x03AD4FFF4ABB)

#define BT_UUID_ADS_STATUS BT_UUID_DECLARE_128(BT_UUID_ADS_STATUS_VAL)

/* ADS Backfill Characteristic UUID – notifications carry historical samples */
#define BT_UUID_ADS_BACKFILL_VAL \
    BT_UUID_128_ENCODE(0x0483DDDE, 0x6C9D, 0x6CA9, 0x5D41, 0x03AD4FFF4ABB)

#define BT_UUID_ADS_BACKFILL BT_UUID_DECLARE_128(BT_UUID_ADS_BACKFILL_VAL)

/* Firmware Version Characteristic UUID — read-only, returns the FW version
 * string (e.g. "0.0.45") so the connected client can show which build is
 * running without needing a custom advertising packet. */
#define BT_UUID_ADS_FW_VERSION_VAL \
    BT_UUID_128_ENCODE(0x0483DDE0, 0x6C9D, 0x6CA9, 0x5D41, 0x03AD4FFF4ABB)

#define BT_UUID_ADS_FW_VERSION BT_UUID_DECLARE_128(BT_UUID_ADS_FW_VERSION_VAL)

    /**
     * @brief ADS sample data structure
     *
     * Contains one sample from ADS131M08 with 8 channels.
     * Total size: 48 bytes (matches BIN file format).
     */
    typedef struct
    {
        uint32_t sample_num;   /* Sample counter (4 bytes) */
        uint64_t timestamp_us; /* Timestamp in microseconds (8 bytes) */
        int32_t channels[8];   /* 8 channel values (32 bytes) */
        uint32_t status;       /* Status word (4 bytes) */
    } __attribute__((packed)) ads_sample_data_t;

/* Maximum supported per-channel compression bit depth (compile-time upper bound).
 * The runtime-configured bit depth (g_config.ble_compress_bits) is read at startup
 * and does not change during runtime. We still allocate a compile-time maximum
 * buffer large enough to hold 8 channels × ADS_COMPRESSED_MAX_BITS bits.
 */
#define ADS_COMPRESSED_MAX_BITS 32
#define ADS_COMPRESSED_MAX_BYTES ((8 * ADS_COMPRESSED_MAX_BITS + 7) / 8)

    /**
     * @brief Compressed ADS sample data structure
     *
     * Delta-encoded + linear-compressed sample for efficient BLE transmission.
     * Size is dynamic depending on configured per-channel bit depth; buffer here
     * reserves the compile-time maximum number of bytes to support up to
     * ADS_COMPRESSED_MAX_BITS per channel.
     */
    typedef struct
    {
        uint8_t sample_num;                           /* Sample counter (1 byte, 0-255 rolling) */
        uint8_t status;                               /* Status word (1 byte, reduced from 32-bit) */
        uint8_t compressed[ADS_COMPRESSED_MAX_BYTES]; /* Packed per-channel N-bit deltas (MSB-first) */
    } __attribute__((packed)) ads_compressed_sample_t;

    /**
     * @brief Control commands
     */
    typedef enum
    {
        ADS_CONTROL_START_STREAMING = 0x01,
        ADS_CONTROL_STOP_STREAMING = 0x02,
        ADS_CONTROL_GET_STATUS = 0x03,
        /* Commands for uploading a runtime config.txt via the Control characteristic
         * Protocol: write single byte 0x10 to begin upload, then one or more
         * writes with first byte 0x11 followed by chunk bytes, then write 0x12
         * to finalize. On finalize the device writes the assembled file to
         * "config.txt" on the SD card and reboots to apply the new config.
         */
        ADS_CONTROL_UPLOAD_BEGIN = 0x10,
        ADS_CONTROL_UPLOAD_CHUNK = 0x11,
        ADS_CONTROL_UPLOAD_END = 0x12,
        /* Backfill commands – request ranges of SD-stored samples for gap filling.
         * Protocol:
         *   0x20 + 4B first_sample_num (LE) + 4B last_sample_num (LE)
         *       → firmware queues up to BACKFILL_CHUNK_N samples from SD and
         *         sends them as notifications on the Backfill characteristic.
         *         Each packet starts with header 0xBF and contains up to N
         *         packed ads_sample_data_t records.
         *   0x21 → cancel any in-progress backfill immediately
         *   0x22 → pause live-stream BLE notifications so backfill gets the
         *         whole BLE link. Producer keeps sampling and the SD path
         *         keeps writing; only the live-notification work is gated.
         *   0x23 → resume live-stream BLE notifications.
         */
        ADS_CONTROL_BACKFILL_REQUEST = 0x20,
        ADS_CONTROL_BACKFILL_CANCEL  = 0x21,
        ADS_CONTROL_PAUSE_LIVE       = 0x22,
        ADS_CONTROL_RESUME_LIVE      = 0x23,
        /* File-service commands – browse the SD filesystem and read files
         * over BLE. Results stream on the Backfill characteristic
         * with distinct header bytes (0xF0..0xF5) so they don't collide with
         * the EEG backfill 0xBF/0xBE/0xBD framing; the single backfill worker
         * handles both, so they never interleave.
         *   0x30 + path(ASCII)               → LIST: stream 0xF0 entries then 0xF1 done
         *   0x31 + 4B offset(LE) + 4B len(LE) + path(ASCII)
         *                                     → READ: stream 0xF3 data chunks then
         *                                       0xF4 done (len=0 ⇒ read to EOF)
         *   0x35                             → FILE_ABORT (cancel list/read)
         */
        ADS_CONTROL_FILE_LIST        = 0x30,
        ADS_CONTROL_FILE_READ        = 0x31,
        ADS_CONTROL_FILE_ABORT       = 0x35,
        /* Wall-time sync (fw 0.0.64+). The device has no RTC; the app sends
         * its LOCAL time right after connecting:
         *   0x40 + [yy(0..99, year-2000), MM, dd, hh, mm, ss]  (7 bytes)
         * The firmware stores time-of-sync + uptime reference and then, on
         * the backfill worker (serialized against range reads / file ops),
         * renames the ACTIVE recording to carry date+time
         * (AD0007.BDF → AD0007_20260703_2214.BDF) and patches the BDF
         * header's startdate/starttime. Repeated syncs only refresh the
         * clock reference — the file is renamed once. */
        ADS_CONTROL_TIME_SYNC        = 0x40,
        /* Noise measurement with shorted inputs (all channels' input mux →
         * AGND, see subsystems/noise_test.h). [0]=0x50, optional [1..2]=seconds
         * LE (1..600; missing/0 → config NOISE_TEST_SECS, else 10). Result is
         * appended to NOISE.TXT (readable via FILE_READ 0x31); progress is in
         * the status characteristic (noise state + sequence counter). */
        ADS_CONTROL_NOISE_TEST       = 0x50,
    } ads_control_cmd_t;

    /* Backfill-characteristic header bytes for the file service (host parses
     * these alongside the EEG-backfill 0xBF/0xBE/0xBD). */
#define ADS_FILE_HDR_DIR_ENTRY 0xF0 /* [0xF0][nameLen:1][name][isDir:1][size:4 LE] */
#define ADS_FILE_HDR_DIR_DONE  0xF1 /* [0xF1] */
#define ADS_FILE_HDR_DIR_ERR   0xF2 /* [0xF2][errcode:1] */
#define ADS_FILE_HDR_DATA      0xF3 /* [0xF3][bytes...] */
#define ADS_FILE_HDR_DATA_DONE 0xF4 /* [0xF4] */
#define ADS_FILE_HDR_DATA_ERR  0xF5 /* [0xF5][errcode:1] */

    /**
     * @brief Streaming status
     */
    typedef struct
    {
        bool is_streaming;            /* Currently streaming? */
        uint32_t samples_sent;        /* Total samples sent */
        uint32_t samples_dropped;     /* Samples dropped due to full queue */
        uint32_t notification_errors; /* BLE notification failures */
    } ads_status_t;

    /**
     * @brief Initialize ADS streaming service
     *
     * Sets up the BLE service, characteristics, and internal queues.
     *
     * @return 0 on success, negative errno on failure
     */
    int ads_service_init(void);

    /**
     * @brief Send ADS sample via BLE notification
     *
     * This function is ISR-safe and uses a message queue to pass data
     * from interrupt context to the BLE thread.
     *
     * @param sample Pointer to ADS sample data
     * @return 0 on success, -ENOMEM if queue is full
     */
    int ads_service_send_sample(const ads_sample_data_t *sample);

    /**
     * @brief Send compressed ADS sample via BLE notification
     *
     * Sends delta-encoded + linear-compressed 12-byte sample for efficient transmission.
     * This function is ISR-safe and uses a message queue to pass data
     * from interrupt context to the BLE thread.
     *
     * @param sample Pointer to compressed ADS sample data
     * @return 0 on success, -ENOMEM if queue is full
     */
    int ads_service_send_compressed_sample(const ads_compressed_sample_t *sample);

    /**
     * @brief Get current streaming status
     *
     * @param status Pointer to status structure to fill
     * @return 0 on success, negative errno on failure
     */
    int ads_service_get_status(ads_status_t *status);

    /**
     * @brief Connection callback
     *
     * Called when BLE connection is established.
     * Initiates MTU exchange, PHY update, and data length update.
     *
     * @param conn Connection handle
     */
    void ads_service_on_connected(struct bt_conn *conn);

    /**
     * @brief Disconnection callback
     *
     * Called when BLE connection is lost.
     *
     * @param conn Connection handle
     */
    void ads_service_on_disconnected(struct bt_conn *conn);

    int ads_service_set_forced_streaming(bool enable);

    bool ads_service_is_forced_streaming(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_SERVICE_H_ */
