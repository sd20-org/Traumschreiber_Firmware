/*
 * Traumschreiber2509 - Data Logger Subsystem
 *
 * Handles SD card filesystem operations for data logging and storage:
 * - SD card initialization and mounting
 * - File system operations (create, read, write)
 * - Directory listing and management
 * - Error handling and recovery
 *
 * @author ISDT
 * @date 2025
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Return codes */
    typedef enum
    {
        DATA_LOGGER_OK = 0,
        DATA_LOGGER_ERROR = -1,
        DATA_LOGGER_NO_SD_CARD = -2,
        DATA_LOGGER_MOUNT_FAILED = -3,
        DATA_LOGGER_FILE_ERROR = -4,
        DATA_LOGGER_WRITE_ERROR = -5,
        DATA_LOGGER_READ_ERROR = -6
    } data_logger_result_t;

    /* SD card information structure */
    typedef struct
    {
        uint32_t block_count;
        uint32_t block_size;
        uint64_t memory_size_mb;
        bool is_mounted;
        char mount_point[32];
    } sd_card_info_t;

    /**
     * @brief Initialize the data logger subsystem
     *
     * Initializes SD card access, mounts the filesystem, and performs basic
     * filesystem operations including creating a test file.
     *
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_init(void);

    /**
     * @brief Write the firmware version string to /SD/firmware-version.txt.
     *
     * Called once on boot after data_logger_init succeeds. The file is
     * overwritten each boot — pulling the SD into a PC lets the user check
     * which firmware build produced the EDF files alongside it.
     *
     * @param version  Null-terminated version string (e.g. "0.0.45").
     * @return DATA_LOGGER_OK on success, negative error code on failure.
     */
    data_logger_result_t data_logger_write_firmware_version_file(
        const char *version);

    /**
     * @brief Get SD card information
     *
     * @param info Pointer to structure to fill with SD card information
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_get_sd_info(sd_card_info_t *info);

    /**
     * @brief SD-writer health snapshot (v0.0.53+).
     *
     * Surfaces counters the writer thread maintains so the app can warn the
     * user mid-recording when SD writes have started to fail and we're
     * losing samples that BLE-live still delivers. Each output pointer
     * may be NULL.
     *
     * @param last_write_uptime_ms   k_uptime_get_32() of the most recent successful EDF record write
     * @param consecutive_failures   number of consecutive write failures since the last success
     * @param total_remounts_attempted  cumulative auto-remount attempts since boot
     * @param total_remounts_succeeded  cumulative auto-remount successes since boot
     * @param dropped_samples        cumulative samples dropped from the ring buffer because the writer couldn't keep up
     * @param unhealthy              true if the writer is currently failing past the auto-recovery threshold
     */
    void data_logger_get_sd_writer_health(
        uint32_t *last_write_uptime_ms,
        uint32_t *consecutive_failures,
        uint32_t *total_remounts_attempted,
        uint32_t *total_remounts_succeeded,
        uint32_t *dropped_samples,
        bool *unhealthy);

    /**
     * @brief List directory contents
     *
     * @param path Absolute path to list
     * @return Number of entries on success, negative error code on failure
     */
    int data_logger_list_directory(const char *path);

    /**
     * @brief List files in a directory and invoke a callback for each filename
     *
     * @param path Absolute path to list
     * @param cb Callback invoked for each filename (relative name)
     * @param ctx Opaque context passed to callback
     * @return Number of files listed on success, negative error code on failure
     */
    int data_logger_list_files(const char *path, void (*cb)(const char *name, void *ctx), void *ctx);

    /**
     * @brief Create a file with content
     *
     * @param filename Name of file to create (relative to mount point)
     * @param content Content to write to file
     * @param content_size Size of content in bytes
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    /**
     * Append content to a file in the SD root (created if missing), then sync
     * and close. Used for small log files such as NOISE.TXT.
     */
    data_logger_result_t data_logger_append_file(const char *filename,
                                                 const char *content,
                                                 size_t content_size);

    data_logger_result_t data_logger_create_file(const char *filename,
                                                 const char *content,
                                                 size_t content_size);

    /**
     * @brief Read file content
     *
     * @param filename Name of file to read (relative to mount point)
     * @param buffer Buffer to store content
     * @param buffer_size Size of buffer
     * @param bytes_read Pointer to variable to store actual bytes read
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_read_file(const char *filename,
                                               char *buffer,
                                               size_t buffer_size,
                                               size_t *bytes_read);

    /**
     * @brief Get file size
     *
     * @param filename Name of file (relative to mount point)
     * @param size Pointer to variable to store file size
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_get_file_size(const char *filename, size_t *size);

    /**
     * @brief Initialize CSV logging system
     *
     * Creates /log directory if it doesn't exist and finds the next available
     * log file number (0001_log.csv, 0002_log.csv, etc.)
     *
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_init_csv_logging(void);

    /**
     * @brief Log a CSV entry to the current log file
     *
     * Logs data in CSV format with timestamp. Format:
     * timestamp,uptime_ms,message,led_status
     *
     * @param uptime_ms System uptime in milliseconds
     * @param message Log message
     * @param led_status Current LED status
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_log_csv(uint64_t uptime_ms,
                                             const char *message,
                                             const char *led_status);

    /**
     * @brief Get current CSV log filename
     *
     * @param filename Buffer to store filename (relative to mount point)
     * @param buffer_size Size of filename buffer
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_get_csv_filename(char *filename, size_t buffer_size);

    /**
     * @brief Deinitialize the data logger subsystem
     *
     * Unmounts the filesystem and cleans up resources.
     *
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_deinit(void);

    /* Log an RTT-formatted timestamp and message to the CSV log.
     * rtt_timestamp: RTT timestamp string, e.g. "[00:00:12.853,576]" or "00:00:12.853,576"
     * rtt_message: exact message text as printed to RTT
     */
    data_logger_result_t data_logger_log_rtt(const char *rtt_timestamp, const char *rtt_message);

    /**
     * @brief Initialize ADS131M08 data logging system
     *
     * Creates ADS directory and initializes binary data logging for high-frequency ADC data.
     * Data is buffered and written in chunks to optimize SD card performance.
     *
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_init_ads131_logging(void);

    /**
     * @brief Returns true when an ADS131 recording is active and the buffer
     *        is accepting samples; false otherwise.
     *
     * Callers from the ADC data callback should consult this BEFORE invoking
     * data_logger_buffer_ads131_sample() so they don't pay a mutex lock per
     * sample (250Hz) when no recording is open. Returns the live state of
     * g_ads131_logging_initialized.
     */
    bool data_logger_is_ads131_logging_initialized(void);

    /**
     * @brief Buffer ADS131M08 sample data for logging
     *
     * Adds sample to internal buffer. When buffer is full (500 samples),
     * data is automatically written to SD card in binary format.
     *
     * @param sample_num Sample number/counter
     * @param timestamp_us Timestamp in microseconds
     * @param channels Array of 8 channel values (32-bit signed)
     * @param status ADC status word
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_buffer_ads131_sample(uint32_t sample_num,
                                                          uint64_t timestamp_us,
                                                          const int32_t *channels,
                                                          uint32_t status);

    /**
     * @brief Sync ADS131M08 data to SD card (safe during active recording)
     *
     * Drains all complete 250-sample chunks from the ring buffer to SD.
     * Does NOT write a partial padded record and does NOT touch the EDF header.
     * Use this from the BLE backfill path to ensure the file is current.
     *
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_sync_ads131_data(void);

    /**
     * @brief Flush any remaining ADS131M08 data to SD card
     *
     * Forces write of any buffered data to SD card, even if buffer is not full.
     * Useful for shutdown or periodic flushing.
     *
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_flush_ads131_data(void);

    /**
     * @brief Get current ADS131M08 log filename
     *
     * @param filename Buffer to store filename (relative to mount point)
     * @param buffer_size Size of filename buffer
     * @return DATA_LOGGER_OK on success, negative error code on failure
     */
    data_logger_result_t data_logger_get_ads131_filename(char *filename, size_t buffer_size);

    /**
     * @brief Set battery and brightness values for EDF slow channels
     *
     * Call once just before starting ADS131 recording (before SAADC is stopped).
     * Values are held constant for the entire recording; no SAADC reads during recording.
     *
     * @param battery_mv    Battery voltage in mV (AIN4)
     * @param brightness_mv Ambient light sensor voltage in mV (AIN0)
     */
    void data_logger_set_slow_channels(int16_t battery_mv, int16_t brightness_mv);

    /**
     * @brief Record a button event for the current EDF record
     *
     * Writes the event code (button_event_t value, 1..13) into the
     * ButtonEvent EDF channel for the current 1-second record, then
     * auto-clears. If multiple events occur within the same record window
     * the latest one wins. Code 0 is reserved for "no event" and ignored.
     * Safe from any context.
     */
    void data_logger_record_button_event(int event_code);

    /**
     * @brief Back-compat wrapper: records BUTTON_EVENT_PRESSED (code 12).
     */
    void data_logger_record_button_press(void);

    /**
     * @brief Phone-synced wall-clock time (the device has no RTC).
     *
     * Civil LOCAL time as sent by the app over BLE (control 0x40). The
     * logger keeps the sync moment's uptime so it can derive "now" later.
     */
    typedef struct
    {
        uint16_t year;   /* full year, e.g. 2026 */
        uint8_t  month;  /* 1..12 */
        uint8_t  day;    /* 1..31 */
        uint8_t  hour;   /* 0..23 */
        uint8_t  minute; /* 0..59 */
        uint8_t  second; /* 0..59 */
    } data_logger_wall_time_t;

    /**
     * @brief Store a wall-clock sync (uptime-referenced). Cheap, no I/O.
     */
    void data_logger_set_wall_time(const data_logger_wall_time_t *t);

    /**
     * @brief Compute the current wall time from the last sync + elapsed
     *        uptime. Returns false if no sync has happened since boot.
     */
    bool data_logger_wall_time_now(data_logger_wall_time_t *out);

    /**
     * @brief Rename the ACTIVE recording to carry date+time and patch the
     *        BDF header's startdate/starttime fields.
     *
     * AD0007.BDF → AD0007_YYYYMMDD_HHMM.BDF (connect-moment time; needs
     * FatFS LFN). The keep-open writer handle is closed under the recording
     * mutex first (FatFS forbids renaming open objects) and lazily reopens
     * under the new name on the next chunk. The AD####.SNX sidecar is
     * intentionally NOT renamed — every consumer derives its path from the
     * log NUMBER. No-op when no recording is active or the file already
     * carries a date suffix (only the header is then refreshed if it still
     * holds the EDF placeholder date).
     *
     * MUST be called from the backfill worker (or another context that is
     * serialized against data_logger_read_ads131_range) — never concurrently
     * with a range read, whose open read handle a rename could corrupt.
     */
    data_logger_result_t data_logger_apply_wall_time_to_recording(void);

    /**
     * @brief Pause or resume ADS131M08 data logging
     *
     * When paused, data is still buffered but not written to SD card.
     * This allows other high-priority SD card operations (like WAV writing)
     * to proceed without contention.
     *
     * @param pause true to pause, false to resume
     */
    void data_logger_pause_ads131_logging(bool pause);

    /**
     * @brief Callback type for ads131 range read
     *
     * Called once per matching sample. `record_48b` points to a packed 48-byte
     * ads131_sample_record_t (same layout as ads_sample_data_t in ads_service.h).
     * The pointer is only valid for the duration of the callback.
     *
     * @param record_48b  Pointer to the packed 48-byte sample record
     * @param ctx         Opaque context pointer passed to data_logger_read_ads131_range
     * @return 0 to continue, non-zero to stop the read early (request cancelled
     *         or the connection it belonged to is gone)
     */
    typedef int (*data_logger_ads131_rd_cb_t)(const void *record_48b, void *ctx);

    /**
     * Per-RECORD slow-channel callback for range reads. The BDF stores three
     * slow channels (1 sample per 250-sample record): battery_mv (AIN4),
     * brightness_mv (AIN0) and the button event code. Invoked once per fully
     * read record whose sample range intersects the requested range, so the
     * BLE backfill can restore brightness/battery history after a disconnect
     * (they are otherwise live-notification-only and lost with the link).
     */
    typedef void (*data_logger_slow_rd_cb_t)(uint32_t rec_first_sn,
                                             int32_t battery_mv,
                                             int32_t brightness_mv,
                                             int32_t button_code,
                                             void *ctx);

    /**
     * @brief Read a range of ADS131 samples from SD card
     *
    * Scans the active ADxxxx.EDF file for the current recording session and
    * invokes `cb` for every sample whose sample_num falls within
    * [start_sample_num, end_sample_num].
    * Timestamps and status are reconstructed (not stored verbatim in EDF).
    *
    * The read path is serialized through the global SD-card access manager,
    * but only for short open/seek/read/close chunks so the live writer can
    * interleave its own SD work instead of timing out behind one long backfill
    * read hold.
     *
     * This function is intended to be called from a low-priority background
     * thread (the backfill worker in ads_service.c).
     *
     * @param start_sample_num First sample_num to include (inclusive)
     * @param end_sample_num   Last sample_num to include (inclusive)
     * @param cb               Callback invoked for every matching sample
     * @param ctx              Opaque context passed to cb
     * @return DATA_LOGGER_OK on success, DATA_LOGGER_NO_SD_CARD if not ready,
     *         DATA_LOGGER_ERROR on other failures
     */
    data_logger_result_t data_logger_read_ads131_range(
        uint32_t start_sample_num,
        uint32_t end_sample_num,
        data_logger_ads131_rd_cb_t cb,
        void *ctx,
        data_logger_slow_rd_cb_t slow_cb,
        void *slow_ctx);

    /**
     * @brief Callback type for directory listing with metadata.
     *
     * @param name   Entry name (relative, NUL-terminated; valid only during cb)
     * @param is_dir true for a sub-directory, false for a regular file
     * @param size   File size in bytes (0 for directories)
     * @param ctx    Opaque context passed to data_logger_list_dir_meta
     */
    typedef void (*data_logger_dir_meta_cb_t)(const char *name, bool is_dir,
                                              uint32_t size, void *ctx);

    /**
     * @brief List a directory, invoking `cb` once per entry with name/type/size.
     *
     * Unlike data_logger_list_files (name only), this surfaces the dirent's
     * type and size so the host can render a real file browser. Serialized
     * through sd_card_manager_lock for the (short) readdir scan.
     *
     * @param path Path RELATIVE to the SD mount ("" = root, "DATA" = recordings)
     * @param cb   Callback invoked per entry
     * @param ctx  Opaque context passed to cb
     * @return entry count on success, negative error code on failure
     */
    int data_logger_list_dir_meta(const char *path,
                                  data_logger_dir_meta_cb_t cb, void *ctx);

    /**
     * @brief Callback type for streaming file reads.
     *
     * @param buf  Chunk bytes (valid only during the callback)
     * @param len  Number of valid bytes in buf
     * @param ctx  Opaque context passed to data_logger_read_file_stream
     * @return 0 to continue streaming, non-zero to abort the stream
     */
    typedef int (*data_logger_file_chunk_cb_t)(const uint8_t *buf, size_t len,
                                               void *ctx);

    /**
     * @brief Stream a file's bytes to `cb`, chunk by chunk (for BLE download).
     *
     * Opens the file ONCE (single positioning seek to start_offset), then reads
     * forward sequentially — taking sd_card_manager_lock only briefly per chunk
     * read and releasing it between chunks, so the live writer interleaves. Same
     * O(n) discipline as data_logger_read_ads131_range (re-seeking per chunk on
     * FF_USE_FASTSEEK=0 would be O(n^2) on a large BDF). Reuses the internal
     * backfill read buffer, so it MUST NOT run concurrently with a range read
     * (both are driven by the single backfill worker, so they never are).
     *
     * @param filename    Path relative to the SD mount point (e.g. "DATA/AD0001.BDF")
     * @param start_offset Byte offset to start at (0 = beginning)
     * @param max_bytes   Max bytes to stream (0 = until EOF)
     * @param cb          Callback invoked per chunk
     * @param ctx         Opaque context passed to cb
     * @return DATA_LOGGER_OK on clean completion/EOF/abort, error code on open/read failure
     */
    data_logger_result_t data_logger_read_file_stream(
        const char *filename, uint32_t start_offset, uint32_t max_bytes,
        data_logger_file_chunk_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DATA_LOGGER_H */