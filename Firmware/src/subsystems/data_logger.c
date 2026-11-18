/*
 * Traumschreiber2509 - Data Logger Subsystem Implementation
 *
 * Handles SD card filesystem operations for data logging and storage.
 * Based on Zephyr filesystem sample with FAT and EXT2 support.
 *
 * @author ISDT
 * @date 2025
 */

#include "data_logger.h"
#include "charge_guard.h"
#include "sd_card_manager.h"
#include "nordic_adc.h"
#include "system_config.h" /* g_config.ads_gain for BDF physical calibration */
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#if defined(CONFIG_FAT_FILESYSTEM_ELM)
#include <ff.h>
#endif

/* EXT2 not used on this platform; keep FAT-only to simplify build */

LOG_MODULE_REGISTER(data_logger, LOG_LEVEL_INF);

/* Configuration based on filesystem type */
#if defined(CONFIG_FAT_FILESYSTEM_ELM)

#if defined(CONFIG_DISK_DRIVER_MMC)
#define DISK_DRIVE_NAME "SD2"
#else
#define DISK_DRIVE_NAME "SD"
#endif

#define DISK_MOUNT_PT "/" DISK_DRIVE_NAME ":"

static FATFS fat_fs;
static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
    .storage_dev = (void *)DISK_DRIVE_NAME,
    .mnt_point = DISK_MOUNT_PT,
};

#endif

/* Return code mapping */
#if defined(CONFIG_FAT_FILESYSTEM_ELM)
#define FS_RET_OK FR_OK
#else
#define FS_RET_OK 0
#endif

/* Constants */
#define MAX_PATH 128
#define TEST_FILE_NAME "moin.txt"

/* BDF (BioSemi Data Format) constants — 24-bit variant of EDF.
 * BDF stores each sample as 3 bytes (little-endian, two's-complement int24)
 * instead of EDF's 2 bytes. The header structure is identical to EDF; only
 * the version field, the reserved "24BIT" tag, and the digital min/max differ.
 * Keeping the full 24 bits avoids the lossy >>8 truncation that the old EDF
 * path used and lets us run lower PGA gain (more DC headroom) without losing
 * resolution. */
#define BDF_BPS             3   /* bytes per sample (24-bit) */
#define EDF_NUM_SLOW_CH     3   /* Battery, Brightness, Button — 1 sample/record each */
#define EDF_NUM_SIGNALS     (ADS131_MAX_CHANNELS + EDF_NUM_SLOW_CH)  /* 11 */
#define EDF_HEADER_BYTES    (256 + EDF_NUM_SIGNALS * 256)  /* 3072 bytes */
#define EDF_ADS_BYTES       (ADS131_MAX_CHANNELS * ADS131_WRITE_CHUNK_SIZE * BDF_BPS) /* 6000 bytes */
#define EDF_SLOW_BYTES      (EDF_NUM_SLOW_CH * BDF_BPS)                               /* 9 bytes */
#define EDF_RECORD_BYTES    (EDF_ADS_BYTES + EDF_SLOW_BYTES)                    /* 6009 bytes */
#define EDF_NUM_REC_OFFSET  236 /* byte offset of num_data_records field in general header */
#define EDF_REC_DUR_OFFSET  244 /* byte offset of record_duration field in general header */

/* Global variables */
static sd_card_info_t g_sd_info = {0};
static const char *disk_mount_pt = DISK_MOUNT_PT;

/* CSV logging variables */
static char g_csv_filename[64] = {0};
static uint32_t g_csv_log_number = 0;
static bool g_csv_logging_initialized = false;

/* ADS131M08 data logging variables */
#define ADS131_BUFFER_SIZE 512      /* Buffer 512 samples to cut RAM while preserving >2 s headroom at 250 Hz */
#define ADS131_MAX_CHANNELS 8       /* 8 ADC channels */
#define ADS131_WRITE_CHUNK_SIZE 250 /* Write 250 samples per SD operation */

typedef struct
{
    uint32_t sample_num;                   /* Sample counter */
    uint64_t timestamp_us;                 /* Timestamp in microseconds */
    int32_t channels[ADS131_MAX_CHANNELS]; /* 8 channel values */
    uint32_t status;                       /* ADC status word */
} __attribute__((packed)) ads131_sample_record_t;

/* Circular buffer for ADS131 data */
static ads131_sample_record_t g_ads131_buffer[ADS131_BUFFER_SIZE];
/* Use a static staging buffer in BSS to avoid large stack allocations in writer thread */
static ads131_sample_record_t g_ads131_staging[ADS131_WRITE_CHUNK_SIZE];
static uint32_t g_ads131_write_index = 0;      /* Where to write new data */
static uint32_t g_ads131_read_index = 0;       /* Where to read for SD write */
static uint32_t g_ads131_buffered_samples = 0; /* Number of samples in buffer */
static char g_ads131_filename[64] = {0};
static uint32_t g_ads131_log_number = 0;
static bool g_ads131_logging_initialized = false;
static bool g_ads131_first_write_successful = false;
static uint32_t g_ads131_write_error_count = 0;
static uint32_t g_ads131_dropped_samples = 0; /* Track dropped samples due to buffer overflow */
/* Current ring-overflow episode (0.0.92): the producer (ADS reader thread)
 * only records it; the writer logs it once the episode is over, so a silent
 * SD gap shows up in RTT with its sample-number range. */
static uint32_t g_ads131_ovf_first_sn = 0;
static uint32_t g_ads131_ovf_last_sn = 0;
static uint32_t g_ads131_ovf_count = 0;
/* v0.0.53 SD-writer health diagnostics — added after a real overnight
 * recording lost ~2.5 h of EDF data with no logged symptom: the writer
 * silently dropped samples while BLE-live kept streaming. Reading these
 * via BLE STATUS or the post-mortem `firmware-version.txt` companion lets
 * us catch the failure mode next time. */
static uint32_t g_ads131_last_write_uptime_ms = 0;     /* k_uptime_get_32 of last OK write */
static uint32_t g_ads131_consecutive_failures = 0;
static uint32_t g_ads131_total_remounts_attempted = 0;
static uint32_t g_ads131_total_remounts_succeeded = 0;
/* Flag flipped by writer-thread after N consecutive failures so the BLE
 * STATUS path can surface a "SD unhealthy" indicator without taking the
 * SD mutex itself. */
static volatile bool g_ads131_sd_unhealthy = false;
static struct k_mutex g_ads131_mutex;
static volatile bool g_ads131_paused = false;

/* Remount attempt: triggered after N consecutive write failures. fs_unmount
 * + fs_mount via the same mountpoint that data_logger_init used. Returns
 * 0 on success, negative errno on failure. Caller is the writer thread —
 * holds no SD locks at the call site. */
static int try_remount_filesystem(void);
/* Append a one-line health snapshot to /SD/sd_health.log so the
 * post-mortem of the next failure is visible without RTT. */
static void log_sd_health_event(const char *event);

/* EDF state variables */
static uint32_t  g_edf_data_records     = 0;
static uint32_t  g_edf_first_sample_num = 0;
static bool      g_edf_first_sample_seen = false;
static uint64_t  g_edf_first_timestamp_us = 0;
static uint64_t  g_edf_last_timestamp_us  = 0;
/* Slow channels: snapshotted once at recording start, held constant during recording */
static int16_t   g_edf_battery_mv    = 0;
static int16_t   g_edf_brightness_mv = 0;
/* Button slow channel: 0 = no event, else button_event_t code (1..13). Set by
 * data_logger_record_button_event(), cleared after each record write. */
static volatile uint8_t g_edf_button_event_code = 0;
/* Set by data_logger_sync_ads131_data(): the writer thread fs_syncs the BDF and
 * .SNX handles right after draining, independent of SD_SYNC_INTERVAL_S. */
static volatile bool g_edf_sync_request = false;
/* Phone-synced wall clock (device has no RTC; app sends local time via BLE
 * control 0x40). Stored as days-since-epoch + second-of-day plus the uptime
 * at sync so "now" derives from elapsed uptime. Guarded by g_ads131_mutex
 * (recursive, so nested use from the writer/init paths is fine).
 * ref_uptime_ms == 0 means "never synced" (RAM is byte-tight: the sentinel
 * replaces a padded bool; a real sync stamps max(uptime,1)). Fields ordered
 * widest-first — the linker RAM region has single-digit bytes of slack. */
static struct
{
    int64_t ref_uptime_ms;   /* k_uptime_get() at sync; 0 = never synced */
    int32_t base_days;       /* days since 1970-01-01 (local civil date) */
    int32_t base_sec_of_day; /* seconds within that day */
} g_wall_clock;
/* BSS-allocated conversion buffers — too large for writer-thread stack */
/* BDF record buffers: one byte per byte of the on-disk record (3 bytes/sample). */
static uint8_t   g_edf_write_buf[EDF_RECORD_BYTES];
static uint8_t   g_edf_backfill_buf[EDF_RECORD_BYTES];

/* Persistent BDF data-file handle, kept OPEN across chunks by the writer.
 *
 * Why: previously each chunk did sd_card_safe_open()->fs_seek(0,FS_SEEK_END)->
 * write->sd_card_safe_close(). sd_card_safe_open holds sd_card_manager_lock for
 * the whole open..close span, and the seek-to-end is O(file-size) (FF_USE_FASTSEEK
 * =0 -> full FAT cluster-chain walk). On a multi-hour (~185 MB) recording that
 * seek held the SD lock for hundreds of ms PER chunk, starving the prio-15
 * backfill reader past the app's ~15 s idle-watchdog (every backfill chunk got
 * dropped). Keeping the file open at end-of-file (fptr stays at EOF after each
 * fs_write — the Zephyr FATFS shim only re-seeks-to-end when FS_O_APPEND is set,
 * which we do NOT set) eliminates the per-chunk seek, so the lock is held only
 * for fs_write()+fs_sync() ~= a few ms. Opened with RAW fs_open (NOT
 * sd_card_safe_open) + explicit per-chunk sd_card_manager_lock so the lock is
 * RELEASED between chunks (sd_card_safe_open would hold it until close). */
static struct fs_file_t g_edf_data_file;
static bool             g_edf_data_file_open   = false;
static bool             g_edf_data_file_at_end = false; /* one-time seek-to-end done */
/* Refresh num_data_records in the header only every N chunks. The trailing
 * fs_seek(0,FS_SEEK_END) after the count write is the ONLY remaining
 * O(file-size) seek; doing it 1/min instead of 1/chunk keeps the SD lock free
 * for backfill while bounding the crash-time count staleness to <=N records
 * (the data itself is fs_sync'd every chunk; finalize_edf_header() writes the
 * exact count on a clean stop). */
#define EDF_HDR_UPDATE_INTERVAL 60
static uint32_t         g_edf_hdr_dirty_chunks = 0;

/* ── Per-record sample-number sidecar index (.SNX) ──────────────────────────
 * The backfill reader located samples POSITIONALLY (record_idx =
 * (sn-first_sn)/250; rec_first_sn = first_sn + record_idx*250), assuming every
 * record holds 250 CONSECUTIVE sample-numbers. The capture ring drops samples
 * on overflow (the hw sn counter keeps advancing) WITHOUT zero-filling, so
 * after any drop that correspondence shifts: the reader seeks to the wrong
 * record or past the written region and returns nothing (bare 0xBE) even though
 * the data is on the card. Fix: alongside AD####.BDF keep a tiny AD####.SNX with
 * ONE uint32 LE per data record = that record's REAL first sample_num
 * (data[0].sample_num). The reader bisects it to find the record holding a
 * requested sn and labels samples from the stored first_sn, tolerating capture
 * gaps. Kept open like the BDF handle (same per-chunk lock + fs_sync). Old
 * recordings have no .SNX -> reader falls back to the positional formula. */
static struct fs_file_t g_edf_snx_file;
static bool             g_edf_snx_open   = false;
static bool             g_edf_snx_at_end = false;
/* Latched on any .SNX open/write error: stop touching the index for the rest of
 * the recording so it never desyncs from the BDF (a short-but-consistent .SNX is
 * usable; a misaligned one is not). Cleared at recording start. */
static bool             g_edf_snx_failed = false;
#define EDF_SNX_ENTRY_BYTES 4

/* Sync + close the kept-open BDF data handle under the SD lock. Idempotent.
 * Best-effort: if the lock can't be taken, still drop the handle (a stale FIL
 * after e.g. unmount must not be reused). */
static void edf_close_data_file(void)
{
    if (!g_edf_data_file_open && !g_edf_snx_open)
    {
        g_edf_hdr_dirty_chunks = 0;
        return;
    }
    /* Close ONLY under the lock. FF_FS_REENTRANT=0 → an unlocked fs_sync/fs_close
     * racing the writer's locked fs_write on the shared FATFS volume could corrupt
     * the FAT/dir entry. All callers (flush/deinit/start-reset/remount) run in
     * low-contention contexts, so the 5 s acquire normally succeeds; on the
     * pathological timeout we leave the handle open (still valid — the writer
     * keeps appending / a later call closes it) rather than do unsynchronized I/O. */
    if (sd_card_manager_lock(5000) != 0)
    {
        LOG_WRN("edf_close_data_file: SD lock timeout; leaving handle open");
        return;
    }
    if (g_edf_data_file_open)
    {
        (void)fs_sync(&g_edf_data_file);
        (void)fs_close(&g_edf_data_file);
        g_edf_data_file_open   = false;
        g_edf_data_file_at_end = false;
    }
    /* Close the sidecar sample-number index in lockstep with the BDF. */
    if (g_edf_snx_open)
    {
        (void)fs_sync(&g_edf_snx_file);
        (void)fs_close(&g_edf_snx_file);
        g_edf_snx_open   = false;
        g_edf_snx_at_end = false;
    }
    g_edf_hdr_dirty_chunks = 0;
    sd_card_manager_unlock();
}

/* Store a signed value as 24-bit little-endian two's complement at buf[off]. */
static inline void bdf_put_i24(uint8_t *buf, size_t off, int32_t v)
{
    uint32_t u = (uint32_t)v;
    buf[off + 0] = (uint8_t)(u & 0xFF);
    buf[off + 1] = (uint8_t)((u >> 8) & 0xFF);
    buf[off + 2] = (uint8_t)((u >> 16) & 0xFF);
}

/* Read a 24-bit little-endian two's-complement value, sign-extended to int32. */
static inline int32_t bdf_get_i24(const uint8_t *buf, size_t off)
{
    uint32_t u = (uint32_t)buf[off] |
                 ((uint32_t)buf[off + 1] << 8) |
                 ((uint32_t)buf[off + 2] << 16);
    if (u & 0x00800000u)
        u |= 0xFF000000u; /* sign-extend */
    return (int32_t)u;
}

/* Background thread for SD card writes */
K_THREAD_STACK_DEFINE(ads131_writer_stack, 3072);
static struct k_thread ads131_writer_thread;
static struct k_sem ads131_writer_sem;

/* Internal function prototypes */
static data_logger_result_t init_sd_card_hardware(void);
static data_logger_result_t mount_filesystem(void);
static data_logger_result_t create_test_file(void);
static data_logger_result_t create_log_directory(void);
static uint32_t find_next_log_number(void);
static data_logger_result_t create_csv_header(void);

/* ADS131M08 logging helper functions */
static data_logger_result_t create_ads_directory(void);
static uint32_t find_next_ads131_log_number(void);
static data_logger_result_t write_ads131_data_to_sd(const ads131_sample_record_t *data,
                                                    uint32_t samples);
static void ads131_writer_thread_entry(void *p1, void *p2, void *p3);

/* EDF helper functions */
static data_logger_result_t write_edf_header(const char *full_path);
static void finalize_edf_header(void);

/* Helper function to safely copy string with escape handling */
static void safe_string_copy(char *dest, const char *src, size_t dest_size)
{
    size_t i, j = 0;

    if (!dest || !src || dest_size < 2)
    {
        if (dest && dest_size > 0)
        {
            dest[0] = '\0';
        }
        return;
    }

    for (i = 0; i < strlen(src) && j < (dest_size - 1); i++)
    {
        char c = src[i];
        /* Replace non-printable characters with space */
        if (c < 32 || c > 126)
        {
            c = ' ';
        }
        /* Escape quotes */
        if (c == '"')
        {
            if (j < (dest_size - 2))
            {
                dest[j++] = '\\';
                dest[j++] = '"';
            }
        }
        else
        {
            dest[j++] = c;
        }
    }
    dest[j] = '\0';
}

data_logger_result_t data_logger_init(void)
{
    data_logger_result_t ret;

    LOG_INF("=== Initializing Data Logger Subsystem ===");
    LOG_INF("Filesystem: %s", CONFIG_FAT_FILESYSTEM_ELM ? "FAT" : "EXT2");
    LOG_INF("Mount point: %s", disk_mount_pt);

    /* Initialize SD card hardware */
    ret = init_sd_card_hardware();
    k_sleep(K_MSEC(100)); /* Short delay to ensure stability */
    if (ret != DATA_LOGGER_OK)
    {
        LOG_ERR("SD card hardware initialization failed: %d", ret);
        return ret;
    }
    else
    {
        LOG_INF("SD card hardware initialized successfully");
    }
    k_sleep(K_MSEC(100)); /* Short delay to ensure stability */
    /* Mount filesystem */
    ret = mount_filesystem();
    if (ret != DATA_LOGGER_OK)
    {
        LOG_ERR("Filesystem mount failed: %d", ret);
        return ret;
    }
    else
    {
        LOG_INF("Filesystem mounted successfully");
    }

    /* List root directory contents */
    LOG_INF("=== SD Card Contents ===");
    int entries = data_logger_list_directory(disk_mount_pt);
    if (entries >= 0)
    {
        LOG_INF("Found %d entries in root directory", entries);
    }
    else
    {
        LOG_WRN("Failed to list directory contents: %d", entries);
    }

    /* Create test file */
    ret = create_test_file();
    if (ret != DATA_LOGGER_OK)
    {
        LOG_WRN("Test file creation failed: %d", ret);
        /* Don't fail initialization for test file issues */
    }

    /* Initialize CSV logging system */
    ret = data_logger_init_csv_logging();
    if (ret != DATA_LOGGER_OK)
    {
        LOG_ERR("CSV logging initialization failed: %d", ret);
        /* Don't fail main initialization for CSV logging issues */
    }

    /* Initialize ADS131M08 data logging system */
    ret = data_logger_init_ads131_logging();
    if (ret != DATA_LOGGER_OK)
    {
        LOG_ERR("ADS131 logging initialization failed: %d", ret);
        /* Don't fail main initialization for ADS131 logging issues */
    }

    LOG_INF("Data logger subsystem initialized successfully");
    return DATA_LOGGER_OK;
}

static data_logger_result_t init_sd_card_hardware(void)
{
    static const char *disk_pdrv = DISK_DRIVE_NAME;
    int ret;

    LOG_INF("Initializing SD card hardware...");

    /* Initialize storage */
    ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_CTRL_INIT, NULL);
    if (ret != 0)
    {
        LOG_ERR("Storage init failed: %d", ret);
        return DATA_LOGGER_NO_SD_CARD;
    }
    else
    {
        LOG_INF("Storage initialized successfully");
    }

    k_sleep(K_MSEC(100)); /* Short delay to ensure stability */

    /* Get sector count */
    ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &g_sd_info.block_count);
    if (ret != 0)
    {
        LOG_ERR("Unable to get sector count: %d", ret);
        return DATA_LOGGER_ERROR;
    }
    else
    {
        LOG_INF("Block count: %u", g_sd_info.block_count);
    }

    /* Get sector size */
    ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &g_sd_info.block_size);
    if (ret != 0)
    {
        LOG_ERR("Unable to get sector size: %d", ret);
        return DATA_LOGGER_ERROR;
    }
    else
    {
        LOG_INF("Block size: %u bytes", g_sd_info.block_size);
    }

    /* Calculate memory size */
    g_sd_info.memory_size_mb = (uint64_t)g_sd_info.block_count * g_sd_info.block_size;

    LOG_INF("SD Card detected:");
    LOG_INF("  Block count: %u", g_sd_info.block_count);
    LOG_INF("  Block size: %u bytes", g_sd_info.block_size);
    LOG_INF("  Total size: %u MB", (uint32_t)(g_sd_info.memory_size_mb >> 20));
    k_sleep(K_MSEC(100)); /* Short delay to ensure stability */

    /* Deinitialize storage (will be re-initialized during mount) */
    ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_CTRL_DEINIT, NULL);
    if (ret != 0)
    {
        LOG_WRN("Storage deinit failed: %d", ret);
        /* Continue anyway */
    }
    else
    {
        LOG_INF("Storage deinitialized successfully");
    }

    return DATA_LOGGER_OK;
}

static data_logger_result_t mount_filesystem(void)
{
    int ret;

    LOG_INF("Mounting filesystem...");

    mp.mnt_point = disk_mount_pt;
    strncpy(g_sd_info.mount_point, disk_mount_pt, sizeof(g_sd_info.mount_point) - 1);

    ret = fs_mount(&mp);
    if (ret != FS_RET_OK)
    {
        LOG_ERR("Filesystem mount failed: %d", ret);
        g_sd_info.is_mounted = false;
        return DATA_LOGGER_MOUNT_FAILED;
    }

    LOG_INF("Filesystem mounted successfully");
    g_sd_info.is_mounted = true;

    /* Test unmount/remount cycle */
    LOG_INF("Testing unmount/remount cycle...");
    ret = fs_unmount(&mp);
    if (ret != FS_RET_OK)
    {
        LOG_ERR("Filesystem unmount failed: %d", ret);
        return DATA_LOGGER_ERROR;
    }

    ret = fs_mount(&mp);
    if (ret != FS_RET_OK)
    {
        LOG_ERR("Filesystem remount failed: %d", ret);
        g_sd_info.is_mounted = false;
        return DATA_LOGGER_MOUNT_FAILED;
    }

    LOG_INF("Unmount/remount cycle successful");
    return DATA_LOGGER_OK;
}

static data_logger_result_t create_test_file(void)
{
    const char *test_content = "Hallo Traumschreiber!";
    data_logger_result_t ret;

    LOG_INF("Creating test file: %s", TEST_FILE_NAME);

    ret = data_logger_create_file(TEST_FILE_NAME, test_content, strlen(test_content));
    if (ret != DATA_LOGGER_OK)
    {
        LOG_ERR("Failed to create test file: %d", ret);
        return ret;
    }

    LOG_INF("Test file created successfully");

    /* Verify file was created by checking its size */
    size_t file_size;
    ret = data_logger_get_file_size(TEST_FILE_NAME, &file_size);
    if (ret == DATA_LOGGER_OK)
    {
        LOG_INF("Test file size: %zu bytes", file_size);
        if (file_size == strlen(test_content))
        {
            LOG_INF("Test file size matches expected content");
        }
        else
        {
            LOG_WRN("Test file size mismatch: expected %zu, got %zu",
                    strlen(test_content), file_size);
        }
    }

    return DATA_LOGGER_OK;
}

data_logger_result_t data_logger_get_sd_info(sd_card_info_t *info)
{
    if (!info)
    {
        return DATA_LOGGER_ERROR;
    }

    memcpy(info, &g_sd_info, sizeof(sd_card_info_t));
    return DATA_LOGGER_OK;
}

int data_logger_list_directory(const char *path)
{
    struct fs_dir_t dirp;
    /* On the stack (not static): ~60 B fits every caller's 3 KB stack, and
     * since LFN (fw 0.0.64) each static instance would cost +36 B of the
     * last few free RAM bytes (MAX_FILE_NAME grew 12 -> 48). */
    struct fs_dirent entry;
    int res;
    int count = 0;

    if (!path)
    {
        LOG_ERR("Invalid path parameter");
        return DATA_LOGGER_ERROR;
    }

    LOG_INF("Listing directory: %s", path);

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, path);
    if (res)
    {
        LOG_ERR("Error opening directory %s [%d]", path, res);
        return res;
    }

    LOG_INF("Directory contents:");
    for (;;)
    {
        res = fs_readdir(&dirp, &entry);

        /* End of directory */
        if (res || entry.name[0] == 0)
        {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR)
        {
            LOG_INF("  [DIR ] %s", entry.name);
        }
        else
        {
            LOG_INF("  [FILE] %s (size = %zu bytes)", entry.name, entry.size);
        }
        count++;
    }

    fs_closedir(&dirp);

    if (res == 0)
    {
        res = count;
    }

    return res;
}

int data_logger_list_files(const char *path, void (*cb)(const char *name, void *ctx), void *ctx)
{
    struct fs_dir_t dirp;
    /* On the stack (not static): ~60 B fits every caller's 3 KB stack, and
     * since LFN (fw 0.0.64) each static instance would cost +36 B of the
     * last few free RAM bytes (MAX_FILE_NAME grew 12 -> 48). */
    struct fs_dirent entry;
    int res;
    int count = 0;

    if (!path || !cb)
    {
        LOG_ERR("Invalid parameters to data_logger_list_files");
        return DATA_LOGGER_ERROR;
    }

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, path);
    if (res)
    {
        LOG_ERR("Error opening directory %s [%d]", path, res);
        return res;
    }

    for (;;)
    {
        res = fs_readdir(&dirp, &entry);
        if (res || entry.name[0] == 0)
        {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_FILE)
        {
            cb(entry.name, ctx);
            count++;
        }
    }

    fs_closedir(&dirp);
    return count;
}

data_logger_result_t data_logger_write_firmware_version_file(
    const char *version)
{
    if (!version) return DATA_LOGGER_ERROR;
    if (!g_sd_info.is_mounted) return DATA_LOGGER_MOUNT_FAILED;

    char full_path[MAX_PATH];
    snprintf(full_path, sizeof(full_path), "%s/firmware-version.txt",
             disk_mount_pt);

    /* Delete any stale version file from a previous build so the new content
     * is the only thing on disk (we can't FS_O_TRUNC reliably here). Ignore
     * "not found" errors. */
    (void)fs_unlink(full_path);

    struct fs_file_t file;
    fs_file_t_init(&file);
    int ret = sd_card_safe_open(&file, full_path, FS_O_CREATE | FS_O_WRITE);
    if (ret != 0)
    {
        LOG_ERR("firmware-version.txt: open failed: %d", ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%s\n", version);
    if (n < 0 || n >= (int)sizeof(buf)) n = (int)strlen(version);

    ssize_t written = sd_card_safe_write(&file, buf, n);
    (void)sd_card_safe_sync(&file);
    (void)sd_card_safe_close(&file);

    if (written != n)
    {
        LOG_ERR("firmware-version.txt: write short %zd/%d", written, n);
        return DATA_LOGGER_WRITE_ERROR;
    }
    LOG_INF("firmware-version.txt written: %s", version);
    return DATA_LOGGER_OK;
}

data_logger_result_t data_logger_create_file(const char *filename,
                                             const char *content,
                                             size_t content_size)
{
    struct fs_file_t file;
    char full_path[MAX_PATH];
    int ret;
    ssize_t written;

    if (!filename || !content)
    {
        return DATA_LOGGER_ERROR;
    }

    if (!g_sd_info.is_mounted)
    {
        LOG_ERR("SD card not mounted");
        return DATA_LOGGER_MOUNT_FAILED;
    }

    /* Build full path */
    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, filename);

    LOG_DBG("Creating file: %s", full_path);

    fs_file_t_init(&file);

    ret = sd_card_safe_open(&file, full_path, FS_O_CREATE | FS_O_WRITE);
    if (ret != 0)
    {
        LOG_ERR("Failed to open file %s for writing: %d", full_path, ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    written = sd_card_safe_write(&file, content, content_size);
    if (written != (ssize_t)content_size)
    {
        LOG_ERR("Write error: wrote %zd bytes, expected %zu", written, content_size);
        sd_card_safe_close(&file);
        return DATA_LOGGER_WRITE_ERROR;
    }

    /* Truncate to exactly content_size: without this a shorter new file leaves
     * stale trailing bytes from a larger previous file. Critical for config.txt
     * (the parser is last-value-wins, so old trailing lines would override the
     * new ones on the next boot). */
    ret = fs_truncate(&file, (off_t)content_size);
    if (ret != 0)
    {
        LOG_WRN("File truncate to %zu failed: %d (stale trailing bytes possible)",
                content_size, ret);
    }

    ret = sd_card_safe_sync(&file);
    if (ret != 0)
    {
        LOG_WRN("File sync failed: %d", ret);
        /* Continue with close */
    }

    ret = sd_card_safe_close(&file);
    if (ret != 0)
    {
        LOG_ERR("Failed to close file: %d", ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    LOG_DBG("File created successfully: %s (%zu bytes)", filename, content_size);
    return DATA_LOGGER_OK;
}

data_logger_result_t data_logger_append_file(const char *filename,
                                             const char *content,
                                             size_t content_size)
{
    struct fs_file_t file;
    char full_path[MAX_PATH];

    if (!filename || !content)
        return DATA_LOGGER_ERROR;
    if (!g_sd_info.is_mounted)
        return DATA_LOGGER_MOUNT_FAILED;

    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, filename);
    fs_file_t_init(&file);
    int ret = sd_card_safe_open(&file, full_path, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (ret != 0)
    {
        LOG_ERR("Append: open %s failed: %d", full_path, ret);
        return DATA_LOGGER_FILE_ERROR;
    }
    ssize_t written = sd_card_safe_write(&file, content, content_size);
    (void)sd_card_safe_sync(&file);
    (void)sd_card_safe_close(&file);
    if (written != (ssize_t)content_size)
    {
        LOG_ERR("Append: wrote %zd of %zu bytes to %s", written, content_size, filename);
        return DATA_LOGGER_WRITE_ERROR;
    }
    return DATA_LOGGER_OK;
}

data_logger_result_t data_logger_read_file(const char *filename,
                                           char *buffer,
                                           size_t buffer_size,
                                           size_t *bytes_read)
{
    struct fs_file_t file;
    char full_path[MAX_PATH];
    int ret;
    ssize_t read_bytes;

    if (!filename || !buffer || !bytes_read)
    {
        return DATA_LOGGER_ERROR;
    }

    if (!g_sd_info.is_mounted)
    {
        LOG_ERR("SD card not mounted");
        return DATA_LOGGER_MOUNT_FAILED;
    }

    /* Build full path */
    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, filename);

    fs_file_t_init(&file);

    ret = sd_card_safe_open(&file, full_path, FS_O_READ);
    if (ret != 0)
    {
        LOG_ERR("Failed to open file %s for reading: %d", full_path, ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    read_bytes = sd_card_safe_read(&file, buffer, buffer_size);
    if (read_bytes < 0)
    {
        LOG_ERR("Read error: %zd", read_bytes);
        sd_card_safe_close(&file);
        return DATA_LOGGER_READ_ERROR;
    }

    *bytes_read = (size_t)read_bytes;

    ret = sd_card_safe_close(&file);
    if (ret != 0)
    {
        LOG_ERR("Failed to close file: %d", ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    return DATA_LOGGER_OK;
}

data_logger_result_t data_logger_get_file_size(const char *filename, size_t *size)
{
    struct fs_dirent stat;
    char full_path[MAX_PATH];
    int ret;

    if (!filename || !size)
    {
        return DATA_LOGGER_ERROR;
    }

    if (!g_sd_info.is_mounted)
    {
        LOG_ERR("SD card not mounted");
        return DATA_LOGGER_MOUNT_FAILED;
    }

    /* Build full path */
    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, filename);

    ret = fs_stat(full_path, &stat);
    if (ret != 0)
    {
        LOG_ERR("Failed to stat file %s: %d", full_path, ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    *size = stat.size;
    return DATA_LOGGER_OK;
}

static data_logger_result_t create_log_directory(void)
{
    char log_dir_path[MAX_PATH];
    int ret;

    snprintf(log_dir_path, sizeof(log_dir_path), "%s/LOG", disk_mount_pt);

    /* Check if directory already exists */
    struct fs_dirent stat;
    ret = fs_stat(log_dir_path, &stat);
    if (ret == 0 && stat.type == FS_DIR_ENTRY_DIR)
    {
        LOG_INF("Log directory already exists: %s", log_dir_path);
        return DATA_LOGGER_OK;
    }

    /* Create the directory */
    ret = fs_mkdir(log_dir_path);
    if (ret != 0)
    {
        LOG_ERR("Failed to create log directory %s: %d", log_dir_path, ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    LOG_INF("Created log directory: %s", log_dir_path);
    return DATA_LOGGER_OK;
}

static uint32_t find_next_log_number(void)
{
    struct fs_dir_t dirp;
    /* On the stack (not static): ~60 B fits every caller's 3 KB stack, and
     * since LFN (fw 0.0.64) each static instance would cost +36 B of the
     * last few free RAM bytes (MAX_FILE_NAME grew 12 -> 48). */
    struct fs_dirent entry;
    char log_dir_path[MAX_PATH];
    uint32_t max_number = 0;
    int ret;

    snprintf(log_dir_path, sizeof(log_dir_path), "%s/LOG", disk_mount_pt);

    fs_dir_t_init(&dirp);

    ret = fs_opendir(&dirp, log_dir_path);
    if (ret)
    {
        LOG_WRN("Could not open log directory: %d", ret);
        return 1; /* Start with 0001 */
    }

    /* Scan for existing log files (case-insensitive) */
    for (;;)
    {
        ret = fs_readdir(&dirp, &entry);

        if (ret || entry.name[0] == 0)
        {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_FILE)
        {
            /* Build uppercase copy for case-insensitive match */
            char name_upper[64];
            size_t n = strlen(entry.name);
            if (n >= sizeof(name_upper))
            {
                n = sizeof(name_upper) - 1;
            }
            for (size_t i = 0; i < n; i++)
            {
                name_upper[i] = toupper((unsigned char)entry.name[i]);
            }
            name_upper[n] = '\0';

            if (strstr(name_upper, "_LOG.CSV") != NULL)
            {
                uint32_t number = 0;
                /* Try to parse leading 4-digit number */
                if (sscanf(entry.name, "%4u", &number) == 1)
                {
                    if (number > max_number)
                    {
                        max_number = number;
                    }
                    LOG_DBG("Found existing log file: %s (number: %u)", entry.name, number);
                }
            }
        }
    }

    fs_closedir(&dirp);

    uint32_t next_number = max_number + 1;
    LOG_INF("Next log file number: %04u", next_number);
    return next_number;
}

static data_logger_result_t create_csv_header(void)
{
    const char *header = "timestamp,message\n";
    return data_logger_create_file(g_csv_filename, header, strlen(header));
}

data_logger_result_t data_logger_init_csv_logging(void)
{
    data_logger_result_t ret;

    if (!g_sd_info.is_mounted)
    {
        LOG_ERR("SD card not mounted - cannot initialize CSV logging");
        return DATA_LOGGER_MOUNT_FAILED;
    }

    LOG_INF("Initializing CSV logging system");

    /* Create log directory */
    ret = create_log_directory();
    if (ret != DATA_LOGGER_OK)
    {
        return ret;
    }

    /* Find next available log file number */
    g_csv_log_number = find_next_log_number();

    /* Generate CSV filename (use LOG directory consistently) */
    snprintf(g_csv_filename, sizeof(g_csv_filename), "LOG/%04u_log.csv", g_csv_log_number);

    LOG_INF("CSV log file: %s", g_csv_filename);

    /* Create CSV file with header */
    ret = create_csv_header();
    if (ret != DATA_LOGGER_OK)
    {
        LOG_ERR("Failed to create CSV header: %d", ret);
        return ret;
    }

    g_csv_logging_initialized = true;
    LOG_INF("CSV logging initialized successfully");
    return DATA_LOGGER_OK;
}

data_logger_result_t data_logger_log_csv(uint64_t uptime_ms,
                                         const char *message,
                                         const char *led_status)
{
    struct fs_file_t file;
    char full_path[MAX_PATH];
    char csv_line[512];
    char timestamp[32];
    char safe_message[128];
    int ret;
    ssize_t written;

    if (!g_csv_logging_initialized)
    {
        LOG_WRN("CSV logging not initialized");
        return DATA_LOGGER_ERROR;
    }

    if (!g_sd_info.is_mounted)
    {
        LOG_ERR("SD card not mounted");
        return DATA_LOGGER_MOUNT_FAILED;
    }

    if (!message)
    {
        return DATA_LOGGER_ERROR;
    }

    /* Safely copy and sanitize strings */
    safe_string_copy(safe_message, message, sizeof(safe_message));

    /* Generate timestamp in format: HH:MM:SS.mmm,000 (no brackets) */
    uint32_t hours = uptime_ms / (1000 * 60 * 60);
    uint32_t mins = (uptime_ms / (1000 * 60)) % 60;
    uint32_t secs = (uptime_ms / 1000) % 60;
    uint32_t msecs = uptime_ms % 1000;
    snprintf(timestamp, sizeof(timestamp), "%02u:%02u:%02u.%03u,000",
             hours, mins, secs, msecs);

    /* Format CSV line: timestamp,message (message quoted) */
    snprintf(csv_line, sizeof(csv_line), "%s,\"%s\"\n",
             timestamp, safe_message);

    /* Build full path */
    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, g_csv_filename);

    fs_file_t_init(&file);

    /* Open file in append mode */
    ret = sd_card_safe_open(&file, full_path, FS_O_WRITE | FS_O_APPEND);
    if (ret != 0)
    {
        LOG_ERR("Failed to open CSV file %s for append: %d", full_path, ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    /* Write CSV line */
    written = sd_card_safe_write(&file, csv_line, strlen(csv_line));
    if (written != (ssize_t)strlen(csv_line))
    {
        LOG_ERR("CSV write error: wrote %zd bytes, expected %zu",
                written, strlen(csv_line));
        sd_card_safe_close(&file);
        return DATA_LOGGER_WRITE_ERROR;
    }

    /* Sync to ensure data is written to SD card */
    ret = sd_card_safe_sync(&file);
    if (ret != 0)
    {
        LOG_WRN("CSV file sync failed: %d", ret);
    }

    ret = sd_card_safe_close(&file);
    if (ret != 0)
    {
        LOG_ERR("Failed to close CSV file: %d", ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    LOG_DBG("CSV logged: %s", csv_line);
    return DATA_LOGGER_OK;
}

data_logger_result_t data_logger_get_csv_filename(char *filename, size_t buffer_size)
{
    if (!filename || buffer_size == 0)
    {
        return DATA_LOGGER_ERROR;
    }

    if (!g_csv_logging_initialized)
    {
        LOG_WRN("CSV logging not initialized");
        return DATA_LOGGER_ERROR;
    }

    strncpy(filename, g_csv_filename, buffer_size - 1);
    filename[buffer_size - 1] = '\0';
    return DATA_LOGGER_OK;
}

/* v0.0.53: best-effort filesystem remount after the writer has racked up a
 * streak of failures. Called by the writer thread, NOT from ISR context.
 * Returns 0 on success, negative errno on failure.
 *
 * Race-safety note: this happens while the producer (ADC ISR) is still
 * actively buffering samples into g_ads131_buffer. The buffer itself is in
 * RAM and not affected by the fs cycle; once we're remounted, the writer
 * resumes pulling chunks. The window where fs ops would fail is the few
 * tens of ms of fs_unmount+fs_mount. */
static int try_remount_filesystem(void)
{
    int ret = fs_unmount(&mp);
    if (ret != FS_RET_OK && ret != -EINVAL /* already unmounted */)
    {
        LOG_WRN("try_remount: fs_unmount returned %d (proceeding anyway)", ret);
        /* Still attempt the re-mount: depending on the failure mode, the
         * fs may be in a half-state that a fresh mount can re-init. */
    }
    /* Short settle so the disk driver's caches commit. */
    k_sleep(K_MSEC(50));
    ret = fs_mount(&mp);
    if (ret != FS_RET_OK)
    {
        g_sd_info.is_mounted = false;
        return ret;
    }
    g_sd_info.is_mounted = true;
    return 0;
}

/* v0.0.53: append a single line to /SD/sd_health.log so the post-mortem of
 * the *next* failure is visible even without RTT. Best-effort; if the SD is
 * the very thing that's broken, this write also fails — but we already log
 * to RTT and via BLE STATUS, so a missed sd_health.log line is acceptable. */
static void log_sd_health_event(const char *event)
{
    struct fs_file_t file;
    char path[MAX_PATH];
    char line[128];
    snprintf(path, sizeof(path), "%s/sd_health.log", disk_mount_pt);
    int n = snprintf(line, sizeof(line),
                     "[uptime_ms=%u] %s (consec=%u total_remounts=%u/%u)\n",
                     k_uptime_get_32(), event,
                     (unsigned)g_ads131_consecutive_failures,
                     (unsigned)g_ads131_total_remounts_succeeded,
                     (unsigned)g_ads131_total_remounts_attempted);
    if (n <= 0) return;

    fs_file_t_init(&file);
    int rc = sd_card_safe_open(&file, path, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (rc != 0)
    {
        LOG_WRN("sd_health.log: open failed (%d)", rc);
        return;
    }
    (void)sd_card_safe_write(&file, line, (size_t)n);
    (void)sd_card_safe_sync(&file);
    (void)sd_card_safe_close(&file);
}

/* Accessor used by ads_service BLE STATUS path to surface SD-writer health
 * to the app. All counters except `unhealthy` are monotonic — caller
 * decides what to do with them. */
void data_logger_get_sd_writer_health(
    uint32_t *last_write_uptime_ms,
    uint32_t *consecutive_failures,
    uint32_t *total_remounts_attempted,
    uint32_t *total_remounts_succeeded,
    uint32_t *dropped_samples,
    bool *unhealthy)
{
    if (last_write_uptime_ms)    *last_write_uptime_ms = g_ads131_last_write_uptime_ms;
    if (consecutive_failures)    *consecutive_failures = g_ads131_consecutive_failures;
    if (total_remounts_attempted) *total_remounts_attempted = g_ads131_total_remounts_attempted;
    if (total_remounts_succeeded) *total_remounts_succeeded = g_ads131_total_remounts_succeeded;
    if (dropped_samples)         *dropped_samples = g_ads131_dropped_samples;
    if (unhealthy)               *unhealthy = g_ads131_sd_unhealthy;
}

data_logger_result_t data_logger_deinit(void)
{
    int ret;

    LOG_INF("Deinitializing data logger subsystem");

    edf_close_data_file(); /* flush + close the kept-open recording handle before unmount */

    if (g_sd_info.is_mounted)
    {
        ret = fs_unmount(&mp);
        if (ret != FS_RET_OK)
        {
            LOG_ERR("Failed to unmount filesystem: %d", ret);
            return DATA_LOGGER_ERROR;
        }
        g_sd_info.is_mounted = false;
        LOG_INF("Filesystem unmounted");
    }

    /* Clear SD info structure */
    memset(&g_sd_info, 0, sizeof(g_sd_info));

    /* Clear CSV logging state */
    g_csv_logging_initialized = false;
    g_csv_log_number = 0;
    memset(g_csv_filename, 0, sizeof(g_csv_filename));

    LOG_INF("Data logger subsystem deinitialized");
    return DATA_LOGGER_OK;
}

data_logger_result_t data_logger_log_rtt(const char *rtt_timestamp, const char *rtt_message)
{
    struct fs_file_t file;
    char full_path[MAX_PATH];
    char csv_line[512];
    char timestamp[64];
    char safe_message[256];
    int ret;
    ssize_t written;

    if (!g_csv_logging_initialized)
    {
        LOG_WRN("CSV logging not initialized");
        return DATA_LOGGER_ERROR;
    }

    if (!g_sd_info.is_mounted)
    {
        LOG_ERR("SD card not mounted");
        return DATA_LOGGER_MOUNT_FAILED;
    }

    if (!rtt_timestamp || !rtt_message)
    {
        return DATA_LOGGER_ERROR;
    }

    /* Sanitize message (replace non-printables) */
    safe_string_copy(safe_message, rtt_message, sizeof(safe_message));

    /* Normalize timestamp: remove surrounding brackets if present
     * Accept formats: "[HH:MM:SS.mmm,uuu]" or "HH:MM:SS.mmm,uuu"
     */
    size_t tlen = strlen(rtt_timestamp);
    if (tlen > 0 && rtt_timestamp[0] == '[')
    {
        /* copy without first and last char if last is ']' */
        if (rtt_timestamp[tlen - 1] == ']')
        {
            size_t copy_len = tlen - 2;
            if (copy_len >= sizeof(timestamp))
            {
                copy_len = sizeof(timestamp) - 1;
            }
            memcpy(timestamp, &rtt_timestamp[1], copy_len);
            timestamp[copy_len] = '\0';
        }
        else
        {
            /* unexpected, just copy and strip leading '[' */
            strncpy(timestamp, rtt_timestamp + 1, sizeof(timestamp) - 1);
            timestamp[sizeof(timestamp) - 1] = '\0';
        }
    }
    else
    {
        strncpy(timestamp, rtt_timestamp, sizeof(timestamp) - 1);
        timestamp[sizeof(timestamp) - 1] = '\0';
    }

    /* Ensure timestamp uses comma before microseconds (already present in RTT) */

    /* Format CSV line: timestamp,message (message quoted) */
    snprintf(csv_line, sizeof(csv_line), "%s,\"%s\"\n", timestamp, safe_message);

    /* Build full path */
    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, g_csv_filename);

    fs_file_t_init(&file);

    /* Open file in append mode */
    ret = sd_card_safe_open(&file, full_path, FS_O_WRITE | FS_O_APPEND);
    if (ret != 0)
    {
        LOG_ERR("Failed to open CSV file %s for append: %d", full_path, ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    /* Write CSV line */
    written = sd_card_safe_write(&file, csv_line, strlen(csv_line));
    if (written != (ssize_t)strlen(csv_line))
    {
        LOG_ERR("CSV write error: wrote %zd bytes, expected %zu", written, strlen(csv_line));
        sd_card_safe_close(&file);
        return DATA_LOGGER_WRITE_ERROR;
    }

    /* Sync to ensure data is written to SD card */
    ret = sd_card_safe_sync(&file);
    if (ret != 0)
    {
        LOG_WRN("CSV file sync failed: %d", ret);
    }

    ret = sd_card_safe_close(&file);
    if (ret != 0)
    {
        LOG_ERR("Failed to close CSV file: %d", ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    LOG_DBG("CSV logged (RTT): %s", csv_line);
    return DATA_LOGGER_OK;
}

/* =============================================================================
 * ADS131M08 Data Logging Implementation
 * =============================================================================
 */

/* Recording directory on the SD card (fw >= 0.0.96: DATA/, before: ADC/) */
#define REC_DIR     "DATA"
#define REC_DIR_OLD "ADC"

/* Create recording directory on SD card for ADS131 data */
static data_logger_result_t create_ads_directory(void)
{
    char ads_dir_path[MAX_PATH];
    int ret;

    snprintf(ads_dir_path, sizeof(ads_dir_path), "%s/" REC_DIR, disk_mount_pt);

    /* Check if directory already exists */
    struct fs_dirent stat;
    ret = fs_stat(ads_dir_path, &stat);
    if (ret != 0)
    {
        /* fw <= 0.0.95 recorded into ADC/: move it once, so earlier recordings
         * (e.g. the sample recording from the production test) stay together. */
        char old_dir[24]; /* "/SD:/ADC" */
        snprintf(old_dir, sizeof(old_dir), "%s/" REC_DIR_OLD, disk_mount_pt);
        if (fs_stat(old_dir, &stat) == 0 && stat.type == FS_DIR_ENTRY_DIR)
        {
            int rr = fs_rename(old_dir, ads_dir_path);
            LOG_INF("Renamed %s -> %s: %d", old_dir, ads_dir_path, rr);
            ret = fs_stat(ads_dir_path, &stat);
        }
    }
    if (ret == 0 && stat.type == FS_DIR_ENTRY_DIR)
    {
        LOG_INF("Recording directory already exists: %s", ads_dir_path);
        return DATA_LOGGER_OK;
    }

    /* Create the directory */
    ret = fs_mkdir(ads_dir_path);
    if (ret != 0)
    {
        LOG_ERR("Failed to create recording directory %s: %d", ads_dir_path, ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    LOG_INF("Created recording directory: %s", ads_dir_path);
    return DATA_LOGGER_OK;
}

/* Write a complete EDF header (2304 bytes) to a freshly created file.
 * Uses a 256-byte stack buffer reused per signal to stay within stack budget.
 * General header layout (all fields ASCII, space-padded, no null terminator):
 *   version(8) patient(80) recording(80) startdate(8) starttime(8)
 *   header_bytes(8) reserved(44) num_data_records(8) record_duration(8) num_signals(4)
 * Signal header layout per channel (256 bytes):
 *   label(16) transducer(80) dim(8) phys_min(8) phys_max(8)
 *   dig_min(8) dig_max(8) prefilter(80) num_samples(8) reserved(32)
 */
static data_logger_result_t write_edf_header(const char *full_path)
{
    struct fs_file_t file;
    char buf[256];
    int ret;
    ssize_t written;

    fs_file_t_init(&file);
    ret = sd_card_safe_open(&file, full_path, FS_O_WRITE | FS_O_APPEND);
    if (ret != 0)
    {
        LOG_ERR("EDF header: open failed %d", ret);
        return DATA_LOGGER_FILE_ERROR;
    }

    /* ── General header (256 bytes) ─────────────────────────────────────── */
    memset(buf, ' ', sizeof(buf));
    /* version (8): BDF identifier — byte 0 = 0xFF, then "BIOSEMI" */
    buf[0] = (char)0xFF;
    memcpy(&buf[1], "BIOSEMI", 7);
    /* local patient id (80): offset 8 */
    memcpy(&buf[8], "X X X X", 7);
    /* local recording id (80): offset 88 */
    memcpy(&buf[88], "Startdate X X X X", 17);
    /* startdate/starttime (8+8) at offsets 168/176: the real clock when the
     * phone has already synced wall time (the recording starts ~now relative
     * to this header write); otherwise the EDF/EDF+ placeholder for unknown
     * date — later patched in place by the 0x40 time sync. */
    {
        data_logger_wall_time_t wt;
        if (data_logger_wall_time_now(&wt))
        {
            char ds[17]; /* 16 chars "dd.mm.yyhh.mm.ss" + NUL */
            snprintf(ds, sizeof(ds), "%02u.%02u.%02u%02u.%02u.%02u",
                     wt.day, wt.month, (unsigned)(wt.year % 100),
                     wt.hour, wt.minute, wt.second);
            memcpy(&buf[168], ds, 16);
        }
        else
        {
            memcpy(&buf[168], "01.01.85", 8);
            memcpy(&buf[176], "00.00.00", 8);
        }
    }
    /* bytes in header (8): offset 184 — "3072    " (256 general + 11 × 256 signal) */
    memcpy(&buf[184], "3072    ", 8);
    /* reserved (44): offset 192 — "24BIT" marks a plain BDF file (BioSemi) */
    memcpy(&buf[192], "24BIT", 5);
    /* num_data_records (8): offset 236 — "-1      " (unknown until finalize) */
    memcpy(&buf[236], "-1      ", 8);
    /* record_duration (8): offset 244 — "1       " (placeholder, updated at finalize) */
    memcpy(&buf[244], "1       ", 8);
    /* num_signals (4): offset 252 — "11  " (8 ADS + Battery + Light + Button) */
    memcpy(&buf[252], "11  ", 4);

    written = sd_card_safe_write(&file, buf, 256);
    if (written != 256)
    {
        LOG_ERR("EDF header: general header write failed (%zd)", written);
        sd_card_safe_close(&file);
        return DATA_LOGGER_WRITE_ERROR;
    }

    /* ── Signal headers (11 × 256 bytes) ───────────────────────────────── */
    /* EDF field order: label(16), transducer(80), dim(8), phys_min(8), phys_max(8),
     * dig_min(8), dig_max(8), prefilter(80), nr_samples(8), reserved(32).
     * All fields for all signals written field-by-field (not signal-by-signal).
     * Signals 0-7: ADS131M08 EEG channels, 250 samples/record.
     * Signal 8:  Battery voltage (mV), 1 sample/record.
     * Signal 9:  Brightness/Light (mV), 1 sample/record.
     * Signal 10: Button event code (0..13, button_event_t), 1 sample/record.
     */
    static const int field_widths[] = {16, 80, 8, 8, 8, 8, 8, 80, 8, 32};
    int total_fields = 10;
    int ns = EDF_NUM_SIGNALS; /* 11 */

    /* EEG physical full scale depends on the active PGA gain:
     *   ±(1.2 V / gain) = ±(1200000 / gain) µV
     * e.g. ×4 → ±300000 µV, ×128 → ±9375 µV. Digital range is the full
     * 24-bit two's-complement span (−8388608 … 8388607). A reader recovers
     * µV via slope = (phys_max − phys_min)/(dig_max − dig_min), so it stays
     * correct for any gain without hardcoding. */
    /* Per channel: the gain may differ (ADS_GAIN_CHn), so the physical range is
     * computed per signal below. */
    char phys_min_str[ADS131_MAX_CHANNELS][9], phys_max_str[ADS131_MAX_CHANNELS][9];
    for (int c = 0; c < ADS131_MAX_CHANNELS; c++)
    {
        long phys_fs_uv = 1200000L / (long)system_config_ads_gain_for_channel(c);
        snprintf(phys_min_str[c], sizeof(phys_min_str[c]), "%ld", -phys_fs_uv);
        snprintf(phys_max_str[c], sizeof(phys_max_str[c]), "%ld", phys_fs_uv);
    }

    for (int f = 0; f < total_fields; f++)
    {
        int fw = field_widths[f];
        for (int ch = 0; ch < ns; ch++)
        {
            memset(buf, ' ', fw);
            bool is_slow = (ch >= ADS131_MAX_CHANNELS);
            int slow_idx = ch - ADS131_MAX_CHANNELS; /* 0=Battery, 1=Light, 2=Button */

            switch (f)
            {
            case 0: /* label (16) */
            {
                char label[17];
                if (!is_slow)
                {
                    /* 0-based channel labels (ch0..ch7) to match the rest of
                     * the toolchain (app + Python). Was 1-based "EEG Ch1..8". */
                    snprintf(label, sizeof(label), "EEG Ch%d", ch);
                }
                else
                {
                    const char *slow_labels[] = {"Battery", "Light", "ButtonEvent"};
                    snprintf(label, sizeof(label), "%s", slow_labels[slow_idx]);
                }
                memcpy(buf, label, strlen(label));
                break;
            }
            case 1: /* transducer (80): montage text from config.txt CH_LABELn */
                if (!is_slow && g_config.ch_label[ch][0] != '\0')
                {
                    size_t l = strlen(g_config.ch_label[ch]);
                    memcpy(buf, g_config.ch_label[ch], l < 80 ? l : 80);
                }
                break;
            case 2: /* physical dim (8) */
                if (!is_slow)
                {
                    memcpy(buf, "uV", 2);
                }
                else if (slow_idx < 2) /* Battery, Light */
                {
                    memcpy(buf, "mV", 2);
                }
                /* Button: spaces (dimensionless) */
                break;
            case 3: /* physical min (8) */
                if (!is_slow)       { memcpy(buf, phys_min_str[ch], strlen(phys_min_str[ch])); }
                else                { memcpy(buf, "0", 1); }
                break;
            case 4: /* physical max (8) */
                if (!is_slow)       { memcpy(buf, phys_max_str[ch], strlen(phys_max_str[ch])); }
                else if (slow_idx == 0) { memcpy(buf, "5000", 4); } /* Battery 0-5000 mV */
                else if (slow_idx == 1) { memcpy(buf, "3600", 4); } /* Light  0-3600 mV */
                else                    { memcpy(buf, "15", 2); }   /* Button event 0-15 */
                break;
            case 5: /* digital min (8) — full 24-bit span */
                if (!is_slow) { memcpy(buf, "-8388608", 8); }
                else          { memcpy(buf, "0", 1); }
                break;
            case 6: /* digital max (8) — full 24-bit span */
                if (!is_slow)       { memcpy(buf, "8388607", 7); }
                else if (slow_idx == 0) { memcpy(buf, "5000", 4); }
                else if (slow_idx == 1) { memcpy(buf, "3600", 4); }
                else                    { memcpy(buf, "15", 2); }
                break;
            case 7: /* prefiltering (80) — spaces */
                break;
            case 8: /* nr of samples (8) */
                if (!is_slow) { memcpy(buf, "250", 3); }
                else          { memcpy(buf, "1", 1); }
                break;
            case 9: /* reserved (32) — spaces */
                break;
            }
            written = sd_card_safe_write(&file, buf, fw);
            if (written != fw)
            {
                LOG_ERR("EDF header: signal field %d ch %d write failed (%zd)", f, ch, written);
                sd_card_safe_close(&file);
                return DATA_LOGGER_WRITE_ERROR;
            }
        }
    }

    sd_card_safe_sync(&file);
    sd_card_safe_close(&file);
    LOG_INF("EDF header written (%d bytes, %d signals) to %s", EDF_HEADER_BYTES, EDF_NUM_SIGNALS, full_path);
    return DATA_LOGGER_OK;
}

/* Find next available ADS131 log file number */
static uint32_t find_next_ads131_log_number(void)
{
    struct fs_dir_t dir;
    struct fs_dirent entry;
    char dir_path[MAX_PATH];
    uint32_t highest_number = 0;
    int ret;

    snprintf(dir_path, sizeof(dir_path), "%s/" REC_DIR, disk_mount_pt);

    fs_dir_t_init(&dir);
    ret = fs_opendir(&dir, dir_path);
    if (ret != 0)
    {
        LOG_WRN("Cannot open recording directory, starting from 0001");
        return 1;
    }

    while (fs_readdir(&dir, &entry) == 0)
    {
        if (entry.name[0] == '\0')
            break;

        /* Look for files matching 8.3 pattern: ADNNNN.EDF (case-insensitive) */
        if (entry.type == FS_DIR_ENTRY_FILE)
        {
            char upper[32];
            size_t n = strlen(entry.name);
            if (n >= sizeof(upper))
                n = sizeof(upper) - 1;
            for (size_t i = 0; i < n; i++)
                upper[i] = (char)toupper((unsigned char)entry.name[i]);
            upper[n] = '\0';

            /* Expect AD####.BDF (new), AD####.EDF (legacy) or the fw-0.0.64
             * wall-time form AD####_YYYYMMDD_HHMM.BDF. Count them all so the
             * numbering stays monotonic and we never overwrite existing data
             * — a renamed file that fell out of this scan would let its
             * number be REUSED on the next recording. */
            uint32_t num = 0;
            const char *dot = strrchr(upper, '.');
            if (sscanf(upper, "AD%4u", &num) == 1 && dot != NULL &&
                (strcmp(dot + 1, "BDF") == 0 || strcmp(dot + 1, "EDF") == 0))
            {
                if (num > highest_number)
                    highest_number = num;
            }
        }
    }

    fs_closedir(&dir);

    LOG_INF("Next ADS131 log number: %u", highest_number + 1);
    return highest_number + 1;
}

/* Low-level writer: convert one staging chunk to an EDF data record and append it.
 * Each call writes exactly one EDF record (ADS131_WRITE_CHUNK_SIZE samples per channel).
 * The file must already exist with the EDF header pre-written.
 */
static data_logger_result_t write_ads131_data_to_sd(const ads131_sample_record_t *data,
                                                    uint32_t samples)
{
    LOG_DBG("EDF write: entering write (samples=%u)", (unsigned)samples);
    char full_path[MAX_PATH];
    ssize_t written;
    int ret;

    /* Track first/last timestamps for sample-rate calibration at finalization */
    if (!g_edf_first_sample_seen && samples > 0)
    {
        g_edf_first_sample_num   = data[0].sample_num;
        g_edf_first_timestamp_us = data[0].timestamp_us;
        g_edf_first_sample_seen  = true;
    }
    if (samples > 0)
    {
        g_edf_last_timestamp_us = data[samples - 1].timestamp_us;
    }

    /* Build BDF data record into BSS buffer: channel-major layout.
     * All samples for channel 0 first, then channel 1, etc.
     * Each sample is the FULL 24-bit ADC value (no >>8 truncation), stored as
     * 3 bytes little-endian two's complement. */
    for (int ch = 0; ch < ADS131_MAX_CHANNELS; ch++)
    {
        for (uint32_t s = 0; s < samples; s++)
        {
            size_t off = ((size_t)ch * samples + s) * BDF_BPS;
            bdf_put_i24(g_edf_write_buf, off, data[s].channels[ch]);
        }
    }

    /* Append slow-channel samples (1 per record) after ADS data.
     * One-shot SAADC read here gives live values; the read is embedded in the
     * SD write window so any supply noise is masked by the larger SD current.
     * The periodic SAADC timer remains stopped — only this one-shot fires.
     */
    size_t slow_base = (size_t)ADS131_MAX_CHANNELS * samples * BDF_BPS;
    uint8_t btn_code = g_edf_button_event_code;
    g_edf_button_event_code = 0;
    {
        nordic_adc_sample_t snap;
        if (nordic_adc_sample_once(&snap) == NORDIC_ADC_OK)
        {
            g_edf_battery_mv    = (int16_t)snap.voltage_mv[4]; /* AIN4 = battery */
            g_edf_brightness_mv = (int16_t)snap.voltage_mv[0]; /* AIN0 = brightness */
            charge_guard_report_vbus_mv(snap.voltage_mv[5]); /* AIN5 = VBUS (E7) */
        }
    }
    bdf_put_i24(g_edf_write_buf, slow_base + 0 * BDF_BPS, (int32_t)g_edf_battery_mv);
    bdf_put_i24(g_edf_write_buf, slow_base + 1 * BDF_BPS, (int32_t)g_edf_brightness_mv);
    bdf_put_i24(g_edf_write_buf, slow_base + 2 * BDF_BPS, (int32_t)btn_code);

    /* Build full path */
    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, g_ads131_filename);

    /* Take the SD lock ONLY for this chunk's write (released at the end / on
     * every error return), NOT across open..close. This is what frees the lock
     * for the backfill reader between chunks. */
    if (sd_card_manager_lock(2000) != 0)
    {
        LOG_ERR("EDF write: SD lock timeout");
        return DATA_LOGGER_ERROR; /* chunk stays buffered; writer retries */
    }

    /* Lazily open the data file ONCE and keep it open (raw fs_open, FS_O_RDWR,
     * NO FS_O_APPEND — append would make fs_write re-seek to EOF every call). */
    if (!g_edf_data_file_open)
    {
        fs_file_t_init(&g_edf_data_file);
        ret = fs_open(&g_edf_data_file, full_path, FS_O_RDWR);
        if (ret != 0)
        {
            LOG_ERR("EDF write: open failed for %s: %d", full_path, ret);
            sd_card_manager_unlock();
            return DATA_LOGGER_FILE_ERROR;
        }
        g_edf_data_file_open   = true;
        g_edf_data_file_at_end = false;
    }
    /* One-time seek to end (file is tiny at recording start -> O(1)). After this
     * the fptr stays at EOF across chunks, so subsequent fs_write appends with
     * NO seek. */
    if (!g_edf_data_file_at_end)
    {
        if (fs_seek(&g_edf_data_file, 0, FS_SEEK_END) != 0)
        {
            LOG_ERR("EDF write: initial seek-to-end failed");
            (void)fs_close(&g_edf_data_file);
            g_edf_data_file_open = false;
            sd_card_manager_unlock();
            return DATA_LOGGER_FILE_ERROR;
        }
        g_edf_data_file_at_end = true;
    }

    /* Lazily open the sidecar sample-number index (best-effort; the BDF is the
     * source of truth). Same kept-open + seek-to-end-once discipline as the BDF.
     * On any failure we mark it failed and never touch it again this recording
     * (the reader then falls back to the positional formula), but we NEVER fail
     * the BDF write because of the index. */
    if (!g_edf_snx_open && !g_edf_snx_failed)
    {
        char snx_path[MAX_PATH];
        snprintf(snx_path, sizeof(snx_path), "%s/" REC_DIR "/AD%04u.SNX",
                 disk_mount_pt, (unsigned)g_ads131_log_number);
        fs_file_t_init(&g_edf_snx_file);
        if (fs_open(&g_edf_snx_file, snx_path, FS_O_CREATE | FS_O_RDWR) == 0 &&
            fs_seek(&g_edf_snx_file, 0, FS_SEEK_END) == 0)
        {
            g_edf_snx_open   = true;
            g_edf_snx_at_end = true;
        }
        else
        {
            LOG_WRN("SNX index open failed for %s — backfill falls back to "
                    "positional addressing", snx_path);
            (void)fs_close(&g_edf_snx_file);
            g_edf_snx_open    = false;
            g_edf_snx_failed  = true;
        }
    }

    /* Append one BDF data record (6009 bytes: 8×250×3 ADS + 3×1×3 slow) */
    size_t data_size = (size_t)EDF_RECORD_BYTES;
    LOG_DBG("EDF write: writing %zu bytes", data_size);
    written = fs_write(&g_edf_data_file, g_edf_write_buf, data_size);
    if (written != (ssize_t)data_size)
    {
        LOG_ERR("EDF write error: wrote %zd bytes, expected %zu", written, data_size);
        /* fptr may be mid-record; drop the handle so the next chunk re-opens and
         * re-seeks cleanly to EOF. */
        (void)fs_sync(&g_edf_data_file);
        (void)fs_close(&g_edf_data_file);
        g_edf_data_file_open   = false;
        g_edf_data_file_at_end = false;
        sd_card_manager_unlock();
        return DATA_LOGGER_WRITE_ERROR;
    }
    g_edf_data_records++;

    /* Sync every SD_SYNC_INTERVAL_S records (default 10) instead of every
     * record: the card's ~300 ms programming phase after f_sync couples into
     * high-impedance inputs (reduces SD write interference). f_sync
     * writes the data sector + FAT + directory entry/objsize; a sudden power-off
     * leaves a valid file with all SYNCED records, so up to the interval of the
     * SD copy can be lost (the app keeps its live copy). The .SNX index below
     * is synced in the same step and only AFTER the data, so it is never ahead
     * of the data. NOT an O(file-size) operation. */
    uint32_t sync_every = g_config.sd_sync_interval_s ? g_config.sd_sync_interval_s : 1;
    bool sync_now = (g_edf_data_records % sync_every) == 0;
    if (sync_now)
    {
        (void)fs_sync(&g_edf_data_file);
    }

    /* Append this record's REAL first sample_num to the .SNX index (4 bytes LE),
     * AFTER the BDF record is synced so the index is never ahead of the data.
     * On any error, latch the failure and drop the handle: the index stops at
     * its last consistent length (still usable for earlier records) and the
     * reader falls back to positional addressing beyond it. */
    if (g_edf_snx_open && samples > 0)
    {
        uint32_t rec_first_sn = data[0].sample_num;
        uint8_t  snx_entry[EDF_SNX_ENTRY_BYTES] = {
            (uint8_t)(rec_first_sn & 0xFF),
            (uint8_t)((rec_first_sn >> 8) & 0xFF),
            (uint8_t)((rec_first_sn >> 16) & 0xFF),
            (uint8_t)((rec_first_sn >> 24) & 0xFF),
        };
        ssize_t snw = fs_write(&g_edf_snx_file, snx_entry, EDF_SNX_ENTRY_BYTES);
        if (snw == (ssize_t)EDF_SNX_ENTRY_BYTES)
        {
            if (sync_now)
            {
                (void)fs_sync(&g_edf_snx_file);
            }
        }
        else
        {
            LOG_WRN("SNX index write failed (%zd) — disabling index for this "
                    "recording", snw);
            (void)fs_sync(&g_edf_snx_file);
            (void)fs_close(&g_edf_snx_file);
            g_edf_snx_open   = false;
            g_edf_snx_at_end = false;
            g_edf_snx_failed = true;
        }
    }

    /* Refresh num_data_records only every EDF_HDR_UPDATE_INTERVAL chunks. On a
     * crash before the next refresh the header count lags <=N records;
     * finalize_edf_header() corrects it on a clean stop, and an un-finalized
     * file is recoverable as (file_size - EDF_HEADER_BYTES) / EDF_RECORD_BYTES.
     *
     * 0.0.92: through a SECOND, short-lived handle. The kept-open data handle
     * used to seek to the header and back to EOF; with FF_USE_FASTSEEK=0 the
     * way back walks the whole cluster chain — O(file size), once a minute,
     * holding the SD lock inside the writer. On a ~15 h night file that stall
     * outlasts the ring's ~1 s headroom and the samples right after the chunk
     * are lost (field 2026-09-28: 14 samples missing on SD, a clean gap at a
     * record start). The second handle only touches the first cluster; the
     * data handle's fptr stays at EOF. Its close rewrites the directory entry
     * with the size it saw at open — never larger than the real size; the data
     * handle's next fs_sync writes the current size again. Best-effort: if no
     * file slot is free (CONFIG_FS_FATFS_NUM_FILES) we just try next minute. */
    if (++g_edf_hdr_dirty_chunks >= EDF_HDR_UPDATE_INTERVAL)
    {
        g_edf_hdr_dirty_chunks = 0;
        char count_str[9];
        snprintf(count_str, sizeof(count_str), "%-8u", g_edf_data_records);
        /* Make the data handle's size durable first, so the header handle's
         * close can never write back a size older than the last sync. */
        (void)fs_sync(&g_edf_data_file);
        struct fs_file_t hf;
        fs_file_t_init(&hf);
        if (fs_open(&hf, full_path, FS_O_RDWR) == 0)
        {
            if (fs_seek(&hf, EDF_NUM_REC_OFFSET, FS_SEEK_SET) == 0)
            {
                (void)fs_write(&hf, count_str, 8);
            }
            (void)fs_close(&hf);
        }
        else
        {
            g_edf_hdr_dirty_chunks = EDF_HDR_UPDATE_INTERVAL - 1; /* retry next chunk */
        }
    }

    sd_card_manager_unlock();
    return DATA_LOGGER_OK;
}

/* Seek back into the EDF general header and update num_data_records and record_duration
 * with the true values computed from hardware timestamps.  Called once at end of recording.
 */
static void finalize_edf_header(void)
{
    char full_path[MAX_PATH];
    char num_rec_str[9];
    char dur_str[9];

    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, g_ads131_filename);
    snprintf(num_rec_str, sizeof(num_rec_str), "%-8u", g_edf_data_records);

    /* Compute actual record duration from hardware timestamps.
     * Each record holds ADS131_WRITE_CHUNK_SIZE samples.
     * total_time covers from first sample of record 0 to last sample of final record.
     */
    double rec_dur = 1.0;
    if (g_edf_first_sample_seen && g_edf_data_records > 0 &&
        g_edf_last_timestamp_us > g_edf_first_timestamp_us)
    {
        double total_us      = (double)(g_edf_last_timestamp_us - g_edf_first_timestamp_us);
        double total_samples = (double)(g_edf_data_records * ADS131_WRITE_CHUNK_SIZE);
        double actual_hz     = total_samples * 1e6 / total_us;
        rec_dur              = (double)ADS131_WRITE_CHUNK_SIZE / actual_hz;
    }
    /* Format: up to 6 significant figures, left-aligned in 8-char ASCII field */
    snprintf(dur_str, sizeof(dur_str), "%-8.6g", rec_dur);

    struct fs_file_t file;
    fs_file_t_init(&file);
    int ret = sd_card_safe_open(&file, full_path, FS_O_RDWR);
    if (ret != 0)
    {
        LOG_WRN("EDF finalize: open failed %d — header not updated", ret);
        return;
    }

    if (fs_seek(&file, EDF_NUM_REC_OFFSET, FS_SEEK_SET) == 0)
    {
        sd_card_safe_write(&file, num_rec_str, 8);
    }
    if (fs_seek(&file, EDF_REC_DUR_OFFSET, FS_SEEK_SET) == 0)
    {
        sd_card_safe_write(&file, dur_str, 8);
    }

    /* Truncate to actual recorded size — frees the pre-allocated space */
    off_t actual_size = (off_t)EDF_HEADER_BYTES + (off_t)g_edf_data_records * EDF_RECORD_BYTES;
    if (fs_truncate(&file, actual_size) != 0)
    {
        LOG_WRN("EDF finalize: truncate failed (pre-allocated space may remain)");
    }

    sd_card_safe_sync(&file);
    sd_card_safe_close(&file);
    LOG_INF("EDF finalized: %u records, duration=%s s (~%.1f Hz)",
            g_edf_data_records, dur_str, (double)ADS131_WRITE_CHUNK_SIZE / rec_dur);
}

/* Pre-allocate EDF file clusters using f_expand() so that FAT cluster-chain writes
 * (which cause ~8-second-periodic power-supply transients picked up by the 128x-gain
 * ECG front-end) happen once at file creation rather than during recording. */
static void preallocate_edf_file(const char *full_path)
{
#if defined(CONFIG_FS_FATFS_EXTRA_NATIVE_API)
    struct fs_file_t file;
    fs_file_t_init(&file);

    int ret = sd_card_safe_open(&file, full_path, FS_O_RDWR);
    if (ret != 0)
    {
        LOG_WRN("EDF pre-alloc: open failed %d — spikes may occur every ~8 s", ret);
        return;
    }

    /* Reserve cluster chain for up to 8 h (non-contiguous, fast: only FAT writes, no data written).
     * If the SD card has less free space, fall back to shorter durations gracefully. */
    static const struct { uint32_t hours; const char *label; } prealloc_tiers[] = {
        { 12, "12 h" }, { 8, "8 h" }, { 4, "4 h" }, { 2, "2 h" }, { 1, "1 h" }
    };
    FRESULT fres = FR_DENIED;
    FSIZE_t prealloc = 0;
    for (int i = 0; i < (int)ARRAY_SIZE(prealloc_tiers) && fres != FR_OK; i++) {
        prealloc = (FSIZE_t)EDF_HEADER_BYTES +
                   (FSIZE_t)prealloc_tiers[i].hours * 3600U * EDF_RECORD_BYTES;
        fres = f_expand((FIL *)file.filep, prealloc, 0);
        if (fres == FR_OK) {
            LOG_INF("EDF: pre-allocated %u MB (%s capacity) — no cluster-allocation spikes",
                    (unsigned)(prealloc >> 20), prealloc_tiers[i].label);
        }
    }
    if (fres != FR_OK)
    {
        LOG_WRN("EDF: f_expand failed (%d) — cluster-allocation spikes may occur every ~8 s", fres);
    }

    sd_card_safe_sync(&file);
    sd_card_safe_close(&file);
#else
    LOG_WRN("EDF pre-alloc: CONFIG_FS_FATFS_EXTRA_NATIVE_API not enabled");
#endif
}

/* Background thread for ADS131 SD card writes */
static void ads131_writer_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("ADS131 writer thread started (stack=%u)", (unsigned)K_THREAD_STACK_SIZEOF(ads131_writer_stack));

    while (1)
    {
        /* Wait for signal to write data */
        int sem_rc = k_sem_take(&ads131_writer_sem, K_FOREVER);
        if (sem_rc != 0)
        {
            LOG_WRN("ADS131 writer: k_sem_take returned %d", sem_rc);
        }

        /* If paused, wait until resumed */
        while (g_ads131_paused)
        {
            k_sleep(K_MSEC(100));
        }

        LOG_DBG("ADS131 writer: wake to flush chunk (buffered=%u)", g_ads131_buffered_samples);

        /* Write data while we have enough buffered */
        while (1)
        {
            /* Check if we should pause */
            if (g_ads131_paused)
            {
                break;
            }

            /* Stage a chunk without blocking producers during SD I/O */
            /* NOTE: Use static BSS buffer to avoid stack overflow */

            /* Check availability */
            k_mutex_lock(&g_ads131_mutex, K_FOREVER);
            LOG_DBG("ADS131 writer: locked buffer (write=%u read=%u count=%u)",
                    g_ads131_write_index, g_ads131_read_index, g_ads131_buffered_samples);
            if (g_ads131_buffered_samples < ADS131_WRITE_CHUNK_SIZE)
            {
                k_mutex_unlock(&g_ads131_mutex);
                LOG_DBG("ADS131 writer: not enough samples to write (have=%u need=%u)",
                        g_ads131_buffered_samples, (unsigned)ADS131_WRITE_CHUNK_SIZE);
                break; /* Not enough data */
            }
            /* Copy out one contiguous chunk starting at read_index */
            uint32_t idx = g_ads131_read_index;
            for (uint32_t i = 0; i < ADS131_WRITE_CHUNK_SIZE; i++)
            {
                g_ads131_staging[i] = g_ads131_buffer[idx];
                idx = (idx + 1) % ADS131_BUFFER_SIZE;
            }
            k_mutex_unlock(&g_ads131_mutex);
            LOG_DBG("ADS131 writer: staged %u samples for SD write (static buffer)", (unsigned)ADS131_WRITE_CHUNK_SIZE);

            /* Perform SD write without holding buffer mutex */
            uint32_t wr_t0 = k_uptime_get_32();
            data_logger_result_t ret = write_ads131_data_to_sd(g_ads131_staging, ADS131_WRITE_CHUNK_SIZE);
            uint32_t wr_ms = k_uptime_get_32() - wr_t0;
            /* The ring holds ~2 s; a chunk write that takes > 0.5 s eats most
             * of the headroom — make such stalls visible. */
            if (wr_ms > 500)
            {
                LOG_WRN("ADS131 writer: chunk write took %u ms (ret=%d, sn %u..)",
                        wr_ms, (int)ret, g_ads131_staging[0].sample_num);
            }

            if (ret != DATA_LOGGER_OK)
            {
                g_ads131_write_error_count++;
                g_ads131_consecutive_failures++;
                if (g_ads131_write_error_count <= 3 || (g_ads131_write_error_count % 100 == 0))
                {
                    LOG_ERR("ADS131 background write failed: %d (error count: %u, consec: %u, buffered: %u)",
                            ret, g_ads131_write_error_count,
                            g_ads131_consecutive_failures,
                            g_ads131_buffered_samples);
                }

                /* v0.0.53: after a streak of consecutive failures the most
                 * likely state is a wedged fs handle / FAT cache → unmount
                 * + remount and try again. This was added after a real
                 * overnight where the writer dropped ~2.5 h silently with
                 * no recovery. */
                #define SD_REMOUNT_THRESHOLD 5
                if (g_ads131_consecutive_failures == SD_REMOUNT_THRESHOLD)
                {
                    g_ads131_sd_unhealthy = true;
                    LOG_WRN("ADS131 writer: %u consecutive failures — attempting fs remount",
                            (unsigned)g_ads131_consecutive_failures);
                    log_sd_health_event("REMOUNT_ATTEMPT");
                    g_ads131_total_remounts_attempted++;
                    /* Drop the kept-open data handle BEFORE remount: fs_unmount
                     * invalidates the FIL, so it must not be reused afterwards.
                     * Close while the volume is still mounted (frees the FIL slab
                     * slot); the next chunk re-opens on the fresh mount. */
                    edf_close_data_file();
                    int rc = try_remount_filesystem();
                    if (rc == 0)
                    {
                        g_ads131_total_remounts_succeeded++;
                        LOG_WRN("ADS131 writer: remount OK");
                        log_sd_health_event("REMOUNT_OK");
                    }
                    else
                    {
                        LOG_ERR("ADS131 writer: remount failed (%d)", rc);
                        log_sd_health_event("REMOUNT_FAIL");
                    }
                }

                /* Different retry strategies based on error type */
                if (ret == DATA_LOGGER_ERROR)
                {
                    /* SD card busy - short wait and continue trying */
                    k_sleep(K_MSEC(500));
                    /* Try again later without consuming from buffer */
                }
                else if (ret == DATA_LOGGER_FILE_ERROR)
                {
                    /* File system error - longer wait */
                    k_sleep(K_MSEC(2000));
                    break; /* Exit loop to wait for next semaphore signal */
                }
                else
                {
                    /* Other errors - long wait */
                    k_sleep(K_MSEC(5000));
                    break;
                }
            }
            else
            {
                /* On success, commit the chunk (advance read index) under mutex */
                k_mutex_lock(&g_ads131_mutex, K_FOREVER);
                g_ads131_read_index = (g_ads131_read_index + ADS131_WRITE_CHUNK_SIZE) % ADS131_BUFFER_SIZE;
                g_ads131_buffered_samples -= ADS131_WRITE_CHUNK_SIZE;

                /* Track successful write */
                if (!g_ads131_first_write_successful)
                {
                    // LOG_INF("ADS131 first write successful! Buffered remaining: %u",
                    //         g_ads131_buffered_samples);
                    g_ads131_first_write_successful = true;
                }
                /* Reset error counters on success — keeps the unhealthy flag
                 * clear when we recover from a transient stall. */
                g_ads131_write_error_count = 0;
                if (g_ads131_consecutive_failures > 0)
                {
                    LOG_INF("ADS131 writer: recovered after %u consec failures",
                            (unsigned)g_ads131_consecutive_failures);
                    log_sd_health_event("RECOVERED");
                    g_ads131_consecutive_failures = 0;
                    g_ads131_sd_unhealthy = false;
                }
                g_ads131_last_write_uptime_ms = k_uptime_get_32();
                uint32_t ovf_n = g_ads131_ovf_count;
                uint32_t ovf_a = g_ads131_ovf_first_sn;
                uint32_t ovf_b = g_ads131_ovf_last_sn;
                g_ads131_ovf_count = 0;
                k_mutex_unlock(&g_ads131_mutex);
                if (ovf_n > 0)
                {
                    LOG_WRN("ADS131 ring overflow: %u samples NOT written to SD (sn %u..%u, total %u)",
                            ovf_n, ovf_a, ovf_b, g_ads131_dropped_samples);
                }
            }
        }

        /* Backfill asked for the newest records (data_logger_sync_ads131_data):
         * with SD_SYNC_INTERVAL_S > 1 the records just drained above may not be
         * synced yet, and the backfill reader only sees synced records (FatFS
         * freezes objsize at open). Sync both handles now, data before index. */
        if (g_edf_sync_request && !g_ads131_paused)
        {
            g_edf_sync_request = false;
            if (sd_card_manager_lock(1000) == 0)
            {
                if (g_edf_data_file_open)
                {
                    (void)fs_sync(&g_edf_data_file);
                }
                if (g_edf_snx_open)
                {
                    (void)fs_sync(&g_edf_snx_file);
                }
                sd_card_manager_unlock();
            }
        }

        /* Small delay before next check */
        k_sleep(K_MSEC(100));
    }
}

/* Initialize ADS131M08 data logging system */
data_logger_result_t data_logger_init_ads131_logging(void)
{
    data_logger_result_t ret;

    if (!g_sd_info.is_mounted)
    {
        LOG_ERR("SD card not mounted - cannot initialize ADS131 logging");
        return DATA_LOGGER_NO_SD_CARD;
    }

    LOG_INF("Initializing ADS131M08 data logging system");
    LOG_INF("Mount point: '%s'", disk_mount_pt);

    /* Initialize mutex and semaphore */
    k_mutex_init(&g_ads131_mutex);
    k_sem_init(&ads131_writer_sem, 0, 1);

    /* Create ADS directory if it doesn't exist */
    ret = create_ads_directory();
    if (ret != DATA_LOGGER_OK)
    {
        return ret;
    }

    /* Find next log file number */
    g_ads131_log_number = find_next_ads131_log_number();

    /* Generate the BDF filename. Plain 8.3 stem when wall time is unknown;
     * if the phone has already synced time (recording started while
     * connected), embed date+time right away — no later rename needed.
     * Either way the name starts with the AD#### stem every consumer
     * (scan parser, .SNX derivation) keys on. */
    {
        data_logger_wall_time_t wt;
        if (data_logger_wall_time_now(&wt))
        {
            snprintf(g_ads131_filename, sizeof(g_ads131_filename),
                     REC_DIR "/AD%04u_%04u%02u%02u_%02u%02u.BDF",
                     g_ads131_log_number, (unsigned)wt.year, wt.month, wt.day,
                     wt.hour, wt.minute);
        }
        else
        {
            snprintf(g_ads131_filename, sizeof(g_ads131_filename),
                     REC_DIR "/AD%04u.BDF", g_ads131_log_number);
        }
    }

    LOG_INF("ADS131 logging initialized:");
    LOG_INF("  File: %s", g_ads131_filename);
    LOG_INF("  Full path: %s/%s", disk_mount_pt, g_ads131_filename);
    LOG_INF("  Directory: %s/" REC_DIR, disk_mount_pt);
    LOG_INF("  Buffer size: %d samples", ADS131_BUFFER_SIZE);
    LOG_INF("  EDF record size: %d bytes", EDF_RECORD_BYTES);
    LOG_INF("  EDF header size: %d bytes", EDF_HEADER_BYTES);

    /* Reset circular buffer and status */
    g_ads131_write_index = 0;
    g_ads131_read_index = 0;
    g_ads131_buffered_samples = 0;
    g_ads131_first_write_successful = false;
    g_ads131_write_error_count = 0;
    g_ads131_dropped_samples = 0;
    g_ads131_ovf_count = 0;

    /* Reset EDF state */
    edf_close_data_file(); /* drop any stale handle from a previous recording */
    g_edf_data_records     = 0;
    g_edf_first_sample_num = 0;
    g_edf_first_sample_seen = false;
    g_edf_snx_failed       = false; /* re-enable the sidecar index for this run */
    g_edf_first_timestamp_us = 0;
    g_edf_last_timestamp_us  = 0;
    g_edf_button_event_code = 0;
    /* g_edf_battery_mv / g_edf_brightness_mv set by caller via data_logger_set_slow_channels() */

    /* Create background writer thread */
    k_thread_create(&ads131_writer_thread, ads131_writer_stack,
                    K_THREAD_STACK_SIZEOF(ads131_writer_stack),
                    ads131_writer_thread_entry, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
    k_thread_name_set(&ads131_writer_thread, "ads131_writer");

    g_ads131_logging_initialized = true;

    LOG_INF("ADS131 background writer thread created");

    /* Pre-create the EDF file and write its header */
    {
        struct fs_file_t file;
        char full_path[MAX_PATH];
        fs_file_t_init(&file);
        snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, g_ads131_filename);

        /* Remove any ORPHAN sample-number index for this log number before the
         * recording's first .SNX append. find_next_ads131_log_number() only
         * scans .BDF/.EDF, so a reused number whose .BDF was deleted (e.g. card
         * pulled + file removed on a PC) could leave a stale AD####.SNX; the
         * writer's lazy-open uses FS_O_CREATE (NOT truncating, so a mid-recording
         * remount-reopen keeps the index), which would then APPEND onto stale
         * entries -> index<->record misalignment -> WRONG backfill data with no
         * error. Unlinking here guarantees a fresh index per recording. */
        {
            char snx_path[MAX_PATH];
            snprintf(snx_path, sizeof(snx_path), "%s/" REC_DIR "/AD%04u.SNX",
                     disk_mount_pt, (unsigned)g_ads131_log_number);
            (void)fs_unlink(snx_path); /* best-effort; ENOENT is fine */
        }

        /* Create empty file first (truncate if it somehow exists) */
        int open_ret = sd_card_safe_open(&file, full_path, FS_O_CREATE | FS_O_WRITE);
        if (open_ret == 0)
        {
            (void)sd_card_safe_sync(&file);
            (void)sd_card_safe_close(&file);
        }
        else
        {
            LOG_WRN("Could not create EDF file (%d): %s", open_ret, full_path);
        }

        /* Write EDF header (2304 bytes) */
        data_logger_result_t hdr_ret = write_edf_header(full_path);
        if (hdr_ret != DATA_LOGGER_OK)
        {
            LOG_ERR("EDF header write failed: %d", hdr_ret);
        }

        /* Pre-allocate clusters to avoid mid-recording FAT writes (cluster-allocation spikes) */
        preallocate_edf_file(full_path);
    }

    return DATA_LOGGER_OK;
}

bool data_logger_is_ads131_logging_initialized(void)
{
    return g_ads131_logging_initialized;
}

/* Buffer ADS131M08 sample data for logging */
data_logger_result_t data_logger_buffer_ads131_sample(uint32_t sample_num,
                                                      uint64_t timestamp_us,
                                                      const int32_t *channels,
                                                      uint32_t status)
{
    data_logger_result_t ret = DATA_LOGGER_OK;

    if (!g_ads131_logging_initialized)
    {
        return DATA_LOGGER_ERROR;
    }

    if (!channels)
    {
        return DATA_LOGGER_ERROR;
    }

    /* Reject event/sentinel sample_nums: button (0xFFFFFFFE), speaker (0xFFFFFFFC),
     * mic (0xFFFFFFFB), nordic SAADC (0xFFFFFFFF) etc. These carry non-EEG payload
     * (event ids, mV values) in `channels[]`. Writing them into the EDF as if they
     * were ADS samples creates single-sample step artefacts that show up as
     * vertical broadband stripes in any spectrogram of the recording. Slow-channel
     * events (battery, brightness, button code) have their own dedicated EDF
     * channels and are filled by write_ads131_data_to_sd(), so dropping the
     * mis-routed copies here is safe. */
    if (sample_num >= 0xFFFFFFF0u)
    {
        return DATA_LOGGER_OK;
    }

    /* Lock buffer access */
    k_mutex_lock(&g_ads131_mutex, K_FOREVER);

    /* Check if buffer is full */
    if (g_ads131_buffered_samples >= ADS131_BUFFER_SIZE)
    {
        /* Buffer overflow - drop oldest sample */
        g_ads131_dropped_samples++;
        {
            const uint32_t lost_sn = g_ads131_buffer[g_ads131_read_index].sample_num;
            if (g_ads131_ovf_count == 0)
            {
                g_ads131_ovf_first_sn = lost_sn;
            }
            g_ads131_ovf_last_sn = lost_sn;
            g_ads131_ovf_count++;
        }
        g_ads131_read_index = (g_ads131_read_index + 1) % ADS131_BUFFER_SIZE;
        g_ads131_buffered_samples--;
        ret = DATA_LOGGER_ERROR; /* Indicate data loss */
    }

    /* Add sample to circular buffer */
    ads131_sample_record_t *record = &g_ads131_buffer[g_ads131_write_index];
    record->sample_num = sample_num;
    record->timestamp_us = timestamp_us;
    record->status = status;

    /* Copy all 8 channels */
    for (int i = 0; i < ADS131_MAX_CHANNELS; i++)
    {
        record->channels[i] = channels[i];
    }

    g_ads131_write_index = (g_ads131_write_index + 1) % ADS131_BUFFER_SIZE;
    g_ads131_buffered_samples++;

    /* Signal background thread if we have enough data for a chunk */
    if (g_ads131_buffered_samples >= ADS131_WRITE_CHUNK_SIZE)
    {
        k_sem_give(&ads131_writer_sem);
    }

    k_mutex_unlock(&g_ads131_mutex);

    return ret;
}

/* Drain all complete 250-sample chunks from the ring buffer to SD.
 * Safe to call during an active recording session — does NOT write a partial
 * padded record and does NOT touch the EDF header.
 * Used by the BLE backfill path to ensure the file is up-to-date before reading.
 */
data_logger_result_t data_logger_sync_ads131_data(void)
{
    if (!g_ads131_logging_initialized)
    {
        return DATA_LOGGER_ERROR;
    }
    g_edf_sync_request = true; /* writer drains AND fs_syncs (SD_SYNC_INTERVAL_S) */
    k_sem_give(&ads131_writer_sem);
    k_sleep(K_MSEC(300));
    return DATA_LOGGER_OK;
}

/* Flush any remaining ADS131M08 data to SD card and finalize EDF header */
data_logger_result_t data_logger_flush_ads131_data(void)
{
    if (!g_ads131_logging_initialized)
    {
        return DATA_LOGGER_ERROR;
    }

    /* Signal the background thread to drain any full 250-sample chunks */
    k_sem_give(&ads131_writer_sem);

    /* Give it time to drain full chunks */
    k_sleep(K_MSEC(500));

    /* Handle any remaining partial record (< 250 samples) directly.
     * We zero-pad to a full EDF record so the record count stays consistent.
     */
    k_mutex_lock(&g_ads131_mutex, K_FOREVER);
    uint32_t remaining = g_ads131_buffered_samples;
    if (remaining > 0)
    {
        memset(g_ads131_staging, 0, sizeof(g_ads131_staging));
        for (uint32_t i = 0; i < remaining; i++)
        {
            g_ads131_staging[i] =
                g_ads131_buffer[(g_ads131_read_index + i) % ADS131_BUFFER_SIZE];
        }
        g_ads131_read_index =
            (g_ads131_read_index + remaining) % ADS131_BUFFER_SIZE;
        g_ads131_buffered_samples = 0;
        k_mutex_unlock(&g_ads131_mutex);

        LOG_INF("EDF flush: writing padded partial record (%u samples)", remaining);
        write_ads131_data_to_sd(g_ads131_staging, ADS131_WRITE_CHUNK_SIZE);
    }
    else
    {
        k_mutex_unlock(&g_ads131_mutex);
    }

    /* Close the kept-open data handle (flushes it) so finalize_edf_header()
     * re-opens cleanly and writes the true final num_data_records. */
    edf_close_data_file();

    /* Update num_data_records and record_duration in the EDF header */
    finalize_edf_header();

    return DATA_LOGGER_OK;
}

/* Get current ADS131M08 log filename */
data_logger_result_t data_logger_get_ads131_filename(char *filename, size_t buffer_size)
{
    if (!filename || buffer_size == 0)
    {
        return DATA_LOGGER_ERROR;
    }

    if (!g_ads131_logging_initialized)
    {
        return DATA_LOGGER_ERROR;
    }

    strncpy(filename, g_ads131_filename, buffer_size - 1);
    filename[buffer_size - 1] = '\0';

    return DATA_LOGGER_OK;
}

void data_logger_pause_ads131_logging(bool pause)
{
    g_ads131_paused = pause;
    if (pause)
    {
        LOG_INF("ADS131 logging paused");
        /* Give the writer thread time to finish current write and see the pause flag */
        k_sleep(K_MSEC(200));
    }
    else
    {
        LOG_INF("ADS131 logging resumed");
        /* Wake up the writer thread to process any buffered data */
        k_sem_give(&ads131_writer_sem);
    }
}

/* Set battery and brightness values that will be embedded as slow EDF channels.
 * Call once just before starting ADS131 recording (before SAADC is stopped).
 * Values are held constant for the entire recording session.
 */
void data_logger_set_slow_channels(int16_t battery_mv, int16_t brightness_mv)
{
    g_edf_battery_mv    = battery_mv;
    g_edf_brightness_mv = brightness_mv;
    LOG_INF("EDF slow channels: battery=%d mV, brightness=%d mV", battery_mv, brightness_mv);
}

/* Record a button event in the current EDF record window.
 * The event code (button_event_t value, 1..13) is written into the Button EDF
 * channel for that 1-second record, then auto-cleared. Multiple events within
 * the same record window: the LATEST one wins (rare during sleep). Code 0
 * means no event. Safe to call from any context (ISR, thread).
 */
void data_logger_record_button_event(int event_code)
{
    if (event_code <= 0 || event_code > 0xFF)
    {
        return;
    }
    g_edf_button_event_code = (uint8_t)event_code;
}

/* Back-compat wrapper: legacy PRESSED-only call. */
void data_logger_record_button_press(void)
{
    /* BUTTON_EVENT_PRESSED == 12 in button_event_t (see hw_abstraction/button.h);
     * encode directly to avoid a dependency on the hardware header here. */
    g_edf_button_event_code = 12;
}

/* ── Wall-clock sync + recording rename (fw 0.0.64) ─────────────────────────
 * Civil-date math: Howard Hinnant's days_from_civil / civil_from_days
 * algorithms (public domain), valid far beyond this device's lifetime. */

static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= (m <= 2);
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153U * (m + (m > 2 ? (unsigned)-3 : 9U)) + 2U) / 5U + d - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

static void civil_from_days(int64_t z, int *py, unsigned *pm, unsigned *pd)
{
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int y = (int)yoe + (int)(era * 400);
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned m = mp + (mp < 10 ? 3 : (unsigned)-9);
    *py = y + (m <= 2);
    *pm = m;
    *pd = d;
}

/* Convert a wall time that lies `seconds_back` in the past of (days, sod). */
static void wall_time_from_days_sod(int64_t days, int64_t sod,
                                    data_logger_wall_time_t *out)
{
    int y;
    unsigned m, d;
    civil_from_days(days, &y, &m, &d);
    out->year   = (uint16_t)y;
    out->month  = (uint8_t)m;
    out->day    = (uint8_t)d;
    out->hour   = (uint8_t)(sod / 3600);
    out->minute = (uint8_t)((sod % 3600) / 60);
    out->second = (uint8_t)(sod % 60);
}

void data_logger_set_wall_time(const data_logger_wall_time_t *t)
{
    if (t == NULL || t->month < 1 || t->month > 12 || t->day < 1 ||
        t->day > 31 || t->hour > 23 || t->minute > 59 || t->second > 60)
    {
        LOG_WRN("wall-time sync rejected (invalid fields)");
        return;
    }
    k_mutex_lock(&g_ads131_mutex, K_FOREVER);
    g_wall_clock.base_days =
        (int32_t)days_from_civil((int)t->year, t->month, t->day);
    g_wall_clock.base_sec_of_day =
        (int32_t)t->hour * 3600 + (int32_t)t->minute * 60 + t->second;
    g_wall_clock.ref_uptime_ms = MAX(k_uptime_get(), 1); /* 0 = never synced */
    k_mutex_unlock(&g_ads131_mutex);
    LOG_INF("wall-time synced: %04u-%02u-%02u %02u:%02u:%02u",
            t->year, t->month, t->day, t->hour, t->minute, t->second);
}

/* Current (or `seconds_back`-shifted) wall time. Returns false pre-sync. */
static bool wall_time_get(uint32_t seconds_back, data_logger_wall_time_t *out)
{
    k_mutex_lock(&g_ads131_mutex, K_FOREVER);
    if (g_wall_clock.ref_uptime_ms == 0)
    {
        k_mutex_unlock(&g_ads131_mutex);
        return false;
    }
    int64_t total = (int64_t)g_wall_clock.base_days * 86400LL +
                    g_wall_clock.base_sec_of_day +
                    (k_uptime_get() - g_wall_clock.ref_uptime_ms) / 1000 -
                    seconds_back;
    k_mutex_unlock(&g_ads131_mutex);
    int64_t days = total / 86400LL;
    int64_t sod  = total % 86400LL;
    if (sod < 0)
    {
        sod += 86400;
        days -= 1;
    }
    wall_time_from_days_sod(days, sod, out);
    return true;
}

bool data_logger_wall_time_now(data_logger_wall_time_t *out)
{
    return wall_time_get(0, out);
}

/* Patch startdate (offset 168, "dd.mm.yy") and starttime (offset 176,
 * "hh.mm.ss") of an existing BDF header in place. Raw fs ops under one
 * explicit SD-manager lock (mirrors edf_close_data_file's locking style). */
static data_logger_result_t patch_bdf_header_start(const char *full_path,
                                                   const data_logger_wall_time_t *st)
{
    char fields[17]; /* 16 chars "dd.mm.yyhh.mm.ss" + NUL */
    snprintf(fields, sizeof(fields), "%02u.%02u.%02u%02u.%02u.%02u",
             st->day, st->month, (unsigned)(st->year % 100),
             st->hour, st->minute, st->second);

    if (sd_card_manager_lock(5000) != 0)
    {
        return DATA_LOGGER_FILE_ERROR;
    }
    struct fs_file_t f;
    fs_file_t_init(&f);
    data_logger_result_t res = DATA_LOGGER_OK;
    if (fs_open(&f, full_path, FS_O_RDWR) != 0)
    {
        sd_card_manager_unlock();
        return DATA_LOGGER_FILE_ERROR;
    }
    if (fs_seek(&f, 168, FS_SEEK_SET) != 0 ||
        fs_write(&f, fields, 16) != 16)
    {
        res = DATA_LOGGER_FILE_ERROR;
    }
    (void)fs_sync(&f);
    (void)fs_close(&f);
    sd_card_manager_unlock();
    return res;
}

data_logger_result_t data_logger_apply_wall_time_to_recording(void)
{
    data_logger_wall_time_t now;
    if (!data_logger_wall_time_now(&now))
    {
        return DATA_LOGGER_ERROR;
    }

    k_mutex_lock(&g_ads131_mutex, K_FOREVER);
    if (!g_ads131_logging_initialized || g_ads131_filename[0] == '\0')
    {
        k_mutex_unlock(&g_ads131_mutex);
        return DATA_LOGGER_OK; /* no active recording — nothing to label */
    }

    /* Rename once: a '_' in the name marks an already-dated file. */
    if (strchr(g_ads131_filename, '_') == NULL)
    {
        /* FatFS forbids renaming open objects — drop the keep-open writer
         * handles first (they lazily reopen under the new name on the next
         * chunk; the .SNX stays number-addressed and untouched). */
        edf_close_data_file();

        char new_name[sizeof(g_ads131_filename)];
        snprintf(new_name, sizeof(new_name),
                 REC_DIR "/AD%04u_%04u%02u%02u_%02u%02u.BDF",
                 (unsigned)g_ads131_log_number, (unsigned)now.year, now.month,
                 now.day, now.hour, now.minute);
        char old_path[MAX_PATH];
        char new_path[MAX_PATH];
        snprintf(old_path, sizeof(old_path), "%s/%s", disk_mount_pt,
                 g_ads131_filename);
        snprintf(new_path, sizeof(new_path), "%s/%s", disk_mount_pt, new_name);

        int rc = -1;
        if (sd_card_manager_lock(5000) == 0)
        {
            rc = fs_rename(old_path, new_path);
            sd_card_manager_unlock();
        }
        if (rc == 0)
        {
            strncpy(g_ads131_filename, new_name, sizeof(g_ads131_filename) - 1);
            g_ads131_filename[sizeof(g_ads131_filename) - 1] = '\0';
            LOG_INF("recording renamed: %s", g_ads131_filename);
        }
        else
        {
            /* Keep recording under the old name — rename is cosmetic and must
             * never endanger the data path. */
            LOG_WRN("recording rename failed (%d); keeping %s", rc,
                    g_ads131_filename);
        }
    }

    /* Patch the BDF header's startdate/starttime with the recording START
     * (= now minus one second per written record). Idempotent — repeated
     * syncs recompute the same instant (±1-2 s estimate jitter). */
    data_logger_wall_time_t start;
    uint32_t records = g_edf_data_records;
    char full_path[MAX_PATH];
    snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt,
             g_ads131_filename);
    k_mutex_unlock(&g_ads131_mutex);

    if (!wall_time_get(records, &start))
    {
        return DATA_LOGGER_ERROR;
    }
    data_logger_result_t res = patch_bdf_header_start(full_path, &start);
    if (res == DATA_LOGGER_OK)
    {
        LOG_INF("BDF header start patched: %02u.%02u.%02u %02u:%02u:%02u",
                start.day, start.month, (unsigned)(start.year % 100),
                start.hour, start.minute, start.second);
    }
    else
    {
        LOG_WRN("BDF header start patch failed");
    }
    return res;
}

/* ── Backfill: read a sample_num range from stored ADxxxx.EDF files ───────── */

/* Small staging buffer for per-file reads; avoids large stack allocations.
 * Only one backfill reader runs at a time (backfill worker thread is singleton).
 */
/* Keep backfill reads large enough for long gaps (minutes to hours) while
 * still releasing the SD lock frequently enough that live writer work can
 * interleave without timing out behind one long read.
 *
 * 56 records ~= 2.6 KiB, which stays within the current board limit while
 * still avoiding the very high reopen/seek overhead of tiny chunks.
 */
#define BACKFILL_READ_CHUNK 56
/* Max bytes per positioning step under one SD-lock hold (see the stepped seek
 * in data_logger_read_ads131_range). 1 MiB ≈ 175 records ≈ 3 min of data. */
#define BACKFILL_SEEK_STEP_BYTES ((off_t)1024 * 1024)
#define BACKFILL_LOCK_TIMEOUT_MS 200
/* Max consecutive failed lock attempts before a reader gives up (~5 s total).
 * 0.0.70: unbounded retry wedged the backfill worker forever when the lock
 * holder died in a degraded-SD op (field 2026-07-16 08:02). */
#define BACKFILL_LOCK_MAX_TRIES  25
static ads131_sample_record_t g_backfill_read_buf[BACKFILL_READ_CHUNK];

static int backfill_position_file(struct fs_file_t *f, off_t target_offset, uint32_t file_num)
{
    if (target_offset <= 0)
    {
        return 0;
    }
    const off_t start_pos = fs_tell(f);

    int seek_err = fs_seek(f, target_offset, FS_SEEK_SET);
    if (seek_err == 0)
    {
        return 0;
    }

    LOG_WRN("backfill: fs_seek failed for file %u at %lld: %d; falling back to sequential skip",
            file_num, (long long)target_offset, seek_err);

    /* The stepped caller may already be part-way: skip from the current fptr
     * (a failed fs_seek leaves it unchanged). */
    if (fs_seek(f, start_pos, FS_SEEK_SET) != 0)
    {
        return -EIO;
    }
    off_t skipped = start_pos;
    while (skipped < target_offset)
    {
        size_t remaining = (size_t)(target_offset - skipped);
        size_t chunk = remaining < sizeof(g_backfill_read_buf)
                           ? remaining
                           : sizeof(g_backfill_read_buf);

        ssize_t bytes = fs_read(f, g_backfill_read_buf, chunk);
        if (bytes <= 0)
        {
            LOG_WRN("backfill: sequential skip failed for file %u at %lld/%lld: %zd",
                    file_num, (long long)skipped, (long long)target_offset, bytes);
            return bytes < 0 ? (int)bytes : -EIO;
        }

        skipped += (off_t)bytes;
    }

    return 0;
}

/* Read one uint32 LE entry [idx] from an open .SNX index handle. */
static bool snx_read_entry(struct fs_file_t *sf, uint32_t idx, uint32_t *out)
{
    if (fs_seek(sf, (off_t)idx * EDF_SNX_ENTRY_BYTES, FS_SEEK_SET) != 0)
    {
        return false;
    }
    uint8_t b[EDF_SNX_ENTRY_BYTES];
    if (fs_read(sf, b, EDF_SNX_ENTRY_BYTES) != (ssize_t)EDF_SNX_ENTRY_BYTES)
    {
        return false;
    }
    *out = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return true;
}

/* Load a window of the per-record sample-number index AD####.SNX for `file_num`.
 * Bisects for the record whose stored first_sn is the largest <= target_sn (the
 * record that can contain target_sn), sets *out_base to its index, and copies up
 * to max_out consecutive real first_sns from there into out[]. Returns the count
 * copied, or 0 if no usable index exists (caller then falls back to the
 * positional sn->record formula). Takes the SD lock internally for the brief
 * O(log N) bisect + window read. */
static size_t snx_load_window(uint32_t file_num, uint32_t target_sn,
                              uint32_t *out_base, uint32_t *out, size_t max_out)
{
    char snx_path[MAX_PATH];
    snprintf(snx_path, sizeof(snx_path), "%s/" REC_DIR "/AD%04u.SNX",
             disk_mount_pt, (unsigned)file_num);

    if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) != 0)
    {
        return 0;
    }
    struct fs_file_t sf;
    fs_file_t_init(&sf);
    if (fs_open(&sf, snx_path, FS_O_READ) != 0)
    {
        sd_card_manager_unlock();
        return 0; /* no index (old recording) -> positional fallback */
    }
    size_t n = 0;
    if (fs_seek(&sf, 0, FS_SEEK_END) == 0)
    {
        off_t sz = fs_tell(&sf);
        if (sz > 0)
        {
            n = (size_t)(sz / EDF_SNX_ENTRY_BYTES);
        }
    }
    if (n == 0)
    {
        (void)fs_close(&sf);
        sd_card_manager_unlock();
        return 0;
    }
    /* upper_bound: first idx with snx[idx] > target_sn. */
    size_t lo = 0, hi = n;
    while (lo < hi)
    {
        size_t mid = lo + (hi - lo) / 2;
        uint32_t v;
        if (!snx_read_entry(&sf, (uint32_t)mid, &v))
        {
            (void)fs_close(&sf);
            sd_card_manager_unlock();
            return 0;
        }
        if (v <= target_sn)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid;
        }
    }
    size_t base = (lo > 0) ? (lo - 1) : 0;
    size_t cnt = 0;
    for (size_t i = base; i < n && cnt < max_out; i++, cnt++)
    {
        if (!snx_read_entry(&sf, (uint32_t)i, &out[cnt]))
        {
            break;
        }
    }
    *out_base = (uint32_t)base;
    (void)fs_close(&sf);
    sd_card_manager_unlock();
    return cnt;
}

data_logger_result_t data_logger_read_ads131_range(
    uint32_t start_sample_num,
    uint32_t end_sample_num,
    data_logger_ads131_rd_cb_t cb,
    void *ctx,
    data_logger_slow_rd_cb_t slow_cb,
    void *slow_ctx)
{
    if (!cb)
    {
        return DATA_LOGGER_ERROR;
    }

    /* Intentionally NOT checking g_sd_info.is_mounted here: the live writer
     * path (`write_ads131_data_to_sd`) doesn't check it either and is the
     * source of truth for whether SD access is working. In v0.0.51 testing,
     * backfill reads were failing with -2 (NO_SD_CARD) for every chunk even
     * though writes succeeded continuously, because is_mounted is a stale
     * flag that nothing keeps in sync. If the mount is truly gone, fs_open
     * below will fail and we return OK with matched=0 (worker emits 0xBE,
     * app gets an empty range and moves on instead of retrying forever). */

    /* Snapshot state under a short-timeout lock */
    if (k_mutex_lock(&g_ads131_mutex, K_MSEC(100)) != 0)
    {
        return DATA_LOGGER_ERROR;
    }
    uint32_t current_file_num      = g_ads131_log_number;
    uint32_t first_sn              = g_edf_first_sample_num;
    bool     first_seen            = g_edf_first_sample_seen;
    /* Snapshot the ACTUAL filename too — since fw 0.0.64 the active file may
     * carry a wall-time name (AD####_YYYYMMDD_HHMM.BDF), so re-deriving the
     * path from the number alone would fail to open after a rename. */
    char current_filename[sizeof(g_ads131_filename)];
    memcpy(current_filename, g_ads131_filename, sizeof(current_filename));
    k_mutex_unlock(&g_ads131_mutex);

    if (current_file_num == 0 || !first_seen)
    {
        return DATA_LOGGER_OK; /* no data written yet */
    }

    /* If the requested range lies entirely before our first sample, nothing to do */
    if (end_sample_num < first_sn)
    {
        return DATA_LOGGER_OK;
    }

    size_t   matched_records        = 0;
    uint32_t first_matched_sn       = 0;
    uint32_t last_matched_sn        = 0;
    bool     past_range             = false;

    char path[MAX_PATH];
    snprintf(path, sizeof(path), "%s/%s", disk_mount_pt, current_filename);

    /* Compute the FIRST record that can contain start_sample_num and jump
     * straight to it. Each EDF record holds ADS131_WRITE_CHUNK_SIZE (250)
     * samples; record index = (start - first_sn)/250. Previously this loop
     * started at record 0 and iterated linearly to find the first hit, which
     * meant a 1-sample backfill for sample 162k did ~650 open/seek/read/close
     * cycles (≈10 s) before even producing one byte of output. Jumping puts
     * us at O(num_records_in_range) instead of O(records_from_first_sn).
     *
     * The file is opened + positioned ONCE here (not per record). Re-seeking to
     * the gap offset on every record is itself O(file-size) (FF_USE_FASTSEEK=0),
     * which — together with the live writer's lock holds — was the large-file
     * starvation. Now only this single positioning seek is O(offset); each
     * subsequent record is a plain sequential fs_read, with the SD lock taken
     * only briefly per read and released between, so the live writer interleaves. */
    /* Gap-tolerant positioning: consult the per-record .SNX index so we locate
     * the record by its REAL stored first sample_num instead of assuming a
     * gapless (sn = first_sn + record*250) file. snx_window holds the real
     * first_sns for records [snx_base .. snx_base+snx_have); snx_have==0 means
     * no usable index (old recording) -> positional fallback below. The window
     * (32 records ≈ 32 s) dwarfs a backfill chunk (≤1000 samples ≈ 4 records),
     * so it always covers the requested range in practice; any record beyond
     * the window also falls back to positional. Stack-local (the reader runs
     * only on the single backfill worker thread) and kept small (128 B) because
     * RAM headroom on this device is ~bytes. */
    uint32_t snx_window[32];
    uint32_t snx_base = 0;
    const size_t snx_have = snx_load_window(
        current_file_num, start_sample_num, &snx_base, snx_window, 32);

    uint32_t start_record_idx = 0;
    /* Decide UP FRONT whether the .SNX actually covers this request. It does
     * only if the window reaches start_sample_num — i.e. the target is within
     * the last loaded record (+one chunk). If the index STOPPED mid-recording
     * (a transient .SNX write error latched g_edf_snx_failed while the BDF kept
     * growing — observed 2026-07-15: 8.2 h indexed, then ~1 M samples of BDF
     * tail with NO .SNX entries), the bisect lands on the last indexed record
     * and the window never reaches the target. In that case DO NOT stay in
     * indexed mode (it would serve one stale record then break → matched 0 →
     * the app marks the whole un-indexed tail a permanent leak, even though the
     * BDF holds it). Fall through to positional addressing of the contiguous
     * BDF tail instead. */
    bool use_index = (snx_have > 0) &&
        (start_sample_num <
         snx_window[snx_have - 1] + (uint32_t)ADS131_WRITE_CHUNK_SIZE);
    if (use_index)
    {
        start_record_idx = snx_base; /* bisect landed on the containing record */
    }
    else if (start_sample_num > first_sn)
    {
        start_record_idx =
            (start_sample_num - first_sn) / ADS131_WRITE_CHUNK_SIZE;
    }
    uint32_t record_idx = start_record_idx;
    off_t  file_offset =
        (off_t)EDF_HEADER_BYTES +
        (off_t)record_idx * (off_t)EDF_RECORD_BYTES;

    /* Open + position ONCE, under the lock (retry/yield on contention, as the
     * per-record path used to). The reader uses its OWN read-only handle,
     * distinct from the writer's RDWR handle (FF_FS_LOCK=0 permits the concurrent
     * open; sd_card_manager_lock serializes the actual SD accesses). */
    struct fs_file_t f;
    fs_file_t_init(&f);
    bool f_open = false;
    /* BOUNDED lock retries (0.0.70): these loops used to retry FOREVER. If
     * the SD lock stays unobtainable (holder wedged in a degraded-card op —
     * candidate mechanism for the 2026-07-16 08:02 field wedge, where the
     * backfill responder AND the BDF writer went silent together), the worker
     * spun here for good: no 0xBE was ever emitted and every later request
     * queued unanswered. Give up after ~5 s instead — the app re-requests
     * later, and "delivered 0 now" beats "responder dead forever". */
    int lock_tries = 0;
    while (!f_open)
    {
        if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) != 0)
        {
            if (++lock_tries >= BACKFILL_LOCK_MAX_TRIES)
            {
                LOG_WRN("backfill EDF: SD lock unavailable after %d tries — "
                        "giving up (delivered 0)", lock_tries);
                return DATA_LOGGER_OK;
            }
            k_yield();
            continue;
        }
        lock_tries = 0;
        /* FatFS freezes FIL.objsize from the on-disk dir entry at open time
         * (NCS ff.c f_open) and hard-clamps every f_read to it. So this
         * open-once reader can only see records the writer fs_sync'd BEFORE
         * this point — records appended/synced afterwards are invisible within
         * this call. That is correct here ONLY because the backfill worker
         * (ads_service.c) calls data_logger_sync_ads131_data() to push the
         * file past end_sample_num before invoking this range read. Do not
         * remove that pre-read sync, and if a true tail-follow is ever needed,
         * reopen when fs_read hits EOF before end_sample_num. */
        int open_err = fs_open(&f, path, FS_O_READ);
        if (open_err != 0)
        {
            sd_card_manager_unlock();
            LOG_WRN("backfill EDF: failed to open file %u: %d", current_file_num, open_err);
            return DATA_LOGGER_OK; /* matched 0; worker emits 0xBE, app moves on */
        }
        f_open = true;
        sd_card_manager_unlock();
    }

    /* Position in steps of BACKFILL_SEEK_STEP_BYTES, releasing the SD lock in
     * between (0.0.92). With FF_USE_FASTSEEK=0 a seek walks the cluster chain;
     * in one go that is O(offset) under the lock — on a ~15 h night file long
     * enough to stall the BDF writer past its ring headroom (lost samples on
     * SD). FatFS continues a FORWARD seek from the current cluster, so the
     * steps add up to one walk, while each lock hold stays a few ms. */
    {
        uint32_t pos_t0 = k_uptime_get_32();
        uint32_t max_hold_ms = 0;
        uint32_t steps = 0;
        off_t pos = 0;
        int pos_err = 0;
        while (pos < file_offset)
        {
            off_t next = pos + BACKFILL_SEEK_STEP_BYTES;
            if (next > file_offset)
            {
                next = file_offset;
            }
            if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) != 0)
            {
                if (++lock_tries >= BACKFILL_LOCK_MAX_TRIES)
                {
                    pos_err = -EBUSY;
                    break;
                }
                k_yield();
                continue;
            }
            lock_tries = 0;
            uint32_t h0 = k_uptime_get_32();
            pos_err = backfill_position_file(&f, next, current_file_num);
            uint32_t hold = k_uptime_get_32() - h0;
            sd_card_manager_unlock();
            if (hold > max_hold_ms)
            {
                max_hold_ms = hold;
            }
            steps++;
            if (pos_err != 0)
            {
                break;
            }
            pos = next;
            k_yield();
        }
        uint32_t pos_ms = k_uptime_get_32() - pos_t0;
        if (pos_ms > 200 || max_hold_ms > 50)
        {
            LOG_INF("backfill EDF: positioned to record %u in %u ms (%u steps, max lock hold %u ms)",
                    record_idx, pos_ms, steps, max_hold_ms);
        }
        if (pos_err != 0)
        {
            LOG_WRN("backfill EDF: positioning failed (%d) — delivered 0", pos_err);
            if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) == 0)
            {
                (void)fs_close(&f);
                sd_card_manager_unlock();
            }
            else
            {
                (void)fs_close(&f);
            }
            return DATA_LOGGER_OK;
        }
    }

    while (!past_range)
    {
        /* First sample_num of this data record. */
        uint32_t rec_first_sn;
        if (use_index)
        {
            /* Indexed file: use the REAL stored first_sn. NEVER positional-
             * fallback mid-read here — on a gappy file the positional estimate
             * mislabels samples (delivers this record's bytes under a wrong sn),
             * silently corrupting the backfill. If we run off the loaded window
             * we STOP; the worker re-requests the next slice from the last
             * delivered sn+1. The window (32 records ≈ 8000 samples) far exceeds
             * the app's backfill chunk (≤1000 samples), so a real request never
             * reaches this bound. record_idx starts at snx_base and only
             * increments, so the subtraction never underflows. */
            if ((size_t)(record_idx - snx_base) >= snx_have)
            {
                break;
            }
            rec_first_sn = snx_window[record_idx - snx_base];
        }
        else
        {
            /* No index (legacy recording): positional addressing as before. */
            rec_first_sn = first_sn + record_idx * ADS131_WRITE_CHUNK_SIZE;
        }

        /* If even the first sample of this record is beyond end_sample_num, stop */
        if (rec_first_sn > end_sample_num)
        {
            break;
        }

        /* Read the next record sequentially (fptr already positioned — no seek).
         * Lock held only for the read, released immediately after. Bounded
         * retries (0.0.70) — see the open loop above. */
        if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) != 0)
        {
            if (++lock_tries >= BACKFILL_LOCK_MAX_TRIES)
            {
                LOG_WRN("backfill EDF: SD lock lost mid-read after %d tries — "
                        "stopping early at record %u", lock_tries, record_idx);
                break;
            }
            k_yield();
            continue;
        }
        lock_tries = 0;
        ssize_t bytes = fs_read(&f, g_edf_backfill_buf, EDF_RECORD_BYTES);
        sd_card_manager_unlock();

        if (bytes <= 0)
        {
            break; /* EOF or error */
        }

        /* How many *fully-readable* samples did we actually get? Each EDF
         * record stores the 8 channels in channel-major layout:
         *   ch0[0..249] | ch1[0..249] | … | ch7[0..249] | slow_ch[0..2]
         * That means sample s is fully reconstructed only when bytes
         * cover ALL eight per-channel slots for index s. The byte offset of
         * the last channel's sample s is:
         *   (ADS131_MAX_CHANNELS - 1) * ADS131_WRITE_CHUNK_SIZE * 2 + s * 2
         *     + 2 (one int16)
         *
         * If the read returned a partial record (e.g. because the writer
         * thread hasn't yet flushed the latest 250-sample chunk), the
         * trailing bytes for ch1..ch7 are STALE from the previous record's
         * read — using `bytes / 16` as the sample count would silently mix
         * the current record's ch0 with the previous record's ch1..ch7,
         * which shows up as a broadband impulse / vertical stripe in the
         * spectrogram at every chunk boundary.
         */
        const size_t channels = (size_t)ADS131_MAX_CHANNELS;
        const size_t per_channel_bytes =
            (size_t)ADS131_WRITE_CHUNK_SIZE * BDF_BPS;
        const size_t min_bytes_for_one_sample =
            (channels - 1) * per_channel_bytes + BDF_BPS;
        size_t samples_in_record;
        if ((size_t)bytes < min_bytes_for_one_sample)
        {
            samples_in_record = 0;
        }
        else
        {
            /* For sample s, the highest byte we touch is
             *   (channels - 1) * per_channel_bytes + (s + 1) * BDF_BPS.
             * Require that <= bytes. */
            samples_in_record =
                ((size_t)bytes - (channels - 1) * per_channel_bytes) /
                BDF_BPS;
        }
        if (samples_in_record == 0)
        {
            break;
        }
        if (samples_in_record > ADS131_WRITE_CHUNK_SIZE)
        {
            samples_in_record = ADS131_WRITE_CHUNK_SIZE;
        }

        /* Deliver the record's slow channels (battery/brightness/button —
         * 1 sample per record, stored after the 8 ADS channel blocks) when
         * the record was read completely AND intersects the requested range.
         * bytes == EDF_RECORD_BYTES guarantees the 9 trailing slow bytes in
         * g_edf_backfill_buf are from THIS record (a short read would leave
         * stale bytes from the previous record there). */
        if (slow_cb != NULL && (size_t)bytes >= EDF_RECORD_BYTES &&
            rec_first_sn <= end_sample_num &&
            rec_first_sn + samples_in_record > start_sample_num)
        {
            const size_t slow_base =
                (size_t)ADS131_MAX_CHANNELS * ADS131_WRITE_CHUNK_SIZE * BDF_BPS;
            slow_cb(rec_first_sn,
                    bdf_get_i24(g_edf_backfill_buf, slow_base + 0 * BDF_BPS),
                    bdf_get_i24(g_edf_backfill_buf, slow_base + 1 * BDF_BPS),
                    bdf_get_i24(g_edf_backfill_buf, slow_base + 2 * BDF_BPS),
                    slow_ctx);
        }

        /* Reconstruct and deliver matching samples */
        for (size_t s = 0; s < samples_in_record; s++)
        {
            uint32_t sn = rec_first_sn + (uint32_t)s;

            if (sn > end_sample_num)
            {
                past_range = true;
                break;
            }

            if (sn >= start_sample_num)
            {
                /* Build a synthetic ads131_sample_record_t for the BLE callback.
                 * Timestamps are reconstructed from nominal sample position.
                 * Status is 0 (not stored in EDF).
                 */
                ads131_sample_record_t rec;
                rec.sample_num   = sn;
                rec.timestamp_us = (uint64_t)(record_idx * ADS131_WRITE_CHUNK_SIZE + s) * 4000u;
                rec.status       = 0;
                for (int ch = 0; ch < ADS131_MAX_CHANNELS; ch++)
                {
                    /* BDF layout: ch0 block first, then ch1, etc. Each sample
                     * is a full 24-bit value (3 bytes LE) — recovered exactly,
                     * no <<8 reconstruction needed. */
                    size_t off = ((size_t)ch * ADS131_WRITE_CHUNK_SIZE + s) * BDF_BPS;
                    rec.channels[ch] = bdf_get_i24(g_edf_backfill_buf, off);
                }

                if (matched_records == 0)
                {
                    first_matched_sn = sn;
                }
                last_matched_sn = sn;
                matched_records++;
                if (cb(&rec, ctx) != 0)
                {
                    /* Request cancelled / its connection is gone: stop reading
                     * instead of churning through the rest of the range. */
                    past_range = true;
                    break;
                }
            }
        }

        if (past_range)
        {
            break;
        }

        record_idx++;

        /* Yield between records so live writer can interleave */
        k_yield();
    }

    /* Close the read handle (under the lock if obtainable; a read-only handle is
     * harmless to close even if the lock can't be taken). */
    if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) == 0)
    {
        (void)fs_close(&f);
        sd_card_manager_unlock();
    }
    else
    {
        (void)fs_close(&f);
    }

    if (matched_records == 0)
    {
        LOG_WRN("backfill BDF range [%u .. %u]: matched 0 samples in AD%04u.BDF (first_sn=%u)",
                start_sample_num, end_sample_num, (unsigned)current_file_num, first_sn);
    }
    else
    {
        LOG_INF("backfill BDF range [%u .. %u]: matched %u samples in AD%04u.BDF [%u .. %u]",
                start_sample_num, end_sample_num,
                (unsigned)matched_records, (unsigned)current_file_num,
                first_matched_sn, last_matched_sn);
    }

    return DATA_LOGGER_OK;
}

int data_logger_list_dir_meta(const char *path,
                              data_logger_dir_meta_cb_t cb, void *ctx)
{
    if (!path || !cb)
    {
        return DATA_LOGGER_ERROR;
    }

    /* `path` is RELATIVE to the SD mount (same convention as
     * data_logger_read_file_stream): "" = root, "DATA" = the recordings dir (fw <= 0.0.95: "ADC").
     * Prepend the mount point so the app never needs to know it. */
    char full_path[MAX_PATH];
    if (path[0] != '\0')
    {
        snprintf(full_path, sizeof(full_path), "%s/%s", disk_mount_pt, path);
    }
    else
    {
        snprintf(full_path, sizeof(full_path), "%s", disk_mount_pt);
    }

    struct fs_dir_t dirp;
    struct fs_dirent ent; /* on the worker stack — no static RAM */
    fs_dir_t_init(&dirp);

    /* Directory scans are short; hold the lock for the whole readdir loop. */
    if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) != 0)
    {
        return DATA_LOGGER_ERROR;
    }

    int res = fs_opendir(&dirp, full_path);
    if (res)
    {
        sd_card_manager_unlock();
        LOG_WRN("list_dir_meta: opendir %s failed: %d", full_path, res);
        return res;
    }

    int count = 0;
    for (;;)
    {
        res = fs_readdir(&dirp, &ent);
        if (res || ent.name[0] == 0)
        {
            break;
        }
        cb(ent.name, ent.type == FS_DIR_ENTRY_DIR, (uint32_t)ent.size, ctx);
        count++;
    }

    fs_closedir(&dirp);
    sd_card_manager_unlock();

    return (res == 0) ? count : res;
}

data_logger_result_t data_logger_read_file_stream(
    const char *filename, uint32_t start_offset, uint32_t max_bytes,
    data_logger_file_chunk_cb_t cb, void *ctx)
{
    if (!filename || !cb)
    {
        return DATA_LOGGER_ERROR;
    }

    char path[MAX_PATH];
    snprintf(path, sizeof(path), "%s/%s", disk_mount_pt, filename);

    /* Open + position ONCE (single O(offset) seek), under a brief lock with
     * retry/yield on contention. Own read-only handle (FF_FS_LOCK=0 permits a
     * concurrent open vs the writer; sd_card_manager_lock serializes the SD
     * accesses). Mirrors data_logger_read_ads131_range's discipline. */
    struct fs_file_t f;
    fs_file_t_init(&f);
    bool f_open = false;
    /* Bounded lock retries (0.0.70) — same rationale as the backfill reader:
     * an unobtainable lock must not wedge the shared worker forever. */
    int lock_tries = 0;
    while (!f_open)
    {
        if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) != 0)
        {
            if (++lock_tries >= BACKFILL_LOCK_MAX_TRIES)
            {
                LOG_WRN("file_stream: SD lock unavailable after %d tries — "
                        "aborting", lock_tries);
                return DATA_LOGGER_FILE_ERROR;
            }
            k_yield();
            continue;
        }
        lock_tries = 0;
        int oerr = fs_open(&f, path, FS_O_READ);
        if (oerr != 0)
        {
            sd_card_manager_unlock();
            LOG_WRN("file_stream: open %s failed: %d", path, oerr);
            return DATA_LOGGER_FILE_ERROR;
        }
        if (start_offset > 0)
        {
            int serr = fs_seek(&f, (off_t)start_offset, FS_SEEK_SET);
            if (serr != 0)
            {
                (void)fs_close(&f);
                sd_card_manager_unlock();
                LOG_WRN("file_stream: seek %u failed: %d", start_offset, serr);
                return DATA_LOGGER_READ_ERROR;
            }
        }
        f_open = true;
        sd_card_manager_unlock();
    }

    /* Reuse the backfill read scratch (only the single backfill worker drives
     * either path, so they never overlap). */
    uint32_t remaining = (max_bytes == 0) ? UINT32_MAX : max_bytes;
    data_logger_result_t result = DATA_LOGGER_OK;
    int aborted = 0;
    while (remaining > 0 && !aborted)
    {
        size_t want = (remaining < sizeof(g_edf_backfill_buf))
                          ? (size_t)remaining
                          : sizeof(g_edf_backfill_buf);

        if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) != 0)
        {
            if (++lock_tries >= BACKFILL_LOCK_MAX_TRIES)
            {
                LOG_WRN("file_stream: SD lock lost mid-read after %d tries — "
                        "stopping early", lock_tries);
                result = DATA_LOGGER_READ_ERROR;
                break;
            }
            k_yield();
            continue;
        }
        lock_tries = 0;
        ssize_t got = fs_read(&f, g_edf_backfill_buf, want);
        sd_card_manager_unlock();

        if (got < 0)
        {
            result = DATA_LOGGER_READ_ERROR;
            break;
        }
        if (got == 0)
        {
            break; /* EOF */
        }

        aborted = cb(g_edf_backfill_buf, (size_t)got, ctx);
        remaining -= (uint32_t)got;
        if ((size_t)got < want)
        {
            break; /* short read → EOF reached */
        }
        k_yield(); /* let the live writer interleave between chunks */
    }

    if (sd_card_manager_lock(BACKFILL_LOCK_TIMEOUT_MS) == 0)
    {
        (void)fs_close(&f);
        sd_card_manager_unlock();
    }
    else
    {
        (void)fs_close(&f);
    }

    return result;
}