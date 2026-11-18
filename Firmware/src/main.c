/*
 * Traumschreiber2509 - ADS131M08 ADC Integration Demo
 *
 * Professional firmware demonstrating power-efficient biometric data acquisition:
 * - ADS131M08 8-channel simultaneous sampling at 250Hz
 * - Very low power mode (2.48MHz clock)
 * - Internal oscillator disabled for power savings
 * - Interrupt-driven data acquisition
 * - RTT logging every 100ms (every 25th sample)
 * - BLE LED Button Service for remote control
 *
 * @author ISDT
 * @date 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <strings.h> /* For strcasecmp() */
#include "drivers/ads131m08_ti.h"
#include "drivers/sd_card_manager.h"
#include "drivers/nordic_adc.h"
#include "hw_abstraction/rgb_led.h"
#include "hw_abstraction/button.h"
#include "subsystems/stimulator.h"
#include "subsystems/data_logger.h"
#include "subsystems/audio_player.h"
#include "subsystems/battery_monitor.h"
#include "subsystems/led_animation.h"
#include "subsystems/wav_writer.h"
#include "i2s_out.h"
#include "ble/ble_manager.h"
#include "selftest.h"
#include "ble/ads_service.h"
#include "firmware_version.h"
#include "debug_ads131_osr.h"
#include "subsystems/system_config.h"
#include "subsystems/noise_test.h"
#include "subsystems/charge_guard.h"
#include <zephyr/sys/poweroff.h>
#include <zephyr/fs/fs.h>
#include <zephyr/dfu/mcuboot.h>

#define BUTTON_ACTION_GREEN_LED_DEMO_MS 600

/* Helper to format RTT-style timestamp from uptime (ms
 * Produces string: HH:MM:SS.mmm,uuu (no brackets)
 * NOTE: Uses uint32_t for printing to avoid toolchain issues with 64-bit format specifiers.
 */
static void format_rtt_timestamp(int64_t uptime_ms, char *out, size_t out_size)
{
        uint32_t hours = (uint32_t)(uptime_ms / (1000LL * 60LL * 60LL));
        uint32_t mins = (uint32_t)((uptime_ms / (1000LL * 60LL)) % 60LL);
        uint32_t secs = (uint32_t)((uptime_ms / 1000LL) % 60LL);
        uint32_t msecs = (uint32_t)(uptime_ms % 1000LL);
        /* Zephyr uptime has millisecond resolution; use msecs for microseconds field as best-effort */
        uint32_t usec = msecs;

        snprintf(out, out_size, "%02u:%02u:%02u.%03u,%03u",
                 hours, mins, secs, msecs, usec);
}

/* Small helper to trim whitespace in-place and return pointer to trimmed string */
static char *trim_ws(char *s)
{
        if (!s)
                return s;
        while (*s && isspace((unsigned char)*s))
                s++;
        if (*s == '\0')
                return s;
        char *end = s + strlen(s) - 1;
        while (end > s && isspace((unsigned char)*end))
                end--;
        end[1] = '\0';
        return s;
}

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static void fota_confirm_running_image_if_needed(void)
{
        int confirmed = boot_is_img_confirmed();
        if (confirmed == 1)
        {
                LOG_INF("FOTA: running image already confirmed");
                return;
        }

        if (confirmed < 0)
        {
                LOG_WRN("FOTA: boot_is_img_confirmed() failed: %d", confirmed);
                return;
        }

        int ret = boot_write_img_confirmed();
        if (ret == 0)
        {
                LOG_INF("FOTA: running image confirmed successfully");
        }
        else
        {
                LOG_ERR("FOTA: failed to confirm running image: %d", ret);
        }
}

/* ADS131M08 TI data callback function */
static void ads131m08_ti_data_callback(const ads131m08_sample_t *sample)
{
        static uint32_t callback_error_count = 0;

        /* Noise measurement with shorted inputs (no-op unless running). */
        noise_test_on_sample(sample->channels, sample->sample_number);

        /* Buffer sample for SD card logging only when a recording is active.
         * data_logger_buffer_ads131_sample() returns DATA_LOGGER_ERROR for
         * every sample when !g_ads131_logging_initialized, which happens any
         * time the device is streaming live without an active recording — that
         * filled RTT with 250 "Failed to buffer" lines/sec. Gating here avoids
         * both the spam and the wasted mutex lock. */
        data_logger_result_t log_ret = DATA_LOGGER_OK;
        if (data_logger_is_ads131_logging_initialized()) {
            uint32_t dl_t0 = k_cycle_get_32();
            log_ret = data_logger_buffer_ads131_sample(
                sample->sample_number,
                sample->timestamp_us,
                sample->channels,
                sample->status);
            uint32_t dl_dt_us = k_cyc_to_us_floor32(k_cycle_get_32() - dl_t0);
            if (dl_dt_us > 2000) {
                static uint32_t dl_worst_us = 0;
                if (dl_dt_us > dl_worst_us) {
                    dl_worst_us = dl_dt_us;
                    LOG_WRN("data_logger buffer call slow: %u us (sample #%u)",
                            dl_dt_us, sample->sample_number);
                }
            }
        }

        if (log_ret != DATA_LOGGER_OK)
        {
                callback_error_count++;
                /* Only log first few errors and then occasionally to avoid spam */
                if (callback_error_count <= 5 || (callback_error_count % 1000 == 0))
                {
                        LOG_ERR("Failed to buffer ADS131 sample #%u: %d (error count: %u)",
                                sample->sample_number, log_ret, callback_error_count);
                }
        }

        /* Send sample to BLE streaming service (non-blocking, ISR-safe) */
        if (ble_manager_is_connected())
        {
                int ble_err;
                ads_sample_data_t ble_sample = {
                    .sample_num = sample->sample_number,
                    .timestamp_us = sample->timestamp_us,
                    .status = sample->status};

                for (int i = 0; i < 8; i++)
                {
                        ble_sample.channels[i] = sample->channels[i];
                }

                ble_err = ads_service_send_sample(&ble_sample);

                if (ble_err && (callback_error_count % 100 == 0))
                {
                        /* Only log occasionally to avoid spam */
                        LOG_WRN("BLE sample queue full, dropped sample #%u",
                                sample->sample_number);
                }
        }

        if (sample->sample_number % 10000 == 0)
        {
                LOG_INF("Sample #%u - CH0: %d, CH1: %d, CH7: %d, Status: 0x%08X",
                        sample->sample_number,
                        sample->channels[0],
                        sample->channels[1],
                        sample->channels[7],
                        sample->status);

                /* Log BLE streaming statistics every 10000 samples */
                if (ble_manager_is_connected())
                {
                        ads_status_t ble_status;
                        if (ads_service_get_status(&ble_status) == 0)
                        {
                                LOG_INF("BLE Streaming: sent=%u dropped=%u errors=%u",
                                        ble_status.samples_sent,
                                        ble_status.samples_dropped,
                                        ble_status.notification_errors);
                        }
                }

                /* NOTE: Removed CSV logging from this callback as it causes 17-50ms gaps
                 * in the data stream due to SD card write blocking. The CSV log was causing
                 * periodic timestamp gaps every 1000 samples.
                 * RTT/console logging is fast enough to not cause issues.
                 */
        }
} /* Nordic ADC data callback function - called every 10 seconds */

/* Message type for passing Nordic ADC samples to logger thread */
typedef struct
{
        uint32_t timestamp_ms;
        int32_t voltage_mv[8];
        uint16_t channels_raw[8]; /* Raw ADC values for logging */
        uint32_t status;
} nordic_adc_msg_t;

// Event logging for button, LED, speaker
typedef struct
{
        uint32_t event_type; // Sentinel sample_num value
        uint64_t timestamp_us;
        int32_t data[8]; // Event-specific data
        uint32_t status; // Event-specific status
} event_log_msg_t;

/* Event messages; allow some burst headroom */
K_MSGQ_DEFINE(event_log_msgq, sizeof(event_log_msg_t), 16, 8);

/* Button action executor queue (holds up to 4 events) */
K_MSGQ_DEFINE(button_action_msgq, sizeof(button_event_t), 4, 4);

/* Non-blocking enqueue helper (ISR-safe): if full, drop one and retry once */
static inline void event_log_enqueue(const event_log_msg_t *msg)
{
        if (k_msgq_put(&event_log_msgq, msg, K_NO_WAIT) != 0)
        {
                event_log_msg_t drop;
                (void)k_msgq_get(&event_log_msgq, &drop, K_NO_WAIT); /* drop oldest */
                (void)k_msgq_put(&event_log_msgq, msg, K_NO_WAIT);
        }
}

/* Event logger stack */
K_THREAD_STACK_DEFINE(event_logger_stack, 1024);
static struct k_thread event_logger_thread;
static void event_logger_thread_entry(void *p1, void *p2, void *p3)
{
        ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        event_log_msg_t msg;
        while (1)
        {
                int rc = k_msgq_get(&event_log_msgq, &msg, K_FOREVER);
                if (rc == 0)
                {
                        /* Best-effort: ignore errors if ADS logger not yet initialized */
                        (void)data_logger_buffer_ads131_sample(
                            msg.event_type, msg.timestamp_us, msg.data, msg.status);
                }
                else
                {
                        k_sleep(K_MSEC(5));
                }
        }
}


/* Button action executor thread */
#if 1
/* Forward declaration: ensure executor can call apply_button_action() without
 * causing an implicit non-static declaration prior to the static definition. */
static void apply_button_action(button_event_t event);
#endif
/* 3072 B: the button-action executor may perform SD/FATFS *writes* (e.g. the
 * BLE_CONFIG_TOGGLE persists config.txt). The FATFS write+sync path needs the
 * same headroom as the EDF writer thread (also 3072); 1024 overflowed and the
 * MPU stack guard faulted the thread mid-write, so the config never changed. */
#define ACTION_EXEC_STACK_SIZE 3072
K_THREAD_STACK_DEFINE(action_exec_stack, ACTION_EXEC_STACK_SIZE);
static struct k_thread action_exec_thread;
static void action_exec_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    button_event_t ev;
    while (1) {
        int rc = k_msgq_get(&button_action_msgq, &ev, K_FOREVER);
        if (rc == 0) {
            apply_button_action(ev);
        }
    }
}

static inline void button_action_enqueue(button_event_t ev)
{
        if (k_msgq_put(&button_action_msgq, &ev, K_NO_WAIT) != 0) {
                button_event_t drop;
                (void)k_msgq_get(&button_action_msgq, &drop, K_NO_WAIT);
                (void)k_msgq_put(&button_action_msgq, &ev, K_NO_WAIT);
        }
        LOG_DBG("Enqueued button action: %s", button_event_to_string(ev));
}

static bool button_action_uses_direct_led_feedback(button_event_t event)
{
        if (event <= BUTTON_EVENT_NONE || event >= BUTTON_EVENT_COUNT)
        {
                return false;
        }

        return ((button_action_t)g_config.button_action_map[event] == BUTTON_ACTION_SHOW_BATTERY_LED_STATUS);
}

static void led_oneshot_complete_cb(button_event_t event, void *ctx)
{
        (void)ctx;
        /* Only enqueue an action if runtime config maps this event to a real action. */
        button_action_t configured = (button_action_t)g_config.button_action_map[event];
        if (configured == BUTTON_ACTION_NONE)
        {
                LOG_INF("LED oneshot complete for %s - configured action NONE, skipping enqueue", button_event_to_string(event));
                return;
        }

        LOG_INF("LED oneshot complete for %s - enqueueing action %s",
                button_event_to_string(event), button_action_to_string(configured));
        button_action_enqueue(event);
}

/* Microphone recording buffer and state */
#define MIC_SAMPLE_RATE_HZ 8000
/* NOTE: Limited to 2500ms (2.5s) due to RAM constraints (256KB total, ~200KB used).
 * 10s @ 8kHz would require 160KB buffer, which exceeds available RAM.
 */
#define MIC_RECORDING_DURATION_MS 2500
#define MIC_BUFFER_SIZE_SAMPLES ((MIC_SAMPLE_RATE_HZ * MIC_RECORDING_DURATION_MS) / 1000)
/* non-static: der HW-Selbsttest (selftest.c) nutzt diesen 40-KB-Puffer als
 * WAV-Schreib-Chunk wieder (spart RAM im fast vollen Speicher). */
int16_t g_mic_recording_buffer[MIC_BUFFER_SIZE_SAMPLES];
static size_t g_mic_samples_recorded = 0;
static volatile bool g_mic_recording_in_progress = false;

#define LONG_PRESS_RECORD_SAMPLE_RATE_HZ 8000
#define LONG_PRESS_RECORD_DURATION_MS 10000
#define STARTUP_CHUNK_SAMPLES 2048
#define RECORDINGS_DIR_NAME "SNDREC"
static int16_t g_startup_chunk_buf[STARTUP_CHUNK_SAMPLES];

static int startup_record_microphone_to_wav(const char *filename, uint32_t sample_rate_hz, uint32_t duration_ms);

static int get_next_sndrec_filename(char *out_rel_path, size_t out_size)
{
        if (!out_rel_path || out_size < 16)
        {
                return -EINVAL;
        }

        int ret = -1;
        uint32_t max_num = 0;
        char dir_path[64];
        snprintf(dir_path, sizeof(dir_path), "/SD:/%s", RECORDINGS_DIR_NAME);

        if (sd_card_manager_lock(2000) != 0)
        {
                return -EBUSY;
        }

        struct fs_dirent st;
        int sret = fs_stat(dir_path, &st);
        if (sret != 0)
        {
                int mret = fs_mkdir(dir_path);
                if (mret != 0)
                {
                        sd_card_manager_unlock();
                        return mret;
                }
        }

        struct fs_dir_t dir;
        struct fs_dirent entry;
        fs_dir_t_init(&dir);

        if (fs_opendir(&dir, dir_path) == 0)
        {
                while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0')
                {
                        if (entry.type != FS_DIR_ENTRY_FILE)
                        {
                                continue;
                        }

                        size_t len = strlen(entry.name);
                        if (len != 8)
                        {
                                continue;
                        }

                        if (!isdigit((unsigned char)entry.name[0]) ||
                            !isdigit((unsigned char)entry.name[1]) ||
                            !isdigit((unsigned char)entry.name[2]) ||
                            !isdigit((unsigned char)entry.name[3]))
                        {
                                continue;
                        }

                        if (entry.name[4] != '.' ||
                            toupper((unsigned char)entry.name[5]) != 'W' ||
                            toupper((unsigned char)entry.name[6]) != 'A' ||
                            toupper((unsigned char)entry.name[7]) != 'V')
                        {
                                continue;
                        }

                        uint32_t n = (uint32_t)((entry.name[0] - '0') * 1000 +
                                                (entry.name[1] - '0') * 100 +
                                                (entry.name[2] - '0') * 10 +
                                                (entry.name[3] - '0'));
                        if (n > max_num)
                        {
                                max_num = n;
                        }
                }
                fs_closedir(&dir);
        }

        sd_card_manager_unlock();

        uint32_t next = max_num + 1;
        if (next > 9999)
        {
                next = 1;
        }

        ret = snprintf(out_rel_path, out_size, "%s/%04u.WAV", RECORDINGS_DIR_NAME, next);
        if (ret < 0 || (size_t)ret >= out_size)
        {
                return -ENOSPC;
        }

        return 0;
}

/* Test override parameters for quick recording tests (0 = use defaults) */
static int g_mic_test_sample_rate_hz = 0;
static int g_mic_test_duration_ms = 0;

/* Work item for delayed playback */
static struct k_work_delayable mic_playback_work;

/* Thread for microphone recording to avoid blocking system workqueue */
#define MIC_RECORD_STACK_SIZE 2048
#define MIC_RECORD_PRIORITY 5
static K_THREAD_STACK_DEFINE(mic_record_stack, MIC_RECORD_STACK_SIZE);
static struct k_thread mic_record_thread;

static K_THREAD_STACK_DEFINE(hold_record_stack, 3072);
static struct k_thread hold_record_thread;

/* Demo mode thread */
#define DEMO_MODE_STACK_SIZE 4096
#define DEMO_MODE_PRIORITY 5
static K_THREAD_STACK_DEFINE(demo_mode_stack, DEMO_MODE_STACK_SIZE);
static struct k_thread demo_mode_thread;
static volatile bool g_demo_mode_in_progress = false;
static bool g_ads_sampling_active = true;

static bool start_demo_mode_async(void);
static bool start_mic_record_test_async(void);
static bool start_long_record_async(void);
static void set_ads_sampling_enabled(bool enable);
static void start_advertising_worst_case_mode(void);
static void start_streaming_worst_case_mode(void);
static void run_green_led_demo(void);
static void apply_button_action(button_event_t event);
static void mic_recording_thread_entry(void *p1, void *p2, void *p3);

static void demo_mode_blink(const rgb_color_t *color, int count, uint32_t on_ms, uint32_t off_ms)
{
        for (int i = 0; i < count; i++) {
                rgb_led_set_color(color);
                k_sleep(K_MSEC(on_ms));
                rgb_led_off();
                if (i < count - 1) {
                        k_sleep(K_MSEC(off_ms));
                }
        }
}

/* 5x red LED together with a harsh low beep: 400-Hz square wave — the
 * fundamental sits below the ~850-Hz speaker resonance, the odd harmonics
 * (1.2/2/2.8/3.6 kHz) land in the 2-5 kHz band the FS-1540 radiates. The
 * peak is clamped to I2S_OUTPUT_PEAK_LIMIT like every playback. At the boot
 * check I2S is not up yet, so it is initialised here (idempotent). */
static void charge_lockout_alarm(void)
{
        const tone_config_t beep = {
                .type = TONE_SQUARE,
                .frequency_hz = 400,
                .amplitude_percent = 100,
                .duration_ms = 250,
        };
        const bool sound = (i2s_out_init() == 0);

        for (int i = 0; i < 5; i++)
        {
                rgb_led_set_color(&RGB_COLOR_RED);
                if (sound)
                {
                        (void)i2s_out_play_tone(&beep);
                }
                k_sleep(K_MSEC(250));
                if (sound)
                {
                        (void)i2s_out_stop();
                }
                rgb_led_off();
                k_sleep(K_MSEC(250));
        }
}

/* USB plugged in -> no operation while charging (fw 0.0.95, Aktionsplan E7).
 * Closes a running recording (final BDF header), blinks red 5x with a low
 * beep and enters System OFF. Waking up needs the slide switch (off/on); a boot with USB still
 * plugged in ends here again. Runs on the main thread (3 KB stack, enough for
 * the FATFS close). Never returns. */
static void charge_lockout_power_off(bool recording, int vbus_mv)
{
        LOG_WRN("USB plugged in (AIN5 %d mV): no operation while charging -> power off", vbus_mv);
        if (recording)
        {
                ads131m08_ti_stop_sampling();
                g_ads_sampling_active = false;
                (void)data_logger_flush_ads131_data();
                LOG_INF("Recording closed");
        }
        led_animation_shutdown();
        charge_lockout_alarm();
        LOG_INF("System OFF");
        k_sleep(K_MSEC(200)); /* let the log reach RTT */
        sys_poweroff();
}

static void demo_mode_thread_entry(void *p1, void *p2, void *p3)
{
        ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);

        LOG_INF("=== DEMO MODE STARTED ===");

        /* Step 1: Red LED full brightness for 5 seconds */
        LOG_INF("Demo: Red LED ON (5s)");
        rgb_led_set_color(&RGB_COLOR_RED);
        k_sleep(K_SECONDS(5));

        /* Step 2: Dark for 5 seconds */
        LOG_INF("Demo: LEDs OFF (5s)");
        rgb_led_off();
        k_sleep(K_SECONDS(5));

        /* Step 3: Blue LED full brightness for 5 seconds */
        LOG_INF("Demo: Blue LED ON (5s)");
        rgb_led_set_color(&RGB_COLOR_BLUE);
        k_sleep(K_SECONDS(5));

        /* Step 4: Both LEDs (red + blue = magenta) for 5 seconds */
        LOG_INF("Demo: Red+Blue LED ON (5s)");
        rgb_led_set_color(&RGB_COLOR_MAGENTA);
        k_sleep(K_SECONDS(5));

        rgb_led_off();

        /* Step 5: Record microphone 10 seconds, save as WAV */
        LOG_INF("Demo: Starting 10s microphone recording...");
        rgb_led_set_color(&RGB_COLOR_RED);

        char wav_rel_path[32];
        int name_ret = get_next_sndrec_filename(wav_rel_path, sizeof(wav_rel_path));
        if (name_ret != 0) {
                LOG_ERR("Demo: Failed to allocate recording filename: %d", name_ret);
                rgb_led_off();
                g_demo_mode_in_progress = false;
                return;
        }

        audio_player_stop();
        int rec_ret = startup_record_microphone_to_wav(wav_rel_path,
                                                       LONG_PRESS_RECORD_SAMPLE_RATE_HZ,
                                                       LONG_PRESS_RECORD_DURATION_MS);
        rgb_led_off();

        if (rec_ret != 0) {
                LOG_ERR("Demo: Recording failed: %d", rec_ret);
                demo_mode_blink(&RGB_COLOR_RED, 5, 100, 100);
                g_demo_mode_in_progress = false;
                return;
        }

        LOG_INF("Demo: Recording saved as %s", wav_rel_path);

        /* Step 6: Red LED blink 3x quickly (recording done) */
        LOG_INF("Demo: Red LED 3x blink (recording done)");
        k_sleep(K_MSEC(300));
        demo_mode_blink(&RGB_COLOR_RED, 3, 200, 200);

        /* Step 7: Play back WAV over speaker at full volume */
        LOG_INF("Demo: Playing back %s", wav_rel_path);
        k_sleep(K_MSEC(500));
        audio_player_result_t play_ret = audio_player_play_wav_from_sd(wav_rel_path);
        if (play_ret != AUDIO_PLAYER_OK) {
                LOG_ERR("Demo: Playback failed: %d", play_ret);
        } else {
                LOG_INF("Demo: Playback complete");
        }

        /* Step 8: Blue LED blink 3x quickly (demo mode done) */
        LOG_INF("Demo: Blue LED 3x blink (demo done)");
        k_sleep(K_MSEC(300));
        demo_mode_blink(&RGB_COLOR_BLUE, 3, 200, 200);

        LOG_INF("=== DEMO MODE FINISHED ===");
        g_demo_mode_in_progress = false;
}

static void hold_record_thread_entry(void *p1, void *p2, void *p3)
{
        ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);

        char wav_rel_path[32];
        int name_ret = get_next_sndrec_filename(wav_rel_path, sizeof(wav_rel_path));
        if (name_ret != 0)
        {
                LOG_ERR("Failed to allocate recording filename: %d", name_ret);
                rgb_led_off();
                g_mic_recording_in_progress = false;
                return;
        }

        LOG_INF("Starting long-press 10s microphone recording to %s", wav_rel_path);

        audio_player_stop();
        int ret = startup_record_microphone_to_wav(wav_rel_path,
                                                   LONG_PRESS_RECORD_SAMPLE_RATE_HZ,
                                                   LONG_PRESS_RECORD_DURATION_MS);

        if (ret == 0)
        {
                LOG_INF("Long-press recording saved as %s", wav_rel_path);
        }
        else
        {
                LOG_ERR("Long-press recording failed");
        }

        rgb_led_off();
        g_mic_recording_in_progress = false;
}

static void set_ads_sampling_enabled(bool enable)
{
        if (enable == g_ads_sampling_active)
        {
                return;
        }

        if (enable)
        {
                /* Stop SAADC periodic reads: the nRF52840 SAADC draws a current pulse
                 * that couples into the 128x-gain ADS131M08 front-end and creates spikes.
                 * Live battery/brightness are read once per EDF record inside the SD write window. */
                nordic_adc_stop_periodic_sampling();
                (void)data_logger_pause_ads131_logging(false);
                (void)ads131m08_ti_start_sampling();
                g_ads_sampling_active = true;
                LOG_INF("ADS sampling enabled");
        }
        else
        {
                ads131m08_ti_stop_sampling();
                (void)data_logger_pause_ads131_logging(true);
                g_ads_sampling_active = false;
                /* Resume SAADC now that the sensitive ADS front-end is idle */
                nordic_adc_start_periodic_sampling();
                LOG_INF("ADS sampling disabled");
        }
}

static void mic_playback_work_handler(struct k_work *work)
{
        if (g_mic_samples_recorded == 0)
        {
                LOG_WRN("No microphone samples to play back");
                return;
        }

        LOG_INF("Playing back recorded microphone audio (%zu samples)", g_mic_samples_recorded);

        /* Log speaker play event */
        event_log_msg_t spk_msg = {
            .event_type = 0xFFFFFFFCu,
            .timestamp_us = (uint64_t)k_uptime_get() * 1000ULL,
            .data = {3, (int32_t)g_mic_samples_recorded, MIC_SAMPLE_RATE_HZ, 0, 0, 0, 0, 0}, /* 3 = play from RAM */
            .status = 0};
        event_log_enqueue(&spk_msg);

        /* Play back the recorded audio */
        audio_player_result_t audio_ret = audio_player_play_from_buffer_async(
            g_mic_recording_buffer,
            g_mic_samples_recorded,
            MIC_SAMPLE_RATE_HZ);

        if (audio_ret != AUDIO_PLAYER_OK)
        {
                LOG_ERR("Failed to start microphone playback: %d", audio_ret);
                rgb_led_set_color(&RGB_COLOR_RED);
                k_sleep(K_MSEC(200));
                rgb_led_off();
        }
        else
        {
                LOG_INF("Microphone playback started");
        }
}

static void run_green_led_demo(void)
{
        rgb_led_set_color(&RGB_COLOR_GREEN);
        k_sleep(K_MSEC(BUTTON_ACTION_GREEN_LED_DEMO_MS));
        rgb_led_off();
}

static rgb_color_t battery_percent_to_led_color(uint8_t percent)
{
        rgb_color_t color;

        if (percent > 100)
        {
                percent = 100;
        }

        /* Two-segment red->yellow->green map so that 50% renders as a clear
         * yellow (R=G=100) instead of a dim 50/50 mix. */
        if (percent <= 50)
        {
                color.red = 100;
                color.green = (uint8_t)(percent * 2);
        }
        else
        {
                color.red = (uint8_t)((100 - percent) * 2);
                color.green = 100;
        }
        color.blue = 0;

        return color;
}

static void show_battery_led_status(void)
{
        const uint8_t percent = battery_monitor_get_level_percent();
        const rgb_color_t level_color = battery_percent_to_led_color(percent);

        /* Number of 0.3 s flashes scales with battery: round-to-nearest-10%,
         * clamped to [1, 10]. So 0-10% -> 1 red flash, 45% -> 5 yellow,
         * 65% -> 7 green-yellow, 95-100% -> 10 green. */
        int num_flashes = ((int)percent + 5) / 10;
        if (num_flashes < 1)
        {
                num_flashes = 1;
        }
        else if (num_flashes > 10)
        {
                num_flashes = 10;
        }

        LOG_INF("Battery LED status: %u%% -> R=%u%% G=%u%% B=%u%%, %d flashes",
                percent, level_color.red, level_color.green, level_color.blue, num_flashes);

        /* Die Flashes über das LED-Animations-Subsystem spielen (nicht direkt via
         * rgb_led_*): dessen Worker-Thread blockiert währenddessen in run_oneshot(),
         * wodurch das blaue Advertising-Blinken (advertising_step) automatisch für
         * die Dauer der Batterie-Flashes pausiert und danach wieder aufgenommen wird.
         * So überlagern sich die 1–10 Farb-Flashes nicht mehr mit dem BLE-Blau. */
        (void)led_animation_play_oneshot(&level_color, (uint8_t)num_flashes,
                                         300, 150, BUTTON_EVENT_SINGLE_CLICK,
                                         NULL, NULL);
}

static bool start_mic_record_test_async(void)
{
        if (g_mic_recording_in_progress)
        {
                LOG_WRN("Microphone recording already in progress");
                return false;
        }

        audio_player_stop();
        k_sleep(K_MSEC(100));
        g_mic_test_sample_rate_hz = 0;
        g_mic_test_duration_ms = 0;
        g_mic_recording_in_progress = true;
        k_thread_create(&mic_record_thread, mic_record_stack,
                        K_THREAD_STACK_SIZEOF(mic_record_stack),
                        mic_recording_thread_entry, NULL, NULL, NULL,
                        MIC_RECORD_PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&mic_record_thread, "mic_record");
        return true;
}

static bool start_demo_mode_async(void)
{
        if (g_demo_mode_in_progress || g_mic_recording_in_progress)
        {
                LOG_WRN("Demo mode or recording already in progress");
                return false;
        }

        g_demo_mode_in_progress = true;
        k_thread_create(&demo_mode_thread, demo_mode_stack,
                        K_THREAD_STACK_SIZEOF(demo_mode_stack),
                        demo_mode_thread_entry, NULL, NULL, NULL,
                        DEMO_MODE_PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&demo_mode_thread, "demo_mode");
        return true;
}

static bool start_long_record_async(void)
{
        if (g_mic_recording_in_progress)
        {
                LOG_WRN("Long recording requested but recording already in progress");
                return false;
        }

        g_mic_recording_in_progress = true;
        rgb_led_set_color(&RGB_COLOR_RED);
        k_thread_create(&hold_record_thread, hold_record_stack,
                        K_THREAD_STACK_SIZEOF(hold_record_stack),
                        hold_record_thread_entry, NULL, NULL, NULL,
                        MIC_RECORD_PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&hold_record_thread, "hold_record");
        return true;
}

static void start_advertising_worst_case_mode(void)
{
        LOG_INF("Entering Advertising Worst Case mode");
        (void)ads_service_set_forced_streaming(false);
        (void)ble_manager_set_adv_mode(BLE_MANAGER_ADV_MODE_WORST_CASE);
        (void)ble_manager_set_radio_enabled(true);
}

static void start_streaming_worst_case_mode(void)
{
        LOG_INF("Entering Streaming Worst Case mode");
        g_config.ble_transfer_channel_mask = 0xFF;
        g_config.ble_transfer_channel_count = 8;
        for (int i = 0; i < 8; i++)
        {
                g_config.ble_transfer_channels[i] = i;
        }
        (void)ble_manager_set_adv_mode(BLE_MANAGER_ADV_MODE_NORMAL);
        (void)ble_manager_set_radio_enabled(true);
        (void)ads_service_set_forced_streaming(true);
}

static void apply_button_action(button_event_t event)
{
        button_action_t action;

        if (event <= BUTTON_EVENT_NONE || event >= BUTTON_EVENT_COUNT)
        {
                return;
        }

        action = (button_action_t)g_config.button_action_map[event];
        LOG_INF("Button action dispatch: %s -> %s",
                button_event_to_string(event),
                button_action_to_string(action));

        switch (action)
        {
        case BUTTON_ACTION_NONE:
        case BUTTON_ACTION_LOG_ONLY:
                break;
        case BUTTON_ACTION_GREEN_LED_DEMO:
                run_green_led_demo();
                break;
        case BUTTON_ACTION_MIC_RECORD_TEST:
                (void)start_mic_record_test_async();
                break;
        case BUTTON_ACTION_DEMO_MODE:
                (void)start_demo_mode_async();
                break;
        case BUTTON_ACTION_LONG_RECORD_10S:
                (void)start_long_record_async();
                break;
        case BUTTON_ACTION_BLE_RADIO_TOGGLE:
                (void)ble_manager_set_radio_enabled(!ble_manager_is_radio_enabled());
                break;
        case BUTTON_ACTION_BLE_RADIO_ON:
                (void)ble_manager_set_radio_enabled(true);
                break;
        case BUTTON_ACTION_BLE_RADIO_OFF:
                (void)ble_manager_set_radio_enabled(false);
                break;
        case BUTTON_ACTION_RECORDING_TOGGLE:
                set_ads_sampling_enabled(!g_ads_sampling_active);
                break;
        case BUTTON_ACTION_ADVERTISING_WORST_CASE:
                start_advertising_worst_case_mode();
                break;
        case BUTTON_ACTION_STREAMING_WORST_CASE:
                start_streaming_worst_case_mode();
                break;
        case BUTTON_ACTION_SHOW_BATTERY_LED_STATUS:
                show_battery_led_status();
                break;
        case BUTTON_ACTION_PLAY_BATTERY_PERCENT_SOUND:
        {
                /* Play nearest 10%% battery WAV from SD card (SOUNDS/NN.WAV) */
                uint8_t pct = battery_monitor_get_level_percent();
                int bucket = (((int)pct + 5) / 10) * 10; /* round to nearest 10 */
                if (bucket < 0)
                        bucket = 0;
                if (bucket > 100)
                        bucket = 100;
                LOG_INF("Battery percent: %u%% -> bucket %d", pct, bucket);
                char fname[32];
                snprintf(fname, sizeof(fname), "SOUNDS/%d.WAV", bucket);
                audio_player_result_t audio_rc = audio_player_queue_play_wav(fname);
                if (audio_rc != AUDIO_PLAYER_OK)
                {
                        LOG_WRN("Failed to enqueue battery sound %s: %d", fname, audio_rc);
                        /* Fallback to 0.WAV if available */
                        (void)audio_player_queue_play_wav("SOUNDS/0.WAV");
                }
        }
        break;
        case BUTTON_ACTION_RELOAD_RUNTIME_CONFIG:
                if (system_config_load_from_sd("config.txt") == SYSTEM_CONFIG_OK)
                {
                        system_config_print_summary();
                        int vol_ret = i2s_out_set_volume(g_config.firmware_max_volume_percent);
                        if (vol_ret < 0)
                        {
                                LOG_WRN("Failed to apply firmware max volume after config reload: %d", vol_ret);
                        }
                }
                break;
        case BUTTON_ACTION_BLE_CONFIG_TOGGLE:
        {
                /* Persistently flip BLE_ENABLED in config.txt; effective on the
                 * next restart. White flash confirms; red flash = write failed. */
                bool new_ble = false;
                system_config_result_t cfg_rc =
                        system_config_toggle_ble_enabled_persisted("config.txt", &new_ble);
                if (cfg_rc == SYSTEM_CONFIG_OK)
                {
                        LOG_INF("config.txt BLE_ENABLED -> %d (active after restart)", new_ble);
                        rgb_led_set_color(&RGB_COLOR_WHITE);
                        k_sleep(K_MSEC(300));
                        rgb_led_off();
                }
                else
                {
                        LOG_WRN("BLE config toggle failed (%d)", cfg_rc);
                        rgb_led_set_color(&RGB_COLOR_RED);
                        k_sleep(K_MSEC(300));
                        rgb_led_off();
                }
        }
        break;
        default:
                LOG_WRN("Unhandled button action %d", action);
                break;
        }
}

/* Microphone recording thread entry point */
static void mic_recording_thread_entry(void *p1, void *p2, void *p3)
{
        LOG_INF("Starting microphone recording in dedicated thread...");

        /* Brief LED feedback to indicate recording start */
        rgb_led_set_color(&RGB_COLOR_BLUE);
        k_sleep(K_MSEC(50));
        rgb_led_off();

        /* Use test overrides if set, otherwise defaults */
        uint32_t sample_rate = (g_mic_test_sample_rate_hz > 0) ? (uint32_t)g_mic_test_sample_rate_hz : (uint32_t)MIC_SAMPLE_RATE_HZ;
        uint32_t duration_ms = (g_mic_test_duration_ms > 0) ? (uint32_t)g_mic_test_duration_ms : (uint32_t)MIC_RECORDING_DURATION_MS;

        /* Clear overrides immediately so subsequent recordings use defaults */
        g_mic_test_sample_rate_hz = 0;
        g_mic_test_duration_ms = 0;

        /* Stop ADS131 sampling to prevent interrupts and noise during recording */
        ads131m08_ti_stop_sampling();

        /* Stop BLE advertising if not connected to reduce radio noise */
        bool was_advertising = false;
        if (!ble_manager_is_connected())
        {
                ble_manager_stop_advertising();
                was_advertising = true;
        }

        /* Pause ADS131 logging to prevent SD card contention and reduce interrupts during recording */
        data_logger_pause_ads131_logging(true);

        /* Record microphone audio into RAM buffer */
        g_mic_samples_recorded = 0;
        nordic_adc_result_t mic_ret = nordic_adc_record_microphone(
            sample_rate,
            duration_ms,
            g_mic_recording_buffer,
            MIC_BUFFER_SIZE_SAMPLES,
            &g_mic_samples_recorded);

        if (mic_ret != NORDIC_ADC_OK || g_mic_samples_recorded == 0)
        {
                LOG_ERR("Failed to record microphone or no samples recorded: %d (samples=%zu)", mic_ret, g_mic_samples_recorded);
                rgb_led_set_color(&RGB_COLOR_RED);
                k_sleep(K_MSEC(200));
                rgb_led_off();
                g_mic_recording_in_progress = false;
                /* Resume ADS131 logging */
                data_logger_pause_ads131_logging(false);
                ads131m08_ti_start_sampling();
                if (was_advertising)
                {
                        ble_manager_start_advertising();
                }
                return;
        }

        LOG_INF("Microphone recording complete: %zu samples at %u Hz", g_mic_samples_recorded, sample_rate);

        /* Trigger playback immediately after recording (runs in parallel with saving) */
        /* k_work_submit(&mic_playback_work); */

        /* DEBUG: Print first 10 samples to verify recording buffer content */
        LOG_INF("DEBUG: First 10 recorded samples:");
        for (int i = 0; i < 10; i++)
        {
                LOG_INF("  [%d]: %d", i, g_mic_recording_buffer[i]);
        }

        /* Log microphone recording event */
        event_log_msg_t mic_msg = {
            .event_type = 0xFFFFFFFBu, /* Microphone event */
            .timestamp_us = (uint64_t)k_uptime_get() * 1000ULL,
            .data = {1, (int32_t)g_mic_samples_recorded, (int32_t)sample_rate, 0, 0, 0, 0, 0}, /* 1 = record */
            .status = 0};
        event_log_enqueue(&mic_msg);

        /* Brief LED feedback to indicate recording complete */
        rgb_led_set_color(&RGB_COLOR_GREEN);
        k_sleep(K_MSEC(100));
        rgb_led_off();

        /* Save recording to SD card as WAV file */
        LOG_INF("Saving recording to SD card...");

        /* ADS131 logging is already paused */

        static uint32_t recording_number = 0;
        char wav_filename[32];
        snprintf(wav_filename, sizeof(wav_filename), "REC%05u.WAV", recording_number++);

        wav_writer_result_t wav_ret = wav_writer_write_file(
            wav_filename,
            g_mic_recording_buffer,
            g_mic_samples_recorded,
            sample_rate,
            1); /* 1 = mono */

        /* Resume ADS131 logging */
        data_logger_pause_ads131_logging(false);
        ads131m08_ti_start_sampling();
        if (was_advertising)
        {
                ble_manager_start_advertising();
        }

        if (wav_ret != WAV_WRITER_OK)
        {
                LOG_ERR("Failed to save WAV file: %d", wav_ret);
                rgb_led_set_color(&RGB_COLOR_RED);
                k_sleep(K_MSEC(500));
                rgb_led_off();
        }
        else
        {
                LOG_INF("Recording saved as %s", wav_filename);
                /* Green LED pulse to indicate successful save */
                rgb_led_set_color(&RGB_COLOR_GREEN);
                k_sleep(K_MSEC(200));
                rgb_led_off();
                k_sleep(K_MSEC(100));
                rgb_led_set_color(&RGB_COLOR_GREEN);
                k_sleep(K_MSEC(200));
                rgb_led_off();
        }

        g_mic_recording_in_progress = false;
}

/*
 * Startup "ready" animation: fast blue blink for 5 seconds.
 * Called after system initialization when the device is ready to run.
 */
static void startup_ready_animation(void)
{
        if (!rgb_led_is_initialized())
        {
                return;
        }

        const uint32_t total_ms = 5000;
        const uint32_t on_ms = 50;   /* LED on time */
        const uint32_t off_ms = 50;  /* LED off time */
        uint64_t end = k_uptime_get() + total_ms;

        while (k_uptime_get() < end)
        {
                rgb_led_set_color(&RGB_COLOR_BLUE);
                k_sleep(K_MSEC(on_ms));
                rgb_led_off();
                k_sleep(K_MSEC(off_ms));
        }
}

/*
 * Record microphone for given duration (ms) and stream to SD card as WAV.
 * Uses per-sample ADC reads and writes chunks to the filesystem to avoid large RAM usage.
 */
static int startup_record_microphone_to_wav(const char *filename, uint32_t sample_rate_hz, uint32_t duration_ms)
{
        if (!nordic_adc_is_ready())
        {
                LOG_ERR("ADC not ready for microphone recording");
                return -1;
        }

        size_t expected_samples = (size_t)((uint64_t)sample_rate_hz * duration_ms / 1000ULL);
        if (expected_samples == 0)
        {
                LOG_ERR("Requested zero samples for recording");
                return -1;
        }

        LOG_INF("Starting streaming microphone recording: %u Hz, %u ms, samples=%zu", sample_rate_hz, duration_ms, expected_samples);
        LOG_INF("Startup recording begins at uptime=%lld ms", (long long)k_uptime_get());

        /* Pause ADS131 logging/sampling to avoid concurrent SD access while writing WAV */
        data_logger_pause_ads131_logging(true);
        ads131m08_ti_stop_sampling();

        /* Write to temporary file first, then atomically rename to final name */
        char final_path[128];
        char tmp_path[128];
        snprintf(final_path, sizeof(final_path), "/SD:/%s", filename);
        snprintf(tmp_path, sizeof(tmp_path), "/SD:/%s/REC.TMP", RECORDINGS_DIR_NAME);

        /* Recovery for interrupted previous run:
         * - If STARTUP.TMP has payload and STARTUP.WAV is missing, promote TMP to WAV.
         * - Otherwise remove stale TMP if WAV already exists.
         */
        if (sd_card_manager_lock(1000) == 0)
        {
                struct fs_dirent tmp_st;
                struct fs_dirent final_st;
                int tmp_ret = fs_stat(tmp_path, &tmp_st);
                int final_ret = fs_stat(final_path, &final_st);

                if (tmp_ret == 0)
                {
                        bool promote_tmp = false;

                        if (final_ret != 0)
                        {
                                promote_tmp = (tmp_st.size >= 44);
                        }
                        else
                        {
                                /* If final file is empty/header-only or tmp is larger, recover tmp */
                                if (final_st.size <= 44 || tmp_st.size > final_st.size)
                                {
                                        promote_tmp = (tmp_st.size >= 44);
                                }
                        }

                        if (promote_tmp)
                        {
                                if (final_ret == 0)
                                {
                                        (void)fs_unlink(final_path);
                                }
                                int rec_ret = fs_rename(tmp_path, final_path);
                                LOG_WRN("Recovered interrupted startup recording: %s -> %s (size=%zu, ret=%d)",
                                        tmp_path, final_path, tmp_st.size, rec_ret);
                        }
                        else if (final_ret == 0)
                        {
                                (void)fs_unlink(tmp_path);
                        }
                }
                sd_card_manager_unlock();
        }

        struct fs_file_t file;
        fs_file_t_init(&file);
        int ret = sd_card_safe_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
        if (ret < 0)
        {
                LOG_ERR("Failed to open temp file %s: %d", tmp_path, ret);
                data_logger_pause_ads131_logging(false);
                ads131m08_ti_start_sampling();
                return ret;
        }

        /* Prepare and write empty header (we'll fill real sizes later) */
        typedef struct __attribute__((packed))
        {
                char riff_id[4];
                uint32_t file_size;
                char wave_id[4];
                char fmt_id[4];
                uint32_t fmt_size;
                uint16_t audio_format;
                uint16_t num_channels;
                uint32_t sample_rate;
                uint32_t byte_rate;
                uint16_t block_align;
                uint16_t bits_per_sample;
                char data_id[4];
                uint32_t data_size;
        } wav_header_t;

        wav_header_t header;
        memset(&header, 0, sizeof(header));
        memcpy(header.riff_id, "RIFF", 4);
        memcpy(header.wave_id, "WAVE", 4);
        memcpy(header.fmt_id, "fmt ", 4);
        header.fmt_size = 16;
        header.audio_format = 1;
        header.num_channels = 1; /* mono */
        header.sample_rate = sample_rate_hz;
        header.bits_per_sample = 16;
        header.byte_rate = sample_rate_hz * header.num_channels * (header.bits_per_sample / 8);
        header.block_align = header.num_channels * (header.bits_per_sample / 8);
        memcpy(header.data_id, "data", 4);
        header.data_size = 0; /* unknown yet */
        header.file_size = 36 + header.data_size;

        ret = sd_card_safe_write(&file, &header, sizeof(header));
        if (ret != sizeof(header))
        {
                LOG_ERR("Failed to write WAV header placeholder: %d", ret);
                sd_card_safe_close(&file);
                data_logger_pause_ads131_logging(false);
                ads131m08_ti_start_sampling();
                return -1;
        }

        /* Persist placeholder header immediately so file is never 0-byte after crash */
        (void)sd_card_safe_sync(&file);

        /* Stop periodic sampling while we do blocking ADC reads */
        nordic_adc_stop_periodic_sampling();

        /* Streaming buffer for samples */
        const size_t CHUNK_SAMPLES = STARTUP_CHUNK_SAMPLES;
        const size_t SYNC_INTERVAL_SAMPLES = 2048; /* sync every chunk for power-loss robustness */
        size_t chunk_idx = 0;
        size_t samples_written = 0;
        size_t samples_synced = 0;
        bool io_error = false;

        /* Simple integer exponential moving average DC remover */
        int32_t dc = 0;

        uint32_t sample_interval_us = 1000000U / sample_rate_hz;

        for (size_t i = 0; i < expected_samples; i++)
        {
                uint32_t start = k_cycle_get_32();

                /* Read single sample from ADC channel (AIN3) */
                int16_t raw = 0;
                struct adc_sequence mic_seq = {
                        .channels = BIT(3),
                        .buffer = &raw,
                        .buffer_size = sizeof(raw),
                        .resolution = NORDIC_ADC_RESOLUTION,
                        .oversampling = 3,
                        .calibrate = false};

                int r = adc_read(DEVICE_DT_GET(DT_NODELABEL(adc)), &mic_seq);
                if (r < 0)
                {
                        LOG_ERR("ADC read failed at sample %zu: %d", i, r);
                        break;
                }

                /* Update running DC estimate (N=128) */
                dc += ((int32_t)raw - dc) / 128;
                int32_t centered = (int32_t)raw - dc;

                /* Scale to 16-bit PCM similar to nordic_adc_record_microphone (shift left 7) */
                int32_t pcm = centered << 7;
                if (pcm > 32767)
                        pcm = 32767;
                if (pcm < -32768)
                        pcm = -32768;

                g_startup_chunk_buf[chunk_idx++] = (int16_t)pcm;

                if (chunk_idx >= CHUNK_SAMPLES)
                {
                        /* Write chunk to file */
                        size_t bytes = chunk_idx * sizeof(int16_t);
                        ssize_t wrote = sd_card_safe_write(&file, g_startup_chunk_buf, bytes);
                        if (wrote != (ssize_t)bytes)
                        {
                                LOG_ERR("Failed to write audio chunk: %d", (int)wrote);
                                io_error = true;
                                break;
                        }
                        samples_written += chunk_idx;
                        chunk_idx = 0;

                        if ((samples_written - samples_synced) >= SYNC_INTERVAL_SAMPLES)
                        {
                                header.data_size = (uint32_t)(samples_written * sizeof(int16_t));
                                header.file_size = 36 + header.data_size;
                                if (fs_seek(&file, 0, FS_SEEK_SET) == 0)
                                {
                                        ssize_t hw = sd_card_safe_write(&file, &header, sizeof(header));
                                        if (hw != (ssize_t)sizeof(header))
                                        {
                                                LOG_ERR("Failed to update WAV header during capture: %d", (int)hw);
                                                io_error = true;
                                                break;
                                        }
                                        if (fs_seek(&file, 0, FS_SEEK_END) != 0)
                                        {
                                                LOG_ERR("Failed to seek back to end after header update");
                                                io_error = true;
                                                break;
                                        }
                                        int sret = sd_card_safe_sync(&file);
                                        if (sret != 0)
                                        {
                                                LOG_ERR("Failed to sync startup recording chunk: %d", sret);
                                                io_error = true;
                                                break;
                                        }
                                        samples_synced = samples_written;
                                }
                                else
                                {
                                        LOG_ERR("Failed to seek to WAV header during capture");
                                        io_error = true;
                                        break;
                                }
                        }
                }

                /* Maintain sampling timing */
                uint32_t end = k_cycle_get_32();
                uint32_t spent_us = k_cyc_to_us_floor32(end - start);
                if (spent_us < sample_interval_us)
                {
                        k_busy_wait(sample_interval_us - spent_us);
                }
        }

        /* Write any remaining samples */
        if (chunk_idx > 0)
        {
                size_t bytes = chunk_idx * sizeof(int16_t);
                ssize_t wrote = sd_card_safe_write(&file, g_startup_chunk_buf, bytes);
                if (wrote == (ssize_t)bytes)
                {
                        samples_written += chunk_idx;
                }
                else
                {
                        LOG_ERR("Failed to write final audio chunk: %d", (int)wrote);
                        io_error = true;
                }
        }

        /* Update header sizes */
        header.data_size = (uint32_t)(samples_written * sizeof(int16_t));
        header.file_size = 36 + header.data_size;

        if (fs_seek(&file, 0, FS_SEEK_SET) == 0)
        {
                ssize_t hw = sd_card_safe_write(&file, &header, sizeof(header));
                if (hw != (ssize_t)sizeof(header))
                {
                        LOG_ERR("Failed to write final WAV header: %d", (int)hw);
                        io_error = true;
                }
                else
                {
                        int sret = sd_card_safe_sync(&file);
                        if (sret != 0)
                        {
                                LOG_ERR("Failed to sync final WAV data: %d", sret);
                                io_error = true;
                        }
                }
        }
        else
        {
                LOG_ERR("Failed to seek to beginning for final WAV header update");
                io_error = true;
        }

        sd_card_safe_close(&file);

        if (samples_written == 0 || io_error)
        {
                LOG_ERR("Startup recording incomplete (samples=%zu, io_error=%d), keeping temp file for recovery", samples_written, io_error ? 1 : 0);
                if (samples_written == 0 && sd_card_manager_lock(1000) == 0)
                {
                        (void)fs_unlink(tmp_path);
                        sd_card_manager_unlock();
                }
                nordic_adc_start_periodic_sampling();
                data_logger_pause_ads131_logging(false);
                ads131m08_ti_start_sampling();
                return -1;
        }

        /* Atomic-ish publish: remove old file and rename temp to final */
        if (sd_card_manager_lock(2000) == 0)
        {
                struct fs_dirent final_st;
                if (fs_stat(final_path, &final_st) == 0)
                {
                        (void)fs_unlink(final_path);
                }
                ret = fs_rename(tmp_path, final_path);
                sd_card_manager_unlock();
        }
        else
        {
                ret = -1;
        }

        if (ret != 0)
        {
                LOG_ERR("Failed to finalize startup WAV (rename %s -> %s): %d", tmp_path, final_path, ret);
        }

        /* Restart periodic sampling if desired */
        nordic_adc_start_periodic_sampling();
        data_logger_pause_ads131_logging(false);
        ads131m08_ti_start_sampling();

        LOG_INF("Recording finished and saved: %s (%u bytes)", filename, header.data_size);
        LOG_INF("Startup recording end uptime=%lld ms", (long long)k_uptime_get());

        return (ret == 0) ? 0 : -1;
}

/* Message queue for Nordic ADC samples. The SAADC samples every 10 s
 * (sample_interval_ms below), so 8 slots are 80 s of backlog; was 32 slots
 * (5+ min, 1792 B). Reduced in fw 0.0.91 to free RAM for the per-channel gain /
 * montage config and the noise test (link failed: RAM overflow by 1156 B). */
K_MSGQ_DEFINE(nordic_adc_msgq, sizeof(nordic_adc_msg_t), 8, 4);

/* Logger worker thread for Nordic ADC -- performs SD buffering outside interrupt context */
K_THREAD_STACK_DEFINE(nordic_logger_stack, 2048);
static struct k_thread nordic_logger_thread;
static void nordic_logger_thread_entry(void *p1, void *p2, void *p3)
{
        ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);

        nordic_adc_msg_t msg;
        while (1)
        {
                int rc = k_msgq_get(&nordic_adc_msgq, &msg, K_FOREVER);
                if (rc == 0)
                {
                        /* Only buffer into ADS131 binary log (no CSV SD logging, minimal console output) */
                        int32_t channels32[8];
                        for (int i = 0; i < 8; i++)
                        {
                                channels32[i] = msg.voltage_mv[i];
                        }
                        /* Convert battery channel (AI4) from raw mV to percentage:
                         * 2800 mV = 100%, 2500 mV = 0%; extrapolates linearly (can go negative) */
                        channels32[4] = ((int32_t)channels32[4] - 2500) * 100 / (2800 - 2500);
                        uint32_t nordic_sample_num = 0xFFFFFFFFu;
                        uint64_t timestamp_us = (uint64_t)msg.timestamp_ms * 1000ULL;
                        (void)data_logger_buffer_ads131_sample(
                            nordic_sample_num, timestamp_us, channels32, msg.status);
                }
                else
                {
                        k_sleep(K_MSEC(5));
                }
        }
}

static void nordic_adc_data_callback(const nordic_adc_sample_t *sample)
{
        /* ISR-safe: only queue the data, no heavy work or SD I/O here */
        nordic_adc_msg_t msg;
        msg.timestamp_ms = sample->timestamp_ms;
        for (int i = 0; i < 8; i++)
        {
                msg.voltage_mv[i] = sample->voltage_mv[i];
                msg.channels_raw[i] = sample->channels[i];
        }
        msg.status = 0;

        (void)k_msgq_put(&nordic_adc_msgq, &msg, K_NO_WAIT);
}

/* Button event callback function */
static void button_event_callback(button_event_t event)
{
        /* Always log into the EDF ButtonEvent slow channel (signal 10) so any
         * downstream tool can recover when the user pressed / clicked / etc. */
        data_logger_record_button_event((int)event);

        if (event == BUTTON_EVENT_PRESSED || event == BUTTON_EVENT_RELEASED)
        {
                LOG_INF("Button %s", event == BUTTON_EVENT_PRESSED ? "pressed" : "released");
                if (event == BUTTON_EVENT_RELEASED)
                {
                        /* No LED hold was started on press, so nothing to stop here. */
                }
                return;
        }

        LOG_INF("*** BUTTON GESTURE DETECTED: %s ***", button_event_to_string(event));

                /* Play LED feedback first, then execute action via executor after animation completes */
                switch (event) {
                case BUTTON_EVENT_SINGLE_CLICK:
                case BUTTON_EVENT_DOUBLE_CLICK:
                case BUTTON_EVENT_TRIPLE_CLICK: {
                        button_action_t configured = (button_action_t)g_config.button_action_map[event];
                        if (configured == BUTTON_ACTION_NONE) {
                                LOG_INF("Configured action NONE for %s; skipping enqueue", button_event_to_string(event));
                                break;
                        }
                        if (button_action_uses_direct_led_feedback(event)) {
                                LOG_INF("Skipping generic click LED feedback for %s -> %s",
                                        button_event_to_string(event), button_action_to_string(configured));
                                button_action_enqueue(event);
                                break;
                        }
                        uint8_t pulses = (event == BUTTON_EVENT_SINGLE_CLICK) ? 1 : ((event == BUTTON_EVENT_DOUBLE_CLICK) ? 2 : 3);
                        bool oneshot_ok = led_animation_play_oneshot(&RGB_COLOR_WHITE, pulses, 500, 200, event, led_oneshot_complete_cb, NULL);
                        if (!oneshot_ok) {
                                LOG_WRN("LED oneshot request failed for %s", button_event_to_string(event));
                                button_action_enqueue(event);
                        } else {
                                LOG_DBG("LED oneshot scheduled for %s", button_event_to_string(event));
                        }
                } break;
                default:
                        /* No specific LED pattern defined -> enqueue immediately if configured */
                                button_action_t configured = (button_action_t)g_config.button_action_map[event];
                                if (configured != BUTTON_ACTION_NONE) {
                                        button_action_enqueue(event);
                                } else {
                                        LOG_INF("Configured action NONE for %s; nothing to do", button_event_to_string(event));
                                }
                        break;
                }

        /* Continue with prior switch (if any) */

        switch (event)
        {
        case BUTTON_EVENT_SINGLE_CLICK:
        case BUTTON_EVENT_DOUBLE_CLICK:
        case BUTTON_EVENT_TRIPLE_CLICK:
        case BUTTON_EVENT_LONG_PRESS:
        case BUTTON_EVENT_EXTRA_LONG_PRESS:
        case BUTTON_EVENT_SHORT_SHORT_LONG:
        case BUTTON_EVENT_SHORT_LONG:
        case BUTTON_EVENT_LONG_SHORT:
        case BUTTON_EVENT_SHORT_LONG_SHORT:
        case BUTTON_EVENT_LONG_SHORT_SHORT:
                break;
        default:
                break;
        }
}

int main(void)
{
        int ret;

        /* Short delay to let DC/DC regulator and LFXO crystal stabilize after boot */
        k_sleep(K_MSEC(100));

        LOG_INF("=== Traumschreiber Board Bring-Up ===");
        LOG_INF("Firmware version: %s", FIRMWARE_VERSION_STR);

        /* Early RGB LED init; the start sequence itself runs after the SD init */
        const int rgb_ok = rgb_led_init();
        led_animation_init();

        fota_confirm_running_image_if_needed();

        /* No operation while USB is plugged in (E7). After the FOTA confirm, so
         * a freshly updated image booting on the charger is not reverted. */
        {
                int mv1 = charge_guard_read_vbus_mv();
                k_sleep(K_MSEC(20));
                int mv2 = charge_guard_read_vbus_mv();
                LOG_INF("VBUS check: AIN5 %d / %d mV (USB above %d mV)", mv1, mv2,
                        CHARGE_GUARD_THRESHOLD_MV);
                charge_guard_report_vbus_mv(mv1);
                charge_guard_report_vbus_mv(mv2);
                if (charge_guard_wait(K_NO_WAIT))
                {
                        charge_lockout_power_off(false, mv2);
                }
        }

        /* Early SD / data logger initialization so runtime config can be read first */
        LOG_INF("=== Early SD init for runtime configuration ===");
        sd_card_manager_init();
        /* Retry the SD init (fw 0.0.94): a card that does not answer the first
         * init used to leave the TS running WITHOUT SD (no EEG file, no boot
         * sound). Up to 6 tries, 500 ms apart. Note (03.10.): after repeated
         * resets the card can get stuck until it is powered off — no firmware
         * retry helps then (seen on the bench TS after many J-Link resets). */
        data_logger_result_t early_data_ret = DATA_LOGGER_ERROR;
        for (int attempt = 1; attempt <= 6; attempt++)
        {
                early_data_ret = data_logger_init();
                if (early_data_ret == DATA_LOGGER_OK)
                {
                        if (attempt > 1)
                        {
                                LOG_WRN("SD init succeeded on attempt %d", attempt);
                        }
                        break;
                }
                LOG_WRN("SD init attempt %d failed (%d) - retry in 500 ms", attempt, early_data_ret);
                k_sleep(K_MSEC(500));
        }
        /* Start sequence only once the SD is up (since 03.10.2026): fade to green
         * (0.5 s), stays on through the boot sound, then two blinks + 1 s dark +
         * blue advertising pulse (led_animation_boot_done after the sound). */
        if (rgb_ok == 0) {
                led_animation_play_startup();
        }
        if (early_data_ret != DATA_LOGGER_OK)
        {
                LOG_WRN("Early data logger init failed: %d - runtime config will use defaults", early_data_ret);
                system_config_init_defaults();
        }
        else
        {
                LOG_INF("Data logger initialized for runtime config read");
                (void)data_logger_write_firmware_version_file(FIRMWARE_VERSION_STR);

                system_config_result_t cfg_ret = system_config_load_from_sd("config.txt");
                if (cfg_ret == SYSTEM_CONFIG_OK)
                {
                        LOG_INF("=== runtime config applied (config.txt) ===");

                        /* Print a concise summary from system_config */
                        system_config_print_summary();
                }
                else if (cfg_ret == SYSTEM_CONFIG_NO_SD)
                {
                        LOG_WRN("Runtime config: no SD card available, using defaults");
                }
                else
                {
                        LOG_WRN("Runtime config: failed to load config.txt (code=%d), using defaults", cfg_ret);
                }
        }

        /* BLE runtime name will be applied inside ble_manager after bt_enable() */
        LOG_INF("BLE runtime name will be applied by BLE manager at init time");

        /* RGB LED already initialized at top of main() for startup blink */
        LOG_INF("=== RGB LED System (already initialized) ===");

        /* Initialize Button system */
        LOG_INF("=== Initializing Button System ===");
        button_config_t button_config = {
            .callback = button_event_callback,
                        .multi_click_timeout_ms = g_config.button_multi_click_timeout_ms,
                        .debounce_time_ms = 30,
                        .long_press_ms = g_config.button_long_press_ms,
                        .extra_long_press_ms = g_config.button_extra_long_press_ms,
        };

        ret = button_init(&button_config);
        if (ret < 0)
        {
                LOG_ERR("Button initialization failed: %d", ret);
        }
        else
        {
                LOG_INF("Button system initialized successfully");
                LOG_INF("Button is ready - try single click and double click!");
        }

        /* Initialize microphone playback work item */
        k_work_init_delayable(&mic_playback_work, mic_playback_work_handler);
        /* k_work_init(&mic_recording_work, mic_recording_work_handler); */
        LOG_INF("Microphone recording/playback system initialized");
        LOG_INF("- Sample rate: %u Hz", MIC_SAMPLE_RATE_HZ);
        LOG_INF("- Recording duration: %u ms", MIC_RECORDING_DURATION_MS);
        LOG_INF("- Buffer size: %u samples (%u bytes)",
                MIC_BUFFER_SIZE_SAMPLES, MIC_BUFFER_SIZE_SAMPLES * 2);

        /* Initialize stimulator subsystem (includes LED and audio) */
        LOG_INF("=== Initializing Stimulator Subsystem ===");
        ret = stimulator_init();

        if (ret < 0)
        {
                LOG_ERR("Failed to initialize stimulator subsystem: %d", ret);
        }
        else
        {
                LOG_INF("Stimulator subsystem initialized successfully");

                ret = i2s_out_set_volume(g_config.firmware_max_volume_percent);
                if (ret < 0)
                {
                        LOG_WRN("Failed to apply firmware max volume after stimulator init: %d", ret);
                }
                else
                {
                        LOG_INF("Applied firmware max volume: %u%%", g_config.firmware_max_volume_percent);
                }

                /* Startsound (START_SOUND) wird weiter unten SYNCHRON abgespielt —
                 * VOR dem ADS131-Recording, damit das (evtl. lange) Lied vollstaendig
                 * und ohne SD-Konflikt mit dem BDF-Writer laeuft. Hier nur i2s bereit. */
        }

        /* Data logger was initialized earlier for runtime configuration read */
        LOG_INF("Data logger already initialized for runtime config");

        // Create event logger thread AFTER data logger init to ensure logging system is ready
        k_thread_create(&event_logger_thread, event_logger_stack,
                        K_THREAD_STACK_SIZEOF(event_logger_stack),
                        event_logger_thread_entry, NULL, NULL, NULL,
                        K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
        k_thread_name_set(&event_logger_thread, "event_logger");
        k_thread_create(&action_exec_thread, action_exec_stack,
                        K_THREAD_STACK_SIZEOF(action_exec_stack),
                        action_exec_entry, NULL, NULL, NULL,
                        K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
        k_thread_name_set(&action_exec_thread, "action_exec");

        /* Initialize BLE manager */
        LOG_INF("=== Initializing BLE Manager ===");
        bool skip_ble = !g_config.ble_enabled;
#ifdef TS_HW_SELFTEST
        if (g_config.selftest2)
        {
                /* HW-Selbsttest: BLE gar nicht hochfahren -> kein Advertising,
                 * kein blaues Advertising-Blinken (LED bleibt fuer den Test frei). */
                skip_ble = true;
        }
#endif
        if (skip_ble)
        {
                LOG_WRN("BLE deaktiviert (Config oder HW-Selbsttest); skipping BLE initialization");
                ret = BLE_MANAGER_NOT_INITIALIZED;
        }
        else
        {
                ret = ble_manager_init();
                if (ret != BLE_MANAGER_OK)
                {
                        LOG_ERR("Failed to initialize BLE manager: %d", ret);
                        /* Flash red LED to indicate BLE error */
                        rgb_led_set_color(&RGB_COLOR_RED);
                        k_sleep(K_MSEC(2000));
                        rgb_led_off();
                        /* Continue without BLE functionality */
                }
                else
                {
                        LOG_INF("BLE manager initialized successfully");

                        /* Start BLE advertising */
                        ret = ble_manager_start_advertising();
                        if (ret != BLE_MANAGER_OK)
                        {
                                LOG_ERR("Failed to start BLE advertising: %d", ret);
                        }
                        else
                        {
                                LOG_INF("BLE advertising started - device is discoverable!");
                                LOG_INF("Connect with nRF Connect or your Flutter app");
                                LOG_INF("Service UUID: 00001523-1212-efde-1523-785feabcd123");
                                led_animation_start_advertising();
                        }
                }
        }

        // KA
        // k_sleep(K_SECONDS(1));

        /* Initialize Nordic SAADC for built-in analog inputs */
        LOG_INF("=== Initializing Nordic nRF52840 SAADC (AI0-AI7) ===");

        // Create nordic logger thread before starting ADC to ensure queue is serviced
        k_thread_create(&nordic_logger_thread, nordic_logger_stack,
                        K_THREAD_STACK_SIZEOF(nordic_logger_stack),
                        nordic_logger_thread_entry, NULL, NULL, NULL,
                        K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
        k_thread_name_set(&nordic_logger_thread, "nordic_logger");

        nordic_adc_config_t nordic_adc_config = {
            .data_callback = nordic_adc_data_callback,
            .sample_interval_ms = 10000, /* Sample every 10 seconds */
            .auto_start = true           /* Start sampling automatically */
        };

        nordic_adc_result_t nordic_adc_ret = nordic_adc_init(&nordic_adc_config);
        if (nordic_adc_ret != NORDIC_ADC_OK)
        {
                LOG_ERR("Nordic ADC initialization failed: %d", nordic_adc_ret);
                // /* Flash red LED to indicate ADC error */
                // rgb_led_set_color(&RGB_COLOR_RED);
                // k_sleep(K_MSEC(1000));
                // rgb_led_off();
        }
        else
        {
                LOG_INF("Nordic ADC initialized successfully");

                // /* Flash green LED to indicate ADC success */
                // rgb_led_set_color(&RGB_COLOR_GREEN);
                // k_sleep(K_MSEC(500));
                // rgb_led_off();
        }

        /* Initialize Battery Monitor — AFTER nordic_adc_init: its initial
         * boot-time reading needs the SAADC, and with the old order the
         * "initial update" hit the ADC-not-ready fallback on EVERY boot,
         * leaving the battery level at 0 % until the first 30-s tick (the
         * user-visible "battery unknown for half a minute" after power-on). */
        LOG_INF("=== Initializing Battery Monitor ===");
        ret = battery_monitor_init();
        if (ret < 0)
        {
                LOG_ERR("Failed to initialize battery monitor: %d", ret);
                /* Continue without battery monitoring */
        }
        else
        {
                LOG_INF("Battery monitor initialized successfully");
        }

#ifdef TS_HW_SELFTEST
        /* Hardware-Selbsttest: NUR wenn SD-config.txt "TEST2 = 1" setzt. MUSS HIER
         * laufen — VOR ADS131-Init/Sampling (unten). Die BDF-Aufnahme wird beim
         * ersten ADS131-Sample lazy geoeffnet und OFFEN gehalten; das belegt die
         * FATFS-Open-File-Slots, sodass die Schreibzugriffe des Selbsttests
         * (BRIGHT.TXT / MICTEST.WAV / MICSTAT.TXT) fehlschlagen (leerer SELFTEST-
         * Ordner). Hier ist die SD ruhig. ts_selftest_run() kehrt nie zurueck
         * (endet mit dauerhaft gruener LED) -> es startet nie ein Recording. */
        if (g_config.selftest2)
        {
                LOG_INF("config.txt TEST2=1 -> starte HW-Selbsttest (vor ADS131/Recording)");
                ts_selftest_run(); /* never returns */
        }
        else
        {
                LOG_INF("config.txt TEST2 != 1 -> normaler Betrieb (kein Selbsttest)");
        }
#endif

        /* === Startsound (START_SOUND) SYNCHRON abspielen — VOR dem Recording ===
         * Das (evtl. lange) Lied laeuft vollstaendig, solange die SD noch frei ist
         * (der BDF-Writer startet erst mit dem ADS131-Sampling weiter unten). Sauberer
         * Playback: Unity-Gain, kein Kompressor -> so laut wie moeglich ohne Clipping.
         * Blockiert bis das Lied fertig ist; BLE laeuft in seinem eigenen Thread weiter. */
        {
                const char *snd = (g_config.start_sound[0] != '\0') ? g_config.start_sound : "TSBOOT.WAV";
                char snd_path[64];
                if (strchr(snd, '/') != NULL) {
                        snprintf(snd_path, sizeof(snd_path), "%s", snd);
                } else {
                        snprintf(snd_path, sizeof(snd_path), "SOUNDS/%s", snd);
                }
                LOG_INF("Startsound (START_SOUND): spiele %s @ %u%% synchron, dann Recording",
                        snd_path, g_config.start_sound_volume);
                audio_player_result_t aprc = audio_player_play_wav_clean_from_sd(snd_path, g_config.start_sound_volume);
                if (aprc != AUDIO_PLAYER_OK && strcmp(snd, "TSBOOT.WAV") == 0) {
                        /* SD cards prepared before 0.0.94 have no TSBOOT.WAV. */
                        LOG_WRN("Startsound %s fehlt (%d) -> SOUNDS/BELL2.WAV", snd_path, aprc);
                        aprc = audio_player_play_wav_clean_from_sd("SOUNDS/BELL2.WAV", g_config.start_sound_volume);
                }
                if (aprc != AUDIO_PLAYER_OK) {
                        LOG_WRN("Startsound %s fehlgeschlagen: %d", snd_path, aprc);
                }
                /* LED: two green blinks, 1 s dark, then the blue advertising pulse. */
                led_animation_boot_done();
        }

        LOG_INF("=== Initializing ADS131M08 ADC ===");

        /* Configure ADS131M08. PGA gain comes from the runtime config, which
         * is already loaded here (config.txt is parsed above, before the ADC
         * starts): ADS_GAIN for all channels (default ×4), overridden per
         * channel by ADS_GAIN_CH0..7. system_config_ads_gain_to_code() maps the
         * ×N multiplier to the chip's 3-bit PGA code (0..7); the driver writes
         * the per-channel codes into GAIN1/GAIN2. */
        ads131m08_config_t ads_config = {
            .sample_rate = 250,
            .enabled_channels = 0xFF, /* Enable all 8 channels */
            .enable_dc_block = true, /* Enable DC block filter */
            .data_ready_callback = ads131m08_ti_data_callback};
        for (int ch = 0; ch < 8; ch++)
        {
                ads_config.gain[ch] =
                    system_config_ads_gain_to_code(system_config_ads_gain_for_channel(ch));
        }
        LOG_INF("ADS131M08 gain from config: ×%u ×%u ×%u ×%u ×%u ×%u ×%u ×%u",
                (unsigned)system_config_ads_gain_for_channel(0),
                (unsigned)system_config_ads_gain_for_channel(1),
                (unsigned)system_config_ads_gain_for_channel(2),
                (unsigned)system_config_ads_gain_for_channel(3),
                (unsigned)system_config_ads_gain_for_channel(4),
                (unsigned)system_config_ads_gain_for_channel(5),
                (unsigned)system_config_ads_gain_for_channel(6),
                (unsigned)system_config_ads_gain_for_channel(7));

        ret = ads131m08_ti_init(&ads_config);
        if (ret != ADS131M08_OK)
        {
                LOG_ERR("ADS131M08 TI initialization failed: %d", ret);
                // rgb_led_set_color(&RGB_COLOR_RED);
                // k_sleep(K_SECONDS(2));
                // rgb_led_off();
                // /* Continue with RGB LED test mode */
        }
        else
        {
                LOG_INF("ADS131M08 TI initialized successfully");

                // /* Green LED to indicate successful initialization */
                // rgb_led_set_color(&RGB_COLOR_GREEN);
                // k_sleep(K_MSEC(1000));
                // rgb_led_off();

                /* Stop SAADC before starting ADS131 — SAADC current pulses couple
                 * into the 128× gain front-end and create ~10 s spikes. */
                nordic_adc_stop_periodic_sampling();

                /* Start continuous ADC sampling */
                ret = ads131m08_ti_start_sampling();
                if (ret != ADS131M08_OK)
                {
                        LOG_ERR("Failed to start ADC sampling: %d", ret);
                        // rgb_led_set_color(&RGB_COLOR_RED);
                        // k_sleep(K_SECONDS(1));
                        // rgb_led_off();
                }
                else
                {
                        LOG_INF("ADC sampling started successfully at %u Hz", ads_config.sample_rate);

                        /* Shorted-input noise measurement right after the start
                         * (config.txt NOISE_TEST_SECS, default 10, 0 = off). */
                        if (g_config.noise_test_secs > 0)
                        {
                                int nt = noise_test_start(g_config.noise_test_secs, "boot");
                                if (nt != 0)
                                {
                                        LOG_WRN("Noise test at boot not started: %d", nt);
                                }
                        }

                        // LOG_INF("");
                        // LOG_INF("========================================");
                        // LOG_INF("RUNNING DEBUG OSR REGISTER WRITE TEST");
                        // LOG_INF("========================================");
                        // int debug_ret = debug_ads131_osr_test();
                        // if (debug_ret == 0)
                        // {
                        //         LOG_INF("Debug OSR test completed successfully");
                        //         LOG_INF("Check RTT output and SD card data to verify OSR change");
                        // }
                        // else
                        // {
                        //         LOG_ERR("Debug OSR test failed with code: %d", debug_ret);
                        // }
                        // LOG_INF("========================================");
                        // LOG_INF("");
                }
        }

        /* Main monitoring loop - RGB LED and Button test mode */
        uint32_t loop_count = 0;

        /* Play startup-ready animation: fast blue blink for 5s */
        // LOG_INF("Playing startup ready animation (blue blink)");
        // startup_ready_animation();

        /* After the animation, record 10 seconds of microphone audio and save as WAV */
        // LOG_INF("Starting 10s microphone recording (saving to SD as %s)", g_startup_wav_name);
        // if (startup_record_microphone_to_wav(g_startup_wav_name, 8000, 10000) == 0)
        // {
        //         LOG_INF("Startup recording saved as %s", g_startup_wav_name);
        //         rgb_led_set_color(&RGB_COLOR_GREEN);
        //         k_sleep(K_MSEC(200));
        //         rgb_led_off();
        // }
        // else
        // {
        //         LOG_ERR("Startup recording failed");
        //         rgb_led_set_color(&RGB_COLOR_RED);
        //         k_sleep(K_MSEC(500));
        //         rgb_led_off();
        // }
        LOG_INF("=== Entering LED/Button Test Mode ===");
        LOG_INF("Press button for single/double click tests");
        LOG_INF("LED will cycle colors every 10 seconds");

        /* Log session start to CSV using RTT format */
        static char rtt_timestamp[32];
        format_rtt_timestamp(k_uptime_get(), rtt_timestamp, sizeof(rtt_timestamp));
        data_logger_log_rtt(rtt_timestamp, "System startup - Entering LED/Button Test Mode");

        k_sleep(K_MSEC(200));
        rgb_led_off();

        /* HW-Selbsttest-Hook wurde nach oben verschoben (vor ADS131-Init/Recording),
         * damit die SD ruhig ist und die Selbsttest-Dateien geschrieben werden. */

        while (1)
        {
                loop_count++;
                LOG_INF("=== Loop Count: %u ===", loop_count);

                // /* Log to CSV using RTT format */
                // format_rtt_timestamp(uptime, rtt_timestamp, sizeof(rtt_timestamp));
                // snprintf(status_msg, sizeof(status_msg), "Test Mode Status #%u - Runtime: %d ms", loop_count, (int)uptime);
                // data_logger_log_rtt(rtt_timestamp, status_msg);

                /* Briefly flash the LED to show the board is still alive */
                /* Convert runtime-configured percentages (0-100) into the 0-100 scale used by rgb_led API */
                /* The rgb_led API uses 0-100 percentage fields, so use values directly. */
                rgb_color_t flash_color;
                flash_color.red = g_config.led_flash_color[0];
                flash_color.green = g_config.led_flash_color[1];
                flash_color.blue = g_config.led_flash_color[2];
                // rgb_led_set_color(&flash_color);
                // {
                //         event_log_msg_t led_msg = {
                //             .event_type = 0xFFFFFFFDu,
                //             .timestamp_us = (uint64_t)k_uptime_get() * 1000ULL,
                //             .data = {(int32_t)g_config.led_flash_color[0], (int32_t)g_config.led_flash_color[1], (int32_t)g_config.led_flash_color[2], 0, 0, 0, 0, 0},
                //             .status = 0};
                //         (void)k_msgq_put(&event_log_msgq, &led_msg, K_NO_WAIT);
                // }

                // k_sleep(K_MSEC(200));
                // rgb_led_off();
                // /* Log LED off */
                // {
                //         event_log_msg_t led_msg = {
                //             .event_type = 0xFFFFFFFDu,
                //             .timestamp_us = (uint64_t)k_uptime_get() * 1000ULL,
                //             .data = {0, 0, 0, 0, 0, 0, 0, 0},
                //             .status = 0};
                //         (void)k_msgq_put(&event_log_msgq, &led_msg, K_NO_WAIT);
                // }
                // LOG_INF("LED: OFF");
                // format_rtt_timestamp(k_uptime_get(), rtt_timestamp, sizeof(rtt_timestamp));
                // data_logger_log_rtt(rtt_timestamp, "LED: OFF");

                // ads_log_registers();

                /* Log the last MTU size used for BLE notifications */
                if (ble_manager_is_connected())
                {
                        uint16_t mtu_size = ble_manager_get_mtu_size();
                        LOG_INF("Current BLE MTU size: %u bytes", mtu_size);
                        char mtu_msg[64];
                        snprintf(mtu_msg, sizeof(mtu_msg), "BLE MTU size: %u bytes", mtu_size);
                        format_rtt_timestamp(k_uptime_get(), rtt_timestamp, sizeof(rtt_timestamp));
                }
                else
                {
                        LOG_INF("BLE not connected, skipping MTU size log");
                }

                /* 10 s per loop, checking for USB once per second (E7). While
                 * the ADS records, the data logger reports AIN5 once per EDF
                 * record (inside the SD write window); an extra SAADC read here
                 * would couple into the EEG. */
                for (int sec = 0; sec < 10; sec++)
                {
                        if (!g_ads_sampling_active)
                        {
                                charge_guard_report_vbus_mv(charge_guard_read_vbus_mv());
                        }
                        if (charge_guard_wait(K_SECONDS(1)))
                        {
                                charge_lockout_power_off(g_ads_sampling_active,
                                                         charge_guard_trip_mv());
                        }
                }
        }

        /* Clean up data logger before exit (though we never reach here) */
        /* Flush any remaining ADS131 data */
        data_logger_flush_ads131_data();
        data_logger_deinit();
        return 0;
}
