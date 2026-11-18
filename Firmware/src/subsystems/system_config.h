#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "hw_abstraction/button.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        BUTTON_ACTION_NONE = 0,
        BUTTON_ACTION_LOG_ONLY,
        BUTTON_ACTION_GREEN_LED_DEMO,
        BUTTON_ACTION_MIC_RECORD_TEST,
        BUTTON_ACTION_DEMO_MODE,
        BUTTON_ACTION_LONG_RECORD_10S,
        BUTTON_ACTION_BLE_RADIO_TOGGLE,
        BUTTON_ACTION_BLE_RADIO_ON,
        BUTTON_ACTION_BLE_RADIO_OFF,
        BUTTON_ACTION_RECORDING_TOGGLE,
        BUTTON_ACTION_ADVERTISING_WORST_CASE,
        BUTTON_ACTION_STREAMING_WORST_CASE,
        BUTTON_ACTION_SHOW_BATTERY_LED_STATUS,
        BUTTON_ACTION_PLAY_BATTERY_PERCENT_SOUND,
        BUTTON_ACTION_RELOAD_RUNTIME_CONFIG,
        /* Persistently toggle BLE_ENABLED in config.txt on the SD card. Takes
         * effect on the next device restart (the live radio is left as-is). */
        BUTTON_ACTION_BLE_CONFIG_TOGGLE,
        BUTTON_ACTION_COUNT
    } button_action_t;

    typedef struct
    {
        bool ble_enabled;
        bool rtt_enabled;
        int log_level;
        int sample_rate;
        char ble_device_name[32];   /* optional runtime override for BLE device name; empty = use default */
        uint8_t led_flash_color[3]; /* LED flash color as R,G,B percentages (0-100) */
        /* BLE compression bit depth in bits per channel. 0 = disabled. Allowed: 0..32.
         * Example: 0 => no compression, 10 => 10-bit delta-encoding (default), 12 => 12-bit, etc. */
        uint8_t ble_compress_bits;
        /* BLE transfer channel selection: bitmask of channels 0..7 (1=send), and explicit list */
        uint8_t ble_transfer_channel_mask;  /* bitmask: bit0 => CH0, bit7 => CH7; default = 0xFF (all) */
        uint8_t ble_transfer_channel_count; /* number of channels selected */
        uint8_t ble_transfer_channels[8];   /* explicit channel indices in transmission order */
        /* Physical delta max used by compression scaling. Value in ADC units; default 20000 */
        int32_t ble_delta_max;
        /* Transmit power in dBm, signed. Default: +8 (nRF52840 max). Range depends on controller. */
        int8_t tx_power_dbm;
        /* Firmware audio gain in percent. 100 = unity, values above 100 boost digitally. */
        uint16_t firmware_max_volume_percent;
        /* Boot sound played once at startup (START_SOUND). Filename in SOUNDS/ (e.g.
         * "SITTING.WAV") or a full path containing '/'. Default "BELL2.WAV". */
        char start_sound[48];
        /* Playback volume for the boot sound, percent (START_SOUND_VOLUME). Clean
         * path, no compressor. Kept modest by default so a near-full-scale music
         * file does not drive the MAX98357A/FS-1540 past the ~500 mW speaker /
         * 3.3 V amp limit (analog over-drive). Default 60. */
        uint16_t start_sound_volume;
        uint16_t button_multi_click_timeout_ms;
        uint16_t button_long_press_ms;
        uint16_t button_extra_long_press_ms;
        uint8_t button_action_map[BUTTON_EVENT_COUNT];
        /* ADS131M08 PGA gain (multiplier), applied to all 8 channels.
         * Allowed: 1,2,4,8,16,32,64,128. Default 4 (±300 mV input range —
         * robust against electrode DC offset on sweaty/forehead sites while
         * keeping fine resolution in the 24-bit BDF output). Higher gain =
         * finer LSB but smaller DC headroom (×128 → only ±9.4 mV → rails). */
        uint8_t ads_gain;
        /* Per-channel override of ads_gain (config.txt ADS_GAIN_CH0..ADS_GAIN_CH7).
         * 0 = not set → the channel uses ads_gain. Lets e.g. a bipolar chin-EMG
         * channel run at ×32 (ADC noise 2.88 → 0.77 µV RMS at OSR 4096, see
         * differential chin EMG) while EEG/EOG stay
         * at ×4. The ADS131M08 sets the PGA per channel (GAIN1/GAIN2). */
        uint8_t ads_gain_ch[8];
        /* Montage text per channel (config.txt CH_LABEL0..CH_LABEL7), written into
         * the BDF "transducer" field. The 16-char label stays "EEG Chn" because
         * readers select signal channels by the "EEG" prefix. Empty = not set. */
        char ch_label[8][41];
        /* Noise measurement with shorted inputs (input mux → AGND) right after
         * the ADC starts, in seconds (config.txt NOISE_TEST_SECS). 0 = off.
         * Also triggerable over BLE (ADS control 0x50). Default 10. */
        uint16_t noise_test_secs;
        /* fs_sync interval for the BDF data file and its .SNX index, in seconds
         * (config.txt SD_SYNC_INTERVAL_S, 1..60). The ~300 ms programming phase
         * after fs_sync couples into high-impedance inputs (SD write interference
         * step 3c); syncing every 10 s instead of every 1-s record removes 9 of
         * 10 such steps. Cost: on sudden power loss up to this many seconds of
         * the SD copy are lost (the app keeps its live copy). Default 10;
         * 1 = old behaviour. */
        uint8_t sd_sync_interval_s;

        /* SD-config TEST mode (config.txt key `TEST`). When > 0 the firmware
         * DISCARDS the real ADS131M08 samples and feeds a synthetic pure-tone
         * staircase instead: [test_block_secs] seconds per tone, stepping
         * 1,2,3,…,30 Hz then looping back to 1 Hz. Identical value on all 8
         * channels, DC-centred, amplitude 80000 counts (~±2.9 mV @gain4 →
         * inside the app's ±4 mV int16 window, no clipping). Injected at the ADS
         * read point, so SD (BDF) AND BLE both carry the test signal while
         * sample_number/timestamp_us keep running normally. 0 = off (normal
         * acquisition). `TEST = 5` → 5 s blocks; `TEST = 300` → 300 s blocks. */
        int test_block_secs;

        /* ---- Deterministic replay / simulation mode (DEBUG; see
         * data integrity analysis). All zero/false in normal
         * operation. When sim_mode is set, the firmware treats sim_file as the
         * currently-running recording, replays it as live 0xC0 notifications
         * from sim_start_sn, and serves backfill from the same file — so the
         * "7 h then long backfill with disconnects" case is reproducible on
         * demand without a real night. ---- */
        bool sim_mode;
        char sim_file[32];         /* e.g. "AD0042.BDF" (relative to ADC dir) */
        uint32_t sim_start_sn;     /* live replay start sn ("7h" = 6300000) */
        uint32_t sim_live_speed;   /* speed multiplier; 0/1 = real time */
        uint32_t sim_drop_period_s; /* SIM_DROP_PATTERN "Os@Ps": period P (s) */
        uint32_t sim_drop_outage_s; /* SIM_DROP_PATTERN "Os@Ps": outage O (s) */
        /* SIM_GAP_SN=lo-hi,lo-hi,... : ranges present ONLY on device (withheld
         * from BOTH live and backfill → exercises the leak / self-describing
         * terminal path R5). Bounded to 4 ranges. */
        uint32_t sim_gap_lo[4];
        uint32_t sim_gap_hi[4];
        uint8_t sim_gap_count;

        /* HW-Selbsttest beim Boot (nur TS_HW_SELFTEST-Build): config.txt "TEST2 = 1". */
        bool selftest2;
    } system_config_t;

    /* Convert an ADS_GAIN multiplier (1..128) to the ADS131M08 3-bit PGA code
     * (0..7). Rounds down to the nearest valid power of two and clamps. */
    uint8_t system_config_ads_gain_to_code(uint8_t multiplier);

    /* Effective PGA gain multiplier of one ADS channel (0..7): the per-channel
     * override ADS_GAIN_CHn if set, else ADS_GAIN. */
    uint8_t system_config_ads_gain_for_channel(int ch);

    extern system_config_t g_config;

    typedef enum
    {
        SYSTEM_CONFIG_OK = 0,
        SYSTEM_CONFIG_NO_SD = -1,
        SYSTEM_CONFIG_FILE_ERROR = -2,
        SYSTEM_CONFIG_PARSE_ERROR = -3
    } system_config_result_t;

    /**
     * Initialize g_config with default values
     */
    void system_config_init_defaults(void);

    /**
     * Load configuration from a file on the mounted SD card. Filename is relative
     * to the mount point (e.g. "config.txt"). If file not found or read fails,
     * defaults are kept and a warning is logged — and, when the card is mounted,
     * a default config file is created via system_config_write_defaults_to_sd()
     * so the device stays configurable from the app.
     */
    system_config_result_t system_config_load_from_sd(const char *filename);

    /**
     * Write a minimal config file with the CURRENT g_config values to the
     * mounted SD card, overwriting any existing file of that name.
     *
     * Called automatically when a mounted card carries no usable config: without
     * a file there the app has nothing to display or edit and could only report
     * "no config found", leaving the device unconfigurable. The generated file
     * holds only the everyday settings, each on its default, so writing it
     * changes no behaviour; test/simulation keys are deliberately omitted.
     *
     * @return SYSTEM_CONFIG_OK on success, SYSTEM_CONFIG_NO_SD when no card is
     *         mounted, SYSTEM_CONFIG_FILE_ERROR when the write fails.
     */
    system_config_result_t system_config_write_defaults_to_sd(const char *filename);

    /**
     * Print a concise summary of the runtime configuration via LOG_INF.
     * Useful for debugging and verifying runtime values loaded from SD card.
     */
    void system_config_print_summary(void);

    /**
     * Flip the persisted BLE_ENABLED setting in the given config file on the SD
     * card (rewriting the BLE_ENABLED line in place, or appending it if absent,
     * while preserving all other lines and comments). The change takes effect on
     * the next device restart; the live BLE radio is NOT touched here.
     *
     * @param filename       Config file name relative to mount point (e.g. "config.txt").
     * @param out_new_value  Optional; set to the new BLE_ENABLED value on success.
     * @return SYSTEM_CONFIG_OK on success, or an error code (NO_SD / FILE_ERROR).
     */
    system_config_result_t system_config_toggle_ble_enabled_persisted(
        const char *filename, bool *out_new_value);

    const char *button_action_to_string(button_action_t action);

    bool button_action_from_name(const char *name, button_action_t *out_action);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_CONFIG_H */
