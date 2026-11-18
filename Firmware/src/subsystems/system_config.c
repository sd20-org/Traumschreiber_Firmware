#include "system_config.h"
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include "data_logger.h"
#include "i2s_out.h"

LOG_MODULE_REGISTER(system_config, LOG_LEVEL_DBG);

system_config_t g_config;

/* Shared SD config I/O scratch buffer (~config file size + headroom; default
 * config.txt is ~1.8 KB). Used by both load and the persisted BLE toggle;
 * these never run concurrently (boot, or sequentially on the single
 * button-action thread), so one buffer is safe. Sized at 2048 (down from 4096;
 * config.txt is ~1.8 KB so this still has headroom) to fund the larger
 * action-thread stack needed for FATFS writes AND stay net-neutral on RAM on
 * the RAM-tight 2603 build (action stack +2048, this buffer −2048). */
static char s_cfg_iobuf[2048];

static void system_config_set_default_button_actions(void)
{
    for (int i = 0; i < BUTTON_EVENT_COUNT; i++)
    {
        g_config.button_action_map[i] = BUTTON_ACTION_LOG_ONLY;
    }

    g_config.button_action_map[BUTTON_EVENT_SINGLE_CLICK] = BUTTON_ACTION_GREEN_LED_DEMO;
    g_config.button_action_map[BUTTON_EVENT_DOUBLE_CLICK] = BUTTON_ACTION_MIC_RECORD_TEST;
    /* Default: triple (3x short) click persistently toggles BLE_ENABLED in
     * config.txt and flashes the LED white (can be overridden via config.txt). */
    g_config.button_action_map[BUTTON_EVENT_TRIPLE_CLICK] = BUTTON_ACTION_BLE_CONFIG_TOGGLE;
    g_config.button_action_map[BUTTON_EVENT_LONG_PRESS] = BUTTON_ACTION_LONG_RECORD_10S;
    g_config.button_action_map[BUTTON_EVENT_SHORT_SHORT_LONG] = BUTTON_ACTION_ADVERTISING_WORST_CASE;
    g_config.button_action_map[BUTTON_EVENT_SHORT_LONG] = BUTTON_ACTION_STREAMING_WORST_CASE;
    g_config.button_action_map[BUTTON_EVENT_LONG_SHORT] = BUTTON_ACTION_BLE_RADIO_TOGGLE;
    g_config.button_action_map[BUTTON_EVENT_EXTRA_LONG_PRESS] = BUTTON_ACTION_RELOAD_RUNTIME_CONFIG;
}

void system_config_init_defaults(void)
{
    g_config.ble_enabled = true;
    g_config.rtt_enabled = true;
    g_config.log_level = 1;
    g_config.sample_rate = 250;
    g_config.test_block_secs = 0; /* TEST mode off by default (normal ADS acquisition) */
    g_config.ble_device_name[0] = '\0';
    /* Default LED flash color: cyan-ish (R,G,B) percentages */
    g_config.led_flash_color[0] = 0;   /* R % */
    g_config.led_flash_color[1] = 100; /* G % */
    g_config.led_flash_color[2] = 100; /* B % */
    /* Compression is disabled; BLE always transfers raw 32-bit samples. */
    g_config.ble_compress_bits = 0;
    /* By default, transfer all 8 channels via BLE */
    g_config.ble_transfer_channel_mask = 0xFF;
    g_config.ble_transfer_channel_count = 8;
    for (int i = 0; i < 8; i++)
        g_config.ble_transfer_channels[i] = i;
    /* Compression is disabled; keep delta_max at 0 for status compatibility only. */
    g_config.ble_delta_max = 0;
    /* Default transmit power in dBm (signed). MUST mirror CONFIG_BT_CTLR_TX_PWR_*
     * in prj.conf — that Kconfig is what actually drives the radio; this field
     * only reports the value (status dump, and the stubbed runtime HCI-VS setter
     * in ble_manager.c). Reduced +8 -> +4 on 2026-09-18 for EN 300 328 PSD
     * margin; the reasoning is in prj.conf. Not settable from config.txt any
     * more — deliberately firmware-only. */
    g_config.tx_power_dbm = 4;
    g_config.firmware_max_volume_percent = I2S_VOLUME_DEFAULT_PERCENT;
    g_config.button_multi_click_timeout_ms = 500;
    g_config.button_long_press_ms = 700;
    g_config.button_extra_long_press_ms = 2000;
    /* Default ADS gain ×4 = ±300 mV input range (sweat-proof headroom). */
    g_config.ads_gain = 4;
    /* No per-channel overrides, no montage labels (→ BDF transducer stays blank). */
    for (int i = 0; i < 8; i++)
    {
        g_config.ads_gain_ch[i] = 0;
        g_config.ch_label[i][0] = '\0';
    }
    /* 10 s shorted-input noise measurement after the ADC starts. */
    g_config.noise_test_secs = 10;
    /* fs_sync every 10 s instead of every 1-s record (reduces SD write interference in the EEG). */
    g_config.sd_sync_interval_s = 10;
    /* Boot sound (START_SOUND): default TSBOOT.WAV in SOUNDS/ (fallback BELL2.WAV). */
    strcpy(g_config.start_sound, "TSBOOT.WAV"); /* "Traumschreiber" + bell (0.0.94) */
    g_config.start_sound_volume = 60;   /* analog-sicherer Default; per START_SOUND_VOLUME/App feintunen */
    /* Simulation/replay defaults: OFF (§13.1). */
    g_config.sim_mode = false;
    g_config.selftest2 = false;
    g_config.sim_file[0] = '\0';
    g_config.sim_start_sn = 0;
    g_config.sim_live_speed = 1;
    g_config.sim_drop_period_s = 0;
    g_config.sim_drop_outage_s = 0;
    for (int i = 0; i < 4; i++)
    {
        g_config.sim_gap_lo[i] = 0;
        g_config.sim_gap_hi[i] = 0;
    }
    g_config.sim_gap_count = 0;
    system_config_set_default_button_actions();
}

uint8_t system_config_ads_gain_for_channel(int ch)
{
    if (ch >= 0 && ch < 8 && g_config.ads_gain_ch[ch] != 0)
        return g_config.ads_gain_ch[ch];
    return g_config.ads_gain ? g_config.ads_gain : 4;
}

uint8_t system_config_ads_gain_to_code(uint8_t multiplier)
{
    /* ADS131M08 PGA codes: 0=×1, 1=×2, 2=×4, 3=×8, 4=×16, 5=×32, 6=×64, 7=×128.
     * Round the requested multiplier DOWN to the nearest valid power of two so
     * an out-of-grid value never silently increases gain (which would reduce
     * DC headroom and risk railing). Clamp to [×1, ×128]. */
    if (multiplier >= 128) return 7;
    if (multiplier >= 64)  return 6;
    if (multiplier >= 32)  return 5;
    if (multiplier >= 16)  return 4;
    if (multiplier >= 8)   return 3;
    if (multiplier >= 4)   return 2;
    if (multiplier >= 2)   return 1;
    return 0; /* ×1 */
}

const char *button_action_to_string(button_action_t action)
{
    switch (action)
    {
    case BUTTON_ACTION_NONE:
        return "NONE";
    case BUTTON_ACTION_LOG_ONLY:
        return "LOG_ONLY";
    case BUTTON_ACTION_GREEN_LED_DEMO:
        return "GREEN_LED_DEMO";
    case BUTTON_ACTION_MIC_RECORD_TEST:
        return "MIC_RECORD_TEST";
    case BUTTON_ACTION_DEMO_MODE:
        return "DEMO_MODE";
    case BUTTON_ACTION_LONG_RECORD_10S:
        return "LONG_RECORD_10S";
    case BUTTON_ACTION_BLE_RADIO_TOGGLE:
        return "BLE_RADIO_TOGGLE";
    case BUTTON_ACTION_BLE_RADIO_ON:
        return "BLE_RADIO_ON";
    case BUTTON_ACTION_BLE_RADIO_OFF:
        return "BLE_RADIO_OFF";
    case BUTTON_ACTION_RECORDING_TOGGLE:
        return "RECORDING_TOGGLE";
    case BUTTON_ACTION_ADVERTISING_WORST_CASE:
        return "ADVERTISING_WORST_CASE";
    case BUTTON_ACTION_STREAMING_WORST_CASE:
        return "STREAMING_WORST_CASE";
    case BUTTON_ACTION_SHOW_BATTERY_LED_STATUS:
        return "SHOW_BATTERY_LED_STATUS";
    case BUTTON_ACTION_PLAY_BATTERY_PERCENT_SOUND:
        return "PLAY_BATTERY_PERCENT_SOUND";
    case BUTTON_ACTION_RELOAD_RUNTIME_CONFIG:
        return "RELOAD_RUNTIME_CONFIG";
    case BUTTON_ACTION_BLE_CONFIG_TOGGLE:
        return "BLE_CONFIG_TOGGLE";
    default:
        return "UNKNOWN";
    }
}

bool button_action_from_name(const char *name, button_action_t *out_action)
{
    if (!name || !out_action)
    {
        return false;
    }

    for (button_action_t action = BUTTON_ACTION_NONE; action < BUTTON_ACTION_COUNT; action++)
    {
        if (strcasecmp(name, button_action_to_string(action)) == 0)
        {
            *out_action = action;
            return true;
        }
    }

    return false;
}

static char *trim_whitespace(char *str)
{
    char *end;
    while (isspace((unsigned char)*str))
        str++;
    if (*str == 0)
        return str;
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end))
        end--;
    end[1] = '\0';
    return str;
}

system_config_result_t system_config_write_defaults_to_sd(const char *filename)
{
    if (!filename)
        return SYSTEM_CONFIG_FILE_ERROR;

    sd_card_info_t sd_info;
    if (data_logger_get_sd_info(&sd_info) != DATA_LOGGER_OK || !sd_info.is_mounted)
    {
        LOG_WRN("Cannot write %s: SD card not available", filename);
        return SYSTEM_CONFIG_NO_SD;
    }

    /* Deliberately a MINIMAL file: the handful of settings a user actually
     * changes, each on the value the firmware defaults to anyway, so writing it
     * changes no behaviour. Test/simulation keys (TEST, TEST2, SIM_*) stay out —
     * they are debugging aids and would only invite accidental activation.
     * Values are taken from g_config, which system_config_load_from_sd() has
     * just initialised to the defaults, so file and firmware cannot drift. */
    int n = snprintf(
        s_cfg_iobuf, sizeof(s_cfg_iobuf),
        "# Traumschreiber-Konfiguration\r\n"
        "#\r\n"
        "# Automatisch angelegt, weil auf der SD-Karte keine config.txt lag.\r\n"
        "# Alle Werte entsprechen den Voreinstellungen der Firmware und lassen\r\n"
        "# sich in der sd20-App aendern. Ein Neustart uebernimmt sie.\r\n"
        "#\r\n"
        "# Zeilen mit # sind Kommentare. Format: SCHLUESSEL = WERT\r\n"
        "\r\n"
        "# Funk an (1) oder aus (0)\r\n"
        "BLE_ENABLED = %d\r\n"
        "# Geraetename beim Verbinden; leer = Werksname\r\n"
        "BLE_DEVICE_NAME = %s\r\n"
        "# Abtastrate in Hz\r\n"
        "SAMPLE_RATE = %d\r\n"
        "# Verstaerkung des EEG-Verstaerkers (1,2,4,8,16,32,64,128)\r\n"
        "ADS_GAIN = %u\r\n"
        "# Verstaerkung je Kanal 0..7, ueberschreibt ADS_GAIN (z. B. Kinn-EMG)\r\n"
        "# ADS_GAIN_CH5 = 32\r\n"
        "# Montage je Kanal, steht im BDF-Kopf (Feld Transducer)\r\n"
        "# CH_LABEL5 = EMG Kinn bipolar\r\n"
        "# Rauschmessung mit kurzgeschlossenen Eingaengen nach dem Start, Sekunden (0 = aus)\r\n"
        "NOISE_TEST_SECS = %u\r\n"
        "# SD-Karte alle N Sekunden synchronisieren (1..60, weniger Stoerungen im EEG)\r\n"
        "SD_SYNC_INTERVAL_S = %u\r\n"
        "# Sendeleistung in dBm (+8 = Maximum des nRF52840)\r\n"
        "TX_POWER_DBM = %d\r\n"
        "# Blinkfarbe der LED in Prozent: R,G,B\r\n"
        "LED_FLASH_COLOR = %u,%u,%u\r\n"
        "# Startton: Dateiname auf der Karte (leer = keiner), Lautstaerke in Prozent\r\n"
        "START_SOUND = %s\r\n"
        "START_SOUND_VOLUME = %u\r\n"
        "# Maximale Lautstaerke in Prozent\r\n"
        "FIRMWARE_MAX_VOLUME_PERCENT = %u\r\n"
        "# Debug-Ausgabe ueber RTT (1) und Ausfuehrlichkeit (0-4)\r\n"
        "RTT_ENABLED = %d\r\n"
        "LOG_LEVEL = %d\r\n",
        g_config.ble_enabled ? 1 : 0,
        g_config.ble_device_name,
        g_config.sample_rate,
        (unsigned)g_config.ads_gain,
        (unsigned)g_config.noise_test_secs,
        (unsigned)g_config.sd_sync_interval_s,
        (int)g_config.tx_power_dbm,
        (unsigned)g_config.led_flash_color[0],
        (unsigned)g_config.led_flash_color[1],
        (unsigned)g_config.led_flash_color[2],
        g_config.start_sound,
        (unsigned)g_config.start_sound_volume,
        (unsigned)g_config.firmware_max_volume_percent,
        g_config.rtt_enabled ? 1 : 0,
        g_config.log_level);

    if (n <= 0 || (size_t)n >= sizeof(s_cfg_iobuf))
    {
        LOG_ERR("Default config does not fit the config buffer (%d)", n);
        return SYSTEM_CONFIG_FILE_ERROR;
    }

    if (data_logger_create_file(filename, s_cfg_iobuf, (size_t)n) != DATA_LOGGER_OK)
    {
        LOG_ERR("Failed to write default config %s", filename);
        return SYSTEM_CONFIG_FILE_ERROR;
    }
    return SYSTEM_CONFIG_OK;
}

system_config_result_t system_config_load_from_sd(const char *filename)
{
    if (!filename)
        return SYSTEM_CONFIG_FILE_ERROR;

    /* Ensure defaults are set first */
    system_config_init_defaults();

    /* Ensure data logger/SD is mounted */
    sd_card_info_t sd_info;
    if (data_logger_get_sd_info(&sd_info) != DATA_LOGGER_OK || !sd_info.is_mounted)
    {
        LOG_WRN("SD card not available or not mounted - using defaults");
        return SYSTEM_CONFIG_NO_SD;
    }

    /* Buffer must hold the entire config file. Current default config is ~1.8 KB
     * (gesture/action mapping section sits past the 1 KB mark) so 4 KB gives
     * comfortable headroom for future entries. Shared file-scope scratch. */
    char *buffer = s_cfg_iobuf;
    const size_t buffer_size = sizeof(s_cfg_iobuf);
    size_t bytes_read = 0;
    int ret = data_logger_read_file(filename, buffer, buffer_size - 1, &bytes_read);
    if (ret != DATA_LOGGER_OK || bytes_read == 0)
    {
        /* SD card is mounted but carries no usable config. Write one with the
         * built-in defaults instead of just running on them silently: without a
         * file on the card the app has nothing to show or edit and only reported
         * "no config found", so the device could not be configured at all. After
         * this the user can change every setting from the sd20 app.
         *
         * Only the read result decides — the card is known to be mounted at this
         * point, so a failed read means "absent or empty", not "no SD". We keep
         * running on the defaults either way; the file is a convenience, and a
         * write failure (write-protected or full card) must not stop the boot. */
        LOG_WRN("Config file %s not found or empty - using defaults", filename);
        if (system_config_write_defaults_to_sd(filename) == SYSTEM_CONFIG_OK)
        {
            LOG_INF("Created %s with default settings - editable from the app", filename);
        }
        return SYSTEM_CONFIG_FILE_ERROR;
    }

    buffer[bytes_read] = '\0';
    LOG_INF("Config file %s read: %zu bytes", filename, bytes_read);

    /* Parse line by line */
    char *saveptr = NULL;
    char *line = strtok_r(buffer, "\n", &saveptr);
    while (line)
    {
        char *trimmed = trim_whitespace(line);
        if (trimmed[0] == '\0' || trimmed[0] == '#')
        {
            line = strtok_r(NULL, "\n", &saveptr);
            continue; /* skip empty or comment lines */
        }

        /* Find '=' separator */
        char *eq = strchr(trimmed, '=');
        if (!eq)
        {
            LOG_DBG("Ignoring invalid config line: %s", trimmed);
            line = strtok_r(NULL, "\n", &saveptr);
            continue;
        }

        *eq = '\0';
        char *key = trim_whitespace(trimmed);
        char *val = trim_whitespace(eq + 1);

        LOG_INF("Parse: key='%s', val='%s'", key, val);

        /* Handle known keys */
        if (strcasecmp(key, "BLE_ENABLED") == 0)
        {
            int v = atoi(val);
#ifdef CONFIG_BT
            g_config.ble_enabled = (v != 0);
#else
            LOG_DBG("Build has no Bluetooth support; ignoring BLE_ENABLED");
#endif
        }
        else if (strcasecmp(key, "TEST2") == 0)
        {
            g_config.selftest2 = (atoi(val) == 1);
            LOG_INF("Config TEST2=%d -> HW-Selbsttest %s", atoi(val),
                    g_config.selftest2 ? "AKTIV" : "aus");
        }
        else if (strcasecmp(key, "START_SOUND") == 0)
        {
            /* Boot sound filename (in SOUNDS/) or full path. Default TSBOOT.WAV. */
            strncpy(g_config.start_sound, val, sizeof(g_config.start_sound) - 1);
            g_config.start_sound[sizeof(g_config.start_sound) - 1] = '\0';
            char *p = trim_whitespace(g_config.start_sound);
            memmove(g_config.start_sound, p, strlen(p) + 1);
            if (g_config.start_sound[0] == '\0')
            {
                strcpy(g_config.start_sound, "TSBOOT.WAV");
            }
            LOG_INF("Config START_SOUND='%s'", g_config.start_sound);
        }
        else if (strcasecmp(key, "START_SOUND_VOLUME") == 0)
        {
            int v = atoi(val);
            if (v < 0) v = 0;
            if (v > 400) v = 400;
            g_config.start_sound_volume = (uint16_t)v;
            LOG_INF("Config START_SOUND_VOLUME=%u%%", g_config.start_sound_volume);
        }
        else if (strcasecmp(key, "BLE_DEVICE_NAME") == 0)
        {
            /* Copy device name into config, limit to buffer size-1 */
            LOG_INF("Setting BLE_DEVICE_NAME from config: val='%s' (len=%zu)", val, strlen(val));
            strncpy(g_config.ble_device_name, val, sizeof(g_config.ble_device_name) - 1);
            g_config.ble_device_name[sizeof(g_config.ble_device_name) - 1] = '\0';
            LOG_INF("After strncpy: g_config.ble_device_name='%s'", g_config.ble_device_name);
            /* Trim whitespace */
            char *p = trim_whitespace(g_config.ble_device_name);
            memmove(g_config.ble_device_name, p, strlen(p) + 1);
            LOG_INF("After trim/memmove: g_config.ble_device_name='%s'", g_config.ble_device_name);
        }
        else if (strcasecmp(key, "RTT_ENABLED") == 0)
        {
#ifdef CONFIG_LOG_BACKEND_RTT
            int v = atoi(val);
            g_config.rtt_enabled = (v != 0);
#else
            LOG_DBG("Build has no RTT logging; ignoring RTT_ENABLED");
#endif
        }
        else if (strcasecmp(key, "LOG_LEVEL") == 0)
        {
            int v = atoi(val);
            g_config.log_level = v;
            /* Attempt to set global log level if available */
            /* Note: Zephyr logging allows per-module filter via log_filter_set */
#ifdef CONFIG_LOG
            /* Set global module 'main' level as an example - leave to app to refine */
            /* This requires obtaining backend and calling log_filter_set which is
             * unsafe before logging subsystem fully initialized in some cases. */
#endif
        }
        else if (strcasecmp(key, "SAMPLE_RATE") == 0)
        {
            int v = atoi(val);
            if (v > 0)
                g_config.sample_rate = v;
        }
        else if (strcasecmp(key, "ADS_GAIN") == 0)
        {
            /* PGA gain multiplier for all 8 channels. Accept the literal
             * multiplier (1/2/4/8/16/32/64/128). Out-of-grid values are
             * rounded DOWN to the nearest valid power of two by
             * system_config_ads_gain_to_code() when applied. */
            int v = atoi(val);
            if (v >= 1 && v <= 128)
            {
                uint8_t code = system_config_ads_gain_to_code((uint8_t)v);
                /* store the canonical multiplier that the code maps back to */
                g_config.ads_gain = (uint8_t)(1u << code);
                LOG_INF("Applied ADS_GAIN=%d (effective ×%u, code %u)",
                        v, (unsigned)g_config.ads_gain, (unsigned)code);
            }
            else
            {
                LOG_WRN("ADS_GAIN out of range (1..128): '%s' — keeping ×%u",
                        val, (unsigned)g_config.ads_gain);
            }
        }
        else if (strncasecmp(key, "ADS_GAIN_CH", 11) == 0 && key[11] >= '0' &&
                 key[11] <= '7' && key[12] == '\0')
        {
            /* Per-channel PGA gain override, same grid and rounding as ADS_GAIN. */
            int ch = key[11] - '0';
            int v = atoi(val);
            if (v >= 1 && v <= 128)
            {
                uint8_t code = system_config_ads_gain_to_code((uint8_t)v);
                g_config.ads_gain_ch[ch] = (uint8_t)(1u << code);
                LOG_INF("Applied ADS_GAIN_CH%d=%d (effective ×%u)", ch, v,
                        (unsigned)g_config.ads_gain_ch[ch]);
            }
            else
            {
                LOG_WRN("ADS_GAIN_CH%d out of range (1..128): '%s' — ignored", ch, val);
            }
        }
        else if (strncasecmp(key, "CH_LABEL", 8) == 0 && key[8] >= '0' &&
                 key[8] <= '7' && key[9] == '\0')
        {
            int ch = key[8] - '0';
            strncpy(g_config.ch_label[ch], val, sizeof(g_config.ch_label[ch]) - 1);
            g_config.ch_label[ch][sizeof(g_config.ch_label[ch]) - 1] = '\0';
            LOG_INF("Channel %d montage label: '%s'", ch, g_config.ch_label[ch]);
        }
        else if (strcasecmp(key, "NOISE_TEST_SECS") == 0)
        {
            int v = atoi(val);
            if (v >= 0 && v <= 600)
                g_config.noise_test_secs = (uint16_t)v;
            else
                LOG_WRN("NOISE_TEST_SECS out of range (0..600): '%s'", val);
        }
        else if (strcasecmp(key, "SD_SYNC_INTERVAL_S") == 0)
        {
            int v = atoi(val);
            if (v >= 1 && v <= 60)
                g_config.sd_sync_interval_s = (uint8_t)v;
            else
                LOG_WRN("SD_SYNC_INTERVAL_S out of range (1..60): '%s'", val);
        }
        else if (strcasecmp(key, "LED_FLASH_COLOR") == 0)
        {
            /* Expect value like: 0,100,30  (R,G,B percentages 0-100) */
            int r = -1, g = -1, b = -1;
            if (sscanf(val, "%d,%d,%d", &r, &g, &b) == 3)
            {
                if (r < 0)
                    r = 0;
                if (r > 100)
                    r = 100;
                if (g < 0)
                    g = 0;
                if (g > 100)
                    g = 100;
                if (b < 0)
                    b = 0;
                if (b > 100)
                    b = 100;
                g_config.led_flash_color[0] = (uint8_t)r;
                g_config.led_flash_color[1] = (uint8_t)g;
                g_config.led_flash_color[2] = (uint8_t)b;
                LOG_INF("Applied LED_FLASH_COLOR=%d,%d,%d", r, g, b);
            }
            else
            {
                LOG_DBG("Invalid LED_FLASH_COLOR format: '%s'", val);
            }
        }
        else if (strcasecmp(key, "BLE_COMPRESS_DATA") == 0)
        {
            LOG_INF("Ignoring BLE_COMPRESS_DATA=%s because compression is disabled in this build", val);
        }
        else if (strcasecmp(key, "BLE_DELTA_MAX") == 0)
        {
            LOG_INF("Ignoring BLE_DELTA_MAX=%s because compression is disabled in this build", val);
        }
        else if (strcasecmp(key, "BLE_TRANSFER_CHANNELS") == 0)
        {
            /* Expect comma-separated list of channel indices, e.g. "6,7" or "0,1,2" */
            uint8_t mask = 0;
            uint8_t channels[8];
            int count = 0;

            char tmp[128];
            strncpy(tmp, val, sizeof(tmp) - 1);
            tmp[sizeof(tmp) - 1] = '\0';

            char *save2 = NULL;
            char *tok = strtok_r(tmp, ",", &save2);
            while (tok && count < 8)
            {
                char *t = trim_whitespace(tok);
                int ch = atoi(t);
                if (ch >= 0 && ch < 8)
                {
                    mask |= (1U << ch);
                    channels[count++] = (uint8_t)ch;
                }
                else
                {
                    LOG_DBG("Ignoring invalid BLE_TRANSFER_CHANNELS index: %s", t);
                }
                tok = strtok_r(NULL, ",", &save2);
            }

            if (count > 0)
            {
                g_config.ble_transfer_channel_mask = mask;
                g_config.ble_transfer_channel_count = count;
                for (int i = 0; i < 8; i++)
                    g_config.ble_transfer_channels[i] = (i < count) ? channels[i] : 0;
                LOG_INF("Applied BLE_TRANSFER_CHANNELS mask=0x%02X count=%u", g_config.ble_transfer_channel_mask, g_config.ble_transfer_channel_count);
            }
            else
            {
                LOG_DBG("BLE_TRANSFER_CHANNELS parsing produced no channels, keeping defaults");
            }
        }
        else if (strcasecmp(key, "TX_POWER_DBM") == 0 || strcasecmp(key, "TX_POWER") == 0)
        {
            /* Deliberately ignored since 2026-09-18. Transmit power is fixed at
             * build time via CONFIG_BT_CTLR_TX_PWR_* in prj.conf. A declared
             * maximum EIRP that an end user can raise from a text file on the SD
             * card is not a declared maximum — see prj.conf for the EN 300 328
             * PSD reasoning. Kept as an explicit branch instead of deleted so an
             * existing config.txt reports a reason rather than silently doing
             * nothing. */
            LOG_WRN("TX_POWER/TX_POWER_DBM in config.txt is ignored — transmit power is fixed in firmware");
        }
        else if (strcasecmp(key, "FIRMWARE_MAX_VOLUME_PERCENT") == 0)
        {
            int v = atoi(val);
            if (v < 0)
            {
                v = 0;
            }
            if (v > I2S_VOLUME_MAX_PERCENT)
            {
                v = I2S_VOLUME_MAX_PERCENT;
            }
            g_config.firmware_max_volume_percent = (uint16_t)v;
            LOG_INF("Applied FIRMWARE_MAX_VOLUME_PERCENT=%u%%", (unsigned)v);
        }
        else if (strcasecmp(key, "BUTTON_MULTI_CLICK_TIMEOUT_MS") == 0)
        {
            int v = atoi(val);
            if (v > 50)
            {
                g_config.button_multi_click_timeout_ms = (uint16_t)v;
            }
        }
        else if (strcasecmp(key, "BUTTON_LONG_PRESS_MS") == 0)
        {
            int v = atoi(val);
            if (v > 50)
            {
                g_config.button_long_press_ms = (uint16_t)v;
            }
        }
        else if (strcasecmp(key, "BUTTON_EXTRA_LONG_PRESS_MS") == 0)
        {
            int v = atoi(val);
            if (v > 50)
            {
                g_config.button_extra_long_press_ms = (uint16_t)v;
            }
        }
        else if (strncasecmp(key, "BUTTON_ACTION_", 14) == 0)
        {
            button_event_t event;
            button_action_t action;
            if (!button_event_from_name(key + 14, &event))
            {
                LOG_WRN("Unknown button gesture key: %s", key);
            }
            else if (!button_action_from_name(val, &action))
            {
                LOG_WRN("Unknown button action value: %s", val);
            }
            else
            {
                g_config.button_action_map[event] = action;
                LOG_INF("Applied %s=%s", key, button_action_to_string(action));
            }
        }
        else if (strcasecmp(key, "SIM_MODE") == 0)
        {
            g_config.sim_mode = (atoi(val) != 0);
        }
        else if (strcasecmp(key, "SIM_FILE") == 0)
        {
            strncpy(g_config.sim_file, val, sizeof(g_config.sim_file) - 1);
            g_config.sim_file[sizeof(g_config.sim_file) - 1] = '\0';
        }
        else if (strcasecmp(key, "SIM_START_SN") == 0)
        {
            g_config.sim_start_sn = (uint32_t)strtoul(val, NULL, 0);
        }
        else if (strcasecmp(key, "SIM_LIVE_SPEED") == 0)
        {
            uint32_t v = (uint32_t)strtoul(val, NULL, 0);
            g_config.sim_live_speed = (v == 0) ? 1 : v;
        }
        else if (strcasecmp(key, "SIM_DROP_PATTERN") == 0)
        {
            /* "Os@Ps" — O s outage every P s. */
            unsigned o = 0, p = 0;
            if (sscanf(val, "%us@%us", &o, &p) == 2 && p > 0)
            {
                g_config.sim_drop_outage_s = o;
                g_config.sim_drop_period_s = p;
            }
            else
            {
                LOG_WRN("Invalid SIM_DROP_PATTERN '%s' (want e.g. 300s@1200s)", val);
            }
        }
        else if (strcasecmp(key, "SIM_GAP_SN") == 0)
        {
            /* "lo-hi,lo-hi,..." ranges present only on device (leak test). */
            char tmp[64];
            strncpy(tmp, val, sizeof(tmp) - 1);
            tmp[sizeof(tmp) - 1] = '\0';
            uint8_t cnt = 0;
            char *save3 = NULL;
            char *tok = strtok_r(tmp, ",", &save3);
            while (tok && cnt < 4)
            {
                unsigned long lo = 0, hi = 0;
                if (sscanf(trim_whitespace(tok), "%lu-%lu", &lo, &hi) == 2 && hi >= lo)
                {
                    g_config.sim_gap_lo[cnt] = (uint32_t)lo;
                    g_config.sim_gap_hi[cnt] = (uint32_t)hi;
                    cnt++;
                }
                tok = strtok_r(NULL, ",", &save3);
            }
            g_config.sim_gap_count = cnt;
        }
        else if (strcasecmp(key, "TEST") == 0)
        {
            g_config.test_block_secs = atoi(val);
            LOG_INF("Config TEST mode: %d s/tone (synthetic 1..30 Hz sine "
                    "staircase replaces ADS samples)",
                    g_config.test_block_secs);
        }
        else
        {
            LOG_DBG("Unknown config key: %s", key);
        }

        line = strtok_r(NULL, "\n", &saveptr);
    }

        LOG_INF("Configuration loaded: BLE=%d, RTT=%d, LOG_LEVEL=%d, SAMPLE_RATE=%d, FW_MAX_VOL=%u%%",
            g_config.ble_enabled, g_config.rtt_enabled, g_config.log_level, g_config.sample_rate,
            g_config.firmware_max_volume_percent);
    LOG_INF("Runtime BLE device name: '%s' (len=%zu)", g_config.ble_device_name, strlen(g_config.ble_device_name));

    /* Apply immediate effects: disable RTT backend if requested */
#ifdef CONFIG_LOG_BACKEND_RTT
    if (!g_config.rtt_enabled)
    {
        /* Runtime disabling of the RTT backend via log_backend_get is not portable
         * across Zephyr versions and the symbol may not be exported. Advise rebuild
         * without RTT or implement a backend-specific hook if runtime disable is
         * required. */
        LOG_WRN("RTT backend disable requested at runtime but not supported in this build");
    }
#endif

    /* Apply BLE setting: if disabled, stop BLE advertising or skip init in main */
    /* Main will check g_config.ble_enabled before enabling BLE */

    return SYSTEM_CONFIG_OK;
}

system_config_result_t system_config_toggle_ble_enabled_persisted(
    const char *filename, bool *out_new_value)
{
    if (!filename)
        return SYSTEM_CONFIG_FILE_ERROR;

    sd_card_info_t sd_info;
    if (data_logger_get_sd_info(&sd_info) != DATA_LOGGER_OK || !sd_info.is_mounted)
    {
        LOG_WRN("BLE config toggle: SD card not available");
        return SYSTEM_CONFIG_NO_SD;
    }

    /* Read the current config file into the shared buffer (absent/unreadable
     * → treat as empty). One buffer only; we edit it in place. */
    size_t n = 0;
    int ret = data_logger_read_file(filename, s_cfg_iobuf, sizeof(s_cfg_iobuf) - 1, &n);
    if (ret != DATA_LOGGER_OK)
        n = 0;
    s_cfg_iobuf[n] = '\0';

    /* Locate an existing BLE_ENABLED line and read its value. Default matches
     * the firmware default (BLE enabled) when no such line exists. */
    bool current = true;
    bool found = false;
    char *line_start = NULL, *line_end = NULL; /* [start,end) excludes newline */
    for (char *q = s_cfg_iobuf; *q;)
    {
        char *eol = strchr(q, '\n');
        size_t len = eol ? (size_t)(eol - q) : strlen(q);
        char linecopy[160];
        size_t cl = (len < sizeof(linecopy) - 1) ? len : sizeof(linecopy) - 1;
        memcpy(linecopy, q, cl);
        linecopy[cl] = '\0';
        char *eq = strchr(linecopy, '=');
        if (eq)
        {
            *eq = '\0';
            if (strcasecmp(trim_whitespace(linecopy), "BLE_ENABLED") == 0)
            {
                current = (atoi(trim_whitespace(eq + 1)) != 0);
                found = true;
                line_start = q;
                line_end = q + len;
                break;
            }
        }
        if (!eol)
            break;
        q = eol + 1;
    }
    bool newval = !current;

    char repl[24];
    int rlen = snprintf(repl, sizeof(repl), "BLE_ENABLED=%d", newval ? 1 : 0);

    if (found)
    {
        /* Replace the line content [line_start,line_end) in place, shifting the
         * remainder (incl. the trailing '\0') to absorb any length change. */
        size_t oldlen = (size_t)(line_end - line_start);
        size_t tail = strlen(line_end); /* from newline/'\0' to end, excl. null */
        size_t total = strlen(s_cfg_iobuf);
        if (total + (size_t)rlen - oldlen + 1 > sizeof(s_cfg_iobuf))
            return SYSTEM_CONFIG_FILE_ERROR;
        memmove(line_start + rlen, line_end, tail + 1); /* +1 keeps the null */
        memcpy(line_start, repl, (size_t)rlen);
    }
    else
    {
        /* Append the key, inserting a separating newline if needed. */
        size_t total = strlen(s_cfg_iobuf);
        char add[28];
        int alen = snprintf(add, sizeof(add), "%sBLE_ENABLED=%d\n",
                            (total > 0 && s_cfg_iobuf[total - 1] != '\n') ? "\n" : "",
                            newval ? 1 : 0);
        if (total + (size_t)alen + 1 > sizeof(s_cfg_iobuf))
            return SYSTEM_CONFIG_FILE_ERROR;
        memcpy(s_cfg_iobuf + total, add, (size_t)alen + 1);
    }

    size_t outlen = strlen(s_cfg_iobuf);
    ret = data_logger_create_file(filename, s_cfg_iobuf, outlen);
    if (ret != DATA_LOGGER_OK)
    {
        LOG_ERR("BLE config toggle: failed to write %s (%d)", filename, ret);
        return SYSTEM_CONFIG_FILE_ERROR;
    }

    g_config.ble_enabled = newval; /* keep memory consistent; radio applied at boot */
    if (out_new_value)
        *out_new_value = newval;
    LOG_INF("BLE_ENABLED persisted -> %d (effective after restart)", newval ? 1 : 0);
    return SYSTEM_CONFIG_OK;
}

void system_config_print_summary(void)
{
    LOG_INF("Configuration summary: BLE=%d, RTT=%d, LOG_LEVEL=%d, SAMPLE_RATE=%d",
            g_config.ble_enabled, g_config.rtt_enabled, g_config.log_level, g_config.sample_rate);

    LOG_INF("BLE device name: '%s'", (g_config.ble_device_name[0] != '\0') ? g_config.ble_device_name : "(default)");
    LOG_INF("ADS gain: ×%u (code %u, ±%u mV input range)",
            (unsigned)g_config.ads_gain,
            (unsigned)system_config_ads_gain_to_code(g_config.ads_gain),
            (unsigned)(1200u / (g_config.ads_gain ? g_config.ads_gain : 1)));
    for (int ch = 0; ch < 8; ch++)
    {
        if (g_config.ads_gain_ch[ch] != 0 || g_config.ch_label[ch][0] != '\0')
        {
            LOG_INF("  CH%d: gain ×%u, label '%s'", ch,
                    (unsigned)system_config_ads_gain_for_channel(ch), g_config.ch_label[ch]);
        }
    }
    LOG_INF("Noise test: %u s after ADC start; SD sync every %u s",
            (unsigned)g_config.noise_test_secs, (unsigned)g_config.sd_sync_interval_s);
    LOG_INF("BLE sample format: 24-bit (3 bytes/channel)");
    if (g_config.sim_mode)
    {
        LOG_INF("*** SIM_MODE ACTIVE *** file='%s' start_sn=%u speed=%ux drop=%us@%us gaps=%u",
                g_config.sim_file, g_config.sim_start_sn, g_config.sim_live_speed,
                g_config.sim_drop_outage_s, g_config.sim_drop_period_s,
                (unsigned)g_config.sim_gap_count);
        for (uint8_t i = 0; i < g_config.sim_gap_count; i++)
        {
            LOG_INF("   SIM_GAP_SN[%u] = %u..%u", i, g_config.sim_gap_lo[i], g_config.sim_gap_hi[i]);
        }
    }
    LOG_INF("LED flash color: %u,%u,%u", g_config.led_flash_color[0], g_config.led_flash_color[1], g_config.led_flash_color[2]);
    LOG_INF("Firmware max volume: %u%%", g_config.firmware_max_volume_percent);
        LOG_INF("Button timings: multi=%u ms long=%u ms extra_long=%u ms",
            g_config.button_multi_click_timeout_ms,
            g_config.button_long_press_ms,
            g_config.button_extra_long_press_ms);
    LOG_INF("BLE_TRANSFER_CHANNELS: mask=0x%02X count=%u", g_config.ble_transfer_channel_mask, (unsigned)g_config.ble_transfer_channel_count);
    if (g_config.ble_transfer_channel_count > 0)
    {
        char buf[64];
        size_t off = 0;
        off += snprintf(buf + off, sizeof(buf) - off, "channels:");
        for (int i = 0; i < g_config.ble_transfer_channel_count && off < (int)sizeof(buf); i++)
        {
            off += snprintf(buf + off, sizeof(buf) - off, " %u", g_config.ble_transfer_channels[i]);
        }
        LOG_INF("%s", buf);
    }

    for (button_event_t event = BUTTON_EVENT_SINGLE_CLICK; event < BUTTON_EVENT_COUNT; event++)
    {
        if (event == BUTTON_EVENT_PRESSED || event == BUTTON_EVENT_RELEASED)
        {
            continue;
        }

        LOG_INF("Button action %s => %s",
                button_event_to_string(event),
                button_action_to_string((button_action_t)g_config.button_action_map[event]));
    }
}
