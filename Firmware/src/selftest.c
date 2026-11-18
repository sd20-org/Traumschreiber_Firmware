/*
 * TS Hardware Self-Test — see selftest.h.
 *
 * Runs once at boot (TS_HW_SELFTEST builds only) and writes results to microSD:
 *
 *  Part 1 — Brightness sweep (12 s, TS in complete darkness):
 *      Blue LED stepped 0,1,2,3,5,10,20,25,50,100,200,255 (8-bit), 1 s each;
 *      brightness sensor (AIN0) measured per step -> /SD:/SELFTEST/BRIGHT.TXT
 *
 *  Part 2 — Speaker sweep + microphone recording:
 *      1 kHz sine, 10 loudness steps (quiet -> max, 2 s each = 20 s) while the
 *      microphone records; then 20 s more microphone (silent tail). Everything
 *      goes into ONE file -> /SD:/SELFTEST/MICTEST.WAV (16 kHz mono 16-bit).
 *
 * Flash the SAME signed image onto a TS2607 and a TS2603, pull each SD card,
 * and compare BRIGHT.TXT (R74) and MICTEST.WAV (mic preamp R14/C10 + speaker
 * GAIN_SLOT). Progress is also logged over RTT.
 *
 * Whole file compiles to nothing unless TS_HW_SELFTEST is defined.
 */
#include "selftest.h"

#ifdef TS_HW_SELFTEST

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/fs/fs.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "drivers/nordic_adc.h"
#include "i2s_out.h"
#include "hw_abstraction/rgb_led.h"
#include "subsystems/data_logger.h"
#include "drivers/ads131m08_ti.h"
#include "drivers/sd_card_manager.h"
#include "ble/ble_manager.h"
#include "firmware_version.h"

LOG_MODULE_REGISTER(ts_selftest, LOG_LEVEL_INF);

/* ---- SD output paths (FAT 8.3 names) ---- */
#define ST_DIR        "/SD:/SELFTEST"
#define ST_BRIGHT_TXT ST_DIR "/BRIGHT.TXT"
#define ST_WAV_FINAL  ST_DIR "/MICTEST.WAV"
#define ST_WAV_TMP    ST_DIR "/MICTEST.TMP"
#define ST_MICSTAT_TXT ST_DIR "/MICSTAT.TXT"

/* ---- Part 1: brightness sweep ---- */
static const uint8_t ST_BLUE_STEPS[] = {0, 1, 2, 3, 5, 10, 20, 25, 50, 100, 200, 255};
#define ST_BLUE_N       (sizeof(ST_BLUE_STEPS) / sizeof(ST_BLUE_STEPS[0]))  /* 12 -> 12 s */
#define ST_STEP_MS      1000u
#define ST_CH_BRIGHT    0   /* AIN0 = brightness sensor */

/* ---- Part 2: speaker sweep + mic recording ---- */
#define ST_MIC_RATE       16000u
#define ST_TONE_STEPS     10u
#define ST_VOL_PER_STEP   2u    /* Lautstaerke je Stufe: 2,4,..,20 % (leise, ~20% Skala) */
#define ST_TONE_STEP_MS   2000u
#define ST_TONE_TOTAL_MS  (ST_TONE_STEPS * ST_TONE_STEP_MS)   /* 20 s */
#define ST_TAIL_MS        20000u                              /* 20 s */
#define ST_WAV_TOTAL_MS   (ST_TONE_TOTAL_MS + ST_TAIL_MS)     /* 40 s */
#define ST_TONE_HZ        1000u
/* WAV-Skalierung des DC-bereinigten Mikro-Signals: <<5 (x32) statt <<7 (x128) ->
 * Headroom, kein Clipping der lauten Ton-Stufen (0.0.77 railte bei 100% FS). */
#define ST_MIC_GAIN_SHIFT 5
/* DC-Schaetzung vor der Aufnahme in Stille einschwingen (sonst Start-Transiente). */
#define ST_DC_SETTLE_SAMPLES 512

/* Reuse main.c's already-reserved ~40 KB mic buffer as the WAV write chunk
 * (the normal recording path is idle during the self-test) — avoids adding new
 * .bss to an almost-full RAM (would overflow otherwise). */
extern int16_t g_mic_recording_buffer[];
#define ST_CHUNK_N 512

typedef struct __attribute__((packed)) {
    char riff_id[4]; uint32_t file_size; char wave_id[4];
    char fmt_id[4]; uint32_t fmt_size; uint16_t audio_format; uint16_t num_channels;
    uint32_t sample_rate; uint32_t byte_rate; uint16_t block_align; uint16_t bits_per_sample;
    char data_id[4]; uint32_t data_size;
} st_wav_header_t;

static void st_fill_wav_header(st_wav_header_t *h, uint32_t rate, uint32_t data_size)
{
    memset(h, 0, sizeof(*h));
    memcpy(h->riff_id, "RIFF", 4);
    memcpy(h->wave_id, "WAVE", 4);
    memcpy(h->fmt_id, "fmt ", 4);
    h->fmt_size = 16;
    h->audio_format = 1;
    h->num_channels = 1;
    h->sample_rate = rate;
    h->bits_per_sample = 16;
    h->byte_rate = rate * 1u * (16u / 8u);
    h->block_align = 1u * (16u / 8u);
    memcpy(h->data_id, "data", 4);
    h->data_size = data_size;
    h->file_size = 36u + data_size;
}

/* Ganzzahlige Quadratwurzel (kein libm/Float-printf noetig fuer die RMS-Stats). */
static uint32_t st_isqrt(uint64_t v)
{
    uint64_t x = v, y = (v + 1) / 2;
    while (y < x) { x = y; y = (x + v / x) / 2; }
    return (uint32_t)x;
}

static void st_ensure_dir(void)
{
    /* SELFTEST-Verzeichnis anlegen. Am Boot konkurriert der event_logger/CSV-Writer
     * um den sd_card_manager-Lock -> ein EINMALIGER 1s-Versuch kann ablaufen, dann
     * wird mkdir still uebersprungen und ALLE Datei-Opens scheitern mit -2 (ENOENT,
     * Verzeichnis fehlt). Race: mal frei (0.0.79 ok), mal belegt (0.0.80 leer).
     * Daher mehrfach mit Backoff versuchen und den Ausgang klar loggen. */
    for (int attempt = 0; attempt < 12; attempt++) {
        if (sd_card_manager_lock(2000) == 0) {
            int r = fs_mkdir(ST_DIR);
            sd_card_manager_unlock();
            if (r == 0 || r == -EEXIST) {
                LOG_INF("SELFTEST-Verzeichnis bereit: %s (mkdir=%d, Versuch %d)",
                        ST_DIR, r, attempt + 1);
                return;
            }
            LOG_WRN("mkdir %s -> %d (Versuch %d/12)", ST_DIR, r, attempt + 1);
        } else {
            LOG_WRN("sd_card_manager_lock fuer mkdir belegt (Versuch %d/12)", attempt + 1);
        }
        k_sleep(K_MSEC(300));
    }
    LOG_ERR("SELFTEST-Verzeichnis konnte NICHT angelegt werden -> Datei-Opens werden -2 liefern");
}

/* ============================ Part 1: brightness ============================ */
static void st_brightness_test(void)
{
    LOG_INF("--- PART 1: Helligkeits-Sweep (12 s, TS in KOMPLETTER DUNKELHEIT) ---");

    struct fs_file_t f;
    fs_file_t_init(&f);
    int ret = sd_card_safe_open(&f, ST_BRIGHT_TXT, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
    if (ret < 0) {
        LOG_ERR("Konnte %s nicht oeffnen: %d — Helligkeitswerte nur ueber RTT", ST_BRIGHT_TXT, ret);
    }

    char line[256];   /* muss den mehrzeiligen Header fassen (~150 Zeichen) */
    int n = snprintf(line, sizeof(line),
                     "# Traumschreiber Helligkeits-Sweep (fw %s)\r\n"
                     "# Blaue LED 8-bit gestuft, 1 s/Schritt, Sensor=AIN0. TS in Dunkelheit.\r\n"
                     "step\tblue_8bit\tbright_raw\tbright_mV\r\n",
                     FIRMWARE_VERSION_STR);
    if (ret >= 0 && n > 0) {
        (void)sd_card_safe_write(&f, line, (size_t)n);
    }

    for (size_t i = 0; i < ST_BLUE_N; i++) {
        uint8_t lvl = ST_BLUE_STEPS[i];
        (void)rgb_led_set_blue_raw8(lvl);

        /* Settle, then average several sensor reads near the end of the 1 s window. */
        k_sleep(K_MSEC(600));
        long acc_raw = 0, acc_mv = 0;
        int got = 0;
        for (int k = 0; k < 8; k++) {
            nordic_adc_sample_t s;
            if (nordic_adc_sample_once(&s) == NORDIC_ADC_OK) {
                acc_raw += s.channels[ST_CH_BRIGHT];
                acc_mv  += s.voltage_mv[ST_CH_BRIGHT];
                got++;
            }
            k_sleep(K_MSEC(45));
        }
        int braw = got ? (int)(acc_raw / got) : -1;
        int bmv  = got ? (int)(acc_mv / got) : -1;

        LOG_INF("  [%u/%u] blue=%u -> BRIGHT raw=%d (%d mV)",
                (unsigned)(i + 1), (unsigned)ST_BLUE_N, lvl, braw, bmv);

        if (ret >= 0) {
            n = snprintf(line, sizeof(line), "%u\t%u\t%d\t%d\r\n",
                         (unsigned)(i + 1), lvl, braw, bmv);
            if (n > 0) {
                (void)sd_card_safe_write(&f, line, (size_t)n);
            }
        }
    }

    (void)rgb_led_off();
    if (ret >= 0) {
        (void)sd_card_safe_sync(&f);
        (void)sd_card_safe_close(&f);
        LOG_INF("PART 1 fertig -> %s", ST_BRIGHT_TXT);
    }
}

/* ==================== Part 2: speaker sweep + mic to WAV ==================== */
static void st_tone_mic_test(void)
{
    LOG_INF("--- PART 2: Speaker-Sweep (10 Stufen leise->max) + Mikro-Aufnahme, dann 20 s Nachlauf ---");

    if (!nordic_adc_is_ready()) {
        LOG_ERR("ADC not ready — PART 2 uebersprungen");
        return;
    }

    const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));

    struct fs_file_t f;
    fs_file_t_init(&f);
    int ret = sd_card_safe_open(&f, ST_WAV_TMP, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
    if (ret < 0) {
        LOG_ERR("Konnte %s nicht oeffnen: %d — PART 2 abgebrochen", ST_WAV_TMP, ret);
        return;
    }

    st_wav_header_t header;
    st_fill_wav_header(&header, ST_MIC_RATE, 0);
    if (sd_card_safe_write(&f, &header, sizeof(header)) != (ssize_t)sizeof(header)) {
        LOG_ERR("WAV-Header schreiben fehlgeschlagen");
        (void)sd_card_safe_close(&f);
        return;
    }
    (void)sd_card_safe_sync(&f);

    /* Per-sample ADC capture: periodisches SAADC-Sampling stoppen. */
    nordic_adc_stop_periodic_sampling();

    LOG_INF("WAV: mono, %u s (%u s Ton-Sweep + %u s Nachlauf); Samplerate wird gemessen",
            ST_WAV_TOTAL_MS / 1000u, ST_TONE_TOTAL_MS / 1000u, ST_TAIL_MS / 1000u);

    size_t chunk_idx = 0, samples_written = 0;
    int32_t dc = 0;
    bool io_error = false;
    uint32_t cur_step = 1;   /* 1..10 */
    bool tone_off = false;

    /* Rohsignal-Statistik (gain-unabhaengig), je Ton-Stufe (0..9) + Nachlauf (10).
     * Analog zu BRIGHT.TXT: macht den Mikro-Vergleich TS2607 vs TS2603 quantitativ. */
    int32_t  raw_peak = 0;
    int64_t  step_sumsq[ST_TONE_STEPS + 1] = {0};
    uint32_t step_cnt[ST_TONE_STEPS + 1]   = {0};

    int16_t raw = 0;
    struct adc_sequence mic_seq = {
        .channels = BIT(3), .buffer = &raw, .buffer_size = sizeof(raw),
        .resolution = NORDIC_ADC_RESOLUTION, .oversampling = 3, .calibrate = false,
    };

    /* DC-Schaetzung in Stille einschwingen (VOR dem Ton) -> keine Start-Transiente
     * (0.0.77 railte die ersten ~20 ms, weil dc bei 0 startete). */
    if (adc_read(adc_dev, &mic_seq) == 0) {
        dc = raw;
    }
    for (int p = 0; p < ST_DC_SETTLE_SAMPLES; p++) {
        if (adc_read(adc_dev, &mic_seq) < 0) break;
        dc += ((int32_t)raw - dc) / 128;
    }

    /* EIN Dauerton (1 kHz), Lautstaerke in 10 Stufen (set_volume, kein play_tone-
     * Neustart). Fire-and-forget ueber i2s-Thread + DMA. Die Mic-Schleife nutzt KEIN
     * busy-wait -> adc_read yieldet pro Sample, der i2s-Thread bekommt CPU (kein
     * Underrun/ERROR). Timing per Wall-Clock; echte Samplerate am Ende gemessen. */
    (void)i2s_out_set_volume(ST_VOL_PER_STEP);
    tone_config_t tone = {
        .type = TONE_SINE, .frequency_hz = ST_TONE_HZ,
        .amplitude_percent = 100, .duration_ms = 0, /* infinit bis i2s_out_stop */
    };
    (void)i2s_out_play_tone(&tone);
    LOG_INF("  Ton-Stufe 1/%u (vol %u%%)", ST_TONE_STEPS, ST_VOL_PER_STEP);

    int64_t t0 = k_uptime_get();

    for (;;) {
        int64_t now = k_uptime_get() - t0;
        if (now >= (int64_t)ST_WAV_TOTAL_MS) {
            break;
        }

        /* --- Speaker: Lautstaerke-Stufen (Wall-Clock), danach Nachlauf still --- */
        if (now < (int64_t)ST_TONE_TOTAL_MS) {
            uint32_t step = (uint32_t)(now / (int64_t)ST_TONE_STEP_MS) + 1u; /* 1..10 */
            if (step > cur_step) {
                cur_step = step;
                uint16_t vol = (uint16_t)(step * ST_VOL_PER_STEP); /* leise->max, 20% Skala */
                (void)i2s_out_set_volume(vol);
                LOG_INF("  Ton-Stufe %u/%u (vol %u%%)", step, ST_TONE_STEPS, vol);
            }
        } else if (!tone_off) {
            (void)i2s_out_stop();
            tone_off = true;
            LOG_INF("  Ton aus -> %u s Mikro-Nachlauf (Stille)", ST_TAIL_MS / 1000u);
        }

        /* --- Mikrofon (AIN3), DC-bereinigt. adc_read yieldet -> i2s-Thread laeuft. --- */
        if (adc_read(adc_dev, &mic_seq) < 0) {
            LOG_ERR("ADC read failed nach %zu Samples", samples_written + chunk_idx);
            io_error = true;
            break;
        }
        dc += ((int32_t)raw - dc) / 128;
        int32_t ac = (int32_t)raw - dc;   /* AC-Anteil in Rohcounts (gain-unabhaengig) */

        /* Rohstatistik je Bucket (0..9 = Ton-Stufen, 10 = Nachlauf). */
        int32_t a = ac < 0 ? -ac : ac;
        if (a > raw_peak) raw_peak = a;
        uint32_t b = tone_off ? ST_TONE_STEPS : (cur_step - 1u);
        if (b <= ST_TONE_STEPS) { step_sumsq[b] += (int64_t)ac * ac; step_cnt[b]++; }

        int32_t pcm = ac << ST_MIC_GAIN_SHIFT;
        if (pcm > 32767) pcm = 32767;
        if (pcm < -32768) pcm = -32768;
        g_mic_recording_buffer[chunk_idx++] = (int16_t)pcm;

        if (chunk_idx >= ST_CHUNK_N) {
            size_t bytes = chunk_idx * sizeof(int16_t);
            if (sd_card_safe_write(&f, g_mic_recording_buffer, bytes) != (ssize_t)bytes) {
                LOG_ERR("WAV-Chunk schreiben fehlgeschlagen");
                io_error = true;
                break;
            }
            samples_written += chunk_idx;
            chunk_idx = 0;
        }
    }

    int64_t elapsed_ms = k_uptime_get() - t0;
    (void)i2s_out_stop();

    if (chunk_idx > 0 && !io_error) {
        size_t bytes = chunk_idx * sizeof(int16_t);
        if (sd_card_safe_write(&f, g_mic_recording_buffer, bytes) == (ssize_t)bytes) {
            samples_written += chunk_idx;
        }
    }

    /* Gemessene Samplerate (Samples / echte Dauer) -> WAV-Header. */
    uint32_t meas_rate = (elapsed_ms > 0)
        ? (uint32_t)(((uint64_t)samples_written * 1000ULL) / (uint64_t)elapsed_ms)
        : ST_MIC_RATE;
    if (meas_rate < 1000u) meas_rate = ST_MIC_RATE; /* Sicherheitsnetz */
    LOG_INF("  Mikro: %zu Samples in %lld ms -> gemessene Rate %u Hz",
            samples_written, (long long)elapsed_ms, meas_rate);

    st_fill_wav_header(&header, meas_rate, (uint32_t)(samples_written * sizeof(int16_t)));
    if (fs_seek(&f, 0, FS_SEEK_SET) == 0) {
        (void)sd_card_safe_write(&f, &header, sizeof(header));
        (void)sd_card_safe_sync(&f);
    }
    (void)sd_card_safe_close(&f);

    /* Restart periodic SAADC sampling. */
    (void)nordic_adc_start_periodic_sampling();

    if (samples_written == 0 || io_error) {
        LOG_ERR("PART 2 unvollstaendig (samples=%zu, io_error=%d)", samples_written, io_error ? 1 : 0);
        return;
    }

    /* Publish: rename tmp -> final. */
    int rr = -1;
    if (sd_card_manager_lock(2000) == 0) {
        struct fs_dirent st;
        if (fs_stat(ST_WAV_FINAL, &st) == 0) {
            (void)fs_unlink(ST_WAV_FINAL);
        }
        rr = fs_rename(ST_WAV_TMP, ST_WAV_FINAL);
        sd_card_manager_unlock();
    }
    LOG_INF("PART 2 fertig -> %s (%zu Samples, rename=%d)", ST_WAV_FINAL, samples_written, rr);

    /* --- MICSTAT.TXT: rohe Mikro-Antwort je Ton-Stufe (gain-unabhaengig) ---
     * Analog zu BRIGHT.TXT der sauberste Vergleichswert TS2607 vs TS2603:
     * rms_raw steigt mit der Ton-Lautstaerke; peak_raw zeigt Uebersteuerung. */
    {
        struct fs_file_t sf;
        fs_file_t_init(&sf);
        if (sd_card_safe_open(&sf, ST_MICSTAT_TXT, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC) == 0) {
            char l[256];
            int n = snprintf(l, sizeof(l),
                "# Traumschreiber Mikro-Test (fw %s)\r\n"
                "# 1 kHz Ton, 10 Stufen (vol 2..20%%), dann %u s Nachlauf (Stille).\r\n"
                "# rms_raw/peak_raw = DC-bereinigte ADC-Counts (unabh. vom WAV-Gain).\r\n"
                "# samplerate=%u Hz, samples=%u, peak_raw=%d\r\n"
                "step\tvol_pct\trms_raw\tn\r\n",
                FIRMWARE_VERSION_STR, ST_TAIL_MS / 1000u, meas_rate,
                (unsigned)samples_written, raw_peak);
            if (n > 0) (void)sd_card_safe_write(&sf, l, (size_t)(n < (int)sizeof(l) ? n : (int)sizeof(l) - 1));
            for (uint32_t s = 0; s <= ST_TONE_STEPS; s++) {
                uint32_t rms = step_cnt[s] ? st_isqrt((uint64_t)(step_sumsq[s] / step_cnt[s])) : 0u;
                if (s < ST_TONE_STEPS) {
                    n = snprintf(l, sizeof(l), "%u\t%u\t%u\t%u\r\n",
                                 s + 1u, (s + 1u) * ST_VOL_PER_STEP, rms, step_cnt[s]);
                } else {
                    n = snprintf(l, sizeof(l), "tail\t0\t%u\t%u\r\n", rms, step_cnt[s]);
                }
                if (n > 0) (void)sd_card_safe_write(&sf, l, (size_t)(n < (int)sizeof(l) ? n : (int)sizeof(l) - 1));
                if (s < ST_TONE_STEPS) {
                    LOG_INF("  MICSTAT Stufe %u (vol %u%%): rms_raw=%u (n=%u)",
                            s + 1u, (s + 1u) * ST_VOL_PER_STEP, rms, step_cnt[s]);
                } else {
                    LOG_INF("  MICSTAT tail (Stille): rms_raw=%u (n=%u)", rms, step_cnt[s]);
                }
            }
            (void)sd_card_safe_sync(&sf);
            (void)sd_card_safe_close(&sf);
            LOG_INF("  MICSTAT -> %s (peak_raw=%d)", ST_MICSTAT_TXT, raw_peak);
        } else {
            LOG_WRN("Konnte %s nicht schreiben", ST_MICSTAT_TXT);
        }
    }
}

/* ================================ entry ================================ */
void ts_selftest_run(void)
{
    LOG_INF("############################################################");
    LOG_INF("### TS HARDWARE SELF-TEST (fw %s)", FIRMWARE_VERSION_STR);
    LOG_INF("### Ergebnisse auf microSD: %s + %s", ST_BRIGHT_TXT, ST_WAV_FINAL);
    LOG_INF("### Vergleich TS2607 vs TS2603: Helligkeit (R74), Mikro (R14/C10), Speaker (GAIN_SLOT)");
    LOG_INF("############################################################");

    /* Ruhe fuer saubere Messung + freie SD: ADS131/BLE aus. */
    data_logger_pause_ads131_logging(true);
    (void)ads131m08_ti_stop_sampling();
    /* BLE wird im Testmodus in main() gar nicht erst hochgefahren
     * (kein Advertising / kein blaues Blinken) -> hier nichts zu stoppen. */
    k_sleep(K_MSEC(300));

    st_ensure_dir();
    st_brightness_test();
    st_tone_mic_test();

    /* Fertig: dauerhaft gruen + Log; SD entnehmen und auswerten. */
    LOG_INF("############################################################");
    LOG_INF("### SELF-TEST FERTIG — SD entnehmen: %s + %s", ST_BRIGHT_TXT, ST_WAV_FINAL);
    LOG_INF("############################################################");
    while (1) {
        (void)rgb_led_set_color(&RGB_COLOR_GREEN);
        LOG_INF("Self-test done. Ergebnisse liegen auf der microSD.");
        k_sleep(K_MSEC(5000));
    }
}

#endif /* TS_HW_SELFTEST */
