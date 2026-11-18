/*
 * noise_test — ADC noise measurement with shorted inputs. See noise_test.h.
 */
#include "noise_test.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "data_logger.h"
#include "system_config.h"
#include "../drivers/ads131m08_ti.h"

LOG_MODULE_REGISTER(noise_test, LOG_LEVEL_INF);

#define NT_FS           250U  /* sample rate (fixed by the 2.048 MHz clock, OSR 4096) */
#define NT_SETTLE       250U  /* samples skipped after the mux switch (sinc3 + biquad) */
#define NT_CH           8

/* 2nd-order band-pass (RBJ, 0 dB peak) around 30 Hz with Q 1.2 ≈ 20–45 Hz.
 * Equivalent noise bandwidth ≈ π/2 · 25 Hz ≈ 39 Hz. Coefficients set in start. */
static float s_b0, s_b2, s_a1, s_a2;
static float s_z1[NT_CH], s_z2[NT_CH];

/* Welford accumulators (broadband, mean removed) and band power. */
static float s_mean[NT_CH], s_m2[NT_CH], s_bandsq[NT_CH];
static uint32_t s_n;

static volatile noise_test_state_t s_state = NOISE_TEST_IDLE;
static uint32_t s_target;     /* samples to measure */
static uint32_t s_settle;     /* samples still to skip after the mux is applied */
static uint32_t s_sn_first, s_sn_last;
static bool s_switched_back;
static uint8_t s_seq;
static char s_trigger[8];
static float s_rms_uv[NT_CH], s_band_uv[NT_CH];

static struct k_work s_write_work;

static float uv_per_count(int ch)
{
    /* ±1.2 V / gain over the 24-bit two's-complement range. */
    float gain = (float)system_config_ads_gain_for_channel(ch);
    return 1.2e6f / gain / 8388608.0f;
}

static void write_result_work(struct k_work *work)
{
    ARG_UNUSED(work);
    /* On the system work-queue stack (3 KB) rather than static: RAM is tight. */
    char s_line[400];
    char fname[32] = "?";
    (void)data_logger_get_ads131_filename(fname, sizeof(fname));
    const char *base = strrchr(fname, '/');
    base = base ? base + 1 : fname;

    char when[32] = "unbekannt";
    data_logger_wall_time_t wt;
    if (data_logger_wall_time_now(&wt))
    {
        snprintf(when, sizeof(when), "%04u-%02u-%02u %02u:%02u:%02u", wt.year, wt.month,
                 wt.day, wt.hour, wt.minute, wt.second);
    }

    int n = snprintf(s_line, sizeof(s_line), "%s; %s; sn %u-%u; %s; gain", when, base,
                     (unsigned)s_sn_first, (unsigned)s_sn_last, s_trigger);
    for (int ch = 0; ch < NT_CH && n < (int)sizeof(s_line); ch++)
        n += snprintf(s_line + n, sizeof(s_line) - n, " %u",
                      (unsigned)system_config_ads_gain_for_channel(ch));
    n += snprintf(s_line + n, sizeof(s_line) - n, "; rms_uV");
    for (int ch = 0; ch < NT_CH && n < (int)sizeof(s_line); ch++)
        n += snprintf(s_line + n, sizeof(s_line) - n, " %.3f", (double)s_rms_uv[ch]);
    n += snprintf(s_line + n, sizeof(s_line) - n, "; band20_45_uV");
    for (int ch = 0; ch < NT_CH && n < (int)sizeof(s_line); ch++)
        n += snprintf(s_line + n, sizeof(s_line) - n, " %.3f", (double)s_band_uv[ch]);
    n += snprintf(s_line + n, sizeof(s_line) - n, "\r\n");
    if (n >= (int)sizeof(s_line))
        n = sizeof(s_line) - 1;

    /* First measurement on a card: write a header line explaining the columns. */
    size_t sz = 0;
    if (data_logger_get_file_size(NOISE_TEST_FILE, &sz) != DATA_LOGGER_OK || sz == 0)
    {
        static const char hdr[] =
            "# Rauschmessung mit kurzgeschlossenen Eingaengen (MUX=AGND), je Kanal 0..7.\r\n"
            "# Zeit; BDF-Datei; Samples; Ausloeser; Verstaerkung; RMS breitbandig (uV);"
            " RMS 20-45 Hz, Biquad Q 1,2 um 30 Hz, Rauschbandbreite ~39 Hz (uV)\r\n";
        (void)data_logger_append_file(NOISE_TEST_FILE, hdr, sizeof(hdr) - 1);
    }
    data_logger_result_t r = data_logger_append_file(NOISE_TEST_FILE, s_line, (size_t)n);
    if (r != DATA_LOGGER_OK)
    {
        LOG_WRN("Noise test: writing %s failed (%d)", NOISE_TEST_FILE, r);
    }
    LOG_INF("Noise test done: %s", s_line);
}

int noise_test_start(uint16_t seconds, const char *trigger)
{
    if (seconds == 0 || seconds > 600)
        return -EINVAL;
    if (s_state == NOISE_TEST_RUNNING)
        return -EBUSY;

    static bool work_init;
    if (!work_init)
    {
        k_work_init(&s_write_work, write_result_work);
        work_init = true;
    }

    const float w0 = 2.0f * 3.14159265f * 30.0f / (float)NT_FS;
    const float alpha = sinf(w0) / (2.0f * 1.2f);
    const float a0 = 1.0f + alpha;
    s_b0 = alpha / a0;
    s_b2 = -alpha / a0;
    s_a1 = -2.0f * cosf(w0) / a0;
    s_a2 = (1.0f - alpha) / a0;

    memset(s_z1, 0, sizeof(s_z1));
    memset(s_z2, 0, sizeof(s_z2));
    memset(s_mean, 0, sizeof(s_mean));
    memset(s_m2, 0, sizeof(s_m2));
    memset(s_bandsq, 0, sizeof(s_bandsq));
    s_n = 0;
    s_target = (uint32_t)seconds * NT_FS;
    s_settle = NT_SETTLE;
    s_switched_back = false;
    strncpy(s_trigger, trigger ? trigger : "?", sizeof(s_trigger) - 1);
    s_trigger[sizeof(s_trigger) - 1] = '\0';

    if (ads131m08_ti_request_input_mux(ADS131M08_MUX_AGND) != ADS131M08_OK)
        return -EIO;
    s_state = NOISE_TEST_RUNNING;
    LOG_INF("Noise test started: %u s, trigger %s", (unsigned)seconds, s_trigger);
    return 0;
}

void noise_test_on_sample(const int32_t channels[8], uint32_t sample_number)
{
    if (s_state != NOISE_TEST_RUNNING)
        return;

    /* Mark every record (250 samples) of the shorted period in the BDF. */
    if ((sample_number % NT_FS) == 0)
        data_logger_record_button_event(NOISE_TEST_BDF_MARKER);

    if (s_switched_back)
    {
        /* Wait until the electrodes are connected again, then finish. */
        if (ads131m08_ti_get_input_mux() == ADS131M08_MUX_NORMAL)
        {
            s_state = NOISE_TEST_DONE;
            s_seq++;
            k_work_submit(&s_write_work);
        }
        return;
    }

    if (ads131m08_ti_get_input_mux() != ADS131M08_MUX_AGND)
        return; /* still switching */

    /* Run the band-pass from the first shorted sample so it is settled. */
    float y[NT_CH];
    for (int ch = 0; ch < NT_CH; ch++)
    {
        float x = (float)channels[ch];
        float out = s_b0 * x + s_z1[ch];
        s_z1[ch] = -s_a1 * out + s_z2[ch];
        s_z2[ch] = s_b2 * x - s_a2 * out;
        y[ch] = out;
    }
    if (s_settle > 0)
    {
        s_settle--;
        return;
    }
    if (s_n == 0)
        s_sn_first = sample_number;

    s_n++;
    for (int ch = 0; ch < NT_CH; ch++)
    {
        float x = (float)channels[ch];
        float d = x - s_mean[ch];
        s_mean[ch] += d / (float)s_n;
        s_m2[ch] += d * (x - s_mean[ch]);
        s_bandsq[ch] += y[ch] * y[ch];
    }

    if (s_n >= s_target)
    {
        s_sn_last = sample_number;
        for (int ch = 0; ch < NT_CH; ch++)
        {
            float k = uv_per_count(ch);
            s_rms_uv[ch] = sqrtf(s_m2[ch] / (float)(s_n > 1 ? s_n - 1 : 1)) * k;
            s_band_uv[ch] = sqrtf(s_bandsq[ch] / (float)s_n) * k;
        }
        if (ads131m08_ti_request_input_mux(ADS131M08_MUX_NORMAL) != ADS131M08_OK)
        {
            s_state = NOISE_TEST_FAILED;
            return;
        }
        s_switched_back = true;
    }
}

noise_test_state_t noise_test_get_state(void)
{
    return s_state;
}

uint8_t noise_test_get_seq(void)
{
    return s_seq;
}

void noise_test_get_last(float rms_uv[8], float band_uv[8])
{
    for (int ch = 0; ch < NT_CH; ch++)
    {
        if (rms_uv)
            rms_uv[ch] = s_rms_uv[ch];
        if (band_uv)
            band_uv[ch] = s_band_uv[ch];
    }
}
