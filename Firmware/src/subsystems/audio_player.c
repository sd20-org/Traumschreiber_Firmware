/**
 * @file audio_player.c
 * @brief Audio Player Subsystem Implementation
 *
 * Implements WAV file parsing and streaming playback functionality.
 *
 * @author ISDT
 * @date 2025
 */

#include "audio_player.h"
#include "data_logger.h"
#include "i2s_out.h"
#include "sd_card_manager.h"
#include "ble/lbs_service.h"
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <stdbool.h>

/* For directory iteration */
#include <zephyr/fs/fs.h>

/* Note: Do not include <zephyr/sys/mem_alloc.h>; not available in this SDK version */

LOG_MODULE_REGISTER(audio_player, LOG_LEVEL_DBG);

/* Playback queue: queue filenames (max 4) for sequential playback */
#define AUDIO_PLAYER_QUEUE_DEPTH 4
#define AUDIO_PLAYER_FILENAME_MAX 64
K_MSGQ_DEFINE(audio_player_msgq, AUDIO_PLAYER_FILENAME_MAX, AUDIO_PLAYER_QUEUE_DEPTH, 4);
static struct k_thread playback_queue_thread_data;
static K_THREAD_STACK_DEFINE(playback_queue_stack, 2048);
static k_tid_t playback_queue_tid = NULL;
static struct k_mutex g_status_mutex;

/* WAV file constants */
#define WAV_RIFF_MAGIC 0x46464952 /* "RIFF" */
#define WAV_WAVE_MAGIC 0x45564157 /* "WAVE" */
#define WAV_FMT_MAGIC 0x20746D66  /* "fmt " */
#define WAV_DATA_MAGIC 0x61746164 /* "data" */
#define WAV_PCM_FORMAT 1
#define WAV_MULAW_FORMAT 7

/* Audio buffer size for streaming (must match I2S buffer size) */
#define AUDIO_BUFFER_SIZE_BYTES I2S_BUFFER_SIZE_BYTES
#define AUDIO_BUFFER_SIZE_SAMPLES (AUDIO_BUFFER_SIZE_BYTES / 4) /* 16-bit stereo */

/* Upper bound on the always-on per-file loudness makeup gain. Caps how much a
 * quiet clip is boosted so a near-silent file's noise floor isn't amplified to
 * absurd levels. 1600 % = 16x ≈ +24 dB. */
#define AUDIO_MAX_MAKEUP_PERCENT 1600

/* WAV header structures */
typedef struct __packed
{
    uint32_t riff_magic;
    uint32_t file_size;
    uint32_t wave_magic;
} wav_riff_header_t;

typedef struct __packed
{
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
} wav_fmt_chunk_t;

typedef struct __packed
{
    uint32_t data_magic;
    uint32_t data_size;
} wav_data_header_t;

/* Static variables */
static audio_player_status_t g_status = {0};
static int16_t g_audio_buffer[AUDIO_BUFFER_SIZE_SAMPLES * 2]; /* Stereo buffer */

typedef struct
{
    bool active;
    uint16_t output_gain_percent;
    uint16_t makeup_gain_percent;
    uint8_t compressor_ratio_divisor;
    int16_t peak_limit;
    int16_t compressor_threshold;
    int16_t input_peak;
    int16_t estimated_peak;
} wav_loudness_plan_t;

/* Forward declarations */
static audio_player_result_t convert_audio_chunk(const uint8_t *input, size_t input_bytes,
                                                 int16_t *output, size_t *output_samples,
                                                 const wav_info_t *wav_info);
static void convert_sample_to_16bit(const uint8_t *input, int16_t *output,
                                    uint16_t bits_per_sample, uint16_t audio_format, bool is_signed);
static int16_t decode_mulaw_sample(uint8_t sample);
static audio_player_result_t find_wav_chunk(struct fs_file_t *file, uint32_t chunk_magic,
                                            uint32_t *chunk_size, uint32_t *chunk_offset);
static uint16_t get_current_output_gain_percent(void);
static int16_t abs_sample_16(int16_t sample);
static int16_t scale_and_clip_pcm_sample(int16_t sample, uint16_t gain_percent);
static int16_t compress_pcm_sample(int16_t sample, int16_t threshold, uint8_t ratio_divisor);
static size_t get_max_input_chunk_size(size_t requested_bytes, size_t input_frame_size, size_t read_buffer_size);
static audio_player_result_t build_wav_loudness_plan(struct fs_file_t *file,
                                                     const wav_info_t *wav_info,
                                                     uint16_t output_gain_percent,
                                                     wav_loudness_plan_t *plan);
static void apply_wav_loudness_plan_in_place(int16_t *samples,
                                             size_t sample_count,
                                             const wav_loudness_plan_t *plan);
static void analyze_pcm_chunk_for_current_gain(const int16_t *samples,
                                               size_t sample_count,
                                               uint16_t gain_percent,
                                               int16_t *raw_peak_out,
                                               int16_t *projected_peak_out,
                                               uint32_t *projected_clipped_samples_out);

/* Playback queue thread forward declaration */
static void playback_queue_thread_fn(void *p1, void *p2, void *p3);

/* Context type for WAV listing forwarder */
typedef struct
{
    void (*user_cb)(const char *filename, void *ctx);
    void *user_ctx;
} wav_list_ctx_t;

/* Forwarder callback used by data_logger_list_files to filter .wav files */
static void audio_player_data_logger_cb(const char *name, void *ctx)
{
    wav_list_ctx_t *wctx = (wav_list_ctx_t *)ctx;
    if (!name || !wctx || !wctx->user_cb)
        return;

    size_t len = strlen(name);
    if (len >= 4)
    {
        const char *ext = &name[len - 4];
        if (strncasecmp(ext, ".wav", 4) == 0)
        {
            wctx->user_cb(name, wctx->user_ctx);
        }
    }
}

static uint16_t get_current_output_gain_percent(void)
{
    i2s_status_t i2s_status;

    if (i2s_out_get_status(&i2s_status) == 0)
    {
        return i2s_status.volume;
    }

    return I2S_VOLUME_DEFAULT_PERCENT;
}

static int16_t abs_sample_16(int16_t sample)
{
    if (sample == INT16_MIN)
    {
        return INT16_MAX;
    }

    return (sample < 0) ? (int16_t)(-sample) : sample;
}

static int16_t decode_mulaw_sample(uint8_t sample)
{
    uint8_t decoded = (uint8_t)(~sample);
    int32_t magnitude = ((decoded & 0x0F) << 3) + 0x84;

    magnitude <<= ((decoded & 0x70) >> 4);

    if (decoded & 0x80)
    {
        return (int16_t)(0x84 - magnitude);
    }

    return (int16_t)(magnitude - 0x84);
}

static int16_t scale_and_clip_pcm_sample(int16_t sample, uint16_t gain_percent)
{
    int32_t scaled = ((int32_t)sample * (int32_t)gain_percent) / 100;

    if (scaled > INT16_MAX)
    {
        scaled = INT16_MAX;
    }
    else if (scaled < INT16_MIN)
    {
        scaled = INT16_MIN;
    }

    return (int16_t)scaled;
}

static int16_t compress_pcm_sample(int16_t sample, int16_t threshold, uint8_t ratio_divisor)
{
    int32_t abs_value;
    int32_t compressed_abs;

    if (ratio_divisor == 0)
    {
        return sample;
    }

    abs_value = abs_sample_16(sample);
    if (abs_value <= threshold)
    {
        return sample;
    }

    compressed_abs = threshold + ((abs_value - threshold) / ratio_divisor);
    if (compressed_abs > INT16_MAX)
    {
        compressed_abs = INT16_MAX;
    }

    if (sample < 0)
    {
        compressed_abs = -compressed_abs;
    }

    return (int16_t)compressed_abs;
}

static size_t get_max_input_chunk_size(size_t requested_bytes, size_t input_frame_size, size_t read_buffer_size)
{
    size_t chunk_size;
    size_t max_convertible_bytes;

    if (input_frame_size == 0 || read_buffer_size < input_frame_size)
    {
        return 0;
    }

    max_convertible_bytes = AUDIO_BUFFER_SIZE_SAMPLES * input_frame_size;
    if (max_convertible_bytes > read_buffer_size)
    {
        max_convertible_bytes = read_buffer_size;
    }

    chunk_size = (requested_bytes > max_convertible_bytes) ? max_convertible_bytes : requested_bytes;
    chunk_size -= (chunk_size % input_frame_size);

    return chunk_size;
}

static audio_player_result_t build_wav_loudness_plan(struct fs_file_t *file,
                                                     const wav_info_t *wav_info,
                                                     uint16_t output_gain_percent,
                                                     wav_loudness_plan_t *plan)
{
    static uint8_t analysis_read_buffer[512];
    size_t bytes_remaining;
    size_t chunk_size;
    size_t input_sample_size;
    size_t input_frame_size;
    int ret;

    if (!file || !wav_info || !plan)
    {
        return AUDIO_PLAYER_ERROR;
    }

    memset(plan, 0, sizeof(*plan));
    plan->output_gain_percent = output_gain_percent;

    /* Loudness normalization is ALWAYS on (it used to run only when the global
     * volume was boosted above 100 %). At 100 % volume this normalizes every
     * clip toward full scale, so quiet files are no longer drastically quieter
     * than loud ones. */
    input_sample_size = wav_info->bits_per_sample / 8;
    input_frame_size = input_sample_size * wav_info->channels;
    if (input_frame_size == 0)
    {
        return AUDIO_PLAYER_INVALID_FORMAT;
    }

    /* Target peak = full scale, but leave headroom when the global volume is
     * boosted so the later ×gain stage doesn't clip. Clamp the effective gain
     * to >= 100 % so peak_limit never exceeds full scale (which would overflow
     * the int16 cast) and never divides by zero. */
    {
        uint16_t eff_gain = (output_gain_percent < I2S_VOLUME_DEFAULT_PERCENT)
                                ? I2S_VOLUME_DEFAULT_PERCENT
                                : output_gain_percent;
        /* Aim the makeup at the supply-safe output ceiling instead of full
         * scale, so the i2s clamp never has to cut (0.0.94). */
        int32_t pl = ((int32_t)I2S_OUTPUT_PEAK_LIMIT * I2S_VOLUME_DEFAULT_PERCENT) / eff_gain;
        if (pl > INT16_MAX)
        {
            pl = INT16_MAX;
        }
        plan->peak_limit = (int16_t)pl;
    }
    if (plan->peak_limit < 512)
    {
        plan->peak_limit = 512;
    }

    plan->compressor_ratio_divisor = 8;
    plan->compressor_threshold = plan->peak_limit / 4;
    if (plan->compressor_threshold < 512)
    {
        plan->compressor_threshold = 512;
    }

    if (wav_info->audio_format == WAV_MULAW_FORMAT)
    {
        /* G.711 mu-law is already companded speech. Additional compression
         * made the prompt harsh. Keep only a gentle peak limiter here so
         * speech stays present before the global firmware gain is applied.
         */
        plan->compressor_ratio_divisor = 4;
        plan->compressor_threshold = plan->peak_limit;
    }

    ret = fs_seek(file, wav_info->data_offset, FS_SEEK_SET);
    if (ret != 0)
    {
        return AUDIO_PLAYER_ERROR;
    }

    bytes_remaining = wav_info->data_size;
    while (bytes_remaining > 0)
    {
        ssize_t bytes_read;
        size_t actual_frames;
        size_t sample_count;
        size_t output_samples;

        chunk_size = get_max_input_chunk_size(bytes_remaining,
                                              input_frame_size,
                                              sizeof(analysis_read_buffer));
        if (chunk_size == 0)
        {
            return AUDIO_PLAYER_ERROR;
        }
        bytes_read = sd_card_safe_read(file, analysis_read_buffer, chunk_size);
        if (bytes_read <= 0)
        {
            return AUDIO_PLAYER_ERROR;
        }

        ret = convert_audio_chunk(analysis_read_buffer, (size_t)bytes_read, g_audio_buffer,
                                  &output_samples, wav_info);
        if (ret != AUDIO_PLAYER_OK)
        {
            return ret;
        }

        actual_frames = (size_t)bytes_read / input_frame_size;
        if (actual_frames > AUDIO_BUFFER_SIZE_SAMPLES)
        {
            actual_frames = AUDIO_BUFFER_SIZE_SAMPLES;
        }

        sample_count = actual_frames * 2;
        for (size_t index = 0; index < sample_count; index++)
        {
            int16_t peak = abs_sample_16(g_audio_buffer[index]);
            if (peak > plan->input_peak)
            {
                plan->input_peak = peak;
            }
        }

        bytes_remaining -= (size_t)bytes_read;
    }

    ret = fs_seek(file, wav_info->data_offset, FS_SEEK_SET);
    if (ret != 0)
    {
        return AUDIO_PLAYER_ERROR;
    }

    if (plan->input_peak == 0)
    {
        return AUDIO_PLAYER_OK;
    }

    plan->estimated_peak = plan->input_peak;
    if (plan->estimated_peak > plan->compressor_threshold)
    {
        plan->estimated_peak = plan->compressor_threshold +
                               ((plan->estimated_peak - plan->compressor_threshold) /
                                plan->compressor_ratio_divisor);
    }
    if (plan->estimated_peak <= 0)
    {
        plan->estimated_peak = 1;
    }

    {
        /* Compute in int32 then clamp before the uint16 store: a near-silent
         * file (tiny estimated_peak) could otherwise overflow uint16. */
        int32_t makeup = ((int32_t)plan->peak_limit * 100) / plan->estimated_peak;
        if (makeup > AUDIO_MAX_MAKEUP_PERCENT)
        {
            makeup = AUDIO_MAX_MAKEUP_PERCENT;
        }
        if (wav_info->audio_format == WAV_MULAW_FORMAT && makeup < 85)
        {
            makeup = 85;
        }
        plan->makeup_gain_percent = (uint16_t)makeup;
    }
    plan->active = true;
    return AUDIO_PLAYER_OK;
}

static void apply_wav_loudness_plan_in_place(int16_t *samples,
                                             size_t sample_count,
                                             const wav_loudness_plan_t *plan)
{
    if (!samples || !plan || !plan->active)
    {
        return;
    }

    for (size_t index = 0; index < sample_count; index++)
    {
        int16_t sample = samples[index];
        int16_t limited_sample;

        sample = compress_pcm_sample(sample,
                                     plan->compressor_threshold,
                                     plan->compressor_ratio_divisor);
        sample = scale_and_clip_pcm_sample(sample, plan->makeup_gain_percent);

        limited_sample = sample;
        if (abs_sample_16(limited_sample) > plan->peak_limit)
        {
            limited_sample = (limited_sample < 0) ? (int16_t)(-plan->peak_limit) : plan->peak_limit;
        }

        samples[index] = limited_sample;
    }
}

static void analyze_pcm_chunk_for_current_gain(const int16_t *samples,
                                               size_t sample_count,
                                               uint16_t gain_percent,
                                               int16_t *raw_peak_out,
                                               int16_t *projected_peak_out,
                                               uint32_t *projected_clipped_samples_out)
{
    int16_t raw_peak = 0;
    int16_t projected_peak = 0;
    uint32_t projected_clipped_samples = 0;

    if (!samples)
    {
        if (raw_peak_out)
        {
            *raw_peak_out = 0;
        }
        if (projected_peak_out)
        {
            *projected_peak_out = 0;
        }
        if (projected_clipped_samples_out)
        {
            *projected_clipped_samples_out = 0;
        }
        return;
    }

    for (size_t index = 0; index < sample_count; index++)
    {
        int16_t raw_abs = abs_sample_16(samples[index]);
        int32_t scaled = ((int32_t)samples[index] * (int32_t)gain_percent) / 100;

        if (raw_abs > raw_peak)
        {
            raw_peak = raw_abs;
        }

        if (scaled > INT16_MAX)
        {
            scaled = INT16_MAX;
            projected_clipped_samples++;
        }
        else if (scaled < INT16_MIN)
        {
            scaled = INT16_MIN;
            projected_clipped_samples++;
        }

        int16_t projected_abs = abs_sample_16((int16_t)scaled);
        if (projected_abs > projected_peak)
        {
            projected_peak = projected_abs;
        }
    }

    if (raw_peak_out)
    {
        *raw_peak_out = raw_peak;
    }
    if (projected_peak_out)
    {
        *projected_peak_out = projected_peak;
    }
    if (projected_clipped_samples_out)
    {
        *projected_clipped_samples_out = projected_clipped_samples;
    }
}

audio_player_result_t audio_player_init(void)
{
    if (g_status.is_initialized)
    {
        return AUDIO_PLAYER_OK;
    }

    LOG_INF("Initializing audio player subsystem");

    /* Check I2S driver availability */
    if (!i2s_out_is_initialized())
    {
        LOG_ERR("I2S driver not initialized");
        return AUDIO_PLAYER_I2S_ERROR;
    }

    /* Clear status */
    memset(&g_status, 0, sizeof(g_status));
    g_status.is_initialized = true;

    /* Init mutex */
    k_mutex_init(&g_status_mutex);

    /* Start playback queue thread */
    playback_queue_tid = k_thread_create(&playback_queue_thread_data,
                                         playback_queue_stack,
                                         K_THREAD_STACK_SIZEOF(playback_queue_stack),
                                         playback_queue_thread_fn, NULL, NULL, NULL,
                                         K_PRIO_PREEMPT(6), 0, K_NO_WAIT);
    k_thread_name_set(playback_queue_tid, "audio_q");

    LOG_INF("Audio player initialized successfully");
    return AUDIO_PLAYER_OK;
}

/* When true, the next play_wav_from_sd runs "clean": no loudness compressor and
 * a fixed output gain g_clean_volume (see audio_player_play_wav_clean_from_sd). */
static bool g_clean_playback = false;
static uint16_t g_clean_volume = I2S_VOLUME_DEFAULT_PERCENT;

audio_player_result_t audio_player_play_wav_from_sd(const char *filename)
{
    struct fs_file_t file;
    char full_path[128];
    wav_info_t wav_info;
    static uint8_t read_buffer[512]; /* File read buffer (static to avoid stack overflow) */
    size_t bytes_remaining;
    size_t chunk_size;
    audio_player_result_t result = AUDIO_PLAYER_OK;
    int ret;
    uint16_t output_gain_percent = I2S_VOLUME_DEFAULT_PERCENT;
    uint16_t saved_output_gain_percent = 0; /* for clean-playback volume restore */
    wav_loudness_plan_t wav_loudness_plan = {0};
    int16_t wav_raw_peak = 0;
    int16_t wav_projected_peak = 0;
    uint32_t wav_projected_clipped_samples = 0;
    uint32_t wav_total_samples = 0;

    if (!filename)
    {
        LOG_ERR("Invalid filename");
        return AUDIO_PLAYER_ERROR;
    }

    /* Auto-initialize if not done */
    if (!g_status.is_initialized)
    {
        ret = audio_player_init();
        if (ret != AUDIO_PLAYER_OK)
        {
            return ret;
        }
    }

    LOG_INF("Playing WAV file: %s", filename);

    /* Notify BLE clients that playback started */
    (void)lbs_send_playback_state(LBS_PLAYBACK_STATE_PLAYING);

    /* Get SD card mount point */
    sd_card_info_t sd_info;
    data_logger_result_t data_ret = data_logger_get_sd_info(&sd_info);
    if (data_ret != DATA_LOGGER_OK || !sd_info.is_mounted)
    {
        LOG_ERR("SD card not available");
        return AUDIO_PLAYER_SD_ERROR;
    }

    /* Build full path */
    snprintf(full_path, sizeof(full_path), "%s/%s", sd_info.mount_point, filename);
    LOG_DBG("Full path: %s", full_path);

    /* Parse WAV header */
    result = audio_player_parse_wav_header(filename, &wav_info);
    if (result != AUDIO_PLAYER_OK)
    {
        LOG_ERR("Failed to parse WAV header: %d", result);
        return result;
    }

    /* Log WAV info */
        LOG_INF("WAV Info: fmt=%u, %u Hz, %u-bit, %u channels, %u bytes data",
            wav_info.audio_format, wav_info.sample_rate, wav_info.bits_per_sample,
            wav_info.channels, wav_info.data_size);

    /* Open file for reading with SD card synchronization */
    fs_file_t_init(&file);
    ret = sd_card_safe_open(&file, full_path, FS_O_READ);
    if (ret != 0)
    {
        LOG_ERR("Failed to open WAV file: %d", ret);
        return AUDIO_PLAYER_FILE_NOT_FOUND;
    }

    /* Seek to data chunk */
    ret = fs_seek(&file, wav_info.data_offset, FS_SEEK_SET);
    if (ret != 0)
    {
        LOG_ERR("Failed to seek to data chunk: %d", ret);
        sd_card_safe_close(&file);
        i2s_out_stream_stop();
        return AUDIO_PLAYER_ERROR;
    }

    /* Update status */
    g_status.is_playing = true;
    g_status.current_wav = wav_info;
    strncpy(g_status.current_filename, filename, sizeof(g_status.current_filename) - 1);

    /* Stream audio data */
    bytes_remaining = wav_info.data_size;
    output_gain_percent = get_current_output_gain_percent();

    if (g_clean_playback)
    {
        /* Clean music playback: unity output gain, NO compressor/makeup -> so laut
         * wie die Quelle, ohne Uebersteuerung. Plan bleibt inaktiv (apply = no-op).
         * Datei-Position steht bereits am data_offset (fs_seek oben). */
        saved_output_gain_percent = output_gain_percent;
        output_gain_percent = g_clean_volume;
        (void)i2s_out_set_volume(g_clean_volume);
        LOG_INF("Clean playback: %u%% Gain, Kompressor aus", g_clean_volume);
    }
    else
    {
        result = build_wav_loudness_plan(&file, &wav_info, output_gain_percent, &wav_loudness_plan);
        if (result != AUDIO_PLAYER_OK)
        {
            LOG_ERR("Failed to prepare WAV loudness plan: %d", result);
            goto cleanup;
        }
    }

    /* Start I2S streaming */
    ret = i2s_out_stream_start(wav_info.sample_rate, 16, 2); /* Always output 16-bit stereo */
    if (ret < 0)
    {
        LOG_ERR("Failed to start I2S stream: %d", ret);
        result = AUDIO_PLAYER_I2S_ERROR;
        goto cleanup;
    }

    LOG_INF("Starting audio streaming (%u bytes)", bytes_remaining);
    LOG_INF("WAV loudness probe using current output gain: %u%%", output_gain_percent);
    if (wav_loudness_plan.active)
    {
        LOG_INF("WAV loudness plan for %s: peak_limit=%d threshold=%d ratio=%u:1 makeup=%u%% input_peak=%d estimated_peak=%d",
                filename,
                wav_loudness_plan.peak_limit,
                wav_loudness_plan.compressor_threshold,
                wav_loudness_plan.compressor_ratio_divisor,
                wav_loudness_plan.makeup_gain_percent,
                wav_loudness_plan.input_peak,
                wav_loudness_plan.estimated_peak);
    }

    while (bytes_remaining > 0 && g_status.is_playing)
    {
        /* Calculate chunk size to read */
        chunk_size = get_max_input_chunk_size(bytes_remaining,
                                              wav_info.bits_per_sample / 8 * wav_info.channels,
                                              sizeof(read_buffer));
        if (chunk_size == 0)
        {
            LOG_ERR("Invalid audio chunk sizing for frame width");
            result = AUDIO_PLAYER_ERROR;
            goto cleanup;
        }

        /* Read chunk from file */
        ssize_t bytes_read = sd_card_safe_read(&file, read_buffer, chunk_size);
        if (bytes_read <= 0)
        {
            LOG_ERR("Failed to read audio data: %zd", bytes_read);
            result = AUDIO_PLAYER_ERROR;
            goto cleanup; /* Ensure file is closed on error */
        }

        /* Convert audio format */
        size_t output_samples;
        result = convert_audio_chunk(read_buffer, bytes_read, g_audio_buffer,
                                     &output_samples, &wav_info);
        if (result != AUDIO_PLAYER_OK)
        {
            LOG_ERR("Failed to convert audio chunk");
            goto cleanup; /* Ensure file is closed on error */
        }

        /* Push to I2S (push partial buffers too for end of file) */
        if (output_samples > 0)
        {
            size_t input_sample_size = wav_info.bits_per_sample / 8;
            size_t input_frame_size = input_sample_size * wav_info.channels;
            size_t actual_frames = bytes_read / input_frame_size;
            int16_t chunk_raw_peak = 0;
            int16_t chunk_projected_peak = 0;
            uint32_t chunk_projected_clipped_samples = 0;

            if (actual_frames > AUDIO_BUFFER_SIZE_SAMPLES)
            {
                actual_frames = AUDIO_BUFFER_SIZE_SAMPLES;
            }

            apply_wav_loudness_plan_in_place(g_audio_buffer,
                                             actual_frames * 2,
                                             &wav_loudness_plan);

            analyze_pcm_chunk_for_current_gain(g_audio_buffer,
                                               actual_frames * 2,
                                               output_gain_percent,
                                               &chunk_raw_peak,
                                               &chunk_projected_peak,
                                               &chunk_projected_clipped_samples);

            if (chunk_raw_peak > wav_raw_peak)
            {
                wav_raw_peak = chunk_raw_peak;
            }
            if (chunk_projected_peak > wav_projected_peak)
            {
                wav_projected_peak = chunk_projected_peak;
            }
            wav_projected_clipped_samples += chunk_projected_clipped_samples;
            wav_total_samples += (uint32_t)(actual_frames * 2);

            size_t push_bytes = output_samples * 4; /* 16-bit stereo = 4 bytes per sample */
            ret = i2s_out_stream_push_from_buffer(g_audio_buffer, push_bytes);
            if (ret < 0)
            {
                if (ret == -EAGAIN || ret == -EBUSY)
                {
                    /* I2S busy, wait and retry */
                    k_sleep(K_MSEC(5));
                    ret = i2s_out_stream_push_from_buffer(g_audio_buffer, push_bytes);
                }
                if (ret < 0)
                {
                    LOG_ERR("Failed to push audio data to I2S: %d", ret);
                    result = AUDIO_PLAYER_I2S_ERROR;
                    goto cleanup; /* Ensure file is closed on error */
                }
            }
        }

        bytes_remaining -= bytes_read;

        /* Avoid sleeping here; keep feeding I2S to prevent underruns */
    }

cleanup: /* Centralized cleanup */
    sd_card_safe_close(&file);
    i2s_out_stream_stop();
    g_status.is_playing = false;

    /* Clean playback overrode the output volume to unity -> restore it. */
    if (saved_output_gain_percent != 0)
    {
        (void)i2s_out_set_volume(saved_output_gain_percent);
    }

    if (wav_total_samples > 0)
    {
        LOG_INF("WAV loudness stats [%s]: gain=%u%% raw_peak=%d projected_peak=%d projected_clipped=%u/%u samples",
                filename,
                output_gain_percent,
                wav_raw_peak,
                wav_projected_peak,
                wav_projected_clipped_samples,
                wav_total_samples);
        if (wav_projected_clipped_samples > 0)
        {
            LOG_WRN("WAV %s still clips at current firmware gain; further gain will mainly add distortion",
                    filename);
        }
    }

    if (result == AUDIO_PLAYER_OK)
    {
        LOG_INF("WAV playback completed successfully");
        (void)lbs_send_playback_state(LBS_PLAYBACK_STATE_STOPPED);
    }
    else
    {
        LOG_ERR("WAV playback failed: %d", result);
        (void)lbs_send_playback_state(LBS_PLAYBACK_STATE_ERROR);
    }

    return result;
}

audio_player_result_t audio_player_play_wav_clean_from_sd(const char *filename, uint16_t volume_percent)
{
    /* One-shot "clean" flag + volume consumed by audio_player_play_wav_from_sd. Blocking. */
    if (volume_percent > I2S_VOLUME_MAX_PERCENT)
    {
        volume_percent = I2S_VOLUME_MAX_PERCENT;
    }
    g_clean_volume = volume_percent;
    g_clean_playback = true;
    audio_player_result_t r = audio_player_play_wav_from_sd(filename);
    g_clean_playback = false;
    return r;
}

audio_player_result_t audio_player_stop(void)
{
    /* Acquire mutex to synchronize with playback thread */
    k_mutex_lock(&g_status_mutex, K_FOREVER);

    if (g_status.is_playing)
    {
        LOG_INF("Stopping audio playback");
        g_status.is_playing = false;
        i2s_out_stream_stop();
        (void)lbs_send_playback_state(LBS_PLAYBACK_STATE_STOPPED);
    }

    /* Clear pending queue (stop should cancel queued items) */
    char tmp[AUDIO_PLAYER_FILENAME_MAX];
    while (k_msgq_get(&audio_player_msgq, tmp, K_NO_WAIT) == 0)
    {
        LOG_DBG("Clearing queued playback: %s", tmp);
    }

    k_mutex_unlock(&g_status_mutex);
    return AUDIO_PLAYER_OK;
}

audio_player_result_t audio_player_get_status(audio_player_status_t *status)
{
    if (!status)
    {
        return AUDIO_PLAYER_ERROR;
    }

    *status = g_status;
    return AUDIO_PLAYER_OK;
}

audio_player_result_t audio_player_parse_wav_header(const char *filename, wav_info_t *wav_info)
{
    struct fs_file_t file;
    char full_path[128];
    wav_riff_header_t riff_header;
    wav_fmt_chunk_t fmt_chunk;
    uint32_t chunk_size, chunk_offset;
    int ret;

    if (!filename || !wav_info)
    {
        return AUDIO_PLAYER_ERROR;
    }

    /* Clear output structure */
    memset(wav_info, 0, sizeof(*wav_info));

    /* Get SD card mount point */
    sd_card_info_t sd_info;
    data_logger_result_t data_ret = data_logger_get_sd_info(&sd_info);
    if (data_ret != DATA_LOGGER_OK || !sd_info.is_mounted)
    {
        LOG_ERR("SD card not available");
        return AUDIO_PLAYER_SD_ERROR;
    }

    /* Build full path */
    snprintf(full_path, sizeof(full_path), "%s/%s", sd_info.mount_point, filename);

    /* Open file */
    fs_file_t_init(&file);
    ret = sd_card_safe_open(&file, full_path, FS_O_READ);
    if (ret != 0)
    {
        LOG_ERR("Failed to open WAV file: %d", ret);
        return AUDIO_PLAYER_FILE_NOT_FOUND;
    }

    /* Read RIFF header */
    ssize_t bytes_read = sd_card_safe_read(&file, &riff_header, sizeof(riff_header));
    if (bytes_read != sizeof(riff_header))
    {
        LOG_ERR("Failed to read RIFF header");
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_INVALID_FORMAT;
    }

    /* Validate RIFF header */
    if (riff_header.riff_magic != WAV_RIFF_MAGIC || riff_header.wave_magic != WAV_WAVE_MAGIC)
    {
        LOG_ERR("Invalid WAV file magic");
        LOG_DBG("riff_magic=0x%08X expected=0x%08X, wave_magic=0x%08X expected=0x%08X",
                riff_header.riff_magic, WAV_RIFF_MAGIC, riff_header.wave_magic, WAV_WAVE_MAGIC);
        /* Print first 12 bytes for offline inspection */
        uint8_t hdr_bytes[12];
        if (fs_seek(&file, 0, FS_SEEK_SET) == 0)
        {
            ssize_t read_bytes = sd_card_safe_read(&file, hdr_bytes, sizeof(hdr_bytes));
            if (read_bytes == sizeof(hdr_bytes))
            {
                LOG_DBG("Header bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                        hdr_bytes[0], hdr_bytes[1], hdr_bytes[2], hdr_bytes[3],
                        hdr_bytes[4], hdr_bytes[5], hdr_bytes[6], hdr_bytes[7],
                        hdr_bytes[8], hdr_bytes[9], hdr_bytes[10], hdr_bytes[11]);
            }
        }
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_INVALID_FORMAT;
    }

    /* Find format chunk */
    ret = find_wav_chunk(&file, WAV_FMT_MAGIC, &chunk_size, &chunk_offset);
    if (ret != AUDIO_PLAYER_OK)
    {
        LOG_ERR("Format chunk not found");
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_INVALID_FORMAT;
    }

    /* Read format chunk */
    ret = fs_seek(&file, chunk_offset, FS_SEEK_SET);
    if (ret != 0)
    {
        LOG_ERR("Failed to seek to format chunk");
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_ERROR;
    }

    bytes_read = sd_card_safe_read(&file, &fmt_chunk, sizeof(fmt_chunk));
    if (bytes_read != sizeof(fmt_chunk))
    {
        LOG_ERR("Failed to read format chunk");
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_INVALID_FORMAT;
    }

    /* Debug: show parsed fmt values */
    LOG_DBG("fmt chunk: audio_format=%u, channels=%u, sample_rate=%u, bits_per_sample=%u",
            fmt_chunk.audio_format, fmt_chunk.num_channels,
            fmt_chunk.sample_rate, fmt_chunk.bits_per_sample);

    /* Validate format */
    if (fmt_chunk.audio_format != WAV_PCM_FORMAT && fmt_chunk.audio_format != WAV_MULAW_FORMAT)
    {
        LOG_ERR("Unsupported WAV format %u", fmt_chunk.audio_format);
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_UNSUPPORTED_FORMAT;
    }

    if (fmt_chunk.num_channels == 0 || fmt_chunk.num_channels > 2)
    {
        LOG_ERR("Unsupported channel count: %u", fmt_chunk.num_channels);
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_UNSUPPORTED_FORMAT;
    }

    if (fmt_chunk.bits_per_sample != 8 && fmt_chunk.bits_per_sample != 16 &&
        fmt_chunk.bits_per_sample != 24 && fmt_chunk.bits_per_sample != 32)
    {
        LOG_ERR("Unsupported bit depth: %u", fmt_chunk.bits_per_sample);
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_UNSUPPORTED_FORMAT;
    }

    if (fmt_chunk.audio_format == WAV_MULAW_FORMAT && fmt_chunk.bits_per_sample != 8)
    {
        LOG_ERR("Unsupported mu-law bit depth: %u", fmt_chunk.bits_per_sample);
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_UNSUPPORTED_FORMAT;
    }

    /* Find data chunk */
    ret = find_wav_chunk(&file, WAV_DATA_MAGIC, &chunk_size, &chunk_offset);
    if (ret != AUDIO_PLAYER_OK)
    {
        LOG_ERR("Data chunk not found");
        sd_card_safe_close(&file);
        return AUDIO_PLAYER_INVALID_FORMAT;
    }

    /* Fill output structure */
    wav_info->audio_format = fmt_chunk.audio_format;
    wav_info->sample_rate = fmt_chunk.sample_rate;
    wav_info->channels = fmt_chunk.num_channels;
    wav_info->bits_per_sample = fmt_chunk.bits_per_sample;
    wav_info->data_size = chunk_size;
    wav_info->data_offset = chunk_offset;
    wav_info->is_valid = true;

    sd_card_safe_close(&file);

    LOG_DBG("WAV parsed: %u Hz, %u-bit, %u ch, %u bytes",
            wav_info->sample_rate, wav_info->bits_per_sample,
            wav_info->channels, wav_info->data_size);

    return AUDIO_PLAYER_OK;
}

/* Helper function to find a chunk in WAV file */
static audio_player_result_t find_wav_chunk(struct fs_file_t *file, uint32_t chunk_magic,
                                            uint32_t *chunk_size, uint32_t *chunk_offset)
{
    uint32_t magic, size;
    int ret;

    /* Start search after RIFF header */
    ret = fs_seek(file, sizeof(wav_riff_header_t), FS_SEEK_SET);
    if (ret != 0)
    {
        return AUDIO_PLAYER_ERROR;
    }

    /* Search for chunk */
    while (true)
    {
        /* Read chunk header */
        ssize_t bytes_read = sd_card_safe_read(file, &magic, sizeof(magic));
        if (bytes_read != sizeof(magic))
        {
            break; /* End of file */
        }

        bytes_read = sd_card_safe_read(file, &size, sizeof(size));
        if (bytes_read != sizeof(size))
        {
            break;
        }

        if (magic == chunk_magic)
        {
            /* Found the chunk */
            *chunk_size = size;
            *chunk_offset = fs_tell(file); /* Current position is start of chunk data */
            return AUDIO_PLAYER_OK;
        }

        /* Skip to next chunk */
        ret = fs_seek(file, size, FS_SEEK_CUR);
        if (ret != 0)
        {
            break;
        }
    }

    return AUDIO_PLAYER_INVALID_FORMAT;
}

/* Convert audio chunk to 16-bit stereo */
static audio_player_result_t convert_audio_chunk(const uint8_t *input, size_t input_bytes,
                                                 int16_t *output, size_t *output_samples,
                                                 const wav_info_t *wav_info)
{
    size_t input_sample_size = wav_info->bits_per_sample / 8;
    size_t input_frame_size = input_sample_size * wav_info->channels;
    size_t input_frames = input_bytes / input_frame_size;
    size_t max_output_frames = AUDIO_BUFFER_SIZE_SAMPLES;
    size_t frames_to_convert = (input_frames > max_output_frames) ? max_output_frames : input_frames;

    const uint8_t *input_ptr = input;
    int16_t *output_ptr = output;

    for (size_t frame = 0; frame < frames_to_convert; frame++)
    {
        /* Convert left channel */
        int16_t left_sample;
        convert_sample_to_16bit(input_ptr,
                    &left_sample,
                    wav_info->bits_per_sample,
                    wav_info->audio_format,
                    true);

        /* Down-mix to mono and write the SAME value to both I2S channels. The
         * MAX98357A sums (L+R)/2, so feeding it identical L=R avoids the ~6 dB
         * level loss (and possible phase cancellation) a true stereo file would
         * otherwise suffer on this mono speaker. Mono files are unaffected. */
        int16_t mono_sample;
        if (wav_info->channels == 2)
        {
            int16_t right_sample;
            convert_sample_to_16bit(input_ptr + input_sample_size,
                                    &right_sample,
                                    wav_info->bits_per_sample,
                                    wav_info->audio_format,
                                    true);
            mono_sample = (int16_t)(((int32_t)left_sample + (int32_t)right_sample) / 2);
        }
        else
        {
            mono_sample = left_sample;
        }
        *output_ptr++ = mono_sample; /* Left  */
        *output_ptr++ = mono_sample; /* Right */

        input_ptr += input_frame_size;
    }

    /* Pad remaining samples with zeros if needed */
    while (frames_to_convert < max_output_frames)
    {
        *output_ptr++ = 0; /* Left */
        *output_ptr++ = 0; /* Right */
        frames_to_convert++;
    }

    *output_samples = frames_to_convert;
    return AUDIO_PLAYER_OK;
}

/* Convert single sample to 16-bit */
static void convert_sample_to_16bit(const uint8_t *input, int16_t *output,
                                    uint16_t bits_per_sample, uint16_t audio_format, bool is_signed)
{
    switch (bits_per_sample)
    {
    case 8:
        if (audio_format == WAV_MULAW_FORMAT)
        {
            *output = decode_mulaw_sample(*input);
        }
        else if (is_signed)
        {
            *output = ((int8_t)*input) << 8;
        }
        else
        {
            *output = ((int16_t)*input - 128) << 8;
        }
        break;

    case 16:
        *output = *(int16_t *)input;
        break;

    case 24:
    {
        int32_t sample = (input[0]) | (input[1] << 8) | (input[2] << 16);
        if (sample & 0x800000) /* Sign extend */
        {
            sample |= 0xFF000000;
        }
        *output = (int16_t)(sample >> 8);
    }
    break;

    case 32:
    {
        int32_t sample = *(int32_t *)input;
        *output = (int16_t)(sample >> 16);
    }
    break;

    default:
        *output = 0;
        break;
    }
}

/* Thread for async playback */
#define AUDIO_PLAYER_THREAD_STACK_SIZE 4096
#define AUDIO_PLAYER_THREAD_PRIORITY 5
static K_THREAD_STACK_DEFINE(audio_player_thread_stack, AUDIO_PLAYER_THREAD_STACK_SIZE);
static struct k_thread audio_player_thread_data;

/* Separate thread for buffer playback */
static K_THREAD_STACK_DEFINE(audio_buffer_thread_stack, AUDIO_PLAYER_THREAD_STACK_SIZE);
static struct k_thread audio_buffer_thread_data;

/* Playback queue thread - consumes filenames and plays them sequentially */
static void playback_queue_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    char fname[AUDIO_PLAYER_FILENAME_MAX];

    while (1)
    {
        int rc = k_msgq_get(&audio_player_msgq, &fname, K_FOREVER);
        if (rc == 0)
        {
            LOG_INF("Playback queue: starting %s", fname);
            /* SD-CORRUPTION GUARD: park the ADS131 SD writer BEFORE reading the
             * WAV from SD. Otherwise this thread holds sd_card_manager_lock for
             * the ENTIRE playback (seconds) via sd_card_safe_open/close; the BDF
             * writer then times out its 2 s lock, and after 5 consecutive
             * failures it calls try_remount_filesystem() (fs_unmount/fs_mount
             * WITHOUT the lock) while our WAV FIL is still open mid-fs_read ->
             * FAT corruption (mis-linked dir entries, lost .SNX). Parking the
             * writer (same pattern the mic feature uses) makes it never time out
             * and never reach the remount. Only SD logging pauses; the ADC
             * producer + BLE live stream keep running, and the writer flushes
             * the ring-buffered samples on resume. */
            data_logger_pause_ads131_logging(true);
            /* Blocking playback; respects g_status.is_playing and audio_player_stop */
            audio_player_play_wav_from_sd(fname);
            /* Resume SD logging on EVERY exit path (play_wav_from_sd always
             * returns: normal end, error, or audio_player_stop). */
            data_logger_pause_ads131_logging(false);
        }
    }
}

/* Static filename buffer to avoid heap allocation */
static char g_thread_filename[64];

audio_player_result_t audio_player_play_wav_from_sd_async(const char *filename)
{
    /* Backwards-compatible: enqueue filename for playback */
    return audio_player_queue_play_wav(filename);
}

/* Queue a WAV file for playback */
audio_player_result_t audio_player_queue_play_wav(const char *filename)
{
    if (!filename)
    {
        return AUDIO_PLAYER_ERROR;
    }

    if (!g_status.is_initialized)
    {
        int ret = audio_player_init();
        if (ret != AUDIO_PLAYER_OK)
            return ret;
    }

    size_t len = strlen(filename);
    if (len == 0 || len >= AUDIO_PLAYER_FILENAME_MAX)
    {
        return AUDIO_PLAYER_ERROR;
    }

    char item[AUDIO_PLAYER_FILENAME_MAX];
    strncpy(item, filename, sizeof(item));
    item[sizeof(item) - 1] = '\0';

    int rc = k_msgq_put(&audio_player_msgq, item, K_NO_WAIT);
    if (rc != 0)
    {
        LOG_WRN("Playback queue full, cannot enqueue %s", filename);
        return AUDIO_PLAYER_MEMORY_ERROR;
    }

    LOG_INF("Enqueued playback: %s", filename);
    return AUDIO_PLAYER_OK;
}

/**
 * @brief Play audio from a RAM buffer
 *
 * Plays mono 16-bit PCM audio from RAM buffer through I2S output.
 * Converts mono to stereo for I2S output.
 */
audio_player_result_t audio_player_play_from_buffer(const int16_t *buffer,
                                                    size_t num_samples,
                                                    uint32_t sample_rate)
{
    audio_player_result_t result = AUDIO_PLAYER_OK;
    int ret;

    if (!buffer || num_samples == 0)
    {
        LOG_ERR("Invalid buffer parameters");
        return AUDIO_PLAYER_ERROR;
    }

    /* Auto-initialize if not done */
    if (!g_status.is_initialized)
    {
        ret = audio_player_init();
        if (ret != AUDIO_PLAYER_OK)
        {
            return ret;
        }
    }

    LOG_INF("Playing audio from RAM buffer: %zu samples at %u Hz", num_samples, sample_rate);
    LOG_DBG("audio_player: starting buffer playback, samples=%zu, sample_rate=%u", num_samples, sample_rate);

    /* Start I2S streaming - always 16-bit stereo output */
    ret = i2s_out_stream_start(sample_rate, 16, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to start I2S stream: %d", ret);
        return AUDIO_PLAYER_I2S_ERROR;
    }

    /* Update status */
    g_status.is_playing = true;
    strncpy(g_status.current_filename, "[RAM Buffer]", sizeof(g_status.current_filename) - 1);

    /* Stream audio data - convert mono to stereo */
    size_t samples_remaining = num_samples;
    size_t sample_index = 0;

    while (samples_remaining > 0 && g_status.is_playing)
    {
        /* Calculate how many samples to process in this chunk */
        size_t chunk_samples = (samples_remaining > AUDIO_BUFFER_SIZE_SAMPLES)
                                   ? AUDIO_BUFFER_SIZE_SAMPLES
                                   : samples_remaining;

        /* Convert mono to stereo by duplicating samples */
        for (size_t i = 0; i < chunk_samples; i++)
        {
            int16_t sample = buffer[sample_index++];
            g_audio_buffer[i * 2] = sample;     /* Left channel */
            g_audio_buffer[i * 2 + 1] = sample; /* Right channel */
        }

        /* Push to I2S */
        size_t push_bytes = chunk_samples * 4; /* 16-bit stereo = 4 bytes per sample */
        LOG_DBG("audio_player: pushing %zu bytes to i2s (chunk_samples=%zu)", push_bytes, chunk_samples);
        ret = i2s_out_stream_push_from_buffer(g_audio_buffer, push_bytes);
        LOG_DBG("audio_player: i2s_out_stream_push_from_buffer returned %d", ret);
        if (ret < 0)
        {
            if (ret == -EAGAIN || ret == -EBUSY)
            {
                /* I2S busy, wait and retry */
                k_sleep(K_MSEC(5));
                ret = i2s_out_stream_push_from_buffer(g_audio_buffer, push_bytes);
            }
            if (ret < 0)
            {
                LOG_ERR("Failed to push audio data to I2S: %d", ret);
                result = AUDIO_PLAYER_I2S_ERROR;
                break;
            }
        }

        samples_remaining -= chunk_samples;
    }

    /* Cleanup */
    i2s_out_stream_stop();
    g_status.is_playing = false;

    if (result == AUDIO_PLAYER_OK)
    {
        LOG_INF("RAM buffer playback completed successfully");
    }
    else
    {
        LOG_ERR("RAM buffer playback failed: %d", result);
    }

    return result;
}

/* Structure for passing buffer playback parameters to thread */
typedef struct
{
    const int16_t *buffer;
    size_t num_samples;
    uint32_t sample_rate;
} buffer_playback_params_t;

static buffer_playback_params_t g_buffer_params;

static void audio_buffer_thread_fn(void *p1, void *p2, void *p3)
{
    buffer_playback_params_t *params = (buffer_playback_params_t *)p1;
    if (!params)
    {
        return;
    }

    /* Call blocking player */
    audio_player_play_from_buffer(params->buffer, params->num_samples, params->sample_rate);
}

audio_player_result_t audio_player_play_from_buffer_async(const int16_t *buffer,
                                                          size_t num_samples,
                                                          uint32_t sample_rate)
{
    if (!buffer || num_samples == 0)
    {
        return AUDIO_PLAYER_ERROR;
    }

    /* Store parameters in static buffer */
    g_buffer_params.buffer = buffer;
    g_buffer_params.num_samples = num_samples;
    g_buffer_params.sample_rate = sample_rate;

    /* Start thread */
    k_tid_t tid = k_thread_create(&audio_buffer_thread_data,
                                  audio_buffer_thread_stack,
                                  K_THREAD_STACK_SIZEOF(audio_buffer_thread_stack),
                                  audio_buffer_thread_fn,
                                  &g_buffer_params, NULL, NULL,
                                  AUDIO_PLAYER_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (!tid)
    {
        return AUDIO_PLAYER_ERROR;
    }

    k_thread_name_set(tid, "audio_buffer");
    return AUDIO_PLAYER_OK;
}

/* List WAV files in SOUNDS/ directory and call cb for each filename */
audio_player_result_t audio_player_list_wav_files(void (*cb)(const char *filename, void *ctx), void *ctx)
{
    if (!cb)
    {
        return AUDIO_PLAYER_ERROR;
    }

    /* Ensure SD mounted */
    sd_card_info_t sd_info;
    data_logger_result_t data_ret = data_logger_get_sd_info(&sd_info);
    if (data_ret != DATA_LOGGER_OK || !sd_info.is_mounted)
    {
        LOG_ERR("SD card not available for listing");
        return AUDIO_PLAYER_SD_ERROR;
    }

    char path[128];
    snprintf(path, sizeof(path), "%s/SOUNDS", sd_info.mount_point);

    LOG_INF("Listing WAV files in: %s", path);

    wav_list_ctx_t wctx = {.user_cb = cb, .user_ctx = ctx};
    int ret = data_logger_list_files(path, audio_player_data_logger_cb, &wctx);

    return (ret >= 0) ? AUDIO_PLAYER_OK : AUDIO_PLAYER_SD_ERROR;
}
