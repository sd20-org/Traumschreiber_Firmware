/**
 * @file i2s_out.c
 * @brief I2S Audio Output Driver for MAX98357A Implementation
 *
 * Provides I2S audio output functionality for the MAX98357A Class-D amplifier.
 * Supports tone generation and audio playback for the Traumschreiber2509.
 *
 * @author ISDT
 * @date 2025
 */

#include "i2s_out.h"
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <zephyr/devicetree.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(i2s_out, LOG_LEVEL_INF);

/* Device tree nodes */
#define I2S_NODE DT_NODELABEL(i2s0)

/* Static variables */
static const struct device *i2s_dev;

static i2s_status_t status = {0};
static i2s_config_t current_config = {0};
static bool streaming_mode = false; /* true when in PCM streaming mode */
static bool tx_started = false;     /* true after I2S_TRIGGER_START issued */

/* Audio buffers */
static int16_t audio_buffer[I2S_NUM_BUFFERS][I2S_BUFFER_SIZE_SAMPLES * I2S_CHANNELS];
static uint8_t current_buffer = 0;
/* removed unused buffer_ready */

/* Memory Slab for I2S driver - CRITICAL for avoiding MPU fault */
#define I2S_MEM_SLAB_BLOCKS 12
K_MEM_SLAB_DEFINE(i2s_mem_slab, I2S_BUFFER_SIZE_BYTES, I2S_MEM_SLAB_BLOCKS, 4);

/* Instrumentation for mem_slab allocations in this file */
static atomic_t i2s_mem_slab_outstanding = ATOMIC_INIT(0);

static inline int i2s_mem_slab_alloc_instrument(void **mem_block, k_timeout_t timeout)
{
    int ret = k_mem_slab_alloc(&i2s_mem_slab, mem_block, timeout);
    if (ret == 0)
    {
        atomic_inc(&i2s_mem_slab_outstanding);
        LOG_DBG("mem_slab_alloc OK -> outstanding=%d", (int)atomic_get(&i2s_mem_slab_outstanding));
    }
    else
    {
        LOG_WRN("mem_slab_alloc FAILED (%d) -> outstanding=%d", ret, (int)atomic_get(&i2s_mem_slab_outstanding));
    }
    return ret;
}

static inline void i2s_mem_slab_free_instrument(void **mem_block)
{
    /* free may fail if pointer NULL - guard */
    if (mem_block && *mem_block)
    {
        k_mem_slab_free(&i2s_mem_slab, mem_block);
        atomic_dec(&i2s_mem_slab_outstanding);
        LOG_DBG("mem_slab_free -> outstanding=%d", (int)atomic_get(&i2s_mem_slab_outstanding));
    }
    else
    {
        LOG_WRN("mem_slab_free called with NULL block");
    }
}

/* Redirect k_mem_slab_alloc/free calls in this file to the instrumented wrappers.
 * Note: macros intentionally ignore the first parameter (the slab pointer) because
 * all uses in this file operate on the single i2s_mem_slab instance.
 */
#undef k_mem_slab_alloc
#undef k_mem_slab_free
#define k_mem_slab_alloc(_slab, _mem, _timeout) i2s_mem_slab_alloc_instrument((void **)(_mem), (_timeout))
#define k_mem_slab_free(_slab, _mem) i2s_mem_slab_free_instrument((void **)(_mem))

/* Thread configuration */
#define I2S_THREAD_STACK_SIZE 2048
#define I2S_THREAD_PRIORITY 3

static struct k_thread i2s_thread_data;
static k_tid_t i2s_thread_id;
static K_THREAD_STACK_DEFINE(i2s_thread_stack, I2S_THREAD_STACK_SIZE);

static struct k_sem buffer_sem;
static struct k_mutex config_mutex;
static bool thread_should_stop = false;

/* Amplifier control (optional) - MAX98357A SD pin from DTS */
static const struct gpio_dt_spec amp_sd = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(max98357_sd), gpios, {0});

/* Current tone parameters */
static tone_config_t current_tone = {0};
static bool tone_active = false;
static uint32_t tone_start_time = 0;
static uint32_t sample_counter = 0;

/* Morse-Blues startup melody data */
typedef struct
{
    uint16_t frequency_hz; /* 0 = pause/silence */
    uint16_t duration_ms;
} melody_note_t;

/* C-Blues scale frequencies */
#define NOTE_C 262  /* C4 */
#define NOTE_EB 311 /* Eb4 */
#define NOTE_F 349  /* F4 */
#define NOTE_GB 370 /* Gb4 */
#define NOTE_G 392  /* G4 */
#define NOTE_BB 466 /* Bb4 */
#define SILENCE 0   /* Pause */

/* Morse-Blues melody based on the piano roll provided */
static const melody_note_t startup_melody[] = {
    /* Time 1-3 */
    {NOTE_G, 200},  /* h - long G note */
    {NOTE_C, 100},  /* q - short C note */
    {NOTE_EB, 100}, /* q - short Eb note */
    {NOTE_F, 100},  /* q - short F note */

    /* Time 4-6 */
    {NOTE_GB, 100}, /* q - short Gb note */
    {NOTE_C, 100},  /* q - short C note */
    {NOTE_EB, 100}, /* q - short Eb note */
    {NOTE_BB, 200}, /* h - long Bb note */

    /* Time 7-9 */
    {NOTE_F, 100},  /* q - short F note */
    {SILENCE, 100}, /* . - pause */
    {NOTE_G, 200},  /* h - long G note */
    {NOTE_EB, 100}, /* q - short Eb note */

    /* Time 10-12 */
    {NOTE_C, 100},  /* q - short C note */
    {SILENCE, 100}, /* . - pause */
    {NOTE_F, 100},  /* q - short F note */
    {NOTE_BB, 200}, /* h - long Bb note */

    /* Time 13-15 */
    {SILENCE, 100}, /* . - pause */
    {NOTE_C, 100},  /* q - short C note */
    {NOTE_EB, 100}, /* q - short Eb note */
    {NOTE_GB, 100}, /* q - short Gb note */

    /* Time 16-18 */
    {NOTE_F, 100},  /* q - short F note */
    {NOTE_G, 200},  /* h - long G note */
    {NOTE_C, 100},  /* q - short C note */
    {NOTE_EB, 100}, /* q - short Eb note */

    /* Time 19-21 */
    {NOTE_F, 100},  /* q - short F note */
    {NOTE_GB, 100}, /* q - short Gb note */
    {SILENCE, 100}, /* . - pause */
    {NOTE_C, 100},  /* q - short C note */

    /* Time 22-24 */
    {NOTE_EB, 100}, /* q - short Eb note */
    {NOTE_BB, 200}, /* h - long Bb note */
    {NOTE_F, 100},  /* q - short F note */
    {NOTE_C, 100},  /* q - short C note */

    /* Time 25-27 */
    {SILENCE, 100}, /* . - pause */
    {NOTE_EB, 100}, /* q - short Eb note */
    {NOTE_G, 200},  /* h - long G note */
    {SILENCE, 200}, /* Final pause */
};

#define STARTUP_MELODY_LENGTH (sizeof(startup_melody) / sizeof(melody_note_t))

/* Forward declarations */
static void i2s_thread(void *arg1, void *arg2, void *arg3);
static void fill_tone_buffer(int16_t *buffer, size_t samples, const tone_config_t *tone);
static int16_t generate_tone_sample(tone_type_t type, uint32_t sample_idx, uint16_t frequency, uint32_t sample_rate);
static int16_t scale_pcm_sample(int16_t sample, uint16_t gain_percent);
static void scale_pcm_buffer_in_place(int16_t *samples, size_t sample_count, uint16_t gain_percent);
/* No TX reclaim function; TX blocks are freed by driver */

static int16_t scale_pcm_sample(int16_t sample, uint16_t gain_percent)
{
    int32_t scaled = (int32_t)sample * (int32_t)gain_percent;

    scaled /= 100;
    /* Supply-safety ceiling (see I2S_OUTPUT_PEAK_LIMIT). */
    if (scaled > I2S_OUTPUT_PEAK_LIMIT)
    {
        scaled = I2S_OUTPUT_PEAK_LIMIT;
    }
    else if (scaled < -I2S_OUTPUT_PEAK_LIMIT)
    {
        scaled = -I2S_OUTPUT_PEAK_LIMIT;
    }

    return (int16_t)scaled;
}

static void scale_pcm_buffer_in_place(int16_t *samples, size_t sample_count, uint16_t gain_percent)
{
    /* No early return at 100 %: the output ceiling must apply at unity gain too. */
    if (!samples)
    {
        return;
    }

    for (size_t index = 0; index < sample_count; index++)
    {
        samples[index] = scale_pcm_sample(samples[index], gain_percent);
    }
}

/* No I2S event callback used in this implementation */

int i2s_out_init(void)
{
    if (status.is_initialized)
    {
        return 0;
    }

    LOG_INF("I2S init - step 1: basic setup");

    /* Get I2S device */
    i2s_dev = DEVICE_DT_GET(I2S_NODE);
    if (!i2s_dev)
    {
        LOG_ERR("I2S device is NULL");
        return -ENODEV;
    }

    LOG_INF("I2S init - step 2: checking device readiness");
    if (!device_is_ready(i2s_dev))
    {
        LOG_ERR("I2S device not ready");
        return -ENODEV;
    }

    LOG_INF("I2S init - step 3: device is ready");

    /* Initialize synchronization primitives */
    LOG_INF("I2S init - step 4: initializing synchronization");
    k_sem_init(&buffer_sem, 0, I2S_NUM_BUFFERS);
    k_mutex_init(&config_mutex);

    /* Set default configuration */
    LOG_INF("I2S init - step 5: setting default config");
    current_config.sample_rate = I2S_SAMPLE_RATE_16KHZ;
    current_config.volume_percent = I2S_VOLUME_DEFAULT_PERCENT;
    current_config.mono_mode = false;

    /* Clear buffers */
    LOG_INF("I2S init - step 6: clearing buffers");
    memset(audio_buffer, 0, sizeof(audio_buffer));

    /* Configure amplifier SD pin if present in DT */
    if (amp_sd.port != NULL && gpio_is_ready_dt(&amp_sd))
    {
        int r = gpio_pin_configure_dt(&amp_sd, GPIO_OUTPUT_INACTIVE);
        if (r < 0)
        {
            LOG_WRN("Failed to configure amp SD pin: %d", r);
        }
        else
        {
            LOG_INF("Amplifier SD GPIO configured (pin %d)", amp_sd.pin);
        }
    }

    /* DON'T configure I2S during init - do it later to avoid hang */
    LOG_INF("I2S init - step 7: skipping I2S configuration to avoid hang");

    /* Create I2S thread */
    LOG_INF("I2S init - step 8: creating thread");
    i2s_thread_id = k_thread_create(&i2s_thread_data,
                                    i2s_thread_stack,
                                    K_THREAD_STACK_SIZEOF(i2s_thread_stack),
                                    i2s_thread,
                                    NULL, NULL, NULL,
                                    I2S_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(i2s_thread_id, "i2s_audio");

    /* Update status */
    LOG_INF("I2S init - step 9: updating status");
    status.is_initialized = true;
    status.is_playing = false;
    status.is_enabled = false;
    status.sample_rate = current_config.sample_rate;
    status.volume = current_config.volume_percent;
    status.frames_played = 0;

    LOG_INF("I2S init completed successfully - configuration will be done on first use");
    return 0;
}

int i2s_out_configure(const i2s_config_t *config)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    if (config == NULL)
    {
        return -EINVAL;
    }

    LOG_INF("=== I2S Configure Start ===");
    LOG_INF("Target: %u Hz, %u%% volume", config->sample_rate, config->volume_percent);

    LOG_INF("Step 1: Taking mutex");
    k_mutex_lock(&config_mutex, K_FOREVER);

    LOG_INF("Step 2: Updating internal configuration");
    current_config = *config;
    status.sample_rate = config->sample_rate;
    status.volume = config->volume_percent;

    LOG_INF("Step 3: Preparing I2S config structure");
    struct i2s_config i2s_cfg;
    memset(&i2s_cfg, 0, sizeof(i2s_cfg));

    i2s_cfg.word_size = I2S_BITS_PER_SAMPLE;
    i2s_cfg.channels = I2S_CHANNELS;
    i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
    i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
    i2s_cfg.frame_clk_freq = config->sample_rate;
    i2s_cfg.mem_slab = &i2s_mem_slab;
    i2s_cfg.block_size = I2S_BUFFER_SIZE_BYTES;
    i2s_cfg.timeout = 1000;

    LOG_INF("Step 4: Config prepared");
    LOG_INF("  word_size: %u", i2s_cfg.word_size);
    LOG_INF("  channels: %u", i2s_cfg.channels);
    LOG_INF("  format: %u", i2s_cfg.format);
    LOG_INF("  frame_clk_freq: %u", i2s_cfg.frame_clk_freq);
    LOG_INF("  block_size: %u", i2s_cfg.block_size);
    LOG_INF("  mem_slab: %p", i2s_cfg.mem_slab);

    LOG_INF("Step 5: About to call i2s_configure for TX");

    /* Configure I2S peripheral for TX */
    int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);

    LOG_INF("Step 6: i2s_configure TX returned: %d", ret);

    if (ret < 0)
    {
        LOG_ERR("i2s_configure TX failed with error: %d", ret);
        k_mutex_unlock(&config_mutex);
        return ret;
    }

    LOG_INF("Step 7: I2S TX configured successfully");

    LOG_INF("Step 8: Skipping RX configuration to avoid issues");

    LOG_INF("Step 9: Releasing mutex");
    k_mutex_unlock(&config_mutex);

    LOG_INF("=== I2S Configure Complete ===");
    return 0;
}

int i2s_out_enable(bool enable)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    status.is_enabled = enable;

    LOG_INF("MAX98357A %s", enable ? "enabled" : "disabled");

    /* Toggle amplifier SD pin if available in DeviceTree */
    if (amp_sd.port != NULL && gpio_is_ready_dt(&amp_sd))
    {
        int r;
        if (enable)
        {
            r = gpio_pin_set_dt(&amp_sd, 1);
            if (r < 0)
            {
                LOG_WRN("Failed to set amp SD pin high: %d", r);
            }
            /* Short delay to avoid pops when enabling amp */
            k_sleep(K_MSEC(20));
        }
        else
        {
            r = gpio_pin_set_dt(&amp_sd, 0);
            if (r < 0)
            {
                LOG_WRN("Failed to set amp SD pin low: %d", r);
            }
        }
    }

    return 0;
}

int i2s_out_play_tone(const tone_config_t *config)
{
    static bool i2s_configured = false;

    if (!status.is_initialized)
    {
        LOG_ERR("I2S not initialized");
        return -ENODEV;
    }

    if (config == NULL)
    {
        LOG_ERR("Configuration is NULL");
        return -EINVAL;
    }

    LOG_INF("Play tone request: %u Hz, %u%% amp, %u ms",
            config->frequency_hz, config->amplitude_percent, config->duration_ms);

    /* Configure I2S on first use if not already done */
    if (!i2s_configured)
    {
        LOG_INF("First tone playback - configuring I2S now");
        int ret = i2s_out_configure(&current_config);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure I2S: %d", ret);
            return ret;
        }
        i2s_configured = true;
        LOG_INF("I2S configured successfully on first use");
    }

    k_mutex_lock(&config_mutex, K_FOREVER);

    /* Enable amplifier if not already enabled */
    if (!status.is_enabled)
    {
        LOG_INF("Enabling amplifier");
        int ret = i2s_out_enable(true);
        if (ret < 0)
        {
            LOG_ERR("Failed to enable amplifier: %d", ret);
            k_mutex_unlock(&config_mutex);
            return ret;
        }
    }

    /* Update tone configuration */
    LOG_INF("Setting up tone parameters");
    current_tone = *config;
    tone_active = true;
    tone_start_time = k_uptime_get_32();
    sample_counter = 0;
    current_buffer = 0;
    status.is_playing = true;

    /* Fill first buffer */
    LOG_INF("Filling first buffer");
    fill_tone_buffer(audio_buffer[current_buffer], I2S_BUFFER_SIZE_SAMPLES, &current_tone);

    k_mutex_unlock(&config_mutex);

    /* CRITICAL FIX: Write buffers BEFORE starting I2S (like Nordic example) */
    LOG_INF("=== Writing initial buffers BEFORE starting I2S - using memory slab ===");

    LOG_INF("Step A: Allocating first memory slab buffer");
    /* Allocate buffer from memory slab (as in Nordic example) */
    void *mem_block;
    int ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
    if (ret < 0)
    {
        LOG_ERR("Failed to allocate memory slab block: %d", ret);
        tone_active = false;
        status.is_playing = false;
        return ret;
    }

    LOG_INF("Step B: Allocated mem_block=%p, copying %d bytes from audio_buffer[%d]=%p",
            mem_block, I2S_BUFFER_SIZE_BYTES, current_buffer, (void *)audio_buffer[current_buffer]);

    /* CRITICAL: Validate mem_block pointer before memcpy */
    if (!mem_block)
    {
        LOG_ERR("CRITICAL: Invalid mem_block pointer");
        k_mem_slab_free(&i2s_mem_slab, &mem_block);
        return -EINVAL;
    }

    /* Copy our tone data to the allocated block */
    memcpy(mem_block, audio_buffer[current_buffer], I2S_BUFFER_SIZE_BYTES);

    LOG_INF("Step C: Copied data, writing to I2S device");

    /* Write the memory slab buffer to I2S */
    ret = i2s_write(i2s_dev, mem_block, I2S_BUFFER_SIZE_BYTES);
    if (ret < 0)
    {
        LOG_ERR("Failed to write initial buffer: %d (mem_block=%p)", ret, mem_block);
        k_mem_slab_free(&i2s_mem_slab, &mem_block);
        tone_active = false;
        status.is_playing = false;
        return ret;
    }

    LOG_INF("Step D: First buffer written successfully, advancing current_buffer from %d to %d",
            current_buffer, (current_buffer + 1) % I2S_NUM_BUFFERS);
    current_buffer = (current_buffer + 1) % I2S_NUM_BUFFERS;

    LOG_INF("Step E: Filling second buffer [%d]", current_buffer);
    /* Write second buffer */
    fill_tone_buffer(audio_buffer[current_buffer], I2S_BUFFER_SIZE_SAMPLES, &current_tone);

    LOG_INF("Step F: Allocating second memory slab buffer");
    void *mem_block2;
    ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block2, K_NO_WAIT);
    if (ret < 0)
    {
        LOG_ERR("Failed to allocate second memory slab block: %d", ret);
        tone_active = false;
        status.is_playing = false;
        return ret;
    }

    LOG_INF("Step G: Allocated mem_block2=%p, copying from audio_buffer[%d]=%p",
            mem_block2, current_buffer, (void *)audio_buffer[current_buffer]);

    memcpy(mem_block2, audio_buffer[current_buffer], I2S_BUFFER_SIZE_BYTES);

    LOG_INF("Step H: Writing second buffer to I2S");
    ret = i2s_write(i2s_dev, mem_block2, I2S_BUFFER_SIZE_BYTES);
    if (ret < 0)
    {
        LOG_ERR("Failed to write second buffer: %d (mem_block2=%p)", ret, mem_block2);
        k_mem_slab_free(&i2s_mem_slab, &mem_block2);
        tone_active = false;
        status.is_playing = false;
        return ret;
    }

    LOG_INF("Second buffer written, now starting I2S");
    current_buffer = (current_buffer + 1) % I2S_NUM_BUFFERS;

    /* NOW start I2S after buffers are ready */
    ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0)
    {
        LOG_ERR("Failed to start I2S: %d", ret);
        tone_active = false;
        status.is_playing = false;
        return ret;
    }

    LOG_INF("I2S started successfully with pre-filled buffers");

    /* Signal thread to continue */
    LOG_INF("Signaling thread to continue");
    k_sem_give(&buffer_sem);

    /* No manual reclaim needed */

    LOG_INF("Tone playback started successfully");
    return 0;
}

int i2s_out_stop(void)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    LOG_INF("Stopping I2S audio playback");

    k_mutex_lock(&config_mutex, K_FOREVER);

    /* Stop tone generation */
    tone_active = false;
    status.is_playing = false;

    /* Use Nordic's proper stop sequence: DROP instead of STOP */
    int ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
    if (ret < 0)
    {
        LOG_WRN("Failed to drop I2S buffers: %d", ret);
        /* Try regular stop as fallback */
        ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_STOP);
        if (ret < 0)
        {
            LOG_WRN("Failed to stop I2S trigger: %d", ret);
        }
    }
    else
    {
        LOG_INF("I2S buffers dropped successfully");
    }

    /* Reset buffer state */
    current_buffer = 0;

    k_mutex_unlock(&config_mutex);

    LOG_INF("Audio playback stopped");
    return 0;
}

int i2s_out_set_volume(uint16_t volume_percent)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    if (volume_percent > I2S_VOLUME_MAX_PERCENT)
    {
        return -EINVAL;
    }

    k_mutex_lock(&config_mutex, K_FOREVER);
    current_config.volume_percent = volume_percent;
    status.volume = volume_percent;
    k_mutex_unlock(&config_mutex);

    LOG_DBG("Volume set to %u%%", volume_percent);
    return 0;
}

int i2s_out_get_status(i2s_status_t *status_out)
{
    if (status_out == NULL)
    {
        return -EINVAL;
    }

    *status_out = status;
    return 0;
}

int i2s_out_play_beep_sequence(uint16_t frequency_hz,
                               uint16_t beep_duration_ms,
                               uint16_t pause_duration_ms,
                               uint8_t count)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        tone_config_t beep = {
            .type = TONE_SINE,
            .frequency_hz = frequency_hz,
            .amplitude_percent = 100,
            .duration_ms = beep_duration_ms};

        int ret = i2s_out_play_tone(&beep);
        if (ret < 0)
        {
            return ret;
        }

        /* Wait for beep to finish */
        k_sleep(K_MSEC(beep_duration_ms));

        /* Stop and pause */
        i2s_out_stop();
        if (i < count - 1)
        { /* No pause after last beep */
            k_sleep(K_MSEC(pause_duration_ms));
        }
    }

    return 0;
}

int i2s_out_play_melody(const uint16_t *frequencies,
                        const uint16_t *durations,
                        uint8_t count)
{
    if (!status.is_initialized || frequencies == NULL || durations == NULL)
    {
        return -EINVAL;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        if (frequencies[i] > 0)
        { /* 0 = rest */
            tone_config_t note = {
                .type = TONE_SINE,
                .frequency_hz = frequencies[i],
                .amplitude_percent = 100,
                .duration_ms = durations[i]};

            int ret = i2s_out_play_tone(&note);
            if (ret < 0)
            {
                return ret;
            }
        }

        k_sleep(K_MSEC(durations[i]));
        i2s_out_stop();
    }

    return 0;
}

bool i2s_out_is_initialized(void)
{
    return status.is_initialized;
}

bool i2s_out_is_playing(void)
{
    return status.is_playing;
}

int i2s_out_play_startup_melody(void)
{
    if (!status.is_initialized)
    {
        LOG_ERR("I2S not initialized for startup melody");
        return -ENODEV;
    }

    LOG_INF("Playing Morse-Blues startup melody (%d notes)", STARTUP_MELODY_LENGTH);

    for (uint32_t i = 0; i < STARTUP_MELODY_LENGTH; i++)
    {
        const melody_note_t *note = &startup_melody[i];

        if (note->frequency_hz > 0)
        {
            /* Play the note */
            tone_config_t tone = {
                .type = TONE_SINE,
                .frequency_hz = note->frequency_hz,
                .amplitude_percent = 60, /* Medium volume for startup */
                .duration_ms = note->duration_ms};

            LOG_DBG("Note %u: %u Hz for %u ms", i, note->frequency_hz, note->duration_ms);

            int ret = i2s_out_play_tone(&tone);
            if (ret < 0)
            {
                LOG_ERR("Failed to play note %u: %d", i, ret);
                return ret;
            }

            /* Wait for note to finish */
            k_sleep(K_MSEC(note->duration_ms));
            i2s_out_stop();
        }
        else
        {
            /* Silence/pause */
            LOG_DBG("Pause %u: silence for %u ms", i, note->duration_ms);
            k_sleep(K_MSEC(note->duration_ms));
        }

        /* Small gap between notes for clarity */
        k_sleep(K_MSEC(50));
    }

    LOG_INF("Startup melody completed");
    return 0;
}

int i2s_out_stream_start(uint32_t sample_rate, uint8_t bits_per_sample, uint8_t channels)
{
    if (!status.is_initialized)
    {
        LOG_ERR("I2S not initialized");
        return -ENODEV;
    }

    /* Validate parameters */
    if (sample_rate < 8000 || sample_rate > 48000)
    {
        LOG_ERR("Invalid sample rate: %u", sample_rate);
        return -EINVAL;
    }

    if (bits_per_sample != 16)
    {
        LOG_ERR("Only 16-bit audio supported, got %u", bits_per_sample);
        return -EINVAL;
    }

    if (channels != 1 && channels != 2)
    {
        LOG_ERR("Only mono/stereo supported, got %u channels", channels);
        return -EINVAL;
    }

    LOG_INF("Starting I2S stream: %u Hz, %u-bit, %u channels", sample_rate, bits_per_sample, channels);

    /* Configure I2S with new parameters */
    i2s_config_t stream_config = {
        .sample_rate = sample_rate,
        .volume_percent = current_config.volume_percent,
        .mono_mode = (channels == 1)};

    int ret = i2s_out_configure(&stream_config);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure I2S for streaming: %d", ret);
        return ret;
    }

    /* Enable amplifier */
    ret = i2s_out_enable(true);
    if (ret < 0)
    {
        LOG_ERR("Failed to enable amplifier: %d", ret);
        return ret;
    }

    /* Reset streaming state */
    current_buffer = 0;
    status.is_playing = true;
    streaming_mode = true;
    tx_started = false;

    /* Prefill a few buffers BEFORE START */
    for (int i = 0; i < 4 && i < I2S_MEM_SLAB_BLOCKS; i++)
    {
        void *mem_block = NULL;
        ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
        if (ret < 0 || mem_block == NULL)
        {
            LOG_ERR("Failed to allocate prefill mem slab: %d", ret);
            status.is_playing = false;
            streaming_mode = false;
            tx_started = false;
            return (ret < 0) ? ret : -ENOMEM;
        }
        memset(mem_block, 0, I2S_BUFFER_SIZE_BYTES);
        ret = i2s_write(i2s_dev, mem_block, I2S_BUFFER_SIZE_BYTES);
        if (ret < 0)
        {
            LOG_ERR("Failed to prefill I2S buffer: %d", ret);
            k_mem_slab_free(&i2s_mem_slab, &mem_block);
            status.is_playing = false;
            streaming_mode = false;
            tx_started = false;
            return ret;
        }
    }

    /* Start TX after prefill */
    ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0)
    {
        LOG_ERR("Failed to START I2S after prefill: %d", ret);
        status.is_playing = false;
        streaming_mode = false;
        return ret;
    }
    tx_started = true;

    LOG_INF("I2S stream started successfully");
    return 0;
}

int i2s_out_stream_push_from_buffer(const void *buf, size_t size_bytes)
{
    if (!status.is_initialized || !status.is_playing || !streaming_mode)
    {
        LOG_ERR("I2S not initialized or not in streaming mode");
        return -ENODEV;
    }

    if (!buf || size_bytes == 0)
    {
        LOG_ERR("Invalid buffer or size");
        return -EINVAL;
    }

    /* Validate size is multiple of frame size */
    size_t frame_size = I2S_FRAME_SIZE_BYTES;
    if (size_bytes % frame_size != 0)
    {
        LOG_ERR("Buffer size %zu not multiple of frame size %zu", size_bytes, frame_size);
        return -EINVAL;
    }

    /* Support variable sized chunks up to I2S buffer size */
    if (size_bytes > I2S_BUFFER_SIZE_BYTES)
    {
        LOG_ERR("Buffer size %zu > I2S buffer size %d", size_bytes, I2S_BUFFER_SIZE_BYTES);
        return -EINVAL;
    }

    LOG_DBG("Pushing %zu bytes to I2S stream", size_bytes);

    /* No manual reclaim needed */

    /* Allocate memory slab block */
    void *mem_block;
    int ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_MSEC(50));
    if (ret < 0)
    {
        LOG_ERR("Failed to allocate memory slab: %d", ret);
        return ret;
    }

    /* CRITICAL: Validate mem_block pointer before memcpy */
    if (!mem_block)
    {
        LOG_ERR("CRITICAL: k_mem_slab_alloc returned 0 but mem_block is NULL");
        return -ENOMEM;
    }

    /* Copy audio data to allocated block and pad with zeros if needed */
    memcpy(mem_block, buf, size_bytes);
    if (size_bytes < I2S_BUFFER_SIZE_BYTES)
    {
        memset((uint8_t *)mem_block + size_bytes, 0, I2S_BUFFER_SIZE_BYTES - size_bytes);
    }

    /* Apply global volume scaling in-place on the mem_slab block so all playback paths
     * (WAV streaming, RAM buffer) respect a single firmware volume control.
     */
    uint16_t vol_pct = 100;
    if (k_mutex_lock(&config_mutex, K_MSEC(10)) == 0)
    {
        vol_pct = current_config.volume_percent;
        k_mutex_unlock(&config_mutex);
    }

    scale_pcm_buffer_in_place((int16_t *)mem_block,
                              I2S_BUFFER_SIZE_BYTES / sizeof(int16_t),
                              vol_pct);

    /* Write full I2S buffer size to device (padded if necessary) */
    for (int attempt = 0; attempt < 4; attempt++)
    {
        ret = i2s_write(i2s_dev, mem_block, I2S_BUFFER_SIZE_BYTES);
        if (ret == 0)
        {
            break;
        }
        if (ret == -EAGAIN || ret == -EBUSY)
        {
            /* Queue full or device busy: small backoff and retry */
            k_sleep(K_MSEC(1));
            continue;
        }
        LOG_ERR("Failed to write I2S buffer: %d", ret);
        k_mem_slab_free(&i2s_mem_slab, &mem_block);
        return ret;
    }
    if (ret < 0)
    {
        LOG_ERR("Failed to write I2S buffer after retries: %d", ret);
        k_mem_slab_free(&i2s_mem_slab, &mem_block);
        return ret;
    }

    /* No manual reclaim needed */

    LOG_DBG("Successfully pushed %zu bytes (%d buffer) to I2S", size_bytes, I2S_BUFFER_SIZE_BYTES);
    return 0;
}

int i2s_out_stream_stop(void)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    LOG_INF("Stopping I2S stream");

    status.is_playing = false;
    streaming_mode = false;

    int ret = 0;
    if (i2s_dev != NULL && tx_started)
    {
        /* Use DRAIN to play out remaining buffers, then DROP to stop */
        ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
        if (ret < 0)
        {
            LOG_WRN("Failed to drain I2S buffers: %d. This can happen at end of stream.", ret);
            /* Even if drain fails, proceed to drop to ensure the stream stops */
        }
    }

    tx_started = false;

    if (i2s_dev != NULL)
    {
        /* DROP clears any pending buffers and stops the TX stream immediately */
        ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
        if (ret < 0)
        {
            LOG_WRN("Failed to drop I2S buffers: %d", ret);
            /* As a fallback, try a simple STOP */
            ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_STOP);
            if (ret < 0)
            {
                LOG_WRN("Failed to stop I2S trigger: %d", ret);
            }
        }
    }

    /* No manual reclaim needed - I2S driver frees TX blocks automatically */

    LOG_INF("I2S stream stopped");
    return 0;
}

/* Thread implementation */
static void i2s_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("I2S audio thread started");

    while (!thread_should_stop)
    {
        LOG_DBG("I2S thread: waiting for semaphore...");

        /* Wait for playback request */
        if (k_sem_take(&buffer_sem, K_FOREVER) != 0)
        {
            LOG_WRN("I2S thread: semaphore take failed");
            continue;
        }

        LOG_DBG("I2S thread: got semaphore, checking tone_active=%d, is_playing=%d", tone_active, status.is_playing);

        /* Check if tone is still active */
        if (!tone_active || !status.is_playing)
        {
            LOG_DBG("Tone not active, skipping");
            continue;
        }

        LOG_DBG("I2S thread: filling buffer %d", current_buffer);

        /* Fill next buffer */
        fill_tone_buffer(audio_buffer[current_buffer], I2S_BUFFER_SIZE_SAMPLES, &current_tone);

        LOG_DBG("I2S thread: allocating memory slab");

        /* No manual reclaim needed */

        /* Allocate new memory slab buffer */
        void *mem_block;
        int ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
        if (ret < 0)
        {
            LOG_ERR("Failed to allocate memory slab in thread: %d", ret);
            k_sleep(K_MSEC(10));
            continue;
        }

        LOG_DBG("I2S thread: copying data to mem_block=%p, size=%d", mem_block, I2S_BUFFER_SIZE_BYTES);

        /* CRITICAL: Add bounds checking to prevent memory corruption */
        if (I2S_BUFFER_SIZE_BYTES > 1024)
        {
            LOG_ERR("CRITICAL: Buffer size %d exceeds safe limit", I2S_BUFFER_SIZE_BYTES);
            k_mem_slab_free(&i2s_mem_slab, &mem_block);
            continue;
        }

        memcpy(mem_block, audio_buffer[current_buffer], I2S_BUFFER_SIZE_BYTES);

        LOG_DBG("I2S thread: writing buffer to I2S, mem_block=%p", mem_block);

        /* Write new buffer to I2S */
        ret = i2s_write(i2s_dev, mem_block, I2S_BUFFER_SIZE_BYTES);
        if (ret < 0)
        {
            LOG_ERR("Failed to write I2S buffer: %d (mem_block=%p)", ret, mem_block);
            k_mem_slab_free(&i2s_mem_slab, &mem_block);
            k_sleep(K_MSEC(5));
            continue;
        }

        LOG_DBG("Buffer %d written successfully (mem_block=%p)", current_buffer, mem_block);

        /* No manual reclaim needed */

        /* Advance buffer index */
        current_buffer = (current_buffer + 1) % I2S_NUM_BUFFERS;
        status.frames_played += I2S_BUFFER_SIZE_SAMPLES;

        LOG_DBG("I2S thread: checking duration, elapsed=%u, target=%u",
                k_uptime_get_32() - tone_start_time, current_tone.duration_ms);

        /* Check if tone duration is finished */
        if (current_tone.duration_ms > 0)
        {
            uint32_t elapsed = k_uptime_get_32() - tone_start_time;
            if (elapsed >= current_tone.duration_ms)
            {
                LOG_INF("Tone duration finished (%u ms), stopping playback", elapsed);
                tone_active = false;
                status.is_playing = false;

                /* Proper I2S stop sequence */
                ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
                if (ret < 0)
                {
                    LOG_WRN("Failed to drop I2S buffers: %d", ret);
                }
                LOG_INF("I2S thread: tone finished, breaking loop");
                continue;
            }
        }

        LOG_DBG("I2S thread: continuing playback, giving semaphore after short delay");

        /* Continue playing - give semaphore for next buffer with small delay */
        k_sleep(K_MSEC(8)); /* Increase delay to give I2S more time */
        k_sem_give(&buffer_sem);

        LOG_DBG("I2S thread: semaphore given, loop continuing");
    }

    LOG_INF("I2S audio thread terminated");
}

static void fill_tone_buffer(int16_t *buffer, size_t samples, const tone_config_t *tone)
{
    LOG_DBG("fill_tone_buffer: buffer=%p, samples=%zu", (void *)buffer, samples);

    if (buffer == NULL)
    {
        LOG_ERR("fill_tone_buffer: buffer is NULL");
        return;
    }

    if (tone == NULL)
    {
        LOG_ERR("fill_tone_buffer: tone is NULL");
        return;
    }

    if (samples > I2S_BUFFER_SIZE_SAMPLES)
    {
        LOG_ERR("fill_tone_buffer: samples %zu exceeds buffer size %d", samples, I2S_BUFFER_SIZE_SAMPLES);
        return;
    }

    LOG_DBG("fill_tone_buffer: ENTRY - buffer=%p, samples=%zu, tone=%p", (void *)buffer, samples, (void *)tone);
    LOG_DBG("fill_tone_buffer: tone type=%d, freq=%u, amp=%u",
            tone->type, tone->frequency_hz, tone->amplitude_percent);

    // Calculate total elements needed (samples * channels)
    size_t total_elements = samples * I2S_CHANNELS;
    size_t buffer_capacity = I2S_BUFFER_SIZE_SAMPLES * I2S_CHANNELS;

    LOG_DBG("fill_tone_buffer: total_elements=%zu, capacity=%zu", total_elements, buffer_capacity);

    if (total_elements > buffer_capacity)
    {
        LOG_ERR("fill_tone_buffer: total elements %zu exceeds capacity %zu", total_elements, buffer_capacity);
        return;
    }

    LOG_DBG("fill_tone_buffer: starting sample generation loop for %zu samples", samples);

    for (size_t i = 0; i < samples; i++)
    {
        /* Add periodic logging to track progress */
        if (i % 64 == 0)
        {
            LOG_DBG("fill_tone_buffer: processing sample %zu of %zu", i, samples);
        }

        // Bounds check
        size_t left_idx = i * 2;
        size_t right_idx = i * 2 + 1;

        if (right_idx >= buffer_capacity)
        {
            LOG_ERR("fill_tone_buffer: index out of bounds at sample %zu", i);
            break;
        }

        int16_t sample = generate_tone_sample(tone->type, sample_counter++,
                                              tone->frequency_hz, current_config.sample_rate);

        if (i % 64 == 0)
        {
            LOG_DBG("fill_tone_buffer: generated sample %d for index %zu", sample, i);
        }

        /* Apply amplitude scaling (tone-local). */
        sample = (sample * tone->amplitude_percent) / 100;

        /* Apply the same firmware gain used by the streaming/WAV path. */
        sample = scale_pcm_sample(sample, current_config.volume_percent);

        if (i % 64 == 0)
        {
            LOG_DBG("fill_tone_buffer: scaled sample %d for index %zu", sample, i);
        }

        /* Fill stereo channels - MAX98357A will use left channel (GAIN_SLOT=3V3) */
        buffer[left_idx] = sample;  /* Left channel - used by MAX98357A */
        buffer[right_idx] = sample; /* Right channel - copy for I2S stereo format */
    }
    LOG_DBG("fill_tone_buffer: EXIT - completed filling %zu samples successfully", samples);
}

static int16_t generate_tone_sample(tone_type_t type, uint32_t sample_idx,
                                    uint16_t frequency, uint32_t sample_rate)
{
    const int16_t amplitude = 25000; /* ~75% of max range */

    /* Simplified phase calculation to avoid hanging */
    uint32_t period_samples = sample_rate / frequency;
    if (period_samples == 0)
        period_samples = 1; /* Avoid division by zero */

    uint32_t phase_pos = sample_idx % period_samples;

    switch (type)
    {
    case TONE_SINE:
        /* Simple sine approximation using lookup or basic math */
        /* For now, use square wave to avoid floating point issues */
        return (phase_pos < period_samples / 2) ? amplitude : -amplitude;

    case TONE_SQUARE:
        return (phase_pos < period_samples / 2) ? amplitude : -amplitude;

    case TONE_SAWTOOTH:
        /* Linear ramp */
        return (int16_t)(amplitude * (2.0f * phase_pos / period_samples - 1.0f));

    case TONE_TRIANGLE:
        /* Triangle wave */
        if (phase_pos < period_samples / 2)
        {
            return (int16_t)(amplitude * (4.0f * phase_pos / period_samples - 1.0f));
        }
        else
        {
            return (int16_t)(amplitude * (3.0f - 4.0f * phase_pos / period_samples));
        }

    case TONE_NOISE:
        /* Simple pseudo-random noise */
        return (int16_t)((sample_idx * 1103515245 + 12345) % (2 * amplitude) - amplitude);

    default:
        return 0;
    }
}

/* Reclaim transmitted TX buffers from I2S and return them to the memory slab */
/* This function is removed as the driver handles freeing TX blocks automatically. */
