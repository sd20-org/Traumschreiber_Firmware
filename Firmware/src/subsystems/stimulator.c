/**
 * @file stimulator.c
 * @brief LED and Audio Stimulator Subsystem Implementation
 *
 * Provides high-level LED pattern and audio control for the Traumschreiber2509.
 * Implements various visual and audio stimulation patterns using the RGB LED
 * and MAX98357A audio amplifier.
 *
 * @author ISDT
 * @date 2025
 */

#include "subsystems/stimulator.h"
#include "i2s_out.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(stimulator, LOG_LEVEL_INF);

/* Thread configuration */
#define STIMULATOR_THREAD_STACK_SIZE 2048
#define STIMULATOR_THREAD_PRIORITY 5

/* Static variables */
static struct k_thread stimulator_thread_data;
static k_tid_t stimulator_thread_id;
static K_THREAD_STACK_DEFINE(stimulator_thread_stack, STIMULATOR_THREAD_STACK_SIZE);

static stimulator_config_t current_config;
static stimulator_status_t status = {0};
static struct k_sem config_sem;
static bool thread_should_stop = false;

/* Predefined color sequences */
static const rgb_color_t color_sequence[] = {
    {100, 0, 0},     /* Red */
    {0, 100, 0},     /* Green */
    {0, 0, 100},     /* Blue */
    {100, 100, 0},   /* Yellow */
    {100, 0, 100},   /* Magenta */
    {0, 100, 100},   /* Cyan */
    {100, 100, 100}, /* White */
};

static const int color_sequence_length = ARRAY_SIZE(color_sequence);

/* Forward declarations */
static void stimulator_thread(void *arg1, void *arg2, void *arg3);
static void execute_smooth_cycle(void);
static void execute_breathing_pattern(void);
static void execute_rainbow_pattern(void);
static void execute_color_mixing(void);

int stimulator_init(void)
{
    if (status.is_initialized)
    {
        return 0;
    }

    LOG_INF("Initializing LED and audio stimulator subsystem...");

    /* Initialize hardware abstraction layer */
    int ret = rgb_led_init();
    if (ret < 0)
    {
        LOG_ERR("Failed to initialize RGB LED hardware: %d", ret);
        return ret;
    }

    /* Initialize I2S audio */
    ret = i2s_out_init();
    if (ret < 0)
    {
        LOG_ERR("Failed to initialize I2S audio: %d", ret);
        return ret;
    }

    /* Configure I2S with default settings */
    i2s_config_t audio_config = {
        .sample_rate = I2S_SAMPLE_RATE_16KHZ,
        .volume_percent = I2S_VOLUME_DEFAULT_PERCENT,
        .mono_mode = false};

    ret = i2s_out_configure(&audio_config);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure I2S audio: %d", ret);
        return ret;
    }

    /* Initialize synchronization primitives */
    k_sem_init(&config_sem, 0, 1);

    /* Initialize status */
    status.is_initialized = true;
    status.is_running = false;
    status.current_pattern = STIMULATOR_PATTERN_OFF;

    /* Create stimulator thread */
    stimulator_thread_id = k_thread_create(&stimulator_thread_data,
                                           stimulator_thread_stack,
                                           K_THREAD_STACK_SIZEOF(stimulator_thread_stack),
                                           stimulator_thread,
                                           NULL, NULL, NULL,
                                           STIMULATOR_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(stimulator_thread_id, "stimulator");

    LOG_INF("LED and audio stimulator subsystem initialized successfully");
    return 0;
}

int stimulator_start_pattern(const stimulator_config_t *config)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    if (config == NULL)
    {
        return -EINVAL;
    }

    /* Copy configuration */
    current_config = *config;

    /* Update status */
    status.current_pattern = config->pattern;
    status.pattern_start_time = k_uptime_get_32();
    status.is_running = true;

    /* Signal the thread */
    k_sem_give(&config_sem);

    LOG_INF("Started stimulator pattern: %d", config->pattern);
    return 0;
}

int stimulator_stop(void)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    status.is_running = false;
    status.current_pattern = STIMULATOR_PATTERN_OFF;

    /* Turn off LEDs */
    rgb_led_off();

    LOG_INF("Stopped stimulator");
    return 0;
}

int stimulator_set_static_color(const rgb_color_t *color)
{
    if (!status.is_initialized)
    {
        return -ENODEV;
    }

    if (color == NULL)
    {
        return -EINVAL;
    }

    status.current_pattern = STIMULATOR_PATTERN_STATIC;
    status.is_running = true;

    return rgb_led_set_color(color);
}

int stimulator_get_status(stimulator_status_t *status_out)
{
    if (status_out == NULL)
    {
        return -EINVAL;
    }

    if (status.is_running)
    {
        status.elapsed_time_ms = k_uptime_get_32() - status.pattern_start_time;
    }

    *status_out = status;
    return 0;
}

int stimulator_smooth_transition(const rgb_color_t *start_color,
                                 const rgb_color_t *end_color,
                                 uint16_t steps,
                                 uint16_t delay_ms)
{
    if (!rgb_led_is_initialized())
    {
        return -ENODEV;
    }

    if (start_color == NULL || end_color == NULL || steps == 0)
    {
        return -EINVAL;
    }

    rgb_color_t current_color;

    for (uint16_t step = 0; step <= steps; step++)
    {
        /* Linear interpolation between start and end colors */
        current_color.red = start_color->red +
                            ((int)(end_color->red - start_color->red) * step) / steps;
        current_color.green = start_color->green +
                              ((int)(end_color->green - start_color->green) * step) / steps;
        current_color.blue = start_color->blue +
                             ((int)(end_color->blue - start_color->blue) * step) / steps;

        int ret = rgb_led_set_color(&current_color);
        if (ret < 0)
        {
            return ret;
        }

        k_sleep(K_MSEC(delay_ms));
    }

    return 0;
}

int stimulator_breathing_effect(const rgb_color_t *base_color, uint16_t cycles)
{
    if (!rgb_led_is_initialized())
    {
        return -ENODEV;
    }

    if (base_color == NULL || cycles == 0)
    {
        return -EINVAL;
    }

    LOG_INF("Starting breathing effect: %d cycles", cycles);

    for (uint16_t cycle = 0; cycle < cycles; cycle++)
    {
        /* Fade in */
        int ret = stimulator_smooth_transition(&RGB_COLOR_OFF, base_color, 50, 20);
        if (ret < 0)
        {
            return ret;
        }

        /* Fade out */
        ret = stimulator_smooth_transition(base_color, &RGB_COLOR_OFF, 50, 20);
        if (ret < 0)
        {
            return ret;
        }
    }

    return 0;
}

int stimulator_rainbow_cycle(uint16_t duration_sec)
{
    if (!rgb_led_is_initialized())
    {
        return -ENODEV;
    }

    if (duration_sec == 0)
    {
        return -EINVAL;
    }

    int steps = duration_sec * 10; /* 10 steps per second */
    rgb_color_t color;

    LOG_INF("Starting rainbow cycle for %d seconds", duration_sec);

    for (int step = 0; step < steps; step++)
    {
        float hue = (360.0f * step) / steps; /* 0-360 degrees */

        /* Simple HSV to RGB conversion (S=1, V=1) */
        float c = 1.0f; /* Chroma */
        float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
        float r, g, b;

        if (hue < 60)
        {
            r = c;
            g = x;
            b = 0;
        }
        else if (hue < 120)
        {
            r = x;
            g = c;
            b = 0;
        }
        else if (hue < 180)
        {
            r = 0;
            g = c;
            b = x;
        }
        else if (hue < 240)
        {
            r = 0;
            g = x;
            b = c;
        }
        else if (hue < 300)
        {
            r = x;
            g = 0;
            b = c;
        }
        else
        {
            r = c;
            g = 0;
            b = x;
        }

        /* Convert to percentage */
        color.red = (uint8_t)(r * 100);
        color.green = (uint8_t)(g * 100);
        color.blue = (uint8_t)(b * 100);

        int ret = rgb_led_set_color(&color);
        if (ret < 0)
        {
            return ret;
        }

        k_sleep(K_MSEC(100));
    }

    return 0;
}

int stimulator_color_mixing_demo(void)
{
    if (!rgb_led_is_initialized())
    {
        return -ENODEV;
    }

    LOG_INF("Starting color mixing demonstration");

    rgb_color_t mixed_color;
    int ret;

    /* Red + Green = Yellow */
    LOG_INF("Red + Green = Yellow");
    mixed_color = (rgb_color_t){50, 50, 0};
    ret = rgb_led_set_color(&mixed_color);
    if (ret < 0)
        return ret;
    k_sleep(K_SECONDS(2));

    /* Green + Blue = Cyan */
    LOG_INF("Green + Blue = Cyan");
    mixed_color = (rgb_color_t){0, 50, 50};
    ret = rgb_led_set_color(&mixed_color);
    if (ret < 0)
        return ret;
    k_sleep(K_SECONDS(2));

    /* Red + Blue = Magenta */
    LOG_INF("Red + Blue = Magenta");
    mixed_color = (rgb_color_t){50, 0, 50};
    ret = rgb_led_set_color(&mixed_color);
    if (ret < 0)
        return ret;
    k_sleep(K_SECONDS(2));

    /* All colors = White */
    LOG_INF("Red + Green + Blue = White");
    mixed_color = (rgb_color_t){33, 33, 33};
    ret = rgb_led_set_color(&mixed_color);
    if (ret < 0)
        return ret;
    k_sleep(K_SECONDS(2));

    return 0;
}

bool stimulator_is_initialized(void)
{
    return status.is_initialized;
}

int stimulator_audio_visual_demo(void)
{
    if (!rgb_led_is_initialized() || !i2s_out_is_initialized())
    {
        return -ENODEV;
    }

    LOG_INF("Starting audio-visual demo");

    /* Demo sequence: LED colors with synchronized tones */
    const struct
    {
        rgb_color_t color;
        uint16_t frequency;
        uint16_t duration_ms;
        const char *name;
    } demo_sequence[] = {
        {{100, 0, 0}, 440, 500, "Red - A4"},       /* A4 note */
        {{0, 100, 0}, 494, 500, "Green - B4"},     /* B4 note */
        {{0, 0, 100}, 523, 500, "Blue - C5"},      /* C5 note */
        {{100, 100, 0}, 587, 500, "Yellow - D5"},  /* D5 note */
        {{100, 0, 100}, 659, 500, "Magenta - E5"}, /* E5 note */
        {{0, 100, 100}, 698, 500, "Cyan - F5"},    /* F5 note */
        {{100, 100, 100}, 784, 500, "White - G5"}  /* G5 note */
    };

    const size_t sequence_length = sizeof(demo_sequence) / sizeof(demo_sequence[0]);

    for (size_t i = 0; i < sequence_length; i++)
    {
        LOG_INF("Playing: %s", demo_sequence[i].name);

        /* Set LED color */
        int ret = rgb_led_set_color(&demo_sequence[i].color);
        if (ret < 0)
        {
            LOG_ERR("Failed to set LED color: %d", ret);
            return ret;
        }

        /* Play tone */
        tone_config_t tone = {
            .type = TONE_SINE,
            .frequency_hz = demo_sequence[i].frequency,
            .amplitude_percent = 60,
            .duration_ms = demo_sequence[i].duration_ms};

        ret = i2s_out_play_tone(&tone);
        if (ret < 0)
        {
            LOG_ERR("Failed to play tone: %d", ret);
            return ret;
        }

        /* Wait for tone to finish */
        k_sleep(K_MSEC(demo_sequence[i].duration_ms + 100));

        /* Stop audio and turn off LED briefly */
        i2s_out_stop();
        rgb_led_off();
        k_sleep(K_MSEC(200));
    }

    LOG_INF("Audio-visual demo completed");
    return 0;
}

int stimulator_beep_with_leds(uint16_t frequency_hz, uint8_t beep_count)
{
    if (!rgb_led_is_initialized() || !i2s_out_is_initialized())
    {
        return -ENODEV;
    }

    LOG_INF("Playing %u beeps at %u Hz with LED sync", beep_count, frequency_hz);

    /* Colors to cycle through */
    const rgb_color_t beep_colors[] = {
        {100, 0, 0},   /* Red */
        {0, 100, 0},   /* Green */
        {0, 0, 100},   /* Blue */
        {100, 100, 0}, /* Yellow */
        {100, 0, 100}  /* Magenta */
    };
    const size_t color_count = sizeof(beep_colors) / sizeof(beep_colors[0]);

    for (uint8_t i = 0; i < beep_count; i++)
    {
        /* Set LED color (cycle through colors) */
        const rgb_color_t *color = &beep_colors[i % color_count];
        int ret = rgb_led_set_color(color);
        if (ret < 0)
        {
            return ret;
        }

        /* Play beep */
        tone_config_t beep = {
            .type = TONE_SINE,
            .frequency_hz = frequency_hz,
            .amplitude_percent = 70,
            .duration_ms = 200};

        ret = i2s_out_play_tone(&beep);
        if (ret < 0)
        {
            return ret;
        }

        k_sleep(K_MSEC(250));
        i2s_out_stop();
        rgb_led_off();

        if (i < beep_count - 1)
        {
            k_sleep(K_MSEC(150)); /* Pause between beeps */
        }
    }

    return 0;
}

int stimulator_startup_sequence(void)
{
    LOG_INF("=== Traumschreiber2509 Morse-Blues Startup Sequence ===");

    if (!rgb_led_is_initialized())
    {
        LOG_ERR("RGB LED not initialized in startup sequence");
        return -ENODEV;
    }

    if (!i2s_out_is_initialized())
    {
        LOG_ERR("I2S not initialized in startup sequence");
        return -ENODEV;
    }

    /* Set a warm purple/violet color for the LED during melody */
    rgb_color_t startup_color = {.red = 60, .green = 20, .blue = 80}; /* Purple/violet */
    LOG_INF("Setting startup LED color (purple/violet)");
    int ret = rgb_led_set_color(&startup_color);
    if (ret < 0)
    {
        LOG_ERR("Failed to set startup LED color: %d", ret);
        return ret;
    }

    /* Play the Morse-Blues melody */
    LOG_INF("Playing Morse-Blues startup melody...");
    ret = i2s_out_play_startup_melody();
    if (ret < 0)
    {
        LOG_ERR("Failed to play startup melody: %d", ret);
        return ret;
    }

    /* Flash bright white briefly to indicate completion */
    LOG_INF("Melody completed - flashing completion signal");
    rgb_color_t white = {.red = 100, .green = 100, .blue = 100};
    ret = rgb_led_set_color(&white);
    if (ret < 0)
    {
        LOG_ERR("Failed to set completion LED: %d", ret);
        return ret;
    }

    k_sleep(K_MSEC(300));

    /* Quick rainbow flash to celebrate */
    rgb_color_t colors[] = {
        {100, 0, 0},  /* Red */
        {100, 50, 0}, /* Orange */
        {0, 100, 0},  /* Green */
        {0, 0, 100},  /* Blue */
        {50, 0, 100}  /* Purple */
    };

    for (int i = 0; i < 5; i++)
    {
        rgb_led_set_color(&colors[i]);
        k_sleep(K_MSEC(80));
    }

    rgb_led_off();

    LOG_INF("=== Traumschreiber2509 Startup Complete - System Ready! ===");
    return 0;
}

/* Thread implementation */
static void stimulator_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Stimulator thread started");

    while (!thread_should_stop)
    {
        /* Wait for configuration */
        if (k_sem_take(&config_sem, K_FOREVER) != 0)
        {
            continue;
        }

        if (!status.is_running)
        {
            continue;
        }

        /* Execute pattern based on configuration */
        switch (current_config.pattern)
        {
        case STIMULATOR_PATTERN_OFF:
            rgb_led_off();
            i2s_out_stop();
            break;

        case STIMULATOR_PATTERN_STATIC:
            rgb_led_set_color(&current_config.base_color);
            if (current_config.enable_audio && current_config.audio_frequency_hz > 0)
            {
                tone_config_t tone = {
                    .type = TONE_SINE,
                    .frequency_hz = current_config.audio_frequency_hz,
                    .amplitude_percent = 40,
                    .duration_ms = 0 /* Continuous */
                };
                i2s_out_play_tone(&tone);
            }
            break;

        case STIMULATOR_PATTERN_SMOOTH_CYCLE:
            execute_smooth_cycle();
            break;

        case STIMULATOR_PATTERN_BREATHING:
            execute_breathing_pattern();
            break;

        case STIMULATOR_PATTERN_RAINBOW:
            execute_rainbow_pattern();
            break;

        case STIMULATOR_PATTERN_COLOR_MIX:
            execute_color_mixing();
            break;

        case STIMULATOR_PATTERN_AUDIO_DEMO:
            stimulator_audio_visual_demo();
            status.is_running = false;
            break;

        case STIMULATOR_PATTERN_BEEP_SEQUENCE:
            stimulator_beep_with_leds(current_config.audio_frequency_hz, 5);
            status.is_running = false;
            break;

        case STIMULATOR_PATTERN_MELODY_DEMO:
            stimulator_startup_sequence();
            status.is_running = false;
            break;

        default:
            LOG_WRN("Unknown stimulator pattern: %d", current_config.pattern);
            break;
        }

        /* Check if pattern duration is exceeded */
        if (current_config.duration_ms > 0)
        {
            uint32_t elapsed = k_uptime_get_32() - status.pattern_start_time;
            if (elapsed >= current_config.duration_ms)
            {
                status.is_running = false;
                status.current_pattern = STIMULATOR_PATTERN_OFF;
                rgb_led_off();
            }
        }
    }

    LOG_INF("Stimulator thread terminated");
}

static void execute_smooth_cycle(void)
{
    for (int i = 0; i < color_sequence_length && status.is_running; i++)
    {
        int next_i = (i + 1) % color_sequence_length;

        LOG_DBG("Transitioning from color %d to color %d", i, next_i);
        stimulator_smooth_transition(&color_sequence[i], &color_sequence[next_i],
                                     current_config.transition_steps,
                                     current_config.delay_per_step_ms);
    }
}

static void execute_breathing_pattern(void)
{
    stimulator_breathing_effect(&current_config.base_color, 3);
}

static void execute_rainbow_pattern(void)
{
    stimulator_rainbow_cycle(10);
}

static void execute_color_mixing(void)
{
    stimulator_color_mixing_demo();
}
