/*
 * Nordic nRF52840 SAADC Driver Implementation
 *
 * Implementation of the Nordic SAADC driver for reading all 8 analog input
 * channels (AI0-AI7) using Zephyr's ADC API.
 *
 * @author ISDT
 * @date 2025
 */

#include "nordic_adc.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <nrfx_saadc.h>

LOG_MODULE_REGISTER(nordic_adc, LOG_LEVEL_DBG);

/* ADC device reference */
static const struct device *adc_dev = NULL;

/* ADC configuration */
static struct adc_channel_cfg adc_channels[NORDIC_ADC_NUM_CHANNELS];
static struct adc_sequence adc_sequence;
static int16_t adc_sample_buffer[NORDIC_ADC_NUM_CHANNELS];

/* Driver state */
static bool driver_initialized = false;
static nordic_adc_config_t driver_config;
static struct k_work_delayable adc_sample_work;

/* Per-sample ADC watchdog to detect blocking adc_read() calls */
static struct k_work_delayable adc_read_timeout_work;
static atomic_t adc_read_in_progress = ATOMIC_INIT(0);
static atomic_t adc_read_timed_out = ATOMIC_INIT(0);

/* Forward declarations */
static void adc_sample_work_handler(struct k_work *work);
static void adc_read_timeout_handler(struct k_work *work);

/**
 * @brief Convert raw ADC value to millivolts
 */
int nordic_adc_raw_to_millivolts(uint16_t raw_value, int gain_factor)
{
    /*
     * nRF52840 SAADC:
     * - 12-bit resolution (0-4095)
     * - Internal 0.6V reference
     * - Gain 1/6 means input range is 0-3.6V
     * - Formula: voltage = (raw_value * reference_voltage * gain_factor) / ADC_MAX
     */
    int voltage_mv;

    /* gain_factor values: 1 => gain=1, 2 => gain=1/2, 4 => gain=1/4, 6 => gain=1/6
     * Multiply by the denominator to reconstruct the VIN from the internal ref.
     * Example: ADC_GAIN_1_6 (denominator 6) -> raw * ref * 6 / ADC_MAX.
     */
    switch (gain_factor)
    {
    case 1: /* Gain 1 (no attenuation): 0 - 0.6V input range */
        voltage_mv = (raw_value * NORDIC_ADC_REF_VOLTAGE_MV * 1) / NORDIC_ADC_MAX_VALUE;
        break;
    case 2: /* Gain 1/2: input range 0-1.2V */
        voltage_mv = (raw_value * NORDIC_ADC_REF_VOLTAGE_MV * 2) / NORDIC_ADC_MAX_VALUE;
        break;
    case 4: /* Gain 1/4: input range 0-2.4V */
        voltage_mv = (raw_value * NORDIC_ADC_REF_VOLTAGE_MV * 4) / NORDIC_ADC_MAX_VALUE;
        break;
    case 6: /* Gain 1/6: input range 0-3.6V */
        voltage_mv = (raw_value * NORDIC_ADC_REF_VOLTAGE_MV * 6) / NORDIC_ADC_MAX_VALUE;
        break;
    default:
        /* Default assume no attenuation */
        voltage_mv = (raw_value * NORDIC_ADC_REF_VOLTAGE_MV * 1) / NORDIC_ADC_MAX_VALUE;
        break;
    }

    return voltage_mv;
}

/**
 * @brief Initialize ADC channels configuration
 */
static nordic_adc_result_t init_adc_channels(void)
{
    if (!adc_dev)
    {
        LOG_ERR("ADC device not found");
        return NORDIC_ADC_ERROR_INIT;
    }

/* Configure all ADC channels defined in device tree */
#define INIT_ADC_CHANNEL(node_id)                                                       \
    do                                                                                  \
    {                                                                                   \
        struct adc_channel_cfg cfg = ADC_CHANNEL_CFG_DT(node_id);                       \
        if (cfg.channel_id < NORDIC_ADC_NUM_CHANNELS)                                   \
        {                                                                               \
            adc_channels[cfg.channel_id] = cfg;                                         \
            LOG_INF("Configuring ADC Ch %d: Gain=%d, Ref=%d, AcqTime=%d",               \
                    cfg.channel_id, cfg.gain, cfg.reference, cfg.acquisition_time);     \
            int ret = adc_channel_setup(adc_dev, &adc_channels[cfg.channel_id]);        \
            if (ret < 0)                                                                \
            {                                                                           \
                LOG_ERR("Failed to configure ADC channel %d: %d", cfg.channel_id, ret); \
                return NORDIC_ADC_ERROR_INIT;                                           \
            }                                                                           \
        }                                                                               \
    } while (0);

    DT_FOREACH_CHILD(DT_NODELABEL(adc), INIT_ADC_CHANNEL)
#undef INIT_ADC_CHANNEL

    /* Setup ADC sequence for all channels */
    adc_sequence.channels = BIT_MASK(NORDIC_ADC_NUM_CHANNELS); /* Enable all 8 channels */
    adc_sequence.buffer = adc_sample_buffer;
    adc_sequence.buffer_size = sizeof(adc_sample_buffer);
    adc_sequence.resolution = NORDIC_ADC_RESOLUTION;
    adc_sequence.oversampling = 0; /* No oversampling */
    adc_sequence.calibrate = false;

    LOG_INF("All %d ADC channels configured successfully using device tree", NORDIC_ADC_NUM_CHANNELS);
    return NORDIC_ADC_OK;
}

/**
 * @brief Work handler for periodic ADC sampling
 */
static void adc_sample_work_handler(struct k_work *work)
{
    nordic_adc_sample_t sample;
    nordic_adc_result_t result;

    /* Take a sample */
    result = nordic_adc_sample_once(&sample);

    if (result == NORDIC_ADC_OK)
    {
        /* Call the user callback if configured */
        if (driver_config.data_callback)
        {
            driver_config.data_callback(&sample);
        }
        LOG_DBG("ADC sample taken at %u ms", sample.timestamp_ms);
        LOG_DBG("ADC raw values: AI0=%u, AI1=%u, AI2=%u, AI3=%u, AI4=%u, AI5=%u, AI6=%u, AI7=%u",
                sample.channels[0], sample.channels[1], sample.channels[2], sample.channels[3],
                sample.channels[4], sample.channels[5], sample.channels[6], sample.channels[7]);
        LOG_DBG("ADC sample: AI0=%dmV, AI1=%dmV, AI2=%dmV, AI3=%dmV, AI4=%dmV, AI5=%dmV, AI6=%dmV, AI7=%dmV",
                sample.voltage_mv[0], sample.voltage_mv[1], sample.voltage_mv[2], sample.voltage_mv[3],
                sample.voltage_mv[4], sample.voltage_mv[5], sample.voltage_mv[6], sample.voltage_mv[7]);
    }
    else
    {
        LOG_WRN("Failed to take ADC sample: %d", result);
    }

    /* Reschedule the work if periodic sampling is active */
    if (driver_initialized && driver_config.sample_interval_ms > 0)
    {
        k_work_reschedule(&adc_sample_work, K_MSEC(driver_config.sample_interval_ms));
    }
}

/**
 * @brief Timeout handler for ADC read operations
 */
static void adc_read_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (atomic_get(&adc_read_in_progress))
    {
        atomic_set(&adc_read_timed_out, 1);
        LOG_ERR("nordic_adc: adc_read() timed out (blocking)");
    }
}

/**
 * @brief Initialize the Nordic ADC driver
 */
nordic_adc_result_t nordic_adc_init(const nordic_adc_config_t *config)
{
    nordic_adc_result_t result;

    if (!config)
    {
        LOG_ERR("Invalid configuration parameter");
        return NORDIC_ADC_ERROR_INVALID_PARAM;
    }

    if (driver_initialized)
    {
        LOG_WRN("Nordic ADC driver already initialized");
        return NORDIC_ADC_OK;
    }

    LOG_INF("Initializing Nordic nRF52840 SAADC driver...");

    /* Get ADC device */
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    if (!device_is_ready(adc_dev))
    {
        LOG_ERR("ADC device not ready");
        return NORDIC_ADC_ERROR_INIT;
    }

    /* Store configuration */
    driver_config = *config;

    /* Initialize ADC channels */
    result = init_adc_channels();
    if (result != NORDIC_ADC_OK)
    {
        LOG_ERR("Failed to initialize ADC channels");
        return result;
    }

    /* Initialize work item for periodic sampling */
    k_work_init_delayable(&adc_sample_work, adc_sample_work_handler);
    k_work_init_delayable(&adc_read_timeout_work, adc_read_timeout_handler);

    driver_initialized = true;

    LOG_INF("Nordic ADC driver initialized successfully");
    LOG_INF("- Channels: AI0-AI7 (%d channels)", NORDIC_ADC_NUM_CHANNELS);
    LOG_INF("- Resolution: %d bits", NORDIC_ADC_RESOLUTION);
    LOG_INF("- Reference: Internal 0.6V");
    LOG_INF("- Sample interval: %d ms", config->sample_interval_ms);

    /* Start periodic sampling if requested */
    if (config->auto_start && config->sample_interval_ms > 0)
    {
        result = nordic_adc_start_periodic_sampling();
        if (result != NORDIC_ADC_OK)
        {
            LOG_WRN("Failed to start automatic periodic sampling");
        }
    }

    return NORDIC_ADC_OK;
}

/**
 * @brief Take a single ADC sample
 */
nordic_adc_result_t nordic_adc_sample_once(nordic_adc_sample_t *sample)
{
    int ret;

    if (!driver_initialized)
    {
        LOG_ERR("Nordic ADC driver not initialized");
        return NORDIC_ADC_ERROR_NOT_READY;
    }

    if (!sample)
    {
        LOG_ERR("Invalid sample parameter");
        return NORDIC_ADC_ERROR_INVALID_PARAM;
    }

    /* Clear the sample buffer */
    memset(adc_sample_buffer, 0, sizeof(adc_sample_buffer));

    /* Read all channels */
    ret = adc_read(adc_dev, &adc_sequence);
    if (ret < 0)
    {
        LOG_ERR("ADC read failed: %d", ret);
        return NORDIC_ADC_ERROR_SAMPLE;
    }

    /* Get current timestamp */
    sample->timestamp_ms = k_uptime_get_32();

    /* Return raw values only (no gain/voltage conversion) */
    for (int i = 0; i < NORDIC_ADC_NUM_CHANNELS; i++)
    {
        sample->channels[i] = (uint16_t)adc_sample_buffer[i];
        /* Convert raw ADC reading to millivolts using channel gain */
        int gain_factor = 1; /* denominator: 1,2,4,6 maps to ADC gain */
        switch (adc_channels[i].gain)
        {
        case ADC_GAIN_1_6:
            gain_factor = 6;
            break;
        case ADC_GAIN_1_4:
            gain_factor = 4;
            break;
        case ADC_GAIN_1_2:
            gain_factor = 2;
            break;
        case ADC_GAIN_1:
        default:
            gain_factor = 1;
            break;
        }

        sample->voltage_mv[i] = nordic_adc_raw_to_millivolts((uint16_t)adc_sample_buffer[i], gain_factor);
    }

    return NORDIC_ADC_OK;
}

/**
 * @brief Start periodic ADC sampling
 */
nordic_adc_result_t nordic_adc_start_periodic_sampling(void)
{
    if (!driver_initialized)
    {
        LOG_ERR("Nordic ADC driver not initialized");
        return NORDIC_ADC_ERROR_NOT_READY;
    }

    if (driver_config.sample_interval_ms == 0)
    {
        LOG_ERR("Sample interval not configured");
        return NORDIC_ADC_ERROR_INVALID_PARAM;
    }

    LOG_INF("Starting periodic ADC sampling (interval: %d ms)", driver_config.sample_interval_ms);

    /* Schedule the first sample */
    k_work_reschedule(&adc_sample_work, K_MSEC(driver_config.sample_interval_ms));

    return NORDIC_ADC_OK;
}

/**
 * @brief Stop periodic ADC sampling
 */
nordic_adc_result_t nordic_adc_stop_periodic_sampling(void)
{
    if (!driver_initialized)
    {
        LOG_ERR("Nordic ADC driver not initialized");
        return NORDIC_ADC_ERROR_NOT_READY;
    }

    LOG_INF("Stopping periodic ADC sampling");

    /* Cancel the work item */
    k_work_cancel_delayable(&adc_sample_work);

    return NORDIC_ADC_OK;
}

/**
 * @brief Check if ADC driver is initialized and ready
 */
bool nordic_adc_is_ready(void)
{
    return driver_initialized && adc_dev && device_is_ready(adc_dev);
}

/**
 * @brief Deinitialize the Nordic ADC driver
 */
nordic_adc_result_t nordic_adc_deinit(void)
{
    if (!driver_initialized)
    {
        LOG_WRN("Nordic ADC driver not initialized");
        return NORDIC_ADC_OK;
    }

    LOG_INF("Deinitializing Nordic ADC driver");

    /* Stop periodic sampling */
    nordic_adc_stop_periodic_sampling();

    /* Reset state */
    driver_initialized = false;
    adc_dev = NULL;
    memset(&driver_config, 0, sizeof(driver_config));

    return NORDIC_ADC_OK;
}

/**
 * @brief Record microphone audio at high sample rate
 *
 * This function temporarily pauses periodic sampling and records audio
 * from AIN7 (microphone) at high sample rate into a buffer.
 */
nordic_adc_result_t nordic_adc_record_microphone(uint32_t sample_rate_hz,
                                                 uint32_t duration_ms,
                                                 int16_t *buffer,
                                                 size_t buffer_size_samples,
                                                 size_t *samples_recorded)
{
    if (!driver_initialized)
    {
        LOG_ERR("Nordic ADC driver not initialized");
        return NORDIC_ADC_ERROR_NOT_READY;
    }

    if (!buffer || !samples_recorded)
    {
        LOG_ERR("Invalid parameters");
        return NORDIC_ADC_ERROR_INVALID_PARAM;
    }

    LOG_INF("Recording microphone: %u Hz, %u ms, buffer size: %zu samples",
            sample_rate_hz, duration_ms, buffer_size_samples);

    /* Calculate expected number of samples */
    size_t expected_samples = (sample_rate_hz * duration_ms) / 1000;
    if (expected_samples > buffer_size_samples)
    {
        LOG_WRN("Buffer too small: need %zu, have %zu", expected_samples, buffer_size_samples);
        expected_samples = buffer_size_samples;
    }

    /* Temporarily stop periodic sampling to avoid conflicts */
    bool was_periodic_sampling = driver_config.sample_interval_ms > 0;
    if (was_periodic_sampling)
    {
        k_work_cancel_delayable(&adc_sample_work);
    }

    /* Calculate sample interval in microseconds */
    uint32_t sample_interval_us = 1000000 / sample_rate_hz;

    *samples_recorded = 0;

    for (size_t i = 0; i < expected_samples; i++)
    {
        uint32_t start_cycle = k_cycle_get_32();

        /* Initialize sequence inside loop to ensure clean state for every sample */
        int16_t raw_sample = 0;
        struct adc_sequence mic_seq = {
            .channels = BIT(3), /* AIN3 */
            .buffer = &raw_sample,
            .buffer_size = sizeof(raw_sample),
            .resolution = NORDIC_ADC_RESOLUTION,
            .oversampling = 3, /* 8x Oversampling (2^3) to reduce latency and fix pitch */
            .calibrate = false};

        /* Read ADC channel 3 directly */
        int ret = adc_read(adc_dev, &mic_seq);
        if (ret < 0)
        {
            LOG_ERR("ADC read failed at sample %zu: %d", i, ret);
            break;
        }

        /*
         * Logging disabled to save cycles in the tight loop
         * if ((i % 1000) == 0) { LOG_DBG(...); }
         */

        /* Store the microphone sample */
        buffer[i] = raw_sample;
        (*samples_recorded)++;

        /* Busy wait to maintain sample rate */
        uint32_t end_cycle = k_cycle_get_32();
        uint32_t cycles_spent = end_cycle - start_cycle;
        uint32_t us_spent = k_cyc_to_us_floor32(cycles_spent);

        if (us_spent < sample_interval_us)
        {
            k_busy_wait(sample_interval_us - us_spent);
        }
        else
        {
            /* Optional: Log if we are missing deadlines */
            /* LOG_WRN("Sample %zu took %u us (target %u)", i, us_spent, sample_interval_us); */
        }
    }

    LOG_INF("Microphone recording complete: %zu samples recorded", *samples_recorded);

    /* Post-processing: Remove DC bias and scale to 16-bit */
    if (*samples_recorded > 0)
    {
        int64_t sum = 0;
        for (size_t i = 0; i < *samples_recorded; i++)
        {
            sum += buffer[i];
        }
        int16_t dc_bias = (int16_t)(sum / (int64_t)(*samples_recorded));

        LOG_INF("Removing DC bias: %d", dc_bias);

        for (size_t i = 0; i < *samples_recorded; i++)
        {
            int32_t val = buffer[i] - dc_bias;

            /*
             * Scaling:
             * - 8x Oversampling (sum of 8 samples).
             * - Shift left 7 bits to compensate for lower sum and add gain.
             * - Equivalent to previous 16x oversampling with shift 6.
             */
            val <<= 7;

            /* Clamp to int16 range */
            if (val > 32767)
                val = 32767;
            if (val < -32768)
                val = -32768;

            buffer[i] = (int16_t)val;
        }
    }

    /* Restart periodic sampling if it was active */
    if (was_periodic_sampling)
    {
        k_work_reschedule(&adc_sample_work, K_MSEC(driver_config.sample_interval_ms));
    }

    return NORDIC_ADC_OK;
}