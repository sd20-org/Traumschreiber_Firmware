/*
 * Nordic nRF52840 SAADC Driver
 *
 * Driver for reading all 8 analog input channels (AI0-AI7) using the
 * built-in Successive Approximation ADC (SAADC) of the nRF52840 SoC.
 *
 * Features:
 * - 8-channel simultaneous sampling
 * - 12-bit resolution
 * - Internal 0.6V reference
 * - Configurable gain per channel
 * - Periodic sampling with timer integration
 *
 * @author ISDT
 * @date 2025
 */

#ifndef NORDIC_ADC_H__
#define NORDIC_ADC_H__

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Number of ADC channels (AI0-AI7) */
#define NORDIC_ADC_NUM_CHANNELS 8

/* ADC resolution */
#define NORDIC_ADC_RESOLUTION 12

/* Maximum ADC value (12-bit) */
#define NORDIC_ADC_MAX_VALUE ((1 << NORDIC_ADC_RESOLUTION) - 1)

/* Reference voltage (internal 0.6V reference) */
#define NORDIC_ADC_REF_VOLTAGE_MV 600

    /**
     * @brief Nordic ADC result structure
     */
    typedef struct
    {
        uint16_t channels[NORDIC_ADC_NUM_CHANNELS]; /**< Raw ADC values for AI0-AI7 */
        uint32_t timestamp_ms;                      /**< Timestamp when samples were taken */
        int voltage_mv[NORDIC_ADC_NUM_CHANNELS];    /**< Converted voltages in millivolts */
    } nordic_adc_sample_t;

    /**
     * @brief Nordic ADC return codes
     */
    typedef enum
    {
        NORDIC_ADC_OK = 0,
        NORDIC_ADC_ERROR_INIT = -1,
        NORDIC_ADC_ERROR_NOT_READY = -2,
        NORDIC_ADC_ERROR_SAMPLE = -3,
        NORDIC_ADC_ERROR_TIMEOUT = -4,
        NORDIC_ADC_ERROR_INVALID_PARAM = -5,
    } nordic_adc_result_t;

    /**
     * @brief Data ready callback function type
     *
     * Called when new ADC sample data is available.
     *
     * @param sample Pointer to the ADC sample data
     */
    typedef void (*nordic_adc_callback_t)(const nordic_adc_sample_t *sample);

    /**
     * @brief Nordic ADC configuration structure
     */
    typedef struct
    {
        nordic_adc_callback_t data_callback; /**< Optional data ready callback */
        uint32_t sample_interval_ms;         /**< Sampling interval in milliseconds */
        bool auto_start;                     /**< Automatically start periodic sampling */
    } nordic_adc_config_t;

    /**
     * @brief Initialize the Nordic ADC driver
     *
     * Configures all 8 SAADC channels and prepares the driver for sampling.
     *
     * @param config Pointer to configuration structure
     * @return NORDIC_ADC_OK on success, error code otherwise
     */
    nordic_adc_result_t nordic_adc_init(const nordic_adc_config_t *config);

    /**
     * @brief Start periodic ADC sampling
     *
     * Begins automatic sampling at the configured interval.
     *
     * @return NORDIC_ADC_OK on success, error code otherwise
     */
    nordic_adc_result_t nordic_adc_start_periodic_sampling(void);

    /**
     * @brief Stop periodic ADC sampling
     *
     * @return NORDIC_ADC_OK on success, error code otherwise
     */
    nordic_adc_result_t nordic_adc_stop_periodic_sampling(void);

    /**
     * @brief Take a single ADC sample
     *
     * Performs a one-shot sampling of all 8 channels.
     *
     * @param sample Pointer to store the sample data
     * @return NORDIC_ADC_OK on success, error code otherwise
     */
    nordic_adc_result_t nordic_adc_sample_once(nordic_adc_sample_t *sample);

    /**
     * @brief Check if ADC driver is initialized and ready
     *
     * @return true if ready, false otherwise
     */
    bool nordic_adc_is_ready(void);

    /**
     * @brief Convert raw ADC value to millivolts
     *
     * @param raw_value Raw 12-bit ADC value
     * @param gain_factor Gain factor (1, 2, 4, 6 for gains 1, 1/2, 1/4, 1/6)
     * @return Voltage in millivolts
     */
    int nordic_adc_raw_to_millivolts(uint16_t raw_value, int gain_factor);

    /**
     * @brief Deinitialize the Nordic ADC driver
     *
     * @return NORDIC_ADC_OK on success, error code otherwise
     */
    nordic_adc_result_t nordic_adc_deinit(void);

    /**
     * @brief Record microphone audio at high sample rate
     *
     * Records audio from AIN7 (microphone) at the specified sample rate
     * for the specified duration and stores samples in the provided buffer.
     *
     * @param sample_rate_hz Sample rate in Hz (e.g., 8000, 16000)
     * @param duration_ms Recording duration in milliseconds
     * @param buffer Buffer to store samples (must be pre-allocated)
     * @param buffer_size_samples Size of buffer in samples
     * @param samples_recorded Output: number of samples actually recorded
     * @return NORDIC_ADC_OK on success, error code otherwise
     */
    nordic_adc_result_t nordic_adc_record_microphone(uint32_t sample_rate_hz,
                                                     uint32_t duration_ms,
                                                     int16_t *buffer,
                                                     size_t buffer_size_samples,
                                                     size_t *samples_recorded);

#ifdef __cplusplus
}
#endif

#endif /* NORDIC_ADC_H__ */