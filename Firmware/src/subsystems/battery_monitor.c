/*
 * Battery Monitor - Implementation
 *
 * Monitors battery voltage via Nordic SAADC and updates BLE Battery Service.
 * Assumes battery voltage is connected to one of the analog inputs.
 *
 * Battery voltage curve (Li-ion/Li-Po typical):
 * - 4.2V = 100%
 * - 3.7V = 50%
 * - 3.0V = 0%
 *
 * @author ISDT
 * @date 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "../ble/bas_service.h"
#include "../drivers/nordic_adc.h"

LOG_MODULE_REGISTER(battery_monitor, LOG_LEVEL_INF);

/* Battery ADC channel (AIN4) */
#define BATTERY_ADC_CHANNEL 4

/* Battery voltage thresholds in millivolts (calibrated from hardware measurements) */
#define BATTERY_VOLTAGE_MAX_MV 2800 /* 100% - fully charged */
#define BATTERY_VOLTAGE_MIN_MV 2500 /* 0% - empty */

/* Moving average filter settings */
#define FILTER_SAMPLES 10 /* Number of samples to average */

/* Update interval */
#define BATTERY_UPDATE_INTERVAL_MS (30 * 1000) /* 30 seconds */
/* Fast retry cadence until the FIRST successful ADC sample: boot ordering or
 * a slow-to-settle SAADC must not leave the level at 0 % for a full 30-s
 * tick (a phone connecting right after boot reads that 0 % via BAS). */
#define BATTERY_FIRST_READ_RETRY_MS 1000

/* Current battery state */
static uint32_t battery_voltage_mv = 0; /* Default: ~33% */
static uint8_t battery_level_percent = 0;
/* True once one real ADC sample has been captured (gates the fast retry). */
static bool battery_first_sample_ok = false;

/* Moving average filter state */
static int voltage_samples[FILTER_SAMPLES] = {0};
static uint8_t sample_index = 0;
static uint8_t samples_count = 0;

/* Work item for periodic updates */
static void battery_monitor_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(battery_monitor_work, battery_monitor_work_handler);

/**
 * @brief Convert battery voltage to percentage
 *
 * Linear interpolation between min and max voltage.
 *
 * @param voltage_mv Battery voltage in millivolts
 * @return Battery level in percent (0-100)
 */
static uint8_t voltage_to_percent(uint32_t voltage_mv)
{
    if (voltage_mv >= BATTERY_VOLTAGE_MAX_MV)
    {
        return 100;
    }

    if (voltage_mv <= BATTERY_VOLTAGE_MIN_MV)
    {
        return 0;
    }

    /* Linear interpolation */
    uint32_t range = BATTERY_VOLTAGE_MAX_MV - BATTERY_VOLTAGE_MIN_MV;
    uint32_t offset = voltage_mv - BATTERY_VOLTAGE_MIN_MV;
    uint8_t percent = (offset * 100) / range;

    return percent;
}

/**
 * @brief Apply moving average filter to voltage readings
 *
 * Smooths out fluctuations by averaging the last N samples.
 *
 * @param new_sample New voltage reading in millivolts
 * @return Filtered voltage in millivolts
 */
static uint32_t filter_voltage(int new_sample)
{
    /* Add new sample to circular buffer */
    voltage_samples[sample_index] = new_sample;
    sample_index = (sample_index + 1) % FILTER_SAMPLES;

    /* Track number of valid samples (until buffer is full) */
    if (samples_count < FILTER_SAMPLES)
    {
        samples_count++;
    }

    /* Calculate average */
    int32_t sum = 0;
    for (uint8_t i = 0; i < samples_count; i++)
    {
        sum += voltage_samples[i];
    }

    return (uint32_t)(sum / samples_count);
}

/**
 * @brief Read battery voltage from Nordic ADC channel 4
 *
 * Reads from AIN4 and applies moving average filter to smooth fluctuations.
 *
 * @return Battery voltage in millivolts (filtered)
 */
static uint32_t read_battery_voltage(void)
{
    /* Check if ADC is ready */
    if (!nordic_adc_is_ready())
    {
        LOG_WRN("Nordic ADC not ready, using last value");
        return battery_voltage_mv;
    }

    /* Read ADC sample */
    nordic_adc_sample_t sample;
    int err = nordic_adc_sample_once(&sample);
    if (err)
    {
        LOG_ERR("Failed to read ADC: %d", err);
        return battery_voltage_mv;
    }

    /* Extract battery voltage from channel 4 (AIN4) */
    int raw_voltage_mv = sample.voltage_mv[BATTERY_ADC_CHANNEL];
    battery_first_sample_ok = true;

    LOG_DBG("Raw battery voltage: %d mV", raw_voltage_mv);

    /* Apply moving average filter */
    uint32_t filtered_voltage = filter_voltage(raw_voltage_mv);

    LOG_DBG("Filtered battery voltage: %u mV (samples: %u)",
            filtered_voltage, samples_count);

    return filtered_voltage;
}

int battery_monitor_update(void)
{
    /* Read current battery voltage */
    battery_voltage_mv = read_battery_voltage();

    /* Convert to percentage */
    battery_level_percent = voltage_to_percent(battery_voltage_mv);

    LOG_INF("Battery: %u mV (%u%%)", battery_voltage_mv, battery_level_percent);

    /* Update BLE Battery Service */
    int err = bas_battery_level_update(battery_level_percent);
    if (err)
    {
        LOG_ERR("Failed to update BAS: %d", err);
        return err;
    }

    return 0;
}

static void battery_monitor_work_handler(struct k_work *work)
{
    /* Update battery level */
    battery_monitor_update();

    /* Schedule next update — fast retry until the first real sample landed
     * so an early failure (ADC not ready) is corrected within ~1 s instead
     * of a full 30-s interval. */
    k_work_schedule(&battery_monitor_work,
                    K_MSEC(battery_first_sample_ok
                               ? BATTERY_UPDATE_INTERVAL_MS
                               : BATTERY_FIRST_READ_RETRY_MS));
}

int battery_monitor_init(void)
{
    LOG_INF("=== Initializing Battery Monitor ===");
    LOG_INF("  Update interval: %d seconds", BATTERY_UPDATE_INTERVAL_MS / 1000);
    LOG_INF("  Voltage range: %u - %u mV",
            BATTERY_VOLTAGE_MIN_MV, BATTERY_VOLTAGE_MAX_MV);

    /* Perform initial update */
    battery_monitor_update();

    /* Schedule periodic updates (fast retry while the first read failed) */
    k_work_schedule(&battery_monitor_work,
                    K_MSEC(battery_first_sample_ok
                               ? BATTERY_UPDATE_INTERVAL_MS
                               : BATTERY_FIRST_READ_RETRY_MS));

    LOG_INF("Battery monitor initialized");
    return 0;
}

uint32_t battery_monitor_get_voltage_mv(void)
{
    return battery_voltage_mv;
}

uint8_t battery_monitor_get_level_percent(void)
{
    return battery_level_percent;
}
