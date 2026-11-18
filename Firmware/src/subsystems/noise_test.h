/*
 * noise_test — ADC noise measurement with shorted inputs.
 *
 * Switches the input multiplexer of all eight ADS131M08 channels to AGND
 * (MUXn = 01), lets the sinc3 filter settle for one second, then measures per
 * channel the broadband RMS and the RMS in the EMG band 20–45 Hz for the
 * requested number of seconds, switches back to the electrodes and appends the
 * result (µV, with the per-channel gain) as one line to NOISE.TXT on the SD
 * card. Recording and BLE streaming keep running; the samples of that period
 * carry the shorted-input signal and are marked in the BDF ButtonEvent signal
 * with code NOISE_TEST_BDF_MARKER (14).
 *
 * Purpose: an exact per-channel noise floor for post-processing (noise-power
 * subtraction in the EMG analysis)
 * instead of estimating it from unconnected channels.
 *
 * Triggers: automatically after the ADC starts (config.txt NOISE_TEST_SECS,
 * default 10, 0 = off) and over BLE (ADS control opcode 0x50) at any time.
 */
#ifndef NOISE_TEST_H
#define NOISE_TEST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NOISE_TEST_BDF_MARKER 14 /* ButtonEvent code while inputs are shorted */
#define NOISE_TEST_FILE "NOISE.TXT"

typedef enum
{
    NOISE_TEST_IDLE = 0,
    NOISE_TEST_RUNNING = 1,
    NOISE_TEST_DONE = 2,
    NOISE_TEST_FAILED = 3,
} noise_test_state_t;

/* Start a measurement of `seconds` (1..600). `trigger` is written into the
 * result line ("boot", "ble"). Returns 0, -EBUSY if one is running, -EINVAL. */
int noise_test_start(uint16_t seconds, const char *trigger);

/* Called for every ADC sample from the ADS reader thread (cheap). */
void noise_test_on_sample(const int32_t channels[8], uint32_t sample_number);

noise_test_state_t noise_test_get_state(void);
/* Increments with every finished measurement (wraps at 255). */
uint8_t noise_test_get_seq(void);
/* Last result in µV (0 if none yet). */
void noise_test_get_last(float rms_uv[8], float band_uv[8]);

#ifdef __cplusplus
}
#endif

#endif /* NOISE_TEST_H */
