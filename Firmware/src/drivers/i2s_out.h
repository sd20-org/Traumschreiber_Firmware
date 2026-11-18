/**
 * @file i2s_out.h
 * @brief I2S Audio Output Driver for MAX98357A
 *
 * Provides I2S audio output functionality for the MAX98357A Class-D amplifier.
 * Supports various audio formats and sample rates for the Traumschreiber2509.
 *
 * Features:
 * - 16-bit I2S audio output
 * - Configurable sample rates (8kHz - 48kHz)
 * - Tone generation (sine waves, square waves)
 * - Volume control via amplitude scaling
 * - Power management (SD_MODE control)
 *
 * @author ISDT
 * @date 2025
 */

#ifndef I2S_OUT_H
#define I2S_OUT_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Audio configuration */
#define I2S_SAMPLE_RATE_8KHZ 8000
#define I2S_SAMPLE_RATE_16KHZ 16000
#define I2S_SAMPLE_RATE_22KHZ 22050
#define I2S_SAMPLE_RATE_44KHZ 44100
#define I2S_SAMPLE_RATE_48KHZ 48000

#define I2S_BITS_PER_SAMPLE 16
#define I2S_CHANNELS 2 /* Stereo */
#define I2S_SAMPLE_SIZE_BYTES (I2S_BITS_PER_SAMPLE / 8)
#define I2S_FRAME_SIZE_BYTES (I2S_CHANNELS * I2S_SAMPLE_SIZE_BYTES)

#define I2S_VOLUME_DEFAULT_PERCENT 100

/* Hard output ceiling for EVERY playback path (tones, WAV, RAM buffer), after
 * the volume gain: 5200 = -16 dBFS. Why (fw 0.0.94, measured 2026-10-03 on
 * TS2607): the MAX98357A runs from the +3V3 LDO rail (HT7333); louder output
 * pulls that rail down to ~2.2 V for <1 ms and the SD card fails (CRC errors,
 * card lost until reset) -> EEG recording lost. Peaks <= -16 dBFS kept VDD
 * >= 2.85 V for speech and music. Remove
 * only with a hardware fix (amp on VBAT + bulk capacitor). */
#define I2S_OUTPUT_PEAK_LIMIT 5200
#define I2S_VOLUME_MAX_PERCENT 400

/* Buffer configuration */
#define I2S_BUFFER_SIZE_SAMPLES 256
#define I2S_BUFFER_SIZE_BYTES (I2S_BUFFER_SIZE_SAMPLES * I2S_FRAME_SIZE_BYTES)
#define I2S_NUM_BUFFERS 2

    /* Tone types */
    typedef enum
    {
        TONE_SINE = 0, /**< Sine wave */
        TONE_SQUARE,   /**< Square wave */
        TONE_SAWTOOTH, /**< Sawtooth wave */
        TONE_TRIANGLE, /**< Triangle wave */
        TONE_NOISE     /**< White noise */
    } tone_type_t;

    /* Audio configuration structure */
    typedef struct
    {
        uint32_t sample_rate;    /**< Sample rate in Hz */
        uint16_t volume_percent; /**< Gain in percent (100 = unity, 0-400 supported) */
        bool mono_mode;          /**< true = mono, false = stereo */
    } i2s_config_t;

    /* Tone configuration */
    typedef struct
    {
        tone_type_t type;           /**< Tone type */
        uint16_t frequency_hz;      /**< Frequency in Hz */
        uint16_t amplitude_percent; /**< Amplitude (0-100%) */
        uint32_t duration_ms;       /**< Duration in milliseconds (0 = infinite) */
    } tone_config_t;

    /* Status structure */
    typedef struct
    {
        bool is_initialized;
        bool is_playing;
        bool is_enabled; /**< SD_MODE state */
        uint32_t sample_rate;
        uint16_t volume;
        uint32_t frames_played;
    } i2s_status_t;

    /**
     * @brief Initialize the I2S audio output system
     *
     * Initializes the I2S peripheral, configures the MAX98357A amplifier,
     * and sets up audio buffers.
     *
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_init(void);

    /**
     * @brief Configure I2S audio parameters
     *
     * @param config Pointer to configuration structure
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_configure(const i2s_config_t *config);

    /**
     * @brief Enable/disable the MAX98357A amplifier
     *
     * Controls the SD_MODE pin to enable or disable the amplifier.
     *
     * @param enable true to enable, false to disable
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_enable(bool enable);

    /**
     * @brief Play a tone with specified parameters
     *
     * @param config Pointer to tone configuration
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_play_tone(const tone_config_t *config);

    /**
     * @brief Stop audio playback
     *
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_stop(void);

    /**
     * @brief Set volume level
     *
    * @param volume_percent Gain level (0-400%, 100 = unity)
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_set_volume(uint16_t volume_percent);

    /**
     * @brief Get current audio status
     *
     * @param status Pointer to status structure to fill
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_get_status(i2s_status_t *status);

    /**
     * @brief Play a beep sequence
     *
     * Plays a sequence of beeps with specified parameters.
     *
     * @param frequency_hz Beep frequency
     * @param beep_duration_ms Duration of each beep
     * @param pause_duration_ms Pause between beeps
     * @param count Number of beeps
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_play_beep_sequence(uint16_t frequency_hz,
                                   uint16_t beep_duration_ms,
                                   uint16_t pause_duration_ms,
                                   uint8_t count);

    /**
     * @brief Play a melody from an array of frequencies
     *
     * @param frequencies Array of frequencies in Hz
     * @param durations Array of durations in ms
     * @param count Number of notes
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_play_melody(const uint16_t *frequencies,
                            const uint16_t *durations,
                            uint8_t count);

    /**
     * @brief Check if I2S audio system is initialized
     *
     * @return true if initialized, false otherwise
     */
    bool i2s_out_is_initialized(void);

    /**
     * @brief Check if audio is currently playing
     *
     * @return true if playing, false otherwise
     */
    bool i2s_out_is_playing(void);

    /**
     * @brief Play the Morse-Blues startup melody
     *
     * Plays a predefined Morse-Blues melody sequence at startup.
     * Based on C-Blues scale (C, Eb, F, Gb, G, Bb) with short (q)
     * and long (h) note durations.
     *
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_play_startup_melody(void);

    /**
     * @brief Start streaming audio mode
     *
     * Configures I2S for streaming PCM audio data. Must be called before
     * using i2s_out_stream_push_from_buffer().
     *
     * @param sample_rate Sample rate in Hz (8000-48000)
     * @param bits_per_sample Bits per sample (8, 16, 24, 32)
     * @param channels Number of channels (1 or 2)
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_stream_start(uint32_t sample_rate, uint8_t bits_per_sample, uint8_t channels);

    /**
     * @brief Push audio data to streaming pipeline
     *
     * Copies audio data to I2S mem_slab and writes to I2S device.
     * Data must be signed 16-bit PCM, interleaved stereo format.
     *
     * @param buf Pointer to audio data buffer
     * @param size_bytes Size of data in bytes (must be multiple of frame size)
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_stream_push_from_buffer(const void *buf, size_t size_bytes);

    /**
     * @brief Stop streaming audio mode
     *
     * Stops I2S streaming and cleans up resources.
     *
     * @return 0 on success, negative error code on failure
     */
    int i2s_out_stream_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* I2S_OUT_H */
