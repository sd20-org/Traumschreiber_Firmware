/**
 * @file audio_player.h
 * @brief Audio Player Subsystem for WAV file playback
 *
 * Provides functionality to read WAV files from microSD card and play them
 * through the I2S audio output system (MAX98357A amplifier).
 *
 * Features:
 * - WAV file parsing and validation
 * - PCM format conversion (8/16/24/32-bit to 16-bit)
 * - Mono to stereo conversion
 * - Streaming playback to minimize memory usage
 * - Integration with existing I2S driver
 *
 * @author ISDT
 * @date 2025
 */

#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Error codes */
    typedef enum
    {
        AUDIO_PLAYER_OK = 0,
        AUDIO_PLAYER_ERROR = -1,
        AUDIO_PLAYER_FILE_NOT_FOUND = -2,
        AUDIO_PLAYER_INVALID_FORMAT = -3,
        AUDIO_PLAYER_UNSUPPORTED_FORMAT = -4,
        AUDIO_PLAYER_I2S_ERROR = -5,
        AUDIO_PLAYER_SD_ERROR = -6,
        AUDIO_PLAYER_MEMORY_ERROR = -7
    } audio_player_result_t;

    /* WAV file metadata */
    typedef struct
    {
        uint16_t audio_format;
        uint32_t sample_rate;
        uint16_t channels;
        uint16_t bits_per_sample;
        uint32_t data_size;
        uint32_t data_offset;
        bool is_valid;
    } wav_info_t;

    /* Audio player status */
    typedef struct
    {
        bool is_initialized;
        bool is_playing;
        wav_info_t current_wav;
        char current_filename[64];
    } audio_player_status_t;

    /**
     * @brief Initialize the audio player subsystem
     *
     * Sets up internal state and validates dependencies (I2S, data logger).
     * Optional - can be called automatically on first use.
     *
     * @return AUDIO_PLAYER_OK on success, error code on failure
     */
    audio_player_result_t audio_player_init(void);

    /**
     * @brief Play a WAV file from microSD card
     *
     * Reads and parses the WAV file, converts to I2S format, and streams
     * to the audio output. This is a blocking call that returns when
     * playback is complete or an error occurs.
     *
     * Supported formats:
     * - PCM only (no compression)
     * - 8/16/24/32-bit samples (converted to 16-bit)
     * - Mono/stereo (mono converted to stereo)
     * - Sample rates: 8kHz - 48kHz
     *
     * @param filename Filename on microSD (e.g., "BELL1.WAV")
     * @return AUDIO_PLAYER_OK on success, error code on failure
     */
    audio_player_result_t audio_player_play_wav_from_sd(const char *filename);

    /**
     * @brief Stop audio playback
     *
     * Stops current playback if active. Safe to call even if not playing.
     *
     * @return AUDIO_PLAYER_OK on success, error code on failure
     */
    audio_player_result_t audio_player_stop(void);

    /**
     * @brief Get current audio player status
     *
     * @param status Pointer to status structure to fill
     * @return AUDIO_PLAYER_OK on success, error code on failure
     */
    audio_player_result_t audio_player_get_status(audio_player_status_t *status);

    /**
     * @brief Parse WAV file header
     *
     * Reads and validates WAV file header, extracting metadata.
     * Internal function, exposed for testing.
     *
     * @param filename WAV file to parse
     * @param wav_info Pointer to structure to fill with metadata
     * @return AUDIO_PLAYER_OK on success, error code on failure
     */
    audio_player_result_t audio_player_parse_wav_header(const char *filename, wav_info_t *wav_info);

    /**
     * @brief Play a WAV file from microSD card asynchronously in a dedicated thread
     *
     * Spawns a thread that performs blocking playback so callers are not blocked and
     * the caller's stack is not consumed. The filename is copied into heap memory
     * and freed by the playback thread when finished.
     *
     * @param filename Filename on microSD (e.g., "BELL1.WAV")
     * @return AUDIO_PLAYER_OK if thread started, error code on failure
     */
    audio_player_result_t audio_player_play_wav_from_sd_async(const char *filename);

    /**
     * @brief Queue a WAV file for playback
     *
     * Adds `filename` to the playback queue. If nothing is playing the file
     * will be started immediately; otherwise it is queued and will play
     * after previously enqueued files complete. Queue has fixed depth.
     *
     * @param filename Path relative to SD mount (e.g., "SOUNDS/BELL1.WAV")
     * @return AUDIO_PLAYER_OK on success, error code otherwise
     */
    audio_player_result_t audio_player_queue_play_wav(const char *filename);

    /**
     * @brief Play a WAV file synchronously ("clean" music playback).
     *
     * Blocks until the file finished. Unlike the notification path it does NOT
     * apply the per-file loudness compressor/makeup; it plays at unity output
     * gain (100 %) so a music track is reproduced as loud as its source without
     * over-driving/clipping. Output volume is restored afterwards. Intended for
     * the configurable boot sound (START_SOUND), played before recording starts.
     *
     * @param filename Path relative to SD mount (e.g., "SOUNDS/SITTING.WAV")
     * @param volume_percent Output volume (0-400). Kept modest for the boot sound
     *        so a near-full-scale file does not analog-over-drive the amp/speaker.
     * @return AUDIO_PLAYER_OK on success, error code otherwise
     */
    audio_player_result_t audio_player_play_wav_clean_from_sd(const char *filename, uint16_t volume_percent);

    /**
     * @brief Play audio from a RAM buffer
     *
     * Plays audio samples from a RAM buffer through the I2S output.
     * Buffer must contain mono 16-bit signed PCM samples.
     * This function is blocking and returns when playback is complete.
     *
     * @param buffer Pointer to audio samples (mono, 16-bit signed PCM)
     * @param num_samples Number of samples in buffer
     * @param sample_rate Sample rate in Hz (e.g., 8000, 16000)
     * @return AUDIO_PLAYER_OK on success, error code on failure
     */
    audio_player_result_t audio_player_play_from_buffer(const int16_t *buffer,
                                                        size_t num_samples,
                                                        uint32_t sample_rate);

    /**
     * @brief Play audio from a RAM buffer asynchronously
     *
     * Spawns a thread to play audio from RAM buffer without blocking the caller.
     * The buffer must remain valid until playback completes.
     *
     * @param buffer Pointer to audio samples (mono, 16-bit signed PCM)
     * @param num_samples Number of samples in buffer
     * @param sample_rate Sample rate in Hz (e.g., 8000, 16000)
     * @return AUDIO_PLAYER_OK if thread started, error code on failure
     */
    audio_player_result_t audio_player_play_from_buffer_async(const int16_t *buffer,
                                                              size_t num_samples,
                                                              uint32_t sample_rate);

    /**
     * @brief List WAV files in the SOUNDS directory on SD card
     *
     * Calls the provided callback once per filename (UTF-8, null-terminated).
     * The callback may be called synchronously; filenames will be provided
     * without path (e.g., "BELL1.WAV").
     *
     * @param cb Callback invoked for each filename found
     * @param ctx Opaque context passed to the callback
     * @return AUDIO_PLAYER_OK on success, error code on failure
     */
    audio_player_result_t audio_player_list_wav_files(void (*cb)(const char *filename, void *ctx), void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_PLAYER_H */
