/*
 * WAV File Writer for Traumschreiber2509
 *
 * Simple WAV file writer for saving audio samples to SD card
 *
 * @author ISDT
 * @date 2025
 */

#ifndef WAV_WRITER_H
#define WAV_WRITER_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief WAV writer result codes
 */
typedef enum
{
    WAV_WRITER_OK = 0,
    WAV_WRITER_ERROR_INVALID_PARAM = -1,
    WAV_WRITER_ERROR_FILE_OPEN = -2,
    WAV_WRITER_ERROR_WRITE = -3,
    WAV_WRITER_ERROR_FILE_CLOSE = -4,
    WAV_WRITER_ERROR = -99
} wav_writer_result_t;

/**
 * @brief Write audio samples to a WAV file on SD card
 *
 * @param filename Filename (without path, e.g., "RECORDING.WAV")
 * @param samples Pointer to audio sample buffer (16-bit signed PCM)
 * @param num_samples Number of samples in the buffer
 * @param sample_rate Sample rate in Hz (e.g., 8000, 16000)
 * @param num_channels Number of audio channels (1 = mono, 2 = stereo)
 *
 * @return WAV_WRITER_OK on success, error code otherwise
 */
wav_writer_result_t wav_writer_write_file(const char *filename,
                                          const int16_t *samples,
                                          size_t num_samples,
                                          uint32_t sample_rate,
                                          uint8_t num_channels);

#endif /* WAV_WRITER_H */
