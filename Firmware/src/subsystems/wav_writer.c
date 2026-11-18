/*
 * WAV File Writer for Traumschreiber2509
 *
 * Simple WAV file writer for saving audio samples to SD card
 *
 * @author ISDT
 * @date 2025
 */

#include "wav_writer.h"
#include "sd_card_manager.h"
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(wav_writer, LOG_LEVEL_INF);

/* WAV file header structure */
typedef struct __attribute__((packed))
{
    /* RIFF header */
    char riff_id[4];    /* "RIFF" */
    uint32_t file_size; /* File size - 8 */
    char wave_id[4];    /* "WAVE" */

    /* Format chunk */
    char fmt_id[4];           /* "fmt " */
    uint32_t fmt_size;        /* Format chunk size (16 for PCM) */
    uint16_t audio_format;    /* 1 = PCM */
    uint16_t num_channels;    /* 1 = mono, 2 = stereo */
    uint32_t sample_rate;     /* Sample rate in Hz */
    uint32_t byte_rate;       /* sample_rate * num_channels * bits_per_sample / 8 */
    uint16_t block_align;     /* num_channels * bits_per_sample / 8 */
    uint16_t bits_per_sample; /* 8, 16, etc. */

    /* Data chunk */
    char data_id[4];    /* "data" */
    uint32_t data_size; /* Size of audio data */
} wav_header_t;

wav_writer_result_t wav_writer_write_file(const char *filename,
                                          const int16_t *samples,
                                          size_t num_samples,
                                          uint32_t sample_rate,
                                          uint8_t num_channels)
{
    if (!filename || !samples || num_samples == 0)
    {
        LOG_ERR("Invalid parameters");
        return WAV_WRITER_ERROR_INVALID_PARAM;
    }

    LOG_INF("Writing WAV file: %s", filename);
    LOG_INF("  Samples: %zu", num_samples);
    LOG_INF("  Sample rate: %u Hz", sample_rate);
    LOG_INF("  Channels: %u", num_channels);
    LOG_INF("  Bits per sample: 16");

    /* Build full path - use same mount point as data_logger */
    char full_path[128];
    snprintf(full_path, sizeof(full_path), "/SD:/%s", filename);

    /* Prepare WAV header */
    wav_header_t header;
    memset(&header, 0, sizeof(header));

    /* RIFF header */
    memcpy(header.riff_id, "RIFF", 4);
    memcpy(header.wave_id, "WAVE", 4);

    /* Format chunk */
    memcpy(header.fmt_id, "fmt ", 4);
    header.fmt_size = 16;
    header.audio_format = 1; /* PCM */
    header.num_channels = num_channels;
    header.sample_rate = sample_rate;
    header.bits_per_sample = 16;
    header.byte_rate = sample_rate * num_channels * 2;
    header.block_align = num_channels * 2;

    /* Data chunk */
    memcpy(header.data_id, "data", 4);
    header.data_size = num_samples * num_channels * 2;
    header.file_size = 36 + header.data_size;

    LOG_DBG("WAV header prepared:");
    LOG_DBG("  File size: %u bytes", header.file_size + 8);
    LOG_DBG("  Data size: %u bytes", header.data_size);

    /* Open file for writing */
    struct fs_file_t file;
    fs_file_t_init(&file);

    int ret = fs_open(&file, full_path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
    if (ret < 0)
    {
        LOG_ERR("Failed to open file %s: %d", full_path, ret);
        return WAV_WRITER_ERROR_FILE_OPEN;
    }

    /* Write WAV header */
    ssize_t written = fs_write(&file, &header, sizeof(header));
    if (written != sizeof(header))
    {
        LOG_ERR("Failed to write WAV header: %d", (int)written);
        fs_close(&file);
        return WAV_WRITER_ERROR_WRITE;
    }

    LOG_DBG("WAV header written: %d bytes", (int)written);

    /* Write audio data in chunks to avoid FS/SD driver timeouts */
    size_t total_bytes = num_samples * num_channels * 2;
    size_t bytes_written = 0;
    const uint8_t *data_ptr = (const uint8_t *)samples;
    uint8_t chunk_buffer[512]; /* Small buffer for chunking if needed, or just write direct */

    /* We can write directly from the large buffer, but in smaller chunks */
    size_t chunk_size = 512; /* 512 bytes (1 sector) chunks to be safer */

    while (bytes_written < total_bytes)
    {
        size_t remaining = total_bytes - bytes_written;
        size_t current_chunk = (remaining > chunk_size) ? chunk_size : remaining;

        written = fs_write(&file, data_ptr + bytes_written, current_chunk);
        if (written != (ssize_t)current_chunk)
        {
            LOG_ERR("Failed to write audio data chunk at offset %zu: %d", bytes_written, (int)written);
            fs_close(&file);
            return WAV_WRITER_ERROR_WRITE;
        }

        bytes_written += current_chunk;

        /* Yield to keep system responsive and give SD card time to recover */
        k_sleep(K_MSEC(1));
    }

    LOG_DBG("Audio data written: %zu bytes", bytes_written);

    /* Sync file to ensure all data is flushed to SD card */
    fs_sync(&file);

    /* Close file */
    ret = fs_close(&file);
    if (ret < 0)
    {
        LOG_ERR("Failed to close file: %d", ret);
        return WAV_WRITER_ERROR_FILE_CLOSE;
    }

    LOG_INF("WAV file written successfully: %s", filename);
    LOG_INF("  Total file size: %u bytes", header.file_size + 8);

    return WAV_WRITER_OK;
}
