/*
 * Traumschreiber2509 - SD Card Access Manager Implementation
 *
 * Provides synchronized access to the SD card filesystem to prevent
 * corruption from concurrent reads and writes.
 *
 * @author ISDT
 * @date 2025
 */

#include "sd_card_manager.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sd_card_manager, LOG_LEVEL_INF);

/* Global mutex for SD card access */
static struct k_mutex sd_card_mutex;
static bool is_initialized = false;

void sd_card_manager_init(void)
{
    LOG_DBG("Initializing SD card access manager");
    if (!is_initialized)
    {
        k_mutex_init(&sd_card_mutex);
        is_initialized = true;
        LOG_INF("SD card access manager initialized");
    }
}

int sd_card_manager_lock(uint32_t timeout_ms)
{
    if (!is_initialized)
    {
        LOG_ERR("SD card manager not initialized");
        return -EINVAL;
    }

    int ret = k_mutex_lock(&sd_card_mutex, K_MSEC(timeout_ms));
    if (ret == 0)
    {
        LOG_DBG("SD card access acquired");
    }
    else
    {
        LOG_WRN("Failed to acquire SD card access: %d", ret);
    }
    return ret;
}

void sd_card_manager_unlock(void)
{
    if (!is_initialized)
    {
        LOG_ERR("SD card manager not initialized");
        return;
    }

    k_mutex_unlock(&sd_card_mutex);
    LOG_DBG("SD card access released");
}

int sd_card_safe_open(struct fs_file_t *zfp, const char *file_name, fs_mode_t flags)
{
    int ret;

    ret = sd_card_manager_lock(5000); /* 5 second timeout */
    if (ret != 0)
    {
        LOG_ERR("Failed to acquire SD card lock for open: %d", ret);
        return ret;
    }

    ret = fs_open(zfp, file_name, flags);
    if (ret != 0)
    {
        LOG_ERR("Failed to open file %s: %d", file_name, ret);
        sd_card_manager_unlock();
        return ret;
    }

    LOG_DBG("File opened safely: %s", file_name);
    return 0;
}

int sd_card_safe_close(struct fs_file_t *zfp)
{
    int ret = fs_close(zfp);
    sd_card_manager_unlock();

    if (ret != 0)
    {
        LOG_ERR("Failed to close file: %d", ret);
    }
    else
    {
        LOG_DBG("File closed safely");
    }

    return ret;
}

ssize_t sd_card_safe_read(struct fs_file_t *zfp, void *ptr, size_t size)
{
    return fs_read(zfp, ptr, size);
}

ssize_t sd_card_safe_write(struct fs_file_t *zfp, const void *ptr, size_t size)
{
    return fs_write(zfp, ptr, size);
}

int sd_card_safe_sync(struct fs_file_t *zfp)
{
    return fs_sync(zfp);
}
