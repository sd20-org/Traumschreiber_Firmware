/*
 * Traumschreiber2509 - SD Card Access Manager
 *
 * Provides synchronized access to the SD card filesystem to prevent
 * corruption from concurrent reads and writes.
 *
 * @author ISDT
 * @date 2025
 */

#ifndef SD_CARD_MANAGER_H
#define SD_CARD_MANAGER_H

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize the SD card access manager
     *
     * Must be called once before any other SD card operations.
     */
    void sd_card_manager_init(void);

    /**
     * @brief Acquire exclusive access to the SD card
     *
     * This function will block until exclusive access is granted.
     * Must be called before any filesystem operations.
     *
     * @param timeout_ms Maximum time to wait for access in milliseconds
     * @return 0 on success, negative error code on timeout
     */
    int sd_card_manager_lock(uint32_t timeout_ms);

    /**
     * @brief Release exclusive access to the SD card
     *
     * Must be called after filesystem operations are complete.
     */
    void sd_card_manager_unlock(void);

    /**
     * @brief Safe wrapper for fs_open with automatic locking
     *
     * @param zfp Pointer to file descriptor
     * @param file_name File path
     * @param flags Open flags
     * @return 0 on success, negative error code on failure
     */
    int sd_card_safe_open(struct fs_file_t *zfp, const char *file_name, fs_mode_t flags);

    /**
     * @brief Safe wrapper for fs_close with automatic unlocking
     *
     * @param zfp Pointer to file descriptor
     * @return 0 on success, negative error code on failure
     */
    int sd_card_safe_close(struct fs_file_t *zfp);

    /**
     * @brief Safe wrapper for fs_read
     *
     * Note: Must be called between sd_card_safe_open and sd_card_safe_close
     *
     * @param zfp Pointer to file descriptor
     * @param ptr Buffer to read into
     * @param size Number of bytes to read
     * @return Number of bytes read, or negative error code
     */
    ssize_t sd_card_safe_read(struct fs_file_t *zfp, void *ptr, size_t size);

    /**
     * @brief Safe wrapper for fs_write
     *
     * Note: Must be called between sd_card_safe_open and sd_card_safe_close
     *
     * @param zfp Pointer to file descriptor
     * @param ptr Buffer to write from
     * @param size Number of bytes to write
     * @return Number of bytes written, or negative error code
     */
    ssize_t sd_card_safe_write(struct fs_file_t *zfp, const void *ptr, size_t size);

    /**
     * @brief Safe wrapper for fs_sync
     *
     * Note: Must be called between sd_card_safe_open and sd_card_safe_close
     *
     * @param zfp Pointer to file descriptor
     * @return 0 on success, negative error code on failure
     */
    int sd_card_safe_sync(struct fs_file_t *zfp);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_MANAGER_H */
