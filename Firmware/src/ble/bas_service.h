/*
 * Battery Service (BAS) - Header
 * Standard Bluetooth SIG Battery Service implementation
 *
 * @author ISDT
 * @date 2025
 */

#ifndef BAS_SERVICE_H_
#define BAS_SERVICE_H_

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize the Battery Service
     *
     * @return 0 on success, negative errno on failure
     */
    int bas_init(void);

    /**
     * @brief Update the battery level
     *
     * This will notify connected clients if notifications are enabled.
     *
     * @param level_percent Battery level in percent (0-100)
     * @return 0 on success, negative errno on failure
     */
    int bas_battery_level_update(uint8_t level_percent);

    /**
     * @brief Get current battery level
     *
     * @return Current battery level in percent (0-100)
     */
    uint8_t bas_battery_level_get(void);

#ifdef __cplusplus
}
#endif

#endif /* BAS_SERVICE_H_ */
