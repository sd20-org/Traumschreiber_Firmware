/*
 * Battery Monitor - Header
 *
 * Monitors battery voltage and updates BLE Battery Service
 *
 * @author ISDT
 * @date 2025
 */

#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize battery monitor
     *
     * Starts periodic battery level monitoring and BLE updates.
     *
     * @return 0 on success, negative errno on failure
     */
    int battery_monitor_init(void);

    /**
     * @brief Get current battery voltage in millivolts
     *
     * @return Battery voltage in mV
     */
    uint32_t battery_monitor_get_voltage_mv(void);

    /**
     * @brief Get current battery level in percent
     *
     * @return Battery level (0-100%)
     */
    uint8_t battery_monitor_get_level_percent(void);

    /**
     * @brief Manually trigger a battery level update
     *
     * Reads battery voltage and updates BLE Battery Service.
     *
     * @return 0 on success, negative errno on failure
     */
    int battery_monitor_update(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_MONITOR_H_ */
