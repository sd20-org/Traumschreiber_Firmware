/*
 * BLE Manager - Header
 *
 * Handles Bluetooth Low Energy initialization, advertising,
 * and connection management for Traumschreiber2509
 *
 * @author ISDT
 * @date 2025
 */

#ifndef BLE_MANAGER_H_
#define BLE_MANAGER_H_

#include <zephyr/types.h>

    typedef enum
    {
        BLE_MANAGER_ADV_MODE_NORMAL = 0,
        BLE_MANAGER_ADV_MODE_WORST_CASE = 1,
    } ble_manager_adv_mode_t;

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief BLE Manager result codes
     */
    typedef enum
    {
        BLE_MANAGER_OK = 0,
        BLE_MANAGER_ERROR = -1,
        BLE_MANAGER_NOT_INITIALIZED = -2,
        BLE_MANAGER_ALREADY_INITIALIZED = -3,
    } ble_manager_result_t;

    /**
     * @brief Initialize the BLE manager
     *
     * This function:
     * - Enables the Bluetooth stack
     * - Initializes all BLE services (LBS, etc.)
     * - Starts advertising
     *
     * @return BLE_MANAGER_OK on success, error code otherwise
     */
    ble_manager_result_t ble_manager_init(void);

    /**
     * @brief Start BLE advertising
     *
     * @return BLE_MANAGER_OK on success, error code otherwise
     */
    ble_manager_result_t ble_manager_start_advertising(void);

    /**
     * @brief Stop BLE advertising
     *
     * @return BLE_MANAGER_OK on success, error code otherwise
     */
    ble_manager_result_t ble_manager_stop_advertising(void);

    /**
     * @brief Check if a device is currently connected
     *
     * @return true if connected, false otherwise
     */
    bool ble_manager_is_connected(void);

    /**
     * @brief Return the currently negotiated ATT MTU size for the active connection
     *
     * If not connected return the default ATT_MTU (23).
     *
     * @return ATT MTU in bytes
     */
    uint16_t ble_manager_get_mtu_size(void);

    ble_manager_result_t ble_manager_set_radio_enabled(bool enable);

    bool ble_manager_is_radio_enabled(void);

    ble_manager_result_t ble_manager_set_adv_mode(ble_manager_adv_mode_t mode);

    ble_manager_adv_mode_t ble_manager_get_adv_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_MANAGER_H_ */
