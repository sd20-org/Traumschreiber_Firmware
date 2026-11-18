/*
 * AIN BLE Service - Header
 *
 * Streams Nordic SAADC AIN0-AIN7 raw ADC values periodically (notifications)
 * and supports a read for the latest sample.
 *
 * @author ISDT
 * @date 2025
 */

#ifndef AIN_SERVICE_H_
#define AIN_SERVICE_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* AIN Service UUID: 0483DDDE-6C9D-6CA9-5D41-03AD-4FFF4ABB (custom UUID) */
#define BT_UUID_AIN_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x0483DDDE, 0x6C9D, 0x6CA9, 0x5D41, 0x03AD4FFF4ABB)

#define BT_UUID_AIN_SERVICE BT_UUID_DECLARE_128(BT_UUID_AIN_SERVICE_VAL)

/* AIN Data Characteristic UUID */
#define BT_UUID_AIN_DATA_VAL \
    BT_UUID_128_ENCODE(0x0483DDDF, 0x6C9D, 0x6CA9, 0x5D41, 0x03AD4FFF4ABB)

#define BT_UUID_AIN_DATA BT_UUID_DECLARE_128(BT_UUID_AIN_DATA_VAL)

    /**
     * @brief Initialize AIN BLE service
     *
     * Sets up BLE characteristic, CCC, and periodic sampling/notifications.
     * @return 0 on success, negative errno on failure
     */
    int ain_service_init(void);

#ifdef __cplusplus
}
#endif

#endif /* AIN_SERVICE_H_ */