/*
 * LED Button Service (LBS) - Header
 * Based on Nordic Semiconductor's peripheral_lbs sample
 *
 * This service provides:
 * - LED control via BLE write (from smartphone to device)
 * - Button state notifications (from device to smartphone)
 *
 * @author ISDT
 * @date 2025
 */

#ifndef LBS_SERVICE_H_
#define LBS_SERVICE_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief LBS Service UUID: 00001523-1212-efde-1523-785feabcd123
 */
#define BT_UUID_LBS_VAL \
    BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS BT_UUID_DECLARE_128(BT_UUID_LBS_VAL)

/**
 * @brief Button Characteristic UUID: 00001524-1212-efde-1523-785feabcd123
 */
#define BT_UUID_LBS_BUTTON_VAL \
    BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_BUTTON BT_UUID_DECLARE_128(BT_UUID_LBS_BUTTON_VAL)

/**
 * @brief LED Characteristic UUID: 00001525-1212-efde-1523-785feabcd123
 */
#define BT_UUID_LBS_LED_VAL \
    BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_LED BT_UUID_DECLARE_128(BT_UUID_LBS_LED_VAL)

/**
 * @brief RGB LED Characteristic UUID (custom extension): 00001526-1212-efde-1523-785feabcd123
 *
 * Format: 3 bytes [R, G, B] each 0-255
 */
#define BT_UUID_LBS_RGB_LED_VAL \
    BT_UUID_128_ENCODE(0x00001526, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_RGB_LED BT_UUID_DECLARE_128(BT_UUID_LBS_RGB_LED_VAL)

/**
 * @brief LED STATE Characteristic UUID: 00001527-1212-efde-1523-785feabcd123
 *
 * Format: 3 bytes: [R, G, B] each 0-255
 */
#define BT_UUID_LBS_LED_STATE_VAL \
    BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_LED_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_LED_STATE_VAL)

/**
 * @brief LED STATE REQUEST Characteristic UUID: 00001528-1212-efde-1523-785feabcd123
 *
 * Write-only characteristic. When a connected client writes to this
 * characteristic (any payload), the device will send a LED_STATE notify
 * with the current 3-byte RGB color. This lets a client "ask" the device
 * to emit a notification on demand.
 */
#define BT_UUID_LBS_LED_STATE_REQ_VAL \
    BT_UUID_128_ENCODE(0x00001528, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_LED_STATE_REQ BT_UUID_DECLARE_128(BT_UUID_LBS_LED_STATE_REQ_VAL)

    /**
     * @brief Callback function for LED state change
     *
     * @param led_state true = LED on, false = LED off
     */
    typedef void (*lbs_led_cb_t)(bool led_state);

    /**
     * @brief Callback function for RGB LED color change
     *
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     */
    typedef void (*lbs_rgb_led_cb_t)(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Callback function to get current button state
     *
     * @return true if button is pressed, false otherwise
     */
    typedef bool (*lbs_button_cb_t)(void);

    /* Playback callback types (file play request and control opcode) */
    typedef void (*lbs_play_file_cb_t)(const char *path);
    typedef void (*lbs_playback_control_cb_t)(uint8_t opcode);

    /**
     * @brief LBS service callbacks
     */
    struct lbs_cb
    {
        lbs_led_cb_t led_cb;         /**< LED state change callback */
        lbs_rgb_led_cb_t rgb_led_cb; /**< RGB LED color change callback */
        lbs_button_cb_t button_cb;   /**< Button state query callback */

        /* Playback callbacks */
        lbs_play_file_cb_t play_file_cb;            /**< Play file request (path) */
        lbs_playback_control_cb_t playback_ctrl_cb; /**< Playback control opcodes */
    };

    /**
     * @brief Initialize the LBS service
     *
     * @param callbacks Callback structure for LED and button events
     * @return 0 on success, negative errno on failure
     */
    int lbs_init(struct lbs_cb *callbacks);

    /**
     * @brief Send button state notification to connected client
     *
     * This function sends a notification to the connected smartphone
     * when the button state changes.
     *
     * @param button_state true if button is pressed, false otherwise
     * @return 0 on success, negative errno on failure
     */
    int lbs_send_button_state(bool button_state);

    /**
     * @brief Send LED state notification to connected client
     *
     * Called by firmware when the on-device LED state changes so that a
     * subscribed client receives an immediate notification. The characteristic
     * contains a single byte: 0 = off, 1 = on.
     *
     * @param led_state true if LED is on, false if off
     * @return 0 on success, negative errno on failure
     */
    /**
     * @brief Send LED RGB state notification to connected client
     *
     * Sends a 3-byte payload [R,G,B] (0-255) representing the current LED
     * color. Called by firmware when the on-device LED changes so subscribed
     * clients receive updated color information.
     *
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @return 0 on success, negative errno on failure
     */
    int lbs_send_led_state_rgb(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Check if notifications are enabled
     *
     * @return true if a client is connected and notifications are enabled
     */
    bool lbs_notify_enabled(void);

/* --------------------------------------------------------------------- */
/* Playback (WAV) control over BLE */

/**
 * @brief Playback control characteristic UUID: 00001529-1212-efde-1523-785feabcd123
 *
 * Write-only characteristic that accepts a UTF-8 file path (max 64 bytes)
 * relative to the SD root (e.g., "SOUNDS/BELL1.WAV"). Writing triggers
 * playback of the specified file.
 */
#define BT_UUID_LBS_PLAY_FILE_VAL \
    BT_UUID_128_ENCODE(0x00001529, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_PLAY_FILE BT_UUID_DECLARE_128(BT_UUID_LBS_PLAY_FILE_VAL)

/**
 * @brief Playback control characteristic UUID: 0000152A-1212-efde-1523-785feabcd123
 *
 * Write-only characteristic for simple control opcodes (1 byte):
 * 0 = STOP, 1 = PAUSE (not implemented), 2 = RESUME (not implemented)
 */
#define BT_UUID_LBS_PLAYBACK_CONTROL_VAL \
    BT_UUID_128_ENCODE(0x0000152A, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_PLAYBACK_CONTROL BT_UUID_DECLARE_128(BT_UUID_LBS_PLAYBACK_CONTROL_VAL)

/**
 * @brief Playback state characteristic UUID: 0000152B-1212-efde-1523-785feabcd123
 *
 * Read + Notify. Single byte values: 0 = STOPPED, 1 = PLAYING, 2 = PAUSED, 3 = ERROR
 */
#define BT_UUID_LBS_PLAYBACK_STATE_VAL \
    BT_UUID_128_ENCODE(0x0000152B, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_PLAYBACK_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_PLAYBACK_STATE_VAL)

/**
 * @brief Playback STATE REQUEST Characteristic UUID: 0000152C-1212-efde-1523-785feabcd123
 *
 * Write-only characteristic. Writing any payload requests that the device
 * send the current `PLAYBACK_STATE` as a notification (if enabled).
 */
#define BT_UUID_LBS_PLAYBACK_STATE_REQ_VAL \
    BT_UUID_128_ENCODE(0x0000152C, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_PLAYBACK_STATE_REQ BT_UUID_DECLARE_128(BT_UUID_LBS_PLAYBACK_STATE_REQ_VAL)

/* Playback state values */
#define LBS_PLAYBACK_STATE_STOPPED 0
#define LBS_PLAYBACK_STATE_PLAYING 1
#define LBS_PLAYBACK_STATE_PAUSED 2
#define LBS_PLAYBACK_STATE_ERROR 3

    /**
     * @brief Send playback state notification to connected client
     *
     * @param state One of LBS_PLAYBACK_STATE_* values
     * @return 0 on success, negative errno on failure
     */
    int lbs_send_playback_state(uint8_t state);

/* --------------------------------------------------------------------- */
/* Sound listing (SOUNDS/ directory) */

/**
 * @brief Sound list request Characteristic UUID: 0000152D-1212-efde-1523-785feabcd123
 *
 * Write-only characteristic: writing any payload triggers the device to
 * enumerate WAV files in the `SOUNDS/` directory and send a sequence of
 * `SOUND_ENTRY` notifications (one per filename). After all names are
 * sent a final entry `[END]` will be notified.
 */
#define BT_UUID_LBS_SOUND_LIST_REQ_VAL \
    BT_UUID_128_ENCODE(0x0000152D, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_SOUND_LIST_REQ BT_UUID_DECLARE_128(BT_UUID_LBS_SOUND_LIST_REQ_VAL)

/**
 * @brief Sound entry Characteristic UUID: 0000152E-1212-efde-1523-785feabcd123
 *
 * Notify/Read characteristic. Each notification contains a single UTF-8
 * filename (e.g., "BELL1.WAV"), final `[END]` entry indicates end-of-list.
 */
#define BT_UUID_LBS_SOUND_ENTRY_VAL \
    BT_UUID_128_ENCODE(0x0000152E, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_SOUND_ENTRY BT_UUID_DECLARE_128(BT_UUID_LBS_SOUND_ENTRY_VAL)

    /**
     * @brief Send a single filename as SOUND_ENTRY notification
     *
     * @param name NULL-terminated filename (max ~64 bytes) or "[END]" to signal completion
     * @return 0 on success, negative errno on failure
     */
    int lbs_send_sound_entry(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* LBS_SERVICE_H_ */
