/*
 * Traumschreiber2509 Button Driver
 *
 * Professional button interface with single/double click detection
 * Button is connected to P1.06 (GPIO_ACTIVE_LOW with pull-up)
 *
 * @author ISDT
 * @date 2025
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Button event types */
    typedef enum
    {
        BUTTON_EVENT_NONE = 0,
        BUTTON_EVENT_SINGLE_CLICK,
        BUTTON_EVENT_DOUBLE_CLICK,
        BUTTON_EVENT_TRIPLE_CLICK,
        BUTTON_EVENT_LONG_PRESS,
        BUTTON_EVENT_EXTRA_LONG_PRESS,
        BUTTON_EVENT_SHORT_SHORT_LONG,
        BUTTON_EVENT_SHORT_LONG,
        BUTTON_EVENT_LONG_SHORT,
        BUTTON_EVENT_SHORT_LONG_SHORT,
        BUTTON_EVENT_LONG_SHORT_SHORT,
        BUTTON_EVENT_PRESSED,
        BUTTON_EVENT_RELEASED,
        BUTTON_EVENT_COUNT
    } button_event_t;

    /* Button callback function type */
    typedef void (*button_callback_t)(button_event_t event);

    /* Button configuration structure */
    typedef struct
    {
        button_callback_t callback;
        uint32_t multi_click_timeout_ms;  /* Time window between gesture tokens (default: 500ms) */
        uint32_t debounce_time_ms;        /* Debounce time (default: 50ms) */
        uint32_t long_press_ms;           /* Threshold for a long token (default: 700ms) */
        uint32_t extra_long_press_ms;     /* Threshold for extra long press (default: 2000ms) */
    } button_config_t;

    /**
     * @brief Initialize the button driver
     *
     * @param config Button configuration structure, can be NULL for defaults
     * @return 0 on success, negative error code on failure
     */
    int button_init(const button_config_t *config);

    /**
     * @brief Get current button state
     *
     * @return true if button is pressed, false if released
     */
    bool button_is_pressed(void);

    /**
     * @brief Enable/disable button interrupt
     *
     * @param enable true to enable, false to disable
     * @return 0 on success, negative error code on failure
     */
    int button_set_interrupt_enabled(bool enable);

    /**
     * @brief Register a secondary callback for button events
     *
     * This allows multiple subsystems to receive button events.
     * The secondary callback is called after the primary callback.
     *
     * @param callback Secondary callback function
     * @return 0 on success, negative error code on failure
     */
    int button_register_secondary_callback(button_callback_t callback);

    /**
     * @brief Convert a button event to a stable config/debug string
     */
    const char *button_event_to_string(button_event_t event);

    /**
     * @brief Parse a button event name from config/debug text
     *
     * Accepts case-insensitive names such as SINGLE_CLICK, SHORT_LONG,
     * EXTRA_LONG_PRESS, etc.
     *
     * @param name Input name
     * @param out_event Output event on success
     * @return true on success, false otherwise
     */
    bool button_event_from_name(const char *name, button_event_t *out_event);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */
