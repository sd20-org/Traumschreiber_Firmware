/**
 * @file rgb_led.h
 * @brief RGB LED Hardware Abstraction Layer
 *
 * Provides low-level PWM control for the XL-1615RGBC-RF RGB LED
 * on the Traumschreiber2509 hardware.
 *
 * Hardware mapping:
 * - Red LED:   P1.00 (PWM OUT0)
 * - Green LED: P0.24 (PWM OUT1)
 * - Blue LED:  P0.13 (PWM OUT2)
 *
 * @author ISDT
 * @date 2025
 */

#ifndef RGB_LED_H
#define RGB_LED_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* RGB Color structure (0-100 percentage) */
    typedef struct
    {
        uint8_t red;   /**< Red intensity (0-100%) */
        uint8_t green; /**< Green intensity (0-100%) */
        uint8_t blue;  /**< Blue intensity (0-100%) */
    } rgb_color_t;

    /* Predefined colors */
    extern const rgb_color_t RGB_COLOR_OFF;
    extern const rgb_color_t RGB_COLOR_RED;
    extern const rgb_color_t RGB_COLOR_GREEN;
    extern const rgb_color_t RGB_COLOR_BLUE;
    extern const rgb_color_t RGB_COLOR_YELLOW;
    extern const rgb_color_t RGB_COLOR_MAGENTA;
    extern const rgb_color_t RGB_COLOR_CYAN;
    extern const rgb_color_t RGB_COLOR_WHITE;

    /**
     * @brief Initialize RGB LED hardware
     *
     * Sets up PWM channels for RGB LED control.
     * Must be called before any other RGB LED functions.
     *
     * @return 0 on success, negative error code on failure
     */
    int rgb_led_init(void);

    /**
     * @brief Set RGB LED color
     *
     * @param color Pointer to RGB color structure
     * @return 0 on success, negative error code on failure
     */
    int rgb_led_set_color(const rgb_color_t *color);

    /**
     * @brief Turn off all RGB LEDs
     *
     * @return 0 on success, negative error code on failure
     */
    int rgb_led_off(void);

    /**
     * @brief Check if RGB LED hardware is initialized
     *
     * @return true if initialized, false otherwise
     */
    bool rgb_led_is_initialized(void);

    /**
     * @brief Set the BLUE LED at 8-bit brightness (0..255); red/green off.
     *
     * Finer resolution than the 0-100 rgb_led_set_color() path, for the
     * hardware self-test brightness-sensor sweep.
     *
     * @param level 0 = off .. 255 = full brightness
     * @return 0 on success, negative error code on failure
     */
    int rgb_led_set_blue_raw8(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* RGB_LED_H */
