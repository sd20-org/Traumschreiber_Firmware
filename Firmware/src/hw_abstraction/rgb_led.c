/**
 * @file rgb_led.c
 * @brief RGB LED Hardware Abstraction Layer Implementation
 *
 * Provides low-level PWM control for the XL-1615RGBC-RF RGB LED
 * on the Traumschreiber2509 hardware.
 *
 * @author ISDT
 * @date 2025
 */

#include "hw_abstraction/rgb_led.h"
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
/* Notify BLE clients of LED state changes when available */
#include "../ble/lbs_service.h"

LOG_MODULE_REGISTER(rgb_led, LOG_LEVEL_INF);

/* PWM Configuration */
#define PWM_PERIOD_US 20 /* 20 µs = 50 kHz PWM frequency */

/* PWM Channels (matching device tree configuration) */
#define PWM_CHANNEL_RED 0   /* PWM OUT0 - Red LED on P1.00 */
#define PWM_CHANNEL_GREEN 2 /* PWM OUT2 - Green LED on P0.24 (swapped) */
#define PWM_CHANNEL_BLUE 1  /* PWM OUT1 - Blue LED on P0.13 (swapped) */

/* Static variables */
static const struct device *pwm_dev = NULL;
static bool is_initialized = false;

/* Predefined colors */
const rgb_color_t RGB_COLOR_OFF = {0, 0, 0};
const rgb_color_t RGB_COLOR_RED = {100, 0, 0};
const rgb_color_t RGB_COLOR_GREEN = {0, 100, 0};
const rgb_color_t RGB_COLOR_BLUE = {0, 0, 100};
const rgb_color_t RGB_COLOR_YELLOW = {100, 100, 0};
const rgb_color_t RGB_COLOR_MAGENTA = {100, 0, 100};
const rgb_color_t RGB_COLOR_CYAN = {0, 100, 100};
const rgb_color_t RGB_COLOR_WHITE = {100, 100, 100};

int rgb_led_init(void)
{
    if (is_initialized)
    {
        return 0;
    }

    LOG_INF("Initializing RGB LED hardware abstraction layer...");

    /* Try to find PWM device with various possible names */
    const char *pwm_names[] = {
        "PWM_0", "pwm0", "PWM0", "pwm@4001c000", "4001c000.pwm"};

    for (int i = 0; i < ARRAY_SIZE(pwm_names); i++)
    {
        LOG_DBG("Trying PWM device name: %s", pwm_names[i]);
        pwm_dev = device_get_binding(pwm_names[i]);
        if (pwm_dev != NULL)
        {
            LOG_INF("Found PWM device: %s", pwm_names[i]);
            if (device_is_ready(pwm_dev))
            {
                LOG_INF("PWM device is ready");
                is_initialized = true;

                /* Turn off all LEDs initially */
                rgb_led_off();

                return 0;
            }
            else
            {
                LOG_WRN("PWM device found but not ready: %s", pwm_names[i]);
            }
        }
    }

    LOG_ERR("Could not initialize PWM device for RGB LED");
    return -ENODEV;
}

int rgb_led_set_color(const rgb_color_t *color)
{
    if (!is_initialized || pwm_dev == NULL)
    {
        LOG_ERR("RGB LED not initialized");
        return -ENODEV;
    }

    if (color == NULL)
    {
        LOG_ERR("Invalid color pointer");
        return -EINVAL;
    }

    /* Use the color provided by callers; do not force a channel swap
     * or disable the green channel here. PWM channel mapping is handled
     * by the PWM_CHANNEL_* defines above.
     */
    rgb_color_t mod_color = *color;

    int ret;
    uint32_t period_ns = PWM_PERIOD_US * 1000; /* Convert to nanoseconds */

    /* Set red LED PWM (inverted logic for active-low LEDs) */
    uint32_t red_duty = period_ns * (100 - mod_color.red) / 100;
    ret = pwm_set(pwm_dev, PWM_CHANNEL_RED, period_ns, red_duty, 0);
    if (ret < 0)
    {
        LOG_ERR("Failed to set red LED PWM: %d", ret);
        return ret;
    }

    /* Set green LED PWM (inverted logic for active-low LEDs) */
    uint32_t green_duty = period_ns * (100 - mod_color.green) / 100;
    ret = pwm_set(pwm_dev, PWM_CHANNEL_GREEN, period_ns, green_duty, 0);
    if (ret < 0)
    {
        LOG_ERR("Failed to set green LED PWM: %d", ret);
        return ret;
    }

    /* Set blue LED PWM (inverted logic for active-low LEDs) */
    uint32_t blue_duty = period_ns * (100 - mod_color.blue) / 100;
    ret = pwm_set(pwm_dev, PWM_CHANNEL_BLUE, period_ns, blue_duty, 0);
    if (ret < 0)
    {
        LOG_ERR("Failed to set blue LED PWM: %d", ret);
        return ret;
    }

    LOG_DBG("RGB LED set to: R=%d%%, G=%d%%, B=%d%%",
            mod_color.red, mod_color.green, mod_color.blue);

    /* Notify BLE clients about LED RGB state (convert 0-100 -> 0-255) */
    uint8_t r8 = (uint8_t)((mod_color.red * 255) / 100);
    uint8_t g8 = (uint8_t)((mod_color.green * 255) / 100);
    uint8_t b8 = (uint8_t)((mod_color.blue * 255) / 100);
    (void)lbs_send_led_state_rgb(r8, g8, b8);

    return 0;
}

int rgb_led_off(void)
{
    return rgb_led_set_color(&RGB_COLOR_OFF);
}

int rgb_led_set_blue_raw8(uint8_t level)
{
    if (pwm_dev == NULL)
    {
        return -ENODEV;
    }
    uint32_t period_ns = PWM_PERIOD_US * 1000;
    /* Active-low LED: level 255 => fully on (pulse 0), level 0 => off (pulse = period).
     * Finer than the 0-100 rgb_led_set_color path — used by the HW self-test to
     * characterise the brightness sensor across the full 8-bit range. */
    uint32_t blue_duty = (uint32_t)(((uint64_t)period_ns * (uint32_t)(255U - (uint32_t)level)) / 255U);
    (void)pwm_set(pwm_dev, PWM_CHANNEL_RED, period_ns, period_ns, 0);   /* red off  */
    (void)pwm_set(pwm_dev, PWM_CHANNEL_GREEN, period_ns, period_ns, 0); /* green off */
    return pwm_set(pwm_dev, PWM_CHANNEL_BLUE, period_ns, blue_duty, 0);
}

bool rgb_led_is_initialized(void)
{
    return is_initialized;
}
