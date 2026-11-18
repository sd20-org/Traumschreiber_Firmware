/*
 * Traumschreiber2509 Button Driver
 *
 * Professional button interface with configurable gesture detection
 * Button is connected to P1.06 (GPIO_ACTIVE_LOW with pull-up)
 *
 * @author ISDT
 * @date 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <strings.h>
#include "button.h"

LOG_MODULE_REGISTER(button, LOG_LEVEL_INF);

/* Button hardware configuration from devicetree */
#define BUTTON_NODE DT_ALIAS(sw0)

#if !DT_NODE_HAS_STATUS_OKAY(BUTTON_NODE)
#error "Button devicetree alias 'sw0' is not defined or enabled"
#endif

static const struct gpio_dt_spec button_spec = GPIO_DT_SPEC_GET_OR(BUTTON_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

/* Button state management */
static struct
{
    button_callback_t callback;
    button_callback_t secondary_callback; /* For BLE and other subsystems */
    uint32_t multi_click_timeout_ms;
    uint32_t debounce_time_ms;
    uint32_t long_press_ms;
    uint32_t extra_long_press_ms;

    /* Internal state */
    bool is_pressed;
    uint32_t last_press_time;
    uint8_t token_count;
    bool tokens[3]; /* false=short, true=long */

    /* Work items for delayed processing */
    struct k_work_delayable debounce_work;
    struct k_work_delayable gesture_timeout_work;
    struct k_work_delayable long_press_work;
    struct k_work_delayable extra_long_work;
    bool long_emitted;
    bool extra_long_emitted;
} button_state = {
    .callback = NULL,
    .secondary_callback = NULL,
    .multi_click_timeout_ms = 500,
    .debounce_time_ms = 50,
    .long_press_ms = 700,
    .extra_long_press_ms = 2000,
    .is_pressed = false,
    .last_press_time = 0,
    .token_count = 0};

/* Forward declarations */
static void button_debounce_work_handler(struct k_work *work);
static void button_gesture_timeout_work_handler(struct k_work *work);
static void button_long_press_work_handler(struct k_work *work);
static void button_extra_long_press_work_handler(struct k_work *work);

static void button_emit_event(button_event_t event)
{
    if (button_state.callback)
    {
        button_state.callback(event);
    }
    if (button_state.secondary_callback)
    {
        button_state.secondary_callback(event);
    }
}

static void button_reset_tokens(void)
{
    button_state.token_count = 0;
    memset(button_state.tokens, 0, sizeof(button_state.tokens));
}

static bool button_classify_tokens(button_event_t *out_event)
{
    const uint8_t count = button_state.token_count;
    const bool *tokens = button_state.tokens;

    if (!out_event || count == 0)
    {
        return false;
    }

    if (count == 1)
    {
        *out_event = tokens[0] ? BUTTON_EVENT_LONG_PRESS : BUTTON_EVENT_SINGLE_CLICK;
        return true;
    }

    if (count == 2)
    {
        if (!tokens[0] && !tokens[1])
        {
            *out_event = BUTTON_EVENT_DOUBLE_CLICK;
            return true;
        }
        if (!tokens[0] && tokens[1])
        {
            *out_event = BUTTON_EVENT_SHORT_LONG;
            return true;
        }
        if (tokens[0] && !tokens[1])
        {
            *out_event = BUTTON_EVENT_LONG_SHORT;
            return true;
        }

        return false;
    }

    if (count == 3)
    {
        if (!tokens[0] && !tokens[1] && !tokens[2])
        {
            *out_event = BUTTON_EVENT_TRIPLE_CLICK;
            return true;
        }
        if (!tokens[0] && !tokens[1] && tokens[2])
        {
            *out_event = BUTTON_EVENT_SHORT_SHORT_LONG;
            return true;
        }
        if (!tokens[0] && tokens[1] && !tokens[2])
        {
            *out_event = BUTTON_EVENT_SHORT_LONG_SHORT;
            return true;
        }
        if (tokens[0] && !tokens[1] && !tokens[2])
        {
            *out_event = BUTTON_EVENT_LONG_SHORT_SHORT;
            return true;
        }
    }

    return false;
}

static void button_finalize_tokens(void)
{
    button_event_t event = BUTTON_EVENT_NONE;
    uint8_t count = button_state.token_count;

    if (count == 0)
    {
        return;
    }

    LOG_DBG("Finalizing %u token(s)", (unsigned)count);
    for (uint8_t i = 0; i < count; i++) {
        LOG_DBG(" token[%u]=%s", (unsigned)i, button_state.tokens[i] ? "LONG" : "SHORT");
    }

    if (button_classify_tokens(&event))
    {
        LOG_INF("Button gesture detected: %s", button_event_to_string(event));
        button_emit_event(event);
    }
    else
    {
        LOG_WRN("Unsupported button gesture with %u tokens", count);
    }

    button_reset_tokens();
}

static void button_append_token(bool is_long)
{
    if (button_state.token_count >= ARRAY_SIZE(button_state.tokens))
    {
        button_finalize_tokens();
    }

    button_state.tokens[button_state.token_count++] = is_long;
    LOG_DBG("Token appended: idx=%u is_long=%d", (unsigned)(button_state.token_count - 1), is_long);

    if (button_state.token_count >= ARRAY_SIZE(button_state.tokens))
    {
        button_finalize_tokens();
        return;
    }

    /* Schedule the multi-click finalization window. Rescheduling here is
     * intentional: subsequent tokens will push the timeout out. */
    k_work_reschedule(&button_state.gesture_timeout_work,
                      K_MSEC(button_state.multi_click_timeout_ms));
}

/* Work handlers */
static void button_debounce_work_handler(struct k_work *work)
{
    bool current_state = !gpio_pin_get_dt(&button_spec); /* Active low */
    uint32_t current_time = k_uptime_get_32();

    LOG_DBG("Debounce check: current_state=%u last_known=%u time=%u last_press=%u",
            (unsigned)current_state, (unsigned)button_state.is_pressed,
            (unsigned)current_time, (unsigned)button_state.last_press_time);

    if (current_state != button_state.is_pressed)
    {
        /* Update debounced state once; don't reschedule here - the GPIO
         * interrupt will schedule further checks when bouncing occurs. */
        button_state.is_pressed = current_state;

        if (current_state)
        {
            button_state.last_press_time = current_time;
            LOG_DBG("Button pressed (debounced)");
            /* Reset any hold flags for this new press */
            button_state.long_emitted = false;
            button_state.extra_long_emitted = false;
            /* Cancel pending gesture finalizer in case of weird state */
            k_work_cancel_delayable(&button_state.gesture_timeout_work);
            /* Schedule hold-detection works */
            k_work_schedule(&button_state.long_press_work, K_MSEC(button_state.long_press_ms));
            k_work_schedule(&button_state.extra_long_work, K_MSEC(button_state.extra_long_press_ms));
            button_emit_event(BUTTON_EVENT_PRESSED);
        }
        else
        {
            uint32_t press_duration_ms = current_time - button_state.last_press_time;
            LOG_DBG("Button released (debounced) duration=%u ms", (unsigned)press_duration_ms);
            /* Cancel hold detectors */
            k_work_cancel_delayable(&button_state.long_press_work);
            k_work_cancel_delayable(&button_state.extra_long_work);
            button_emit_event(BUTTON_EVENT_RELEASED);

            /* If a hold event was already emitted while the button was held,
             * ignore the release-based classification to avoid duplicate actions. */
            if (button_state.extra_long_emitted)
            {
                LOG_DBG("Extra-long already emitted on hold; ignoring release classification");
                button_state.extra_long_emitted = false;
                button_state.long_emitted = false;
                return;
            }

            if (button_state.long_emitted)
            {
                LOG_DBG("Long already emitted on hold; ignoring release classification");
                button_state.long_emitted = false;
                return;
            }

            if (press_duration_ms >= button_state.extra_long_press_ms)
            {
                k_work_cancel_delayable(&button_state.gesture_timeout_work);
                button_reset_tokens();
                LOG_INF("Button gesture detected: %s", button_event_to_string(BUTTON_EVENT_EXTRA_LONG_PRESS));
                button_emit_event(BUTTON_EVENT_EXTRA_LONG_PRESS);
            }
            else
            {
                const bool is_long = (press_duration_ms >= button_state.long_press_ms);
                LOG_DBG("Appending token (is_long=%d)", is_long);
                button_append_token(is_long);
            }
        }
    }
}

static void button_gesture_timeout_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    button_finalize_tokens();
}

/* Long-press work handler: emits BUTTON_EVENT_LONG_PRESS while the button is still held.
 * This allows immediate long-press behavior without waiting for release. */
static void button_long_press_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!button_state.is_pressed)
        return;
    /* Mark and emit once */
    button_state.long_emitted = true;
    /* Clear any accumulated tokens since we're handling the long press now */
    button_reset_tokens();
    LOG_INF("Button gesture detected (hold): %s", button_event_to_string(BUTTON_EVENT_LONG_PRESS));
    button_emit_event(BUTTON_EVENT_LONG_PRESS);
}

/* Extra-long-press handler: emits EXTRA_LONG_PRESS while still held. */
static void button_extra_long_press_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!button_state.is_pressed)
        return;
    button_state.extra_long_emitted = true;
    /* Cancel any pending multi-click timeout and clear tokens */
    k_work_cancel_delayable(&button_state.gesture_timeout_work);
    button_reset_tokens();
    LOG_INF("Button gesture detected (hold): %s", button_event_to_string(BUTTON_EVENT_EXTRA_LONG_PRESS));
    button_emit_event(BUTTON_EVENT_EXTRA_LONG_PRESS);
}

/* GPIO interrupt callback */
static void button_gpio_callback(const struct device *dev,
                                 struct gpio_callback *cb,
                                 uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_work_schedule(&button_state.debounce_work,
                    K_MSEC(button_state.debounce_time_ms));
}

/* Public API implementation */
int button_init(const button_config_t *config)
{
    int ret;

    LOG_INF("Initializing Traumschreiber2509 button driver");

    /* Apply configuration */
    if (config)
    {
        button_state.callback = config->callback;
        if (config->multi_click_timeout_ms > 0)
        {
            button_state.multi_click_timeout_ms = config->multi_click_timeout_ms;
        }
        if (config->debounce_time_ms > 0)
        {
            button_state.debounce_time_ms = config->debounce_time_ms;
        }
        if (config->long_press_ms > 0)
        {
            button_state.long_press_ms = config->long_press_ms;
        }
        if (config->extra_long_press_ms > 0)
        {
            button_state.extra_long_press_ms = config->extra_long_press_ms;
        }
    }

    /* Check if button GPIO is ready */
    if (!gpio_is_ready_dt(&button_spec))
    {
        LOG_ERR("Button GPIO device %s is not ready", button_spec.port->name);
        return -ENODEV;
    }

    /* Configure button pin as input with pull-up (active low) */
    ret = gpio_pin_configure_dt(&button_spec, GPIO_INPUT | GPIO_PULL_UP);
    if (ret != 0)
    {
        LOG_ERR("Failed to configure button pin %d: %d", button_spec.pin, ret);
        return ret;
    }

    /* Configure interrupt on both edges */
    ret = gpio_pin_interrupt_configure_dt(&button_spec, GPIO_INT_EDGE_BOTH);
    if (ret != 0)
    {
        LOG_ERR("Failed to configure button interrupt: %d", ret);
        return ret;
    }

    /* Initialize callback */
    gpio_init_callback(&button_cb_data, button_gpio_callback, BIT(button_spec.pin));
    gpio_add_callback(button_spec.port, &button_cb_data);

    /* Initialize work items */
    k_work_init_delayable(&button_state.debounce_work, button_debounce_work_handler);
    k_work_init_delayable(&button_state.gesture_timeout_work, button_gesture_timeout_work_handler);
    k_work_init_delayable(&button_state.long_press_work, button_long_press_work_handler);
    k_work_init_delayable(&button_state.extra_long_work, button_extra_long_press_work_handler);
    button_state.long_emitted = false;
    button_state.extra_long_emitted = false;

    /* Read initial state */
    button_state.is_pressed = !gpio_pin_get_dt(&button_spec); /* Active low */

    LOG_INF("Button driver initialized successfully on %s pin %d",
            button_spec.port->name, button_spec.pin);
        LOG_INF("Gesture timeout: %u ms, Debounce: %u ms",
            button_state.multi_click_timeout_ms, button_state.debounce_time_ms);
        LOG_INF("Long thresholds: %u ms / %u ms",
            button_state.long_press_ms, button_state.extra_long_press_ms);
    LOG_INF("Initial button state: %s", button_state.is_pressed ? "PRESSED" : "RELEASED");

    return 0;
}

bool button_is_pressed(void)
{
    return button_state.is_pressed;
}

int button_set_interrupt_enabled(bool enable)
{
    int ret;

    if (enable)
    {
        ret = gpio_pin_interrupt_configure_dt(&button_spec, GPIO_INT_EDGE_BOTH);
    }
    else
    {
        ret = gpio_pin_interrupt_configure_dt(&button_spec, GPIO_INT_DISABLE);
    }

    if (ret != 0)
    {
        LOG_ERR("Failed to %s button interrupt: %d", enable ? "enable" : "disable", ret);
        return ret;
    }

    LOG_DBG("Button interrupt %s", enable ? "enabled" : "disabled");
    return 0;
}

int button_register_secondary_callback(button_callback_t callback)
{
    if (callback == NULL)
    {
        LOG_ERR("Secondary callback cannot be NULL");
        return -EINVAL;
    }

    button_state.secondary_callback = callback;
    LOG_INF("Secondary button callback registered");
    return 0;
}

const char *button_event_to_string(button_event_t event)
{
    switch (event)
    {
    case BUTTON_EVENT_SINGLE_CLICK:
        return "SINGLE_CLICK";
    case BUTTON_EVENT_DOUBLE_CLICK:
        return "DOUBLE_CLICK";
    case BUTTON_EVENT_TRIPLE_CLICK:
        return "TRIPLE_CLICK";
    case BUTTON_EVENT_LONG_PRESS:
        return "LONG_PRESS";
    case BUTTON_EVENT_EXTRA_LONG_PRESS:
        return "EXTRA_LONG_PRESS";
    case BUTTON_EVENT_SHORT_SHORT_LONG:
        return "SHORT_SHORT_LONG";
    case BUTTON_EVENT_SHORT_LONG:
        return "SHORT_LONG";
    case BUTTON_EVENT_LONG_SHORT:
        return "LONG_SHORT";
    case BUTTON_EVENT_SHORT_LONG_SHORT:
        return "SHORT_LONG_SHORT";
    case BUTTON_EVENT_LONG_SHORT_SHORT:
        return "LONG_SHORT_SHORT";
    case BUTTON_EVENT_PRESSED:
        return "PRESSED";
    case BUTTON_EVENT_RELEASED:
        return "RELEASED";
    default:
        return "NONE";
    }
}

bool button_event_from_name(const char *name, button_event_t *out_event)
{
    if (!name || !out_event)
    {
        return false;
    }

    for (button_event_t event = BUTTON_EVENT_NONE; event < BUTTON_EVENT_COUNT; event++)
    {
        if (strcasecmp(name, button_event_to_string(event)) == 0)
        {
            *out_event = event;
            return true;
        }
    }

    return false;
}
