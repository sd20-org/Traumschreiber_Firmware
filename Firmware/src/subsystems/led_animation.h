/* LED Animation Subsystem - extended API with oneshot and hold */
#ifndef LED_ANIMATION_H_
#define LED_ANIMATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "hw_abstraction/rgb_led.h"
#include "hw_abstraction/button.h"

typedef void (*led_oneshot_complete_cb_t)(button_event_t event, void *ctx);

void led_animation_init(void);
void led_animation_play_startup(void);
void led_animation_start_advertising(void);
/* Call after the boot sound: green blinks twice, 1 s dark, then the blue
 * advertising pulse (if advertising was started meanwhile). */
void led_animation_boot_done(void);
void led_animation_on_connected(void);
void led_animation_on_disconnected(void);

/**
 * Play a short oneshot pattern (non-blocking). Returns true if queued.
 */
bool led_animation_play_oneshot(const rgb_color_t *color,
                                uint8_t pulses,
                                uint32_t on_ms,
                                uint32_t off_ms,
                                button_event_t event,
                                led_oneshot_complete_cb_t on_complete,
                                void *ctx);

/**
 * Hold the LED at a static colour until led_animation_hold_stop() is called.
 */
void led_animation_hold_start(const rgb_color_t *color);
void led_animation_hold_stop(void);
/* Stop the animation thread for good; the caller then owns the LED. */
void led_animation_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_ANIMATION_H_ */
