/* LED Animation Subsystem - oneshot + hold implementation */
#include "subsystems/led_animation.h"
#include "hw_abstraction/rgb_led.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led_animation, LOG_LEVEL_INF);

#define ANIM_STACK_SIZE  1024
#define ANIM_PRIORITY    7
K_THREAD_STACK_DEFINE(anim_stack, ANIM_STACK_SIZE);
static struct k_thread anim_thread;

typedef enum { ANIM_IDLE=0, ANIM_ADVERTISING=1, ANIM_CONNECTED=2, ANIM_BOOT_DONE=3 } anim_state_t;
static volatile anim_state_t g_state = ANIM_IDLE;
static struct k_sem g_wake;
static uint8_t g_adv_step = 0;
/* Boot sequence (fw 0.0.94): green stays on from power-on until the boot sound
 * has played; the blue advertising pulse must not start before that. */
static volatile bool g_boot_phase = false;
static volatile bool g_adv_pending = false;
static volatile bool g_conn = false;
/* Set once by led_animation_shutdown(): the thread turns the LED off and exits. */
static volatile bool g_shutdown = false;

static const uint8_t k_sine_lut[32] = {
     0,  10,  19,  29,  38,  47,  56,  63,
    71,  78,  83,  88,  92,  95,  98,  99,
   100,  99,  98,  95,  92,  88,  83,  78,
    71,  63,  56,  47,  38,  29,  19,  10
};

static void advertising_step(void)
{
    uint8_t brightness = (uint8_t)(5u + ((uint32_t)k_sine_lut[g_adv_step] * 95u) / 100u);
    rgb_color_t c = {0, 0, brightness};
    rgb_led_set_color(&c);
    g_adv_step = (uint8_t)((g_adv_step + 1u) % 32u);
}

static void run_connected_animation(void)
{
    LOG_INF("LED anim: connected sequence start");
    for (int i = 0; i <= 40; i++) {
        if (g_state != ANIM_CONNECTED) return;
        uint8_t blue  = (uint8_t)(100u - (uint32_t)i * 100u / 40u);
        uint8_t green = (uint8_t)((uint32_t)i * 100u / 40u);
        rgb_color_t c = {0, green, blue};
        rgb_led_set_color(&c);
        k_sleep(K_MSEC(50));
    }
    if (g_state == ANIM_CONNECTED) {
        k_sleep(K_SECONDS(1));
    }
    if (g_state == ANIM_CONNECTED) {
        rgb_led_off();
        g_state = ANIM_IDLE;
    }
    LOG_INF("LED anim: connected sequence done");
}

/* After the boot sound: green blinks twice, 1 s dark, then the advertising
 * pulse (if advertising was requested meanwhile). A BLE connection during the
 * sequence (state -> CONNECTED) takes over. */
static void run_boot_done_animation(void)
{
    static const rgb_color_t green = {0, 100, 0};
    for (int i = 0; i < 2 && g_state == ANIM_BOOT_DONE; i++) {
        rgb_led_off();
        k_sleep(K_MSEC(150));
        rgb_led_set_color(&green);
        k_sleep(K_MSEC(150));
    }
    rgb_led_off();
    for (int i = 0; i < 10 && g_state == ANIM_BOOT_DONE; i++) {
        k_sleep(K_MSEC(100));
    }
    g_boot_phase = false;
    if (g_state == ANIM_BOOT_DONE) {
        g_adv_step = 0;
        g_state = (g_adv_pending && !g_conn) ? ANIM_ADVERTISING : ANIM_IDLE;
    }
    LOG_INF("LED anim: boot sequence done");
}

/* Oneshoot queue */
typedef struct {
    rgb_color_t color;
    uint8_t pulses;
    uint32_t on_ms;
    uint32_t off_ms;
    button_event_t event;
    led_oneshot_complete_cb_t on_complete;
    void *ctx;
} oneshot_req_t;

K_MSGQ_DEFINE(g_oneshot_msgq, sizeof(oneshot_req_t), 4, 4);

/* Hold override */
static volatile bool g_hold_active = false;
static rgb_color_t g_hold_color = {0,0,0};

static void run_oneshot(const oneshot_req_t *r)
{
    if (!r || r->pulses == 0) return;
    LOG_INF("LED anim: oneshot start (pulses=%u)", r->pulses);
    for (uint8_t i = 0; i < r->pulses; i++) {
        if (g_hold_active) {
            LOG_INF("LED anim: oneshot preempted by hold");
            break;
        }
        rgb_led_set_color(&r->color);
        k_sleep(K_MSEC(r->on_ms));
        rgb_led_off();
        if (i + 1 < r->pulses) {
            k_sleep(K_MSEC(r->off_ms));
        }
    }
    if (r->on_complete) {
        r->on_complete(r->event, r->ctx);
    }
    LOG_INF("LED anim: oneshot done");
}

static void anim_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    oneshot_req_t req;
    while (1) {
        if (g_shutdown) {
            rgb_led_off();
            return;
        }
        /* Hold has highest priority */
        if (g_hold_active) {
            rgb_led_set_color(&g_hold_color);
            (void)k_sem_take(&g_wake, K_MSEC(50));
            continue;
        }

        /* Process any pending oneshot (non-blocking) */
        if (k_msgq_get(&g_oneshot_msgq, &req, K_NO_WAIT) == 0) {
            run_oneshot(&req);
            continue;
        }

        switch (g_state) {
        case ANIM_ADVERTISING:
            advertising_step();
            (void)k_sem_take(&g_wake, K_MSEC(62));
            break;
        case ANIM_CONNECTED:
            run_connected_animation();
            break;
        case ANIM_BOOT_DONE:
            run_boot_done_animation();
            break;
        case ANIM_IDLE:
        default:
            (void)k_sem_take(&g_wake, K_FOREVER);
            break;
        }
    }
}

void led_animation_init(void)
{
    k_sem_init(&g_wake, 0, 1);
    g_state = ANIM_IDLE;
    g_adv_step = 0;
    k_thread_create(&anim_thread, anim_stack, K_THREAD_STACK_SIZEOF(anim_stack), anim_thread_entry, NULL, NULL, NULL, ANIM_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&anim_thread, "led_anim");
    LOG_INF("LED animation subsystem initialized");
}

/* Stop the animation thread for good (power-off paths): after this call the
 * caller owns the LED. Waits for the current step to finish instead of
 * suspending the thread mid-PWM-update. */
void led_animation_shutdown(void)
{
    g_shutdown = true;
    g_hold_active = false;
    g_state = ANIM_IDLE;
    k_sem_give(&g_wake);
    if (k_thread_join(&anim_thread, K_MSEC(2000)) != 0) {
        LOG_WRN("LED anim: thread did not stop, aborting it");
        k_thread_abort(&anim_thread);
    }
}

void led_animation_play_startup(void)
{
    if (!rgb_led_is_initialized()) return;
    /* fw 0.0.94: black -> green in 0.5 s along the rising half of the sine
     * (slow start, smooth), then green stays on until led_animation_boot_done().
     * Replaces the 3-s colour show (boot is 2.5 s faster). */
    g_boot_phase = true;
    for (int i = 0; i <= 16; i++) {
        rgb_color_t c = {0, k_sine_lut[i], 0};
        rgb_led_set_color(&c);
        k_sleep(K_MSEC(31));
    }
    LOG_INF("LED anim: startup fade to green done (0.5 s)");
}

void led_animation_boot_done(void)
{
    if (!rgb_led_is_initialized()) return;
    if (g_state == ANIM_CONNECTED) {
        g_boot_phase = false;   /* a phone connected meanwhile: its animation wins */
        return;
    }
    g_state = ANIM_BOOT_DONE;
    k_sem_give(&g_wake);
}

bool led_animation_play_oneshot(const rgb_color_t *color,
                                uint8_t pulses,
                                uint32_t on_ms,
                                uint32_t off_ms,
                                button_event_t event,
                                led_oneshot_complete_cb_t on_complete,
                                void *ctx)
{
    if (!rgb_led_is_initialized()) return false;
    oneshot_req_t r;
    r.color = *color;
    r.pulses = pulses;
    r.on_ms = on_ms;
    r.off_ms = off_ms;
    r.event = event;
    r.on_complete = on_complete;
    r.ctx = ctx;

    int rc = k_msgq_put(&g_oneshot_msgq, &r, K_NO_WAIT);
    if (rc != 0) {
        /* Drop oldest and retry once */
        oneshot_req_t drop;
        (void)k_msgq_get(&g_oneshot_msgq, &drop, K_NO_WAIT);
        rc = k_msgq_put(&g_oneshot_msgq, &r, K_NO_WAIT);
        if (rc != 0) {
            LOG_WRN("led_animation: oneshot queue full, request dropped");
            return false;
        }
    }
    k_sem_give(&g_wake);
    return true;
}

void led_animation_hold_start(const rgb_color_t *color)
{
    if (!rgb_led_is_initialized()) return;
    g_hold_color = *color;
    g_hold_active = true;
    k_sem_give(&g_wake);
}

void led_animation_hold_stop(void)
{
    g_hold_active = false;
    k_sem_give(&g_wake);
}

void led_animation_start_advertising(void)
{
    g_adv_pending = true;
    if (g_boot_phase) {
        LOG_INF("LED anim: advertising pulse deferred until the boot sequence is done");
        return;
    }
    LOG_INF("LED anim: start advertising pulse");
    g_adv_step = 0;
    g_state = ANIM_ADVERTISING;
    k_sem_give(&g_wake);
}

void led_animation_on_connected(void)
{
    LOG_INF("LED anim: BLE connected – trigger morph animation");
    g_conn = true;
    g_state = ANIM_CONNECTED;
    k_sem_give(&g_wake);
}

void led_animation_on_disconnected(void)
{
    g_conn = false;
    if (g_boot_phase) {
        return;   /* boot sequence still running; it starts the pulse at its end */
    }
    LOG_INF("LED anim: BLE disconnected – resume advertising pulse");
    rgb_led_off();
    g_adv_step = 0;
    g_state = ANIM_ADVERTISING;
    k_sem_give(&g_wake);
}
