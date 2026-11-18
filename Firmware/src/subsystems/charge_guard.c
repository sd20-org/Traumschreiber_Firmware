#include "subsystems/charge_guard.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(charge_guard, LOG_LEVEL_INF);

#define VBUS_NODE DT_CHILD(DT_NODELABEL(adc), channel_5)

static const struct device *const s_adc = DEVICE_DT_GET(DT_NODELABEL(adc));
static bool s_setup_done;
static atomic_t s_streak = ATOMIC_INIT(0);
static atomic_t s_tripped = ATOMIC_INIT(0);
static atomic_t s_trip_mv = ATOMIC_INIT(0);
static K_SEM_DEFINE(s_trip_sem, 0, 1);

int charge_guard_read_vbus_mv(void)
{
        static const struct adc_channel_cfg cfg = ADC_CHANNEL_CFG_DT(VBUS_NODE);
        int16_t raw = 0;
        struct adc_sequence seq = {
                .channels = BIT(cfg.channel_id),
                .buffer = &raw,
                .buffer_size = sizeof(raw),
                .resolution = DT_PROP(VBUS_NODE, zephyr_resolution),
        };

        if (!device_is_ready(s_adc))
                return -ENODEV;
        if (!s_setup_done) {
                /* Same config nordic_adc sets later; repeating it is harmless. */
                int err = adc_channel_setup(s_adc, &cfg);
                if (err)
                        return err;
                s_setup_done = true;
        }
        int err = adc_read(s_adc, &seq);
        if (err)
                return err;
        int32_t mv = raw < 0 ? 0 : raw;
        err = adc_raw_to_millivolts(adc_ref_internal(s_adc), cfg.gain, seq.resolution, &mv);
        return err ? err : (int)mv;
}

void charge_guard_report_vbus_mv(int mv)
{
        if (mv < CHARGE_GUARD_THRESHOLD_MV) {
                atomic_set(&s_streak, 0);
                return;
        }
        if (atomic_inc(&s_streak) + 1 < CHARGE_GUARD_CONFIRM_COUNT)
                return;
        if (atomic_cas(&s_tripped, 0, 1)) {
                atomic_set(&s_trip_mv, mv);
                k_sem_give(&s_trip_sem);
        }
}

bool charge_guard_wait(k_timeout_t timeout)
{
        return k_sem_take(&s_trip_sem, timeout) == 0;
}

int charge_guard_trip_mv(void)
{
        return (int)atomic_get(&s_trip_mv);
}
