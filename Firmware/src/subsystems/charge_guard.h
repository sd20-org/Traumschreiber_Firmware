/*
 * charge_guard — no operation while USB is plugged in (fw 0.0.95).
 *
 * AIN5 sits behind a 3.3k/1.8k divider on VBUS (after fuse F1): ~1.76 V with
 * USB 5 V, 0 V without (R70 pulls down). The TP4056 charges the cell directly,
 * so the device is never in use on the charger: as soon as VBUS is seen, the
 * firmware closes the recording, blinks red 5x with a low beep and enters
 * System OFF
 * (Aktionsplan Verkaufsstart E7: protective measure instead of an IFU hint).
 */
#ifndef CHARGE_GUARD_H__
#define CHARGE_GUARD_H__

#include <stdbool.h>
#include <zephyr/kernel.h>

/* VBUS present above this AIN5 voltage (~2.8 V VBUS; nominal 1760 mV). */
#define CHARGE_GUARD_THRESHOLD_MV 1000
/* Consecutive readings above the threshold before acting (1 per second). */
#define CHARGE_GUARD_CONFIRM_COUNT 2

/* One-shot AIN5 read in mV (own ADC sequence/buffer). <0 on error. Do NOT
 * call while the ADS131 records (SAADC current couples into the EEG) — use
 * the per-record reading the data logger reports instead. */
int charge_guard_read_vbus_mv(void);

/* Feed one AIN5 reading; after CHARGE_GUARD_CONFIRM_COUNT readings in a row
 * above the threshold the guard trips (charge_guard_wait() returns true).
 * Safe from any thread. */
void charge_guard_report_vbus_mv(int mv);

/* Block up to `timeout` for the guard to trip. */
bool charge_guard_wait(k_timeout_t timeout);

/* Last AIN5 reading that tripped the guard (for the log). */
int charge_guard_trip_mv(void);

#endif /* CHARGE_GUARD_H__ */
