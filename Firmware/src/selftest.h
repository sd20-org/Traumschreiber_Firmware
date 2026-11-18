/*
 * TS Hardware Self-Test (build option TS_HW_SELFTEST).
 *
 * Purpose: compare the parts that changed between TS2603 and TS2607
 *   - microphone preamp gain (R14 100k->1M5, C10 100p->15p)
 *   - brightness sensor load (R74 2M2->10M)
 *   - speaker amp gain (GAIN_SLOT 6 dB -> 15 dB)
 * Flash the SAME signed image onto a TS2607 and a TS2603, read the RTT log,
 * and compare the reported microphone level / brightness value / speaker tone.
 */
#ifndef TS_SELFTEST_H
#define TS_SELFTEST_H

/* Runs forever (never returns). Only invoked in TS_HW_SELFTEST builds. */
void ts_selftest_run(void);

#endif /* TS_SELFTEST_H */
