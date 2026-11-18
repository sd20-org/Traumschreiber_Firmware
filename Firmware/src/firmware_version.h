#ifndef FIRMWARE_VERSION_H
#define FIRMWARE_VERSION_H

/**
 * @file firmware_version.h
 *
 * Firmware version string, taken from the VERSION file (bump PATCHLEVEL there
 * per release tag, e.g. `firmware-v0.0.45` → PATCHLEVEL = 45). At runtime the string is:
 *   - printed once on boot via LOG_INF (RTT)
 *   - written to /SD/firmware-version.txt on every boot (so an SD pull tells
 *     you which firmware produced the EDF)
 *   - exposed as a read-only BLE characteristic so the app's Connection tab
 *     can show it.
 *
 * Keep numeric (no leading 'v') and short — must fit a 24-byte BLE attribute.
 */
/* Single source since 0.0.95: the VERSION file next to CMakeLists.txt. Zephyr
 * turns it into app_version.h, and imgtool writes the same number into the
 * MCUboot image header (before 0.0.95 the header said "0.0.4" on every build).
 * The sd20App reads that header over SMP: "0.0.4" = old bootloader key
 * (needs one cable flash), anything newer = production key. */
#include <app_version.h>
#define FIRMWARE_VERSION_STR APP_VERSION_STRING

#endif /* FIRMWARE_VERSION_H */
