/**
 * @file debug_ads131_osr.h
 * @brief Header file for ADS131M08 OSR register write debug test
 *
 * @author ISDT
 * @date 2025
 */

#ifndef DEBUG_ADS131_OSR_H
#define DEBUG_ADS131_OSR_H

/**
 * @brief Run the OSR=2048 register write debug test
 *
 * This function implements a simple debug sequence:
 * 1. Wait 7 seconds after startup
 * 2. Read all registers and print to RTT
 * 3. Wait 1 second
 * 4. Write OSR=2048 to CLOCK register
 * 5. Wait 1 second
 * 6. Read all registers again and print to RTT
 *
 * Expected behavior: Sample rate should drop by half if write is successful
 *
 * @return 0 on success, negative error code on failure
 */
int debug_ads131_osr_test(void);

#endif /* DEBUG_ADS131_OSR_H */
