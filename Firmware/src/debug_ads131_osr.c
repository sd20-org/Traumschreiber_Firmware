/**
 * @file debug_ads131_osr.c
 * @brief Simple debugging code for ADS131M08 OSR register write testing
 *
 * This file contains a minimal debug function to test register writing on ADS131M08.
 * Specifically tests changing the OSR (Oversampling Ratio) to 2048.
 *
 * Sequence:
 * 1. Wait 7 seconds after startup
 * 2. Read all ADS131M08 registers and print to RTT
 * 3. Wait 1 second
 * 4. Write OSR=2048 to CLOCK register
 * 5. Wait 1 second
 * 6. Read all registers again and print to RTT
 *
 * Expected result: Sample rate should drop by half (from ~1000Hz to ~500Hz)
 * This can be verified by:
 * - RTT output showing CLOCK register change
 * - SD card data showing reduced sample rate
 *
 * @author ISDT
 * @date 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "drivers/ads131m08_ti.h"

LOG_MODULE_REGISTER(debug_ads131_osr, LOG_LEVEL_INF);

/* ADS131M08 register addresses (from TI reference) */
#define ADS131M08_REG_ID 0x00
#define ADS131M08_REG_STATUS 0x01
#define ADS131M08_REG_MODE 0x02
#define ADS131M08_REG_CLOCK 0x03
#define ADS131M08_REG_GAIN1 0x04
#define ADS131M08_REG_GAIN2 0x05
#define ADS131M08_REG_CFG 0x06
#define ADS131M08_REG_THRSHLD_MSB 0x07
#define ADS131M08_REG_THRSHLD_LSB 0x08

/* OSR=4096 value for CLOCK register (from ads131m0x.h) */
/* CLOCK_OSR_4096 = ((uint16_t) 0x0005 << 2) = 0x0014 */
#define ADS131M08_CLOCK_OSR_4096_BITS 0x0014

/* OSR=1024 value for CLOCK register (from ads131m0x.h) */
/* CLOCK_OSR_1024 = ((uint16_t) 0x0003 << 2) = 0x000C */
#define ADS131M08_CLOCK_OSR_1024_BITS 0x000C

/* Number of registers to read */
#define NUM_REGISTERS_TO_READ 9

/**
 * @brief Read and print all important ADS131M08 registers
 *
 * Reads registers 0x00-0x08 and prints them in both hex and binary format
 *
 * NOTE: Uses the same approach as the working code in ads131m08_ti.c
 */
static void read_and_print_all_registers(const char *label)
{
    uint16_t reg_value;
    ads131m08_status_t status;

    LOG_INF("=== %s ===", label);

    for (uint8_t addr = 0; addr < NUM_REGISTERS_TO_READ; addr++)
    {
        /* Read register using 24-bit word length (matching working code) */
        status = ads131m08_ti_read_register(addr, &reg_value, 24);

        if (status == ADS131M08_OK)
        {
            /* Print in binary format with spaces every 4 bits for readability */
            char bin_str[20]; // 16 bits + 3 spaces + null terminator
            int idx = 0;

            for (int bit = 15; bit >= 0; bit--)
            {
                bin_str[idx++] = ((reg_value >> bit) & 1) ? '1' : '0';
                if (bit % 4 == 0 && bit != 0)
                {
                    bin_str[idx++] = ' ';
                }
            }
            bin_str[idx] = '\0';

            LOG_INF("Register 0x%02X: 0x%04X = 0b%s", addr, reg_value, bin_str);

            /* Special parsing for CLOCK register */
            if (addr == ADS131M08_REG_CLOCK)
            {
                uint8_t osr_bits = (reg_value >> 2) & 0x07;
                uint8_t pwr_bits = reg_value & 0x03;
                uint8_t ch_en = (reg_value >> 8) & 0xFF;

                const char *osr_names[] = {
                    "128", "256", "512", "1024", "2048", "4096", "8192", "16384"};
                const char *pwr_names[] = {
                    "VLP", "LP", "HR", "HR"};

                LOG_INF("  CLOCK decoded: OSR=%s, PWR=%s, CH_EN=0x%02X",
                        osr_names[osr_bits], pwr_names[pwr_bits], ch_en);
            }
        }
        else
        {
            LOG_ERR("Failed to read register 0x%02X: %d", addr, status);
        }
    }

    LOG_INF("=== End of %s ===", label);
}

/**
 * @brief Test OSR register write by changing from default to 4096
 *
 * This is the main debugging function implementing the sequence:
 * 1. Wait 7 seconds
 * 2. Read all registers (BEFORE state)
 * 3. Wait 1 second
 * 4. Write OSR=4096 to CLOCK register
 * 5. Wait 1 second
 * 6. Read all registers (AFTER state)
 *
 * @return 0 on success, negative error code on failure
 */
int debug_ads131_osr_test(void)
{

    /* Step 1: Wait 7 seconds after startup */
    LOG_INF("Step 1: Waiting 7 seconds for system to stabilize...");
    k_sleep(K_SECONDS(7));
    LOG_INF("7 seconds elapsed - system should be stable now");

    /* Step 2: Read all registers BEFORE write */
    LOG_INF("Step 2: Reading all registers BEFORE OSR change...");
    read_and_print_all_registers("REGISTERS BEFORE OSR=4096 WRITE");

    /* Step 3: Wait 1 second */
    LOG_INF("Step 3: Waiting 1 second...");
    k_sleep(K_SECONDS(1));

    /* Step 4: Write OSR=4096 to CLOCK register */
    LOG_INF("Step 4: Writing OSR=4096 to CLOCK register...");

    /* Read current CLOCK register value */
    uint16_t current_clock;
    ads131m08_status_t status = ads131m08_ti_read_register(ADS131M08_REG_CLOCK, &current_clock, 24);
    if (status != ADS131M08_OK)
    {
        LOG_ERR("Failed to read current CLOCK register: %d", status);
        return -1;
    }

    LOG_INF("Current CLOCK register value: 0x%04X", current_clock);

    /* Modify only the OSR bits [4:2], preserve all other bits */
    uint16_t new_clock = (current_clock & ~0x001C) | ADS131M08_CLOCK_OSR_1024_BITS;
    LOG_INF("New CLOCK register value: 0x%04X (OSR bits changed to 1024)", new_clock);

    /* Write the new value */
    status = ads131m08_ti_write_register(ADS131M08_REG_CLOCK, new_clock, 24);
    if (status != ADS131M08_OK)
    {
        LOG_ERR("Failed to write CLOCK register: %d", status);
        return -1;
    }

    LOG_INF("CLOCK register write completed");

    /* Step 5: Wait 1 second for change to take effect */
    LOG_INF("Step 5: Waiting 1 second for change to take effect...");
    k_sleep(K_SECONDS(1));

    /* Step 6: Read all registers AFTER write */
    LOG_INF("Step 6: Reading all registers AFTER OSR change...");
    read_and_print_all_registers("REGISTERS AFTER OSR=1024 WRITE");

    /* Final verification */
    LOG_INF("=== Test Complete ===");
    LOG_INF("Check the register dumps above:");
    LOG_INF("- BEFORE: CLOCK register should show OSR=1024 (default)");
    LOG_INF("- AFTER: CLOCK register should show OSR=4096 (if write worked)");
    LOG_INF("");
    LOG_INF("Verification methods:");
    LOG_INF("1. RTT Output: Compare CLOCK register values above");
    LOG_INF("2. SD Card: Check if sample rate dropped to 1/4 in binary log");
    LOG_INF("   Expected: ~1000Hz -> ~250Hz (or whatever your base rate was)");

    return 0;
}
