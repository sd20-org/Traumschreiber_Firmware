#include "ads131m08_ti.h"
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include "../subsystems/system_config.h" /* g_config.test_block_secs (TEST mode) */
LOG_MODULE_REGISTER(ads131m08_ti, LOG_LEVEL_INF);

/* Forward declarations */
static ads131m08_status_t read_register_ti(uint8_t addr, uint16_t *data, uint8_t word_length);
static ads131m08_status_t write_register_ti(uint8_t addr, uint16_t data, uint8_t word_length);
static void validate_register_values(void);

void debug_print_all_registers(void);
static void periodic_register_dump_handler(struct k_work *work);
static void ads_reader_thread_entry(void *p1, void *p2, void *p3);
void ads_log_registers(void);

static ads131m08_driver_t ads_driver;

// DIAGNOSTIC: Global counters to track DRDY IRQs vs samples processed
// DIAGNOSTIC: Global counters to track DRDY IRQs vs samples processed
static volatile uint32_t drdy_irq_total_count = 0;
static volatile uint32_t ads_samples_processed = 0;
static volatile uint32_t ads_spi_errors = 0;
static volatile uint32_t ads_callbacks_invoked = 0;
/* R1 (data integrity analysis): conversions physically lost because
 * the reader drained >1 pending DRDY token (no ADC FIFO). Now advanced into the
 * sn counter as a VISIBLE gap; this counts them for RTT visibility under load. */
static volatile uint32_t ads_dropped_conversions = 0;
static struct k_sem ads_reader_sem;

/* Input-multiplexer switching while the ADC runs (noise measurement with
 * shorted inputs). The request is piggybacked on the regular data frames by the
 * reader thread: one WREG to one CHn_CFG register per frame (the CHn_CFG
 * registers are not contiguous — OCAL/GCAL sit in between and are writable, so
 * a multi-register WREG would clobber the calibration). Datasheet SBAS950B
 * 8.5.1.10.8: the WREG data words follow the command in the same input frame,
 * and the device still shifts out the eight channel words of that frame, so no
 * conversion is lost. The WREG response replaces the STATUS word of the NEXT
 * frame. CHn_CFG keeps its reset value 0x0000 apart from MUX[1:0] (the driver
 * never writes PHASE/DCBLKn_DIS). */
static atomic_t g_mux_request = ATOMIC_INIT(0);   /* 1 = new request pending */
static atomic_t g_mux_target  = ATOMIC_INIT(0);   /* requested MUX value 0..3 */
static atomic_t g_mux_applied = ATOMIC_INIT(0);   /* MUX value on all channels, -1 = switching */
static uint8_t g_mux_next_ch  = ADS131M08_MAX_CHANNELS; /* reader thread only */
static uint8_t g_mux_value    = 0;                      /* reader thread only */
K_THREAD_STACK_DEFINE(ads_reader_stack, 4096);
static struct k_thread ads_reader_thread;

void debug_print_all_registers(void)
{
    LOG_INF("=== ADS131M08 Register Dump ===");

    // Read all important registers first, then log in batches to avoid buffer overflow
    uint16_t id_reg, status_reg, mode_reg, clock_reg, gain1_reg, gain2_reg;
    uint16_t cfg_reg, thrshld_msb_reg, thrshld_lsb_reg;
    uint16_t ch0_cfg_reg, ch1_cfg_reg;
    uint16_t ch0_ocal_msb, ch0_ocal_lsb, ch0_gcal_msb, ch0_gcal_lsb;

    // Read core registers
    if (read_register_ti(ADS131M08_REG_ID, &id_reg, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_STATUS, &status_reg, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_MODE, &mode_reg, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_CLOCK, &clock_reg, 24) == ADS131M08_OK)
    {

        // Log core registers in one message to reduce RTT traffic
        LOG_INF("ID=0x%04X STATUS=0x%04X MODE=0x%04X CLOCK=0x%04X",
                id_reg, status_reg, mode_reg, clock_reg);
    }
    else
    {
        LOG_INF("Failed to read core registers");
    }

    // Small delay to let RTT buffer drain
    k_msleep(10);

    // Read gain and config registers
    if (read_register_ti(ADS131M08_REG_GAIN1, &gain1_reg, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_GAIN2, &gain2_reg, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_CFG, &cfg_reg, 24) == ADS131M08_OK)
    {

        LOG_INF("GAIN1=0x%04X GAIN2=0x%04X CFG=0x%04X",
                gain1_reg, gain2_reg, cfg_reg);
    }

    k_msleep(10);

    // Read threshold registers
    if (read_register_ti(ADS131M08_REG_THRSHLD_MSB, &thrshld_msb_reg, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_THRSHLD_LSB, &thrshld_lsb_reg, 24) == ADS131M08_OK)
    {

        LOG_INF("THRSHLD_MSB=0x%04X THRSHLD_LSB=0x%04X",
                thrshld_msb_reg, thrshld_lsb_reg);
    }

    k_msleep(10);

    // Read channel config registers
    if (read_register_ti(ADS131M08_REG_CH0_CFG, &ch0_cfg_reg, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_CH1_CFG, &ch1_cfg_reg, 24) == ADS131M08_OK)
    {

        LOG_INF("CH0_CFG=0x%04X CH1_CFG=0x%04X", ch0_cfg_reg, ch1_cfg_reg);
    }

    k_msleep(10);

    // Read calibration registers
    if (read_register_ti(ADS131M08_REG_CH0_OCAL_MSB, &ch0_ocal_msb, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_CH0_OCAL_LSB, &ch0_ocal_lsb, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_CH0_GCAL_MSB, &ch0_gcal_msb, 24) == ADS131M08_OK &&
        read_register_ti(ADS131M08_REG_CH0_GCAL_LSB, &ch0_gcal_lsb, 24) == ADS131M08_OK)
    {

        LOG_INF("CH0_OCAL=0x%04X%04X CH0_GCAL=0x%04X%04X",
                ch0_ocal_msb, ch0_ocal_lsb, ch0_gcal_msb, ch0_gcal_lsb);
    }

    LOG_INF("=== End Register Dump ===");
}

/**
 * @brief Periodic register dump work handler
 * Called every 5 seconds to dump all ADS131M08 registers for monitoring
 */
static void periodic_register_dump_handler(struct k_work *work)
{
    static uint32_t dump_counter = 0;

    // Only perform detailed register dumps when not actively sampling to avoid timing interference
    if (!ads_driver.sampling_active && ads_driver.driver_ready)
    {
        LOG_INF("=== Periodic Register Dump #%u (Sampling Idle) ===", ++dump_counter);
        LOG_INF("Driver ready: %s, Total samples collected: %u",
                ads_driver.driver_ready ? "YES" : "NO", ads_driver.sample_count);
        debug_print_all_registers();
    }
    else if (ads_driver.sampling_active)
    {
        // DIAGNOSTIC: Minimal status update during sampling to avoid RTT interference
        if ((++dump_counter % 10) == 0) // Only log every 10th periodic call during sampling (50s intervals)
        {
            uint32_t total_irqs = drdy_irq_total_count;
            uint32_t samples = ads_driver.sample_count;
            LOG_INF("=== DIAGNOSTIC #%u (Sampling) ===", dump_counter);
            uint32_t ms_diag = k_uptime_get_32();
            uint32_t rate_hz = (ms_diag > 0) ? (samples * 1000u) / ms_diag : 0u;
            LOG_INF("IRQs: %u, Samples: %u, Lost: %u, Rate: %u Hz",
                    total_irqs, samples, (total_irqs > samples) ? (total_irqs - samples) : 0,
                    rate_hz);
        }
    }
    else
    {
        LOG_INF("Driver not ready - skipping register dump");
    }

    // Reschedule for next dump in 5 seconds
    k_work_schedule((struct k_work_delayable *)work, K_SECONDS(5));
}

#define ADS131M08_GPIO_DRDY_PORT DT_NODELABEL(gpio1)
#define ADS131M08_GPIO_DRDY_PIN 10
#define ADS131M08_GPIO_SYNC_PORT DT_NODELABEL(gpio1)
#define ADS131M08_GPIO_SYNC_PIN 11
#define ADS131M08_GPIO_CS_PORT DT_NODELABEL(gpio0)
#define ADS131M08_GPIO_CS_PIN 3
#define ADS131M08_SPI_NODE DT_NODELABEL(spi0)
// R11 FIX (data integrity analysis): back to the DT-rated 1 MHz. The
// board overlay declares spi-max-frequency = <1000000>; the earlier "8 MHz for
// testing" bump drove the bus 8× above the qualified trace rating, so any SPI
// bit error was written to the SD as valid EEG (no frame CRC → undetectable
// downstream). At 250 Hz a 27-byte frame needs ~216 µs at 1 MHz, far inside the
// 4 ms DRDY period, so 1 MHz has ample margin. (Enabling the ADS CRC word is a
// separate follow-up.) Verify on bench via RTT: spi_errors=0, steady 250 Hz.
#define ADS131M08_SPI_FREQUENCY 1000000
#define ADS131M08_SPI_MODE 0
#define ADS131M08_RESET_DELAY_MS 50
#define ADS131M08_STARTUP_DELAY_MS 100
#define ADS131M08_DRDY_TIMEOUT_MS 100

/* Read a register from the ADS131M08 via SPI.
 * Uses 8-bit SPI transfers to create atomic 24-bit frames as required by device.
 * Command format: 101a aaaa annn nnnn (RREG with 6-bit address in bits [13:8])
 * IMPORTANT: ADS131M08 uses pipelined responses - command response comes in NEXT transaction.
 */
static ads131m08_status_t read_register_ti(uint8_t addr, uint16_t *data, uint8_t word_length)
// does not work
{
    if (!ads_driver.spi_dev || !data)
    {
        return ADS131M08_ERROR;
    }

    // Configure SPI for 8-bit transfers with Mode 1 (CPOL=0, CPHA=1) as required by ADS131M08
    struct spi_config spi_cfg_8bit = {
        .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8),
        .frequency = ADS131M08_SPI_FREQUENCY,
        .slave = 0,
    };
    spi_cfg_8bit.cs = ads_driver.spi_cfg.cs;

    // Format command: RREG (0xA000) with address in bits [13:7] and word count in bits [6:0]
    // Command format: 101a aaaa annn nnnn (per ADS131M08 datasheet)
    // For single register read: address shifted left by 7, word count = 0
    uint16_t command_word = ADS131M08_CMD_RREG | (addr << 7);

    // Build tx buffer as [b2 b1 b0] MSB-first for 24-bit frame (command + padding)
    uint8_t tx_frame1[3] = {
        (command_word >> 8) & 0xFF, // Command high byte (b2)
        command_word & 0xFF,        // Command low byte (b1)
        0x00                        // Padding byte (b0)
    };
    uint8_t rx_frame1[3] = {0};

    const struct spi_buf tx_buf1 = {.buf = tx_frame1, .len = 3};
    const struct spi_buf rx_buf1 = {.buf = rx_frame1, .len = 3};
    const struct spi_buf_set tx_buf_set1 = {.buffers = &tx_buf1, .count = 1};
    const struct spi_buf_set rx_buf_set1 = {.buffers = &rx_buf1, .count = 1};

    // Send command frame
    int ret = spi_transceive(ads_driver.spi_dev, &spi_cfg_8bit, &tx_buf_set1, &rx_buf_set1);
    if (ret != 0)
    {
        return ADS131M08_ERROR;
    }

    // Add delay between command and response frame (datasheet timing requirement)
    k_sleep(K_USEC(100));

    // Send dummy frame to get the pipelined response
    uint8_t tx_frame2[3] = {0x00, 0x00, 0x00}; // Dummy/NULL command
    uint8_t rx_frame2[3] = {0};

    const struct spi_buf tx_buf2 = {.buf = tx_frame2, .len = 3};
    const struct spi_buf rx_buf2 = {.buf = rx_frame2, .len = 3};
    const struct spi_buf_set tx_buf_set2 = {.buffers = &tx_buf2, .count = 1};
    const struct spi_buf_set rx_buf_set2 = {.buffers = &rx_buf2, .count = 1};

    // Get the response from previous command
    ret = spi_transceive(ads_driver.spi_dev, &spi_cfg_8bit, &tx_buf_set2, &rx_buf_set2);
    if (ret != 0)
    {
        return ADS131M08_ERROR;
    }

    // Extract 16-bit register data from MSB-aligned 24-bit response [b2 b1 b0]
    *data = (rx_frame2[0] << 8) | rx_frame2[1];

    return ADS131M08_OK;
}

/* Write a register into the ADS131M08 via SPI.
 * Uses 8-bit SPI transfers to create atomic 24-bit frames as required by device.
 * Command format: 011a aaaa annn nnnn (WREG with address in bits [13:7], word count in bits [6:0])
 * IMPORTANT: ADS131M08 uses pipelined responses - send command, then data, then dummy for ack.
 */
static ads131m08_status_t write_register_ti(uint8_t addr, uint16_t data, uint8_t word_length)
{
    if (!ads_driver.spi_dev)
    {
        return ADS131M08_ERROR;
    }

    // Configure SPI for 8-bit transfers with Mode 1 (CPOL=0, CPHA=1) as required by ADS131M08
    struct spi_config spi_cfg_8bit = {
        .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8),
        .frequency = ADS131M08_SPI_FREQUENCY,
        .slave = 0,
    };
    spi_cfg_8bit.cs = ads_driver.spi_cfg.cs;

    // CRITICAL FIX: Send WREG command AND data in ONE ATOMIC SPI transaction
    // This keeps CS low for the entire command+data sequence as required by datasheet

    // Format command: WREG (0x6000) with address in bits [13:7] and word count in bits [6:0]
    // Command format: 011a aaaa annn nnnn (per ADS131M08 datasheet)
    // For single register write: address shifted left by 7, word count = 0
    uint16_t command_word = ADS131M08_CMD_WREG | (addr << 7);

    LOG_INF("=== CORRECTED WRITE REGISTER DEBUG ===");
    LOG_INF("Target: Write 0x%04X to register 0x%02X", data, addr);
    LOG_INF("WREG command: 0x%04X", command_word);

    // Build complete 6-byte frame: [WREG_CMD (3 bytes)][DATA (3 bytes)] - sent atomically
    uint8_t tx_complete_frame[6] = {
        // Word 1: WREG command (24-bit, MSB-aligned)
        (command_word >> 8) & 0xFF, // Command high byte
        command_word & 0xFF,        // Command low byte
        0x00,                       // Padding byte
        // Word 2: Data to write (24-bit, MSB-aligned)
        (data >> 8) & 0xFF, // Data high byte
        data & 0xFF,        // Data low byte
        0x00                // Padding byte
    };
    uint8_t rx_complete_frame[6] = {0};

    LOG_INF("TX Atomic Frame (6 bytes): %02X %02X %02X | %02X %02X %02X",
            tx_complete_frame[0], tx_complete_frame[1], tx_complete_frame[2],
            tx_complete_frame[3], tx_complete_frame[4], tx_complete_frame[5]);

    const struct spi_buf tx_buf_complete = {.buf = tx_complete_frame, .len = 6};
    const struct spi_buf rx_buf_complete = {.buf = rx_complete_frame, .len = 6};
    const struct spi_buf_set tx_buf_set_complete = {.buffers = &tx_buf_complete, .count = 1};
    const struct spi_buf_set rx_buf_set_complete = {.buffers = &rx_buf_complete, .count = 1};

    // Send complete WREG transaction atomically (CS stays low for all 6 bytes)
    int ret = spi_transceive(ads_driver.spi_dev, &spi_cfg_8bit, &tx_buf_set_complete, &rx_buf_set_complete);
    if (ret != 0)
    {
        LOG_ERR("WREG atomic transaction failed: %d", ret);
        return ADS131M08_ERROR;
    }

    LOG_INF("RX Response (6 bytes): %02X %02X %02X | %02X %02X %02X",
            rx_complete_frame[0], rx_complete_frame[1], rx_complete_frame[2],
            rx_complete_frame[3], rx_complete_frame[4], rx_complete_frame[5]);

    // Parse response: bytes [3-4] should contain echo of WREG command
    uint16_t wreg_echo = (rx_complete_frame[3] << 8) | rx_complete_frame[4];
    LOG_INF("WREG command echo: 0x%04X (expected: 0x%04X)", wreg_echo, command_word);

    LOG_INF("=== END CORRECTED WRITE REGISTER DEBUG ===");

    // IMMEDIATE VERIFICATION: Send RREG+NULL sequence right after WREG to verify write
    // This tests the pipelining behavior directly

    // Short delay after write for tREGACQ timing
    k_sleep(K_MSEC(2));

    // Implement detailed verification with RREG+NULL sequence and full byte logging
    const int max_retries = 3;
    for (int attempt = 0; attempt < max_retries; attempt++)
    {
        LOG_INF("=== VERIFICATION ATTEMPT %d ===", attempt + 1);

        // Step 1: Send RREG command for the register we just wrote
        uint16_t rreg_cmd = ADS131M08_CMD_RREG | (addr << 8) | 0x00;
        uint8_t rreg_tx[3] = {
            (rreg_cmd >> 8) & 0xFF,
            rreg_cmd & 0xFF,
            0x00};
        uint8_t rreg_rx[3] = {0};

        LOG_INF("TX RREG cmd (3 bytes): %02X %02X %02X", rreg_tx[0], rreg_tx[1], rreg_tx[2]);

        struct spi_buf rreg_tx_buf = {.buf = rreg_tx, .len = 3};
        struct spi_buf rreg_rx_buf = {.buf = rreg_rx, .len = 3};
        struct spi_buf_set rreg_tx_set = {.buffers = &rreg_tx_buf, .count = 1};
        struct spi_buf_set rreg_rx_set = {.buffers = &rreg_rx_buf, .count = 1};

        ret = spi_transceive(ads_driver.spi_dev, &spi_cfg_8bit, &rreg_tx_set, &rreg_rx_set);
        if (ret != 0)
        {
            LOG_ERR("RREG cmd transaction failed: %d", ret);
            if (attempt < max_retries - 1)
            {
                k_sleep(K_MSEC(5 * (attempt + 1)));
                continue;
            }
            else
            {
                return ADS131M08_ERROR;
            }
        }

        LOG_INF("RX from RREG cmd (3 bytes): %02X %02X %02X", rreg_rx[0], rreg_rx[1], rreg_rx[2]);

        // Brief delay between RREG command and response fetch
        k_sleep(K_MSEC(1));

        // Step 2: Send NULL command to get the pipelined response from RREG
        uint8_t null_tx[3] = {0x00, 0x00, 0x00};
        uint8_t null_rx[3] = {0};

        LOG_INF("TX NULL cmd (3 bytes): %02X %02X %02X", null_tx[0], null_tx[1], null_tx[2]);

        struct spi_buf null_tx_buf = {.buf = null_tx, .len = 3};
        struct spi_buf null_rx_buf = {.buf = null_rx, .len = 3};
        struct spi_buf_set null_tx_set = {.buffers = &null_tx_buf, .count = 1};
        struct spi_buf_set null_rx_set = {.buffers = &null_rx_buf, .count = 1};

        ret = spi_transceive(ads_driver.spi_dev, &spi_cfg_8bit, &null_tx_set, &null_rx_set);
        if (ret != 0)
        {
            LOG_ERR("NULL cmd transaction failed: %d", ret);
            if (attempt < max_retries - 1)
            {
                k_sleep(K_MSEC(5 * (attempt + 1)));
                continue;
            }
            else
            {
                return ADS131M08_ERROR;
            }
        }

        LOG_INF("RX from NULL cmd (register data): %02X %02X %02X", null_rx[0], null_rx[1], null_rx[2]);

        // Parse the register value from the NULL response
        uint16_t readback_value = (null_rx[0] << 8) | null_rx[1];
        LOG_INF("Parsed register value: 0x%04X (expected: 0x%04X)", readback_value, data);

        if (readback_value == data)
        {
            LOG_INF("✓ Register 0x%02X write verification SUCCESS (attempt %d)", addr, attempt + 1);
            LOG_INF("=== END VERIFICATION ===");
            return ADS131M08_OK;
        }
        else
        {
            LOG_WRN("✗ Register 0x%02X write verification FAILED (attempt %d): wrote 0x%04X, read 0x%04X",
                    addr, attempt + 1, data, readback_value);

            if (attempt < max_retries - 1)
            {
                LOG_INF("Retrying entire WREG sequence with %d ms backoff...", (attempt + 1) * 10);
                k_sleep(K_MSEC(10 * (attempt + 1)));

                // Re-send the complete atomic frame (WREG + data in one transaction)
                ret = spi_transceive(ads_driver.spi_dev, &spi_cfg_8bit, &tx_buf_set_complete, &rx_buf_set_complete);
                if (ret != 0)
                {
                    LOG_ERR("WREG retry transaction failed: %d", ret);
                    continue;
                }

                LOG_INF("WREG retry sent, response: %02X %02X %02X | %02X %02X %02X",
                        rx_complete_frame[0], rx_complete_frame[1], rx_complete_frame[2],
                        rx_complete_frame[3], rx_complete_frame[4], rx_complete_frame[5]);

                k_sleep(K_MSEC(5 * (attempt + 2))); // Additional delay after retry
            }
        }
        LOG_INF("=== END VERIFICATION ATTEMPT %d ===", attempt + 1);
    }

    LOG_ERR("Register 0x%02X write failed after %d attempts", addr, max_retries);
    return ADS131M08_ERROR;
}

/* Log the register values */
void ads_log_registers(void)
{
    LOG_INF("=== KA ADS131M08 Register Values ===");
    int ret = gpio_pin_interrupt_configure(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN,
                                           GPIO_INT_DISABLE);

    uint16_t reg_value;
    if (read_register_ti(ADS131M08_REG_ID, &reg_value, 24) == ADS131M08_OK)
    {
        char bin_str[20]; // 16 bits + 3 spaces + null
        int idx = 0;
        for (int b = 15; b >= 0; b--)
        {
            bin_str[idx++] = ((reg_value >> b) & 1) ? '1' : '0';
            if (b % 4 == 0 && b != 0)
            {
                bin_str[idx++] = ' ';
            }
        }
        bin_str[idx] = '\0';
        LOG_INF("ID Register: 0x%04X (0b%s)", reg_value, bin_str);
    }

    if (read_register_ti(ADS131M08_REG_STATUS, &reg_value, 24) == ADS131M08_OK)
    {
        char bin_str[20];
        int idx = 0;
        for (int b = 15; b >= 0; b--)
        {
            bin_str[idx++] = ((reg_value >> b) & 1) ? '1' : '0';
            if (b % 4 == 0 && b != 0)
            {
                bin_str[idx++] = ' ';
            }
        }
        bin_str[idx] = '\0';
        LOG_INF("STATUS Register: 0x%04X (0b%s)", reg_value, bin_str);
    }

    if (read_register_ti(ADS131M08_REG_MODE, &reg_value, 24) == ADS131M08_OK)
    {
        char bin_str[20];
        int idx = 0;
        for (int b = 15; b >= 0; b--)
        {
            bin_str[idx++] = ((reg_value >> b) & 1) ? '1' : '0';
            if (b % 4 == 0 && b != 0)
            {
                bin_str[idx++] = ' ';
            }
        }
        bin_str[idx] = '\0';
        LOG_INF("MODE Register: 0x%04X (0b%s)", reg_value, bin_str);
    }

    if (read_register_ti(ADS131M08_REG_CLOCK, &reg_value, 24) == ADS131M08_OK)
    {
        char bin_str[20];
        int idx = 0;
        for (int b = 15; b >= 0; b--)
        {
            bin_str[idx++] = ((reg_value >> b) & 1) ? '1' : '0';
            if (b % 4 == 0 && b != 0)
            {
                bin_str[idx++] = ' ';
            }
        }
        bin_str[idx] = '\0';
        LOG_INF("CLOCK Register: 0x%04X (0b%s)", reg_value, bin_str);
    }

    if (read_register_ti(ADS131M08_REG_GAIN1, &reg_value, 24) == ADS131M08_OK)
    {
        char bin_str[20];
        int idx = 0;
        for (int b = 15; b >= 0; b--)
        {
            bin_str[idx++] = ((reg_value >> b) & 1) ? '1' : '0';
            if (b % 4 == 0 && b != 0)
            {
                bin_str[idx++] = ' ';
            }
        }
        bin_str[idx] = '\0';
        LOG_INF("GAIN1 Register: 0x%04X (0b%s)", reg_value, bin_str);
    }

    if (read_register_ti(ADS131M08_REG_GAIN2, &reg_value, 24) == ADS131M08_OK)
    {
        char bin_str[20];
        int idx = 0;
        for (int b = 15; b >= 0; b--)
        {
            bin_str[idx++] = ((reg_value >> b) & 1) ? '1' : '0';
            if (b % 4 == 0 && b != 0)
            {
                bin_str[idx++] = ' ';
            }
        }
        bin_str[idx] = '\0';
        LOG_INF("GAIN2 Register: 0x%04X (0b%s)", reg_value, bin_str);
    }

    if (read_register_ti(ADS131M08_REG_CFG, &reg_value, 24) == ADS131M08_OK)
    {
        char bin_str[20];
        int idx = 0;
        for (int b = 15; b >= 0; b--)
        {
            bin_str[idx++] = ((reg_value >> b) & 1) ? '1' : '0';
            if (b % 4 == 0 && b != 0)
            {
                bin_str[idx++] = ' ';
            }
        }
        bin_str[idx] = '\0';
        LOG_INF("CFG Register: 0x%04X (0b%s)", reg_value, bin_str);
    }

    gpio_pin_interrupt_configure(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN,
                                 GPIO_INT_EDGE_FALLING);

    LOG_INF("=== KA End Register Values ===");
}

/* Validate register values against documented reset values from datasheet */
static void validate_register_values(void)
{
    uint16_t reg_value;
    LOG_INF("=== Validating Register Reset Values ===");

    // ID Register: Expected reset value 28xxh (bits 15:12 = 0010b, bits 11:8 = 1000b)
    if (read_register_ti(ADS131M08_REG_ID, &reg_value, 24) == ADS131M08_OK)
    {
        uint8_t reserved_bits = (reg_value >> 12) & 0x0F; // Bits 15:12
        uint8_t chan_count = (reg_value >> 8) & 0x0F;     // Bits 11:8

        if (reserved_bits == 0x02 && chan_count == 0x08)
        {
            LOG_INF("✓ ID register format correct: 0x%04X (reserved=0x%X, channels=%d)",
                    reg_value, reserved_bits, chan_count);
        }
        else
        {
            LOG_ERR("✗ ID register format incorrect: 0x%04X (expected reserved=0x2, channels=0x8, got reserved=0x%X, channels=0x%X)",
                    reg_value, reserved_bits, chan_count);
        }
    }

    // STATUS Register: Expected reset value 0500h
    if (read_register_ti(ADS131M08_REG_STATUS, &reg_value, 24) == ADS131M08_OK)
    {
        if ((reg_value & 0xFF00) == 0x0500) // Check upper byte, lower byte varies
        {
            LOG_INF("✓ STATUS register close to expected: 0x%04X (expected 0x05xx)", reg_value);
        }
        else
        {
            LOG_WRN("⚠ STATUS register differs from expected: 0x%04X (expected 0x05xx)", reg_value);
        }
    }

    // CLOCK Register: Expected reset value FF0Eh
    if (read_register_ti(ADS131M08_REG_CLOCK, &reg_value, 24) == ADS131M08_OK)
    {
        if (reg_value == 0xFF0E)
        {
            LOG_INF("✓ CLOCK register matches expected: 0x%04X", reg_value);
        }
        else
        {
            LOG_WRN("⚠ CLOCK register differs from expected: 0x%04X (expected 0xFF0E)", reg_value);
        }
    }

    // MODE Register: Expected reset value 0510h
    if (read_register_ti(ADS131M08_REG_MODE, &reg_value, 24) == ADS131M08_OK)
    {
        if (reg_value == 0x0510)
        {
            LOG_INF("✓ MODE register matches expected: 0x%04X", reg_value);
        }
        else
        {
            LOG_WRN("⚠ MODE register differs from expected: 0x%04X (expected 0x0510)", reg_value);
        }
    }

    LOG_INF("=== End Register Validation ===");
}

/* GPIO Data Ready (DRDY) interrupt handler.
 * Minimal work here: schedule a delayed work item to read the frame from SPI in thread context.
 * DIAGNOSTIC: Added timestamp logging to measure actual DRDY rate vs expected 250Hz.
 */
/* Capture timestamp at DRDY for accurate timing */
static volatile uint64_t last_drdy_timestamp_us = 0;

static void drdy_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);
    drdy_irq_total_count++;
    /* Capture timestamp immediately at interrupt for accurate timing */
    last_drdy_timestamp_us = k_uptime_get() * 1000;
    /* Signal reader thread immediately; keep ISR minimal */
    k_sem_give(&ads_reader_sem);
}

/* Work handler executed in thread context when DRDY fires.
 * Reads an entire frame (status + channels) from the device and converts 24-bit values to signed 32-bit.
 * The function also invokes the user-provided callback when a new sample is available.
 * DIAGNOSTIC: Added counters to track IRQs vs samples processed to detect lost IRQs.
 */
static void ads_reader_do_read(void)
{
    if (!ads_driver.sampling_active)
    {
        return;
    }

    // Defensive checks to catch init ordering issues
    if (ads_driver.spi_dev == NULL)
    {
        LOG_ERR("ADS READER: spi_dev is NULL");
        return;
    }
    if (ads_driver.gpio_dev == NULL)
    {
        LOG_ERR("ADS READER: gpio_dev is NULL");
        return;
    }

    memset(ads_driver.tx_buffer, 0, sizeof(ads_driver.tx_buffer));

    // Implement proper FIFO clearing with two sequential reads (one per DRDY)
    static uint8_t fifo_clear_count = 0;

    if (ads_driver.first_read)
    {
        LOG_DBG("ADS READER: first_read path (fifo_clear_count=%u)", fifo_clear_count + 1);
        struct spi_buf tx_bufs[] = {
            {.buf = ads_driver.tx_buffer, .len = ADS131M08_FRAME_BYTES}};
        struct spi_buf rx_bufs[] = {
            {.buf = ads_driver.rx_buffer, .len = ADS131M08_FRAME_BYTES}};
        struct spi_buf_set tx_set = {.buffers = tx_bufs, .count = 1};
        struct spi_buf_set rx_set = {.buffers = rx_bufs, .count = 1};

        LOG_DBG("ADS READER: spi_transceive (FIFO clear) len=%u", ADS131M08_FRAME_BYTES);
        fifo_clear_count++;
        int ret = spi_transceive(ads_driver.spi_dev, &ads_driver.spi_cfg, &tx_set, &rx_set);
        if (ret < 0)
        {
            ads_spi_errors++;
            LOG_ERR("ADS READER: spi_transceive FIFO clear #%u failed: %d", fifo_clear_count, ret);
        }

        if (fifo_clear_count == 1)
        {
            LOG_INF("First read after sampling start - clearing 2-deep FIFO with two reads");
            LOG_INF("First FIFO clear read result: %d (waiting for next DRDY)", ret);
            return; // Wait for next DRDY
        }
        else if (fifo_clear_count == 2)
        {
            LOG_INF("Second FIFO clear read result: %d", ret);
            ads_driver.first_read = false;
            fifo_clear_count = 0;
            LOG_INF("FIFO cleared with two reads - subsequent reads will provide valid data");
            return;
        }
    }
    LOG_DBG("ADS READER: normal read path");
    /* Pending input-mux change: put WREG CHn_CFG into words 0/1 of this frame. */
    if (atomic_cas(&g_mux_request, 1, 0))
    {
        g_mux_value   = (uint8_t)(atomic_get(&g_mux_target) & 0x3);
        g_mux_next_ch = 0;
        atomic_set(&g_mux_applied, -1);
    }
    bool mux_wreg_in_frame = false;
    if (g_mux_next_ch < ADS131M08_MAX_CHANNELS)
    {
        uint8_t addr = (uint8_t)(ADS131M08_REG_CH0_CFG + 5U * g_mux_next_ch);
        uint16_t cmd = (uint16_t)(ADS131M08_CMD_WREG | ((uint16_t)addr << 7)); /* 1 register */
        uint16_t val = (uint16_t)(0x0000U | g_mux_value);                   /* MUX[1:0] */
        ads_driver.tx_buffer[0] = (uint8_t)(cmd >> 8);
        ads_driver.tx_buffer[1] = (uint8_t)(cmd & 0xFF);
        ads_driver.tx_buffer[2] = 0x00;
        ads_driver.tx_buffer[3] = (uint8_t)(val >> 8);
        ads_driver.tx_buffer[4] = (uint8_t)(val & 0xFF);
        ads_driver.tx_buffer[5] = 0x00;
        mux_wreg_in_frame = true;
    }
    struct spi_buf tx_bufs[] = {
        {.buf = ads_driver.tx_buffer, .len = ADS131M08_FRAME_BYTES}};
    struct spi_buf rx_bufs[] = {
        {.buf = ads_driver.rx_buffer, .len = ADS131M08_FRAME_BYTES}};
    struct spi_buf_set tx_set = {.buffers = tx_bufs, .count = 1};
    struct spi_buf_set rx_set = {.buffers = rx_bufs, .count = 1};
    LOG_DBG("ADS READER: spi_transceive len=%u", ADS131M08_FRAME_BYTES);
    int ret = spi_transceive(ads_driver.spi_dev, &ads_driver.spi_cfg, &tx_set, &rx_set);
    if (ret < 0)
    {
        ads_spi_errors++;
        LOG_ERR("ADS READER: spi_transceive failed: %d", ret);
        return; /* a pending mux WREG is simply retried with the next frame */
    }
    if (mux_wreg_in_frame)
    {
        if (++g_mux_next_ch >= ADS131M08_MAX_CHANNELS)
        {
            atomic_set(&g_mux_applied, g_mux_value);
        }
    }
    static int debug_counter = 0;
    debug_counter++;
    /* Parse 24-bit words from byte buffer: status first, then 8 channels */
    uint32_t offset = 0;
    uint32_t status24 = ((uint32_t)ads_driver.rx_buffer[offset] << 16) |
                        ((uint32_t)ads_driver.rx_buffer[offset + 1] << 8) |
                        ((uint32_t)ads_driver.rx_buffer[offset + 2]);
    ads_driver.current_sample.status = status24; /* Keep as-is (top 8 bits zero) */
    offset += 3;
    for (int ch = 0; ch < ADS131M08_MAX_CHANNELS; ch++)
    {
        uint32_t w24 = ((uint32_t)ads_driver.rx_buffer[offset] << 16) |
                       ((uint32_t)ads_driver.rx_buffer[offset + 1] << 8) |
                       ((uint32_t)ads_driver.rx_buffer[offset + 2]);
        offset += 3;
        /* Sign-extend 24-bit to 32-bit */
        if (w24 & 0x00800000)
            w24 |= 0xFF000000;
        else
            w24 &= 0x00FFFFFF;
        ads_driver.current_sample.channels[ch] = (int32_t)w24;
    }
    ads_driver.current_sample.sample_number = ++ads_driver.sample_count;
    ads_driver.current_sample.timestamp_us = last_drdy_timestamp_us; /* Use timestamp from ISR */

    /* TEST mode (config.txt `TEST = N`): discard the real ADS samples and feed a
     * synthetic pure-tone staircase — N seconds per tone, stepping 1,2,…,30 Hz
     * then looping. Identical signal on all 8 channels so the app spectrogram
     * shows a clean frequency "staircase". sample_number + timestamp_us above are
     * left untouched, so SD (BDF), BLE, dedup and backfill behave exactly as in
     * normal operation. Phase resets per block → each block is a clean single
     * tone. Amplitude 80000 counts (~±2.9 mV @gain4) stays inside the app's
     * ±4 mV int16 storage window (no clipping). sinf() uses the M4F FPU. */
    if (g_config.test_block_secs > 0)
    {
        const int fs = (g_config.sample_rate > 0) ? g_config.sample_rate : 250;
        const uint32_t block_len =
            (uint32_t)g_config.test_block_secs * (uint32_t)fs; /* samples/tone */
        const uint32_t sn = ads_driver.current_sample.sample_number;
        const uint32_t idx_in_block = block_len ? (sn % block_len) : sn;
        const uint32_t block = block_len ? (sn / block_len) : 0;
        const int freq_hz = 1 + (int)(block % 30U); /* 1..30 Hz, looping */
        const float phase = 2.0f * 3.14159265358979f * (float)freq_hz *
                            (float)idx_in_block / (float)fs;
        const int32_t val = (int32_t)lroundf(80000.0f * sinf(phase));
        for (int ch = 0; ch < ADS131M08_MAX_CHANNELS; ch++)
        {
            ads_driver.current_sample.channels[ch] = val;
        }
    }

    if (ads_driver.sample_count <= 10 || (ads_driver.sample_count % 100 == 0))
    {
        LOG_DBG("ADS READER: sample #%u status=0x%06X ch0=%d ch1=%d ch2=%d ch3=%d",
                ads_driver.current_sample.sample_number,
                (unsigned)ads_driver.current_sample.status,
                ads_driver.current_sample.channels[0],
                ads_driver.current_sample.channels[1],
                ads_driver.current_sample.channels[2],
                ads_driver.current_sample.channels[3]);
    }

    if (ads_driver.config.data_ready_callback)
    {
        LOG_DBG("ADS READER: invoking data_ready_callback (sample #%u)", ads_driver.current_sample.sample_number);
        ads_driver.config.data_ready_callback(&ads_driver.current_sample);
        ads_callbacks_invoked++;
    }
    ads_samples_processed++;
}

static void ads_reader_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("ADS READER THREAD: started (stack=%u bytes)", (unsigned)K_THREAD_STACK_SIZEOF(ads_reader_stack));
    for (;;)
    {
        /* Wait for DRDY signal */
        int sem_rc = k_sem_take(&ads_reader_sem, K_FOREVER);
        if (sem_rc != 0)
        {
            LOG_WRN("ADS READER THREAD: k_sem_take returned %d", sem_rc);
        }

        /* Drain any pending DRDY signals quickly to keep up under burst. Each
         * pending token is a conversion whose SPI frame we will NOT read — the
         * ADS131M08 has no readable FIFO, so those conversions are physically
         * lost (unavoidable under reader latency > 4 ms). */
        int drained = k_sem_count_get(&ads_reader_sem);
        for (int d = 0; d < drained; d++)
        {
            (void)k_sem_take(&ads_reader_sem, K_NO_WAIT);
        }
        if (drained > 0)
        {
            /* R1 FIX (data integrity analysis): advance the free-running
             * sn counter by the number of dropped conversions so the loss shows
             * up as an EXPLICIT sn gap. do_read() below then ++'s once more for
             * the frame it actually reads, so the read sample lands at
             * (old + drained + 1), leaving [old+1 .. old+drained] as a visible
             * gap. Previously sample_count only ++'d by 1 → N lost conversions
             * collapsed onto one sn with NO gap → silent time-compression
             * (sample_count/250 < wall-clock). Making it visible lets the phone
             * account for it instead of silently shortening the night. */
            const uint32_t gap_first = ads_driver.sample_count + 1;
            ads_driver.sample_count += (uint32_t)drained;
            ads_dropped_conversions += (uint32_t)drained;
            /* WRN (was DBG) so a lost-conversion gap is visible in RTT with its
             * sn range; rate-limited to one line per second. */
            static int64_t last_drop_log_ms;
            const int64_t now_ms = k_uptime_get();
            if (now_ms - last_drop_log_ms >= 1000)
            {
                last_drop_log_ms = now_ms;
                LOG_WRN("ADS READER: %d conversions lost (sn %u..%u, total %u) — reader blocked",
                        drained, gap_first, ads_driver.sample_count,
                        (unsigned)ads_dropped_conversions);
            }
        }

        /* Perform one frame read */
        ads_reader_do_read();

        // Occasionally report rates and counters
        if ((ads_samples_processed > 0) && (ads_samples_processed % 5000 == 0))
        {
            uint32_t irqs = drdy_irq_total_count;
            LOG_INF("ADS READER STATS: IRQS=%u, samples=%u, spi_errors=%u, callbacks=%u, dropped_conv=%u",
                    irqs, (unsigned)ads_samples_processed, (unsigned)ads_spi_errors, (unsigned)ads_callbacks_invoked,
                    (unsigned)ads_dropped_conversions);
        }
    }
}

/* Wait until the DRDY pin indicates the device is ready. Returns timeout error after ADS131M08_DRDY_TIMEOUT_MS.
 * Useful during initialization to ensure device has left reset/wakeup sequence.
 */
static ads131m08_status_t wait_for_drdy_ready(void)
{
    LOG_INF("Waiting for DRDY to go low (timeout %d ms)", ADS131M08_DRDY_TIMEOUT_MS);
    int timeout = ADS131M08_DRDY_TIMEOUT_MS;
    while (timeout-- > 0)
    {
        int pin_state = gpio_pin_get(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN);
        if (pin_state == 0)
        {
            return ADS131M08_OK;
        }
        k_sleep(K_MSEC(1));
    }
    int final_state = gpio_pin_get(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN);
    LOG_INF("DRDY did not become ready within timeout");
    return ADS131M08_ERROR_TIMEOUT;
}

ads131m08_status_t ads131m08_ti_init(const ads131m08_config_t *config)
{
    // Step 1: Try to read registers before init (may fail if device not ready)
    LOG_INF("=== BEFORE INIT: Attempting register read ===");
    // Try to initialize minimal SPI to attempt register read
    const struct device *spi_dev = DEVICE_DT_GET(ADS131M08_SPI_NODE);
    if (device_is_ready(spi_dev))
    {
        // Temporarily set up SPI config with 8-bit transfers for ADS131M08
        struct spi_config temp_cfg = {
            .operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8)),
            .frequency = ADS131M08_SPI_FREQUENCY,
            .slave = 0};
        const struct device *cs_port = DEVICE_DT_GET(ADS131M08_GPIO_CS_PORT);
        if (device_is_ready(cs_port))
        {
            temp_cfg.cs.gpio.port = cs_port;
            temp_cfg.cs.gpio.pin = ADS131M08_GPIO_CS_PIN;
            temp_cfg.cs.gpio.dt_flags = GPIO_ACTIVE_LOW;
        }
    }

    if (!config)
    {
        return ADS131M08_ERROR_INVALID_PARAM;
    }
    LOG_INF("ADS TI: zeroing driver state");
    memset(&ads_driver, 0, sizeof(ads_driver));
    ads_driver.config = *config;
    ads_driver.first_read = true;
    k_sleep(K_MSEC(150));
    int ret;
    ads_driver.spi_dev = DEVICE_DT_GET(ADS131M08_SPI_NODE);
    if (!device_is_ready(ads_driver.spi_dev))
    {
        LOG_ERR("ADS TI: SPI device not ready");
        return ADS131M08_ERROR_NOT_READY;
    }
    ads_driver.gpio_dev = DEVICE_DT_GET(ADS131M08_GPIO_DRDY_PORT);
    if (!device_is_ready(ads_driver.gpio_dev))
    {
        LOG_ERR("ADS TI: GPIO device not ready");
        return ADS131M08_ERROR_NOT_READY;
    }
    ads_driver.spi_cfg.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8));
    ads_driver.spi_cfg.frequency = ADS131M08_SPI_FREQUENCY;
    ads_driver.spi_cfg.slave = 0;
    const struct device *cs_port = DEVICE_DT_GET(ADS131M08_GPIO_CS_PORT);
    if (!device_is_ready(cs_port))
    {
        LOG_ERR("ADS TI: CS GPIO port not ready");
        return ADS131M08_ERROR_NOT_READY;
    }
    ads_driver.spi_cfg.cs.gpio.port = cs_port;
    ads_driver.spi_cfg.cs.gpio.pin = ADS131M08_GPIO_CS_PIN;
    ads_driver.spi_cfg.cs.gpio.dt_flags = GPIO_ACTIVE_LOW;
    LOG_INF("ADS TI: SPI configured (freq=%u, cs=%d)", (unsigned)ads_driver.spi_cfg.frequency, ADS131M08_GPIO_CS_PIN);
    ret = gpio_pin_configure(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN,
                             GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0)
    {
        return ADS131M08_ERROR;
    }
    int drdy_after_config = gpio_pin_get(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN);
    (void)drdy_after_config;
    const struct device *sync_port = DEVICE_DT_GET(ADS131M08_GPIO_SYNC_PORT);
    if (!device_is_ready(sync_port))
    {
        return ADS131M08_ERROR_NOT_READY;
    }
    ret = gpio_pin_configure(sync_port, ADS131M08_GPIO_SYNC_PIN, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
        return ADS131M08_ERROR;
    }
    gpio_pin_set(sync_port, ADS131M08_GPIO_SYNC_PIN, 0);
    k_sleep(K_MSEC(1));
    gpio_pin_set(sync_port, ADS131M08_GPIO_SYNC_PIN, 1);
    k_sleep(K_MSEC(10));
    int drdy_initial = gpio_pin_get(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN);
    (void)drdy_initial;
    gpio_pin_set(sync_port, ADS131M08_GPIO_SYNC_PIN, 0);
    k_sleep(K_MSEC(1));
    (void)gpio_pin_get(sync_port, ADS131M08_GPIO_SYNC_PIN);
    gpio_pin_set(sync_port, ADS131M08_GPIO_SYNC_PIN, 1);
    k_sleep(K_MSEC(1));
    (void)gpio_pin_get(sync_port, ADS131M08_GPIO_SYNC_PIN);
    for (int i = 0; i < 3; i++)
    {
        gpio_pin_set(sync_port, ADS131M08_GPIO_SYNC_PIN, 0);
        k_sleep(K_MSEC(2));
        gpio_pin_set(sync_port, ADS131M08_GPIO_SYNC_PIN, 1);
        k_sleep(K_MSEC(20));
        int drdy_check = gpio_pin_get(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN);
        if (drdy_check == 1)
        {
            break;
        }
    }
    int retry_count = 0;
    const int max_retries = 3;
    do
    {
        if (retry_count > 0)
        {
            k_sleep(K_MSEC(200));
        }
        ret = wait_for_drdy_ready();
        retry_count++;
    } while (ret != ADS131M08_OK && retry_count <= max_retries);
    if (ret != ADS131M08_OK)
    {
        return ret;
    }

    // Step 3: Send RESET command using full 10-word frame (per datasheet requirement)
    LOG_INF("Sending RESET command (0x0011) using full 10-word frame");

    // CRITICAL: Per ADS131M08 datasheet, RESET command requires a FULL 10-word frame
    // Frame format: [RESET_CMD][CH0_DATA][CH1_DATA]...[CH7_DATA][CRC]
    // Each word is 24-bit (3 bytes), so total frame = 30 bytes
    uint8_t reset_full_frame[30] = {
        // Word 0: RESET command (0x0011) MSB-aligned in 24 bits
        0x00, 0x11, 0x00,
        // Words 1-8: Channel data (all zeros for reset)
        0x00, 0x00, 0x00, // CH0
        0x00, 0x00, 0x00, // CH1
        0x00, 0x00, 0x00, // CH2
        0x00, 0x00, 0x00, // CH3
        0x00, 0x00, 0x00, // CH4
        0x00, 0x00, 0x00, // CH5
        0x00, 0x00, 0x00, // CH6
        0x00, 0x00, 0x00, // CH7
        // Word 9: CRC (zeros for reset)
        0x00, 0x00, 0x00};
    uint8_t reset_rx_frame[30] = {0};

    struct spi_buf reset_tx_buf = {
        .buf = reset_full_frame,
        .len = 30 // 30 bytes = 10 × 24-bit words (full frame)
    };
    struct spi_buf reset_rx_buf = {
        .buf = reset_rx_frame,
        .len = 30 // 30 bytes = 10 × 24-bit words (full frame)
    };
    struct spi_buf_set reset_tx_set = {.buffers = &reset_tx_buf, .count = 1};
    struct spi_buf_set reset_rx_set = {.buffers = &reset_rx_buf, .count = 1};

    struct spi_config reset_spi_cfg = {
        .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8),
        .frequency = ADS131M08_SPI_FREQUENCY,
        .slave = 0,
    };
    reset_spi_cfg.cs = ads_driver.spi_cfg.cs;

    int reset_ret = spi_transceive(ads_driver.spi_dev, &reset_spi_cfg, &reset_tx_set, &reset_rx_set);
    if (reset_ret < 0)
    {
        LOG_ERR("RESET full frame SPI failed: %d", reset_ret);
        return ADS131M08_ERROR_SPI;
    }

    LOG_INF("RESET full frame sent (30 bytes), first response word: %02X %02X %02X",
            reset_rx_frame[0], reset_rx_frame[1], reset_rx_frame[2]);

    // Wait for reset to complete (tREGACQ timing requirement from datasheet)
    LOG_INF("Waiting for tREGACQ (50ms) after RESET...");
    k_sleep(K_MSEC(50));

    // Now send a short NULL command to get the RESET acknowledgment
    // The response to the RESET command comes in the NEXT SPI transaction
    LOG_INF("Sending NULL command to receive RESET acknowledgment");
    uint8_t null_cmd[3] = {0x00, 0x00, 0x00}; // NULL command
    uint8_t reset_ack_response[3] = {0};

    struct spi_buf null_tx_buf = {.buf = null_cmd, .len = 3};
    struct spi_buf null_rx_buf = {.buf = reset_ack_response, .len = 3};
    struct spi_buf_set null_tx_set = {.buffers = &null_tx_buf, .count = 1};
    struct spi_buf_set null_rx_set = {.buffers = &null_rx_buf, .count = 1};

    int null_ret = spi_transceive(ads_driver.spi_dev, &reset_spi_cfg, &null_tx_set, &null_rx_set);
    if (null_ret == 0)
    {
        uint16_t reset_ack = (reset_ack_response[0] << 8) | reset_ack_response[1];
        LOG_INF("RESET acknowledgment received: 0x%04X (expected: 0xFF28)", reset_ack);
        if (reset_ack == 0xFF28)
        {
            LOG_INF("✓ RESET command successfully acknowledged!");
        }
        else
        {
            LOG_WRN("⚠ RESET acknowledgment mismatch - device may not have reset properly");
        }
    }

    // Validate that registers are now in their correct reset state
    LOG_INF("Validating register reset state...");
    validate_register_values();

    // Wait for DRDY after reset
    ret = wait_for_drdy_ready();
    if (ret != ADS131M08_OK)
    {
        LOG_INF("DRDY not ready after reset");
        return ret;
    }

    // Verify that the device is unlocked after reset (UNLOCK command not needed after proper RESET)
    uint16_t status_reg;
    ret = read_register_ti(ADS131M08_REG_STATUS, &status_reg, 24);
    if (ret == ADS131M08_OK)
    {
        bool is_locked = (status_reg >> 15) & 0x01;
        LOG_INF("Post-reset STATUS=0x%04X, LOCK bit=%s",
                status_reg, is_locked ? "LOCKED" : "UNLOCKED");

        if (is_locked)
        {
            LOG_ERR("Device is still locked after RESET - this should not happen!");
            return ADS131M08_ERROR;
        }
        else
        {
            LOG_INF("✓ Device is properly unlocked after RESET");
        }
    }
    else
    {
        LOG_ERR("Failed to read STATUS register after RESET");
        return ADS131M08_ERROR;
    }

    // Setup GPIO interrupt and high-priority reader thread
    gpio_init_callback(&ads_driver.drdy_cb, drdy_interrupt_handler, BIT(ADS131M08_GPIO_DRDY_PIN));
    ret = gpio_add_callback(ads_driver.gpio_dev, &ads_driver.drdy_cb);
    if (ret < 0)
    {
        LOG_ERR("Failed to add GPIO callback for DRDY");
        return ADS131M08_ERROR;
    }

    // Initialize semaphore and create reader thread BEFORE enabling interrupts
    drdy_irq_total_count = 0;
    ads_driver.sampling_active = true;
    k_sem_init(&ads_reader_sem, 0, 1000);
    k_thread_create(&ads_reader_thread, ads_reader_stack, K_THREAD_STACK_SIZEOF(ads_reader_stack),
                    ads_reader_thread_entry, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
    k_thread_name_set(&ads_reader_thread, "ads_reader");

    // Mark driver as ready for data acquisition
    ads_driver.driver_ready = true;

    k_sleep(K_MSEC(100)); // Wait a bit before proceeding
    LOG_INF("=== Set CLOCK values to FF94 (1111 1111 1001 0100) ===");
    LOG_INF("Set to all channels enabled, internal crystal disabled (save energy), external reference disabled (change later if needed), OSR=4096, PWR=Very-low-power");
    uint16_t clock_value = 0xFF94;
    ret = write_register_ti(ADS131M08_REG_CLOCK, clock_value, 24);
    k_sleep(K_MSEC(100)); // Wait a bit before proceeding
    ads_log_registers();
 
    k_sleep(K_MSEC(100)); // Wait a bit before proceeding
    /* Build GAIN1 (CH0-3) / GAIN2 (CH4-7) from the per-channel PGA codes in
     * `config->gain`. Each channel occupies the low 3 bits of one nibble:
     *   GAIN1 = [CH3<<12 | CH2<<8 | CH1<<4 | CH0]
     *   GAIN2 = [CH7<<12 | CH6<<8 | CH5<<4 | CH4]
     * The codes are set from the runtime ADS_GAIN config (default ×4 → code 2
     * → register 0x2222). Falls back to ×4 if no config was passed. */
    uint16_t gain1_value = 0;
    uint16_t gain2_value = 0;
    for (int ch = 0; ch < 4; ch++)
    {
        uint16_t code = config ? (uint16_t)(config->gain[ch] & 0x07) : 0x2;
        gain1_value |= (uint16_t)(code << (ch * 4));
    }
    for (int ch = 0; ch < 4; ch++)
    {
        uint16_t code = config ? (uint16_t)(config->gain[ch + 4] & 0x07) : 0x2;
        gain2_value |= (uint16_t)(code << (ch * 4));
    }
    LOG_INF("=== Set GAIN1=0x%04X GAIN2=0x%04X (from config) ===",
            gain1_value, gain2_value);
    ret = write_register_ti(ADS131M08_REG_GAIN1, gain1_value, 24);
    k_sleep(K_MSEC(100)); // Wait a bit before proceeding
    ret = write_register_ti(ADS131M08_REG_GAIN2, gain2_value, 24);
    k_sleep(K_MSEC(100)); // Wait a bit before proceeding
    ads_log_registers();

    // Start periodic register dumping for monitoring
    LOG_INF("Starting periodic register dump timer (every 5 seconds)");
    k_work_init_delayable(&ads_driver.periodic_dump_work, periodic_register_dump_handler);
    k_work_schedule(&ads_driver.periodic_dump_work, K_SECONDS(5));

    ads_log_registers(); // korrekt

    LOG_INF("=== ADS131M08 TI driver successfully initialized ===");
    return ADS131M08_OK;
}

ads131m08_status_t ads131m08_ti_start_sampling(void)
{
    LOG_INF("Start sampling requested");
    if (!ads_driver.driver_ready)
    {
        return ADS131M08_ERROR_NOT_READY;
    }
    int ret = gpio_pin_interrupt_configure(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN,
                                           GPIO_INT_EDGE_FALLING);
    // int ret = gpio_pin_interrupt_configure(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN,
    //                                        GPIO_INT_DISABLE);
    if (ret < 0)
    {
        return ADS131M08_ERROR;
    }
    ads_driver.sampling_active = true;
    ads_driver.first_read = true;
    // R2 FIX (data integrity analysis): do NOT reset sample_count here.
    // main.c restarts sampling mid-recording on every audio/mic/WAV/stimulus
    // event while the SAME BDF/.SNX stays open. Zeroing the counter made later
    // samples restart at sn=1 → non-monotonic .SNX, duplicate sns, corrupted
    // backfill bisect. The counter is a FREE-RUNNING monotonic value for the
    // whole boot/recording lifetime; it is zeroed exactly once, by the
    // memset(&ads_driver, 0, ...) in ads131m08_ti_init(). Never rewind it.

    // DIAGNOSTIC: Reset diagnostic counters (these are since-restart diagnostics
    // only; they do NOT feed the on-disk sample numbering).
    drdy_irq_total_count = 0;
    LOG_INF("Sampling started, sample_count NOT reset (free-running=%u), diagnostic counters reset",
            ads_driver.sample_count);
    LOG_INF("DIAGNOSTIC MODE: Will log DRDY timing every 25 IRQs, IRQ vs sample counts every 50 samples");

    return ADS131M08_OK;
}

ads131m08_status_t ads131m08_ti_stop_sampling(void)
{
    LOG_INF("Stop sampling requested");
    if (!ads_driver.driver_ready)
    {
        return ADS131M08_ERROR_NOT_READY;
    }
    ads_driver.sampling_active = false;
    gpio_pin_interrupt_configure(ads_driver.gpio_dev, ADS131M08_GPIO_DRDY_PIN, GPIO_INT_DISABLE);
    LOG_INF("Sampling stopped");
    return ADS131M08_OK;
}

ads131m08_status_t ads131m08_ti_request_input_mux(uint8_t mux)
{
    if (mux > 3)
    {
        return ADS131M08_ERROR;
    }
    atomic_set(&g_mux_target, mux);
    atomic_set(&g_mux_request, 1);
    LOG_INF("ADS input mux -> %u requested (applied over the next 8 frames)", mux);
    return ADS131M08_OK;
}

int ads131m08_ti_get_input_mux(void)
{
    return (int)atomic_get(&g_mux_applied);
}

uint32_t ads131m08_ti_get_sample_count(void)
{
    return ads_driver.sample_count;
}

bool ads131m08_ti_is_sampling(void)
{
    return ads_driver.sampling_active;
}

/* Public wrapper: read device id register and return its value. Adds logging for diagnostics. */
ads131m08_status_t ads131m08_ti_read_device_id(uint16_t *device_id)
{
    LOG_INF("Reading device ID register");
    if (!device_id)
    {
        LOG_INF("Device ID pointer is NULL");
        return ADS131M08_ERROR_INVALID_PARAM;
    }
    return read_register_ti(ADS131M08_REG_ID, device_id, 24);
}

/* Public wrapper to read a register using the internal helper. Adds simple logging.
 */
ads131m08_status_t ads131m08_ti_read_register(uint8_t addr, uint16_t *data, uint8_t word_length)
// does not work
{
    LOG_INF("Public read register wrapper addr=0x%02X len=%u", addr, word_length);
    return read_register_ti(addr, data, word_length);
}

ads131m08_status_t ads131m08_ti_write_register(uint8_t addr, uint16_t data, uint8_t word_length)
{
    LOG_INF("Public write register wrapper addr=0x%02X data=0x%04X len=%u", addr, data, word_length);
    return write_register_ti(addr, data, word_length);
}

void ads131m08_ti_debug_print_registers(void)
{
    if (!ads_driver.driver_ready)
    {
        LOG_INF("Driver not ready - cannot dump registers");
        return;
    }
    LOG_INF("Debug: printing all ADS131M08 registers");
    debug_print_all_registers();
}

void ads131m08_ti_start_periodic_register_dump(void)
{
    LOG_INF("Starting/restarting periodic register dump");
    k_work_cancel_delayable(&ads_driver.periodic_dump_work);
    k_work_schedule(&ads_driver.periodic_dump_work, K_SECONDS(5));
}

void ads131m08_ti_stop_periodic_register_dump(void)
{
    LOG_INF("Stopping periodic register dump");
    k_work_cancel_delayable(&ads_driver.periodic_dump_work);
}

/* DIAGNOSTIC: Test different OSR values to find optimal data rate */
ads131m08_status_t ads131m08_ti_test_osr_rates(void)
{
    if (!ads_driver.driver_ready)
    {
        LOG_ERR("Driver not ready for OSR testing");
        return ADS131M08_ERROR_NOT_READY;
    }

    LOG_INF("=== TESTING DIFFERENT OSR VALUES ===");

    // Test OSR values and their expected rates
    struct
    {
        uint16_t osr_value;
        const char *name;
        uint32_t expected_hz;
    } osr_tests[] = {
        {ADS131M08_CLOCK_OSR_256, "OSR_256", 128},
        {ADS131M08_CLOCK_OSR_512, "OSR_512", 64},
        {ADS131M08_CLOCK_OSR_1024, "OSR_1024", 32},
        {ADS131M08_CLOCK_OSR_2048, "OSR_2048", 16},
        {ADS131M08_CLOCK_OSR_4096, "OSR_4096", 8}};

    for (int i = 0; i < 5; i++)
    {
        LOG_INF("Testing %s (expected %u Hz)...", osr_tests[i].name, osr_tests[i].expected_hz);

        // Configure with test OSR
        ads131m08_status_t ret = write_register_ti(ADS131M08_REG_CLOCK,
                                                   ADS131M08_CLOCK_CH_EN_ALL | osr_tests[i].osr_value |
                                                       ADS131M08_CLOCK_PWR_VLP | ADS131M08_CLOCK_XTAL_DIS,
                                                   24);
        if (ret == ADS131M08_OK)
        {
            // Read back and verify
            uint16_t readback;
            ret = read_register_ti(ADS131M08_REG_CLOCK, &readback, 24);
            if (ret == ADS131M08_OK)
            {
                uint8_t actual_osr = (readback >> 2) & 0x07;
                LOG_INF("%s: CLOCK=0x%04X, OSR_bits=%u", osr_tests[i].name, readback, actual_osr);
            }
        }
        k_msleep(100); // Small delay between tests
    }

    LOG_INF("=== OSR TEST COMPLETE - Check which setting gives best rate ===");
    return ADS131M08_OK;
}
