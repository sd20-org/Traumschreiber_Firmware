#include "ads131m08.h"
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ads131m08, LOG_LEVEL_INF);

/* Driver local runtime state and buffers. Contains SPI and GPIO device handles
 * configuration used across the driver implementation. Kept as a single static
 * structure to simplify state management in this small driver.
 */
#define SPIOP_ADS131 (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

static struct
{
    const struct device *spi_bus;
    struct spi_config spi_cfg;
    const struct device *drdy_port;
    const struct device *sync_port;
    struct gpio_callback drdy_cb;
    struct k_work data_work;
    ads131m08_config_t config;
    bool is_initialized;
    bool is_sampling;
    uint32_t sample_count;
    volatile bool interrupt_flag;
    volatile bool busy;
} ads_data = {
    .is_initialized = false,
    .is_sampling = false,
    .sample_count = 0,
    .interrupt_flag = false,
    .busy = false};

/* Forward declarations */
static void drdy_isr_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins);
static void data_work_handler(struct k_work *work);
static ads131m08_status_t spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, size_t len);

/**
 * @brief Convert 24-bit two's complement to 32-bit signed
 *
 * The ADS131M08 provides 24-bit channel words. For convenience the driver
 * sign-extends these into 32-bit signed integers for application use.
 */
static int32_t convert_sample(uint32_t raw_data)
{
    /* Sign extend 24-bit to 32-bit */
    if (raw_data & 0x800000)
    {
        return (int32_t)(raw_data | 0xFF000000);
    }
    else
    {
        return (int32_t)(raw_data & 0x00FFFFFF);
    }
}

/**
 * @brief Perform SPI transfer
 *
 * Wrapper around the Zephyr spi_transceive call. Adds minimal logging for
 * diagnostics and translates return codes to the driver-specific enum.
 */
static ads131m08_status_t spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
    LOG_INF("SPI transfer requested, length=%zu", len);
    struct spi_buf tx = {
        .buf = (void *)tx_buf,
        .len = len};
    struct spi_buf rx = {
        .buf = rx_buf,
        .len = len};

    struct spi_buf_set tx_set = {
        .buffers = &tx,
        .count = 1};
    struct spi_buf_set rx_set = {
        .buffers = &rx,
        .count = 1};

    int ret = spi_transceive(ads_data.spi_bus, &ads_data.spi_cfg, &tx_set, &rx_set);
    return (ret == 0) ? ADS131M08_OK : ADS131M08_ERROR;
}

/**
 * @brief DRDY interrupt handler
 *
 * Quick ISR implementation: set a flag and schedule work to perform the
 * actual SPI transaction in thread context. Avoid lengthy processing in ISR.
 */
static void drdy_isr_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    LOG_INF("DRDY ISR fired");
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    ads_data.interrupt_flag = true;

    /* Schedule work to read the data (don't do SPI in interrupt context) */
    k_work_submit(&ads_data.data_work);
}

/**
 * @brief Work handler to read ADC data frame
 *
 * Executed in a system workqueue context. Performs SPI transaction for the
 * full frame, converts channel data to signed values and forwards to the
 * registered callback (if any). Also logs sample counts and rate estimates
 * at intervals to aid RTT-based debugging.
 */
static void data_work_handler(struct k_work *work)
{
    LOG_INF("Data work handler invoked. sampling=%s busy=%s", ads_data.is_sampling ? "YES" : "NO", ads_data.busy ? "YES" : "NO");
    ARG_UNUSED(work);

    if (!ads_data.is_sampling || ads_data.busy)
    {
        return;
    }

    ads_data.busy = true;
    ads_data.interrupt_flag = false;

    /* Read ADC data frame */
    uint8_t tx_buf[ADS131M08_FRAME_BYTES] = {0};
    uint8_t rx_buf[ADS131M08_FRAME_BYTES] = {0};

    if (spi_transfer(tx_buf, rx_buf, sizeof(tx_buf)) != ADS131M08_OK)
    {
        ads_data.busy = false;
        return;
    }

    /* Parse the data */
    ads131m08_sample_t sample;
    uint8_t *ptr = rx_buf;

    /* First word is status */
    sample.status = (ptr[0] << 16) | (ptr[1] << 8) | ptr[2];
    ptr += 3;

    /* Next 8 words are channel data */
    for (int ch = 0; ch < ADS131M08_MAX_CHANNELS; ch++)
    {
        uint32_t raw_data = (ptr[0] << 16) | (ptr[1] << 8) | ptr[2];
        sample.channels[ch] = convert_sample(raw_data);
        ptr += 3;
    }

    /* Update timestamp and counters */
    sample.timestamp = k_uptime_get();
    ads_data.sample_count++;

    static unsigned int loop_count = 0;
    loop_count++;
    if ((loop_count % 50) == 0)
    {
        LOG_INF("=== Status Update #%u ===", ++loop_count);
        LOG_INF("Total samples: %u", ads_data.sample_count);
        /* crude sample rate estimate placeholder */
        LOG_INF("Sampling active: %s", ads_data.is_sampling ? "YES" : "NO");
    }

    /* Call user callback if configured */
    if (ads_data.config.data_ready_callback)
    {
        ads_data.config.data_ready_callback(&sample);
    }

    ads_data.busy = false;
}

/* Public API Implementation */

ads131m08_status_t ads131m08_read_register(uint8_t reg, uint16_t *value)
{
    if (!ads_data.is_initialized || !value)
    {
        return ADS131M08_ERROR;
    }

    /* First transaction: send read command */
    uint8_t tx_buf[3] = {(ADS131M08_OPCODE_RREG >> 8) | reg, 0x00, 0x00};
    uint8_t rx_buf[3] = {0};

    if (spi_transfer(tx_buf, rx_buf, 3) != ADS131M08_OK)
    {
        return ADS131M08_ERROR;
    }

    /* Second transaction: get the data */
    memset(tx_buf, 0, 3);
    if (spi_transfer(tx_buf, rx_buf, 3) != ADS131M08_OK)
    {
        return ADS131M08_ERROR;
    }

    *value = (rx_buf[0] << 8) | rx_buf[1];
    return ADS131M08_OK;
}

ads131m08_status_t ads131m08_write_register(uint8_t reg, uint16_t value)
{
    if (!ads_data.is_initialized)
    {
        return ADS131M08_ERROR;
    }

    /* Send write command and data in one transaction */
    uint8_t tx_buf[6] = {
        (ADS131M08_OPCODE_WREG >> 8) | reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
        0x00, 0x00, 0x00};
    uint8_t rx_buf[6] = {0};

    return spi_transfer(tx_buf, rx_buf, 6);
}

ads131m08_status_t ads131m08_read_id(uint16_t *id)
{
    return ads131m08_read_register(ADS131M08_REG_ID, id);
}

ads131m08_status_t ads131m08_reset(void)
{
    uint8_t tx_buf[3] = {
        (uint8_t)(ADS131M08_OPCODE_RESET >> 8),
        (uint8_t)(ADS131M08_OPCODE_RESET & 0xFF),
        0x00};
    uint8_t rx_buf[3] = {0};

    return spi_transfer(tx_buf, rx_buf, 3);
}

static ads131m08_status_t configure_power_mode(void)
{
    uint16_t clock_reg = ADS131M08_CLOCK_XTAL_DIS |
                         ADS131M08_CLOCK_OSR_4096 |
                         ADS131M08_CLOCK_PWR_VLP;

    ads131m08_status_t ret = ads131m08_write_register(ADS131M08_REG_CLOCK, clock_reg);
    if (ret != ADS131M08_OK)
    {
        return ret;
    }

    uint16_t read_back;
    ret = ads131m08_read_register(ADS131M08_REG_CLOCK, &read_back);
    if (ret != ADS131M08_OK)
    {
        return ret;
    }

    return ADS131M08_OK;
}

static ads131m08_status_t configure_sampling(void)
{
    uint16_t mode_reg = ADS131M08_MODE_WLENGTH_24;
    ads131m08_status_t ret = ads131m08_write_register(ADS131M08_REG_MODE, mode_reg);
    if (ret != ADS131M08_OK)
    {
        return ret;
    }

    for (int ch = 0; ch < ADS131M08_MAX_CHANNELS; ch++)
    {
        ret = ads131m08_write_register(ADS131M08_REG_CH0_CFG + ch, 0x0000);
        if (ret != ADS131M08_OK)
        {
            return ret;
        }
    }

    return ADS131M08_OK;
}

ads131m08_status_t ads131m08_init(const ads131m08_config_t *config)
{
    LOG_INF("ads131m08_init() entry");
    if (ads_data.is_initialized)
    {
        return ADS131M08_OK;
    }

    if (config)
    {
        ads_data.config = *config;
    }

    ads_data.spi_bus = DEVICE_DT_GET(DT_NODELABEL(spi0));
    if (!device_is_ready(ads_data.spi_bus))
    {
        return ADS131M08_ERROR;
    }
    ads_data.spi_cfg.operation = SPIOP_ADS131;
    ads_data.spi_cfg.frequency = 1000000;
    ads_data.spi_cfg.slave = 0;
    const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0))
    {
        return ADS131M08_ERROR;
    }
    ads_data.spi_cfg.cs.gpio.port = gpio0;
    ads_data.spi_cfg.cs.gpio.pin = 3;
    ads_data.spi_cfg.cs.gpio.dt_flags = GPIO_ACTIVE_LOW;
    ads_data.drdy_port = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    ads_data.sync_port = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    if (!device_is_ready(ads_data.drdy_port) || !device_is_ready(ads_data.sync_port))
    {
        return ADS131M08_ERROR;
    }
    int ret = gpio_pin_configure(ads_data.drdy_port, 10, GPIO_INPUT);
    if (ret < 0)
    {
        return ADS131M08_ERROR;
    }
    ret = gpio_pin_configure(ads_data.sync_port, 11, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
        return ADS131M08_ERROR;
    }
    k_work_init(&ads_data.data_work, data_work_handler);
    gpio_init_callback(&ads_data.drdy_cb, drdy_isr_handler, BIT(10));
    ret = gpio_add_callback(ads_data.drdy_port, &ads_data.drdy_cb);
    if (ret < 0)
    {
        return ADS131M08_ERROR;
    }
    gpio_pin_set(ads_data.sync_port, 11, 0);
    k_usleep(100);
    gpio_pin_set(ads_data.sync_port, 11, 1);
    k_msleep(10);
    uint16_t device_id;
    if (ads131m08_read_id(&device_id) != ADS131M08_OK)
    {
        return ADS131M08_ERROR;
    }
    if (configure_power_mode() != ADS131M08_OK)
    {
        return ADS131M08_ERROR;
    }
    if (configure_sampling() != ADS131M08_OK)
    {
        return ADS131M08_ERROR;
    }
    ads_data.is_initialized = true;
    ads_data.sample_count = 0;
    LOG_INF("ADS131M08 initialization completed successfully");

    return ADS131M08_OK;
}

ads131m08_status_t ads131m08_start_sampling(void)
{
    LOG_INF("ads131m08_start_sampling() requested");
    if (!ads_data.is_initialized)
    {
        return ADS131M08_ERROR;
    }
    if (ads_data.is_sampling)
    {
        return ADS131M08_OK;
    }
    while (gpio_pin_get(ads_data.drdy_port, 10) == 0)
    {
        k_busy_wait(10);
    }
    int ret = gpio_pin_interrupt_configure(ads_data.drdy_port, 10, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0)
    {
        return ADS131M08_ERROR;
    }
    uint8_t tx_buf[3] = {
        (uint8_t)(ADS131M08_OPCODE_WAKEUP >> 8),
        (uint8_t)(ADS131M08_OPCODE_WAKEUP & 0xFF),
        0x00};
    uint8_t rx_buf[3] = {0};
    if (spi_transfer(tx_buf, rx_buf, 3) != ADS131M08_OK)
    {
        return ADS131M08_ERROR;
    }
    ads_data.is_sampling = true;
    ads_data.sample_count = 0;
    ads_data.interrupt_flag = false;
    ads_data.busy = false;
    LOG_INF("ads131m08 sampling started");

    return ADS131M08_OK;
}

ads131m08_status_t ads131m08_stop_sampling(void)
{
    LOG_INF("ads131m08_stop_sampling() requested");
    if (!ads_data.is_sampling)
    {
        return ADS131M08_OK;
    }
    gpio_pin_interrupt_configure(ads_data.drdy_port, 10, GPIO_INT_DISABLE);
    uint8_t tx_buf[3] = {
        (uint8_t)(ADS131M08_OPCODE_STANDBY >> 8),
        (uint8_t)(ADS131M08_OPCODE_STANDBY & 0xFF),
        0x00};
    uint8_t rx_buf[3] = {0};
    spi_transfer(tx_buf, rx_buf, 3);
    ads_data.is_sampling = false;
    LOG_INF("ads131m08 sampling stopped");

    return ADS131M08_OK;
}

uint32_t ads131m08_get_sample_count(void)
{
    return ads_data.sample_count;
}

bool ads131m08_is_sampling(void)
{
    return ads_data.is_sampling;
}