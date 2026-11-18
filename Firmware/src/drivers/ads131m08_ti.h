#ifndef ADS131M08_TI_H
#define ADS131M08_TI_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

/* ADS131M08 Register Addresses */
#define ADS131M08_REG_ID 0x00
#define ADS131M08_REG_STATUS 0x01
#define ADS131M08_REG_MODE 0x02
#define ADS131M08_REG_CLOCK 0x03
#define ADS131M08_REG_GAIN1 0x04
#define ADS131M08_REG_GAIN2 0x05
#define ADS131M08_REG_CFG 0x06
#define ADS131M08_REG_THRSHLD_MSB 0x07
#define ADS131M08_REG_THRSHLD_LSB 0x08
#define ADS131M08_REG_CH0_CFG 0x09
#define ADS131M08_REG_CH0_OCAL_MSB 0x0A
#define ADS131M08_REG_CH0_OCAL_LSB 0x0B
#define ADS131M08_REG_CH0_GCAL_MSB 0x0C
#define ADS131M08_REG_CH0_GCAL_LSB 0x0D
#define ADS131M08_REG_CH1_CFG 0x0E
#define ADS131M08_REG_CH1_OCAL_MSB 0x0F
#define ADS131M08_REG_CH1_OCAL_LSB 0x10
#define ADS131M08_REG_CH1_GCAL_MSB 0x11
#define ADS131M08_REG_CH1_GCAL_LSB 0x12
/* ... Additional channel registers follow same pattern */

/* SPI Command Opcodes */
#define ADS131M08_CMD_NULL 0x0000
#define ADS131M08_CMD_RESET 0x0011
#define ADS131M08_CMD_STANDBY 0x0022
#define ADS131M08_CMD_WAKEUP 0x0033
#define ADS131M08_CMD_LOCK 0x0555
#define ADS131M08_CMD_UNLOCK 0x0655
#define ADS131M08_CMD_RREG 0xA000
#define ADS131M08_CMD_WREG 0x6000

/* Register Bit Masks - Based on TI Documentation */
/* MODE Register (0x02) */
#define ADS131M08_MODE_RESET 0x8000
#define ADS131M08_MODE_WLENGTH_16BIT 0x0000
#define ADS131M08_MODE_WLENGTH_24BIT 0x0100
#define ADS131M08_MODE_WLENGTH_32BIT 0x0200
#define ADS131M08_MODE_WLENGTH_32BIT_SE 0x0300 /* Sign Extended */
#define ADS131M08_MODE_TIMEOUT_ENABLE 0x0010
#define ADS131M08_MODE_DRDY_PULSE 0x0008
#define ADS131M08_MODE_DRDY_NEG 0x0004

/* CLOCK Register (0x03) - Based on Official ADS131M08 Datasheet */
#define ADS131M08_CLOCK_CH_EN_ALL 0xFF00  /* Enable all channels (bits 15-8) */
#define ADS131M08_CLOCK_CH_DIS_ALL 0x0000 /* Disable all channels */

/* OSR bits [4:2] - Oversampling Ratio (3 bits) */
#define ADS131M08_CLOCK_OSR_128 0x0000   /* OSR = 128 (000b) */
#define ADS131M08_CLOCK_OSR_256 0x0004   /* OSR = 256 (001b) */
#define ADS131M08_CLOCK_OSR_512 0x0008   /* OSR = 512 (010b) */
#define ADS131M08_CLOCK_OSR_1024 0x000C  /* OSR = 1024 (011b) - default */
#define ADS131M08_CLOCK_OSR_2048 0x0010  /* OSR = 2048 (100b) */
#define ADS131M08_CLOCK_OSR_4096 0x0014  /* OSR = 4096 (101b) */
#define ADS131M08_CLOCK_OSR_8192 0x0018  /* OSR = 8192 (110b) */
#define ADS131M08_CLOCK_OSR_16384 0x001C /* OSR = 16384 (111b) */

/* PWR bits [1:0] - Power Mode (2 bits) */
#define ADS131M08_CLOCK_PWR_VLP 0x0000 /* Very Low Power (00b) */
#define ADS131M08_CLOCK_PWR_LP 0x0001  /* Low Power (01b) */
#define ADS131M08_CLOCK_PWR_HR 0x0002  /* High Resolution (10b) - default */
#define ADS131M08_CLOCK_PWR_HR2 0x0003 /* High Resolution (11b) - same as 10b */

/* Bit 6 - External Reference Control */
#define ADS131M08_CLOCK_EXTREF_EN 0x0040  /* Enable external reference (bit 6) */
#define ADS131M08_CLOCK_EXTREF_DIS 0x0000 /* Disable external reference (default) */

/* Bit 7 - Internal Oscillator Control */
#define ADS131M08_CLOCK_XTAL_DIS 0x0080 /* Disable internal oscillator (bit 7) */
#define ADS131M08_CLOCK_XTAL_EN 0x0000  /* Enable internal oscillator (default) */

/* GAIN Registers */
#define ADS131M08_GAIN_1 0x0000
#define ADS131M08_GAIN_2 0x0001
#define ADS131M08_GAIN_4 0x0002
#define ADS131M08_GAIN_8 0x0003
#define ADS131M08_GAIN_16 0x0004
#define ADS131M08_GAIN_32 0x0005
#define ADS131M08_GAIN_64 0x0006
#define ADS131M08_GAIN_128 0x0007

/* THRSHLD_LSB Register - DC Block Filter */
#define ADS131M08_DCBLOCK_622_MHZ 0x09 /* 622 mHz corner frequency */

/* SPI Frame Constants */
#define ADS131M08_FRAME_SIZE_WORDS 9 /* Status + 8 channels (no CRC for now) */
#define ADS131M08_MAX_CHANNELS 8
#define ADS131M08_BYTES_PER_WORD 3 /* 24-bit words by default */
#define ADS131M08_FRAME_BYTES (ADS131M08_FRAME_SIZE_WORDS * ADS131M08_BYTES_PER_WORD)

/* Status and Error Codes */
typedef enum
{
    ADS131M08_OK = 0,
    ADS131M08_ERROR = -1,
    ADS131M08_ERROR_TIMEOUT = -2,
    ADS131M08_ERROR_INVALID_PARAM = -3,
    ADS131M08_ERROR_NOT_READY = -4,
    ADS131M08_ERROR_SPI = -5
} ads131m08_status_t;

/* ADC Sample Data Structure */
typedef struct
{
    uint32_t status;                          /* Status word from ADC */
    int32_t channels[ADS131M08_MAX_CHANNELS]; /* Channel data (signed 32-bit) */
    uint32_t sample_number;                   /* Sample counter */
    uint64_t timestamp_us;                    /* Timestamp in microseconds */
} ads131m08_sample_t;

/* Configuration Structure */
typedef struct
{
    uint32_t sample_rate;                 /* Target sample rate in Hz */
    uint16_t enabled_channels;            /* Bitmask of enabled channels */
    uint8_t gain[ADS131M08_MAX_CHANNELS]; /* Gain setting for each channel */
    bool enable_dc_block;                 /* Enable DC block filter */
    void (*data_ready_callback)(const ads131m08_sample_t *sample);
} ads131m08_config_t;

/* Driver State Structure - Private */
typedef struct
{
    const struct device *spi_dev;
    const struct device *gpio_dev;
    struct spi_config spi_cfg;
    struct gpio_callback drdy_cb;
    struct k_work_delayable drdy_work;
    struct k_work_delayable periodic_dump_work;

    ads131m08_config_t config;
    ads131m08_sample_t current_sample;

    uint32_t sample_count;
    bool sampling_active;
    bool first_read;
    bool driver_ready;

    /* SPI Transaction Buffers */
    uint8_t tx_buffer[ADS131M08_FRAME_BYTES];
    uint8_t rx_buffer[ADS131M08_FRAME_BYTES];
} ads131m08_driver_t;

/* Public API Functions */

ads131m08_status_t ads131m08_ti_init(const ads131m08_config_t *config);

ads131m08_status_t ads131m08_ti_start_sampling(void);

ads131m08_status_t ads131m08_ti_stop_sampling(void);

uint32_t ads131m08_ti_get_sample_count(void);

/* Input multiplexer of ALL channels while sampling (MUXn[1:0] in CHn_CFG):
 * 0 = electrode inputs (normal), 1 = AGND (shorted inputs → ADC noise),
 * 2/3 = positive/negative DC test signal. Applied by the reader thread over the
 * next 8 data frames (one channel per 4-ms frame), without losing samples. */
#define ADS131M08_MUX_NORMAL 0
#define ADS131M08_MUX_AGND   1
ads131m08_status_t ads131m08_ti_request_input_mux(uint8_t mux);
/* MUX value currently applied to all channels, or -1 while switching. */
int ads131m08_ti_get_input_mux(void);

bool ads131m08_ti_is_sampling(void);

ads131m08_status_t ads131m08_ti_read_device_id(uint16_t *device_id);

ads131m08_status_t ads131m08_ti_write_register(uint8_t addr, uint16_t data, uint8_t word_length);

ads131m08_status_t ads131m08_ti_read_register(uint8_t addr, uint16_t *data, uint8_t word_length);

void ads131m08_ti_debug_print_registers(void);

void ads131m08_ti_start_periodic_register_dump(void);

void ads131m08_ti_stop_periodic_register_dump(void);

/* KA: Custom function to log registers for diagnostics */
void ads_log_registers(void);
void debug_print_all_registers(void);

#endif /* ADS131M08_TI_H */
