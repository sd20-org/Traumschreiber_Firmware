/*
 * ads131m08.h
 *
 * Public API and constants for the ADS131M08 ADC driver.
 *
 * This header exposes a minimalized, portable interface used by application code
 * to initialize the ADS131M08, start/stop sampling and read/write device registers.
 * The driver expects the Zephyr SPI and GPIO subsystems to be available and
 * configured through devicetree as used by the implementation.
 */

#ifndef ADS131M08_H_
#define ADS131M08_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

/*
 * The following section defines opcodes, registers and bit masks used to talk
 * with the ADS131M08. The constants are intentionally terse but follow the
 * names used in the device datasheet.
 */

#define ADS131M08_OPCODE_NULL 0x0000
#define ADS131M08_OPCODE_RESET 0x0011
#define ADS131M08_OPCODE_RREG 0xA000
#define ADS131M08_OPCODE_WREG 0x6000
#define ADS131M08_OPCODE_STANDBY 0x0022
#define ADS131M08_OPCODE_WAKEUP 0x0033
#define ADS131M08_OPCODE_LOCK 0x0555
#define ADS131M08_OPCODE_UNLOCK 0x0655

#define ADS131M08_REG_ID 0x00
#define ADS131M08_REG_STATUS 0x01
#define ADS131M08_REG_MODE 0x02
#define ADS131M08_REG_CLOCK 0x03
#define ADS131M08_REG_GAIN 0x04
#define ADS131M08_REG_CFG 0x06
#define ADS131M08_REG_THRSHLD_MSB 0x07
#define ADS131M08_REG_THRSHLD_LSB 0x08
#define ADS131M08_REG_CH0_CFG 0x09
#define ADS131M08_REG_CH1_CFG 0x0A
#define ADS131M08_REG_CH2_CFG 0x0B
#define ADS131M08_REG_CH3_CFG 0x0C
#define ADS131M08_REG_CH4_CFG 0x0D
#define ADS131M08_REG_CH5_CFG 0x0E
#define ADS131M08_REG_CH6_CFG 0x0F
#define ADS131M08_REG_CH7_CFG 0x10

#define ADS131M08_CLOCK_XTAL_DIS BIT(7)
#define ADS131M08_CLOCK_EXTREF_EN BIT(6)
#define ADS131M08_CLOCK_OSR_MASK (0x7 << 2)
#define ADS131M08_CLOCK_OSR_128 (0x0 << 2)
#define ADS131M08_CLOCK_OSR_256 (0x1 << 2)
#define ADS131M08_CLOCK_OSR_512 (0x2 << 2)
#define ADS131M08_CLOCK_OSR_1024 (0x3 << 2)
#define ADS131M08_CLOCK_OSR_2048 (0x4 << 2)
#define ADS131M08_CLOCK_OSR_4096 (0x5 << 2)
#define ADS131M08_CLOCK_PWR_MASK (0x3 << 0)
#define ADS131M08_CLOCK_PWR_VLP (0x0 << 0)
#define ADS131M08_CLOCK_PWR_LP (0x1 << 0)
#define ADS131M08_CLOCK_PWR_HR (0x2 << 0)

#define ADS131M08_MODE_REG_CRC_EN BIT(13)
#define ADS131M08_MODE_RX_CRC_EN BIT(12)
#define ADS131M08_MODE_CRC_TYPE BIT(11)
#define ADS131M08_MODE_RESET BIT(10)
#define ADS131M08_MODE_WLENGTH_MASK (0x3 << 8)
#define ADS131M08_MODE_WLENGTH_16 (0x0 << 8)
#define ADS131M08_MODE_WLENGTH_24 (0x1 << 8)
#define ADS131M08_MODE_WLENGTH_32 (0x2 << 8)

#define ADS131M08_MAX_CHANNELS 8
#define ADS131M08_FRAME_SIZE (1 + ADS131M08_MAX_CHANNELS)
#define ADS131M08_BYTES_PER_WORD 3
#define ADS131M08_FRAME_BYTES (ADS131M08_FRAME_SIZE * ADS131M08_BYTES_PER_WORD)

/*
 * ads131m08_status_t
 * Enumerated return codes used by the driver. Functions generally return
 * ADS131M08_OK on success and another code on failure. Keep checks simple
 * in higher level code and map to these codes for diagnostics.
 */
typedef enum
{
    ADS131M08_OK = 0x00,
    ADS131M08_ERROR = 0x01,
    ADS131M08_BUSY = 0x02,
    ADS131M08_TIMEOUT = 0x03,
} ads131m08_status_t;

/*
 * ads131m08_sample_t
 * Structure used to deliver a sampled frame from the driver to the
 * application callback. Channel data is sign-extended to 32-bit values.
 */
typedef struct
{
    int32_t channels[ADS131M08_MAX_CHANNELS];
    uint32_t status;
    uint64_t timestamp;
} ads131m08_sample_t;

/*
 * ads131m08_config_t
 * Minimal configuration structure allowing the caller to register a
 * callback which will be invoked whenever a complete frame is available.
 */
typedef struct
{
    void (*data_ready_callback)(const ads131m08_sample_t *sample);
} ads131m08_config_t;

/*
 * Public API - brief descriptions provided.
 */
ads131m08_status_t ads131m08_init(const ads131m08_config_t *config);
ads131m08_status_t ads131m08_start_sampling(void);
ads131m08_status_t ads131m08_stop_sampling(void);
ads131m08_status_t ads131m08_read_id(uint16_t *id);
ads131m08_status_t ads131m08_reset(void);
ads131m08_status_t ads131m08_read_register(uint8_t reg, uint16_t *value);
ads131m08_status_t ads131m08_write_register(uint8_t reg, uint16_t value);
uint32_t ads131m08_get_sample_count(void);
bool ads131m08_is_sampling(void);

#endif /* ADS131M08_H_ */