# ADS131M08 Initialization and Data Flow Summary

This document describes, step by step and strictly according to the provided source files, how the nRF52840 connects to and operates the ADS131M08 ADC from power-on (firmware start) until continuous measurement data are received and delivered to the application. The text closely follows control flow, register sequences, GPIO/SPI usage, timing and callbacks implemented in the attached files.

1) System boot and application start (main.c)
- main() executes on board bring-up. Before interacting with the ADC the application initializes other subsystems (RGB LED, button, stimulator). These steps are not required for ADC operation but are executed by the sample application prior to ADS131M08 setup.
- The TI-specific ADS131M08 driver is used by the application: ads131m08_ti_init() is called with an ads131m08_config_t that includes a data_ready_callback pointer (ads131m08_ti_data_callback in main.c).

2) Hardware resources used (constants from ads131m08_ti.h / main.c)
- SPI: device node DT_NODELABEL(spi0) (ADS131M08_SPI_NODE), configured initially at ADS131M08_SPI_FREQUENCY = 1 000 000 Hz with SPI word-size 8 bits in the code paths used during register access.
- Chip-Select (CS): GPIO on port DT_NODELABEL(gpio0), pin ADS131M08_GPIO_CS_PIN = 3; configured for active-low CS via spi_config.cs.gpio.
- Data Ready (DRDY): GPIO on port DT_NODELABEL(gpio1), pin ADS131M08_GPIO_DRDY_PIN = 10; configured as input with pull-up and later used as interrupt source (falling edge) to trigger frame reads.
- SYNC: GPIO on port DT_NODELABEL(gpio1), pin ADS131M08_GPIO_SYNC_PIN = 11; toggled during initialization to synchronize the device and to probe DRDY behavior.

3) Entry to ads131m08_ti_init() (ads131m08_ti.c)
- Parameter check: ensures config pointer is not NULL.
- Driver state cleared (memset) and configuration copied into ads_driver.config. first_read flag set true.
- A short stabilization delay is applied: k_sleep(K_MSEC(150)).
- Device handles are acquired with DEVICE_DT_GET for the SPI and GPIO nodes and device_is_ready checks are performed. If devices are not ready the function returns ADS131M08_ERROR_NOT_READY.
- spi_cfg fields set: operation initially set to SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_WORD_SET(8), frequency to ADS131M08_SPI_FREQUENCY (1 MHz), and slave to 0. The CS GPIO port/pin/dt_flags are populated from the device tree constants.

4) GPIO configuration and SYNC toggling
- DRDY GPIO is configured as input with pull-up: gpio_pin_configure(..., GPIO_INPUT | GPIO_PULL_UP).
- SYNC GPIO is configured as an output and driven low then high with small delays in-between (gpio_pin_set + k_sleep) to synchronize the ADC and observe DRDY response. The code toggles SYNC multiple times (including a short 3x pulse loop) while checking DRDY via gpio_pin_get to detect a stable ready state.

5) Wait for DRDY ready
- The helper wait_for_drdy_ready() polls the DRDY pin for up to ADS131M08_DRDY_TIMEOUT_MS (100 ms), sleeping 1 ms per iteration. If DRDY becomes low the routine returns ADS131M08_OK; otherwise ADS131M08_ERROR_TIMEOUT is returned after the timeout.
- The init function will call wait_for_drdy_ready() and retry up to max_retries (3) with a 200 ms sleep between retries.

6) Unlock / wake-up sequence and status read
- After DRDY is confirmed, the code reads the STATUS register via the read_register_ti() helper.
- To ensure register writes can be performed, the driver writes an unlock opcode (ADS131M08_CMD_UNLOCK) over SPI using spi_write with a temporary cmd_spi_cfg (8-bit word settings). The code re-reads STATUS to inspect a potential lock bit.
- If the device indicates it is still locked (checked by extracting bit 15 of the read status_reg), the code sends WAKEUP (ADS131M08_CMD_WAKEUP), waits, re-sends UNLOCK, waits, and re-reads STATUS.

7) Register programming sequence (TI recommended sequence present in code)
- The driver programs a series of registers using write_register_ti():
  a) CLOCK register: disable all channels, choose OSR=4096 (ADS131M08_CLOCK_OSR_4096), set very-low-power (ADS131M08_CLOCK_PWR_VLP) and disable internal oscillator (ADS131M08_CLOCK_XTAL_DIS). The write uses 24-bit word length argument in the helper.
  b) MODE register: set DRDY pulse mode (ADS131M08_MODE_DRDY_PULSE), enable timeout (ADS131M08_MODE_TIMEOUT_ENABLE), and set initially WLENGTH_24BIT.
  c) GAIN1 register: program per-channel gains (example in code: (ADS131M08_GAIN_32 << 12) | (ADS131M08_GAIN_1 << 8) | (ADS131M08_GAIN_32 << 4) | ADS131M08_GAIN_1) — see code for exact mapping.
  d) THRSHLD_LSB: set DC block filter constant ADS131M08_DCBLOCK_622_MHZ.
  e) MODE register again: switch to WLENGTH_32BIT_SE (32-bit sign-extended mode) while keeping DRDY pulse and timeout enable bits.
  f) CLOCK register: enable all channels (ADS131M08_CLOCK_CH_EN_ALL) while keeping OSR=4096, VLP power mode, XTAL disabled and enabling external reference (ADS131M08_CLOCK_EXTREF_EN).

- After writing the final CLOCK value the driver does a readback of the CLOCK register via read_register_ti(). If the readback differs from expected (code checks against 0xFFD4) it attempts an unlock and retries the CLOCK write once more, then re-reads.
- If the final readback equals the expected magic (0xFFD4) the driver switches the internal SPI configuration to 32-bit word transfers for frame reads (ads_driver.spi_cfg.operation = SPI_WORD_SET(32) with CPOL set).

8) Final init housekeeping
- debug_print_all_registers() is invoked to read several registers for diagnostic purposes (the helper calls read_register_ti for ID, STATUS, MODE, CLOCK, GAIN1, GAIN2, CFG, THRSHLD_MSB, THRSHLD_LSB).
- A delayable work item is initialized k_work_init_delayable(&ads_driver.drdy_work, drdy_work_handler) and the DRDY GPIO callback structure is configured with gpio_init_callback(&ads_driver.drdy_cb, drdy_interrupt_handler, BIT(ADS131M08_GPIO_DRDY_PIN)). The callback is added via gpio_add_callback.
- ads_driver.driver_ready is set to true and the function returns ADS131M08_OK.

9) Starting continuous sampling (ads131m08_ti_start_sampling)
- The application calls ads131m08_ti_start_sampling(). Function checks driver_ready and configures the DRDY pin interrupt with gpio_pin_interrupt_configure(..., GPIO_INT_EDGE_FALLING).
- Sampling state is armed by setting ads_driver.sampling_active = true, first_read = true and resetting sample_count to 0.
- From this point DRDY falling-edge interrupts will trigger the DRDY ISR defined in the driver.

10) DRDY interrupt and deferred read (drdy_interrupt_handler → k_work schedule)
- The DRDY GPIO ISR (drdy_interrupt_handler) performs minimal work and schedules the delayable work item immediately via k_work_schedule(&ads_driver.drdy_work, K_NO_WAIT).
- All SPI transfers are performed in drdy_work_handler in thread context (workqueue), avoiding SPI access in interrupt context.

11) First read vs subsequent reads (drdy_work_handler)
- On the first DRDY event after starting sampling the driver builds a full SPI transaction with a zeroed tx_buffer and a rx_buffer sized for ADS131M08_FRAME_SIZE_WORDS (9 words) of 32-bit words. The driver issues spi_transceive with the configured ads_driver.spi_cfg (note: after init it may be configured for 32-bit transfers).
- For the very first read the implementation treats it as a dummy alignment read: if ads_driver.first_read is true the driver performs the spi_transceive, clears first_read and returns without handing data to application. This aligns the SPI stream with device framing.

12) Regular frame reads and parsing
- For every subsequent DRDY event drdy_work_handler issues an SPI transceive that clocks out ADS131M08_FRAME_SIZE_WORDS 32-bit words into ads_driver.rx_buffer.
- The driver converts each received 32-bit big-endian word with sys_be32_to_cpu.
- Parsing logic per code:
  - The first word (rx_buffer[0]) is the Status word: assigned to current_sample.status after endianness conversion.
  - The following 8 words (rx_buffer[1]..rx_buffer[8]) are channel words. Each channel word is a 24-bit sample packed in the lower 24 bits of the 32-bit word received (device-specific framing). The code extracts raw_data = sys_be32_to_cpu(rx_buffer[ch + 1]) and sign-extends: if bit 23 (0x00800000) is set the code ORs raw_data with 0xFF000000 to extend sign; otherwise masks with 0x00FFFFFF.
  - The converted 32-bit signed samples are stored in current_sample.channels[ch].
- Sample metadata: sample_number is incremented (++ads_driver.sample_count) and assigned to current_sample.sample_number; timestamp_us is assigned from k_uptime_get() multiplied by 1000.

13) Delivering data to the application
- If the application registered a callback (ads_driver.config.data_ready_callback), drdy_work_handler calls that callback with a pointer to the filled current_sample structure.
- In the provided main.c the application uses ads131m08_ti_data_callback as the callback. In the sample code the callback is present but its detailed logging lines are commented out.

14) Stopping sampling (ads131m08_ti_stop_sampling)
- ads131m08_ti_stop_sampling() clears ads_driver.sampling_active and disables the DRDY interrupt using gpio_pin_interrupt_configure(..., GPIO_INT_DISABLE). This stops further DRDY-driven reads.

15) Counting and status
- The driver exposes ads131m08_ti_get_sample_count() which returns ads_driver.sample_count as a uint32_t.
- ads131m08_ti_is_sampling() returns ads_driver.sampling_active.
- The application periodically queries these functions (main loop) to compute an observed sample rate and to provide visual/log feedback.

Notes and important timings observed in code
- SPI register operations use 8-bit word transfers at 1 MHz during register writes/reads; after final CLOCK confirmation the driver switches to 32-bit word transfer operation for frame reads.
- Several short delays are used during init: 150 ms initial stabilization, small k_sleep between SYNC toggles, 2–20 ms delays during unlock/wakeup sequences and register programming to allow the device time to accept commands.
- The DRDY ready polling timeout is 100 ms (ADS131M08_DRDY_TIMEOUT_MS) and init will retry up to 3 times with 200 ms waits between attempts.
- Frame size: the driver expects a frame consisting of 9 32-bit words (1 status + 8 channel words). The channel samples are sign-extended 24-bit values packed into 32-bit words by the device framing used in this code.

This summary follows the exact sequences, constants and logic implemented in the provided files and does not add behavior beyond what the code performs.