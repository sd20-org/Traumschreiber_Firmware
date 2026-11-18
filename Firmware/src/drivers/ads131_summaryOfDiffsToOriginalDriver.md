# Comparison: Original ADS131M02 Driver vs. ADS131M08 Implementation

Scope
- This document compares the original ADS131M02 driver implementation (adc_ads131m02.c) provided with the hardware vendor/example repository against the ADS131M08 implementation used in the project (as summarized in ads131m08_summary.md). The comparison is confined strictly to the provided sources and explains differences in detail: SPI/register protocols, framing, GPIO handling, timing/delays, initialization sequences, runtime data flow, power management and software architecture.

Notation
- "Original" refers to adc_ads131m02.c.
- "Our implementation" refers to the TI-flavored ADS131M08 driver (ads131m08_ti.c plus supporting files and the human-readable summary ads131m08_summary.md).

High-level architectural differences
- Original: Implements a Zephyr ADC-class driver using adc_context, semaphores, and the standard synchronous ADC API. Reads are performed on demand using the ADC API and internal synchronization via semaphores and the ADC context helpers.
- Our implementation: Implements a custom, device-specific driver that is interrupt-driven and pushes samples to the application using a user-provided callback whenever the DRDY pin indicates new data. It does not integrate with the Zephyr ADC subsystem in the same way.

Step-by-step comparison (walk original -> compare to our summary)

1) Includes / Module setup and logging
- Original: Uses Zephyr device tree SPI and GPIO DT helpers (spi_dt_spec, gpio_dt_spec) and registers a device using DT_INST macros. Logging with LOG_MODULE_REGISTER(ads131m02, CONFIG_ADC_LOG_LEVEL).
- Our implementation: Uses DEVICE_DT_GET to fetch devices and registers a module logger LOG_MODULE_REGISTER(ads131m08_ti, LOG_LEVEL_INF). The project driver uses explicit DEVICE_DT_GET and manual spi_config manipulation instead of DT convenience wrappers.

Differences:
- Original makes heavy use of DT helper macros (spi_dt_spec, SPI_DT_SPEC_INST_GET, gpio_dt_spec) and spi_transceive_dt/spi_read_dt convenience functions. Our driver uses lower-level APIs (DEVICE_DT_GET, spi_transceive/spi_write) and manages spi_config structure directly.

2) Command/Opcode and register formats
- Original: Defines ADS131M02_RREG_CMD and WREG_CMD equal to 0xA000 and 0x6000 but composes the actual command bytes differently when sending: for reads it forms temp = (RREG_CMD | (addr << 7)) and then writes that 16-bit value (sys_put_be16(temp, tx_buf)). The read helper then sends a 3-byte tx buffer.
- Our implementation: Uses ADS131M08_CMD_RREG (0xA000) and creates a 16-bit cmd_word via ADS131M08_CMD_RREG | addr. In read_register_ti we construct two 16-bit spi_bufs: one for the cmd and one dummy word, each with len=2, then call spi_transceive.

Differences:
- Bit/byte packing differs. The original shifts the register address left by 7 when composing the 16-bit command, and uses a 3-byte tx buffer protocol for register reads. Our implementation uses direct OR with the address into a 16-bit command word and performs two separate 16-bit transfers (cmd + dummy) via spi_bufs. The exact byte-level command sequences on the wire will differ.

3) Register read/write transaction lengths and endianness
- Original register read: ads131m02_reg_read() sends 3 bytes (tx_buf size 3) and reads back "read_buf_len" bytes. The driver commonly expects 3-byte read responses for registers (read_data[3]) and uses sys_get_be16/read as appropriate.
- Our implementation: read_register_ti() uses two 2-byte spi_bufs (4 bytes total) and expects the returned register data in rx_data[1] then sys_be16_to_cpu(rx_data[1]). Some reads in other code use 24-bit/32-bit word_lengths as argument to the helper.

Differences:
- Number of bytes sent and received for register operations differ (3-byte sequences in original vs sequences composed of 2×16-bit buffers in our code). The exact placement of register data in the returned bytes differs and may produce different readback patterns if the device expects the other framing.

4) SPI API usage and DT bindings
- Original: Uses spi_transceive_dt() and spi_read_dt() with spi_dt_spec in the device config. This is the device-tree aware, convenience API that automatically uses the CS pin and device configuration from devicetree.
- Our implementation: Uses spi_transceive() / spi_write() on ads_driver.spi_dev with an explicit ads_driver.spi_cfg and ad-hoc cs.gpio configuration. The driver manually sets the cs.gpio.port/pin/dt_flags.

Differences:
- Original relies on spi_dt_spec that simplifies CS handling and device tree wiring; ours manually sets the cs.gpio on spi_config and manipulates the operation flags at runtime (switching between SPI_WORD_SET(8) and SPI_WORD_SET(32)). This is functionally fine but a different style with potential for subtle runtime mismatches if cs or mode bits aren't identical to devicetree expectations.

5) Device reset and startup sequence
- Original: ads131m02_device_reset() composes a RESET_CMD, sends a 12-byte tx buffer and expects a 3-byte rx buffer response; then checks rx_buf against ADS131M02_RESET_RSP (0xFF22). The init() calls read ID, device_reset, then writes ADS131M02_DISABLE_ADC to CLOCK_REG and then sets device to standby via PM if enabled.
- Our implementation: ads131m08_ti_init() waits for DRDY, issues an UNLOCK command, checks STATUS, possibly sends WAKEUP and UNLOCK again, writes specific CLOCK/MODE/GAIN/THRSHLD registers, then re-reads CLOCK for verification and switches SPI word length to 32-bit for frame reads. There is no explicit RESET command usage in the TI-init code.

Differences:
- Original explicitly resets the device and verifies reset response. Our code uses an unlock/wakeup dance and chooses not to send a RESET in the provided code path.
- Original disables ADC in init and uses PM suspend path. Our code programs ADC registers to power/down or enable channels as required by TI setup sequence.

6) Power management (suspend/resume)
- Original: Implements PM support via ads131m02_pm and ads131m02_pm_action and integrates with PM_DEVICE_DT_INST_DEFINE and pm_device_init_suspended. The init() sends a standby command if PM is enabled.
- Our implementation: No PM hooks are defined in ads131m08_ti.c. There are no explicit suspend/resume callbacks for the ADC device in the TI implementation.

Differences:
- Original integrates with Zephyr power management; our TI driver currently has no explicit PM integration.

7) ADC API / data flow model
- Original: Uses a classical synchronous ADC API with adc_context helpers. The application requests a read via the ADC API (ads131m02_read), the driver sets up context, waits for DRDY using semaphores, reads sample(s) by performing spi transceive, stores values into buffers, and signals completion.
- Our implementation: The application registers a callback (ads_driver.config.data_ready_callback) and the driver autonomously starts sampling when ads131m08_ti_start_sampling() is invoked. The driver arms the DRDY interrupt and asynchronously sends frames to the callback in drdy_work_handler.

Differences:
- Original is read-on-demand and synchronous; ours is continuous, interrupt-driven, and event/callback-based. The concurrency primitives differ (semaphores + adc_context vs gpio_callback + k_work_delayable).

8) DRDY GPIO handling and synchronization
- Original: ads131m02_configure_gpio configures DRDY pin as input, sets gpio_pin_interrupt_configure_dt to GPIO_INT_EDGE_TO_ACTIVE, initializes a gpio callback ads131m02_data_ready_handler that gives a semaphore (drdy_sem) when DRDY triggers. The ADC read path then waits on drdy_sem.
- Our implementation: ads131m08_ti_init configures the DRDY pin with pull-up and later the start function configures GPIO interrupts for GPIO_INT_EDGE_FALLING. The ISR handler schedules a delayable work (k_work_schedule) to perform the SPI transaction in thread context.

Differences:
- Edge polarity: original uses GPIO_INT_EDGE_TO_ACTIVE (typical rising edge), our TI code configures falling edge in ads131m08_ti_start_sampling(). The init code also polls for DRDY low (0) in wait_for_drdy_ready() expecting active-low DRDY.
- Handler action: original's DRDY handler posts a semaphore which synchronous code waits on; our handler schedules a work item to run immediately.

9) SPI frame structure and word width for data frames
- Original: For sampling the original code uses a 12-byte rx_buf and treats status as two bytes (sys_get_be16(&rx_buf[0])) and each channel sample as a 3-byte (24-bit) value (sys_get_be24(&rx_buf[3]) etc.). The tx buffer for sample reads is 4 bytes (tx_buf[4] zeroed) and the code uses ads131m02_transceive which issues a transceive followed by a read.
- Our implementation: Uses 32-bit word transfers and a frame of ADS131M08_FRAME_SIZE_WORDS = 9 32-bit words. The driver after initialization switches its spi_cfg to SPI_WORD_SET(32) to read frames comprised of big-endian 32-bit words where the device places a 24-bit sample into the least-significant 24 bits of each 32-bit word. The driver then uses sys_be32_to_cpu and performs sign extension logic.

Differences:
- Word width used for frame reads: original uses byte-level framing, explicitly reading 12 bytes and extracting 3-byte words; ours uses 32-bit word transfers with a 9-word frame.
- Parsing: original uses sys_get_be24 and stores into an unsigned buffer (no explicit sign extension in code excerpt), ours sign-extends 24-bit samples to 32-bit signed integers.

10) Sign-extension and numeric types
- Original: ads131m02_read_sample assigns *buffer = sys_get_be24(&rx_buf[3]); No sign extension is explicitly shown in this code excerpt; buffer type is uint32_t and later users will interpret or scale as required. The resolution and reference constants are provided (RESOLUTION=24, REF_INTERNAL=1200).
- Our implementation: drdy_work_handler explicitly checks raw_data & 0x00800000 and sets current_sample.channels[ch] to (int32_t)(raw_data | 0xFF000000) when negative, otherwise masked to 0x00FFFFFF. Thus we produce signed 32-bit samples.

Differences:
- Our driver explicitly sign-extends and stores signed int32 samples; the original example extracts raw 24-bit values into unsigned 32-bit variables and leaves interpretation to higher layers.

11) First-read alignment behavior
- Original: The sample read flow waits for DRDY and then performs a transceive that reads the sample(s). There is no explicit "first_read" dummy alignment in the original driver code excerpt.
- Our implementation: ads131m08_ti sets ads_driver.first_read = true and treats the first SPI transceive after sampling_start as an alignment/dummy read: it performs the transceive, clears first_read and returns without delivering the captured data.

Differences:
- Additional alignment dummy read in our TI driver; original driver performs the read and returns a sample directly.

12) Register programming specifics (which registers, values and sequence)
- Original (ads131m02_init): After reset the driver writes ADS131M02_DISABLE_ADC to CLOCK_REG to disable the ADC by default. The driver exposes APIs to configure ADC operating mode (ads131m02_set_adc_mode), power mode (ads131m02_set_power_mode) and per-channel setup functions (channel_setup + configure_gain) which write GAIN_REG, CLOCK_REG, etc. Initialization also verifies the device ID via a reg read and checks reset responses.
- Our implementation: The TI init sequence writes a carefully ordered sequence of registers: CLOCK (disable channels + OSR + power + xtal disable), MODE (24-bit word length, DRDY pulse, timeout enable), GAIN1 (packed two-channel gains), THRSHLD_LSB (DC block filter), MODE again (32-bit sign-extended word length), CLOCK again (enable all channels + OSR + ext ref enable) and then readback CLOCK to verify expected value (0xFFD4). There is an unlock/wakeup dance surrounding writes.

Differences:
- Sequence complexity: Our TI code programs a specific multi-step sequence with toggling of MODE word-length between 24-bit and 32-bit sign-extended and a final readback check. The original driver focuses on resetting, disabling ADC initially and leaving configuration to channel_setup calls and helper APIs rather than a monolithic sequence. The specific bit/field values used differ between the two codebases because of different design choices (original leaves configuration to runtime ADC API; ours programs TI-recommended defaults at init).

13) SPI operation mode and transfers (CPHA/CPOL/wordsize)
- Original SPI config in DT wrapper: SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8). That is CPHA set, CPOL unspecified in the DT macros used.
- Our TI code uses SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_WORD_SET(8) for register accesses and later switches to SPI_WORD_SET(32) (keeps CPOL) for frame reads.

Differences:
- Clock phase/polarity choice differs (original uses SPI_MODE_CPHA, our code uses SPI_MODE_CPOL). This is a critical difference: CPHA vs CPOL changes sample/shift timing and will affect raw SPI transactions. That difference was chosen explicitly in both codebases but could be device- or board-specific. It is a concrete protocol difference to be aware of.

14) Error handling and return codes
- Original: Uses standard Linux/Zephyr-style negative errno return values for internal helper functions and uses LOG_ERR/LOG_WRN for reporting. Device init returns 0 on success and negative error codes on failure.
- Our implementation: Uses a small typedef enum ads131m08_status_t with named negative values for errors (ADS131M08_ERROR, ADS131M08_ERROR_TIMEOUT, ADS131M08_ERROR_INVALID_PARAM, ADS131M08_ERROR_NOT_READY, ADS131M08_ERROR_SPI). The API returns these typed status codes.

Differences:
- Error reporting style differs: original uses plain int / negative errno; our code uses a typed enum for status codes.

15) Device tree / driver registration
- Original: Instantiates driver instances via DT_INST_FOREACH_STATUS_OKAY and DEVICE_DT_INST_DEFINE macros, registering the driver with Zephyr's device model and PM integration.
- Our implementation: The TI driver is consumed directly from main.c via ads131m08_ti_init() and is not registered as a Zephyr device in the same way; there is no DT-based DEVICE_DT_INST_DEFINE for it in the provided files.

Differences:
- Original integrates with Zephyr driver model and DT instancing; our TI driver is a bespoke module initialized by application code.

Step-by-step comparison (walk our summary -> compare to original)

The previous section covered most concrete differences. Below is a reverse pass, enumerating the items called out in the ads131m08_summary.md and indicating how the original differs or corresponds.

A) DRDY polling and SYNC toggles (our summary Step 4/5)
- The original driver configures DRDY input and enables interrupts but does not perform the SYNC toggling sequence or repeated polling for readiness used by our TI init. Instead, the original waits for DRDY by use of semaphores in read flow and expects the device to be responsive after reset and setup commands.

B) Unlock/wakeup and STATUS check (our summary Step 6)
- The original reset->verify flow uses a RESET command and checks a specific reset response. It does not perform the unlock/wakeup loop used in our TI driver. The two drivers therefore use different initial command exchanges to prepare the part for register writes.

C) Register write/read implementation details (our summary Step 7)
- The original reg read/write helpers use a different byte layout for commands (addr shifted left by 7) and use the spi_dt_spec convenience functions. The sequence and semantics of specific register values are not identical: original often leaves configuration until channel_setup and power mode APIs are invoked.

D) Switching to 32-bit transfers for frame reads (our summary Step 7/12)
- The original sample read uses a byte-wise approach (rx buffer of 12 bytes and extracting 3-byte samples). It does not switch spi_config to 32-bit word transfers; instead it performs byte-level transfers and builds 24-bit samples via sys_get_be24. Our approach of changing the driver to SPI_WORD_SET(32) is not present in original.

E) First-read discard (our summary Step 11)
- Original does not implement an explicit first-read discard sequence.

F) Software model (our summary Steps 9-13)
- Original is synchronous (adc_context) and read-driven; our driver is asynchronous, interrupt-driven and pushes data via callback. Original uses semaphores and ADC API flows rather than immediate callback invocation.

G) Diagnostic register dump (our summary Step 8)
- The TI implementation reads multiple registers and prints them for debugging (debug_print_all_registers). The original driver performs device ID read and reset verification but does not include an equivalent register dump helper in the provided excerpt.

H) PM integration (our summary note)
- The original integrates with Zephyr PM and can send standby/wakeup commands in pm callbacks. Our TI implementation has no PM integration in the provided code.

I) Logging frequency and RTT integration
- Our TI code has LOG_INF messages sprinkled throughout init and drdy work handler. The original uses LOG_INF/LOG_ERR as well but oriented around the adc_context operations and driver init messages. Both use Zephyr logging but differ in the exact messages and frequency.

Summary of the most important actionable differences (concise)
- Transfer framing: Original uses byte-wise 3-byte 24-bit reads; our driver switches to 32-bit word transfers for frames and expects a nine-word frame (status + 8 channels). This is a major difference in wire format and SPI configuration.
- SPI mode: Original uses SPI_MODE_CPHA in its DT instantiation; our TI driver explicitly uses SPI_MODE_CPOL. This is a fundamental timing difference.
- Register command composition: Original composes RREG/WREG commands with an address shift (addr << 7) and sends 3/6 byte transactions; our driver composes 16-bit cmd words as CMD|addr and sends sequences of 16-bit buffers. Byte placement differs.
- Software model: Original is synchronous and ADC-framework oriented (semaphores + adc_context); ours is interrupt-driven, callback-based and uses delayed work.
- Reset and PM: Original calls reset and integrates with PM; ours uses unlock/wakeup and no PM integration.
- First-read alignment: Our TI code performs a dummy first read to align frame boundaries; original does not.
- Sign handling: Our TI driver explicitly sign-extends samples to signed 32-bit; original extracts 24-bit sample words and leaves interpretation to upper layers.

Implications
- These differences are not mere style choices — they result in different SPI bitstreams on the wire (mode, wordsize, byte order and command placement). Interoperability is only guaranteed if the ADS131M08 on the hardware wire expects the exact sequences used by the TI implementation. If instead the chip or board expects the exact packet structure produced by the original example, mismatches in readback, register write acceptance or data framing may occur.

Recommendations for reconciliation (based on observed differences)
- Verify physical SPI mode expected by the ADC on the specific hardware (CPHA vs CPOL). Align both drivers to the hardware requirement.
- Decide on a canonical frame protocol (either 3-byte-per-word byte-streams or 32-bit word transfers). If 32-bit words are used, ensure the device indeed supports that mode and that the CS/word-size support is correct in SPI controller.
- Align register command composition (RREG/WREG) to the device's expected bit/byte format. Use the datasheet command formats or reuse the proven original driver routines if they match the silicon.
- If synchronous ADC API integration is required, adapt the TI driver to provide a device instance and ADC APIs or create a thin wrapper that converts the callback model to the adc_context model.
- If power management is required in product, add pm hooks similar to the original.

This document reflects a careful, line-by-line style comparison between the original adc_ads131m02.c file and the project's ADS131M08 code and summary. All statements above are constrained to the code provided and do not assume undocumented device behaviour.