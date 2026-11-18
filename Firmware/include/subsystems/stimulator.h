/**
 * @file stimulator.h
 * @brief LED Stimulator Subsystem
 *
 * Provides high-level LED pattern control for the Traumschreiber2509.
 * Implements various visual stimulation patterns using the RGB LED.
 *
 * Features:
 * - Smooth color transitions
 * - Breathing effects
 * - Rainbow cycles
 * - Color mixing demonstrations
 * - Pattern sequencing and timing control
 *
 * @author ISDT
 * @date 2025
 */

#ifndef STIMULATOR_H
#define STIMULATOR_H

#include "hw_abstraction/rgb_led.h"
#include "i2s_out.h"
#include <zephyr/kernel.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Stimulator patterns */
    typedef enum
    {
        STIMULATOR_PATTERN_OFF = 0,       /**< All LEDs off */
        STIMULATOR_PATTERN_STATIC,        /**< Static color */
        STIMULATOR_PATTERN_SMOOTH_CYCLE,  /**< Smooth color transitions */
        STIMULATOR_PATTERN_BREATHING,     /**< Breathing effect */
        STIMULATOR_PATTERN_RAINBOW,       /**< Rainbow color cycle */
        STIMULATOR_PATTERN_COLOR_MIX,     /**< Color mixing demo */
        STIMULATOR_PATTERN_AUDIO_DEMO,    /**< Audio + visual demo */
        STIMULATOR_PATTERN_BEEP_SEQUENCE, /**< Beep sequence with LEDs */
        STIMULATOR_PATTERN_MELODY_DEMO,   /**< Melody with synchronized LEDs */
        STIMULATOR_PATTERN_CUSTOM         /**< Custom user pattern */
    } stimulator_pattern_t;

    /* Stimulator configuration */
    typedef struct
    {
        stimulator_pattern_t pattern; /**< Current pattern */
        uint32_t duration_ms;         /**< Pattern duration (0 = infinite) */
        uint16_t transition_steps;    /**< Steps for smooth transitions */
        uint16_t delay_per_step_ms;   /**< Delay between transition steps */
        rgb_color_t base_color;       /**< Base color for patterns */
        bool enable_audio;            /**< Enable audio with visual patterns */
        uint16_t audio_frequency_hz;  /**< Base audio frequency */
    } stimulator_config_t;

    /* Stimulator status */
    typedef struct
    {
        stimulator_pattern_t current_pattern;
        uint32_t pattern_start_time;
        uint32_t elapsed_time_ms;
        bool is_running;
        bool is_initialized;
    } stimulator_status_t;

    /**
     * @brief Initialize the LED stimulator subsystem
     *
     * Sets up the stimulator, initializes hardware abstraction layer,
     * and starts the stimulator thread.
     *
     * @return 0 on success, negative error code on failure
     */
    int stimulator_init(void);

    /**
     * @brief Start a stimulation pattern
     *
     * @param config Pointer to stimulator configuration
     * @return 0 on success, negative error code on failure
     */
    int stimulator_start_pattern(const stimulator_config_t *config);

    /**
     * @brief Stop current stimulation pattern
     *
     * @return 0 on success, negative error code on failure
     */
    int stimulator_stop(void);

    /**
     * @brief Set static LED color
     *
     * @param color Pointer to RGB color
     * @return 0 on success, negative error code on failure
     */
    int stimulator_set_static_color(const rgb_color_t *color);

    /**
     * @brief Get current stimulator status
     *
     * @param status Pointer to status structure to fill
     * @return 0 on success, negative error code on failure
     */
    int stimulator_get_status(stimulator_status_t *status);

    /**
     * @brief Run smooth transition between two colors
     *
     * @param start_color Start color
     * @param end_color End color
     * @param steps Number of transition steps
     * @param delay_ms Delay between steps
     * @return 0 on success, negative error code on failure
     */
    int stimulator_smooth_transition(const rgb_color_t *start_color,
                                     const rgb_color_t *end_color,
                                     uint16_t steps,
                                     uint16_t delay_ms);

    /**
     * @brief Run breathing effect with specified color
     *
     * @param base_color Base color for breathing
     * @param cycles Number of breathing cycles
     * @return 0 on success, negative error code on failure
     */
    int stimulator_breathing_effect(const rgb_color_t *base_color, uint16_t cycles);

    /**
     * @brief Run rainbow color cycle
     *
     * @param duration_sec Duration in seconds
     * @return 0 on success, negative error code on failure
     */
    int stimulator_rainbow_cycle(uint16_t duration_sec);

    /**
     * @brief Run color mixing demonstration
     *
     * @return 0 on success, negative error code on failure
     */
    int stimulator_color_mixing_demo(void);

    /**
     * @brief Check if stimulator is initialized
     *
     * @return true if initialized, false otherwise
     */
    bool stimulator_is_initialized(void);

    /**
     * @brief Run audio-visual demo
     *
     * Combines LED patterns with synchronized audio tones.
     *
     * @return 0 on success, negative error code on failure
     */
    int stimulator_audio_visual_demo(void);

    /**
     * @brief Play beep sequence with synchronized LEDs
     *
     * @param frequency_hz Beep frequency
     * @param beep_count Number of beeps
     * @return 0 on success, negative error code on failure
     */
    int stimulator_beep_with_leds(uint16_t frequency_hz, uint8_t beep_count);

    /**
     * @brief Play startup sequence
     *
     * Plays a startup melody with LED pattern to indicate system ready.
     *
     * @return 0 on success, negative error code on failure
     */
    int stimulator_startup_sequence(void);

#ifdef __cplusplus
}
#endif

#endif /* STIMULATOR_H */
