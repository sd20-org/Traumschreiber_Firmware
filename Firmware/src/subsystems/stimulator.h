// LED and Audio Stimulator Subsystem Header
#ifndef SUBSYSTEMS_STIMULATOR_H
#define SUBSYSTEMS_STIMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "hw_abstraction/rgb_led.h"

typedef enum
{
    STIMULATOR_PATTERN_OFF = 0,
    STIMULATOR_PATTERN_STATIC,
    STIMULATOR_PATTERN_SMOOTH_CYCLE,
    STIMULATOR_PATTERN_BREATHING,
    STIMULATOR_PATTERN_RAINBOW,
    STIMULATOR_PATTERN_COLOR_MIX,
    STIMULATOR_PATTERN_AUDIO_DEMO,
    STIMULATOR_PATTERN_BEEP_SEQUENCE,
    STIMULATOR_PATTERN_MELODY_DEMO
} stimulator_pattern_t;

typedef struct
{
    stimulator_pattern_t pattern;
    uint32_t duration_ms;
    uint16_t transition_steps;
    uint16_t delay_per_step_ms;
    rgb_color_t base_color;
    bool enable_audio;
    uint16_t audio_frequency_hz;
} stimulator_config_t;

int stimulator_init(void);
int stimulator_start_pattern(const stimulator_config_t *config);
int stimulator_stop(void);
int stimulator_startup_sequence(void);
int stimulator_audio_visual_demo(void);
int stimulator_beep_with_leds(uint16_t frequency_hz, int beep_count);

#endif // SUBSYSTEMS_STIMULATOR_H
