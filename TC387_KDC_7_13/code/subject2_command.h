#ifndef SUBJECT2_COMMAND_H
#define SUBJECT2_COMMAND_H

#include <stdint.h>

#define SUBJECT2_COMMAND_CODE_MAX        (17U)

typedef enum
{
    SUBJECT2_COMMAND_NONE = 0,
    SUBJECT2_COMMAND_LEFT_INDICATOR = 1,
    SUBJECT2_COMMAND_RIGHT_INDICATOR,
    SUBJECT2_COMMAND_HIGH_BEAM,
    SUBJECT2_COMMAND_LOW_BEAM,
    SUBJECT2_COMMAND_FOG_LIGHT,
    SUBJECT2_COMMAND_HAZARD_LIGHT,
    SUBJECT2_COMMAND_INTERIOR_LIGHT,
    SUBJECT2_COMMAND_WIPER,
    SUBJECT2_COMMAND_HORN_ONE_SECOND,
    SUBJECT2_COMMAND_HORN_TWO_SECONDS,
    SUBJECT2_COMMAND_HORN_THREE_SECONDS,
    SUBJECT2_COMMAND_HORN_TWO_BEEPS,
    SUBJECT2_COMMAND_HORN_THREE_BEEPS,
    SUBJECT2_COMMAND_HORN_FOUR_BEEPS,
    SUBJECT2_COMMAND_HORN_LONG_SHORT,
    SUBJECT2_COMMAND_HORN_URGENT,
    SUBJECT2_COMMAND_HORN_ALARM
} SUBJECT2_COMMAND_CODE;

typedef struct
{
    uint8_t left_indicator_on;
    uint8_t right_indicator_on;
    uint8_t high_beam_on;
    uint8_t low_beam_on;
    uint8_t fog_light_on;
    uint8_t interior_light_on;
    uint8_t wiper_on;
    uint8_t horn_on;
    uint16_t horn_frequency_hz;
} SUBJECT2_OUTPUT_STATE;

typedef struct
{
    uint8_t left_indicator_enabled;
    uint8_t right_indicator_enabled;
    uint8_t hazard_enabled;
    uint8_t indicator_phase;
    uint16_t indicator_elapsed_ms;
    uint8_t high_beam_enabled;
    uint8_t low_beam_enabled;
    uint8_t fog_light_enabled;
    uint8_t interior_light_enabled;
    uint8_t wiper_enabled;
    uint8_t horn_pattern;
    uint8_t horn_segment_index;
    uint16_t horn_segment_remaining_ms;
} SUBJECT2_COMMAND_STATE;

void Subject2_command_init(SUBJECT2_COMMAND_STATE *state);
uint8_t Subject2_command_dispatch(SUBJECT2_COMMAND_STATE *state, uint8_t command_code);
void Subject2_command_tick(SUBJECT2_COMMAND_STATE *state, uint32_t elapsed_ms);
void Subject2_command_get_output(const SUBJECT2_COMMAND_STATE *state,
                                 SUBJECT2_OUTPUT_STATE *output);

#endif
