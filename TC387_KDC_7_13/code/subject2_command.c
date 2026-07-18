#include "subject2_command.h"

#define SUBJECT2_INDICATOR_PERIOD_MS      (500U)
#define SUBJECT2_HORN_NORMAL_FREQUENCY_HZ (1000U)
#define SUBJECT2_HORN_LOW_FREQUENCY_HZ    (500U)

typedef enum
{
    SUBJECT2_HORN_PATTERN_NONE = 0,
    SUBJECT2_HORN_PATTERN_ONE_SECOND,
    SUBJECT2_HORN_PATTERN_TWO_SECONDS,
    SUBJECT2_HORN_PATTERN_THREE_SECONDS,
    SUBJECT2_HORN_PATTERN_TWO_BEEPS,
    SUBJECT2_HORN_PATTERN_THREE_BEEPS,
    SUBJECT2_HORN_PATTERN_FOUR_BEEPS,
    SUBJECT2_HORN_PATTERN_LONG_SHORT,
    SUBJECT2_HORN_PATTERN_URGENT,
    SUBJECT2_HORN_PATTERN_ALARM
} SUBJECT2_HORN_PATTERN;

typedef struct
{
    uint16_t duration_ms;
    uint16_t frequency_hz;
} SUBJECT2_HORN_SEGMENT;

static const SUBJECT2_HORN_SEGMENT subject2_one_second_segments[] =
{
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT subject2_two_seconds_segments[] =
{
    {2000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT subject2_three_seconds_segments[] =
{
    {3000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT subject2_two_beeps_segments[] =
{
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {1000U, 0U},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT subject2_three_beeps_segments[] =
{
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {1000U, 0U},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {1000U, 0U},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT subject2_four_beeps_segments[] =
{
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {1000U, 0U},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {1000U, 0U},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {1000U, 0U},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT subject2_long_short_segments[] =
{
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {1000U, 0U},
    {3000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT subject2_urgent_segments[] =
{
    {500U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {500U, 0U},
    {500U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {500U, 0U},
    {500U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {500U, 0U},
    {500U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}, {500U, 0U},
    {500U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT subject2_alarm_segments[] =
{
    {1000U, SUBJECT2_HORN_LOW_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_LOW_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_LOW_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_LOW_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_LOW_FREQUENCY_HZ},
    {1000U, SUBJECT2_HORN_NORMAL_FREQUENCY_HZ}
};

static const SUBJECT2_HORN_SEGMENT *subject2_horn_segments(uint8_t pattern,
                                                            uint8_t *segment_count)
{
    const SUBJECT2_HORN_SEGMENT *segments = 0;
    *segment_count = 0U;

    switch(pattern)
    {
        case SUBJECT2_HORN_PATTERN_ONE_SECOND:
            segments = subject2_one_second_segments;
            *segment_count = (uint8_t)(sizeof(subject2_one_second_segments) /
                                       sizeof(subject2_one_second_segments[0]));
            break;
        case SUBJECT2_HORN_PATTERN_TWO_SECONDS:
            segments = subject2_two_seconds_segments;
            *segment_count = (uint8_t)(sizeof(subject2_two_seconds_segments) /
                                       sizeof(subject2_two_seconds_segments[0]));
            break;
        case SUBJECT2_HORN_PATTERN_THREE_SECONDS:
            segments = subject2_three_seconds_segments;
            *segment_count = (uint8_t)(sizeof(subject2_three_seconds_segments) /
                                       sizeof(subject2_three_seconds_segments[0]));
            break;
        case SUBJECT2_HORN_PATTERN_TWO_BEEPS:
            segments = subject2_two_beeps_segments;
            *segment_count = (uint8_t)(sizeof(subject2_two_beeps_segments) /
                                       sizeof(subject2_two_beeps_segments[0]));
            break;
        case SUBJECT2_HORN_PATTERN_THREE_BEEPS:
            segments = subject2_three_beeps_segments;
            *segment_count = (uint8_t)(sizeof(subject2_three_beeps_segments) /
                                       sizeof(subject2_three_beeps_segments[0]));
            break;
        case SUBJECT2_HORN_PATTERN_FOUR_BEEPS:
            segments = subject2_four_beeps_segments;
            *segment_count = (uint8_t)(sizeof(subject2_four_beeps_segments) /
                                       sizeof(subject2_four_beeps_segments[0]));
            break;
        case SUBJECT2_HORN_PATTERN_LONG_SHORT:
            segments = subject2_long_short_segments;
            *segment_count = (uint8_t)(sizeof(subject2_long_short_segments) /
                                       sizeof(subject2_long_short_segments[0]));
            break;
        case SUBJECT2_HORN_PATTERN_URGENT:
            segments = subject2_urgent_segments;
            *segment_count = (uint8_t)(sizeof(subject2_urgent_segments) /
                                       sizeof(subject2_urgent_segments[0]));
            break;
        case SUBJECT2_HORN_PATTERN_ALARM:
            segments = subject2_alarm_segments;
            *segment_count = (uint8_t)(sizeof(subject2_alarm_segments) /
                                       sizeof(subject2_alarm_segments[0]));
            break;
        default:
            break;
    }

    return segments;
}

static void subject2_start_horn_pattern(SUBJECT2_COMMAND_STATE *state, uint8_t pattern)
{
    const SUBJECT2_HORN_SEGMENT *segments;
    uint8_t segment_count;

    segments = subject2_horn_segments(pattern, &segment_count);
    if((segments == 0) || (segment_count == 0U))
    {
        state->horn_pattern = SUBJECT2_HORN_PATTERN_NONE;
        state->horn_segment_index = 0U;
        state->horn_segment_remaining_ms = 0U;
        return;
    }

    state->horn_pattern = pattern;
    state->horn_segment_index = 0U;
    state->horn_segment_remaining_ms = segments[0].duration_ms;
}

void Subject2_command_init(SUBJECT2_COMMAND_STATE *state)
{
    if(state == 0)
    {
        return;
    }

    *state = (SUBJECT2_COMMAND_STATE){0};
    state->indicator_phase = 1U;
}

uint8_t Subject2_command_dispatch(SUBJECT2_COMMAND_STATE *state, uint8_t command_code)
{
    if(state == 0)
    {
        return 0U;
    }

    switch(command_code)
    {
        case SUBJECT2_COMMAND_LEFT_INDICATOR:
            state->left_indicator_enabled = 1U;
            break;
        case SUBJECT2_COMMAND_RIGHT_INDICATOR:
            state->right_indicator_enabled = 1U;
            break;
        case SUBJECT2_COMMAND_HIGH_BEAM:
            state->high_beam_enabled = 1U;
            break;
        case SUBJECT2_COMMAND_LOW_BEAM:
            state->low_beam_enabled = 1U;
            break;
        case SUBJECT2_COMMAND_FOG_LIGHT:
            state->fog_light_enabled = 1U;
            break;
        case SUBJECT2_COMMAND_HAZARD_LIGHT:
            state->hazard_enabled = 1U;
            break;
        case SUBJECT2_COMMAND_INTERIOR_LIGHT:
            state->interior_light_enabled = 1U;
            break;
        case SUBJECT2_COMMAND_WIPER:
            state->wiper_enabled = 1U;
            break;
        case SUBJECT2_COMMAND_HORN_ONE_SECOND:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_ONE_SECOND);
            break;
        case SUBJECT2_COMMAND_HORN_TWO_SECONDS:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_TWO_SECONDS);
            break;
        case SUBJECT2_COMMAND_HORN_THREE_SECONDS:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_THREE_SECONDS);
            break;
        case SUBJECT2_COMMAND_HORN_TWO_BEEPS:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_TWO_BEEPS);
            break;
        case SUBJECT2_COMMAND_HORN_THREE_BEEPS:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_THREE_BEEPS);
            break;
        case SUBJECT2_COMMAND_HORN_FOUR_BEEPS:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_FOUR_BEEPS);
            break;
        case SUBJECT2_COMMAND_HORN_LONG_SHORT:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_LONG_SHORT);
            break;
        case SUBJECT2_COMMAND_HORN_URGENT:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_URGENT);
            break;
        case SUBJECT2_COMMAND_HORN_ALARM:
            subject2_start_horn_pattern(state, SUBJECT2_HORN_PATTERN_ALARM);
            break;
        default:
            return 0U;
    }

    return 1U;
}

void Subject2_command_tick(SUBJECT2_COMMAND_STATE *state, uint32_t elapsed_ms)
{
    const SUBJECT2_HORN_SEGMENT *segments;
    uint8_t segment_count;

    if(state == 0)
    {
        return;
    }

    state->indicator_elapsed_ms = (uint16_t)(state->indicator_elapsed_ms + elapsed_ms);
    while(state->indicator_elapsed_ms >= SUBJECT2_INDICATOR_PERIOD_MS)
    {
        state->indicator_elapsed_ms = (uint16_t)(state->indicator_elapsed_ms -
                                                  SUBJECT2_INDICATOR_PERIOD_MS);
        state->indicator_phase = (uint8_t)(state->indicator_phase == 0U);
    }

    while((state->horn_pattern != SUBJECT2_HORN_PATTERN_NONE) && (elapsed_ms > 0U))
    {
        if(elapsed_ms < state->horn_segment_remaining_ms)
        {
            state->horn_segment_remaining_ms =
                (uint16_t)(state->horn_segment_remaining_ms - elapsed_ms);
            break;
        }

        elapsed_ms -= state->horn_segment_remaining_ms;
        segments = subject2_horn_segments(state->horn_pattern, &segment_count);
        state->horn_segment_index++;
        if((segments == 0) || (state->horn_segment_index >= segment_count))
        {
            state->horn_pattern = SUBJECT2_HORN_PATTERN_NONE;
            state->horn_segment_index = 0U;
            state->horn_segment_remaining_ms = 0U;
            break;
        }
        state->horn_segment_remaining_ms = segments[state->horn_segment_index].duration_ms;
    }
}

void Subject2_command_get_output(const SUBJECT2_COMMAND_STATE *state,
                                 SUBJECT2_OUTPUT_STATE *output)
{
    const SUBJECT2_HORN_SEGMENT *segments;
    uint8_t segment_count;

    if((state == 0) || (output == 0))
    {
        return;
    }

    *output = (SUBJECT2_OUTPUT_STATE){0};
    output->left_indicator_on = (uint8_t)(state->indicator_phase != 0U &&
                                           (state->left_indicator_enabled != 0U ||
                                            state->hazard_enabled != 0U));
    output->right_indicator_on = (uint8_t)(state->indicator_phase != 0U &&
                                            (state->right_indicator_enabled != 0U ||
                                             state->hazard_enabled != 0U));
    output->high_beam_on = state->high_beam_enabled;
    output->low_beam_on = state->low_beam_enabled;
    output->fog_light_on = state->fog_light_enabled;
    output->interior_light_on = state->interior_light_enabled;
    output->wiper_on = state->wiper_enabled;

    segments = subject2_horn_segments(state->horn_pattern, &segment_count);
    if((segments != 0) && (state->horn_segment_index < segment_count))
    {
        output->horn_frequency_hz = segments[state->horn_segment_index].frequency_hz;
        output->horn_on = (uint8_t)(output->horn_frequency_hz != 0U);
    }
}
