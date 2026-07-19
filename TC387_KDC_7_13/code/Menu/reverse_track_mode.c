#include "zf_common_headfile.h"
#include "reverse_track_mode.h"

#include <float.h>

typedef struct
{
    float x_m;
    float y_m;
    float progress_m;
    float distance_squared_m2;
    uint16_t segment_index;
    uint8_t valid;
} ReverseTrackProjection_t;

typedef struct
{
    ACKERMANN_CONTROL_CONFIG config;
    float cumulative_distance_m[FLASH_IMU_POINT_CAPACITY];
    float parking_path_x_m[TRACK_PARK_PATH_POINT_COUNT];
    float parking_path_y_m[TRACK_PARK_PATH_POINT_COUNT];
    float anchor_navigation_x_m;
    float anchor_navigation_y_m;
    float anchor_route_x_m;
    float anchor_route_y_m;
    float alignment_heading_rad;
    float progress_m;
    float speed_reference_mps;
    float steering_reference_rad;
    float steering_center_deg;
    float travel_sign;
    uint16_t point_count;
    uint16_t active_point_count;
    uint16_t current_segment_index;
    uint16_t alignment_segment_index;
    volatile uint8_t enabled;
    volatile uint8_t terminal_latched;
    volatile uint8_t aligned;
    volatile uint8_t outputs_enabled;
    volatile uint8_t route_valid;
    volatile uint8_t parking_pending;
    volatile uint8_t parking_active;
    volatile uint8_t parking_final_stop_pending;
} ReverseTrackContext_t;

static ReverseTrackContext_t g_reverse_track;
static volatile ReverseTrackModeStatus_t g_reverse_track_status;

#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    #define REVERSE_TRACK_TRAVEL_SIGN              (1.0f)
#else
    #define REVERSE_TRACK_TRAVEL_SIGN              (-1.0f)
#endif

static float reverse_track_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float reverse_track_minf(float first, float second)
{
    return (first < second) ? first : second;
}

static float reverse_track_clampf(float value,
                                  float minimum,
                                  float maximum)
{
    if(value > maximum) { return maximum; }
    if(value < minimum) { return minimum; }
    return value;
}

static float reverse_track_approach(float current,
                                    float target,
                                    float maximum_step)
{
    if(current < target - maximum_step) { return current + maximum_step; }
    if(current > target + maximum_step) { return current - maximum_step; }
    return target;
}

static float reverse_track_wrap_pi(float angle_rad)
{
    while(angle_rad >= ACKERMANN_PI) { angle_rad -= 2.0f * ACKERMANN_PI; }
    while(angle_rad < -ACKERMANN_PI) { angle_rad += 2.0f * ACKERMANN_PI; }
    return angle_rad;
}

static uint8_t reverse_track_float_valid(float value)
{
    return (uint8_t)((value == value)
                  && (value <= FLT_MAX)
                  && (value >= -FLT_MAX));
}

static float reverse_track_active_x(uint16_t point_index)
{
    if(g_reverse_track.parking_active != 0U)
    {
        return g_reverse_track.parking_path_x_m[point_index];
    }
    return IMU_X[point_index];
}

static float reverse_track_active_y(uint16_t point_index)
{
    if(g_reverse_track.parking_active != 0U)
    {
        return g_reverse_track.parking_path_y_m[point_index];
    }
    return IMU_Y[point_index];
}

static float reverse_track_segment_length(uint16_t segment_index)
{
    float delta_x;
    float delta_y;

    if((segment_index == 0U) ||
       (segment_index >= g_reverse_track.active_point_count))
    {
        return 0.0f;
    }

    delta_x = reverse_track_active_x(segment_index)
            - reverse_track_active_x(segment_index - 1U);
    delta_y = reverse_track_active_y(segment_index)
            - reverse_track_active_y(segment_index - 1U);
    return sqrtf(delta_x * delta_x + delta_y * delta_y);
}

static void reverse_track_clear_control_status(void)
{
    g_reverse_track_status.curvature_inv_m = 0.0f;
    g_reverse_track_status.target_speed_mps = 0.0f;
    g_reverse_track_status.ramped_speed_mps = 0.0f;
    g_reverse_track_status.steering_target_rad = 0.0f;
    g_reverse_track_status.steering_encoder_target_deg =
        g_reverse_track.steering_center_deg;
    g_reverse_track_status.left_target_mps = 0.0f;
    g_reverse_track_status.left_pwm = 0.0f;
    g_reverse_track_status.right_target_mps = 0.0f;
    g_reverse_track_status.right_pwm = 0.0f;
}

static void reverse_track_stop_outputs(void)
{
    g_reverse_track.speed_reference_mps = 0.0f;
    g_reverse_track.steering_reference_rad = 0.0f;
    Motor_stop_all();
    g_reverse_track.outputs_enabled = 0U;
    g_reverse_track_status.outputs_enabled = 0U;
    reverse_track_clear_control_status();
}

static void reverse_track_latch(ReverseTrackModeState_e state,
                                ReverseTrackFault_e fault)
{
    g_reverse_track.terminal_latched = 1U;
    g_reverse_track_status.state = state;
    g_reverse_track_status.fault = fault;
    reverse_track_stop_outputs();
}

static void reverse_track_print_loaded_points(void)
{
    uint16_t point_count = (uint16_t)IMU_savenum;
    uint16_t point_index;

    printf("TRACK_POINT_BEGIN,%u\r\n", (unsigned)point_count);
    if(point_count > FLASH_IMU_POINT_CAPACITY)
    {
        printf("TRACK_POINT_ERROR,POINT_COUNT,%u,%u\r\n",
               (unsigned)point_count,
               (unsigned)FLASH_IMU_POINT_CAPACITY);
        printf("TRACK_POINT_END,0\r\n");
        return;
    }

    for(point_index = 0U; point_index < point_count; ++point_index)
    {
        printf("TRACK_POINT,%u,%.3f,%.3f\r\n",
               (unsigned)point_index,
               (double)IMU_X[point_index],
               (double)IMU_Y[point_index]);
    }
    printf("TRACK_POINT_END,%u\r\n", (unsigned)point_count);
}

static ReverseTrackFault_e reverse_track_load_route(void)
{
    float delta_x;
    float delta_y;
    float segment_length;
    uint16_t index;
    uint16_t first_valid_segment = 0U;
    uint16_t last_valid_segment = 0U;

    DATA_READ();
    IMU_POINT_READ();
    reverse_track_print_loaded_points();

    g_reverse_track.point_count = (uint16_t)IMU_savenum;
    g_reverse_track_status.point_count = g_reverse_track.point_count;
    if((g_reverse_track.point_count < 2U) ||
       (g_reverse_track.point_count > FLASH_IMU_POINT_CAPACITY))
    {
        return REVERSE_TRACK_FAULT_POINT_COUNT;
    }

    for(index = 0U; index < g_reverse_track.point_count; ++index)
    {
        if((reverse_track_float_valid(IMU_X[index]) == 0U) ||
           (reverse_track_float_valid(IMU_Y[index]) == 0U))
        {
            return REVERSE_TRACK_FAULT_POINT_VALUE;
        }
    }

    g_reverse_track.cumulative_distance_m[0] = 0.0f;
    for(index = 1U; index < g_reverse_track.point_count; ++index)
    {
        delta_x = IMU_X[index] - IMU_X[index - 1U];
        delta_y = IMU_Y[index] - IMU_Y[index - 1U];
        segment_length = sqrtf(delta_x * delta_x + delta_y * delta_y);
        if(reverse_track_float_valid(segment_length) == 0U)
        {
            return REVERSE_TRACK_FAULT_POINT_VALUE;
        }
        g_reverse_track.cumulative_distance_m[index] =
            g_reverse_track.cumulative_distance_m[index - 1U]
          + segment_length;
        if(segment_length >= REVERSE_TRACK_SEGMENT_EPSILON_M)
        {
            if(first_valid_segment == 0U)
            {
                first_valid_segment = index;
            }
            last_valid_segment = index;
        }
    }

    if((last_valid_segment == 0U) ||
       (g_reverse_track.cumulative_distance_m[
            g_reverse_track.point_count - 1U]
        < REVERSE_TRACK_SEGMENT_EPSILON_M))
    {
        return REVERSE_TRACK_FAULT_ZERO_LENGTH_ROUTE;
    }

#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    g_reverse_track.alignment_segment_index = first_valid_segment;
    g_reverse_track.current_segment_index = first_valid_segment;
    g_reverse_track.progress_m = 0.0f;
#else
    g_reverse_track.alignment_segment_index = last_valid_segment;
    g_reverse_track.current_segment_index = last_valid_segment;
    g_reverse_track.progress_m =
        g_reverse_track.cumulative_distance_m[
            g_reverse_track.point_count - 1U];
#endif
    g_reverse_track.active_point_count = g_reverse_track.point_count;
    g_reverse_track.parking_pending = 0U;
    g_reverse_track.parking_active = 0U;
    g_reverse_track.parking_final_stop_pending = 0U;
    g_reverse_track.travel_sign = REVERSE_TRACK_TRAVEL_SIGN;
    g_reverse_track.route_valid = 1U;
    g_reverse_track_status.route_valid = 1U;
    g_reverse_track_status.segment_index =
        g_reverse_track.current_segment_index;
    g_reverse_track_status.remaining_distance_m =
        g_reverse_track.cumulative_distance_m[
            g_reverse_track.point_count - 1U];
    return REVERSE_TRACK_FAULT_NONE;
}

static void reverse_track_align(const TopSpeed_GPS_INS_Output *navigation)
{
    float segment_delta_x;
    float segment_delta_y;
    float route_heading_rad;
    float navigation_heading_rad;
    uint16_t anchor_point_index;
    uint16_t segment_index = g_reverse_track.alignment_segment_index;

    segment_delta_x = IMU_X[segment_index]
                    - IMU_X[segment_index - 1U];
    segment_delta_y = IMU_Y[segment_index]
                    - IMU_Y[segment_index - 1U];
    /* Stored X points right and stored Y points forward, matching compass yaw. */
    route_heading_rad = atan2f(segment_delta_x, segment_delta_y);
    navigation_heading_rad = navigation->heading_deg * ACKERMANN_DEG_TO_RAD;

    g_reverse_track.anchor_navigation_x_m = navigation->position_m.x;
    g_reverse_track.anchor_navigation_y_m = navigation->position_m.y;
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    anchor_point_index = 0U;
#else
    anchor_point_index = g_reverse_track.point_count - 1U;
#endif
    g_reverse_track.anchor_route_x_m = IMU_X[anchor_point_index];
    g_reverse_track.anchor_route_y_m = IMU_Y[anchor_point_index];
    g_reverse_track.alignment_heading_rad = reverse_track_wrap_pi(
        navigation_heading_rad - route_heading_rad);
    g_reverse_track.current_segment_index =
        g_reverse_track.alignment_segment_index;
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    g_reverse_track.progress_m = 0.0f;
#else
    g_reverse_track.progress_m =
        g_reverse_track.cumulative_distance_m[
            g_reverse_track.point_count - 1U];
#endif
    g_reverse_track.speed_reference_mps = 0.0f;
    g_reverse_track.steering_reference_rad = 0.0f;
    g_reverse_track.aligned = 1U;
    g_reverse_track_status.aligned = 1U;
}

static void reverse_track_navigation_to_route(
    const TopSpeed_GPS_INS_Output *navigation,
    float *route_x_m,
    float *route_y_m,
    float *route_heading_rad)
{
    float delta_east_m;
    float delta_north_m;
    float cosine;
    float sine;

    delta_east_m = navigation->position_m.x
                 - g_reverse_track.anchor_navigation_x_m;
    delta_north_m = navigation->position_m.y
                  - g_reverse_track.anchor_navigation_y_m;
    cosine = cosf(g_reverse_track.alignment_heading_rad);
    sine = sinf(g_reverse_track.alignment_heading_rad);

    *route_x_m = g_reverse_track.anchor_route_x_m
               + delta_east_m * cosine
               - delta_north_m * sine;
    *route_y_m = g_reverse_track.anchor_route_y_m
               + delta_east_m * sine
               + delta_north_m * cosine;
    *route_heading_rad = reverse_track_wrap_pi(
        navigation->heading_deg * ACKERMANN_DEG_TO_RAD
      - g_reverse_track.alignment_heading_rad);
}

#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD) && \
    (TRACK_FORWARD_PARK_ENABLE != 0U)
static void reverse_track_update_parking_errors(float route_x_m,
                                                float route_y_m,
                                                float route_heading_rad)
{
    float delta_x_m = TRACK_PARK_TARGET_X_M - route_x_m;
    float delta_y_m = TRACK_PARK_TARGET_Y_M - route_y_m;
    float target_heading_rad = TRACK_PARK_TARGET_YAW_DEG
                             * ACKERMANN_DEG_TO_RAD;
    float heading_error_rad = reverse_track_wrap_pi(
        target_heading_rad - route_heading_rad);

    g_reverse_track_status.parking_position_error_m =
        sqrtf(delta_x_m * delta_x_m + delta_y_m * delta_y_m);
    g_reverse_track_status.parking_heading_error_deg =
        reverse_track_absf(heading_error_rad) * ACKERMANN_RAD_TO_DEG;
}

static ReverseTrackFault_e reverse_track_generate_parking_path(
    float start_x_m,
    float start_y_m,
    float start_heading_rad)
{
    float target_heading_rad;
    float chord_x_m;
    float chord_y_m;
    float chord_length_m;
    float handle_length_m;
    float start_forward_x;
    float start_forward_y;
    float target_forward_x;
    float target_forward_y;
    float control1_x_m;
    float control1_y_m;
    float control2_x_m;
    float control2_y_m;
    float one_minus_t;
    float t;
    float delta_x_m;
    float delta_y_m;
    float segment_length_m;
    uint16_t first_valid_segment = 0U;
    uint16_t index;

    if((reverse_track_float_valid(TRACK_PARK_TARGET_X_M) == 0U) ||
       (reverse_track_float_valid(TRACK_PARK_TARGET_Y_M) == 0U) ||
       (reverse_track_float_valid(TRACK_PARK_TARGET_YAW_DEG) == 0U))
    {
        return REVERSE_TRACK_FAULT_PARK_CONFIG;
    }

    target_heading_rad = reverse_track_wrap_pi(
        TRACK_PARK_TARGET_YAW_DEG * ACKERMANN_DEG_TO_RAD);
    chord_x_m = TRACK_PARK_TARGET_X_M - start_x_m;
    chord_y_m = TRACK_PARK_TARGET_Y_M - start_y_m;
    chord_length_m = sqrtf(chord_x_m * chord_x_m + chord_y_m * chord_y_m);
    if((reverse_track_float_valid(chord_length_m) == 0U) ||
       (chord_length_m < TRACK_PARK_MIN_PATH_DISTANCE_M))
    {
        return REVERSE_TRACK_FAULT_PARK_CONFIG;
    }

    handle_length_m = reverse_track_clampf(
        chord_length_m * TRACK_PARK_BEZIER_HANDLE_RATIO,
        TRACK_PARK_BEZIER_HANDLE_MIN_M,
        TRACK_PARK_BEZIER_HANDLE_MAX_M);
    start_forward_x = sinf(start_heading_rad);
    start_forward_y = cosf(start_heading_rad);
    target_forward_x = sinf(target_heading_rad);
    target_forward_y = cosf(target_heading_rad);

    /* The path parameter increases in the reverse travel direction. */
    control1_x_m = start_x_m - start_forward_x * handle_length_m;
    control1_y_m = start_y_m - start_forward_y * handle_length_m;
    control2_x_m = TRACK_PARK_TARGET_X_M
                 + target_forward_x * handle_length_m;
    control2_y_m = TRACK_PARK_TARGET_Y_M
                 + target_forward_y * handle_length_m;

    for(index = 0U; index < TRACK_PARK_PATH_POINT_COUNT; ++index)
    {
        t = (float)index / (float)(TRACK_PARK_PATH_POINT_COUNT - 1U);
        one_minus_t = 1.0f - t;
        g_reverse_track.parking_path_x_m[index] =
            one_minus_t * one_minus_t * one_minus_t * start_x_m
          + 3.0f * one_minus_t * one_minus_t * t * control1_x_m
          + 3.0f * one_minus_t * t * t * control2_x_m
          + t * t * t * TRACK_PARK_TARGET_X_M;
        g_reverse_track.parking_path_y_m[index] =
            one_minus_t * one_minus_t * one_minus_t * start_y_m
          + 3.0f * one_minus_t * one_minus_t * t * control1_y_m
          + 3.0f * one_minus_t * t * t * control2_y_m
          + t * t * t * TRACK_PARK_TARGET_Y_M;
    }

    g_reverse_track.cumulative_distance_m[0] = 0.0f;
    for(index = 1U; index < TRACK_PARK_PATH_POINT_COUNT; ++index)
    {
        delta_x_m = g_reverse_track.parking_path_x_m[index]
                  - g_reverse_track.parking_path_x_m[index - 1U];
        delta_y_m = g_reverse_track.parking_path_y_m[index]
                  - g_reverse_track.parking_path_y_m[index - 1U];
        segment_length_m = sqrtf(delta_x_m * delta_x_m
                               + delta_y_m * delta_y_m);
        if(reverse_track_float_valid(segment_length_m) == 0U)
        {
            return REVERSE_TRACK_FAULT_PARK_CONFIG;
        }
        g_reverse_track.cumulative_distance_m[index] =
            g_reverse_track.cumulative_distance_m[index - 1U]
          + segment_length_m;
        if((first_valid_segment == 0U) &&
           (segment_length_m >= REVERSE_TRACK_SEGMENT_EPSILON_M))
        {
            first_valid_segment = index;
        }
    }
    if((first_valid_segment == 0U) ||
       (g_reverse_track.cumulative_distance_m[
            TRACK_PARK_PATH_POINT_COUNT - 1U]
        < TRACK_PARK_MIN_PATH_DISTANCE_M))
    {
        return REVERSE_TRACK_FAULT_PARK_CONFIG;
    }

    g_reverse_track.active_point_count = TRACK_PARK_PATH_POINT_COUNT;
    g_reverse_track.current_segment_index = first_valid_segment;
    g_reverse_track.progress_m = 0.0f;
    g_reverse_track.speed_reference_mps = 0.0f;
    g_reverse_track.steering_reference_rad = 0.0f;
    g_reverse_track.travel_sign = -1.0f;
    g_reverse_track.parking_pending = 0U;
    g_reverse_track.parking_active = 1U;
    g_reverse_track.parking_final_stop_pending = 0U;
    g_reverse_track_status.parking_active = 1U;
    g_reverse_track_status.segment_index = first_valid_segment;
    g_reverse_track_status.remaining_distance_m =
        g_reverse_track.cumulative_distance_m[
            TRACK_PARK_PATH_POINT_COUNT - 1U];
    return REVERSE_TRACK_FAULT_NONE;
}

static void reverse_track_request_parking(void)
{
    g_reverse_track.parking_pending = 1U;
    g_reverse_track.parking_active = 0U;
    g_reverse_track.parking_final_stop_pending = 0U;
    g_reverse_track_status.parking_active = 0U;
    g_reverse_track_status.state = REVERSE_TRACK_STATE_WAIT_PARK_STOP;
    reverse_track_stop_outputs();
}
#endif

static ReverseTrackProjection_t reverse_track_project_segment(
    uint16_t segment_index,
    float route_x_m,
    float route_y_m)
{
    ReverseTrackProjection_t projection;
    float start_x;
    float start_y;
    float delta_x;
    float delta_y;
    float length_squared;
    float relative_x;
    float relative_y;
    float parameter;
    float error_x;
    float error_y;

    memset(&projection, 0, sizeof(projection));
    projection.segment_index = segment_index;
    if((segment_index == 0U) ||
       (segment_index >= g_reverse_track.active_point_count))
    {
        return projection;
    }

    start_x = reverse_track_active_x(segment_index - 1U);
    start_y = reverse_track_active_y(segment_index - 1U);
    delta_x = reverse_track_active_x(segment_index) - start_x;
    delta_y = reverse_track_active_y(segment_index) - start_y;
    length_squared = delta_x * delta_x + delta_y * delta_y;
    if(length_squared < REVERSE_TRACK_SEGMENT_EPSILON_M
                      * REVERSE_TRACK_SEGMENT_EPSILON_M)
    {
        return projection;
    }

    relative_x = route_x_m - start_x;
    relative_y = route_y_m - start_y;
    parameter = (relative_x * delta_x + relative_y * delta_y)
              / length_squared;
    parameter = reverse_track_clampf(parameter, 0.0f, 1.0f);
    projection.x_m = start_x + parameter * delta_x;
    projection.y_m = start_y + parameter * delta_y;
    error_x = route_x_m - projection.x_m;
    error_y = route_y_m - projection.y_m;
    projection.distance_squared_m2 = error_x * error_x + error_y * error_y;
    projection.progress_m =
        g_reverse_track.cumulative_distance_m[segment_index - 1U]
      + parameter * sqrtf(length_squared);
    projection.valid = 1U;
    return projection;
}

static ReverseTrackProjection_t reverse_track_find_projection(
    float route_x_m,
    float route_y_m)
{
    ReverseTrackProjection_t best;
    ReverseTrackProjection_t candidate;
    uint16_t segment_index = g_reverse_track.current_segment_index;
    uint8_t searched = 0U;

    memset(&best, 0, sizeof(best));
    best.distance_squared_m2 = FLT_MAX;
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    while(segment_index < g_reverse_track.active_point_count)
#else
    while(segment_index > 0U)
#endif
    {
        candidate = reverse_track_project_segment(segment_index,
                                                  route_x_m,
                                                  route_y_m);
        if((candidate.valid != 0U) &&
           ((candidate.distance_squared_m2 < best.distance_squared_m2) ||
             ((candidate.distance_squared_m2 == best.distance_squared_m2) &&
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
              (candidate.progress_m > best.progress_m))))
#else
              (candidate.progress_m < best.progress_m))))
#endif
        {
            best = candidate;
        }

        if(candidate.valid != 0U)
        {
            if(searched >= REVERSE_TRACK_SEGMENT_SEARCH_COUNT) { break; }
            ++searched;
        }
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
        ++segment_index;
#else
        --segment_index;
#endif
    }
    return best;
}

static uint8_t reverse_track_sample_route(float progress_m,
                                          float *route_x_m,
                                          float *route_y_m)
{
    float segment_length;
    float parameter;
    uint16_t segment_index;

    if((route_x_m == 0) || (route_y_m == 0)) { return 0U; }
    if(progress_m <= 0.0f)
    {
        *route_x_m = reverse_track_active_x(0U);
        *route_y_m = reverse_track_active_y(0U);
        return 1U;
    }
    if(progress_m >= g_reverse_track.cumulative_distance_m[
                         g_reverse_track.active_point_count - 1U])
    {
        *route_x_m = reverse_track_active_x(
            g_reverse_track.active_point_count - 1U);
        *route_y_m = reverse_track_active_y(
            g_reverse_track.active_point_count - 1U);
        return 1U;
    }

    segment_index = g_reverse_track.current_segment_index;
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    while(segment_index < g_reverse_track.active_point_count)
#else
    while(segment_index > 0U)
#endif
    {
        segment_length = reverse_track_segment_length(segment_index);
        if((segment_length >= REVERSE_TRACK_SEGMENT_EPSILON_M) &&
           (progress_m >=
                g_reverse_track.cumulative_distance_m[segment_index - 1U]) &&
           (progress_m <=
                g_reverse_track.cumulative_distance_m[segment_index]))
        {
            parameter = (progress_m
                       - g_reverse_track.cumulative_distance_m[
                            segment_index - 1U]) / segment_length;
            *route_x_m = reverse_track_active_x(segment_index - 1U)
                       + parameter * (reverse_track_active_x(segment_index)
                                    - reverse_track_active_x(
                                          segment_index - 1U));
            *route_y_m = reverse_track_active_y(segment_index - 1U)
                       + parameter * (reverse_track_active_y(segment_index)
                                    - reverse_track_active_y(
                                          segment_index - 1U));
            return 1U;
        }
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
        ++segment_index;
#else
        --segment_index;
#endif
    }

#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    *route_x_m = reverse_track_active_x(
        g_reverse_track.active_point_count - 1U);
    *route_y_m = reverse_track_active_y(
        g_reverse_track.active_point_count - 1U);
#else
    *route_x_m = reverse_track_active_x(0U);
    *route_y_m = reverse_track_active_y(0U);
#endif
    return 1U;
}

static void reverse_track_scale_wheel_targets(float *left_target_mps,
                                              float *right_target_mps)
{
    float largest_target;
    float scale;

    largest_target = reverse_track_absf(*left_target_mps);
    if(reverse_track_absf(*right_target_mps) > largest_target)
    {
        largest_target = reverse_track_absf(*right_target_mps);
    }
    if(largest_target > g_reverse_track.config.max_wheel_speed_mps)
    {
        scale = g_reverse_track.config.max_wheel_speed_mps / largest_target;
        *left_target_mps *= scale;
        *right_target_mps *= scale;
    }
}

static void reverse_track_run_control(
    const ACKERMANN_CONTROL_TELEMETRY *telemetry,
    float route_x_m,
    float route_y_m,
    float route_heading_rad)
{
    ReverseTrackProjection_t projection;
    MOTOR_REAR_SPEED_INPUT left_input;
    MOTOR_REAR_SPEED_INPUT right_input;
    MOTOR_REAR_SPEED_STATUS left_status;
    MOTOR_REAR_SPEED_STATUS right_status;
    float lookahead_distance_m;
    float lookahead_progress_m;
    float lookahead_x_m;
    float lookahead_y_m;
    float target_delta_x;
    float target_delta_y;
    float target_distance_squared_m2;
    float target_left_m;
    float curvature_inv_m;
    float desired_steering_rad;
    float maximum_steering_rad;
    float maximum_steering_step_rad;
    float end_speed_limit_mps;
    float curve_speed_limit_mps;
    float speed_magnitude_mps;
    float desired_speed_mps;
    float measured_steering_rad;
    float left_speed_mps;
    float right_speed_mps;
    float left_target_mps;
    float right_target_mps;
    float previous_progress;
    float remaining_distance_m;
    float active_total_distance_m;
    float maximum_speed_mps;
    float stopping_margin_m;
    float parking_slow_speed_mps;
    uint8_t rear_enabled;

    projection = reverse_track_find_projection(route_x_m, route_y_m);
    if(projection.valid == 0U)
    {
        reverse_track_latch(REVERSE_TRACK_STATE_ROUTE_FAULT,
                            REVERSE_TRACK_FAULT_PATH_PROJECTION);
        return;
    }

    previous_progress = g_reverse_track.progress_m;
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    if(projection.progress_m > previous_progress)
#else
    if(projection.progress_m < previous_progress)
#endif
    {
        g_reverse_track.progress_m = projection.progress_m;
        g_reverse_track.current_segment_index = projection.segment_index;
    }
    g_reverse_track_status.segment_index =
        g_reverse_track.current_segment_index;
    active_total_distance_m = g_reverse_track.cumulative_distance_m[
        g_reverse_track.active_point_count - 1U];
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    remaining_distance_m = active_total_distance_m
                         - g_reverse_track.progress_m;
#else
    (void)active_total_distance_m;
    remaining_distance_m = g_reverse_track.progress_m;
#endif
    g_reverse_track_status.remaining_distance_m = remaining_distance_m;

#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD) && \
    (TRACK_FORWARD_PARK_ENABLE != 0U)
    if(g_reverse_track.parking_active != 0U)
    {
        reverse_track_update_parking_errors(route_x_m,
                                            route_y_m,
                                            route_heading_rad);
        if((g_reverse_track_status.parking_position_error_m <=
                TRACK_PARK_POSITION_TOLERANCE_M) &&
           (g_reverse_track_status.parking_heading_error_deg <=
                TRACK_PARK_HEADING_TOLERANCE_DEG))
        {
            g_reverse_track.parking_final_stop_pending = 1U;
            g_reverse_track_status.state =
                REVERSE_TRACK_STATE_WAIT_PARK_COMPLETE;
            reverse_track_stop_outputs();
            return;
        }
        if(remaining_distance_m <= TRACK_PARK_PATH_END_EPSILON_M)
        {
            reverse_track_latch(REVERSE_TRACK_STATE_ROUTE_FAULT,
                                REVERSE_TRACK_FAULT_PARK_POSE);
            return;
        }
    }
    else
#endif
    {
        if(remaining_distance_m <= REVERSE_TRACK_STOP_DISTANCE_M)
        {
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD) && \
    (TRACK_FORWARD_PARK_ENABLE != 0U)
            reverse_track_request_parking();
#else
            reverse_track_latch(REVERSE_TRACK_STATE_COMPLETED,
                                REVERSE_TRACK_FAULT_NONE);
#endif
            return;
        }
    }

    if(g_reverse_track.parking_active != 0U)
    {
        lookahead_distance_m = TRACK_PARK_LOOKAHEAD_BASE_M
            + TRACK_PARK_LOOKAHEAD_TIME_S
            * reverse_track_absf(telemetry->measured_vehicle_speed_mps);
        lookahead_distance_m = reverse_track_clampf(
            lookahead_distance_m,
            TRACK_PARK_LOOKAHEAD_MIN_M,
            TRACK_PARK_LOOKAHEAD_MAX_M);
    }
    else
    {
        lookahead_distance_m = REVERSE_TRACK_LOOKAHEAD_BASE_M
            + REVERSE_TRACK_LOOKAHEAD_TIME_S
            * reverse_track_absf(telemetry->measured_vehicle_speed_mps);
        lookahead_distance_m = reverse_track_clampf(
            lookahead_distance_m,
            REVERSE_TRACK_LOOKAHEAD_MIN_M,
            REVERSE_TRACK_LOOKAHEAD_MAX_M);
    }
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD)
    lookahead_progress_m = g_reverse_track.progress_m + lookahead_distance_m;
    if(lookahead_progress_m > active_total_distance_m)
    {
        lookahead_progress_m = active_total_distance_m;
    }
#else
    lookahead_progress_m = g_reverse_track.progress_m - lookahead_distance_m;
    if(lookahead_progress_m < 0.0f) { lookahead_progress_m = 0.0f; }
#endif
    if(reverse_track_sample_route(lookahead_progress_m,
                                  &lookahead_x_m,
                                  &lookahead_y_m) == 0U)
    {
        reverse_track_latch(REVERSE_TRACK_STATE_ROUTE_FAULT,
                            REVERSE_TRACK_FAULT_PATH_PROJECTION);
        return;
    }

    g_reverse_track_status.lookahead_x_m = lookahead_x_m;
    g_reverse_track_status.lookahead_y_m = lookahead_y_m;
    target_delta_x = lookahead_x_m - route_x_m;
    target_delta_y = lookahead_y_m - route_y_m;
    target_distance_squared_m2 = target_delta_x * target_delta_x
                               + target_delta_y * target_delta_y;
    if(target_distance_squared_m2 < REVERSE_TRACK_SEGMENT_EPSILON_M
                                  * REVERSE_TRACK_SEGMENT_EPSILON_M)
    {
        reverse_track_latch(REVERSE_TRACK_STATE_ROUTE_FAULT,
                            REVERSE_TRACK_FAULT_PATH_PROJECTION);
        return;
    }

    /* Positive target_left_m means the travel-direction target is to the left. */
    target_left_m = -target_delta_x * cosf(route_heading_rad)
                  + target_delta_y * sinf(route_heading_rad);
    curvature_inv_m = 2.0f * target_left_m / target_distance_squared_m2;
    desired_steering_rad = atanf(g_reverse_track.config.wheelbase_m
                               * curvature_inv_m);
    maximum_steering_rad = REVERSE_TRACK_MAX_STEERING_DEG
                         * ACKERMANN_DEG_TO_RAD;
    desired_steering_rad = reverse_track_clampf(desired_steering_rad,
                                                -maximum_steering_rad,
                                                maximum_steering_rad);
    maximum_steering_step_rad = REVERSE_TRACK_MAX_STEERING_RATE_DPS
                              * ACKERMANN_DEG_TO_RAD
                              * ACKERMANN_CONTROL_PERIOD_S;
    g_reverse_track.steering_reference_rad = reverse_track_approach(
        g_reverse_track.steering_reference_rad,
        desired_steering_rad,
        maximum_steering_step_rad);

    maximum_speed_mps = REVERSE_TRACK_MAX_SPEED_MPS;
    stopping_margin_m = REVERSE_TRACK_STOP_DISTANCE_M;
    if(g_reverse_track.parking_active != 0U)
    {
        maximum_speed_mps = TRACK_PARK_MAX_REVERSE_SPEED_MPS;
        stopping_margin_m = 0.0f;
    }
    end_speed_limit_mps = sqrtf(2.0f * REVERSE_TRACK_ACCELERATION_MPS2
        * reverse_track_clampf(remaining_distance_m - stopping_margin_m,
                               0.0f,
                               FLT_MAX));
    curve_speed_limit_mps = sqrtf(REVERSE_TRACK_MAX_LATERAL_ACCEL_MPS2
        / reverse_track_clampf(reverse_track_absf(curvature_inv_m),
                              0.01f,
                              FLT_MAX));
    speed_magnitude_mps = reverse_track_minf(maximum_speed_mps,
                                             end_speed_limit_mps);
    speed_magnitude_mps = reverse_track_minf(speed_magnitude_mps,
                                             curve_speed_limit_mps);
    if(g_reverse_track.parking_active != 0U)
    {
        parking_slow_speed_mps = TRACK_PARK_MAX_REVERSE_SPEED_MPS
            * reverse_track_clampf(
                g_reverse_track_status.parking_position_error_m
                / TRACK_PARK_SLOWDOWN_DISTANCE_M,
                0.0f,
                1.0f);
        speed_magnitude_mps = reverse_track_minf(speed_magnitude_mps,
                                                 parking_slow_speed_mps);
    }
    desired_speed_mps = g_reverse_track.travel_sign * speed_magnitude_mps;
    g_reverse_track.speed_reference_mps = reverse_track_approach(
        g_reverse_track.speed_reference_mps,
        desired_speed_mps,
        REVERSE_TRACK_ACCELERATION_MPS2 * ACKERMANN_CONTROL_PERIOD_S);

    /* Pure-pursuit supplies the steering target, but wheel differential must
     * follow the steering mechanism's actual encoder feedback. */
    measured_steering_rad = Ackermann_encoder_angle_to_road_steering_rad(
        Servo_get_angle(),
        &g_reverse_track.config);
    Ackermann_electronic_differential(
        g_reverse_track.speed_reference_mps,
        measured_steering_rad,
        &g_reverse_track.config,
        &left_speed_mps,
        &right_speed_mps);
    left_target_mps = left_speed_mps;
    right_target_mps = right_speed_mps;
    reverse_track_scale_wheel_targets(&left_target_mps,
                                      &right_target_mps);
    rear_enabled = (uint8_t)(
        reverse_track_absf(g_reverse_track.speed_reference_mps)
        > g_reverse_track.config.stop_speed_threshold_mps);

    left_input.target_mps = left_target_mps;
    left_input.measured_mps = telemetry->left_measured_mps;
    left_input.feedforward_pwm =
        g_reverse_track.config.left_feedforward_pwm_per_mps
        * left_target_mps;
    left_input.output_sign = g_reverse_track.config.left_motor_sign;
    left_input.pwm_limit = REVERSE_TRACK_PWM_LIMIT;
    left_input.enabled = rear_enabled;

    right_input.target_mps = right_target_mps;
    right_input.measured_mps = telemetry->right_measured_mps;
    right_input.feedforward_pwm =
        g_reverse_track.config.right_feedforward_pwm_per_mps
        * right_target_mps;
    right_input.output_sign = g_reverse_track.config.right_motor_sign;
    right_input.pwm_limit = REVERSE_TRACK_PWM_LIMIT;
    right_input.enabled = rear_enabled;

    Motor_rear_speed_control(&left_input, &right_input, 0U);
    Motor_get_rear_speed_status(&left_status, &right_status);

    g_reverse_track_status.steering_encoder_target_deg =
        Ackermann_road_steering_to_encoder_angle_deg(
            g_reverse_track.steering_reference_rad,
            &g_reverse_track.config);
    Servo_set_angle(g_reverse_track_status.steering_encoder_target_deg);
    Servo_position_control(&servo_duty);

    g_reverse_track_status.lookahead_distance_m =
        sqrtf(target_distance_squared_m2);
    g_reverse_track_status.curvature_inv_m = curvature_inv_m;
    g_reverse_track_status.target_speed_mps = desired_speed_mps;
    g_reverse_track_status.ramped_speed_mps =
        g_reverse_track.speed_reference_mps;
    g_reverse_track_status.steering_target_rad =
        g_reverse_track.steering_reference_rad;
    g_reverse_track_status.left_target_mps = left_status.target_mps;
    g_reverse_track_status.left_measured_mps = left_status.measured_mps;
    g_reverse_track_status.left_pwm = left_status.pwm;
    g_reverse_track_status.right_target_mps = right_status.target_mps;
    g_reverse_track_status.right_measured_mps = right_status.measured_mps;
    g_reverse_track_status.right_pwm = right_status.pwm;
}

void ReverseTrackMode_Init(void)
{
    memset(&g_reverse_track, 0, sizeof(g_reverse_track));
    memset((void *)&g_reverse_track_status, 0,
           sizeof(ReverseTrackModeStatus_t));
    Ackermann_get_default_config(&g_reverse_track.config);
    g_reverse_track.steering_center_deg = Ackermann_get_steering_center();
    g_reverse_track.config.steering_center_encoder_deg =
        g_reverse_track.steering_center_deg;
    g_reverse_track_status.state = REVERSE_TRACK_STATE_IDLE;
    g_reverse_track_status.steering_encoder_target_deg =
        g_reverse_track.steering_center_deg;
}

void ReverseTrackMode_Enter(void)
{
    ReverseTrackFault_e route_fault;

    Ackermann_control_enable(0U);
    reverse_track_stop_outputs();
    if(g_reverse_track.terminal_latched != 0U) { return; }

    /* Keep the 5 ms controller disabled while flash data is read and printed. */
    g_reverse_track.enabled = 0U;
    g_reverse_track.aligned = 0U;
    g_reverse_track.route_valid = 0U;
    g_reverse_track_status.state = REVERSE_TRACK_STATE_WAIT_NAV;
    g_reverse_track_status.fault = REVERSE_TRACK_FAULT_NONE;
    g_reverse_track_status.route_valid = 0U;
    g_reverse_track_status.aligned = 0U;
    g_reverse_track_status.point_count = 0U;
    g_reverse_track_status.segment_index = 0U;
    g_reverse_track_status.local_x_m = 0.0f;
    g_reverse_track_status.local_y_m = 0.0f;
    g_reverse_track_status.remaining_distance_m = 0.0f;
    g_reverse_track_status.lookahead_x_m = 0.0f;
    g_reverse_track_status.lookahead_y_m = 0.0f;
    g_reverse_track_status.lookahead_distance_m = 0.0f;
    g_reverse_track_status.parking_active = 0U;
    g_reverse_track_status.parking_position_error_m = 0.0f;
    g_reverse_track_status.parking_heading_error_deg = 0.0f;
    g_reverse_track.parking_pending = 0U;
    g_reverse_track.parking_active = 0U;
    g_reverse_track.parking_final_stop_pending = 0U;
    reverse_track_clear_control_status();

    route_fault = reverse_track_load_route();
    if(route_fault != REVERSE_TRACK_FAULT_NONE)
    {
        reverse_track_latch(REVERSE_TRACK_STATE_ROUTE_FAULT, route_fault);
        return;
    }
    g_reverse_track.enabled = 1U;
}

void ReverseTrackMode_Task(void)
{
    /* Route loading is completed once in Enter(), outside the 5 ms ISR. */
}

void ReverseTrackMode_5msCallback(void)
{
    TopSpeed_GPS_INS_Output navigation;
    ACKERMANN_CONTROL_TELEMETRY telemetry;
    float route_x_m;
    float route_y_m;
    float route_heading_rad;
    ReverseTrackFault_e parking_fault = REVERSE_TRACK_FAULT_NONE;
    uint8_t navigation_valid;
    uint8_t encoder_valid;

    if((g_reverse_track.enabled == 0U) ||
       (g_reverse_track.terminal_latched != 0U))
    {
        return;
    }

    TopSpeed_GPS_INS_PortGetOutput(&navigation);
    Ackermann_get_telemetry(&telemetry);
    g_reverse_track_status.left_measured_mps =
        telemetry.left_measured_mps;
    g_reverse_track_status.right_measured_mps =
        telemetry.right_measured_mps;

    navigation_valid = (uint8_t)(
        (navigation.initialized != 0U)
        && (navigation.valid != 0U)
        && (navigation.heading_valid != 0U)
        && (reverse_track_float_valid(navigation.position_m.x) != 0U)
        && (reverse_track_float_valid(navigation.position_m.y) != 0U)
        && (reverse_track_float_valid(navigation.heading_deg) != 0U));
    encoder_valid = (uint8_t)(
        (telemetry.encoder_valid != 0U)
        && (navigation.encoder_valid != 0U)
        && (reverse_track_float_valid(telemetry.left_measured_mps) != 0U)
        && (reverse_track_float_valid(telemetry.right_measured_mps) != 0U)
        && (reverse_track_float_valid(
                telemetry.measured_vehicle_speed_mps) != 0U));
    if((navigation_valid == 0U) || (encoder_valid == 0U))
    {
        if(g_reverse_track.aligned != 0U)
        {
            reverse_track_latch(
                REVERSE_TRACK_STATE_SENSOR_FAULT,
                (navigation_valid == 0U)
                    ? REVERSE_TRACK_FAULT_NAVIGATION
                    : REVERSE_TRACK_FAULT_ENCODER);
        }
        else
        {
            g_reverse_track_status.state = REVERSE_TRACK_STATE_WAIT_NAV;
        }
        return;
    }

    if(g_reverse_track.aligned == 0U)
    {
        reverse_track_align(&navigation);
        Motor_rear_speed_pid_reset();
        Servo_pid_reset();
        Motor_enable_channels(1U, 1U, 1U);
        g_reverse_track.outputs_enabled = 1U;
        g_reverse_track_status.outputs_enabled = 1U;
        g_reverse_track_status.state = REVERSE_TRACK_STATE_RUNNING;
    }

    reverse_track_navigation_to_route(&navigation,
                                      &route_x_m,
                                      &route_y_m,
                                      &route_heading_rad);
    g_reverse_track_status.local_x_m = route_x_m;
    g_reverse_track_status.local_y_m = route_y_m;

#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD) && \
    (TRACK_FORWARD_PARK_ENABLE != 0U)
    if(g_reverse_track.parking_final_stop_pending != 0U)
    {
        reverse_track_update_parking_errors(route_x_m,
                                            route_y_m,
                                            route_heading_rad);
        g_reverse_track_status.state =
            REVERSE_TRACK_STATE_WAIT_PARK_COMPLETE;
        if(reverse_track_absf(telemetry.measured_vehicle_speed_mps) >
           TRACK_PARK_ENTRY_STOP_SPEED_MPS)
        {
            return;
        }
        if((g_reverse_track_status.parking_position_error_m <=
                TRACK_PARK_POSITION_TOLERANCE_M) &&
           (g_reverse_track_status.parking_heading_error_deg <=
                TRACK_PARK_HEADING_TOLERANCE_DEG))
        {
            reverse_track_latch(REVERSE_TRACK_STATE_COMPLETED,
                                REVERSE_TRACK_FAULT_NONE);
        }
        else
        {
            reverse_track_latch(REVERSE_TRACK_STATE_ROUTE_FAULT,
                                REVERSE_TRACK_FAULT_PARK_POSE);
        }
        return;
    }

    if(g_reverse_track.parking_pending != 0U)
    {
        reverse_track_update_parking_errors(route_x_m,
                                            route_y_m,
                                            route_heading_rad);
        g_reverse_track_status.state = REVERSE_TRACK_STATE_WAIT_PARK_STOP;
        if(reverse_track_absf(telemetry.measured_vehicle_speed_mps) >
           TRACK_PARK_ENTRY_STOP_SPEED_MPS)
        {
            return;
        }

        parking_fault = reverse_track_generate_parking_path(
            route_x_m,
            route_y_m,
            route_heading_rad);
        if(parking_fault != REVERSE_TRACK_FAULT_NONE)
        {
            reverse_track_latch(REVERSE_TRACK_STATE_ROUTE_FAULT,
                                parking_fault);
            return;
        }
        Motor_rear_speed_pid_reset();
        Servo_pid_reset();
        Motor_enable_channels(1U, 1U, 1U);
        g_reverse_track.outputs_enabled = 1U;
        g_reverse_track_status.outputs_enabled = 1U;
        g_reverse_track_status.state = REVERSE_TRACK_STATE_PARKING;
    }
#else
    (void)parking_fault;
#endif

    reverse_track_run_control(&telemetry,
                              route_x_m,
                              route_y_m,
                              route_heading_rad);
}

void ReverseTrackMode_Exit(void)
{
    g_reverse_track.enabled = 0U;
    Ackermann_control_enable(0U);
    reverse_track_stop_outputs();
    if(g_reverse_track.terminal_latched == 0U)
    {
        g_reverse_track_status.state = REVERSE_TRACK_STATE_IDLE;
        g_reverse_track_status.fault = REVERSE_TRACK_FAULT_NONE;
    }
}

void ReverseTrackMode_GetStatus(ReverseTrackModeStatus_t *status)
{
    if(status == 0) { return; }

    status->state = g_reverse_track_status.state;
    status->fault = g_reverse_track_status.fault;
    status->point_count = g_reverse_track_status.point_count;
    status->segment_index = g_reverse_track_status.segment_index;
    status->route_valid = g_reverse_track_status.route_valid;
    status->aligned = g_reverse_track_status.aligned;
    status->outputs_enabled = g_reverse_track_status.outputs_enabled;
    status->parking_active = g_reverse_track_status.parking_active;
    status->local_x_m = g_reverse_track_status.local_x_m;
    status->local_y_m = g_reverse_track_status.local_y_m;
    status->remaining_distance_m =
        g_reverse_track_status.remaining_distance_m;
    status->lookahead_x_m = g_reverse_track_status.lookahead_x_m;
    status->lookahead_y_m = g_reverse_track_status.lookahead_y_m;
    status->lookahead_distance_m =
        g_reverse_track_status.lookahead_distance_m;
    status->curvature_inv_m = g_reverse_track_status.curvature_inv_m;
    status->target_speed_mps = g_reverse_track_status.target_speed_mps;
    status->ramped_speed_mps = g_reverse_track_status.ramped_speed_mps;
    status->steering_target_rad =
        g_reverse_track_status.steering_target_rad;
    status->steering_encoder_target_deg =
        g_reverse_track_status.steering_encoder_target_deg;
    status->left_target_mps = g_reverse_track_status.left_target_mps;
    status->left_measured_mps =
        g_reverse_track_status.left_measured_mps;
    status->left_pwm = g_reverse_track_status.left_pwm;
    status->right_target_mps = g_reverse_track_status.right_target_mps;
    status->right_measured_mps =
        g_reverse_track_status.right_measured_mps;
    status->right_pwm = g_reverse_track_status.right_pwm;
    status->parking_position_error_m =
        g_reverse_track_status.parking_position_error_m;
    status->parking_heading_error_deg =
        g_reverse_track_status.parking_heading_error_deg;
}
