/*
 * Minimal TC387 port of the GIE path that is actually used by TopSpeed.
 * The unused/broken experimental position KF, complementary filter and CTRV
 * branches from the TC377 project are intentionally not copied here.
 */

#include "zf_common_headfile.h"

#define TOPSPEED_GPS_INS_PI              (3.14159265358979323846f)
#define TOPSPEED_GPS_INS_AGE_NOT_SEEN_S  (1000000.0f)
#define TOPSPEED_GPS_INS_MIN_DT_S        (0.000001f)
#define TOPSPEED_GPS_INS_MAX_DT_S        (1.0f)

typedef struct
{
    TopSpeed_GPS_INS_Config config;
    TopSpeed_GPS_INS_Output output;
    uint8_t gps_has_fix;
    uint8_t encoder_has_sample;
} TopSpeed_GPS_INS_Context;

static TopSpeed_GPS_INS_Context g_topSpeedGpsIns;

static uint8_t TopSpeed_GPS_INS_IsFinite(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

static float TopSpeed_GPS_INS_NormalizeHeading(float heading_deg)
{
    while (heading_deg >= 180.0f)
    {
        heading_deg -= 360.0f;
    }
    while (heading_deg < -180.0f)
    {
        heading_deg += 360.0f;
    }
    return heading_deg;
}

static float TopSpeed_GPS_INS_Distance(TopSpeed_GPS_INS_Point a,
                                      TopSpeed_GPS_INS_Point b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}

static uint8_t TopSpeed_GPS_INS_TimeoutValid(float age_s, float timeout_s)
{
    if (timeout_s <= 0.0f)
    {
        return 1U;
    }
    return (uint8_t)(age_s <= timeout_s);
}

static uint8_t TopSpeed_GPS_INS_SpeedValueValid(float speed_mps)
{
    if (!TopSpeed_GPS_INS_IsFinite(speed_mps))
    {
        return 0U;
    }
    if ((g_topSpeedGpsIns.config.max_abs_speed_mps > 0.0f) &&
        (fabsf(speed_mps) > g_topSpeedGpsIns.config.max_abs_speed_mps))
    {
        return 0U;
    }
    return 1U;
}

static void TopSpeed_GPS_INS_RefreshValidityAndSpeed(void)
{
    TopSpeed_GPS_INS_Output *output = &g_topSpeedGpsIns.output;

    output->gps_valid = (uint8_t)(g_topSpeedGpsIns.gps_has_fix &&
        TopSpeed_GPS_INS_TimeoutValid(output->gps_age_s,
                                     g_topSpeedGpsIns.config.gps_timeout_s));

    output->encoder_valid = (uint8_t)(g_topSpeedGpsIns.encoder_has_sample &&
        TopSpeed_GPS_INS_TimeoutValid(output->encoder_age_s,
                                     g_topSpeedGpsIns.config.encoder_timeout_s));

    if (output->encoder_valid)
    {
        output->speed_mps = output->encoder_speed_mps;
        output->speed_source = TOPSPEED_GPS_INS_SPEED_ENCODER;
    }
    else if (output->gps_valid)
    {
        output->speed_mps = output->gps_speed_mps;
        output->speed_source = TOPSPEED_GPS_INS_SPEED_GPS;
    }
    else
    {
        output->speed_mps = 0.0f;
        output->speed_source = TOPSPEED_GPS_INS_SPEED_NONE;
    }

    output->valid = (uint8_t)(output->initialized && output->heading_valid &&
                              (output->gps_valid || output->encoder_valid));
}

static void TopSpeed_GPS_INS_Advance(float dt_s)
{
    TopSpeed_GPS_INS_Output *output = &g_topSpeedGpsIns.output;
    float heading_rad;
    float distance_m;

    if (!TopSpeed_GPS_INS_IsFinite(dt_s) ||
        (dt_s < TOPSPEED_GPS_INS_MIN_DT_S) ||
        (dt_s > TOPSPEED_GPS_INS_MAX_DT_S))
    {
        return;
    }

    if (output->gps_age_s < TOPSPEED_GPS_INS_AGE_NOT_SEEN_S)
    {
        output->gps_age_s += dt_s;
    }
    if (output->encoder_age_s < TOPSPEED_GPS_INS_AGE_NOT_SEEN_S)
    {
        output->encoder_age_s += dt_s;
    }

    TopSpeed_GPS_INS_RefreshValidityAndSpeed();

    if (!output->valid ||
        (output->speed_source == TOPSPEED_GPS_INS_SPEED_NONE))
    {
        return;
    }

    heading_rad = output->heading_deg * (TOPSPEED_GPS_INS_PI / 180.0f);
    distance_m = output->speed_mps * dt_s;

    /* 0 deg north, clockwise positive: east=sin(h), north=cos(h). */
    output->position_m.x += sinf(heading_rad) * distance_m;
    output->position_m.y += cosf(heading_rad) * distance_m;
}

void TopSpeed_GPS_INS_GetDefaultConfig(TopSpeed_GPS_INS_Config *config)
{
    if (config == 0)
    {
        return;
    }

    config->gps_delay_s = 0.30f;
    config->gps_timeout_s = 0.50f;
    config->encoder_timeout_s = 0.20f;
    config->gps_jump_gate_m = 0.0f;
    config->max_abs_speed_mps = 30.0f;
}

void TopSpeed_GPS_INS_Init(const TopSpeed_GPS_INS_Config *config)
{
    TopSpeed_GPS_INS_Config selected;

    TopSpeed_GPS_INS_GetDefaultConfig(&selected);
    if (config != 0)
    {
        selected = *config;
    }

    memset(&g_topSpeedGpsIns, 0, sizeof(g_topSpeedGpsIns));
    g_topSpeedGpsIns.config = selected;
    g_topSpeedGpsIns.output.gps_age_s = TOPSPEED_GPS_INS_AGE_NOT_SEEN_S;
    g_topSpeedGpsIns.output.encoder_age_s = TOPSPEED_GPS_INS_AGE_NOT_SEEN_S;
    g_topSpeedGpsIns.output.speed_source = TOPSPEED_GPS_INS_SPEED_NONE;
}

void TopSpeed_GPS_INS_Reset(void)
{
    TopSpeed_GPS_INS_Config config = g_topSpeedGpsIns.config;
    TopSpeed_GPS_INS_Init(&config);
}

void TopSpeed_GPS_INS_StartDeadReckoning(
    TopSpeed_GPS_INS_Point initial_position_m)
{
    TopSpeed_GPS_INS_Output *output = &g_topSpeedGpsIns.output;

    if (!TopSpeed_GPS_INS_IsFinite(initial_position_m.x) ||
        !TopSpeed_GPS_INS_IsFinite(initial_position_m.y))
    {
        return;
    }

    /*
     * GPS-free positioning has no geodetic reference, so (x, y) is a local
     * relative coordinate.  Mark only the local position initialized; do not
     * manufacture a GPS fix or GPS speed sample.
     */
    output->position_m = initial_position_m;
    output->initialized = 1U;
    TopSpeed_GPS_INS_RefreshValidityAndSpeed();
}

void TopSpeed_GPS_INS_Tick(float dt_s)
{
    TopSpeed_GPS_INS_Advance(dt_s);
}

void TopSpeed_GPS_INS_UpdateIMU(float heading_deg,
                               float forward_acc_mps2,
                               float dt_s)
{
    if (TopSpeed_GPS_INS_IsFinite(heading_deg))
    {
        g_topSpeedGpsIns.output.heading_deg =
            TopSpeed_GPS_INS_NormalizeHeading(heading_deg);
        g_topSpeedGpsIns.output.heading_valid = 1U;
    }

    if (TopSpeed_GPS_INS_IsFinite(forward_acc_mps2))
    {
        g_topSpeedGpsIns.output.forward_acc_mps2 = forward_acc_mps2;
    }

    g_topSpeedGpsIns.output.imu_update_count++;
    TopSpeed_GPS_INS_RefreshValidityAndSpeed();
    TopSpeed_GPS_INS_Advance(dt_s);
}

void TopSpeed_GPS_INS_UpdateGPS(TopSpeed_GPS_INS_Point position_m,
                               float speed_mps,
                               uint8_t valid)
{
    TopSpeed_GPS_INS_Output *output = &g_topSpeedGpsIns.output;
    TopSpeed_GPS_INS_Point compensated;
    float delay_speed_mps;
    float heading_rad;

    if (!valid ||
        !TopSpeed_GPS_INS_IsFinite(position_m.x) ||
        !TopSpeed_GPS_INS_IsFinite(position_m.y) ||
        !TopSpeed_GPS_INS_SpeedValueValid(speed_mps))
    {
        g_topSpeedGpsIns.gps_has_fix = 0U;
        output->gps_valid = 0U;
        TopSpeed_GPS_INS_RefreshValidityAndSpeed();
        return;
    }

    output->gps_position_m = position_m;
    output->gps_speed_mps = speed_mps;
    output->gps_age_s = 0.0f;
    g_topSpeedGpsIns.gps_has_fix = 1U;
    TopSpeed_GPS_INS_RefreshValidityAndSpeed();

    compensated = position_m;
    if (output->heading_valid && (g_topSpeedGpsIns.config.gps_delay_s > 0.0f))
    {
        /* Signed encoder speed makes delay compensation correct in reverse. */
        delay_speed_mps = output->encoder_valid ?
            output->encoder_speed_mps : output->gps_speed_mps;
        heading_rad = output->heading_deg * (TOPSPEED_GPS_INS_PI / 180.0f);
        compensated.x += sinf(heading_rad) * delay_speed_mps *
                         g_topSpeedGpsIns.config.gps_delay_s;
        compensated.y += cosf(heading_rad) * delay_speed_mps *
                         g_topSpeedGpsIns.config.gps_delay_s;
    }
    output->compensated_gps_position_m = compensated;

    if (output->initialized &&
        (g_topSpeedGpsIns.config.gps_jump_gate_m > 0.0f) &&
        (TopSpeed_GPS_INS_Distance(output->position_m, compensated) >
         g_topSpeedGpsIns.config.gps_jump_gate_m))
    {
        output->gps_reject_count++;
        return;
    }

    output->position_m = compensated;
    output->initialized = 1U;
    output->gps_update_count++;
    TopSpeed_GPS_INS_RefreshValidityAndSpeed();
}

void TopSpeed_GPS_INS_UpdateEncoderSpeed(float speed_mps, uint8_t valid)
{
    TopSpeed_GPS_INS_Output *output = &g_topSpeedGpsIns.output;

    if (!valid || !TopSpeed_GPS_INS_SpeedValueValid(speed_mps))
    {
        g_topSpeedGpsIns.encoder_has_sample = 0U;
        output->encoder_valid = 0U;
        TopSpeed_GPS_INS_RefreshValidityAndSpeed();
        return;
    }

    output->encoder_speed_mps = speed_mps;
    output->encoder_age_s = 0.0f;
    output->encoder_update_count++;
    g_topSpeedGpsIns.encoder_has_sample = 1U;
    TopSpeed_GPS_INS_RefreshValidityAndSpeed();
}

TopSpeed_GPS_INS_Output TopSpeed_GPS_INS_GetOutput(void)
{
    TopSpeed_GPS_INS_RefreshValidityAndSpeed();
    return g_topSpeedGpsIns.output;
}
