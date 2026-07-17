#include "zf_common_headfile.h"

static uint8_t g_ackermann_port_ready = ZF_FALSE;

uint8_t Ackermann_port_init(void)
{
#if ACKERMANN_PORT_ENABLE
    ACKERMANN_CONTROL_CONFIG config;

    /* 调用用户 encoder.c 中原有的三个编码器初始化函数。 */
    QdInit();

    Ackermann_get_default_config(&config);
    config.left_encoder_sign = ACKERMANN_PORT_LEFT_ENCODER_SIGN;
    config.right_encoder_sign = ACKERMANN_PORT_RIGHT_ENCODER_SIGN;
    config.left_motor_sign = ACKERMANN_PORT_LEFT_MOTOR_SIGN;
    config.right_motor_sign = ACKERMANN_PORT_RIGHT_MOTOR_SIGN;
    config.steering_center_encoder_deg = ACKERMANN_PORT_STEERING_CENTER_DEG;
    config.steering_encoder_deg_per_road_deg = ACKERMANN_PORT_STEERING_RATIO;
    config.steering_sign = ACKERMANN_PORT_STEERING_SIGN;

    if(Ackermann_control_init(&config) != 0U)
    {
        return 1U;
    }
    g_ackermann_port_ready = ZF_TRUE;
#endif
    return 0U;
}

void Ackermann_port_start_periodic(void)
{
#if ACKERMANN_PORT_ENABLE
    if(g_ackermann_port_ready != ZF_FALSE)
    {
        /* CCU61_CH1 专用于 CPU2 Ackermann 解算，不占用 CPU0 系统节拍。 */
        pit_ms_init(CCU61_CH1, ACKERMANN_CONTROL_PERIOD_MS);
    }
#endif
}

void Ackermann_port_5ms_callback(void)
{
#if ACKERMANN_PORT_ENABLE
    ACKERMANN_CONTROL_TELEMETRY telemetry;

    if(g_ackermann_port_ready == ZF_FALSE)
    {
        return;
    }

    /* encoder.c 负责读取三路编码器，PID 直接使用 car_speed[]。 */
    GetSpeed();
    Ackermann_control_step(car_speed[1], car_speed[2], ZF_TRUE);
    Ackermann_get_telemetry(&telemetry);
    if(telemetry.encoder_valid != ZF_FALSE)
    {
        /* Feed signed vehicle speed to the IMU/encoder dead-reckoning path. */
        TopSpeed_GPS_INS_PortEncoderSpeedUpdate(
            telemetry.measured_vehicle_speed_mps);
    }
    else
    {
        TopSpeed_GPS_INS_PortEncoderInvalidate();
    }
#endif
}
