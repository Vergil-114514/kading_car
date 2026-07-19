#ifndef _ACKERMANN_PORT_H_
#define _ACKERMANN_PORT_H_

#include "ackermann_control.h"
#include "encoder.h"

/*
 * Motor and rear encoder pins are configured from the supplied schematic.
 * The controller still starts disabled and cannot drive until the application
 * explicitly calls Ackermann_control_enable(1).
 */
#define ACKERMANN_PORT_ENABLE                       (1U)

/*
 * 编码器的初始化和速度计算放在用户的 encoder.c/.h 中。
 * car_speed[1] 和 car_speed[2] 以 m/s 直接传入 Ackermann 和后轮 PID。
 */
#define ACKERMANN_PORT_LEFT_ENCODER_SIGN            (1.0f)
#define ACKERMANN_PORT_RIGHT_ENCODER_SIGN           (1.0f)
#define ACKERMANN_PORT_LEFT_MOTOR_SIGN              (1.0f)
#define ACKERMANN_PORT_RIGHT_MOTOR_SIGN             (1.0f)

/* Fixed calibration: raw 876 is straight; increasing angle is left. */
#define ACKERMANN_PORT_STEERING_CENTER_DEG           \
    (ACKERMANN_STEERING_CENTER_ENCODER_DEG)
#define ACKERMANN_PORT_STEERING_RATIO                \
    (ACKERMANN_STEERING_ENCODER_RATIO)
#define ACKERMANN_PORT_STEERING_SIGN                 \
    (ACKERMANN_STEERING_SIGN)

uint8_t Ackermann_port_init(void);
void Ackermann_port_start_periodic(void);
void Ackermann_port_5ms_callback(void);

#endif
