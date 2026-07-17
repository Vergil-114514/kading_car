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
 * Ackermann 控制器直接使用 car_speed[1] 和 car_speed[2]。
 */
#define ACKERMANN_PORT_LEFT_ENCODER_SIGN            (1.0f)
#define ACKERMANN_PORT_RIGHT_ENCODER_SIGN           (1.0f)
#define ACKERMANN_PORT_LEFT_MOTOR_SIGN              (1.0f)
#define ACKERMANN_PORT_RIGHT_MOTOR_SIGN             (1.0f)

/* -1 captures the steering encoder angle at startup as straight ahead. */
#define ACKERMANN_PORT_STEERING_CENTER_DEG           (-1.0f)
#define ACKERMANN_PORT_STEERING_RATIO                (1.0f)
#define ACKERMANN_PORT_STEERING_SIGN                 (1.0f)

uint8_t Ackermann_port_init(void);
void Ackermann_port_start_periodic(void);
void Ackermann_port_5ms_callback(void);

#endif
