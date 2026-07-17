#ifndef _ALL_INIT_H_
#define _ALL_INIT_H_

#include "GPS_INS/TopSpeed_GPS_INS_Port.h"

/*
 * TC387 使用 CPU0、CPU1、CPU2、CPU3 四个核心，因此本文件提供四个入口。
 * 每个函数只完成本核外设初始化；四核启动、看门狗/全局中断设置和
 * cpu_wait_event_ready() 同步仍由各自的 main 函数负责。
 */
void CPU0_Peripheral_Init(uint8_t use_gps, uint8_t navigation_solution);
void CPU1_Peripheral_Init(void);
void CPU2_Peripheral_Init(void);
void CPU3_Peripheral_Init(void);

#endif
