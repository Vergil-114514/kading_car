#ifndef _Doro_device_oid_encoder_h_
#define _Doro_device_oid_encoder_h_

#include "zf_common_typedef.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"

//==============================================欧艾迪编码器 引脚宏定义==============================================
// 请根据实际硬件连接修改以下宏定义（当前默认使用 P20 组的 SPI0）
#define OID_ENCODER_SPI_N       (SPI_1)                 // 使用的硬件 SPI 模块
#define OID_ENCODER_BAUDRATE    (500000)               // SPI 通信波特率 (最大支持 1MHz 即 1000000)
#define OID_ENCODER_SPI_SCK     (SPI1_SCLK_P10_2)       // 时钟 CLK 引脚 (绿线)
#define OID_ENCODER_SPI_MOSI    (SPI1_MOSI_P10_3)      // MOSI 引脚 (编码器无此线，但初始化库函数需要填入)
#define OID_ENCODER_SPI_MISO    (SPI1_MISO_P10_1)      // 数据输出 DO 引脚 (白线)
#define OID_ENCODER_CS_PIN      (P00_12)                 // 软件控制的 CS 片选引脚 (黄线)
//===================================================================================================================

//==============================================欧艾迪编码器 基础函数================================================
void    oid_encoder_init       (void);
uint16  oid_encoder_get_angle  (void);
//==============================================欧艾迪编码器 基础函数================================================

#endif
