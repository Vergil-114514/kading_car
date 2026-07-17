/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_common_headfile
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/

#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

//===================================================C语言 函数库===================================================
#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "float.h"
#include "limits.h"
#include "stddef.h"
//===================================================C语言 函数库===================================================

//===================================================芯片 SDK 底层===================================================
#include "ifxAsclin_reg.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"
//===================================================芯片 SDK 底层===================================================

//====================================================开源库公共层====================================================
#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_common_interrupt.h"
#include "isr_config.h"
//====================================================开源库公共层====================================================

//===================================================芯片外设驱动层===================================================
#include "zf_driver_adc.h"
#include "zf_driver_delay.h"
#include "zf_driver_dma.h"
#include "zf_driver_encoder.h"
#include "zf_driver_exti.h"
#include "zf_driver_flash.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"
#include "zf_driver_pwm.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_spi.h"
#include "zf_driver_soft_spi.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
//===================================================芯片外设驱动层===================================================

//===================================================外接设备驱动层===================================================
#include "zf_device_absolute_encoder.h"
#include "zf_device_ble6a20.h"
#include "zf_device_bluetooth_ch9141.h"
#include "zf_device_gnss.h"
#include "zf_device_camera.h"
#include "zf_device_dl1a.h"
#include "zf_device_dl1b.h"
#include "zf_device_icm20602.h"
#include "zf_device_imu660ra.h"
#include "zf_device_imu660rb.h"
#include "zf_device_imu660rc.h"
#include "zf_device_imu660rx.h"
#include "zf_device_imu963ra.h"
#include "zf_device_ips114.h"
#include "zf_device_ips200.h"
#include "zf_device_ips200pro.h"
#include "zf_device_key.h"
#include "zf_device_menc15a.h"
#include "zf_device_mpu6050.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_device_oled.h"
#include "zf_device_ov7725.h"
#include "zf_device_scc8660.h"
#include "zf_device_tft180.h"
#include "zf_device_tsl1401.h"
#include "zf_device_type.h"
#include "zf_device_uart_receiver.h"
#include "zf_device_virtual_oscilloscope.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_wireless_uart.h"
//===================================================外接设备驱动层===================================================

//====================================================应用组件层=====================================================
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
//====================================================应用组件层=====================================================

//=====================================================用户层=======================================================
// 本区域是整车应用代码的统一头文件入口。
// 应用层 .c 文件只需包含 zf_common_headfile.h，不再分别维护模块头文件列表。

// 自定义底层设备
#include "Doro_device_bk450.h"
#include "Doro_device_oid_encoder.h"
#include "Doro_driver_pwm_comp.h"

// 姿态、惯导与定位
#include "FusionConvention.h"
#include "FusionMath.h"
#include "FusionModel.h"
#include "FusionRemap.h"
#include "FusionBias.h"
#include "FusionCompass.h"
#include "FusionAhrs.h"
#include "Fusion.h"
#include "IMU.h"
#include "imu_N.h"
#include "MahonyFilter.h"
#include "TopSpeed_GPS_INS.h"
#include "TopSpeed_GPS_INS_Port.h"

// LoRa 遥控
#include "zf_device_lora3a22.h"
#include "lora_remote.h"

// TLD7002 与点阵屏
#include "TLD7002.h"
#include "TLD7002_Definitions.h"
#include "TLD7002_ControlLayer.h"
#include "TLD7002_ServiceLayer.h"
#include "TLD7002FuncLayer.h"
#include "zf_device_tld7002.h"
#include "zf_device_dot_matrix_screen.h"

// 整车执行器、传感器与控制
#include "encoder.h"
#include "encoder.h"
#include "motor.h"
#include "ackermann_control.h"
#include "ackermann_port.h"
#include "flash.h"

// 菜单与工作模式
#include "board_mode_switch.h"
#include "board_mode_switch.h"
#include "remote_mode.h"
#include "test_mode.h"
#include "Menu.h"

// 四核外设初始化与中断共享声明
#include "All_init.h"
#include "isr.h"
#include "cpu0_main.h"
//=====================================================用户层=======================================================

#endif
