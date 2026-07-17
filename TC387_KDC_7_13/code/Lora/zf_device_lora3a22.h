/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          zf_device_lora3a22
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-03-29       JKS            first version
********************************************************************************************************************/


#ifndef CODE_ZF_DEVICE_LORA3A22_H_
#define CODE_ZF_DEVICE_LORA3A22_H_

#include "zf_common_headfile.h"

/**
 * @file zf_device_lora3a22.h
 * @brief 逐飞 3A22 LoRa 遥控接收机的 TC387 串口驱动接口。
 *
 * 接收机通过 UART2 发送固定 18 字节二进制帧。驱动只负责找帧头、定长收包、
 * 和校验及超时检测；摇杆死区、方向和电机安全逻辑由 lora_remote 模块处理。
 *
 * 接线采用交叉连接：接收机 RX 接主控 TX(P10.5)，接收机 TX 接主控 RX(P10.6)。
 */

/* UART 硬件配置。宏名称沿用原驱动，RX_PIN 实际传入主控 TX，TX_PIN 反之。 */

#define LORA3A22_UART_INDEX            (UART_10)              // 定义串口遥控器使用的串口
#define LORA3A22_UART_RX_PIN           (UART10_TX_P13_0)      // 遥控器接收机的RX引脚 连接单片机的TX引脚
#define LORA3A22_UART_TX_PIN           (UART10_RX_P13_1)      // 遥控器接收机的TX引脚 连接单片机的RX引脚
#define LORA3A22_UART_BAUDRATE         (115200)              // 指定 lora3a22 串口所使用的的串口波特率

#define LORA3A22_DATA_LEN              ( 18  )               // lora3a22帧长
#define LORA3A22_FRAME_STAR            ( 0XA3 )              // 帧头信息
#define LORA3A22_LINK_TIMEOUT_MS        ( 300U )              // 超时后立即判定遥控失联



/**
 * 18 字节线协议（按 TC387 小端内存布局直接复制）：
 * byte 0      : 0xA3 帧头
 * byte 1      : 除 byte 1 自身外其余 17 字节的 uint8 累加和
 * byte 2..9   : joystick[0..3]，四个 int16
 * byte 10..13 : key[0..3]，1=按下
 * byte 14..17 : switch_key[0..3]，1=拨码开启
 */
typedef struct
{
    uint8 head;                         // 帧头
    uint8 sum_check;                    // 和校验

    int16 joystick[4];
	// joystick[0]:左边摇杆左右值      
	// joystick[1]:左边摇杆上下值
	// joystick[2]:右边摇杆左右值      
	// joystick[3]:右边摇杆上下值

    uint8 key[4];
	// 按下1 松开0
    // key[0]-摇杆左边
    // key[1]-摇杆右边
    // key[2]-侧向按键左边
    // key[3]-侧向按键右边

    uint8 switch_key[4];
    // switch_key[0]-左边拨码开关_1
    // switch_key[1]-左边拨码开关_2
    // switch_key[2]-右边拨码开关_1
    // switch_key[3]-右边拨码开关_2

}lora3a22_uart_transfer_dat_struct;


/*
 * 以下全局量保留用于兼容原逐飞接口。业务控制代码应优先调用
 * lora3a22_get_snapshot()，不要直接读取正在被串口中断更新的结构体。
 */
extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;
extern uint8   lora3a22_uart_data[LORA3A22_DATA_LEN];       // lora3a22接收原始数据
extern vuint8  lora3a22_finsh_flag;
extern vuint8  lora3a22_state_flag;                         // 遥控器状态(1表示正常，否则表示失控)
extern vuint16 lora3a22_response_time;
extern vuint32 lora3a22_frame_sequence;

/** 初始化 UART2、注册接收回调并打开接收中断。 */
void lora3a22_init(void);
/** UART 每收到一个字节时调用，完成定长组帧和校验。 */
void lora3a22_uart_callback(void);
/** 每 5 ms 调用，累计无响应时间并更新底层链路状态。 */
void lora3a22_5ms_callback(void);
/**
 * 在短临界区内复制一帧完整数据并消费完成标志。
 * @return 1=成功取得一帧，0=无新帧或 snapshot 为空。
 */
uint8 lora3a22_get_snapshot(lora3a22_uart_transfer_dat_struct *snapshot,
                            uint32 *sequence);

#endif
