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

#include "zf_common_headfile.h"

/*
 * 本文件只实现 3A22 的串口收帧和链路状态，不直接解释摇杆方向或控制电机。
 * UART 回调属于中断上下文，必须保持定长、无阻塞；完整帧由主循环通过
 * lora3a22_get_snapshot() 取走。
 *
 * 全局状态含义：
 * - lora3a22_uart_data：UART 中断逐字节填充的 18 字节缓冲；
 * - finish_flag：存在一帧尚未被主循环消费的数据；
 * - state_flag：底层链路在 300 ms 超时窗口内有效；
 * - response_time：距离最近有效帧的时间；
 * - frame_sequence：校验正确的有效帧累计序号。
 */

uint8   lora3a22_uart_data[LORA3A22_DATA_LEN]  = {0};               // 遥控器接收器原始数据

vuint8  lora3a22_finsh_flag = 0;                                    // 表示成功接收到一帧遥控器数据
vuint8  lora3a22_state_flag = 0;                                    // 遥控器状态(1表示正常，否则表示失控)
vuint16 lora3a22_response_time = LORA3A22_LINK_TIMEOUT_MS;
vuint32 lora3a22_frame_sequence = 0;

/* 最近一帧校验正确的数据。业务层应通过 get_snapshot 读取它。 */
lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;

//--------------------------------  -----------------------------------------------------------------------------------
// 函数简介     lora3a22串口回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     lora3a22_uart_callback();
// 备注信息     此函数需要在串口接收中断内进行调用
//-------------------------------------------------------------------------------------------------------------------

void lora3a22_uart_callback(void )
{
    /* length 跨中断保留，表示下一字节写入位置。 */
    static uint8 length = 0 ;
    uint8  parity_bit_sum  = 0, parity_bit  = 0;
    int i;

    /* 每次回调只读取一个 UART 字节，避免在中断中等待。 */
    lora3a22_uart_data[length++] = uart_read_byte(LORA3A22_UART_INDEX);

    /* 第一个字节不是 0xA3 时立即丢弃，下一字节重新作为候选帧头。 */
    if((1 == length) && (LORA3A22_FRAME_STAR != lora3a22_uart_data[0]))
    {
        length =  0;
    }                                                             // 起始位判断

    if(LORA3A22_DATA_LEN <= length)                            	  // 数据长度判断
    {
        /* 协议校验：除 byte[1] 外其余 17 字节做 uint8 累加。 */
        parity_bit = lora3a22_uart_data[1];
        for(i = 0; i < LORA3A22_DATA_LEN; i ++)
        {
            if(i != 1)
            {
                parity_bit_sum += lora3a22_uart_data[i];
            }
        }

        if (parity_bit_sum == parity_bit)                          // 和校验判断
        {
            // 将接收到的数据拷贝到结构体中
            /* 一次复制完整帧，所有字段更新完后才发布完成标志。 */
            memcpy((uint8*)&lora3a22_uart_transfer, (uint8*)lora3a22_uart_data, \
            sizeof(lora3a22_uart_data));

            // 完整拷贝后再发布新帧标志，主循环不会读到半帧数据。
            lora3a22_response_time = 0U;
            lora3a22_state_flag = 1U;
            ++lora3a22_frame_sequence;
            lora3a22_finsh_flag = 1U;

        }
        else
        {
            lora3a22_finsh_flag = 0;
        }
        /* 无论校验是否成功，定长帧结束后都从头接收下一帧。 */
        parity_bit_sum = 0;
        length = 0;
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     lora3a22初始化函数
// 参数说明     void
// 返回参数     void
// 使用示例     lora3a22_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void lora3a22_init(void)
{
    /* 上电默认失联；第一帧校验成功前不允许上层使用遥控命令。 */
    lora3a22_finsh_flag = 0U;
    lora3a22_state_flag = 0U;
    lora3a22_response_time = LORA3A22_LINK_TIMEOUT_MS;
    lora3a22_frame_sequence = 0U;

    /* 先注册回调并配置 UART2，最后打开接收中断。 */
    set_wireless_type(WIRELESS_UART, lora3a22_uart_callback);
    uart_init(LORA3A22_UART_INDEX, LORA3A22_UART_BAUDRATE, LORA3A22_UART_RX_PIN, LORA3A22_UART_TX_PIN);

    uart_rx_interrupt(LORA3A22_UART_INDEX, 1);
}

void lora3a22_5ms_callback(void)
{
    /* 饱和计时防止 uint16 溢出后错误地重新判定为在线。 */
    if(lora3a22_response_time <= (uint16)(65535U - 5U))
    {
        lora3a22_response_time = (uint16)(lora3a22_response_time + 5U);
    }

    if(lora3a22_response_time > LORA3A22_LINK_TIMEOUT_MS)
    {
        /* 超时同时撤销链路和待消费标志，禁止上层取走过期命令。 */
        lora3a22_state_flag = 0U;
        lora3a22_finsh_flag = 0U;
    }
}

uint8 lora3a22_get_snapshot(lora3a22_uart_transfer_dat_struct *snapshot,
                            uint32 *sequence)
{
    uint32 interrupt_state;
    uint8 result = 0U;

    if(snapshot == NULL)
    {
        return 0U;
    }

    /*
     * 复制期间短暂屏蔽中断，保证数据、序号和完成标志来自同一帧。
     * 保存并恢复原中断状态，避免无条件开启调用者原本关闭的中断。
     */
    interrupt_state = interrupt_global_disable();
    if((lora3a22_finsh_flag != 0U) && (lora3a22_state_flag != 0U))
    {
        memcpy((uint8 *)snapshot,
               (const uint8 *)&lora3a22_uart_transfer,
               sizeof(lora3a22_uart_transfer));
        if(sequence != NULL)
        {
            *sequence = lora3a22_frame_sequence;
        }
        /* 一帧只消费一次；下一帧校验成功后中断会再次置位。 */
        lora3a22_finsh_flag = 0U;
        result = 1U;
    }
    interrupt_global_enable(interrupt_state);
    return result;
}
