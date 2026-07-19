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
* 文件名称          cpu0_main
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
#include "zf_common_headfile.h"
//         主函数            定时器中断                    串口中断
//0核      按钮，屏幕        任务调度，蜂鸣器
//1核                    Mahony航向与速度积分XY              GPS
//2核                 遥控、编码器、阿克曼、PID及最终PWM       LORA
//3核   科目二语言识别+灯板

//CCU60_CH0   CPU0   timecnt
//CCU60_CH1   CPU1   Mahony/定位/存点XY积分
//CCU61_CH0   CPU2   统一5ms控制链
//CCU61_CH1   CPU2   保留，当前不启动


/*
 * CPU1 GPS 使用开关：
 * 0 = 不初始化 GPS，只使用 IMU 和编码器；
 * 1 = 初始化并融合 GPS。
 */
#define CPU1_USE_GPS                     (0U)

/* CPU1 姿态解算固定选择 Mahony，供存点局部 XY 积分直接使用。 */
#define CPU1_NAVIGATION_SOLUTION         (NAVIGATION_SOLUTION_MAHONY)

#if (CPU1_NAVIGATION_SOLUTION != NAVIGATION_SOLUTION_MAHONY)
    #error "SAVE POINT local odometry requires CPU1 Mahony yaw"
#endif


#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

//    CAR_MODE_TRACK = 0,       /**< 00：按编译期方向跟踪闪存路线。 */
//    CAR_MODE_SAVE_POINT = 1,  /**< 01：遥控行驶并按 0.5 m 记录点位。 */
//    CAR_MODE_TEST = 2,        /**< 10：电机/PID 测试模式。 */
//    CAR_MODE_REMOTE = 3,      /**< 11：LoRa 遥控运动模式。 */


// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    system_start();                 // 启动 CPU1、CPU2、CPU3
    debug_init();
    /* All_init 只负责 CPU0 外设初始化。 */
    CPU0_Peripheral_Init(CPU1_USE_GPS, CPU1_NAVIGATION_SOLUTION);
    cpu_wait_event_ready();         // 等待所有核心初始化完毕

    /*
     * 四核同步后统一启动 PIT：菜单不在中断中运行，ISR 只负责系统节拍、
     * CPU1 定位/存点积分和 CPU2 统一实时控制。
     */
    pit_ms_init(CCU60_CH0, 1U);                          // CPU0：timecnt
    pit_ms_init(CCU60_CH1, TOPSPEED_GPS_INS_PERIOD_MS);  // CPU1：惯导定位
    pit_ms_init(CCU61_CH0, ACKERMANN_CONTROL_PERIOD_MS); // CPU2：遥控/Ackermann/PID


    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        /* 仅处理模式进入及 WiFi/闪存/打印等非实时任务。 */
        Menu_Task();

        /* 菜单完全位于 CPU0 主循环，每 100 ms 刷新一次。 */
        if(timecnt[1] == 0)
        {
            timecnt[1] = 100;
            Menu_Display();
            Menu_100msTask();
            GetCount();
        }
        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
// **************************** 代码区域 ****************************
