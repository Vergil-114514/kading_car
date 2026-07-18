#include "zf_common_headfile.h"
#include "subject2_voice.h"

/**
 * @brief CPU0 外设初始化。
 *
 * CPU0 当前负责调试串口、惯导、三个电机及编码器、LoRa 和菜单。
 * 系统时钟、四核启动/同步以及同步后的 PIT 启动仍保留在 cpu0_main.c。
 */
void CPU0_Peripheral_Init(uint8_t use_gps, uint8_t navigation_solution)
{
    uint8_t navigation_init_state;
    uint8_t motor_init_state;
    MenuPeripheralStatus_t peripheral_status = {0};

    /* clock_init() 已在 CPU0 主函数中执行。 */
    peripheral_status.clock_ready = 1U;


    peripheral_status.debug_uart_ready = 1U;

    navigation_init_state = TopSpeed_GPS_INS_PortInit(use_gps,navigation_solution);
    peripheral_status.imu_ready =(navigation_init_state == 0U) ? 1U : 0U;
    peripheral_status.gps_enabled = use_gps;
    peripheral_status.gps_initialized = use_gps;

    if(navigation_init_state != 0U)
    {
        printf("TopSpeed INS: IMU963RA init failed; heading invalid.\r\n");
    }
    else
    {
        printf("Navigation: %s ready, GPS %s.\r\n",
               TopSpeed_GPS_INS_PortGetNavigationSolutionName(),
               (use_gps != 0U) ? "enabled" : "disabled");
    }

    motor_init_state = Ackermann_port_init();
    peripheral_status.motor_encoder_ready =(motor_init_state == 0U) ? 1U : 0U;
    if(motor_init_state != 0U)
    {
        printf("Ackermann motor/encoder init failed.\r\n");
    }

    Menu_SetPeripheralInitStatus(&peripheral_status);
    Menu_init();
}

/**
 * @brief CPU1 外设初始化。
 * @note  当前预留给 WiFi/SPI 通信和组合导航。
 */
void CPU1_Peripheral_Init(void)
{
    /* TODO：在这里加入 CPU1 的 WiFi/SPI、GPS 等外设初始化。 */
}

/**
 * @brief CPU2 外设初始化。
 * @note  当前预留给 MPC/车辆控制计算。
 */
void CPU2_Peripheral_Init(void)
{
    /* TODO：在这里加入 CPU2 的控制算法相关外设初始化。 */
}

/**
 * @brief CPU3 外设初始化。
 * @note  当前预留给语音识别和灯板。
 */
void CPU3_Peripheral_Init(void)
{
    Subject2_voice_init();
}
