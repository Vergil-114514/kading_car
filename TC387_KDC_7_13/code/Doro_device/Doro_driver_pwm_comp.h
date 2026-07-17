/*********************************************************************************************************************
* 文件名称          Doro_driver_pwm_hl.h
* 功能说明          基于 TC377 iLLD 的互补 PWM 驱动封装 (带死区)
* 依赖文件          zf_driver_pwm.h
* 修改记录
* 日期              作者                备注
* 2026-04-04       Doro & Gemini      first version 互补PWM驱动
********************************************************************************************************************/

#ifndef _DORO_DRIVER_PWM_COMP_H_
#define _DORO_DRIVER_PWM_COMP_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"       // 借用逐飞库的引脚枚举 pwm_channel_enum 和 PWM_DUTY_MAX 宏

//====================================================互补 PWM 基础函数====================================================

/**
 * @brief  初始化互补 PWM（带死区）
 * @param  ccx_pin      主通道引脚 (Top)
 * @param  coutx_pin    互补通道引脚 (Bottom)
 * @param  freq         PWM频率 (Hz)
 * @param  duty         初始占空比 (0 ~ PWM_DUTY_MAX，通常为 10000)
 * @note   同一 ATOM 模块 (如 ATOM0) 的不同通道，共享同一个底层 Timer (占用通道0)，因此同 ATOM 的频率必须一致！
 * 且初始化时不能将 ATOMx_CH0 作为 PWM 输出引脚使用。
 * ATOM0_CH0_P21_2,ATOM0_CH1_P21_3
 */
void pwm_hl_init(pwm_channel_enum ccx_pin, pwm_channel_enum coutx_pin, uint32 freq, uint32 duty);

/**
 * @brief  设置互补 PWM 的占空比
 * @param  ccx_pin      主通道引脚 (Top)，必须与初始化时一致
 * @param  duty         占空比 (0 ~ PWM_DUTY_MAX)
 */
void pwm_hl_set_duty(pwm_channel_enum ccx_pin, uint32 duty);

//========================================================================================================================

#endif
