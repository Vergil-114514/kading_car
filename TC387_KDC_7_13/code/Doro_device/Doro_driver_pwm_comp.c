/*********************************************************************************************************************
* 文件名称          Doro_driver_pwm_hl.c
* 功能说明          基于 TC377 iLLD 的互补 PWM 驱动封装 (带死区)
* 依赖文件          zf_driver_pwm.h
* 修改记录
* 日期              作者                备注
* 2026-04-04       Doro & Gemini      first version 互补PWM驱动
********************************************************************************************************************/
#include <Doro_device/Doro_driver_pwm_comp.h>
#include "IfxGtm_Atom_PwmHl.h"
#include "IfxGtm_Atom_Timer.h"
#include "ifxGtm_PinMap.h"
#include "zf_common_debug.h"

// CMU时钟频率 (与 zf_driver_pwm.c 保持一致)
#define CMU_CLK_FREQ           20000000.0f

// 死区时间设置，单位秒 (50ns = 50.0e-9f)
//只能是0或者50.0e-9f的倍数
#define PWM_HL_DEADTIME_SEC    50.0e-9f

// 全局句柄存储，用于在 set_duty 时访问
// ATOM 模块最多 4 个 (TC377 中 ATOM0 ~ ATOM3)
static IfxGtm_Atom_Timer g_timer_hl[4];
static boolean           g_timer_hl_init_flag[4] = {FALSE, FALSE, FALSE, FALSE};

// PwmHl 驱动句柄：每个 ATOM 有 8 个通道，使用主通道(ccx_ch)作为索引
static IfxGtm_Atom_PwmHl g_pwm_hl[4][8];

//-------------------------------------------------------------------------------------------------------------------
// 内部引脚查找函数（与 zf_driver_pwm.c 隔离，防止 static 报错）
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 内部引脚查找函数（完全独立，解决 static 导致的 unresolved 报错）
//-------------------------------------------------------------------------------------------------------------------
static IfxGtm_Atom_ToutMap* get_pwm_hl_pin(pwm_channel_enum atom_pin)
{
    IfxGtm_Atom_ToutMap* pwm_pwm_pin_config;

    switch(atom_pin)
    {
        case ATOM0_CH0_P00_0: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT9_P00_0_OUT;     break;
        case ATOM0_CH0_P02_0: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT0_P02_0_OUT;     break;
        case ATOM0_CH0_P02_8: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT8_P02_8_OUT;     break;
        case ATOM0_CH0_P14_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT85_P14_5_OUT;    break;
        case ATOM0_CH0_P21_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT53_P21_2_OUT;    break;
        case ATOM0_CH0_P22_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT48_P22_1_OUT;    break;

        case ATOM0_CH1_P00_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT10_P00_1_OUT;    break;
        case ATOM0_CH1_P00_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT11_P00_2_OUT;    break;
        case ATOM0_CH1_P02_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT1_P02_1_OUT;     break;
        case ATOM0_CH1_P14_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT84_P14_4_OUT;    break;
        case ATOM0_CH1_P21_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT54_P21_3_OUT;    break;
        case ATOM0_CH1_P22_0: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT47_P22_0_OUT;    break;
        case ATOM0_CH1_P33_9: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT31_P33_9_OUT;    break;

        case ATOM0_CH2_P00_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT12_P00_3_OUT;    break;
        case ATOM0_CH2_P02_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT2_P02_2_OUT;     break;
        case ATOM0_CH2_P14_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT83_P14_3_OUT;    break;
        case ATOM0_CH2_P21_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT55_P21_4_OUT;    break;
        case ATOM0_CH2_P33_11:pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT33_P33_11_OUT;   break;

        case ATOM0_CH3_P00_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT13_P00_4_OUT;    break;
        case ATOM0_CH3_P02_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT3_P02_3_OUT;     break;
        case ATOM0_CH3_P14_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT82_P14_2_OUT;    break;
        case ATOM0_CH3_P21_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT56_P21_5_OUT;    break;
        case ATOM0_CH3_P22_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT49_P22_2_OUT;    break;

        case ATOM0_CH4_P00_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT14_P00_5_OUT;    break;
        case ATOM0_CH4_P02_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT4_P02_4_OUT;     break;
        case ATOM0_CH4_P14_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT81_P14_1_OUT;    break;
        case ATOM0_CH4_P20_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT61_P20_3_OUT;    break;
        case ATOM0_CH4_P21_6: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT57_P21_6_OUT;    break;
        case ATOM0_CH4_P22_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT50_P22_3_OUT;    break;

        case ATOM0_CH5_P00_6: pwm_pwm_pin_config = &IfxGtm_ATOM0_5_TOUT15_P00_6_OUT;    break;
        case ATOM0_CH5_P02_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_5_TOUT5_P02_5_OUT;     break;
        case ATOM0_CH5_P21_7: pwm_pwm_pin_config = &IfxGtm_ATOM0_5_TOUT58_P21_7_OUT;    break;
        case ATOM0_CH5_P32_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_5_TOUT40_P32_4_OUT;    break;

        case ATOM0_CH6_P00_7: pwm_pwm_pin_config = &IfxGtm_ATOM0_6_TOUT16_P00_7_OUT;    break;
        case ATOM0_CH6_P02_6: pwm_pwm_pin_config = &IfxGtm_ATOM0_6_TOUT6_P02_6_OUT;     break;
        case ATOM0_CH6_P20_0: pwm_pwm_pin_config = &IfxGtm_ATOM0_6_TOUT59_P20_0_OUT;    break;
        case ATOM0_CH6_P23_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_6_TOUT42_P23_1_OUT;    break;

        case ATOM0_CH7_P00_8: pwm_pwm_pin_config = &IfxGtm_ATOM0_7_TOUT17_P00_8_OUT;    break;
        case ATOM0_CH7_P02_7: pwm_pwm_pin_config = &IfxGtm_ATOM0_7_TOUT7_P02_7_OUT;     break;
        case ATOM0_CH7_P20_8: pwm_pwm_pin_config = &IfxGtm_ATOM0_7_TOUT64_P20_8_OUT;    break;

        case ATOM1_CH0_P00_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT9_P00_0_OUT;     break;
        case ATOM1_CH0_P02_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT0_P02_0_OUT;     break;
        case ATOM1_CH0_P02_8: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT8_P02_8_OUT;     break;
        case ATOM1_CH0_P15_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT76_P15_5_OUT;    break;
        case ATOM1_CH0_P15_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT77_P15_6_OUT;    break;
        case ATOM1_CH0_P20_12:pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT68_P20_12_OUT;   break;

        case ATOM1_CH1_P00_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT10_P00_1_OUT;    break;
        case ATOM1_CH1_P00_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT11_P00_2_OUT;    break;
        case ATOM1_CH1_P02_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT1_P02_1_OUT;     break;
        case ATOM1_CH1_P10_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT103_P10_1_OUT;   break;
        case ATOM1_CH1_P14_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT86_P14_6_OUT;    break;
        case ATOM1_CH1_P15_7: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT78_P15_7_OUT;    break;
        case ATOM1_CH1_P15_8: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT79_P15_8_OUT;    break;
        case ATOM1_CH1_P20_13:pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT69_P20_13_OUT;   break;
        case ATOM1_CH1_P33_9: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT31_P33_9_OUT;    break;

        case ATOM1_CH2_P00_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT12_P00_3_OUT;    break;
        case ATOM1_CH2_P02_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT2_P02_2_OUT;     break;
        case ATOM1_CH2_P10_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT104_P10_2_OUT;   break;
        case ATOM1_CH2_P10_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT107_P10_5_OUT;   break;
        case ATOM1_CH2_P14_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT80_P14_0_OUT;    break;
        case ATOM1_CH2_P20_14:pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT70_P20_14_OUT;   break;
        case ATOM1_CH2_P33_11:pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT33_P33_11_OUT;   break;

        case ATOM1_CH3_P00_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT13_P00_4_OUT;    break;
        case ATOM1_CH3_P02_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT3_P02_3_OUT;     break;
        case ATOM1_CH3_P10_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT105_P10_3_OUT;   break;
        case ATOM1_CH3_P10_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT108_P10_6_OUT;   break;
        case ATOM1_CH3_P15_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT71_P15_0_OUT;    break;

        case ATOM1_CH4_P00_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT14_P00_5_OUT;    break;
        case ATOM1_CH4_P02_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT4_P02_4_OUT;     break;
        case ATOM1_CH4_P15_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT72_P15_1_OUT;    break;
        case ATOM1_CH4_P20_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT61_P20_3_OUT;    break;

        case ATOM1_CH5_P00_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT15_P00_6_OUT;    break;
        case ATOM1_CH5_P02_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT5_P02_5_OUT;     break;
        case ATOM1_CH5_P15_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT73_P15_2_OUT;    break;
        case ATOM1_CH5_P20_9: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT65_P20_9_OUT;    break;
        case ATOM1_CH5_P32_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT40_P32_4_OUT;    break;

        case ATOM1_CH6_P00_7: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT16_P00_7_OUT;    break;
        case ATOM1_CH6_P02_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT6_P02_6_OUT;     break;
        case ATOM1_CH6_P15_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT74_P15_3_OUT;    break;
        case ATOM1_CH6_P20_10:pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT66_P20_10_OUT;   break;
        case ATOM1_CH6_P23_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT42_P23_1_OUT;    break;

        case ATOM1_CH7_P00_8: pwm_pwm_pin_config = &IfxGtm_ATOM1_7_TOUT17_P00_8_OUT;    break;
        case ATOM1_CH7_P02_7: pwm_pwm_pin_config = &IfxGtm_ATOM1_7_TOUT7_P02_7_OUT;     break;
        case ATOM1_CH7_P15_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_7_TOUT75_P15_4_OUT;    break;
        case ATOM1_CH7_P20_11:pwm_pwm_pin_config = &IfxGtm_ATOM1_7_TOUT67_P20_11_OUT;   break;

        case ATOM2_CH0_P00_9: pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT18_P00_9_OUT;    break;
        case ATOM2_CH0_P13_3: pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT94_P13_3_OUT;    break;
        case ATOM2_CH0_P33_4: pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT26_P33_4_OUT;    break;
        case ATOM2_CH0_P33_10:pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT32_P33_10_OUT;   break;

        case ATOM2_CH1_P11_2: pwm_pwm_pin_config = &IfxGtm_ATOM2_1_TOUT95_P11_2_OUT;    break;
        case ATOM2_CH1_P33_5: pwm_pwm_pin_config = &IfxGtm_ATOM2_1_TOUT27_P33_5_OUT;    break;

        case ATOM2_CH2_P11_3: pwm_pwm_pin_config = &IfxGtm_ATOM2_2_TOUT96_P11_3_OUT;    break;
        case ATOM2_CH2_P33_6: pwm_pwm_pin_config = &IfxGtm_ATOM2_2_TOUT28_P33_6_OUT;    break;

        case ATOM2_CH3_P00_12:pwm_pwm_pin_config = &IfxGtm_ATOM2_3_TOUT21_P00_12_OUT;   break;
        case ATOM2_CH3_P11_6: pwm_pwm_pin_config = &IfxGtm_ATOM2_3_TOUT97_P11_6_OUT;    break;
        case ATOM2_CH3_P33_7: pwm_pwm_pin_config = &IfxGtm_ATOM2_3_TOUT29_P33_7_OUT;    break;

        case ATOM2_CH4_P11_9: pwm_pwm_pin_config = &IfxGtm_ATOM2_4_TOUT98_P11_9_OUT;    break;
        case ATOM2_CH4_P33_8: pwm_pwm_pin_config = &IfxGtm_ATOM2_4_TOUT30_P33_8_OUT;    break;
        case ATOM2_CH4_P33_12:pwm_pwm_pin_config = &IfxGtm_ATOM2_4_TOUT34_P33_12_OUT;   break;

        case ATOM2_CH5_P11_10:pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT99_P11_10_OUT;   break;
        case ATOM2_CH5_P13_0: pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT91_P13_0_OUT;    break;
        case ATOM2_CH5_P20_9: pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT65_P20_9_OUT;    break;
        case ATOM2_CH5_P33_13:pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT35_P33_13_OUT;   break;

        case ATOM2_CH6_P11_11:pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT100_P11_11_OUT;  break;
        case ATOM2_CH6_P13_1: pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT92_P13_1_OUT;    break;
        case ATOM2_CH6_P20_6: pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT62_P20_6_OUT;    break;
        case ATOM2_CH6_P32_0: pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT36_P32_0_OUT;    break;

        case ATOM2_CH7_P11_12:pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT101_P11_12_OUT;  break;
        case ATOM2_CH7_P13_2: pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT93_P13_2_OUT;    break;
        case ATOM2_CH7_P20_7: pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT63_P20_7_OUT;    break;

        case ATOM3_CH0_P00_9: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT18_P00_9_OUT;    break;
        case ATOM3_CH0_P13_3: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT94_P13_3_OUT;    break;
        case ATOM3_CH0_P33_4: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT26_P33_4_OUT;    break;
        case ATOM3_CH0_P33_10:pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT32_P33_10_OUT;   break;

        case ATOM3_CH1_P11_2: pwm_pwm_pin_config = &IfxGtm_ATOM3_1_TOUT95_P11_2_OUT;    break;
        case ATOM3_CH1_P33_5: pwm_pwm_pin_config = &IfxGtm_ATOM3_1_TOUT27_P33_5_OUT;    break;

        case ATOM3_CH2_P11_3: pwm_pwm_pin_config = &IfxGtm_ATOM3_2_TOUT96_P11_3_OUT;    break;
        case ATOM3_CH2_P33_6: pwm_pwm_pin_config = &IfxGtm_ATOM3_2_TOUT28_P33_6_OUT;    break;

        case ATOM3_CH3_P00_12:pwm_pwm_pin_config = &IfxGtm_ATOM3_3_TOUT21_P00_12_OUT;   break;
        case ATOM3_CH3_P11_6: pwm_pwm_pin_config = &IfxGtm_ATOM3_3_TOUT97_P11_6_OUT;    break;
        case ATOM3_CH3_P33_7: pwm_pwm_pin_config = &IfxGtm_ATOM3_3_TOUT29_P33_7_OUT;    break;

        case ATOM3_CH4_P11_9: pwm_pwm_pin_config = &IfxGtm_ATOM3_4_TOUT98_P11_9_OUT;    break;
        case ATOM3_CH4_P33_8: pwm_pwm_pin_config = &IfxGtm_ATOM3_4_TOUT30_P33_8_OUT;    break;
        case ATOM3_CH4_P33_12:pwm_pwm_pin_config = &IfxGtm_ATOM3_4_TOUT34_P33_12_OUT;   break;

        case ATOM3_CH5_P11_10:pwm_pwm_pin_config = &IfxGtm_ATOM3_5_TOUT99_P11_10_OUT;   break;
        case ATOM3_CH5_P13_0: pwm_pwm_pin_config = &IfxGtm_ATOM3_5_TOUT91_P13_0_OUT;    break;
        case ATOM3_CH5_P33_13:pwm_pwm_pin_config = &IfxGtm_ATOM3_5_TOUT35_P33_13_OUT;   break;

        case ATOM3_CH6_P11_11:pwm_pwm_pin_config = &IfxGtm_ATOM3_6_TOUT100_P11_11_OUT;  break;
        case ATOM3_CH6_P13_1: pwm_pwm_pin_config = &IfxGtm_ATOM3_6_TOUT92_P13_1_OUT;    break;
        case ATOM3_CH6_P20_6: pwm_pwm_pin_config = &IfxGtm_ATOM3_6_TOUT62_P20_6_OUT;    break;
        case ATOM3_CH6_P32_0: pwm_pwm_pin_config = &IfxGtm_ATOM3_6_TOUT36_P32_0_OUT;    break;

        case ATOM3_CH7_P11_12:pwm_pwm_pin_config = &IfxGtm_ATOM3_7_TOUT101_P11_12_OUT;  break;
        case ATOM3_CH7_P13_2: pwm_pwm_pin_config = &IfxGtm_ATOM3_7_TOUT93_P13_2_OUT;    break;
        case ATOM3_CH7_P20_7: pwm_pwm_pin_config = &IfxGtm_ATOM3_7_TOUT63_P20_7_OUT;    break;

        default: zf_assert(FALSE); pwm_pwm_pin_config = NULL;
    }
    return pwm_pwm_pin_config;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     初始化互补 PWM
//  参数说明     ccx_pin        主 PWM 引脚
//  参数说明     coutx_pin      互补 PWM 引脚
//  参数说明     freq           频率 (Hz)
//  参数说明     duty           占空比
//  示例：   pwm_hl_init(ATOM0_CH1_P21_3, ATOM0_CH2_P21_4, 10000, 0);
//  注：不可以使用ATOMx_CH0
//-------------------------------------------------------------------------------------------------------------------
void pwm_hl_init(pwm_channel_enum ccx_pin, pwm_channel_enum coutx_pin, uint32 freq, uint32 duty)
{
    IfxGtm_Atom_ToutMap* ccx_map = get_pwm_hl_pin(ccx_pin);
    IfxGtm_Atom_ToutMap* coutx_map = get_pwm_hl_pin(coutx_pin);

    zf_assert(ccx_map != NULL);
    zf_assert(coutx_map != NULL);
    zf_assert(ccx_map->atom == coutx_map->atom); // 互补PWM必须处于同一个 ATOM 模块
    zf_assert(duty <= PWM_DUTY_MAX);

    uint8 atom_num = ccx_map->atom;
    uint8 ccx_ch   = ccx_map->channel;

    // 1. 初始化 GTM 时钟
    IfxGtm_enable(&MODULE_GTM);
    if(!(MODULE_GTM.CMU.CLK_EN.U & 0x2))
    {
        IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CMU_CLK_FREQ);
        IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
    }

    // 2. 初始化对应 ATOM 模块的 Timer (使用通道 0)
    // 互补PWM底层需要一个单独的 Timer 通道做同步，这里默认占用 ATOMx_CH0
    if (!g_timer_hl_init_flag[atom_num])
    {
        IfxGtm_Atom_Timer_Config timerConfig;
        IfxGtm_Atom_Timer_initConfig(&timerConfig, &MODULE_GTM);

        timerConfig.atom            = (IfxGtm_Atom)atom_num;
        timerConfig.timerChannel    = IfxGtm_Atom_Ch_0;         // 默认占用 CH0 做底层定时器
        timerConfig.clock           = IfxGtm_Cmu_Clk_0;
        timerConfig.base.frequency  = (float32)freq;
        timerConfig.base.isrPriority= 0;
        timerConfig.base.countDir   = IfxStdIf_Timer_CountDir_up; // PwmHl 必须是 Up 计数

        IfxGtm_Atom_Timer_init(&g_timer_hl[atom_num], &timerConfig);
        g_timer_hl_init_flag[atom_num] = TRUE;
    }

    // 3. 配置互补 PWM
    IfxGtm_Atom_PwmHl_Config pwmhlConfig;
    IfxGtm_Atom_PwmHl_initConfig(&pwmhlConfig);

    pwmhlConfig.timer               = &g_timer_hl[atom_num];
    pwmhlConfig.atom                = (IfxGtm_Atom)atom_num;
    pwmhlConfig.base.channelCount   = 1;                     // 本次配置1对通道
    pwmhlConfig.base.deadtime       = PWM_HL_DEADTIME_SEC;   // 设置死区 50ns
    pwmhlConfig.base.minPulse       = 0.0f;

    // 引脚输出状态设置（默认高电平有效）
    pwmhlConfig.base.ccxActiveState   = Ifx_ActiveState_high;
    pwmhlConfig.base.coutxActiveState = Ifx_ActiveState_high;
    pwmhlConfig.base.outputMode       = IfxPort_OutputMode_pushPull;
    pwmhlConfig.base.outputDriver     = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    // 绑定映射的引脚
    const IfxGtm_Atom_ToutMap *ccx_array[1]   = {ccx_map};
    const IfxGtm_Atom_ToutMap *coutx_array[1] = {coutx_map};

    pwmhlConfig.ccx   = ccx_array;
    pwmhlConfig.coutx = coutx_array;

    // 执行初始化并设置为中心对齐模式（适合电机等应用，也可选左对齐 Ifx_Pwm_Mode_leftAligned）
    IfxGtm_Atom_PwmHl_init(&g_pwm_hl[atom_num][ccx_ch], &pwmhlConfig);
    IfxGtm_Atom_PwmHl_setMode(&g_pwm_hl[atom_num][ccx_ch], Ifx_Pwm_Mode_centerAligned);

    // 4. 设置初始占空比并启动
    pwm_hl_set_duty(ccx_pin, duty);
    IfxGtm_Atom_Timer_run(&g_timer_hl[atom_num]);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     设置互补 PWM 占空比
//  参数说明     ccx_pin        主 PWM 引脚
//  参数说明     duty           占空比
//-------------------------------------------------------------------------------------------------------------------
void pwm_hl_set_duty(pwm_channel_enum ccx_pin, uint32 duty)
{
    IfxGtm_Atom_ToutMap* ccx_map = get_pwm_hl_pin(ccx_pin);
    zf_assert(ccx_map != NULL);

    uint8 atom_num = ccx_map->atom;
    uint8 ccx_ch   = ccx_map->channel;

    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

    IfxGtm_Atom_Timer *timer = &g_timer_hl[atom_num];

    // 获取当前周期(Ticks)
    Ifx_TimerValue period = timer->base.period;

    // 计算导通时间(Ticks)
    Ifx_TimerValue tOn[1];
    tOn[0] = (Ifx_TimerValue)(((uint64)duty * period) / PWM_DUTY_MAX);

    // 暂停更新 -> 设置通道占空比 -> 应用更新
    IfxGtm_Atom_Timer_disableUpdate(timer);
    IfxGtm_Atom_PwmHl_setOnTime(&g_pwm_hl[atom_num][ccx_ch], tOn);
    IfxGtm_Atom_Timer_applyUpdate(timer);
}
