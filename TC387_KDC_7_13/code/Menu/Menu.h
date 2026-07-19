#ifndef _MENU_H_
#define _MENU_H_

#include <stdint.h>
#include "board_mode_switch.h"

/**
 * @file Menu.h
 * @brief 基于 2 寸 IPS200 的整车模式显示和任务调度模块。
 *
 * 页面框架仿照参考 UI：屏幕按 8x16 字体划分为 30 列、20 行，先逐行显示
 * 固定文本模板，再覆盖实时数据。主板拨码仅在 Menu_init() 中读取一次，
 * 因此页面和工作模式在本次上电期间保持一致。
 */

#define MENU_COLUMN_COUNT              (30U)
#define MENU_ROW_COUNT                 (20U)
#define MENU_FONT_WIDTH                (8U)
#define MENU_FONT_HEIGHT               (16U)
#define MENU_CURSOR_ROW                (1U)
#define MENU_CURSOR_FIRST_CHAR         ('-')
#define MENU_CURSOR_SECOND_CHAR        ('>')
#define MENU_STARTUP_PAGE_HOLD_MS      (2000U)

/** Automatic pages: startup, remote, test tuning, and save-point status. */
typedef enum
{
    MENU_PAGE_SYSTEM = 0,
    MENU_PAGE_REMOTE,
    MENU_PAGE_TEST,
    MENU_PAGE_SAVE_POINT
} MenuPage_e;

/** CPU0 在 Menu_init() 前提交的外设初始化结果。 */
typedef struct
{
    uint8_t clock_ready;
    uint8_t debug_uart_ready;
    uint8_t motor_encoder_ready;
    uint8_t imu_ready;
    uint8_t gps_enabled;
    uint8_t gps_initialized;
    uint8_t lora_initialized;
    uint8_t display_initialized;
} MenuPeripheralStatus_t;

/** 给循迹、存点等外部控制模块预留的生命周期回调。 */
typedef struct
{
    void (*enter)(void);  /**< 启动模式且全部电机已安全停止后调用一次。 */
    void (*task)(void);   /**< 在主循环中反复调用，不允许阻塞。 */
    void (*exit)(void);   /**< 预留退出接口；当前启动模式运行中不会切换。 */
} MenuModeHooks_t;

/**
 * 菜单运行状态，结构形式与参考 UI_CLASS 相同：显示函数与页面状态集中保存。
 * cursor_row 当前固定在标题行，因为本项目不再使用主板上的四个菜单按钮。
 */
typedef struct
{
    void (*Disp)(void);
    volatile CarMode_e mode;          /**< 主循环和 5 ms 回调共享的启动模式。 */
    volatile MenuPage_e page;         /**< 当前自动显示的页面。 */
    uint8_t cursor_row;
    volatile uint8_t entered;         /**< 1 表示 CPU0 主循环已完成模式安全进入。 */
    uint16_t startup_page_ticks;
} MENU_CLASS;

extern MENU_CLASS menu;

/** 在 Menu_init() 前保存 CPU0 已完成的外设初始化结果。 */
void Menu_SetPeripheralInitStatus(const MenuPeripheralStatus_t *status);

/** 初始化拨码、LoRa、控制模式和 IPS200；首次画面由 Menu_Display() 绘制。 */
void Menu_init(void);

/** 主循环非阻塞任务：接收 LoRa、进入启动模式和处理模式数据，不刷新屏幕。 */
void Menu_Task(void);

/** CPU0 100 ms task: update page timing and save-point WiFi retry timing. */
void Menu_100msTask(void);

/** CPU2 的 5 ms 控制中断接口：执行失联保护和当前模式电机控制，不访问屏幕。 */
void Control_5msCallback(void);

/** 根据 menu.page 刷新页面；由 CPU0 的低速显示时间片调用。 */
void Menu_Display(void);

/**
 * 逐行显示一个 30x20 字符页面，并在 cursor_row 行绘制“->”光标。
 * 页面字符串可以短于 30 字符，剩余区域会自动补空格以清除旧数据。
 */
void Menu_DisplayPage(
    const char page[MENU_ROW_COUNT][MENU_COLUMN_COUNT + 1U],
    uint8_t cursor_row);

/** 返回初始化时由主板拨码选择的工作模式。 */
CarMode_e Menu_GetMode(void);

/** 返回模式英文名称，非法枚举返回 "UNKNOWN"。 */
const char *Menu_GetModeName(CarMode_e mode);

/** 为指定模式注册生命周期接口。 */
void Menu_RegisterModeHooks(CarMode_e mode, const MenuModeHooks_t *hooks);

#endif
