#include "zf_common_headfile.h"
#include "save_point_mode.h"
#include "reverse_track_mode.h"

/* 菜单页面计时由 CPU0 主循环中的 100 ms 显示任务推进。 */
#define MENU_MAIN_PERIOD_MS            (100U)
#define MENU_STARTUP_PAGE_TICKS        (MENU_STARTUP_PAGE_HOLD_MS / MENU_MAIN_PERIOD_MS)

/* 第 1 页：外设初始化状态、启动拨码模式和当前导航结果。 */
static const char g_system_page[MENU_ROW_COUNT][MENU_COLUMN_COUNT + 1U] =
{
    "                              ",
    "       SYSTEM / INS STATUS    ",
    "   CLOCK:         DEBUG:      ",
    " IPS INIT:     LORA INIT:     ",
    "   MOTOR / ENCODER:           ",
    "   IMU963RA:                  ",
    "   GPS INIT:      GPS FIX:    ",
    "   LORA LINK:                 ",
    "   S6 BOOT MODE:              ",
    "   NAV SOLUTION:              ",
    "   NAV VALID:     SRC:        ",
    "   CAR X (m):                 ",
    "   CAR Y (m):                 ",
    "   HEADING (deg):             ",
    "   SPEED (m/s):               ",
    "   ENCODER VALID:             ",
    "   GPS LOCAL X (m):           ",
    "   GPS LOCAL Y (m):           ",
    "   GPS SPEED (m/s):           ",
    "   PAGE 1 / STARTUP STATUS    "
};

/* 第 2 页：遥控器、三电机、编码器、解算姿态、车辆坐标和 GPS 实时数据。 */
static const char g_remote_page[MENU_ROW_COUNT][MENU_COLUMN_COUNT + 1U] =
{
    "                              ",
    "       REMOTE DATA / PAGE 2   ",
    "   LINK:        AGE(ms):      ",
    "   JOY0:        JOY1:         ",
    "   JOY2:        JOY3:         ",
    "   KEY MASK:    SW MASK:      ",
    "   PWM LEFT:                  ",
    "   PWM RIGHT:                 ",
    "   PWM STEER:                 ",
    "   ENC LEFT (m/s):            ",
    "   ENC RIGHT(m/s):            ",
    "   ENC STEER (deg):           ",
    "   IMU ROLL (deg):            ",
    "   IMU PITCH(deg):            ",
    "   IMU HEADING(deg):          ",
    "   CAR X (m):                 ",
    "   CAR Y (m):                 ",
    "   GPS SPEED(m/s):            ",
    "   GPS FIX:   DIR:      SAT:  ",
    "   LOST => ALL MOTORS OFF     "
};

/* 测试模式继续使用在线 PID 调参页面。 */
static const char g_test_page[MENU_ROW_COUNT][MENU_COLUMN_COUNT + 1U] =
{
    "                              ",
    "       MOTOR PID TEST         ",
    "   LINK:          AGE:        ",
    "   CHANNEL:                   ",
    "   A TARGET:                  ",
    "   A MEASURE:                 ",
    "   A ERROR:                   ",
    "   A PWM:                     ",
    "   B TARGET:                  ",
    "   B MEASURE:                 ",
    "   B ERROR:                   ",
    "   B PWM:                     ",
    "   STEER TAR:                 ",
    "   STEER MEA:                 ",
    "   STEER PWM:                 ",
    "   Kp:                        ",
    "   Ki:                        ",
    "   Kd:                        ",
    " K0 TERM K1 CLR  K2- K3+      ",
    " SW0 S SW1 L SW2 R SW3 DRIVE  "
};

static const char g_save_point_page[MENU_ROW_COUNT][MENU_COLUMN_COUNT + 1U] =
{
    "                              ",
    "       SAVE POINT MODE        ",
    "   STATE:                     ",
    "   POINTS:       /1024        ",
    "   CUR X (m):                 ",
    "   CUR Y (m):                 ",
    "   LAST X (m):                ",
    "   LAST Y (m):                ",
    "   REL YAW (deg):             ",
    "   TO NEXT (m):               ",
    "   LORA LINK:                 ",
    "   WIFI INIT:                 ",
    "   TCP LINK:                  ",
    "   WIFI SENT:                 ",
    "   WIFI ERR:                  ",
    "   FLASH:                     ",
    "   LX: STEER  RY: SPEED      ",
    "   S5: 0 OPEN / 1 ACK DIFF   ",
    "   S3: START / RESTART       ",
    "   S4: STOP  S3+S4: SAVE     "
};

static const char *const g_mode_names[CAR_MODE_COUNT] =
{
    "TRACK",
    "SAVE POINT",
    "TEST",
    "REMOTE"
};

static MenuModeHooks_t g_mode_hooks[CAR_MODE_COUNT];
static MenuPeripheralStatus_t g_peripheral_status;
/* 固定页面只在首次显示或页面切换时重画，避免每 100 ms 先擦掉动态数字再覆盖。 */
static uint8_t g_display_page_valid;
static MenuPage_e g_displayed_page;

MENU_CLASS menu =
{
    Menu_Display,
    CAR_MODE_TRACK,
    MENU_PAGE_SYSTEM,
    MENU_CURSOR_ROW,
    0U,
    0U
};

static const char *menu_ready_text(uint8_t ready)
{
    return (ready != 0U) ? "OK  " : "FAIL";
}

/**
 * @brief 按固定字符宽度显示字符串，并用空格清除上一次较长内容的尾部。
 * @note  用于链路状态、速度来源和测试通道等长度会变化的动态文本。
 */
static void menu_show_fixed_string(uint16_t x,
                                   uint16_t y,
                                   const char *text,
                                   uint8_t width)
{
    char buffer[MENU_COLUMN_COUNT + 1U];
    uint8_t index;

    if(text == 0) { return; }
    if(width > MENU_COLUMN_COUNT) { width = MENU_COLUMN_COUNT; }

    memset(buffer, ' ', width);
    for(index = 0U; (index < width) && (text[index] != '\0'); ++index)
    {
        buffer[index] = text[index];
    }
    buffer[width] = '\0';
    ips200_show_string(x, y, buffer);
}

static const char *menu_speed_source_name(TopSpeed_GPS_INS_SpeedSource source)
{
    if(source == TOPSPEED_GPS_INS_SPEED_ENCODER) { return "ENC"; }
    if(source == TOPSPEED_GPS_INS_SPEED_GPS) { return "GPS"; }
    return "NONE";
}

/** 将四个独立按键/拨码状态压缩为低四位掩码，便于屏幕显示。 */
static uint8_t menu_pack_bits(const uint8_t values[4])
{
    uint8_t mask = 0U;
    uint8_t index;

    for(index = 0U; index < 4U; ++index)
    {
        if(values[index] != 0U)
        {
            mask = (uint8_t)(mask | (uint8_t)(1U << index));
        }
    }
    return mask;
}

/** 关闭所有可能写电机的控制器，允许在初始化阶段重复调用。 */
static void menu_emergency_stop(void)
{
    Ackermann_control_enable(0U);
    ReverseTrackMode_Exit();
    TestMode_EmergencyStop();
    RemoteMode_EmergencyStop();
    SavePointMode_Exit();
    Motor_stop_all();
}

static void menu_enter_internal_mode(CarMode_e mode)
{
    if(mode == CAR_MODE_TRACK)
    {
        ReverseTrackMode_Enter();
    }
    else if(mode == CAR_MODE_TEST)
    {
        TestMode_Enter();
    }
    else if(mode == CAR_MODE_SAVE_POINT)
    {
        /* SAVE POINT keeps the recording keys while reusing the proven
         * joystick, S5 differential and LoRa failsafe motor controller. */
        RemoteMode_Enter();
        SavePointMode_Enter();
    }
    else if(mode == CAR_MODE_REMOTE)
    {
        RemoteMode_Enter();
    }
}

static const char *menu_test_channel_name(TestModeChannel_e channel)
{
    static const char *const names[] =
    {
        "NONE",
        "STEERING",
        "LEFT REAR",
        "RIGHT REAR",
        "DRIVE",
        "SW CONFLICT"
    };

    if(channel > TEST_MODE_CHANNEL_SWITCH_CONFLICT) { return "UNKNOWN"; }
    return names[channel];
}

static const char *menu_save_point_state(const SavePointModeStatus_t *status)
{
    if(status->flash_save_pending != 0U) { return "SAVING"; }
    if(status->full != 0U) { return "FULL"; }
    if(status->recording != 0U) { return "RECORDING"; }
    if(status->start_rejected != 0U) { return "WAIT NAV/IMU"; }
    if(status->flash_saved != 0U) { return "SAVED"; }
    if(status->point_count != 0U) { return "STOPPED"; }
    return "IDLE";
}

static const char *menu_save_point_flash_state(
    const SavePointModeStatus_t *status)
{
    if(status->flash_save_pending != 0U) { return "WRITING"; }
    if(status->flash_saved != 0U) { return "SAVED"; }
    if(status->point_count != 0U) { return "UNSAVED"; }
    return "EMPTY";
}

/** 第 1 页动态数据：外设、拨码模式、解算方式和车辆坐标。 */
static void menu_display_system_data(void)
{
    TopSpeed_GPS_INS_Output navigation;
    TopSpeed_GPS_INS_GPS_Output gps;
    LoraRemoteState_t remote;

    TopSpeed_GPS_INS_PortGetOutput(&navigation);
    TopSpeed_GPS_INS_PortGetGPSOutput(&gps);
    LoraRemote_GetState(&remote);

    ips200_show_string(72U, 2U * MENU_FONT_HEIGHT,
                       menu_ready_text(g_peripheral_status.clock_ready));
    ips200_show_string(208U, 2U * MENU_FONT_HEIGHT,
                       menu_ready_text(g_peripheral_status.debug_uart_ready));
    ips200_show_string(80U, 3U * MENU_FONT_HEIGHT,
                       menu_ready_text(g_peripheral_status.display_initialized));
    ips200_show_string(200U, 3U * MENU_FONT_HEIGHT,
                       menu_ready_text(g_peripheral_status.lora_initialized));
    ips200_show_string(152U, 4U * MENU_FONT_HEIGHT,
                       menu_ready_text(g_peripheral_status.motor_encoder_ready));
    ips200_show_string(112U, 5U * MENU_FONT_HEIGHT,
                       menu_ready_text(g_peripheral_status.imu_ready));
    ips200_show_string(96U, 6U * MENU_FONT_HEIGHT,
        (g_peripheral_status.gps_enabled != 0U)
        ? menu_ready_text(g_peripheral_status.gps_initialized) : "OFF ");
    ips200_show_string(208U, 6U * MENU_FONT_HEIGHT,
                       (gps.valid != 0U) ? "OK  " : "NO  ");
    ips200_show_string(112U, 7U * MENU_FONT_HEIGHT,
                       (remote.link_ok != 0U) ? "OK  " : "LOST");
    ips200_show_string(128U, 8U * MENU_FONT_HEIGHT,
                       Menu_GetModeName(menu.mode));
    ips200_show_string(136U, 9U * MENU_FONT_HEIGHT,
                       TopSpeed_GPS_INS_PortGetNavigationSolutionName());
    ips200_show_string(104U, 10U * MENU_FONT_HEIGHT,
                       (navigation.valid != 0U) ? "OK  " : "NO  ");
    menu_show_fixed_string(208U, 10U * MENU_FONT_HEIGHT,
                           menu_speed_source_name(navigation.speed_source), 4U);

    ips200_show_float(128U, 11U * MENU_FONT_HEIGHT,
                      navigation.position_m.x, 8U, 2U);
    ips200_show_float(128U, 12U * MENU_FONT_HEIGHT,
                      navigation.position_m.y, 8U, 2U);
    ips200_show_float(152U, 13U * MENU_FONT_HEIGHT,
                      navigation.heading_deg, 7U, 1U);
    ips200_show_float(136U, 14U * MENU_FONT_HEIGHT,
                      navigation.speed_mps, 7U, 2U);
    ips200_show_string(152U, 15U * MENU_FONT_HEIGHT,
                       (navigation.encoder_valid != 0U) ? "OK  " : "NO  ");
    ips200_show_float(152U, 16U * MENU_FONT_HEIGHT,
                      navigation.gps_position_m.x, 7U, 2U);
    ips200_show_float(152U, 17U * MENU_FONT_HEIGHT,
                      navigation.gps_position_m.y, 7U, 2U);
    ips200_show_float(152U, 18U * MENU_FONT_HEIGHT,
                      navigation.gps_speed_mps, 7U, 2U);
}

/** 第 2 页动态数据：遥控器、三电机、编码器、解算姿态、车辆坐标和 GPS。 */
static void menu_display_remote_data(void)
{
    LoraRemoteState_t remote;
    RemoteModeStatus_t motor;
    TopSpeed_GPS_INS_Output navigation;
    TopSpeed_GPS_INS_IMU_Output imu;
    TopSpeed_GPS_INS_GPS_Output gps;

    LoraRemote_GetState(&remote);
    RemoteMode_GetStatus(&motor);
    TopSpeed_GPS_INS_PortGetOutput(&navigation);
    TopSpeed_GPS_INS_PortGetIMUOutput(&imu);
    TopSpeed_GPS_INS_PortGetGPSOutput(&gps);

    ips200_show_string(64U, 2U * MENU_FONT_HEIGHT,
                       (remote.link_ok != 0U) ? "OK  " : "LOST");
    ips200_show_uint(192U, 2U * MENU_FONT_HEIGHT, remote.age_ms, 4U);

    ips200_show_int(64U, 3U * MENU_FONT_HEIGHT, remote.joystick[0], 5U);
    ips200_show_int(176U, 3U * MENU_FONT_HEIGHT, remote.joystick[1], 5U);
    ips200_show_int(64U, 4U * MENU_FONT_HEIGHT, remote.joystick[2], 5U);
    ips200_show_int(176U, 4U * MENU_FONT_HEIGHT, remote.joystick[3], 5U);
    ips200_show_uint(96U, 5U * MENU_FONT_HEIGHT,
                     menu_pack_bits(remote.key), 2U);
    ips200_show_uint(208U, 5U * MENU_FONT_HEIGHT,
                     menu_pack_bits(remote.switch_key), 2U);

    /* PWM 没有小数位，必须用 show_int；show_float 不允许 pointnum=0。 */
    ips200_show_int(128U, 6U * MENU_FONT_HEIGHT, (int32)motor.left_pwm, 7U);
    ips200_show_int(128U, 7U * MENU_FONT_HEIGHT, (int32)motor.right_pwm, 7U);
    ips200_show_int(128U, 8U * MENU_FONT_HEIGHT,
                    (int32)motor.steering_pwm, 7U);
    /* 标签在第 19 列结束；数值从第 20 列开始，缩短整数位后仍保留符号。 */
    ips200_show_float(160U, 9U * MENU_FONT_HEIGHT,
                      motor.left_measured_mps, 5U, 2U);
    ips200_show_float(160U, 10U * MENU_FONT_HEIGHT,
                      motor.right_measured_mps, 5U, 2U);
    ips200_show_float(160U, 11U * MENU_FONT_HEIGHT,
                      motor.steering_measured_deg, 5U, 1U);

    ips200_show_float(160U, 12U * MENU_FONT_HEIGHT, imu.roll_deg, 7U, 1U);
    ips200_show_float(160U, 13U * MENU_FONT_HEIGHT, imu.pitch_deg, 7U, 1U);
    ips200_show_float(160U, 14U * MENU_FONT_HEIGHT, imu.heading_deg, 7U, 1U);
    ips200_show_float(128U, 15U * MENU_FONT_HEIGHT,
                      navigation.position_m.x, 8U, 2U);
    ips200_show_float(128U, 16U * MENU_FONT_HEIGHT,
                      navigation.position_m.y, 8U, 2U);
    ips200_show_float(144U, 17U * MENU_FONT_HEIGHT, gps.speed_mps, 7U, 2U);
    menu_show_fixed_string(88U, 18U * MENU_FONT_HEIGHT,
        (gps.valid != 0U) ? "OK" : ((gps.enabled != 0U) ? "NO" : "OFF"), 3U);
    ips200_show_float(144U, 18U * MENU_FONT_HEIGHT,
                      gps.direction_deg, 3U, 1U);
    ips200_show_uint(224U, 18U * MENU_FONT_HEIGHT,
                     gps.satellite_used, 2U);
}

static void menu_display_test_data(void)
{
    TestModeStatus_t status;
    ACKERMANN_PID_GAIN gain;

    TestMode_GetStatus(&status);
    ips200_show_string(64U, 2U * MENU_FONT_HEIGHT,
                       (status.lora_link_ok != 0U) ? "OK  " : "LOST");
    ips200_show_uint(176U, 2U * MENU_FONT_HEIGHT,
                     status.lora_age_ms, 4U);
    menu_show_fixed_string(88U, 3U * MENU_FONT_HEIGHT,
                           menu_test_channel_name(status.channel), 12U);
    ips200_show_float(112U, 4U * MENU_FONT_HEIGHT, status.target_a, 8U, 2U);
    ips200_show_float(112U, 5U * MENU_FONT_HEIGHT, status.measured_a, 8U, 2U);
    ips200_show_float(112U, 6U * MENU_FONT_HEIGHT, status.error_a, 8U, 2U);
    ips200_show_int(112U, 7U * MENU_FONT_HEIGHT, (int32)status.pwm_a, 8U);
    ips200_show_float(112U, 8U * MENU_FONT_HEIGHT, status.target_b, 8U, 2U);
    ips200_show_float(112U, 9U * MENU_FONT_HEIGHT, status.measured_b, 8U, 2U);
    ips200_show_float(112U, 10U * MENU_FONT_HEIGHT, status.error_b, 8U, 2U);
    ips200_show_int(112U, 11U * MENU_FONT_HEIGHT, (int32)status.pwm_b, 8U);
    ips200_show_float(112U, 12U * MENU_FONT_HEIGHT,
                      status.steering_target_deg, 8U, 1U);
    ips200_show_float(112U, 13U * MENU_FONT_HEIGHT,
                      status.steering_measured_deg, 8U, 1U);
    ips200_show_int(112U, 14U * MENU_FONT_HEIGHT,
                    (int32)status.steering_pwm, 8U);

    if(TestMode_GetPidGains(status.channel, &gain) == 0U)
    {
        ips200_show_string(8U, 15U * MENU_FONT_HEIGHT,
            (status.selected_term == TEST_MODE_PID_KP) ? ">" : " ");
        ips200_show_string(8U, 16U * MENU_FONT_HEIGHT,
            (status.selected_term == TEST_MODE_PID_KI) ? ">" : " ");
        ips200_show_string(8U, 17U * MENU_FONT_HEIGHT,
            (status.selected_term == TEST_MODE_PID_KD) ? ">" : " ");
        ips200_show_float(56U, 15U * MENU_FONT_HEIGHT, gain.kp, 8U, 2U);
        ips200_show_float(56U, 16U * MENU_FONT_HEIGHT, gain.ki, 8U, 2U);
        ips200_show_float(56U, 17U * MENU_FONT_HEIGHT, gain.kd, 8U, 2U);
    }
    else
    {
        /* 切到整车测试/无选择时，清除上一个单电机通道留下的选择符和 PID 数值。 */
        menu_show_fixed_string(8U, 15U * MENU_FONT_HEIGHT, "", 1U);
        menu_show_fixed_string(8U, 16U * MENU_FONT_HEIGHT, "", 1U);
        menu_show_fixed_string(8U, 17U * MENU_FONT_HEIGHT, "", 1U);
        menu_show_fixed_string(56U, 15U * MENU_FONT_HEIGHT, "", 12U);
        menu_show_fixed_string(56U, 16U * MENU_FONT_HEIGHT, "", 12U);
        menu_show_fixed_string(56U, 17U * MENU_FONT_HEIGHT, "", 12U);
    }
}

static void menu_display_save_point_data(void)
{
    SavePointModeStatus_t status;
    LoraRemoteState_t remote;
    float distance_to_next_m = 0.0f;

    SavePointMode_GetStatus(&status);
    LoraRemote_GetState(&remote);
    if(status.recording != 0U)
    {
        distance_to_next_m = SAVE_POINT_MODE_SAMPLE_DISTANCE_M
                           - status.distance_since_last_point_m;
        if(distance_to_next_m < 0.0f) { distance_to_next_m = 0.0f; }
        if(distance_to_next_m > SAVE_POINT_MODE_SAMPLE_DISTANCE_M)
        {
            distance_to_next_m = SAVE_POINT_MODE_SAMPLE_DISTANCE_M;
        }
    }

    menu_show_fixed_string(80U, 2U * MENU_FONT_HEIGHT,
                           menu_save_point_state(&status), 14U);
    ips200_show_uint(88U, 3U * MENU_FONT_HEIGHT,
                     status.point_count, 4U);
    ips200_show_float(112U, 4U * MENU_FONT_HEIGHT,
                      status.local_x_m, 8U, 2U);
    ips200_show_float(112U, 5U * MENU_FONT_HEIGHT,
                      status.local_y_m, 8U, 2U);
    ips200_show_float(120U, 6U * MENU_FONT_HEIGHT,
                      status.last_point_x_m, 8U, 2U);
    ips200_show_float(120U, 7U * MENU_FONT_HEIGHT,
                      status.last_point_y_m, 8U, 2U);
    ips200_show_float(136U, 8U * MENU_FONT_HEIGHT,
                      status.relative_yaw_deg, 7U, 1U);
    ips200_show_float(120U, 9U * MENU_FONT_HEIGHT,
                      distance_to_next_m, 5U, 2U);
    menu_show_fixed_string(112U, 10U * MENU_FONT_HEIGHT,
                           (remote.link_ok != 0U) ? "OK" : "LOST", 5U);
    menu_show_fixed_string(112U, 11U * MENU_FONT_HEIGHT,
        (status.wifi_initialized != 0U)
        ? "OK" : ((status.wifi_error_count != 0U) ? "RETRY" : "WAIT"), 5U);
    menu_show_fixed_string(104U, 12U * MENU_FONT_HEIGHT,
        (status.wifi_connected != 0U)
        ? "OK" : ((status.wifi_error_count != 0U) ? "RETRY" : "WAIT"), 5U);
    ips200_show_uint(112U, 13U * MENU_FONT_HEIGHT,
                     status.wifi_sent_count, 4U);
    ips200_show_uint(104U, 14U * MENU_FONT_HEIGHT,
                     status.wifi_error_count, 5U);
    menu_show_fixed_string(88U, 15U * MENU_FONT_HEIGHT,
                           menu_save_point_flash_state(&status), 8U);
}

void Menu_DisplayPage(
    const char page[MENU_ROW_COUNT][MENU_COLUMN_COUNT + 1U],
    uint8_t cursor_row)
{
    char row[MENU_COLUMN_COUNT + 1U];
    uint8_t y;
    uint8_t x;

    for(y = 0U; y < MENU_ROW_COUNT; ++y)
    {
        memset(row, ' ', MENU_COLUMN_COUNT);
        for(x = 0U; x < MENU_COLUMN_COUNT; ++x)
        {
            if(page[y][x] == '\0') { break; }
            row[x] = page[y][x];
        }
        if(y == cursor_row)
        {
            row[0] = MENU_CURSOR_FIRST_CHAR;
            row[1] = MENU_CURSOR_SECOND_CHAR;
        }
        else
        {
            row[0] = ' ';
            row[1] = ' ';
        }
        row[MENU_COLUMN_COUNT] = '\0';
        ips200_show_string(0U, (uint16_t)y * MENU_FONT_HEIGHT, row);
    }
}

void Menu_SetPeripheralInitStatus(const MenuPeripheralStatus_t *status)
{
    if(status != 0)
    {
        g_peripheral_status = *status;
    }
}

void Menu_init(void)
{
    /* BoardModeSwitch_Init 内部完成本次上电唯一一次主板拨码读取。 */
    menu.mode = BoardModeSwitch_Init();
    printf("S6 boot mode: %u (%s)\r\n",
           (unsigned int)menu.mode,
           (menu.mode < CAR_MODE_COUNT)
               ? g_mode_names[menu.mode] : "UNKNOWN");
    menu.page = MENU_PAGE_SYSTEM;
    menu.startup_page_ticks = 0U;
    g_display_page_valid = 0U;

    LoraRemote_Init();
    g_peripheral_status.lora_initialized = 1U;
    TestMode_Init();
    RemoteMode_Init();
    SavePointMode_Init();
    ReverseTrackMode_Init();
    menu_emergency_stop();

    menu.cursor_row = MENU_CURSOR_ROW;
    menu.entered = 0U;

    /* IPS200 的方向必须在初始化前设置，初始化后再设置只会改变软件宽高。 */
    ips200_set_dir(IPS200_PORTAIT_180);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_clear();
    g_peripheral_status.display_initialized = 1U;
}

void Menu_Task(void)
{
    if(menu.entered == 0U)
    {
        menu_emergency_stop();
        menu_enter_internal_mode(menu.mode);
        if((menu.mode < CAR_MODE_COUNT) && (g_mode_hooks[menu.mode].enter != 0))
        {
            g_mode_hooks[menu.mode].enter();
        }
        menu.entered = 1U;
    }

    if(menu.mode == CAR_MODE_TRACK)
    {
        ReverseTrackMode_Task();
    }
    else if(menu.mode == CAR_MODE_SAVE_POINT)
    {
        SavePointMode_Task();
    }
    if((menu.mode < CAR_MODE_COUNT) && (g_mode_hooks[menu.mode].task != 0))
    {
        g_mode_hooks[menu.mode].task();
    }

    /*
     * 屏幕刷新不在这里执行。
     * CPU0 的 100 ms 显示任务统一调用 Menu_Display()，避免主循环每次处理
     * LoRa 数据时都访问 IPS200，影响 5 ms 电机控制任务的实时性。
     */
}

void Menu_100msTask(void)
{
    MenuPage_e next_page;

    if(menu.mode == CAR_MODE_SAVE_POINT)
    {
        SavePointMode_100msTask();
    }

    /*
     * 该函数只在 CPU0 主循环的 100 ms 菜单任务中调用。
     * 上电先显示状态页 2 秒，再按锁存的启动模式进入工作页面。
     */
    if(menu.startup_page_ticks < MENU_STARTUP_PAGE_TICKS)
    {
        ++menu.startup_page_ticks;
        if(menu.startup_page_ticks == MENU_STARTUP_PAGE_TICKS)
        {
            next_page = MENU_PAGE_SYSTEM;
            if(menu.mode == CAR_MODE_REMOTE) { next_page = MENU_PAGE_REMOTE; }
            else if(menu.mode == CAR_MODE_TEST) { next_page = MENU_PAGE_TEST; }
            else if(menu.mode == CAR_MODE_SAVE_POINT)
            {
                next_page = MENU_PAGE_SAVE_POINT;
            }
            menu.page = next_page;
        }
    }
}

void Control_5msCallback(void)
{
    /* 本函数只做实时控制，不访问 IPS200，也不修改菜单页面。 */
    /* UART2 and the complete LoRa control path are owned by CPU2. */
    LoraRemote_Task();
    LoraRemote_5msCallback();

    /* Ordered CPU2 chain: encoder -> Ackermann telemetry -> mode control ->
     * final motor/steering PID and PWM. */
    Ackermann_port_5ms_callback();

    /* Menu_Task 完成安全 enter 之前，不允许任何电机闭环运行。 */
    if(menu.entered == 0U)
    {
        return;
    }
    if(menu.mode == CAR_MODE_TEST)
    {
        TestMode_Task();
        TestMode_5msCallback();
    }
    else if(menu.mode == CAR_MODE_REMOTE)
    {
        RemoteMode_5msCallback();
    }
    else if(menu.mode == CAR_MODE_SAVE_POINT)
    {
        /* RemoteMode is the sole motor owner; SavePointMode only samples
         * navigation and handles S3/S4 recording commands. */
        RemoteMode_5msCallback();
        SavePointMode_5msCallback();
    }
    else if(menu.mode == CAR_MODE_TRACK)
    {
        ReverseTrackMode_5msCallback();
    }
}

void Menu_Display(void)
{
    MenuPage_e current_page = menu.page;

    if((current_page != MENU_PAGE_REMOTE) &&
       (current_page != MENU_PAGE_TEST) &&
       (current_page != MENU_PAGE_SAVE_POINT))
    {
        current_page = MENU_PAGE_SYSTEM;
    }

    /*
     * 页面模板含有大量用于清屏的空格。若每 100 ms 都重画，空格和冒号会先覆盖
     * 动态数字，随后数字又覆盖回来，肉眼看到的就是字符反复闪烁。
     * 因此固定模板只在首次显示及页面切换时绘制一次。
     */
    if((g_display_page_valid == 0U) || (g_displayed_page != current_page))
    {
        if(current_page == MENU_PAGE_REMOTE)
        {
            Menu_DisplayPage(g_remote_page, menu.cursor_row);
        }
        else if(current_page == MENU_PAGE_TEST)
        {
            Menu_DisplayPage(g_test_page, menu.cursor_row);
        }
        else if(current_page == MENU_PAGE_SAVE_POINT)
        {
            Menu_DisplayPage(g_save_point_page, menu.cursor_row);
        }
        else
        {
            Menu_DisplayPage(g_system_page, menu.cursor_row);
        }
        g_displayed_page = current_page;
        g_display_page_valid = 1U;
    }

    if(current_page == MENU_PAGE_REMOTE)
    {
        menu_display_remote_data();
    }
    else if(current_page == MENU_PAGE_TEST)
    {
        menu_display_test_data();
    }
    else if(current_page == MENU_PAGE_SAVE_POINT)
    {
        menu_display_save_point_data();
    }
    else
    {
        menu_display_system_data();
    }
}

CarMode_e Menu_GetMode(void)
{
    return menu.mode;
}

const char *Menu_GetModeName(CarMode_e mode)
{
    if(mode >= CAR_MODE_COUNT) { return "UNKNOWN"; }
    return g_mode_names[mode];
}

void Menu_RegisterModeHooks(CarMode_e mode, const MenuModeHooks_t *hooks)
{
    if((mode >= CAR_MODE_COUNT) || (hooks == 0)) { return; }
    g_mode_hooks[mode] = *hooks;
}
