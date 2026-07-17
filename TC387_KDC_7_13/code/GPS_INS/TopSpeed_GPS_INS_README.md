# TopSpeed 组合导航 TC387 移植说明

## 已移植的运行链

本移植提取的是 TopSpeed 实车代码中真正接通的 `GIE` 路径：

1. TAU1201 的有效 GPS 位置转换为局部平面坐标。
2. GPS 更新时，按当前航向和 `gps_delay_s` 对位置做延迟前推。
3. IMU963RA 以 5 ms 周期运行 Madgwick 6DOF 姿态解算。
4. 两次 GPS 更新之间，按航向和当前速度递推位置。
5. 编码器有效时优先使用有符号编码器速度；未接编码器时自动回退到 GPS 速度。

当 `CPU0_USE_GPS` 为 `0` 时，导航会从局部坐标 `(0, 0)` 启动，使用 IMU 航向和
后轮编码器有符号速度进行相对航位推算；此时坐标不具有绝对经纬度参考，累计误差也不会被 GPS 修正。

TopSpeed 原文件后半段的位置 KF、互补滤波和 CTRV 没有进入实际运行链，并且存在时间戳混用、除零、角度单位、脉冲累计和协方差更新错误，因此没有把这些实验分支标成可用代码移入 TC387。

## 当前接入点

- `cpu0_main.c` 通过 `CPU0_NAVIGATION_SOLUTION` 在 Mahony、Fusion 和
  GPS_INS 三种姿态/航向后端之间选择；三者共用编码器坐标递推层。
- `cpu0_main.c` 通过 `CPU0_USE_GPS` 选择 IMU+编码器惯导或 GPS/INS 融合，
  并在启用 GPS 时于主循环解析 GNSS。
- `isr.c` 的 `CCU61_CH0` 每 5 ms 调用一次导航更新。
- UART3 GNSS 接收中断继续使用逐飞库已有的 `gnss_uart_callback()`。
- PIT 优先级改为 43～46，避开模板中 UART6/UART8 的 30～33。
- UART3/11 RX 优先级提高到 50/51，高于导航 PIT；GNSS 回调会在 UART FIFO 内逐句处理，避免 NMEA 粘包。
- `zf_device_gnss.c` 按 TopSpeed 的实际用法只保留 10 Hz RMC 输出；坏帧会安全丢弃并恢复接收，不会卡死后续定位。GGA 因此默认关闭。
- TASKING Debug/Release 浮点模型均改为 `fastDouble`，避免经纬度在求局部坐标前被压成单精度。

## 三种解算后端选择

在 `user/cpu0_main.c` 中只修改 `CPU0_NAVIGATION_SOLUTION`：

```c
/* Mahony 6DOF */
#define CPU0_NAVIGATION_SOLUTION (NAVIGATION_SOLUTION_MAHONY)

/* 或 x-io Fusion AHRS */
#define CPU0_NAVIGATION_SOLUTION (NAVIGATION_SOLUTION_FUSION)

/* 或原 GPS_INS/Madgwick，工程默认值 */
#define CPU0_NAVIGATION_SOLUTION (NAVIGATION_SOLUTION_GPS_INS)
```

Mahony、Fusion 和 GPS_INS 在本工程中负责计算姿态与车辆航向。后轮编码器提供
有符号车速，统一位置层按航向递推局部 `X/Y`；`CPU0_USE_GPS=1` 时，TAU1201
再对局部位置和速度进行校正。因此切换后端不会改变菜单、控制器或坐标读取接口。

三种模式都以 5 ms 更新。Mahony 和 GPS_INS 使用加速度计与陀螺仪；当前 Fusion
接法同样采用无磁力计更新，以避免三种模式混用不同的磁场标定条件。上电期间车辆
必须静止，完成陀螺仪零偏标定后再运动。

GPS 启用时，首次有效 GPS 会自动成为局部坐标原点。也可以在
`TopSpeed_GPS_INS_PortInit(CPU0_USE_GPS, CPU0_NAVIGATION_SOLUTION)` 返回后、启动 PIT 前手动设置
（不能在 `PortInit()` 之前调用，初始化会清空适配层状态）：

```c
TopSpeed_GPS_INS_PortSetOrigin(39.1234567, 119.1234567);
```

重新以“下一帧有效 GPS”为原点：

```c
TopSpeed_GPS_INS_PortResetOrigin();
```

## 编码器接口

导航模块不绑定某一种编码器或电机板协议。速度单位必须为 `m/s`，向前为正、倒车为负。

如果已有轮速：

```c
TopSpeed_GPS_INS_PortEncoderSpeedUpdate(wheel_speed_mps);
```

如果得到的是某个采样周期内的增量计数：

```c
TopSpeed_GPS_INS_PortEncoderDeltaUpdate(delta_count,
                                        metre_per_count,
                                        sample_period_s);
```

编码器断线或数据无效时：

```c
TopSpeed_GPS_INS_PortEncoderInvalidate();
```

原 TopSpeed 外部速度板使用的标定值是 `0.009091 m/pulse`，但这里没有把它写死；请根据当前轮径、减速比、编码方式和实测距离重新标定。

GNSS 地速本身没有正负号。尚未接编码器时，模块会默认车辆沿车头方向前进；如果需要倒车导航，必须接入有符号编码器速度，否则倒车阶段的延迟补偿和位置外推方向会错误。

## 读取结果

```c
TopSpeed_GPS_INS_Output nav;

TopSpeed_GPS_INS_PortGetOutput(&nav);
if (nav.valid)
{
    // nav.position_m.x: 向东位置（m）
    // nav.position_m.y: 向北位置（m）
    // nav.heading_deg: 0°北，+90°东，顺时针为正
    // nav.speed_mps: 当前选用的有符号速度
}
```

GPS 启用时，`nav.valid` 需要已取得初始 GPS；GPS 禁用时，本地位置会在 `(0,0)` 初始化。
两种模式都还要求航向有效，并且 GPS 或编码器至少一个仍未超时。
`speed_source` 可判断当前使用编码器、GPS 还是无有效速度；`gps_age_s`、
`encoder_age_s` 和各类 `*_valid` 标志可用于故障处理。

## IMU 安装方向

TopSpeed 原代码的 yaw 是“左转为正”，而 GNSS 标准航向是“右转为正”。本移植对外统一为：

- `x` 向东，`y` 向北；
- 航向 `0°` 向北；
- 航向 `+90°` 向东，顺时针为正。

因此同 TopSpeed 安装方向下默认使用 `heading = -raw_yaw`。若模块方向或车体安装不同，可修改 `TopSpeed_GPS_INS_Port.h` 中的：

```c
#define TOPSPEED_GPS_INS_IMU_HEADING_SIGN        (-1.0f)
#define TOPSPEED_GPS_INS_IMU_HEADING_OFFSET_DEG  (0.0f)
```

也可以运行时调用：

```c
TopSpeed_GPS_INS_PortSetHeadingAlignment(sign, offset_deg);
```

该调用同样应放在 `TopSpeed_GPS_INS_PortInit()` 返回后；若要在首个 5 ms 周期起就使用正确航向，应在启动 PIT 前完成。

本方案与 TopSpeed 一样使用 6DOF 姿态解算，没有磁力计提供绝对北向。上电后原始 yaw 从 0° 起算，因此车辆若不是朝正北启动，必须把启动时的真实航向写入 `offset_deg`；也可以让车辆朝正北静止上电。该偏置只负责初始对北，运行中的转角仍由陀螺仪积分得到。

当前按 TopSpeed 车体安装假设将 `-AccY` 作为前向加速度。该加速度目前仅输出和预留，不参与速度积分，避免车辆俯仰和零偏造成速度发散。

启动时会在车辆静止条件下采集 100 个陀螺仪样本计算零偏。上电标定期间不要移动车辆。

## GPS 串口与引脚

本 TC387 工程保留当前逐飞库默认配置：

- TAU1201：UART3；
- MCU TX：P15.6；
- MCU RX：P15.7。

TopSpeed 原工程把 GPS 改到了 UART11（P21.5/P21.2），并把 UART3 留给电机/编码器板。如果你的实车完全沿用 TopSpeed 接线，需要同步修改 `zf_device_gnss.h` 的 UART/引脚宏，并把 `gnss_uart_callback()` 从 UART3 RX ISR 移到 UART11 RX ISR。当前编码器适配层本身不占用任何 UART。

## 可调参数

核心默认参数位于 `TopSpeed_GPS_INS_GetDefaultConfig()`：

- GPS 数据延迟：0.30 s；
- GPS 速度超时：0.50 s；
- 编码器速度超时：0.20 s；
- 最大绝对速度：30 m/s；
- GPS 跳点门限：默认关闭（0 m）。

硬件适配参数位于 `TopSpeed_GPS_INS_Port.h`，包括 5 ms 周期、IMU 零偏采样数、航向符号和航向偏置。

## 工程刷新

AURIX Development Studio 导入或新增源码后，请先对工程执行 **Refresh**，确认 `code/TopSpeed_GPS_INS.c` 和 `code/TopSpeed_GPS_INS_Port.c` 已进入构建，再编译。
