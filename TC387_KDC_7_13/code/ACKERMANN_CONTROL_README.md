# TC387 阿克曼三电机控制说明

本控制器对应以下结构：

- 一个前轮转向电机，带绝对位置编码器；
- 左、右后轮各一个驱动电机，分别带转速编码器；
- 无电流采样，全部闭环只依赖编码器；
- 控制周期为 5 ms，即 200 Hz。

控制链为：

```text
横向误差 + 航向误差 + 目标车速
              ↓
           Stanley
              ↓
       等效前轮目标转角
              ↓
        阿克曼电子差速
          ↙         ↘
左后轮速度 PID    右后轮速度 PID

等效前轮目标转角 → 编码器绝对角度 → 转向位置 PID
```

## 已填写的车辆参数

在 `ackermann_control.h` 中：

```c
#define ACKERMANN_WHEELBASE_M             (0.58f)
#define ACKERMANN_TRACK_WIDTH_M           (0.60f)
#define ACKERMANN_MAX_WHEEL_SPEED_MPS     (3.0f)
```

后轮编码器、电子差速、速度 PID 和遥测统一使用 m/s。编码器按实车每米计数标定，控制器不需要轮胎直径或半径。

左转为正。横向误差为正表示目标路径在车辆前轴中心的左侧；航向误差为 `路径航向 - 车辆航向`，输入单位为弧度。

## 上电前必须修改

### 1. 三个电机引脚

已经按照原理图配置，并约定 J1 为左后轮电机、J2 为右后轮电机、J3 为转向电机：

| 电机 | 使能 | A1 | A2 | B1 | B2 |
| --- | --- | --- | --- | --- | --- |
| 左后轮 J1 | P33.10 | P33.9 | P33.11 | P33.12 | P33.13 |
| 右后轮 J2 | P11.12 | P11.2 | P11.3 | P11.9 | P11.10 |
| 转向 J3 | P20.10 | P20.3 | P20.9 | P20.6 | P20.7 |

PWM 通道已经在 ATOM0～ATOM3 之间分配，避免三个电机占用同一底层通道。

### 2. 配置两个后轮编码器

已经按照原理图配置为：

| 编码器 | A | B/Dir | GPT12 |
| --- | --- | --- | --- |
| 左后轮 P3 | P02.6 | P02.7 | TIM3 |
| 右后轮 P4 | P02.8 | P00.9 | TIM4 |

当前左右后轮按“脉冲 + 方向”模式初始化。如果实际硬件输出的是 A/B 正交信号，需把对应后轮改为 `encoder_quad_init()`。

仍需确认或标定：

- `encoder.c` 中每米计数是否与实车一致；
- 如果轮速符号相反，修改对应的 `ENCODER_SIGN`；
- 如果正 PWM 使车辆后退，修改对应的 `MOTOR_SIGN`。

当前实测标定为：

```text
ENCODER_B:     6684 / 2 m = 3342 count/m
ENCODER_BL/BR: 5130 / 2 m = 2565 count/m
```

上表是绝对值。当前硬件前进时计数为负，因此 `encoder.c` 中使用 `-3342` 和 `-2565 count/m`，使车辆前进速度最终为正 m/s。

重新标定时应让车辆在地面直线行驶已知距离，用累计计数除以实际距离，这会同时包含减速比、编码器倍频和轮胎有效滚动周长。

### 3. 标定转向中心和传动比

`ACKERMANN_PORT_STEERING_CENTER_DEG = -1.0f` 表示初始化时把当前位置作为直行中心。因此上电时必须先把前轮摆正。

转向传动比的定义为：

```text
转向编码器变化角度 ÷ 实际前轮等效转角变化角度
```

将结果写入 `ACKERMANN_PORT_STEERING_RATIO`。如果控制器要求左转但车轮向右，修改 `ACKERMANN_PORT_STEERING_SIGN`。

### 4. 硬件接入状态

`ACKERMANN_PORT_ENABLE` 已设置为 `1U`。工程使用 `CCU60_CH0` 产生独立的 5 ms 电机控制中断，原有 GPS/INS 仍使用 `CCU61_CH0`。控制器初始化后保持禁用，只有应用显式调用 `Ackermann_control_enable(ZF_TRUE)` 才会驱动电机。

## 路径模块调用方法

路径模块需要持续刷新误差和目标车速；默认命令看门狗为 0.2 s，超过时间没有新命令会停止后轮并回正转向。

```c
/* 初始化完成且确认车辆周围安全后调用。重复调用 enable 也是安全的。 */
Ackermann_control_enable(ZF_TRUE);

/* 路径解算每次更新时调用，heading_error_deg 需转换成弧度。 */
Ackermann_set_path_error(cross_track_error_m,
                         heading_error_deg * ACKERMANN_DEG_TO_RAD,
                         target_speed_mps);

/* 路径丢失时立即调用。 */
Ackermann_invalidate_path();

/* 急停或退出自动驾驶时调用。 */
Ackermann_control_enable(ZF_FALSE);
```

可以使用 `Ackermann_get_telemetry()` 读取目标轮速、实测轮速、PWM、目标转角和看门狗状态，送到串口示波器观察。

## 推荐调参顺序

第一次调试时必须架空驱动轮，并把后轮目标速度限制在较低值。

1. 先只检查三个电机和三个编码器的正负方向，不做 PID 调参。
2. 锁定转向目标为直行中心，只调转向位置 PID。先令 `ki=0, kd=0`，增加 `kp` 到能快速回中但不持续振荡，再少量增加 `kd`；最后仅在存在静差时增加 `ki`。
3. 将 Stanley 误差置零，分别调左、右后轮速度 PID。先令 `ki=0, kd=0`，增加 `kp`；再增加 `ki` 消除负载静差。编码器速度噪声较大时不要急于加入 `kd`。
4. 检查电子差速：左转时左目标轮速必须小于右目标轮速，右转相反。
5. 最后从低速开始调 Stanley。摆动明显时减小 `stanley_gain` 或增大 `stanley_softening_speed_mps`；收敛太慢时反向调整。

默认 PID 只是安全的起调值，不是最终参数。真实值会明显受到电机电压、减速比、整车质量、轮胎和 PWM 驱动方式影响。

默认 `allow_active_braking = 0`，后轮 PID 不会为了减速而主动施加反向电压。这对没有电流采样的初次调试更安全；确认驱动器和电机能承受再生/反向制动电流后，才考虑将其打开。
