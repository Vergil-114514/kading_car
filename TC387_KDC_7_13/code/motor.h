#ifndef _MOTOR_H
#define _MOTOR_H

#include "Doro_driver_pwm_comp.h"
#include "zf_common_headfile.h"

#define ENABLE  (1)
#define DISABLE (0)

#define GO start()

#define MOTOR_PWM_FREQ_HZ       (10000u)
#define MOTOR_CONTROL_HZ        (200.0f)
#define MOTOR_CONTROL_DT        (1.0f / MOTOR_CONTROL_HZ)
#define MOTOR_FULL_ANGLE_DEG    (360.0f)

/*
 * J2锛氬乏鍚庤疆椹卞姩鐢垫満銆�
 * 杩炴帴鍣� 3 鑴� P11.12 涓轰娇鑳斤紱1/5 鑴氬拰 2/6 鑴氬垎鍒粍鎴愪袱缁勫崐妗ヤ俊鍙枫��
 */
#define motor_L_DIS   P11_12
#define motor_L_PWM1  ATOM2_CH1_P11_2
#define motor_L_PWM1N ATOM2_CH2_P11_3
#define motor_L_PWM2  ATOM2_CH4_P11_9
#define motor_L_PWM2N ATOM2_CH5_P11_10

/*
 * J3锛氬彸鍚庤疆椹卞姩鐢垫満銆�
 * 杩炴帴鍣� 5 鑴� P20.10 涓轰娇鑳斤紱1/3 鑴氬拰 2/6 鑴氬垎鍒粍鎴愪袱缁勫崐妗ヤ俊鍙枫��
 */
#define motor_R_DIS   P20_10
#define motor_R_PWM1  ATOM1_CH4_P20_3
#define motor_R_PWM1N ATOM1_CH5_P20_9
#define motor_R_PWM2  ATOM3_CH6_P20_6
#define motor_R_PWM2N ATOM3_CH7_P20_7

/*
 * J1锛氬墠杞浆鍚戠數鏈恒��
 * 杩炴帴鍣� 3 鑴� P33.10 涓轰娇鑳斤紱1/5 鑴氬拰 2/6 鑴氬垎鍒粍鎴愪袱缁勫崐妗ヤ俊鍙枫��
 */
#define motor_T_DIS   P33_10
#define motor_T_PWM1  ATOM0_CH1_P33_9
#define motor_T_PWM1N ATOM0_CH2_P33_11
#define motor_T_PWM2  ATOM3_CH4_P33_12
#define motor_T_PWM2N ATOM3_CH5_P33_13

typedef struct
{
    pwm_channel_enum top_pin;
    pwm_channel_enum bottom_pin;
} MOTOR_PWM_HALF_BRIDGE;

typedef struct
{
    gpio_pin_enum enable_pin;
    MOTOR_PWM_HALF_BRIDGE phase_a;
    MOTOR_PWM_HALF_BRIDGE phase_b;
    uint32 pwm_freq_hz;
    float pwm_limit;
} MOTOR_DRIVER;

typedef struct
{
    float motor_speed;       // Legacy name: rear speed target, steering angle target (deg)
    float feedforward_pwm;   // Optional PWM feedforward, default 0
} MOTOR_DUTY;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float deadband;
} MOTOR_PID_GAIN;

typedef struct
{
    float error;
    float last_error;
    float last_measurement;
    float integral;
    float derivative;
    float output;
    uint8 initialized;
} MOTOR_PID_STATE;

typedef struct
{
    MOTOR_PID_GAIN gain;
    MOTOR_PID_STATE state;
} MOTOR_PID;

/**
 * @brief 鍗曚釜鍚庤疆閫熷害鐜湪涓�涓帶鍒跺懆鏈熷唴鐨勮緭鍏ャ��
 *
 * Ackermann銆侀仴鎺у拰娴嬭瘯妯″潡鍙礋璐ｅ～鍐欑洰鏍囬�熷害鍙� encoder.c 宸茬粡璁＄畻濂界殑
 * 閫熷害鍙嶉锛汸ID 鐘舵�佸拰 PWM 璁＄畻鍏ㄩ儴鐢� motor.c 绠＄悊銆�
 */
typedef struct
{
    float target_rad_s;       /**< 鐩爣杞�燂紝鍗曚綅 rad/s銆� */
    float measured_rad_s;     /**< encoder.c 璁＄畻鍑虹殑瀹為檯杞�燂紝鍗曚綅 rad/s銆� */
    float feedforward_pwm;    /**< 鍙�夊墠棣� PWM锛屼笉浣跨敤鏃跺～ 0銆� */
    float output_sign;        /**< 鐢垫満杈撳嚭鏂瑰悜锛岃礋鏁拌〃绀哄弽鐩革紝闈炶礋鏁拌〃绀哄悓鐩搞�� */
    float pwm_limit;          /**< 鏈杩愯鐨勫畨鍏ㄩ檺骞咃紱<=0 鏃朵娇鐢� PID 鍙傛暟鍐呴檺骞呫�� */
    uint8 enabled;            /**< 0锛氳杞緭鍑轰负 0 骞跺浣� PID锛涢潪 0锛氭墽琛岄棴鐜�� */
} MOTOR_REAR_SPEED_INPUT;

/** @brief 鍗曚釜鍚庤疆閫熷害鐜渶杩戜竴娆¤绠楃粨鏋滐紝渚涜彍鍗曞拰閬ユ祴鏄剧ず銆� */
typedef struct
{
    float target_rad_s;
    float measured_rad_s;
    float error_rad_s;
    float pwm;
} MOTOR_REAR_SPEED_STATUS;

typedef struct
{
    MOTOR_PID position;
    MOTOR_PID speed;
    float target_angle_deg;
    float target_speed_dps;
    float angle_deg;
    float speed_dps;
} MOTOR_CASCADE_PID;

extern MOTOR_DUTY motor_L_duty;
extern MOTOR_DUTY motor_R_duty;
extern MOTOR_DUTY servo_duty;
extern MOTOR_CASCADE_PID servo_pid;
extern MOTOR_PID servo_position_pid;

/**
 * @brief 璇诲彇 motor.c 涓粺涓�閰嶇疆鐨勪笁璺數鏈洪粯璁� PID 鍙傛暟銆�
 * @note  浠绘剰杈撳嚭鎸囬拡鍙互浼� 0锛涙祴璇曟ā寮忓拰闃垮厠鏇兼帶鍒跺潎浠庤繖閲屽彇寰楅粯璁ゅ�笺��
 */
void Motor_get_default_pid_gains(MOTOR_PID_GAIN *steering_position_gain,
                                 MOTOR_PID_GAIN *left_rear_speed_gain,
                                 MOTOR_PID_GAIN *right_rear_speed_gain);

/** 璁剧疆 motor.c 鍐呭疄闄呬娇鐢ㄧ殑宸﹀彸鍚庤疆閫熷害 PID 鍙傛暟锛屽苟娓呯┖鏃х姸鎬併�� */
void Motor_set_rear_speed_pid_gains(const MOTOR_PID_GAIN *left_gain,
                                    const MOTOR_PID_GAIN *right_gain);

/** 璇诲彇 motor.c 鍐呭綋鍓嶇敓鏁堢殑宸﹀彸鍚庤疆閫熷害 PID 鍙傛暟銆� */
void Motor_get_rear_speed_pid_gains(MOTOR_PID_GAIN *left_gain,
                                    MOTOR_PID_GAIN *right_gain);

/** 娓呯┖宸﹀彸鍚庤疆閫熷害 PID 鐨勭Н鍒嗐�佸井鍒嗗拰鍘嗗彶璇樊锛屼笉淇敼鍙傛暟銆� */
void Motor_rear_speed_pid_reset(void);

/**
 * @brief 鎵ц涓�娆″乏鍙冲悗杞嫭绔嬮�熷害 PID锛屽苟鐩存帴鍐欏叆涓よ矾 PWM銆�
 * @param left  宸﹀悗杞洰鏍囥�乪ncoder.c 閫熷害鍙嶉鍙婂畨鍏ㄩ檺鍒躲��
 * @param right 鍙冲悗杞洰鏍囥�乪ncoder.c 閫熷害鍙嶉鍙婂畨鍏ㄩ檺鍒躲��
 * @param allow_active_braking 0 鏃剁姝㈣緭鍑轰笌鐩爣鏂瑰悜鐩稿弽鐨勫埗鍔ㄥ姏鐭┿��
 * @note 鍥哄畾浠� MOTOR_CONTROL_DT锛�5 ms锛夎绠楋紝璋冪敤鍛ㄦ湡搴斾负 200 Hz銆�
 */
void Motor_rear_speed_control(const MOTOR_REAR_SPEED_INPUT *left,
                              const MOTOR_REAR_SPEED_INPUT *right,
                              uint8 allow_active_braking);

/** 璇诲彇宸﹀彸鍚庤疆閫熷害鐜渶杩戜竴娆＄洰鏍囥�佸弽棣堛�佽宸拰瀹為檯 PWM銆� */
void Motor_get_rear_speed_status(MOTOR_REAR_SPEED_STATUS *left_status,
                                 MOTOR_REAR_SPEED_STATUS *right_status);

/**
 * @brief  閸掓繂顫愰崠鏍т箯閸欏啿鎮楁潪顔炬暩閺堟椽鈹嶉崝銊ｏ拷锟�
 */
void motor_init(void);

/**
 * @brief 鍒濆鍖栬浆鍚戠數鏈洪┍鍔ㄥ拰杞悜 PID銆�
 * @note  缂栫爜鍣ㄧ敱 encoder.c 鐙珛鍒濆鍖栧拰閲囬泦锛屾湰鍑芥暟鍙鍙栧叾璁＄畻缁撴灉銆�
 */
void Servo_init(void);

/**
 * @brief  娴ｈ儻鍏樺锕�褰搁崥搴ょ枂閻㈠灚婧�閸滃矁娴嗛崥鎴犳暩閺堟椽鈹嶉崝銊ｏ拷锟�
 */
void start(void);

void Motor_set_pwm(float left_pwm, float right_pwm);

/* Enable only the requested driver channels. Intended for safe bench tests;
 * PWM commands must be set to zero before changing these enables. */
void Motor_enable_channels(uint8 left_enable,
                           uint8 right_enable,
                           uint8 steering_enable);

/** Stop every PWM output and disable all three motor drivers. */
void Motor_stop_all(void);

/**
 * @brief  閹笛嗩攽娑擄拷濞喡ゆ祮閸氭垹鏁搁張杞拌缁撅拷 PID 閹貉冨煑閵嗭拷
 * @param  duty 閻╊喗鐖ｆ潏鎾冲弳閿涘苯鍙炬稉锟� motor_speed 鐞涖劎銇氶惄顔界垼鐟欐帒瀹抽敍灞藉礋娴ｏ拷 deg閵嗭拷
 */
void Servo_control(MOTOR_DUTY *duty);

/**
 * @brief Direct steering position PID. The output is signed PWM.
 * @note  Call at MOTOR_CONTROL_HZ. This is the steering loop used by the
 *        Ackermann controller; no current feedback is required.
 */
void Servo_position_control(MOTOR_DUTY *duty);

/**
 * @brief  鐠佸墽鐤嗘潪顒�鎮滈悽鍨簚閻╊喗鐖ｇ憴鎺戝閵嗭拷
 * @param  angle_deg 閻╊喗鐖ｇ憴鎺戝閿涘苯宕熸担锟� deg閵嗭拷
 */
void Servo_set_angle(float angle_deg);

/**
 * @brief  鐠佸墽鐤嗘潪顒�鎮滈悽鍨簚娴ｅ秶鐤嗛悳顖氭嫲闁喎瀹抽悳锟� PID 閸欏倹鏆熼妴锟�
 * @param  position_gain 娴ｅ秶鐤嗛悳顖氬棘閺佸府绱濇导锟� 0 鐞涖劎銇氭稉宥勬叏閺�骞匡拷锟�
 * @param  speed_gain    闁喎瀹抽悳顖氬棘閺佸府绱濇导锟� 0 鐞涖劎銇氭稉宥勬叏閺�骞匡拷锟�
 */
void Servo_set_pid_gain(const MOTOR_PID_GAIN *position_gain, const MOTOR_PID_GAIN *speed_gain);

/** Configure the direct steering position PID used by Servo_position_control. */
void Servo_set_position_pid_gain(const MOTOR_PID_GAIN *position_gain);

/**
 * @brief 杩斿洖鏈�鏂扮殑杞悜缂栫爜鍣ㄧ疮璁¤搴︼紝鍗曚綅涓哄害銆�
 * @note  瑙掑害浠ユ湰娆� Servo_init 鏃剁殑浣嶇疆涓洪浂鐐广��
 */
float Servo_get_angle(void);

/**
 * @brief  濞撳懐鈹栨潪顒�鎮滈悽鍨簚娑撹尙楠� PID 鏉╂劘顢戦悩鑸碉拷浣碉拷锟�
 */
void Servo_pid_reset(void);

#endif
