#ifndef SAVE_POINT_MODE_HOST_STUBS_H
#define SAVE_POINT_MODE_HOST_STUBS_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define _zf_common_headfile_h_
#define _FLASH_H

#define FLASH_IMU_POINT_CAPACITY    (1024U)
#define TOPSPEED_GPS_INS_PERIOD_S   (0.005f)
#define WIFI_SPI_AUTO_CONNECT       (0)
#define WIFI_SPI_TARGET_IP          "192.168.137.1"
#define WIFI_SPI_TARGET_PORT        "8086"
#define WIFI_SPI_LOCAL_PORT         "6666"

typedef uint8_t uint8;
typedef uint32_t uint32;

typedef struct
{
    int16_t joystick[4];
    uint8_t key[4];
    uint8_t switch_key[4];
    uint32_t sequence;
    uint16_t age_ms;
    uint8_t link_ok;
} LoraRemoteState_t;

typedef struct
{
    float x;
    float y;
} TopSpeed_GPS_INS_Point;

typedef struct
{
    TopSpeed_GPS_INS_Point position_m;
    float encoder_speed_mps;
    uint8_t initialized;
    uint8_t encoder_valid;
} TopSpeed_GPS_INS_Output;

typedef struct
{
    float heading_deg;
    uint8_t valid;
} TopSpeed_GPS_INS_IMU_Output;

typedef struct
{
    TopSpeed_GPS_INS_Point position_m;
    float relative_yaw_deg;
    float encoder_speed_mps;
    float distance_m;
    uint32_t reset_sequence;
    uint8_t initialized;
    uint8_t heading_valid;
    uint8_t encoder_valid;
    uint8_t valid;
} TopSpeed_GPS_INS_LocalOdometryOutput;

extern uint16_t IMU_savenum;
extern float IMU_X[FLASH_IMU_POINT_CAPACITY];
extern float IMU_Y[FLASH_IMU_POINT_CAPACITY];

void LoraRemote_GetState(LoraRemoteState_t *state);
void TopSpeed_GPS_INS_PortGetOutput(TopSpeed_GPS_INS_Output *output);
void TopSpeed_GPS_INS_PortGetIMUOutput(TopSpeed_GPS_INS_IMU_Output *output);
uint32_t TopSpeed_GPS_INS_PortRequestLocalOdometryReset(void);
void TopSpeed_GPS_INS_PortGetLocalOdometryOutput(
    TopSpeed_GPS_INS_LocalOdometryOutput *output);
void IMU_POINT_SAVE(void);
void DATA_SAVE(void);
uint8 wifi_spi_init(char *wifi_ssid, char *pass_word);
uint8 wifi_spi_socket_connect(char *transport_type,
                              char *ip_addr,
                              char *port,
                              char *local_port);
uint32 wifi_spi_send_buffer(const uint8 *buffer, uint32 length);
int save_point_test_printf(const char *format, ...);

#define printf save_point_test_printf

#endif
