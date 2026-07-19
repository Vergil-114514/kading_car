#ifndef _SAVE_POINT_MODE_H_
#define _SAVE_POINT_MODE_H_

#include <stdint.h>

/* Physical keys S3/S4 map to LoRa protocol key[2]/key[3]. */
#define SAVE_POINT_MODE_START_KEY_INDEX       (2U)
#define SAVE_POINT_MODE_STOP_KEY_INDEX        (3U)
#define SAVE_POINT_MODE_SAMPLE_DISTANCE_M     (0.5f)
/* Demo defaults; the TCP IP/ports remain configurable in zf_device_wifi_spi.h. */
#define SAVE_POINT_MODE_WIFI_SSID             "SEEKFREE"
#define SAVE_POINT_MODE_WIFI_PASSWORD         "12345678"
/* SavePointMode_100msTask() converts this count to a 1 s retry interval. */
#define SAVE_POINT_MODE_WIFI_RETRY_TICKS      (10U)

typedef struct
{
    uint16_t point_count;
    float local_x_m;
    float local_y_m;
    float last_point_x_m;
    float last_point_y_m;
    float relative_yaw_deg;
    float distance_since_last_point_m;
    uint32_t wifi_session;
    uint16_t wifi_sent_count;
    uint16_t wifi_error_count;
    uint8_t recording;
    uint8_t full;
    uint8_t flash_saved;
    uint8_t flash_save_pending;
    uint8_t start_rejected;
    uint8_t wifi_initialized;
    uint8_t wifi_connected;
} SavePointModeStatus_t;

void SavePointMode_Init(void);
void SavePointMode_Enter(void);
void SavePointMode_Task(void);
void SavePointMode_100msTask(void);
void SavePointMode_5msCallback(void);
void SavePointMode_Exit(void);
void SavePointMode_GetStatus(SavePointModeStatus_t *status);

#endif
