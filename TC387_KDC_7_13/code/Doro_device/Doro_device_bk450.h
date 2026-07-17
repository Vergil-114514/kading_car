#ifndef _bk450_gnss_h_
#define _bk450_gnss_h_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

#define BK450_GNSS_UART        (UART_3)
#define BK450_GNSS_RX_PIN      (UART3_TX_P15_6)
#define BK450_GNSS_TX_PIN      (UART3_RX_P15_7)
#define BK450_GNSS_BAUDRATE    (115200)

typedef struct
{
    uint16 year;
    uint8  month;
    uint8  day;
    uint8  hour;
    uint8  minute;
    uint8  second;
}bk450_gnss_time_struct;

typedef struct
{
    bk450_gnss_time_struct time;

    uint8  fix_valid;
    uint8  fix_quality;
    uint8  satellite_used;

    double latitude;
    double  longitude;
    int8   ns;
    int8   ew;

    float  speed_kmh;
    float  course_deg;
    float  height_m;
}bk450_gnss_info_struct;

extern volatile uint8 bk450_gnss_flag;
extern bk450_gnss_info_struct bk450_gnss;

void bk450_gnss_init(void);
uint8 bk450_gnss_consume(void);
void bk450_gnss_uart_callback(void);
void bk450_gnss_show_ips200(uint16 x, uint16 y);

#endif
