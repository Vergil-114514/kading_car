#include <Doro_device_bk450.h>
#include "zf_common_function.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_device_ips200.h"

#include <string.h>

#define BK450_LINE_BUFFER_SIZE      (128)
#define BK450_FIELD_BUFFER_SIZE     (20)

volatile uint8 bk450_gnss_flag = 0;
bk450_gnss_info_struct bk450_gnss;

static uint8 bk450_line_buffer[BK450_LINE_BUFFER_SIZE];
static uint8 bk450_line_index = 0;
static uint8 bk450_started = 0;
static uint8 bk450_pending_line_buffer[BK450_LINE_BUFFER_SIZE];
static volatile uint8 bk450_pending_line_ready = 0;
static volatile uint8 bk450_pending_line_length = 0;

static uint8 hex_to_value(char ch)
{
    if(('0' <= ch) && ('9' >= ch)) return (uint8)(ch - '0');
    if(('A' <= ch) && ('F' >= ch)) return (uint8)(ch - 'A' + 10);
    if(('a' <= ch) && ('f' >= ch)) return (uint8)(ch - 'a' + 10);
    return 0xFF;
}

static uint8 nmea_check(const char *line)
{
    uint8 checksum = 0;
    uint8 origin = 0;
    uint16 i = 0;

    if('$' != line[0])
    {
        return 0;
    }

    for(i = 1; ('\0' != line[i]) && ('*' != line[i]); i ++)
    {
        checksum ^= (uint8)line[i];
    }

    if(('*' != line[i]) || (0xFF == hex_to_value(line[i + 1])) || (0xFF == hex_to_value(line[i + 2])))
    {
        return 0;
    }

    origin = (uint8)((hex_to_value(line[i + 1]) << 4) | hex_to_value(line[i + 2]));
    return (uint8)(checksum == origin);
}

static uint8 field_copy(const char *line, uint8 field, char *out, uint8 out_size)
{
    uint8 current = 0;
    uint8 len = 0;
    uint16 i = 0;

    if(0 == out_size)
    {
        return 0;
    }

    out[0] = '\0';
    for(i = 0; ('\0' != line[i]) && ('*' != line[i]) && ('\r' != line[i]) && ('\n' != line[i]); i ++)
    {
        if('$' == line[i])
        {
            continue;
        }

        if(',' == line[i])
        {
            if(current == field)
            {
                break;
            }
            current ++;
            continue;
        }

        if(current == field)
        {
            if(len < (out_size - 1))
            {
                out[len ++] = line[i];
            }
        }
    }

    out[len] = '\0';
    return len;
}

static int32 field_int(const char *line, uint8 field)
{
    char buffer[BK450_FIELD_BUFFER_SIZE];
    field_copy(line, field, buffer, sizeof(buffer));
    return func_str_to_int(buffer);
}

static float field_float(const char *line, uint8 field)
{
    char buffer[BK450_FIELD_BUFFER_SIZE];
    field_copy(line, field, buffer, sizeof(buffer));
    return func_str_to_float(buffer);
}

static double field_double(const char *line, uint8 field)
{
    char buffer[BK450_FIELD_BUFFER_SIZE];
    field_copy(line, field, buffer, sizeof(buffer));
    return func_str_to_double(buffer);
}

static char field_char(const char *line, uint8 field)
{
    char buffer[4];
    field_copy(line, field, buffer, sizeof(buffer));
    return buffer[0];
}

static double nmea_to_degree(double nmea_value)
{
    int32 degree = (int32)(nmea_value / 100.0);
    double minute = nmea_value - degree * 100.0;
    return degree + minute / 60.0;
}

static void utc_to_beijing(bk450_gnss_time_struct *time)
{
    uint8 day_num = 31;

    time->hour += 8;
    if(time->hour < 24)
    {
        return;
    }

    time->hour -= 24;
    time->day ++;

    if(2 == time->month)
    {
        day_num = 28;
        if(((0 == time->year % 4) && (0 != time->year % 100)) || (0 == time->year % 400))
        {
            day_num = 29;
        }
    }
    else if((4 == time->month) || (6 == time->month) || (9 == time->month) || (11 == time->month))
    {
        day_num = 30;
    }

    if(time->day <= day_num)
    {
        return;
    }

    time->day = 1;
    time->month ++;
    if(12 < time->month)
    {
        time->month = 1;
        time->year ++;
    }
}

static void parse_time_date(const char *line)
{
    char time_buf[BK450_FIELD_BUFFER_SIZE];
    char date_buf[BK450_FIELD_BUFFER_SIZE];

    field_copy(line, 1, time_buf, sizeof(time_buf));
    field_copy(line, 9, date_buf, sizeof(date_buf));

    if(strlen(time_buf) >= 6)
    {
        bk450_gnss.time.hour   = (uint8)((time_buf[0] - '0') * 10 + (time_buf[1] - '0'));
        bk450_gnss.time.minute = (uint8)((time_buf[2] - '0') * 10 + (time_buf[3] - '0'));
        bk450_gnss.time.second = (uint8)((time_buf[4] - '0') * 10 + (time_buf[5] - '0'));
    }

    if(strlen(date_buf) >= 6)
    {
        bk450_gnss.time.day   = (uint8)((date_buf[0] - '0') * 10 + (date_buf[1] - '0'));
        bk450_gnss.time.month = (uint8)((date_buf[2] - '0') * 10 + (date_buf[3] - '0'));
        bk450_gnss.time.year  = (uint16)((date_buf[4] - '0') * 10 + (date_buf[5] - '0') + 2000);
        utc_to_beijing(&bk450_gnss.time);
    }
}

static void parse_rmc(const char *line)
{
    char state = field_char(line, 2);
    double latitude = field_double(line, 3);
    double longitude = field_double(line, 5);

    parse_time_date(line);

    bk450_gnss.fix_valid = (uint8)(('A' == state) || ('D' == state));
    if(0 == bk450_gnss.fix_valid)
    {
        return;
    }

    bk450_gnss.ns = (int8)field_char(line, 4);
    bk450_gnss.ew = (int8)field_char(line, 6);
    bk450_gnss.latitude = nmea_to_degree(latitude);
    bk450_gnss.longitude = nmea_to_degree(longitude);

    if('S' == bk450_gnss.ns) bk450_gnss.latitude = -bk450_gnss.latitude;
    if('W' == bk450_gnss.ew) bk450_gnss.longitude = -bk450_gnss.longitude;

    bk450_gnss.speed_kmh = field_float(line, 7) * 1.852f;
    bk450_gnss.course_deg = field_float(line, 8);
}

static void parse_gga(const char *line)
{
    double latitude = field_double(line, 2);
    double longitude = field_double(line, 4);

    bk450_gnss.fix_quality = (uint8)field_int(line, 6);
    bk450_gnss.satellite_used = (uint8)field_int(line, 7);
    bk450_gnss.height_m = field_float(line, 9);

    if(0 != bk450_gnss.fix_quality)
    {
        bk450_gnss.fix_valid = 1;
        bk450_gnss.ns = (int8)field_char(line, 3);
        bk450_gnss.ew = (int8)field_char(line, 5);
        bk450_gnss.latitude = nmea_to_degree(latitude);
        bk450_gnss.longitude = nmea_to_degree(longitude);
        if('S' == bk450_gnss.ns) bk450_gnss.latitude = -bk450_gnss.latitude;
        if('W' == bk450_gnss.ew) bk450_gnss.longitude = -bk450_gnss.longitude;
    }
}

static void parse_vtg(const char *line)
{
    bk450_gnss.course_deg = field_float(line, 1);
    bk450_gnss.speed_kmh = field_float(line, 7);
}

static void parse_line(const char *line)
{
    if(0 == nmea_check(line))
    {
        return;
    }

    if(0 == strncmp(&line[3], "RMC", 3))
    {
        parse_rmc(line);
        bk450_gnss_flag = 1;
    }
    else if(0 == strncmp(&line[3], "GGA", 3))
    {
        parse_gga(line);
        bk450_gnss_flag = 1;
    }
    else if(0 == strncmp(&line[3], "VTG", 3))
    {
        parse_vtg(line);
        bk450_gnss_flag = 1;
    }
}

void bk450_gnss_init(void)
{
    memset(&bk450_gnss, 0, sizeof(bk450_gnss));
    bk450_line_index = 0;
    bk450_started = 0;
    bk450_gnss_flag = 0;
    bk450_pending_line_ready = 0;
    bk450_pending_line_length = 0;

    system_delay_ms(500);
    uart_init(BK450_GNSS_UART, BK450_GNSS_BAUDRATE, BK450_GNSS_RX_PIN, BK450_GNSS_TX_PIN);
    uart_rx_interrupt(BK450_GNSS_UART, 1);
}

uint8 bk450_gnss_consume(void)
{
    uint8 has_pending = 0u;
    uint8 line_length = 0u;
    uint32 interrupt_state;
    uint8 line[BK450_LINE_BUFFER_SIZE];

    interrupt_state = interrupt_global_disable();
    if(0u != bk450_pending_line_ready)
    {
        line_length = bk450_pending_line_length;
        if(line_length >= BK450_LINE_BUFFER_SIZE)
        {
            line_length = BK450_LINE_BUFFER_SIZE - 1u;
        }
        memcpy(line, bk450_pending_line_buffer, (uint32)line_length + 1u);
        bk450_pending_line_ready = 0u;
        bk450_pending_line_length = 0u;
        has_pending = 1u;
    }
    interrupt_global_enable(interrupt_state);

    if(0u == has_pending)
    {
        return 1u;
    }

    line[line_length] = '\0';
    bk450_gnss_flag = 0u;
    parse_line((const char *)line);

    return (0u != bk450_gnss_flag) ? 0u : 2u;
}

void bk450_gnss_uart_callback(void)
{
    uint8 data = 0;

    while(uart_query_byte(BK450_GNSS_UART, &data))
    {
        if('$' == data)
        {
            bk450_started = 1;
            bk450_line_index = 0;
        }

        if(0 == bk450_started)
        {
            continue;
        }

        if(bk450_line_index < (BK450_LINE_BUFFER_SIZE - 1))
        {
            bk450_line_buffer[bk450_line_index ++] = data;
        }
        else
        {
            bk450_started = 0;
            bk450_line_index = 0;
            continue;
        }

        if('\n' == data)
        {
            bk450_line_buffer[bk450_line_index] = '\0';
            if(0u == bk450_pending_line_ready)
            {
                memcpy(bk450_pending_line_buffer, bk450_line_buffer, (uint32)bk450_line_index + 1u);
                bk450_pending_line_length = bk450_line_index;
                bk450_pending_line_ready = 1u;
            }
            bk450_started = 0;
            bk450_line_index = 0;
        }
    }
}

void bk450_gnss_show_ips200(uint16 x, uint16 y)
{
    ips200_show_string(x, y + 16 * 0, "BK450 GNSS");
    ips200_show_string(x, y + 16 * 1, "DATE:");
    ips200_show_uint  (x + 48,  y + 16 * 1, bk450_gnss.time.year, 4);
    ips200_show_uint  (x + 96,  y + 16 * 1, bk450_gnss.time.month, 2);
    ips200_show_uint  (x + 128, y + 16 * 1, bk450_gnss.time.day, 2);

    ips200_show_string(x, y + 16 * 2, "TIME:");
    ips200_show_uint  (x + 48,  y + 16 * 2, bk450_gnss.time.hour, 2);
    ips200_show_uint  (x + 80,  y + 16 * 2, bk450_gnss.time.minute, 2);
    ips200_show_uint  (x + 112, y + 16 * 2, bk450_gnss.time.second, 2);

    ips200_show_string(x, y + 16 * 3, "FIX :");
    ips200_show_uint  (x + 48,  y + 16 * 3, bk450_gnss.fix_valid, 1);
    ips200_show_string(x + 72, y + 16 * 3, "Q:");
    ips200_show_uint  (x + 96, y + 16 * 3, bk450_gnss.fix_quality, 1);
    ips200_show_string(x + 120, y + 16 * 3, "SAT:");
    ips200_show_uint  (x + 160, y + 16 * 3, bk450_gnss.satellite_used, 2);

    ips200_show_string(x, y + 16 * 4, "LAT :");
    ips200_show_float (x + 48, y + 16 * 4, bk450_gnss.latitude, 4, 6);

    ips200_show_string(x, y + 16 * 5, "LON :");
    ips200_show_float (x + 48, y + 16 * 5, bk450_gnss.longitude, 4, 6);

    ips200_show_string(x, y + 16 * 6, "SPD :");
    ips200_show_float (x + 48, y + 16 * 6, bk450_gnss.speed_kmh, 4, 2);
    ips200_show_string(x + 120, y + 16 * 6, "km/h");

    ips200_show_string(x, y + 16 * 7, "COG :");
    ips200_show_float (x + 48, y + 16 * 7, bk450_gnss.course_deg, 3, 2);

    ips200_show_string(x, y + 16 * 8, "ALT :");
    ips200_show_float (x + 48, y + 16 * 8, bk450_gnss.height_m, 4, 2);
    ips200_show_string(x + 120, y + 16 * 8, "m");
}
