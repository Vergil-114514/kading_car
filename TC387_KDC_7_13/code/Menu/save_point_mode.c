#include "zf_common_headfile.h"
#include "save_point_mode.h"

#define SAVE_POINT_MODE_WIFI_FRAME_SIZE    (64U)
#define SAVE_POINT_MODE_ENDPOINT_EPSILON_M (0.0001f)

typedef struct
{
    volatile float local_x_m;
    volatile float local_y_m;
    volatile float relative_yaw_deg;
    volatile float distance_since_last_point_m;
    float odometry_distance_m;
    uint32_t local_odometry_reset_sequence;
    uint8_t previous_start_key;
    uint8_t previous_stop_key;
    volatile uint8_t enabled;
    volatile uint8_t recording;
    volatile uint8_t start_pending;
    volatile uint8_t full;
    volatile uint8_t flash_saved;
    volatile uint8_t start_rejected;
    volatile uint8_t save_pending;
    volatile uint8_t endpoint_finalized;
    volatile uint8_t print_pending;
    volatile uint8_t print_header_pending;
    volatile uint16_t print_point_count;
    uint16_t print_next_point_index;
    volatile uint16_t wifi_published_point_count;
    volatile uint16_t wifi_sent_count;
    volatile uint16_t wifi_error_count;
    volatile uint32_t wifi_session;
    uint32_t wifi_active_session;
    uint16_t wifi_next_point_index;
    volatile uint16_t wifi_endpoint_republish_index;
    volatile uint8_t wifi_endpoint_republish_pending;
    volatile uint8_t wifi_initialized;
    volatile uint8_t wifi_connected;
    volatile uint8_t wifi_retry_ticks;
} SavePointModeContext_t;

static SavePointModeContext_t g_save_point_mode;

static void save_point_wifi_record_error(void)
{
    if(g_save_point_mode.wifi_error_count < UINT16_MAX)
    {
        ++g_save_point_mode.wifi_error_count;
    }
}

static uint8_t save_point_wifi_prepare_task(void)
{
    const char *transport_type = "TCP";

    if(g_save_point_mode.wifi_retry_ticks != 0U)
    {
        return 0U;
    }

    if(g_save_point_mode.wifi_initialized == 0U)
    {
        if(wifi_spi_init(SAVE_POINT_MODE_WIFI_SSID,
                         SAVE_POINT_MODE_WIFI_PASSWORD) != 0U)
        {
            save_point_wifi_record_error();
            g_save_point_mode.wifi_retry_ticks =
                SAVE_POINT_MODE_WIFI_RETRY_TICKS;
            return 0U;
        }

        g_save_point_mode.wifi_initialized = 1U;
#if (WIFI_SPI_AUTO_CONNECT != 0)
        g_save_point_mode.wifi_connected = 1U;
#endif
        /* Perform at most one potentially blocking WiFi operation per task. */
        return 0U;
    }

    if(g_save_point_mode.wifi_connected != 0U)
    {
        return 1U;
    }

#if (WIFI_SPI_AUTO_CONNECT == 2)
    transport_type = "UDP";
#endif
    if(wifi_spi_socket_connect((char *)transport_type,
                               WIFI_SPI_TARGET_IP,
                               WIFI_SPI_TARGET_PORT,
                               WIFI_SPI_LOCAL_PORT) != 0U)
    {
        save_point_wifi_record_error();
        g_save_point_mode.wifi_retry_ticks =
            SAVE_POINT_MODE_WIFI_RETRY_TICKS;
        return 0U;
    }

    g_save_point_mode.wifi_connected = 1U;
    return 0U;
}

static void save_point_wifi_send_task(void)
{
    char frame[SAVE_POINT_MODE_WIFI_FRAME_SIZE];
    float point_x_m;
    float point_y_m;
    uint32_t session;
    uint16_t published_count;
    uint16_t point_index;
    uint8_t endpoint_republish;
    int frame_length;

    if(save_point_wifi_prepare_task() == 0U)
    {
        return;
    }

    session = g_save_point_mode.wifi_session;
    if(g_save_point_mode.wifi_active_session != session)
    {
        g_save_point_mode.wifi_active_session = session;
        g_save_point_mode.wifi_next_point_index = 0U;
        g_save_point_mode.wifi_sent_count = 0U;
    }

    published_count = g_save_point_mode.wifi_published_point_count;
    endpoint_republish = 0U;
    if(g_save_point_mode.wifi_next_point_index < published_count)
    {
        point_index = g_save_point_mode.wifi_next_point_index;
    }
    else if(g_save_point_mode.wifi_endpoint_republish_pending != 0U)
    {
        point_index = g_save_point_mode.wifi_endpoint_republish_index;
        endpoint_republish = 1U;
    }
    else
    {
        return;
    }

    point_x_m = IMU_X[point_index];
    point_y_m = IMU_Y[point_index];
    if(session != g_save_point_mode.wifi_session)
    {
        return;
    }

    /* PC line format: SP,<session>,<point index>,<local X>,<local Y>. */
    frame_length = snprintf(
        frame,
        sizeof(frame),
        "SP,%lu,%u,%.2f,%.2f\r\n",
        (unsigned long)session,
        (unsigned)point_index,
        (double)point_x_m,
        (double)point_y_m);
    if((frame_length <= 0) ||
       ((uint32_t)frame_length >= SAVE_POINT_MODE_WIFI_FRAME_SIZE))
    {
        save_point_wifi_record_error();
        return;
    }

    if(wifi_spi_send_buffer((const uint8 *)frame,
                            (uint32)frame_length) != 0U)
    {
        save_point_wifi_record_error();
        g_save_point_mode.wifi_connected = 0U;
        g_save_point_mode.wifi_retry_ticks =
            SAVE_POINT_MODE_WIFI_RETRY_TICKS;
        return;
    }

    if(endpoint_republish != 0U)
    {
        g_save_point_mode.wifi_endpoint_republish_pending = 0U;
    }
    else
    {
        ++g_save_point_mode.wifi_next_point_index;
        g_save_point_mode.wifi_sent_count =
            g_save_point_mode.wifi_next_point_index;
    }
}

static void save_point_print_task(void)
{
    uint16_t point_index;

    if(g_save_point_mode.print_pending == 0U)
    {
        return;
    }

    if(g_save_point_mode.print_header_pending != 0U)
    {
        printf("SAVE_POINT_BEGIN,%u\r\n",
               (unsigned)g_save_point_mode.print_point_count);
        g_save_point_mode.print_header_pending = 0U;
        return;
    }

    point_index = g_save_point_mode.print_next_point_index;
    if(point_index < g_save_point_mode.print_point_count)
    {
        printf("SAVE_POINT,%u,%.3f,%.3f\r\n",
               (unsigned)point_index,
               (double)IMU_X[point_index],
               (double)IMU_Y[point_index]);
        ++g_save_point_mode.print_next_point_index;
        return;
    }

    printf("SAVE_POINT_END,%u\r\n",
           (unsigned)g_save_point_mode.print_point_count);
    g_save_point_mode.print_pending = 0U;
}

static void save_point_queue_print(void)
{
    g_save_point_mode.print_point_count = IMU_savenum;
    g_save_point_mode.print_next_point_index = 0U;
    g_save_point_mode.print_header_pending = 1U;
    g_save_point_mode.print_pending = 1U;
}

static float save_point_clear_roundoff(float value)
{
    return (fabsf(value) < 0.00001f) ? 0.0f : value;
}

static void save_point_update_local_position(
    const TopSpeed_GPS_INS_LocalOdometryOutput *odometry)
{
    g_save_point_mode.local_x_m = save_point_clear_roundoff(
        odometry->position_m.x);
    g_save_point_mode.local_y_m = save_point_clear_roundoff(
        odometry->position_m.y);
    g_save_point_mode.relative_yaw_deg = odometry->relative_yaw_deg;
}

static void save_point_start(
    const TopSpeed_GPS_INS_LocalOdometryOutput *odometry)
{
    uint32_t reset_sequence;

    if((odometry->initialized == 0U) ||
       (odometry->heading_valid == 0U))
    {
        g_save_point_mode.start_rejected = 1U;
        return;
    }

    reset_sequence = TopSpeed_GPS_INS_PortRequestLocalOdometryReset();
    if(reset_sequence == 0U)
    {
        g_save_point_mode.start_rejected = 1U;
        return;
    }

    g_save_point_mode.local_odometry_reset_sequence = reset_sequence;
    g_save_point_mode.local_x_m = 0.0f;
    g_save_point_mode.local_y_m = 0.0f;
    g_save_point_mode.relative_yaw_deg = 0.0f;
    g_save_point_mode.distance_since_last_point_m = 0.0f;
    g_save_point_mode.odometry_distance_m = 0.0f;
    g_save_point_mode.full = 0U;
    g_save_point_mode.flash_saved = 0U;
    g_save_point_mode.start_rejected = 0U;
    g_save_point_mode.save_pending = 0U;
    g_save_point_mode.endpoint_finalized = 0U;
    g_save_point_mode.start_pending = 1U;
    g_save_point_mode.print_pending = 0U;
    g_save_point_mode.print_header_pending = 0U;
    g_save_point_mode.print_point_count = 0U;
    g_save_point_mode.print_next_point_index = 0U;

    IMU_savenum = 0U;
    g_save_point_mode.wifi_published_point_count = 0U;
    g_save_point_mode.wifi_sent_count = 0U;
    g_save_point_mode.wifi_endpoint_republish_pending = 0U;
    if(g_save_point_mode.wifi_session == UINT32_MAX)
    {
        g_save_point_mode.wifi_session = 1U;
    }
    else
    {
        ++g_save_point_mode.wifi_session;
    }
    IMU_X[0] = 0.0f;
    IMU_Y[0] = 0.0f;
    IMU_savenum = 1U;
    g_save_point_mode.wifi_published_point_count = IMU_savenum;
    g_save_point_mode.recording = 0U;
}

static void save_point_append_current(void)
{
    if(IMU_savenum >= FLASH_IMU_POINT_CAPACITY)
    {
        g_save_point_mode.recording = 0U;
        g_save_point_mode.full = 1U;
        return;
    }

    IMU_X[IMU_savenum] = g_save_point_mode.local_x_m;
    IMU_Y[IMU_savenum] = g_save_point_mode.local_y_m;
    ++IMU_savenum;
    g_save_point_mode.wifi_published_point_count = IMU_savenum;

    if(IMU_savenum >= FLASH_IMU_POINT_CAPACITY)
    {
        g_save_point_mode.recording = 0U;
        g_save_point_mode.full = 1U;
    }
}

static uint8_t save_point_current_is_new(void)
{
    float delta_x_m;
    float delta_y_m;

    if(IMU_savenum == 0U)
    {
        return 0U;
    }

    delta_x_m = g_save_point_mode.local_x_m - IMU_X[IMU_savenum - 1U];
    delta_y_m = g_save_point_mode.local_y_m - IMU_Y[IMU_savenum - 1U];
    return (uint8_t)(
        (fabsf(delta_x_m) > SAVE_POINT_MODE_ENDPOINT_EPSILON_M) ||
        (fabsf(delta_y_m) > SAVE_POINT_MODE_ENDPOINT_EPSILON_M));
}

static void save_point_finish_recording(void)
{
    uint16_t endpoint_index;

    g_save_point_mode.recording = 0U;
    g_save_point_mode.start_pending = 0U;
    if((IMU_savenum == 0U) ||
       (g_save_point_mode.endpoint_finalized != 0U))
    {
        return;
    }

    if(save_point_current_is_new() != 0U)
    {
        if(IMU_savenum < FLASH_IMU_POINT_CAPACITY)
        {
            save_point_append_current();
        }
        else
        {
            /* Capacity is fixed: keep the true endpoint as the final point. */
            endpoint_index = FLASH_IMU_POINT_CAPACITY - 1U;
            IMU_X[endpoint_index] = g_save_point_mode.local_x_m;
            IMU_Y[endpoint_index] = g_save_point_mode.local_y_m;
            g_save_point_mode.wifi_endpoint_republish_index = endpoint_index;
            g_save_point_mode.wifi_endpoint_republish_pending = 1U;
        }
    }

    g_save_point_mode.distance_since_last_point_m = 0.0f;
    g_save_point_mode.endpoint_finalized = 1U;
    save_point_queue_print();
}

void SavePointMode_Init(void)
{
    memset(&g_save_point_mode, 0, sizeof(g_save_point_mode));
    IMU_savenum = 0U;
}

void SavePointMode_Enter(void)
{
    g_save_point_mode.previous_start_key = 0U;
    g_save_point_mode.previous_stop_key = 0U;
    g_save_point_mode.enabled = 1U;
    g_save_point_mode.recording = 0U;
    g_save_point_mode.wifi_initialized = 0U;
    g_save_point_mode.wifi_connected = 0U;
    g_save_point_mode.wifi_retry_ticks = 0U;
}

void SavePointMode_Task(void)
{
    if(g_save_point_mode.enabled == 0U)
    {
        return;
    }

    if(g_save_point_mode.save_pending != 0U)
    {
        /* Point pages first; metadata count is committed only after they finish. */
        IMU_POINT_SAVE();
        DATA_SAVE();
        g_save_point_mode.flash_saved = 1U;
        g_save_point_mode.save_pending = 0U;
        return;
    }

    save_point_print_task();
    save_point_wifi_send_task();
}

void SavePointMode_100msTask(void)
{
    if((g_save_point_mode.enabled != 0U) &&
       (g_save_point_mode.wifi_retry_ticks != 0U))
    {
        --g_save_point_mode.wifi_retry_ticks;
    }
}

void SavePointMode_5msCallback(void)
{
    LoraRemoteState_t remote;
    TopSpeed_GPS_INS_LocalOdometryOutput odometry;
    uint8_t start_key;
    uint8_t stop_key;
    uint8_t combination_rising;

    if(g_save_point_mode.enabled == 0U)
    {
        return;
    }

    LoraRemote_GetState(&remote);
    TopSpeed_GPS_INS_PortGetLocalOdometryOutput(&odometry);

    if((IMU_savenum != 0U) &&
       (g_save_point_mode.start_pending == 0U))
    {
        save_point_update_local_position(&odometry);
    }
    if((g_save_point_mode.start_pending != 0U) &&
       (odometry.reset_sequence ==
        g_save_point_mode.local_odometry_reset_sequence))
    {
        save_point_update_local_position(&odometry);
        g_save_point_mode.odometry_distance_m = odometry.distance_m;
        g_save_point_mode.start_pending = 0U;
        if(g_save_point_mode.endpoint_finalized == 0U)
        {
            g_save_point_mode.recording = 1U;
        }
    }

    /* Keep the RAM route immutable while CPU0 is committing it to flash. */
    if(g_save_point_mode.save_pending != 0U)
    {
        if(remote.link_ok != 0U)
        {
            g_save_point_mode.previous_start_key =
                remote.key[SAVE_POINT_MODE_START_KEY_INDEX];
            g_save_point_mode.previous_stop_key =
                remote.key[SAVE_POINT_MODE_STOP_KEY_INDEX];
        }
        else
        {
            g_save_point_mode.previous_start_key = 0U;
            g_save_point_mode.previous_stop_key = 0U;
        }
        return;
    }

    if(remote.link_ok == 0U)
    {
        g_save_point_mode.previous_start_key = 0U;
        g_save_point_mode.previous_stop_key = 0U;
    }
    else
    {
        start_key = remote.key[SAVE_POINT_MODE_START_KEY_INDEX];
        stop_key = remote.key[SAVE_POINT_MODE_STOP_KEY_INDEX];
        combination_rising = (uint8_t)(
            (start_key != 0U) && (stop_key != 0U) &&
            ((g_save_point_mode.previous_start_key == 0U) ||
             (g_save_point_mode.previous_stop_key == 0U)));

        /* The combination has priority so it cannot also restart or stop. */
        if(combination_rising != 0U)
        {
            save_point_finish_recording();
            if((IMU_savenum != 0U) &&
               (g_save_point_mode.save_pending == 0U))
            {
                g_save_point_mode.save_pending = 1U;
            }
        }
        else
        {
            if((start_key != 0U) &&
               (g_save_point_mode.previous_start_key == 0U))
            {
                save_point_start(&odometry);
            }
            if((stop_key != 0U) &&
               (g_save_point_mode.previous_stop_key == 0U))
            {
                save_point_finish_recording();
            }
        }

        g_save_point_mode.previous_start_key = start_key;
        g_save_point_mode.previous_stop_key = stop_key;
    }

    if((g_save_point_mode.recording != 0U) &&
       (odometry.encoder_valid != 0U))
    {
        float distance_delta_m = odometry.distance_m
                               - g_save_point_mode.odometry_distance_m;

        g_save_point_mode.odometry_distance_m = odometry.distance_m;
        if(distance_delta_m < 0.0f)
        {
            distance_delta_m = 0.0f;
        }
        g_save_point_mode.distance_since_last_point_m +=
            distance_delta_m;
        while((g_save_point_mode.recording != 0U) &&
              (g_save_point_mode.distance_since_last_point_m >=
               SAVE_POINT_MODE_SAMPLE_DISTANCE_M))
        {
            g_save_point_mode.distance_since_last_point_m -=
                SAVE_POINT_MODE_SAMPLE_DISTANCE_M;
            save_point_append_current();
        }
    }
}

void SavePointMode_Exit(void)
{
    g_save_point_mode.enabled = 0U;
    g_save_point_mode.recording = 0U;
    g_save_point_mode.start_pending = 0U;
    g_save_point_mode.print_pending = 0U;
    g_save_point_mode.print_header_pending = 0U;
    g_save_point_mode.previous_start_key = 0U;
    g_save_point_mode.previous_stop_key = 0U;
}

void SavePointMode_GetStatus(SavePointModeStatus_t *status)
{
    if(status == 0)
    {
        return;
    }

    status->point_count = IMU_savenum;
    status->local_x_m = g_save_point_mode.local_x_m;
    status->local_y_m = g_save_point_mode.local_y_m;
    if(IMU_savenum != 0U)
    {
        status->last_point_x_m = IMU_X[IMU_savenum - 1U];
        status->last_point_y_m = IMU_Y[IMU_savenum - 1U];
    }
    else
    {
        status->last_point_x_m = 0.0f;
        status->last_point_y_m = 0.0f;
    }
    status->relative_yaw_deg = g_save_point_mode.relative_yaw_deg;
    status->distance_since_last_point_m =
        g_save_point_mode.distance_since_last_point_m;
    status->wifi_session = g_save_point_mode.wifi_session;
    status->wifi_sent_count = g_save_point_mode.wifi_sent_count;
    status->wifi_error_count = g_save_point_mode.wifi_error_count;
    status->recording = g_save_point_mode.recording;
    status->full = g_save_point_mode.full;
    status->flash_saved = g_save_point_mode.flash_saved;
    status->flash_save_pending = g_save_point_mode.save_pending;
    status->start_rejected = g_save_point_mode.start_rejected;
    status->wifi_initialized = g_save_point_mode.wifi_initialized;
    status->wifi_connected = g_save_point_mode.wifi_connected;
}
