#include "zf_common_headfile.h"
#include "IfxStm.h"
#include "subject2_command.h"
#include "subject2_horn.h"
#include "subject2_light.h"
#include "subject2_protocol.h"
#include "subject2_voice.h"
#include "subject2_voice_config.h"

#pragma section all "cpu3_dsram"
static uint8 subject2_audio_buffer[SUBJECT2_AUDIO_PCM_BYTES];
static SUBJECT2_COMMAND_STATE subject2_command_state;
static SUBJECT2_VOICE_STATUS subject2_status;
#pragma section all restore

static uint32 subject2_stm_frequency;
static uint32 subject2_ticks_per_sample;
static uint32 subject2_ticks_per_ms;
static uint32 subject2_last_task_tick;
static uint32 subject2_last_network_attempt_tick;
static uint16 subject2_noise_floor;
static uint8 subject2_vad_hits;
static uint8 subject2_network_initialized;

static uint32 subject2_now(void)
{
    return (uint32)IfxStm_get(&MODULE_STM3);
}

static uint32 subject2_elapsed_ticks(uint32 start, uint32 end)
{
    return (uint32)(end - start);
}

static void subject2_wait_until(uint32 deadline)
{
    while((sint32)(subject2_now() - deadline) < 0)
    {
    }
}

static void subject2_tick_outputs(void)
{
    uint32 now;
    uint32 elapsed_ticks;
    uint32 elapsed_ms;
    SUBJECT2_OUTPUT_STATE output;

    now = subject2_now();
    elapsed_ticks = subject2_elapsed_ticks(subject2_last_task_tick, now);
    elapsed_ms = (subject2_ticks_per_ms == 0U) ? 0U :
                 (elapsed_ticks / subject2_ticks_per_ms);
    if(elapsed_ms == 0U)
    {
        return;
    }

    subject2_last_task_tick += elapsed_ms * subject2_ticks_per_ms;
    Subject2_command_tick(&subject2_command_state, elapsed_ms);
    Subject2_command_get_output(&subject2_command_state, &output);
    Subject2_horn_apply(&output);
    Subject2_light_apply(&output);
}

static uint16 subject2_sample_amplitude(void)
{
    sint32 centered;
    uint16 sample;

    sample = adc_convert(ADC0_CH4_A4);
    centered = (sint32)sample - 2048;
    if(centered < 0)
    {
        centered = -centered;
    }
    return (uint16)centered;
}

static void subject2_update_noise_floor(uint16 amplitude)
{
    subject2_noise_floor = (uint16)(((uint32)subject2_noise_floor * 15U + amplitude) / 16U);
    if(subject2_status.noise_floor != subject2_noise_floor)
    {
        subject2_status.noise_floor = subject2_noise_floor;
    }
}

static uint8 subject2_voice_triggered(void)
{
    uint16 amplitude;
    uint16 trigger_level;

    amplitude = subject2_sample_amplitude();
    subject2_update_noise_floor(amplitude);
    trigger_level = (uint16)(subject2_noise_floor + SUBJECT2_VAD_TRIGGER_DELTA);

    if(amplitude > trigger_level)
    {
        if(subject2_vad_hits < SUBJECT2_VAD_REQUIRED_HITS)
        {
            subject2_vad_hits++;
        }
    }
    else
    {
        subject2_vad_hits = 0U;
    }

    return (uint8)(subject2_vad_hits >= SUBJECT2_VAD_REQUIRED_HITS);
}

static void subject2_capture_audio(void)
{
    uint32 sample_index;
    uint32 next_sample_tick;
    uint16 sample;
    sint32 pcm;

    next_sample_tick = subject2_now();
    for(sample_index = 0U; sample_index < SUBJECT2_AUDIO_CAPTURE_SAMPLES; sample_index++)
    {
        subject2_wait_until(next_sample_tick);
        sample = adc_convert(ADC0_CH4_A4);
        pcm = ((sint32)sample - 2048) << 4;
        if(pcm > 32767)
        {
            pcm = 32767;
        }
        else if(pcm < -32768)
        {
            pcm = -32768;
        }
        subject2_audio_buffer[sample_index * 2U] = (uint8)(pcm & 0xFF);
        subject2_audio_buffer[sample_index * 2U + 1U] =
            (uint8)(((uint32)pcm >> 8U) & 0xFFU);
        next_sample_tick += subject2_ticks_per_sample;
    }
}

static uint8 subject2_network_connect(void)
{
    uint8 wifi_state;
    char *ssid;
    char *password;

    ssid = (SUBJECT2_WIFI_SSID[0] == '\0') ? NULL : (char *)SUBJECT2_WIFI_SSID;
    password = (SUBJECT2_WIFI_PASSWORD[0] == '\0') ? NULL : (char *)SUBJECT2_WIFI_PASSWORD;
    wifi_state = wifi_spi_init(ssid, password);
    if(wifi_state != 0U)
    {
        return 0U;
    }

    return (uint8)(wifi_spi_socket_connect("TCP",
                                           (char *)SUBJECT2_ASR_SERVER_IP,
                                           (char *)SUBJECT2_ASR_SERVER_PORT,
                                           (char *)SUBJECT2_WIFI_LOCAL_PORT) == 0U);
}

static uint8 subject2_send_audio(void)
{
    uint8 length_header[4];
    uint8 wav_header[SUBJECT2_WAV_HEADER_SIZE];
    uint32 wav_bytes;

    wav_bytes = SUBJECT2_WAV_HEADER_SIZE + SUBJECT2_AUDIO_PCM_BYTES;
    length_header[0] = (uint8)((wav_bytes >> 24U) & 0xFFU);
    length_header[1] = (uint8)((wav_bytes >> 16U) & 0xFFU);
    length_header[2] = (uint8)((wav_bytes >> 8U) & 0xFFU);
    length_header[3] = (uint8)(wav_bytes & 0xFFU);
    Subject2_protocol_build_wav_header(wav_header, SUBJECT2_AUDIO_PCM_BYTES);

    if(wifi_spi_send_buffer(length_header, sizeof(length_header)) != 0U)
    {
        return 0U;
    }
    if(wifi_spi_send_buffer(wav_header, sizeof(wav_header)) != 0U)
    {
        return 0U;
    }
    return (uint8)(wifi_spi_send_buffer(subject2_audio_buffer,
                                        SUBJECT2_AUDIO_PCM_BYTES) == 0U);
}

static uint8 subject2_read_exact(uint8 *buffer, uint32 length)
{
    uint32 received;
    uint32 start_tick;
    uint32 timeout_ticks;
    uint32 read_size;

    received = 0U;
    start_tick = subject2_now();
    timeout_ticks = (subject2_stm_frequency / 1000U) * SUBJECT2_RESPONSE_TIMEOUT_MS;
    while(received < length)
    {
        read_size = wifi_spi_read_buffer(&buffer[received], length - received);
        received += read_size;
        subject2_tick_outputs();
        if(subject2_elapsed_ticks(start_tick, subject2_now()) > timeout_ticks)
        {
            return 0U;
        }
        if(read_size == 0U)
        {
            system_delay_ms(10U);
        }
    }
    return 1U;
}

static uint8 subject2_receive_command(uint8 *command_code)
{
    uint8 length_header[4];
    uint8 response[SUBJECT2_RESPONSE_MAX_BYTES];
    uint32 response_length;

    if(subject2_read_exact(length_header, sizeof(length_header)) == 0U)
    {
        return 0U;
    }
    response_length = ((uint32)length_header[0] << 24U) |
                      ((uint32)length_header[1] << 16U) |
                      ((uint32)length_header[2] << 8U) |
                      (uint32)length_header[3];
    if((response_length == 0U) || (response_length > sizeof(response)))
    {
        return 0U;
    }
    if(subject2_read_exact(response, response_length) == 0U)
    {
        return 0U;
    }
    return Subject2_protocol_parse_command_code(response,
                                                (uint16)response_length,
                                                command_code);
}

static void subject2_initialize_noise_floor(void)
{
    uint32 sample_index;
    uint32 sum;

    sum = 0U;
    for(sample_index = 0U; sample_index < SUBJECT2_VAD_BASELINE_SAMPLES; sample_index++)
    {
        sum += subject2_sample_amplitude();
        system_delay_ms(1U);
    }
    subject2_noise_floor = (uint16)(sum / SUBJECT2_VAD_BASELINE_SAMPLES);
    subject2_status.noise_floor = subject2_noise_floor;
}

void Subject2_voice_init(void)
{
    subject2_stm_frequency = (uint32)IfxStm_getFrequency(&MODULE_STM3);
    subject2_ticks_per_sample = (subject2_stm_frequency + 8000U) / SUBJECT2_AUDIO_SAMPLE_RATE;
    subject2_ticks_per_ms = (subject2_stm_frequency + 500U) / 1000U;
    subject2_last_task_tick = subject2_now();
    subject2_last_network_attempt_tick = subject2_last_task_tick;
    subject2_network_initialized = 0U;
    subject2_vad_hits = 0U;
    Subject2_command_init(&subject2_command_state);
    subject2_status = (SUBJECT2_VOICE_STATUS){0};
    subject2_status.state = SUBJECT2_VOICE_OFFLINE;

    adc_init(ADC0_CH4_A4, ADC_12BIT);
    Subject2_horn_init();
    Subject2_light_init();
    subject2_initialize_noise_floor();
    subject2_network_initialized = subject2_network_connect();
    subject2_status.network_ready = subject2_network_initialized;
    subject2_status.state = (subject2_network_initialized != 0U) ?
                           SUBJECT2_VOICE_READY : SUBJECT2_VOICE_ERROR;
}

void Subject2_voice_task(void)
{
    uint32 now;
    uint8 command_code;
    SUBJECT2_OUTPUT_STATE output;

    subject2_tick_outputs();
    Subject2_light_task();
    now = subject2_now();

    if(subject2_network_initialized == 0U)
    {
        if(subject2_elapsed_ticks(subject2_last_network_attempt_tick, now) >=
           ((subject2_stm_frequency / 1000U) * SUBJECT2_NETWORK_RETRY_MS))
        {
            subject2_last_network_attempt_tick = now;
            subject2_network_initialized = subject2_network_connect();
            subject2_status.network_ready = subject2_network_initialized;
            subject2_status.state = (subject2_network_initialized != 0U) ?
                                   SUBJECT2_VOICE_READY : SUBJECT2_VOICE_ERROR;
        }
        system_delay_ms(SUBJECT2_VAD_POLL_MS);
        return;
    }

    if(subject2_status.state == SUBJECT2_VOICE_READY)
    {
        if(subject2_voice_triggered() != 0U)
        {
            subject2_vad_hits = 0U;
            subject2_status.state = SUBJECT2_VOICE_CAPTURING;
            subject2_capture_audio();
            subject2_status.state = SUBJECT2_VOICE_WAITING_RESPONSE;
            if(subject2_send_audio() == 0U)
            {
                wifi_spi_socket_disconnect();
                subject2_network_initialized = 0U;
                subject2_status.network_ready = 0U;
                subject2_status.state = SUBJECT2_VOICE_ERROR;
            }
            else if(subject2_receive_command(&command_code) == 0U)
            {
                wifi_spi_socket_disconnect();
                subject2_network_initialized = 0U;
                subject2_status.network_ready = 0U;
                subject2_status.state = SUBJECT2_VOICE_ERROR;
            }
            else
            {
                subject2_status.last_command_code = command_code;
                subject2_status.last_command_accepted =
                    Subject2_command_dispatch(&subject2_command_state, command_code);
                Subject2_command_get_output(&subject2_command_state, &output);
                Subject2_horn_apply(&output);
                Subject2_light_apply(&output);
                subject2_status.state = SUBJECT2_VOICE_READY;
            }
        }
    }
    system_delay_ms(SUBJECT2_VAD_POLL_MS);
}

void Subject2_voice_get_status(SUBJECT2_VOICE_STATUS *status)
{
    if(status != 0)
    {
        *status = subject2_status;
    }
}
