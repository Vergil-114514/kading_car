#include "zf_common_headfile.h"
#include "IfxStm.h"
#include "subject2_command.h"
#include "subject2_horn.h"
#include "subject2_light.h"
#include "subject2_protocol.h"
#include "subject2_voice.h"
#include "subject2_voice_config.h"

#pragma section all "lmubss"
static uint8 subject2_audio_buffer[SUBJECT2_AUDIO_MAX_PCM_BYTES];
static uint8 subject2_preroll_buffer[SUBJECT2_AUDIO_PREROLL_PCM_BYTES];
#pragma section all restore

#pragma section all "cpu3_dsram"
static SUBJECT2_COMMAND_STATE subject2_command_state;
static SUBJECT2_VOICE_STATUS subject2_status;
static uint32 subject2_runtime_ticks;
static uint8 subject2_runtime_enabled;
#pragma section all restore

static uint32 subject2_stm_frequency;
static uint32 subject2_ticks_per_sample;
static uint32 subject2_last_network_attempt_tick;
static uint32 subject2_next_sample_tick;
static uint32 subject2_capture_samples;
static uint32 subject2_frame_abs_sum;
static uint16 subject2_frame_sample_count;
static uint16 subject2_noise_floor;
static uint16 subject2_dc_estimate;
static uint16 subject2_preroll_write_index;
static uint16 subject2_silence_frames;
static uint16 subject2_rearm_frames;
static uint8 subject2_start_frames;
static uint8 subject2_dc_ready;
static uint8 subject2_listening_armed;
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
    SUBJECT2_OUTPUT_STATE output;

    Subject2_command_tick(&subject2_command_state, SUBJECT2_RUNTIME_TICK_MS);
    Subject2_command_get_output(&subject2_command_state, &output);
    Subject2_horn_apply(&output);
    Subject2_light_apply(&output);
}

IFX_INTERRUPT(subject2_runtime_tick_isr, 3, SUBJECT2_RUNTIME_TICK_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    IfxStm_clearCompareFlag(&MODULE_STM3, IfxStm_Comparator_1);
    IfxStm_increaseCompare(&MODULE_STM3, IfxStm_Comparator_1, subject2_runtime_ticks);

    if(subject2_runtime_enabled != 0U)
    {
        subject2_tick_outputs();
    }
}

static void subject2_runtime_init(void)
{
    IfxStm_CompareConfig config;

    subject2_runtime_ticks =
        (uint32)IfxStm_getTicksFromMilliseconds(&MODULE_STM3, SUBJECT2_RUNTIME_TICK_MS);
    if(subject2_runtime_ticks == 0U)
    {
        subject2_runtime_ticks = 1U;
    }

    IfxStm_initCompareConfig(&config);
    config.comparator = IfxStm_Comparator_1;
    config.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir1;
    config.compareOffset = IfxStm_ComparatorOffset_0;
    config.compareSize = IfxStm_ComparatorSize_32Bits;
    config.ticks = subject2_runtime_ticks;
    config.triggerPriority = SUBJECT2_RUNTIME_TICK_ISR_PRIORITY;
    config.typeOfService = IfxSrc_Tos_cpu3;

    subject2_runtime_enabled = (uint8)(IfxStm_initCompare(&MODULE_STM3, &config) != FALSE);
}

static uint16 subject2_read_sample(void)
{
    return adc_convert(ADC0_CH4_A4);
}

static void subject2_update_dc_estimate(uint16 sample)
{
    sint32 delta;

    if(subject2_dc_ready == 0U)
    {
        subject2_dc_estimate = sample;
        subject2_dc_ready = 1U;
        return;
    }

    delta = (sint32)sample - (sint32)subject2_dc_estimate;
    if(delta > 0)
    {
        subject2_dc_estimate += (uint16)((delta > 255) ? (delta / 256) : 1);
    }
    else if(delta < 0)
    {
        delta = -delta;
        subject2_dc_estimate -= (uint16)((delta > 255) ? (delta / 256) : 1);
    }
}

static uint16 subject2_sample_amplitude(uint16 sample)
{
    sint32 centered;

    centered = (sint32)sample - (sint32)subject2_dc_estimate;
    if(centered < 0)
    {
        centered = -centered;
    }
    return (uint16)centered;
}

static uint16 subject2_vad_threshold(void)
{
    uint16 scaled_floor;
    uint16 offset_floor;

    scaled_floor = (uint16)(subject2_noise_floor * SUBJECT2_VAD_NOISE_MULTIPLIER);
    offset_floor = (uint16)(subject2_noise_floor + SUBJECT2_VAD_MIN_DELTA);
    return (scaled_floor > offset_floor) ? scaled_floor : offset_floor;
}

static void subject2_store_preroll(uint16 sample)
{
    subject2_preroll_buffer[subject2_preroll_write_index * 2U] =
        (uint8)(sample & 0xFFU);
    subject2_preroll_buffer[subject2_preroll_write_index * 2U + 1U] =
        (uint8)((sample >> 8U) & 0xFFU);
    subject2_preroll_write_index++;
    if(subject2_preroll_write_index >= SUBJECT2_AUDIO_PREROLL_SAMPLES)
    {
        subject2_preroll_write_index = 0U;
    }
}

static void subject2_copy_preroll(void)
{
    uint16 index;
    uint16 source_index;

    for(index = 0U; index < SUBJECT2_AUDIO_PREROLL_SAMPLES; index++)
    {
        source_index = (uint16)(subject2_preroll_write_index + index);
        if(source_index >= SUBJECT2_AUDIO_PREROLL_SAMPLES)
        {
            source_index -= SUBJECT2_AUDIO_PREROLL_SAMPLES;
        }
        subject2_audio_buffer[index * 2U] = subject2_preroll_buffer[source_index * 2U];
        subject2_audio_buffer[index * 2U + 1U] =
            subject2_preroll_buffer[source_index * 2U + 1U];
    }
    subject2_capture_samples = SUBJECT2_AUDIO_PREROLL_SAMPLES;
}

static void subject2_start_capture(void)
{
    subject2_copy_preroll();
    subject2_frame_abs_sum = 0U;
    subject2_frame_sample_count = 0U;
    subject2_silence_frames = 0U;
    subject2_listening_armed = 0U;
    subject2_status.state = SUBJECT2_VOICE_CAPTURING;
}

static void subject2_finish_capture(void)
{
    subject2_status.state = SUBJECT2_VOICE_WAITING_RESPONSE;
    subject2_listening_armed = 0U;
    subject2_rearm_frames = 0U;
}

static void subject2_process_listening_frame(uint16 energy)
{
    uint16 threshold;

    threshold = subject2_vad_threshold();
    if(energy <= threshold)
    {
        subject2_noise_floor = (uint16)(((uint32)subject2_noise_floor * 31U + energy) / 32U);
        subject2_status.noise_floor = subject2_noise_floor;
    }

    if(subject2_listening_armed == 0U)
    {
        if(energy <= threshold)
        {
            if(subject2_rearm_frames < SUBJECT2_VAD_REARM_FRAMES)
            {
                subject2_rearm_frames++;
            }
            if(subject2_rearm_frames >= SUBJECT2_VAD_REARM_FRAMES)
            {
                subject2_listening_armed = 1U;
                subject2_start_frames = 0U;
            }
        }
        else
        {
            subject2_rearm_frames = 0U;
        }
        return;
    }

    if(energy > threshold)
    {
        if(subject2_start_frames < SUBJECT2_VAD_START_FRAMES)
        {
            subject2_start_frames++;
        }
        if(subject2_start_frames >= SUBJECT2_VAD_START_FRAMES)
        {
            subject2_start_capture();
        }
    }
    else
    {
        subject2_start_frames = 0U;
    }
}

static void subject2_process_capture_frame(uint16 energy)
{
    uint16 threshold;

    threshold = subject2_vad_threshold();
    if((subject2_capture_samples >= SUBJECT2_AUDIO_MIN_SAMPLES) &&
       (energy <= threshold))
    {
        if(subject2_silence_frames < SUBJECT2_VAD_SILENCE_FRAMES)
        {
            subject2_silence_frames++;
        }
        if(subject2_silence_frames >= SUBJECT2_VAD_SILENCE_FRAMES)
        {
            subject2_finish_capture();
        }
    }
    else
    {
        subject2_silence_frames = 0U;
    }

    if(subject2_capture_samples >= SUBJECT2_AUDIO_MAX_SAMPLES)
    {
        subject2_capture_samples = SUBJECT2_AUDIO_MAX_SAMPLES;
        subject2_finish_capture();
    }
}

static void subject2_sample_step(void)
{
    uint16 sample;
    uint16 energy;

    sample = subject2_read_sample();
    subject2_update_dc_estimate(sample);
    subject2_store_preroll(sample);

    if(subject2_status.state == SUBJECT2_VOICE_CAPTURING)
    {
        if(subject2_capture_samples < SUBJECT2_AUDIO_MAX_SAMPLES)
        {
            subject2_audio_buffer[subject2_capture_samples * 2U] =
                (uint8)(sample & 0xFFU);
            subject2_audio_buffer[subject2_capture_samples * 2U + 1U] =
                (uint8)((sample >> 8U) & 0xFFU);
            subject2_capture_samples++;
        }
    }

    subject2_frame_abs_sum += subject2_sample_amplitude(sample);
    subject2_frame_sample_count++;
    if(subject2_frame_sample_count < SUBJECT2_VAD_FRAME_SAMPLES)
    {
        return;
    }

    energy = (uint16)(subject2_frame_abs_sum / SUBJECT2_VAD_FRAME_SAMPLES);
    subject2_frame_abs_sum = 0U;
    subject2_frame_sample_count = 0U;
    if(subject2_status.state == SUBJECT2_VOICE_CAPTURING)
    {
        subject2_process_capture_frame(energy);
    }
    else if(subject2_status.state == SUBJECT2_VOICE_READY)
    {
        subject2_process_listening_frame(energy);
    }
}

static void subject2_listener_step(void)
{
    uint32 now;

    now = subject2_now();
    if((sint32)(now - subject2_next_sample_tick) < 0)
    {
        return;
    }
    subject2_next_sample_tick += subject2_ticks_per_sample;
    if((sint32)(now - subject2_next_sample_tick) >= 0)
    {
        subject2_next_sample_tick = now + subject2_ticks_per_sample;
    }
    subject2_sample_step();
}

static uint32 subject2_prepare_pcm(void)
{
    uint32 index;
    uint32 sum;
    uint32 peak;
    uint32 gain;
    uint16 raw;
    uint16 mean;
    sint32 centered;
    sint32 pcm;

    sum = 0U;
    for(index = 0U; index < subject2_capture_samples; index++)
    {
        raw = (uint16)subject2_audio_buffer[index * 2U] |
              ((uint16)subject2_audio_buffer[index * 2U + 1U] << 8U);
        sum += raw;
    }
    mean = (uint16)(sum / subject2_capture_samples);
    peak = 1U;
    for(index = 0U; index < subject2_capture_samples; index++)
    {
        raw = (uint16)subject2_audio_buffer[index * 2U] |
              ((uint16)subject2_audio_buffer[index * 2U + 1U] << 8U);
        centered = ((sint32)raw - (sint32)mean) * 16;
        if(centered < 0)
        {
            centered = -centered;
        }
        if((uint32)centered > peak)
        {
            peak = (uint32)centered;
        }
    }
    gain = SUBJECT2_AGC_TARGET_PEAK / peak;
    if(gain < 1U)
    {
        gain = 1U;
    }
    if(gain > SUBJECT2_AGC_MAX_GAIN)
    {
        gain = SUBJECT2_AGC_MAX_GAIN;
    }

    for(index = 0U; index < subject2_capture_samples; index++)
    {
        raw = (uint16)subject2_audio_buffer[index * 2U] |
              ((uint16)subject2_audio_buffer[index * 2U + 1U] << 8U);
        centered = ((sint32)raw - (sint32)mean) * 16;
        pcm = centered * (sint32)gain;
        if(pcm > 32767)
        {
            pcm = 32767;
        }
        else if(pcm < -32768)
        {
            pcm = -32768;
        }
        subject2_audio_buffer[index * 2U] = (uint8)((uint32)pcm & 0xFFU);
        subject2_audio_buffer[index * 2U + 1U] =
            (uint8)(((uint32)pcm >> 8U) & 0xFFU);
    }
    return subject2_capture_samples * 2U;
}

static uint8 subject2_network_init(void)
{
    char *ssid;
    char *password;

    ssid = (SUBJECT2_WIFI_SSID[0] == '\0') ? NULL : (char *)SUBJECT2_WIFI_SSID;
    password = (SUBJECT2_WIFI_PASSWORD[0] == '\0') ? NULL : (char *)SUBJECT2_WIFI_PASSWORD;
    return (uint8)(wifi_spi_init(ssid, password) == 0U);
}

static uint8 subject2_socket_connect(void)
{
    return (uint8)(wifi_spi_socket_connect("TCP",
                                           (char *)SUBJECT2_ASR_SERVER_IP,
                                           (char *)SUBJECT2_ASR_SERVER_PORT,
                                           (char *)SUBJECT2_WIFI_LOCAL_PORT) == 0U);
}

static uint8 subject2_send_audio(uint32 pcm_bytes)
{
    uint8 length_header[4];
    uint8 wav_header[SUBJECT2_WAV_HEADER_SIZE];
    uint32 wav_bytes;

    wav_bytes = SUBJECT2_WAV_HEADER_SIZE + pcm_bytes;
    length_header[0] = (uint8)((wav_bytes >> 24U) & 0xFFU);
    length_header[1] = (uint8)((wav_bytes >> 16U) & 0xFFU);
    length_header[2] = (uint8)((wav_bytes >> 8U) & 0xFFU);
    length_header[3] = (uint8)(wav_bytes & 0xFFU);
    Subject2_protocol_build_wav_header(wav_header, pcm_bytes);

    if(wifi_spi_send_buffer(length_header, sizeof(length_header)) != 0U)
    {
        return 0U;
    }
    if(wifi_spi_send_buffer(wav_header, sizeof(wav_header)) != 0U)
    {
        return 0U;
    }
    return (uint8)(wifi_spi_send_buffer(subject2_audio_buffer, pcm_bytes) == 0U);
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
    uint32 next_sample_tick;
    uint32 sum;
    uint16 sample;

    sum = 0U;
    next_sample_tick = subject2_now();
    for(sample_index = 0U; sample_index < SUBJECT2_VAD_BASELINE_SAMPLES; sample_index++)
    {
        subject2_wait_until(next_sample_tick);
        sample = subject2_read_sample();
        subject2_update_dc_estimate(sample);
        if(sample_index > (SUBJECT2_AUDIO_SAMPLE_RATE / 10U))
        {
            sum += subject2_sample_amplitude(sample);
        }
        next_sample_tick += subject2_ticks_per_sample;
    }
    subject2_noise_floor = (uint16)(sum /
        (SUBJECT2_VAD_BASELINE_SAMPLES - (SUBJECT2_AUDIO_SAMPLE_RATE / 10U)));
    subject2_status.noise_floor = subject2_noise_floor;
    subject2_preroll_write_index = 0U;
    for(sample_index = 0U; sample_index < SUBJECT2_AUDIO_PREROLL_SAMPLES; sample_index++)
    {
        subject2_preroll_buffer[sample_index * 2U] =
            (uint8)(subject2_dc_estimate & 0xFFU);
        subject2_preroll_buffer[sample_index * 2U + 1U] =
            (uint8)((subject2_dc_estimate >> 8U) & 0xFFU);
    }
}

void Subject2_voice_init(void)
{
    subject2_stm_frequency = (uint32)IfxStm_getFrequency(&MODULE_STM3);
    subject2_ticks_per_sample = (subject2_stm_frequency + 8000U) /
                                SUBJECT2_AUDIO_SAMPLE_RATE;
    if(subject2_ticks_per_sample == 0U)
    {
        subject2_ticks_per_sample = 1U;
    }
    subject2_last_network_attempt_tick = subject2_now();
    subject2_next_sample_tick = subject2_last_network_attempt_tick;
    subject2_capture_samples = 0U;
    subject2_frame_abs_sum = 0U;
    subject2_frame_sample_count = 0U;
    subject2_noise_floor = 0U;
    subject2_dc_estimate = 0U;
    subject2_preroll_write_index = 0U;
    subject2_silence_frames = 0U;
    subject2_rearm_frames = SUBJECT2_VAD_REARM_FRAMES;
    subject2_start_frames = 0U;
    subject2_dc_ready = 0U;
    subject2_listening_armed = 1U;
    subject2_network_initialized = 0U;
    Subject2_command_init(&subject2_command_state);
    subject2_status = (SUBJECT2_VOICE_STATUS){0};
    subject2_status.state = SUBJECT2_VOICE_OFFLINE;

    adc_init(ADC0_CH4_A4, ADC_12BIT);
    Subject2_horn_init();
    Subject2_light_init();
    subject2_runtime_init();
    subject2_initialize_noise_floor();
    subject2_network_initialized = subject2_network_init();
    subject2_status.network_ready = subject2_network_initialized;
    subject2_status.state = (subject2_network_initialized != 0U) ?
                           SUBJECT2_VOICE_READY : SUBJECT2_VOICE_ERROR;
}

void Subject2_voice_task(void)
{
    uint32 now;
    uint32 interrupt_state;
    uint32 pcm_bytes;
    uint8 command_code;
    uint8 transaction_ok;

    now = subject2_now();
    if(subject2_network_initialized == 0U)
    {
        if(subject2_elapsed_ticks(subject2_last_network_attempt_tick, now) >=
           ((subject2_stm_frequency / 1000U) * SUBJECT2_NETWORK_RETRY_MS))
        {
            subject2_last_network_attempt_tick = now;
            subject2_network_initialized = subject2_network_init();
            subject2_status.network_ready = subject2_network_initialized;
            subject2_status.state = (subject2_network_initialized != 0U) ?
                                   SUBJECT2_VOICE_READY : SUBJECT2_VOICE_ERROR;
            subject2_next_sample_tick = subject2_now() + subject2_ticks_per_sample;
        }
        system_delay_ms(2U);
        return;
    }

    if(subject2_status.state == SUBJECT2_VOICE_READY)
    {
        subject2_listener_step();
    }

    if(subject2_status.state == SUBJECT2_VOICE_WAITING_RESPONSE)
    {
        pcm_bytes = subject2_prepare_pcm();
        transaction_ok = subject2_socket_connect();
        if(transaction_ok != 0U)
        {
            transaction_ok = subject2_send_audio(pcm_bytes);
        }
        if(transaction_ok != 0U)
        {
            transaction_ok = subject2_receive_command(&command_code);
        }
        wifi_spi_socket_disconnect();
        if(transaction_ok == 0U)
        {
            subject2_network_initialized = 0U;
            subject2_status.network_ready = 0U;
            subject2_status.state = SUBJECT2_VOICE_ERROR;
        }
        else
        {
            subject2_status.last_command_code = command_code;
            interrupt_state = interrupt_global_disable();
            subject2_status.last_command_accepted =
                Subject2_command_dispatch(&subject2_command_state, command_code);
            interrupt_global_enable(interrupt_state);
            subject2_status.state = SUBJECT2_VOICE_READY;
            subject2_rearm_frames = 0U;
            subject2_listening_armed = 0U;
            subject2_next_sample_tick = subject2_now() + subject2_ticks_per_sample;
        }
    }
}

void Subject2_voice_get_status(SUBJECT2_VOICE_STATUS *status)
{
    if(status != 0)
    {
        *status = subject2_status;
    }
}
