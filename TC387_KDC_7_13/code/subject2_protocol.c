#include "subject2_protocol.h"

static void subject2_write_u16_le(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value & 0xFFU);
    buffer[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void subject2_write_u32_le(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)(value & 0xFFU);
    buffer[1] = (uint8_t)((value >> 8U) & 0xFFU);
    buffer[2] = (uint8_t)((value >> 16U) & 0xFFU);
    buffer[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

uint8_t Subject2_protocol_build_wav_header(uint8_t *header, uint32_t pcm_bytes)
{
    uint32_t byte_rate;

    if(header == 0)
    {
        return 0U;
    }

    byte_rate = SUBJECT2_AUDIO_SAMPLE_RATE * SUBJECT2_AUDIO_CHANNELS *
                (SUBJECT2_AUDIO_BITS_PER_SAMPLE / 8U);
    header[0] = 'R';
    header[1] = 'I';
    header[2] = 'F';
    header[3] = 'F';
    subject2_write_u32_le(&header[4], pcm_bytes + 36U);
    header[8] = 'W';
    header[9] = 'A';
    header[10] = 'V';
    header[11] = 'E';
    header[12] = 'f';
    header[13] = 'm';
    header[14] = 't';
    header[15] = ' ';
    subject2_write_u32_le(&header[16], 16U);
    subject2_write_u16_le(&header[20], 1U);
    subject2_write_u16_le(&header[22], SUBJECT2_AUDIO_CHANNELS);
    subject2_write_u32_le(&header[24], SUBJECT2_AUDIO_SAMPLE_RATE);
    subject2_write_u32_le(&header[28], byte_rate);
    subject2_write_u16_le(&header[32],
                          SUBJECT2_AUDIO_CHANNELS * (SUBJECT2_AUDIO_BITS_PER_SAMPLE / 8U));
    subject2_write_u16_le(&header[34], SUBJECT2_AUDIO_BITS_PER_SAMPLE);
    header[36] = 'd';
    header[37] = 'a';
    header[38] = 't';
    header[39] = 'a';
    subject2_write_u32_le(&header[40], pcm_bytes);
    return 1U;
}

uint8_t Subject2_protocol_parse_command_code(const uint8_t *json,
                                             uint16_t json_length,
                                             uint8_t *command_code)
{
    static const uint8_t key[] = {'"', 'c', 'o', 'm', 'm', 'a', 'n', 'd', '_',
                                  'c', 'o', 'd', 'e', '"', ':'};
    uint16_t index;
    uint16_t key_index;
    uint16_t value;
    uint8_t digit_seen;
    uint8_t key_found;

    if((json == 0) || (command_code == 0) || (json_length < sizeof(key)))
    {
        return 0U;
    }

    key_found = 0U;
    for(index = 0U; index <= (uint16_t)(json_length - sizeof(key)); index++)
    {
        for(key_index = 0U; key_index < sizeof(key); key_index++)
        {
            if(json[index + key_index] != key[key_index])
            {
                break;
            }
        }
        if(key_index == sizeof(key))
        {
            index = (uint16_t)(index + sizeof(key));
            key_found = 1U;
            break;
        }
    }

    if(key_found == 0U)
    {
        return 0U;
    }

    while((index < json_length) && ((json[index] == ' ') || (json[index] == '\t')))
    {
        index++;
    }

    value = 0U;
    digit_seen = 0U;
    while((index < json_length) && (json[index] >= '0') && (json[index] <= '9'))
    {
        value = (uint16_t)(value * 10U + (uint16_t)(json[index] - '0'));
        if(value > 255U)
        {
            return 0U;
        }
        digit_seen = 1U;
        index++;
    }

    if(digit_seen == 0U)
    {
        return 0U;
    }

    *command_code = (uint8_t)value;
    return 1U;
}
