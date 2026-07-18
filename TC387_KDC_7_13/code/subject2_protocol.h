#ifndef SUBJECT2_PROTOCOL_H
#define SUBJECT2_PROTOCOL_H

#include <stdint.h>

#define SUBJECT2_WAV_HEADER_SIZE      (44U)
#define SUBJECT2_AUDIO_SAMPLE_RATE    (16000U)
#define SUBJECT2_AUDIO_CHANNELS       (1U)
#define SUBJECT2_AUDIO_BITS_PER_SAMPLE (16U)

uint8_t Subject2_protocol_build_wav_header(uint8_t *header, uint32_t pcm_bytes);
uint8_t Subject2_protocol_parse_command_code(const uint8_t *json,
                                             uint16_t json_length,
                                             uint8_t *command_code);

#endif
