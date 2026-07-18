#ifndef SUBJECT2_VOICE_H
#define SUBJECT2_VOICE_H

#include <stdint.h>

typedef enum
{
    SUBJECT2_VOICE_OFFLINE = 0,
    SUBJECT2_VOICE_READY,
    SUBJECT2_VOICE_CAPTURING,
    SUBJECT2_VOICE_WAITING_RESPONSE,
    SUBJECT2_VOICE_ERROR
} SUBJECT2_VOICE_STATE;

typedef struct
{
    SUBJECT2_VOICE_STATE state;
    uint16_t noise_floor;
    uint8_t network_ready;
    uint8_t last_command_code;
    uint8_t last_command_accepted;
} SUBJECT2_VOICE_STATUS;

void Subject2_voice_init(void);
void Subject2_voice_task(void);
void Subject2_voice_get_status(SUBJECT2_VOICE_STATUS *status);

#endif
