#ifndef SUBJECT2_VOICE_CONFIG_H
#define SUBJECT2_VOICE_CONFIG_H

#include "subject2_protocol.h"

#define SUBJECT2_AUDIO_CAPTURE_SECONDS       (2U)
#define SUBJECT2_AUDIO_CAPTURE_SAMPLES       (SUBJECT2_AUDIO_SAMPLE_RATE * SUBJECT2_AUDIO_CAPTURE_SECONDS)
#define SUBJECT2_AUDIO_PCM_BYTES            (SUBJECT2_AUDIO_CAPTURE_SAMPLES * 2U)

#define SUBJECT2_VAD_POLL_MS                (2U)
#define SUBJECT2_VAD_TRIGGER_DELTA          (120U)
#define SUBJECT2_VAD_REQUIRED_HITS          (3U)
#define SUBJECT2_VAD_BASELINE_SAMPLES       (64U)

#define SUBJECT2_ASR_SERVER_IP              "192.168.137.1"
#define SUBJECT2_ASR_SERVER_PORT            "9001"
#define SUBJECT2_WIFI_LOCAL_PORT            "0"
#define SUBJECT2_WIFI_SSID                  ""
#define SUBJECT2_WIFI_PASSWORD              ""

#define SUBJECT2_NETWORK_RETRY_MS           (5000U)
#define SUBJECT2_RESPONSE_TIMEOUT_MS        (15000U)
#define SUBJECT2_RESPONSE_MAX_BYTES        (768U)

#define SUBJECT2_HORN_PWM_CHANNEL           ATOM0_CH3_P21_5
#define SUBJECT2_HORN_PWM_DUTY              (5000U)

/* Set to 1 only after H2 pin mapping has been updated in the lamp driver. */
#define SUBJECT2_LIGHT_DRIVER_ENABLED       (0U)

#endif
