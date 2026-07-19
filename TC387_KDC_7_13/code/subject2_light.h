#ifndef SUBJECT2_LIGHT_H
#define SUBJECT2_LIGHT_H

#include "subject2_command.h"

void Subject2_light_init(void);
void Subject2_light_apply(const SUBJECT2_OUTPUT_STATE *output);
void Subject2_light_sync_callback(void);
void Subject2_light_uart_callback(void);

#endif
