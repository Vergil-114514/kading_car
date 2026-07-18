#include "zf_common_headfile.h"
#include "subject2_horn.h"
#include "subject2_voice_config.h"

void Subject2_horn_init(void)
{
    pwm_init(SUBJECT2_HORN_PWM_CHANNEL, 1000U, 0U);
}

void Subject2_horn_apply(const SUBJECT2_OUTPUT_STATE *output)
{
    static uint16 last_frequency_hz;

    if(output == 0)
    {
        return;
    }

    if((output->horn_on == 0U) || (output->horn_frequency_hz == 0U))
    {
        pwm_set_duty(SUBJECT2_HORN_PWM_CHANNEL, 0U);
        last_frequency_hz = 0U;
        return;
    }

    if(output->horn_frequency_hz != last_frequency_hz)
    {
        pwm_init(SUBJECT2_HORN_PWM_CHANNEL,
                 output->horn_frequency_hz,
                 SUBJECT2_HORN_PWM_DUTY);
        last_frequency_hz = output->horn_frequency_hz;
    }
    else
    {
        pwm_set_duty(SUBJECT2_HORN_PWM_CHANNEL, SUBJECT2_HORN_PWM_DUTY);
    }
}
