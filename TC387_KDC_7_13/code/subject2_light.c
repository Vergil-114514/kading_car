#include "zf_common_headfile.h"
#include "subject2_light.h"
#include "subject2_voice_config.h"

#if SUBJECT2_LIGHT_DRIVER_ENABLED
#include "zf_device_dot_matrix_screen.h"
#endif

#if SUBJECT2_LIGHT_DRIVER_ENABLED
static const char *subject2_light_symbol(const SUBJECT2_OUTPUT_STATE *output)
{
    if(output->left_indicator_on != 0U && output->right_indicator_on != 0U)
    {
        return "!\"#";
    }
    if(output->left_indicator_on != 0U)
    {
        return "$%&";
    }
    if(output->right_indicator_on != 0U)
    {
        return "'()";
    }
    if(output->high_beam_on != 0U)
    {
        return "-./";
    }
    if(output->low_beam_on != 0U)
    {
        return "*+,";
    }
    if(output->fog_light_on != 0U)
    {
        return "{|}";
    }
    if(output->interior_light_on != 0U)
    {
        return "I  ";
    }
    if(output->wiper_on != 0U)
    {
        return "W  ";
    }
    return "   ";
}
#endif

void Subject2_light_init(void)
{
#if SUBJECT2_LIGHT_DRIVER_ENABLED
    dot_matrix_screen_init();
#endif
}

void Subject2_light_apply(const SUBJECT2_OUTPUT_STATE *output)
{
#if SUBJECT2_LIGHT_DRIVER_ENABLED
    static char last_symbol[4] = "???";
    const char *symbol;

    if(output == 0)
    {
        return;
    }

    symbol = subject2_light_symbol(output);
    if((last_symbol[0] == symbol[0]) &&
       (last_symbol[1] == symbol[1]) &&
       (last_symbol[2] == symbol[2]))
    {
        return;
    }

    dot_matrix_screen_show_string(symbol);
    last_symbol[0] = symbol[0];
    last_symbol[1] = symbol[1];
    last_symbol[2] = symbol[2];
#else
    (void)output;
#endif
}

void Subject2_light_task(void)
{
#if SUBJECT2_LIGHT_DRIVER_ENABLED
    dot_matrix_screen_scan();
#endif
}
