#ifndef _HAL_OUTPUTS_H_
#define _HAL_OUTPUTS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

    typedef enum  {
        PWM_CH_BRAKE = 0,
        PWM_CH_DSEL = 1,
        PWM_CH_DEN = 2,
        PWM_CH_4_0 = 3,
        PWM_CH_3_1 = 4,
        PWM_CH_3_0 = 5,
        PWM_CH_2_1 = 6,
        PWM_CH_2_0 = 7,
        PWM_CH_STARTER = 8,
        PWM_CH_0_0 = 9,
        PWM_CH_0_1 = 10,
        PWM_CH_1_0 = 11,
        PWM_CH_1_1 = 12,
        PWM_CH_LED_LOCAL = 13,
        PWM_CH_AUX_SIDELIGHTS = 14,
        PWM_CH_AUX_LOGO = 15,
        PWM_CH_NUM = 16
    } output_channel_enum_t;

    void hal_outputs_init(void);

    void hal_outputs_set_state(output_channel_enum_t channel, bool state);

    void hal_outputs_set_state(output_channel_enum_t channel, bool state, bool immediate);

    void hal_outputs_set_duty_cycle(output_channel_enum_t channel, double duty_cycle);

    void hal_outputs_update_all(void);

#endif /* _HAL_OUTPUTS_H_ */