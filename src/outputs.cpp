#include "outputs.h"
#include "hal.h"
#include "hal_outputs.h"

void outputClass::set(bool newState) {
    hal_outputs_set_state(channel, state);
}

void outputClass::set_duty_cycle(double duty_cycle) {
    hal_outputs_set_duty_cycle(channel, duty_cycle);
}

outputClass outputs[16] {
    outputClass(PWM_CH_BRAKE),
    outputClass(PWM_CH_DSEL),
    outputClass(PWM_CH_DEN),
    outputClass(PWM_CH_4_0),
    outputClass(PWM_CH_3_1),
    outputClass(PWM_CH_3_0),
    outputClass(PWM_CH_2_1),
    outputClass(PWM_CH_2_0),
    outputClass(PWM_CH_STARTER),
    outputClass(PWM_CH_0_0),
    outputClass(PWM_CH_0_1),
    outputClass(PWM_CH_1_0),
    outputClass(PWM_CH_1_1),
    outputClass(PWM_CH_LED_LOCAL),
    outputClass(PWM_CH_AUX_SIDELIGHTS),
    outputClass(PWM_CH_AUX_LOGO)
};