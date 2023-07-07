#include "power.h"
#include "outputs.h"
#include "hal.h"

mm_power::mm_power(void) {}

void mm_power::init(void) {

    
    // link power outputs to appropriate pwm channel on controller
    power_outputs[POWER_OUTPUT_IGNITION].output = &outputs[PWM_CH_1_0];
    power_outputs[POWER_OUTPUT_SIGNAL_LEFT].output = &outputs[PWM_CH_0_0];
    power_outputs[POWER_OUTPUT_SIGNAL_RIGHT].output = &outputs[PWM_CH_0_1];
    power_outputs[POWER_OUTPUT_HEADLIGHT].output = &outputs[PWM_CH_2_1];
    power_outputs[POWER_OUTPUT_BRAKE_LIGHT].output = &outputs[PWM_CH_BRAKE];
    power_outputs[POWER_OUTPUT_HIGHBEAM].output = &outputs[PWM_CH_3_0];
    power_outputs[POWER_OUTPUT_TAIL_LIGHT].output = &outputs[PWM_CH_3_1];
    power_outputs[POWER_OUTPUT_HORN].output = &outputs[PWM_CH_1_1];
    power_outputs[POWER_OUTPUT_AUXILLARY].output = &outputs[PWM_CH_2_0];
    power_outputs[POWER_OUTPUT_STARTER].output = &outputs[PWM_CH_STARTER];
    power_outputs[POWER_OUTPUT_COMPRESSOR].output = &outputs[PWM_CH_4_0];

    // adc begin
    // configure ads131m0x to switch DSEL pin after readings to alternate between output
    // current monitoring.

    // setup analog channels on local board?
}

void mm_power::set(power_output_enum_t output_num, bool state) {
    power_outputs[output_num].output->set(state);
}

bool mm_power::get(power_output_enum_t output_num) {
    return power_outputs[output_num].output->get();
}

void mm_power::set_duty_cycle(power_output_enum_t output_num, float duty_cycle) {
    power_outputs[output_num].output->set_duty_cycle(duty_cycle);
}


mm_power pwr_output;