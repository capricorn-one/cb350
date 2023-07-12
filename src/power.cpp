#include "power.h"
#include "outputs.h"
#include "hal.h"

mm_power::mm_power(void) {}

// Definition for pinout on power board. Power Output pins are in the back row.
// Pin 1 is the furthest to the right when looking at the board from the front.
// In order to not have to send the DSEL signal to the ADC, the power outputs on channel 0 are the ones
// that should be active normally for measuring current.
// Only outputs on channel 1 of the BTS chip should be rarely used items such as horn or turn signals.
// A routine can be run in ADC secondary mode to measure alternate currents. Due to this, the pin out is as follows:

//  -- PIN --       -- FUNCTION --          -- PWM (BTS) CHANNEL --
//  1               Auxillary               0-0
//  2               Horn                        0-1
//  3               Ignition                1-0
//  4               Compressor                  1-1
//  5               Headlight               2-0
//  6               Signal Left                 2-1
//  7               Tail Light              3-0
//  8               Signal Right                3-1
//  9               Highbeam                4-0
//  10              Brake Light                 4-1 (BRAKE)
//  11              Starter                 N/A

void mm_power::init(void) {

    // link power outputs to appropriate pwm channel on controller
    power_outputs[POWER_OUTPUT_AUXILLARY].output = &outputs[PWM_CH_0_0];
    power_outputs[POWER_OUTPUT_HORN].output = &outputs[PWM_CH_0_1];
    power_outputs[POWER_OUTPUT_IGNITION].output = &outputs[PWM_CH_1_0];
    power_outputs[POWER_OUTPUT_COMPRESSOR].output = &outputs[PWM_CH_1_1];
    power_outputs[POWER_OUTPUT_HEADLIGHT].output = &outputs[PWM_CH_2_0];
    power_outputs[POWER_OUTPUT_SIGNAL_LEFT].output = &outputs[PWM_CH_2_1];
    power_outputs[POWER_OUTPUT_TAIL_LIGHT].output = &outputs[PWM_CH_3_0];
    power_outputs[POWER_OUTPUT_SIGNAL_RIGHT].output = &outputs[PWM_CH_3_1];
    power_outputs[POWER_OUTPUT_HIGHBEAM].output = &outputs[PWM_CH_4_0];
    power_outputs[POWER_OUTPUT_BRAKE_LIGHT].output = &outputs[PWM_CH_BRAKE];
    power_outputs[POWER_OUTPUT_STARTER].output = &outputs[PWM_CH_STARTER];

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