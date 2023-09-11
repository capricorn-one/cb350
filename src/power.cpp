#include "power.h"
#include "hal.h"
#include "hal_outputs.h"
#include "moto_can.h"

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

uint16_t mm_power::output_states = 0;
uint16_t mm_power::last_output_states = 0;

mm_power::mm_power(uint8_t pwm_channel, uint8_t pwr_output_num) {
    channel = pwm_channel;
    output_num = pwr_output_num;
    enabled = false;
}

// need to update the 16bit output_states variable with the new state of the output
// from mm_types
// typedef enum {
//     POWER_OUTPUT_IGNITION = 0,
//     POWER_OUTPUT_SIGNAL_LEFT = 1,
//     POWER_OUTPUT_SIGNAL_RIGHT = 2,
//     POWER_OUTPUT_HEADLIGHT = 3,
//     POWER_OUTPUT_BRAKE_LIGHT = 4,
//     POWER_OUTPUT_HIGHBEAM = 5,
//     POWER_OUTPUT_TAIL_LIGHT = 6,
//     POWER_OUTPUT_HORN = 7,
//     POWER_OUTPUT_AUXILLARY = 8,
//     POWER_OUTPUT_STARTER = 9,
//     POWER_OUTPUT_COMPRESSOR = 10,
// 	POWER_OUTPUT_NUM = 11
// } power_output_enum_t;

void mm_power::set(bool state) {
    set(state, true);
}

void mm_power::set(bool state, bool immediate) {

    if(state == enabled)
        return;

    if(state)
        output_states |= (1 << output_num);
    else
        output_states &= ~(1 << output_num);
    
    if(immediate) {
        moto_can.send_output_state_change(output_states);
        last_output_states = output_states;
    }

    hal_outputs_set_state((output_channel_enum_t)channel, enabled, immediate);

    enabled = state;
}

void mm_power::set_duty_cycle(float duty_cycle) {
    hal_outputs_set_duty_cycle((output_channel_enum_t)channel, duty_cycle);
}

void mm_power::update_all(void) {
    if(output_states != last_output_states) {
        moto_can.send_output_state_change(output_states);
        last_output_states = output_states;
        delay(24);  // this sucks dickkkkkk
        hal_outputs_update_all();
    }
}

mm_power pwr_output[POWER_OUTPUT_NUM] = {
    mm_power(PWM_CH_1_0, POWER_OUTPUT_IGNITION),            // 0 Igntion
    mm_power(PWM_CH_2_1, POWER_OUTPUT_SIGNAL_LEFT),         // 1 Signal Left
    mm_power(PWM_CH_3_1, POWER_OUTPUT_SIGNAL_RIGHT),        // 2 Signal Right
    mm_power(PWM_CH_2_0, POWER_OUTPUT_HEADLIGHT),           // 3 Headlight
    mm_power(PWM_CH_BRAKE, POWER_OUTPUT_BRAKE_LIGHT),       // 4 Brake Light
    mm_power(PWM_CH_4_0, POWER_OUTPUT_HIGHBEAM),            // 5 High Beam
    mm_power(PWM_CH_3_0, POWER_OUTPUT_TAIL_LIGHT),          // 6 Tail Light
    mm_power(PWM_CH_0_1, POWER_OUTPUT_HORN),                // 7 Horn
    mm_power(PWM_CH_0_0, POWER_OUTPUT_AUXILLARY),           // 8 Auxilllary
    mm_power(PWM_CH_STARTER, POWER_OUTPUT_STARTER),         // 9 Starter
    mm_power(PWM_CH_1_1, POWER_OUTPUT_COMPRESSOR)           // 10 Compressor
};