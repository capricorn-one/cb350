#ifndef OUTPUT_H
#define OUTPUT_H

#include <stdint.h>
#include <stdbool.h>
#include "pca9685.h"

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
} output_channel_enum_t;

class outputClass { 

public:

    outputClass(uint8_t channel_num) { channel = channel_num; }

    void set(bool newState);

    void set_duty_cycle(double duty_cycle);
    
    bool get(void) { return state; }
    
private:

    uint8_t channel;

    bool state;

    uint16_t pwm;

};

extern outputClass outputs[16];

#endif