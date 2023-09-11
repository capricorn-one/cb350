#ifndef _MM_POWER_H
#define _MM_POWER_H

#include <stdint.h>
#include <stdbool.h>
#include "mm_types.h"

/* The power class handles the power output channels and telemetry for updating and reading from the
    higher powered output channels, controlled via PWM driver and the high current output drivers
*/

class mm_power {

public:

    mm_power(uint8_t pwm_channel, uint8_t pwr_output_num);

    void set(bool state);
    void set(bool state, bool immediate);
    void set_duty_cycle(float duty_cycle);

    bool isEnabled(void) { return enabled; }

    float   get_current(void) { return current; }
    void    set_current(float value) { current = value; }

    static void update_all(void);

private:

    uint8_t channel;
    uint8_t output_num;

    float current;

    bool enabled;

    static uint16_t output_states;
    static uint16_t last_output_states;

};

extern mm_power pwr_output[POWER_OUTPUT_NUM];

#endif // _MM_POWER_H