#ifndef _MM_POWER_H
#define _MM_POWER_H

#include <stdint.h>
#include <stdbool.h>
#include "outputs.h"
#include "pca9685.h"
#include "mm_types.h"

/* The power class handles the power output channels and telemetry for updating and reading from the
    higher powered output channels, controlled via PWM driver and the high current output drivers
*/

typedef struct {
    outputClass *output;
    float current;
} power_output_t;

class mm_power {

public:

    mm_power(void);

    void init(void);

    void set(power_output_enum_t output_num, bool state);
    bool get(power_output_enum_t output_num);

    void set_duty_cycle(power_output_enum_t output_num, float duty_cycle);

    float   get_current(power_output_enum_t output_num) { return power_outputs[output_num].current; }
    void    set_current(power_output_enum_t output_num, float current) { power_outputs[output_num].current = current; }

private:

    power_output_t power_outputs[POWER_OUTPUT_NUM];

};

extern mm_power pwr_output;

#endif // _MM_POWER_H