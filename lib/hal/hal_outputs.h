#ifndef _HAL_OUTPUTS_H_
#define _HAL_OUTPUTS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

    void hal_outputs_init(void);

    void hal_outputs_set_state(uint8_t channel, bool state);

    void hal_outputs_set_duty_cycle(uint8_t channel, double duty_cycle);

#endif /* _HAL_OUTPUTS_H_ */