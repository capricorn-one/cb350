#ifndef _HAL_OUTPUTS_H_
#define _HAL_OUTPUTS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

    void hal_outputs_init(void);

    int8_t hal_outputs_transfer(uint8_t addr, uint8_t reg, uint8_t *data, size_t len, bool read);

#endif /* _HAL_OUTPUTS_H_ */