#include "hal_outputs.h"
#include "hal.h"
#include <Wire.h>

void hal_outputs_init(void) {
    OUTPUTS_WIRE.begin();    

}

int8_t hal_outputs_transfer(uint8_t addr, uint8_t reg, uint8_t *data, size_t len, bool read) {
    return hal_wire_transfer(&OUTPUTS_WIRE, addr, reg, data, len, read);
}