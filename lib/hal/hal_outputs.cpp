#include "hal_outputs.h"
#include "hal.h"
#include "pca9685.h"
#include <Wire.h>

pca9685 output_controller;
pca9685_hal_t pca9685_hal;

static int8_t hal_outputs_transfer(uint8_t addr, uint8_t reg, uint8_t *data, size_t len, bool read) {
    return hal_wire_transfer(&OUTPUTS_WIRE, addr, reg, data, len, read);
}

void hal_outputs_init(void) {

    pca9685_hal.transfer = hal_outputs_transfer;
    pca9685_hal.delay_ms = hal_delay_ms;

    OUTPUTS_WIRE.begin();
    OUTPUTS_WIRE.setClock(400000);

    output_controller.begin(&pca9685_hal);
}

void hal_outputs_set_state(uint8_t channel, bool state) {
    output_controller.set_state(channel, state);
}

void hal_outputs_set_duty_cycle(uint8_t channel, double duty_cycle) {
    output_controller.set_duty_cycle(channel, duty_cycle);
}

