#include "hal_can.h"
#include "hal.h"
#include <SPI.h>

void hal_can_init(void) {

    CAN_SPI.begin();
}