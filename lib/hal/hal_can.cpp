#include "hal_can.h"
#include "hal.h"
#include <SPI.h>

ACAN2515 can (PIN_CAN_CS, CAN_SPI, PIN_CAN_INT) ;

static const uint32_t QUARTZ_FREQUENCY = 16 * 1000 * 1000 ; // 16 MHz

void hal_can_init(void) {

    CAN_SPI.begin();

    // need to enable 16MHz clock here as well...

    ACAN2515Settings settings (QUARTZ_FREQUENCY, 250 * 1000) ; // CAN bit rate 250 kb/s

    const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
    if (0 == errorCode) {
        LOG_INF("Can init ok") ;
    } else {
        LOG_ERR("Can init failed: 0x%0X", errorCode) ;
    }
}