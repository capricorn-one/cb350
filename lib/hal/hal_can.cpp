#include "hal_can.h"
#include "hal.h"
#include <SPI.h>
#include <ACAN2515.h>
#include "mm_types.h"

ACAN2515 can (PIN_CAN_CS, CAN_SPI, PIN_CAN_INT) ;

static const uint32_t QUARTZ_FREQUENCY = 16 * 1000 * 1000 ; // 16 MHz

static void (*handlebar_callback_func_ptr)(uint16_t *);
static void (*vfd_callback_func_ptr)(uint16_t *);

static void mm_handlebar_swtich_states_callback_func(const CANMessage &message) {
    // LOG_INF("Received handlebar switch states");
    if(handlebar_callback_func_ptr != NULL) {
        handlebar_callback_func_ptr((uint16_t *)message.data);
    }
}

static void mm_vfd_telemetry_callback_func(const CANMessage &message) {
    // LOG_INF("Received VFD telemetry");
    if(vfd_callback_func_ptr != NULL) {
        vfd_callback_func_ptr((uint16_t *)message.data);
    }
}

void hal_can_init(void) {

    CAN_SPI.begin();

    // need to enable 16MHz clock here as well...

    ACAN2515Settings settings (QUARTZ_FREQUENCY, 250 * 1000) ; // CAN bit rate 250 kb/s

    settings.mRequestedMode = ACAN2515Settings::NormalMode;

    const ACAN2515Mask rxm0 = standard2515Mask (MM_CAN_ID_MASK, 0xFF, 0xFF) ; // last two param are data mask bytes
    
    const ACAN2515AcceptanceFilter filters [] = {
        {standard2515Filter (MM_CAN_ID_HANDLEBAR_SWITCH_STATES, 0xFF, 0xFF), mm_handlebar_swtich_states_callback_func},
        {standard2515Filter (MM_CAN_ID_VFD_TELEM, 0xFF, 0xFF), mm_vfd_telemetry_callback_func}
    } ;


    const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }, rxm0, filters, 2) ;
    if (0 == errorCode) {
        LOG_INF("Can init ok") ;
    } else {
        LOG_ERR("Can init failed: 0x%0X", errorCode) ;
    }
}

void hal_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    CANMessage frame ;
    frame.id = id ;
    frame.len = len ;
    memcpy (frame.data, data, len) ;
    const uint32_t errorCode = can.tryToSend (frame) ;
    if (0 == errorCode) {
        LOG_INF("Can send ok") ;
    } else {
        LOG_ERR("Can send failed: 0x%0X", errorCode) ;
    }
}

void hal_register_handlebar_callback(void (*callback)(uint16_t *data)) {
    handlebar_callback_func_ptr = callback;
}

void hal_register_vfd_callback(void (*callback)(uint16_t *data)) {
    vfd_callback_func_ptr = callback;
}