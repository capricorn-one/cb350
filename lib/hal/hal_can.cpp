#include "hal_can.h"
#include "hal.h"
#include <SPI_DMA.h>
#include <ACAN2515.h>
#include "mm_types.h"

ACAN2515 can (PIN_CAN_CS, CAN_SPI, PIN_CAN_INT) ;

static const uint32_t QUARTZ_FREQUENCY = 8 * 1000 * 1000 ; // 8 MHz

static void (*handlebar_callback_func_ptr)(uint8_t *);
static void (*vfd_callback_func_ptr)(uint8_t *);

static bool can_ready = false;

static void mm_handlebar_swtich_states_callback_func(const CANMessage &message) {
    if(handlebar_callback_func_ptr != NULL) {
        handlebar_callback_func_ptr((uint8_t *)message.data);
    }
}

static void mm_vfd_telemetry_callback_func(const CANMessage &message) {
    if(vfd_callback_func_ptr != NULL) {
        vfd_callback_func_ptr((uint8_t *)message.data);
    }
}

bool hal_can_init(void) {

    // Assumes clock already configured...

    CAN_SPI.begin();


    ACAN2515Settings settings (QUARTZ_FREQUENCY, 500 * 1000) ; // CAN bit rate 500 kb/s

    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    settings.mReceiveBufferSize = 16;


    const ACAN2515Mask rxm0 = standard2515Mask (MM_CAN_ID_MASK, 0x00, 0x00) ; // last two param are data mask bytes
    
    const ACAN2515AcceptanceFilter filters [] = {
        {standard2515Filter (MM_CAN_ID_HANDLEBAR_SWITCH_STATES, 0x00, 0x00), mm_handlebar_swtich_states_callback_func},
        {standard2515Filter (MM_CAN_ID_VFD_TELEM, 0x00, 0x00), mm_vfd_telemetry_callback_func}
    } ;


    const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }, rxm0, filters, 2) ;

    if (0 == errorCode) {
        can_ready = true;
        return true;
    } else {
        can_ready = false;
        LOG_ERR("----Can init failed: 0x%0X", errorCode) ;
        return false;
    }
}

void hal_can_update(void) {
    can.dispatchReceivedMessage () ;
}

void hal_can_send(uint32_t id, uint8_t *data, uint8_t len) {

    static uint32_t gSentFrameCount = 0 ;

    if(!can_ready) {
        return;
    }
    CANMessage frame ;
    frame.id = id ;
    frame.len = len ;
    memcpy (frame.data, data, len) ;
    const bool ok = can.tryToSend (frame) ;
    if (!ok) {
        LOG_ERR("Can send failed") ;
    }
    // else
    // {
    //     gSentFrameCount += 1 ;
    //     LOG_INF("Sent: %u DATA 0x%02X 0x%02X", gSentFrameCount, frame.data[0], frame.data[1]) ;
    // }

}

void hal_register_handlebar_callback(void (*callback)(uint8_t *data)) {
    handlebar_callback_func_ptr = callback;
}

void hal_register_vfd_callback(void (*callback)(uint8_t *data)) {
    vfd_callback_func_ptr = callback;
}