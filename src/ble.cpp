#include "ble.h"
#include "hal.h"

void ble_class::init(void) {

    BLE_SERIAL.begin(1000000UL);        // 1Mbaud serial communication with BT840X NRF52840 module
    BLE_SERIAL.setTimeout(1);           // 1ms timeout for serial communication
}

bool ble_class::begin(void) {

    return true;
}

void ble_class::update(void) {

    if(BLE_SERIAL.available()  == 0) return;

    StaticJsonDocument<250> doc;

    // Unfortunately this won't work... resets json on every call, fucking dumb for a stream if you ask me..
    DeserializationError err = deserializeJson(doc, BLE_SERIAL);
    
    // If no error... process the json data
    if(err == DeserializationError::Ok) {

        switch((ble_msg_t)(doc["msg"])) {
            case BLE_MSG_BLE:
                ble_msg_callback(doc);
            break;

            case BLE_MSG_NFC:
                ble_nfc_callback(doc);
            break;
        }

    }

}



bool ble_class::ble_msg_callback(const JsonDocument &doc) {

    if( doc.containsKey("ble")) {
        LOG_INF("BLE JSON RECEIVED");
        return true;
    }
    else
        return false;
}

bool ble_class::ble_nfc_callback(const JsonDocument &doc) {
    if( doc.containsKey("nfc")) {
        LOG_INF("NFC JSON RECEIVED");
        return true;
    }
    else
        return false;
}

ble_class ble("ble");