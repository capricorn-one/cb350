#ifndef __BLE_H
#define __BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "moto_task.h"
#include <ArduinoJson.h>

typedef enum {
    BLE_MSG_BLE = 0,
    BLE_MSG_NFC = 1,
} ble_msg_t;

class ble_class : public moto_task {

public:

    ble_class(const char *name) : moto_task(name, 100) {}
    ble_class(const char *name, unsigned long aInterval) : moto_task(name, aInterval) {}

protected:

    bool begin(void);
    bool start(void) { return true; }
    bool update(void);
    void exit(void) {}

private:


    bool ble_msg_callback(const JsonDocument &doc);
    bool ble_nfc_callback(const JsonDocument &doc);
    

};

extern ble_class ble;

#endif // __BLE_H