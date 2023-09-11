#include "hal_eeprom.h"
#include "hal.h"
#include <Wire.h>
#include <extEEPROM.h>

extEEPROM myEEPROM(kbits_256, 1, 64, 0x57);


bool hal_eeprom_init(void) {

    // Wire.begin();    // shares Wire with IMU

    uint8_t i2cStat = myEEPROM.begin(myEEPROM.twiClock400kHz);
    if ( i2cStat != 0 ) {
        LOG_ERR("I2C Problem");
        return false;
    }

    // SerialUSB.println(F("EEPROM Memory commands:  read:(a)(l)(r) , write:(a)(d)(w), next read data (n)"));
    // SerialUSB.println(F("- Commands TO PRESS:"));
    // SerialUSB.println(F("\t a : memory address to read / write"));
    // SerialUSB.println(F("\t d : data to write"));
    // SerialUSB.println(F("\t l : data to write"));
    // SerialUSB.println(F("\t r : read command"));
    // SerialUSB.println(F("\t w : write command"));


    return true;
}