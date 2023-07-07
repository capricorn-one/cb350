#include "hal.h"
#include <Arduino.h>

#include "hal_adc.h"
#include "hal_can.h"
#include "hal_lcd.h"

#include "hal_outputs.h"
#include "hal_imu.h"

int8_t hal_wire_transfer(TwoWire *twi, uint8_t addr, uint8_t reg, uint8_t *data, size_t len, bool read) {
    int8_t ret = 0;

    twi->beginTransmission(addr);
    twi->write(reg);
    if (read) {
        twi->endTransmission(false);
        twi->requestFrom(addr, len);
        for (size_t i = 0; i < len; i++) {
            data[i] = twi->read();
        }
    } else {
        twi->write(data, len);
        ret = twi->endTransmission();
    }

    return ret;
}


void hal_init(void) {
    
    digitalWrite(PIN_ADC_CS, HIGH);
    digitalWrite(PIN_CAN_CS, HIGH);
    digitalWrite(PIN_LCD_CS, HIGH);

    digitalWrite(PIN_ADC_SYNC_RESET, HIGH);
    digitalWrite(PIN_LCD_RESET, HIGH);

    pinMode(PIN_ADC_CS, OUTPUT);
    pinMode(PIN_CAN_CS, OUTPUT);
    pinMode(PIN_LCD_CS, OUTPUT);
    
    pinMode(PIN_ADC_SYNC_RESET, OUTPUT);
    pinMode(PIN_LCD_RESET, OUTPUT);


    pinMode(PIN_ADC_DRDY, INPUT);
    pinMode(PIN_IMU_INT, INPUT);
    pinMode(PIN_CAN_INT, INPUT);
    pinMode(PIN_LCD_INT, INPUT);
    pinMode(PIN_PARK, INPUT);
    pinMode(PIN_KICKSTAND, INPUT);

    Serial.begin(9600);

    while(!Serial);

    hal_adc_init();

    hal_can_init();

    hal_lcd_init();;

    hal_outputs_init();

    hal_imu_init();

    // hal_led_chain_init();

}

void hal_delay_ms(uint32_t ms) {
    delay(ms);
}

void hal_delay_us(uint32_t us) {
    delayMicroseconds(us);
}

uint32_t hal_millis(void) {
    return millis();
}