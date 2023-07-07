#ifndef _HAL_H_
#define _HAL_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <Arduino.h>
#include <Wire.h>

#define LOG_INF(...)    Serial.printf(__VA_ARGS__)

#define LOG_ERR(...)    Serial.printf(__VA_ARGS__)


    void hal_init(void);

    void hal_delay_ms(uint32_t ms);

    void hal_delay_us(uint32_t us);

    uint32_t hal_millis(void);

    
    int8_t hal_wire_transfer(TwoWire *twi, uint8_t addr, uint8_t reg, uint8_t *data, size_t len, bool read);


#endif