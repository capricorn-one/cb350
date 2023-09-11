#ifndef _HAL_H_
#define _HAL_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <Arduino.h>
#include <Wire.h>

    void hal_init(void);

    void hal_delay_ms(uint32_t ms);

    void hal_delay_us(uint32_t us);

    uint32_t hal_millis(void);
    
    int8_t hal_wire_transfer(TwoWire *twi, uint8_t addr, uint8_t reg, uint8_t *data, size_t len, bool read);

    void hal_wire_scanner(TwoWire *twi);

    void hal_info_print(const char format[], ...);

    void hal_error_print(const char format[], ...);

    void hal_rtt_print(char * buffer);

    #define LOG_INF(...) hal_info_print(__VA_ARGS__)

    #define LOG_ERR(...) hal_error_print(__VA_ARGS__)

#endif