#include "hal.h"
#include <Arduino.h>
#include <RTTStream.h>

#include "hal_adc.h"
#include "hal_can.h"

#include "hal_outputs.h"
#include "hal_imu.h"

#include <stdarg.h>

RTTStream rtt;

int8_t hal_wire_transfer(TwoWire *twi, uint8_t address, uint8_t reg, uint8_t *data, size_t len, bool read) {
    
    twi->beginTransmission(address);

    if (read) {
        if(len > 0) {
            twi->write(reg);
            twi->endTransmission();
            twi->requestFrom(address, len);
            twi->readBytes(data, len);
        }
        else {
            return twi->endTransmission();
        }
    } else {
        twi->write(reg);
        if(len > 0)
            twi->write(data, len);
        return twi->endTransmission();
    }

    return 0;
}

void hal_wire_scanner(TwoWire *twi) {
    uint8_t error, address;
    int nDevices;

    rtt.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ ) {
        twi->beginTransmission(address);
        error = twi->endTransmission();

        if (error == 0) {
            rtt.print("I2C device found at address 0x");
            if (address<16) {
                rtt.print("0");
            }
            rtt.print(address,HEX);
            rtt.println("  !");

            nDevices++;
        } else if (error==4) {
            rtt.print("Unknown error at address 0x");
            if (address<16) {
                rtt.print("0");
            }
            rtt.println(address,HEX);
        }    
    }
    if (nDevices == 0) {
        rtt.println("No I2C devices found\n");
    } else {
        rtt.println("done\n");
    }
}

// Output GLCK3 8Mhz clock on PA17
void hal_can_clock_init(void) {

    PORT->Group[0].PINCFG[17].reg = PORT_PINCFG_PMUXEN;
    PORT->Group[0].PMUX[17 >> 1].reg |= PORT_PMUX_PMUXO_H;  // GCLK_IO[3]

    GCLK->GENDIV.reg = GCLK_GENDIV_ID( 3 ) ; // Generic Clock Generator 3
    
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 3 ) |          // Generic Clock Generator 3
                        GCLK_GENCTRL_IDC |              // Improve duty-cycle to 50%
                        GCLK_GENCTRL_SRC_OSC8M |        // Selected source is RC OSC 8MHz (already enabled at reset)
                        GCLK_GENCTRL_OE |               // Output clock to a pin for tests
                        GCLK_GENCTRL_GENEN ;

    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
    {
      /* Wait for synchronization */
    }
}

// Output GLCK4 8Mhz clock on PA10
void hal_adc_clock_init(void) {
    // hal_sam_tcc_pwm_init();

    PORT->Group[0].PINCFG[10].reg = PORT_PINCFG_PMUXEN;             // Switch on port pin PA10's multiplexer
    PORT->Group[PORTA].PMUX[10 >> 1].reg |= PORT_PMUX_PMUXE_H;      // Switch the PA10's port multiplexer to GCLK IO  

    GCLK->GENDIV.reg = GCLK_GENDIV_ID( 4 ) ; // Generic Clock Generator 4

    GCLK->GENCTRL.reg =     GCLK_GENCTRL_ID(4) |            // Set the GCLK ID to GCLK4
                            GCLK_GENCTRL_IDC |              // Improve duty-cycle to 50%
                            GCLK_GENCTRL_SRC_OSC8M |        // Selected source is RC OSC 8MHz (already enabled at reset)
                            GCLK_GENCTRL_OE |               // Output clock to a pin for tests
                            GCLK_GENCTRL_GENEN ;
                            

    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
    {
      /* Wait for synchronization */
    }

}


void hal_init(void) {

    hal_can_clock_init();

    hal_adc_clock_init();
    
    rtt.begin(115200);

    Serial.begin(115200);

    LCD_SERIAL.begin(1000000UL);

    LOG_INF("\n***************** CB350 *****************");

    hal_outputs_init();

    SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN; // Enable temperature sensor

    // hal_led_chain_init();        // at the moment unused, but will be used for the LED data possibly

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

void hal_info_print(const char format[], ...) {

    char buffer[80];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buffer, sizeof(buffer), format, ap);
    Serial.print("INF: ");
    Serial.println(buffer);
    rtt.print("INF: ");
    rtt.println(buffer);
    va_end(ap);
}

void hal_error_print(const char format[], ...) {

    char buffer[80];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buffer, sizeof(buffer), format, ap);
    Serial.print("ERR: ");
    Serial.println(buffer);
    rtt.print("ERR: ");
    rtt.println(buffer);
    va_end(ap);
}

void hal_rtt_print(char * buffer) {
    rtt.println(buffer);
}

void hal_error_print(const char format[], ...);