#include "hal_adc.h"
#include "hal.h"
#include "utility/dma.h"
#include "sam.h"
#include "ATSAMD21_ADC.h"
#include <SPI_DMA.h>


/*********** static variables / data storage ************/
ads131m0x_hal_t adc_hal;

uint8_t adc_tx_buffer[FRAME_LENGTH];
uint8_t adc_rx_buffer[FRAME_LENGTH];

/********* HAL functions **********/
static void adc_set_syncResetPin(bool state) {
    digitalWrite(PIN_ADC_SYNC_RESET, state);
}

// Used for communicating with the chip, not for continuous reading
static void adc_transfer_frame(uint8_t frame_length) {

    ADC_SPI.waitForTransfer();
    
    digitalWrite(PIN_ADC_CS, LOW);
    ADC_SPI.transfer(adc_tx_buffer, adc_rx_buffer, frame_length, true);     //blocking
    digitalWrite(PIN_ADC_CS, HIGH);
}


/****** CONTINUOUS READ DMA CALLBACK FUNCTIONS**********/

// called by drdy pin falling, meaning data is ready to be read
// non-blocking DMA SPI transaction, record timestamp so we know when to check back for data
static void drdy_pin_change_callback(void) {
    
    digitalWrite(PIN_ADC_CS, LOW);

    ADC_SPI.transfer(adc_tx_buffer, adc_rx_buffer, FRAME_LENGTH, false);

}

static volatile bool new_data_flag = false;

static void dma_complete_callback(void) {

    digitalWrite(PIN_ADC_CS, HIGH);

    new_data_flag = true;

}

/*************** INITIALIZATION ROUTINES ******************/

void adc_configure_spi(void) {

    pinMode(PIN_ADC_CS, OUTPUT);
    digitalWrite(PIN_ADC_CS, HIGH);

    ADC_SPI.begin();
    ADC_SPI.beginTransaction(SPISettings(12000000UL, MSBFIRST, SPI_MODE1));
}

void drdy_pin_callback_enable(bool state) {
    if(state) {

        ADC_SPI.set_receive_complete_callback(&dma_complete_callback);

        pinMode(PIN_ADC_DRDY, INPUT);
        attachInterrupt(PIN_ADC_DRDY, drdy_pin_change_callback, FALLING);
    }
    else {

        ADC_SPI.set_receive_complete_callback(NULL);

        detachInterrupt(PIN_ADC_DRDY);
    }
}

bool hal_adc_init(adc_conversion_t * adc_raw_data_ptr) {

    LOG_INF("Initializing ADC");

    pinMode(PIN_ADC_DRDY, INPUT);

    digitalWrite(PIN_ADC_SYNC_RESET, HIGH);
    pinMode(PIN_ADC_SYNC_RESET, OUTPUT);

    adc_configure_spi();

    memset(adc_tx_buffer, 0, sizeof(adc_tx_buffer));       // clear transmit buffer
    memset(adc_rx_buffer, 0, sizeof(adc_rx_buffer));       // clear receive buffer

    adc_hal.delay_ms = &hal_delay_ms;
    adc_hal.delay_us = &hal_delay_us;
    adc_hal.millis = &hal_millis;

    adc_hal.set_syncResetPin = &adc_set_syncResetPin;
    adc_hal.transferFrame = &adc_transfer_frame;

    adc_hal.tx_buffer = adc_tx_buffer;      // if using single buffer, point tx and rx to same buffer
    adc_hal.rx_buffer = adc_rx_buffer;

    adc_hal.conversion = adc_raw_data_ptr;

    adc_hal.clock =     CLOCK_CH0_EN_ENABLED |
                        CLOCK_CH1_EN_ENABLED | 
                        CLOCK_CH2_EN_ENABLED | 
                        CLOCK_CH3_EN_ENABLED | 
                        CLOCK_CH4_EN_ENABLED | 
                        CLOCK_CH5_EN_ENABLED | 
                        CLOCK_CH6_EN_ENABLED | 
                        CLOCK_CH7_EN_ENABLED | 
                        CLOCK_XTAL_DISABLED |
                        CLOCK_EXTREF_DISABLED | 
                        CLOCK_OSR_16256 | 
                        CLOCK_PWR_HR;

    adc_hal.config = CFG_GC_DLY_65536 | CFG_GC_EN_ENABLED;

    adc_hal.gain[ADC_STARTER_CURRENT] = ADS131M0X_GAIN_1X; // 1x
    adc_hal.gain[ADC_REGULATOR_CURRENT] = ADS131M0X_GAIN_1X; // 1x
    adc_hal.gain[ADC_LOAD_CURRENT] = ADS131M0X_GAIN_1X; // 1x
    adc_hal.gain[ADC_IS0_CURRENT] = ADS131M0X_GAIN_4X; // 8x
    adc_hal.gain[ADC_IS1_CURRENT] = ADS131M0X_GAIN_4X; // 4x
    adc_hal.gain[ADC_IS2_CURRENT] = ADS131M0X_GAIN_4X; // 4x
    adc_hal.gain[ADC_IS3_CURRENT] = ADS131M0X_GAIN_4X; // 4x
    adc_hal.gain[ADC_IS4_CURRENT] = ADS131M0X_GAIN_4X; // 4x

    uint16_t id = ads131m0x_init(&adc_hal);

    LOG_INF("ADC Status: 0x%04X", id);

    /**** configure local adc settings ***/
    // analogCalibrate();
    // analogReference2(ADC_REF_INT1V);           // internal 1.0V reference
    // // analogReferenceCompensation(false);          // compensate for VREFA drift
    // analogPrescaler(ADC_PRESCALER_DIV32);      // 48MHz / 128 = 375kHz
    // analogGain(ADC_GAIN_1);                     // 1x gain
    // analogReadExtended(ADC_RESOLUTION);         // 14 bit resolution

    // // Disable DAC
    // syncDAC();
    // DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
    // syncDAC();

    // pinPeripheral(ADC_SAM_BATTERY_POS, PIO_ANALOG); // set pins to analog mode
    // pinPeripheral(ADC_SAM_BATTERY_NEG, PIO_ANALOG);

    return true;
}

void hal_adc_reset(void) {
    ads131m0x_reset(&adc_hal);
}

void hal_adc_sleep(void) {
    ads131m0x_standby(&adc_hal);
}

void hal_adc_wakeup(void) {
    ads131m0x_wakeup(&adc_hal);
}

void hal_adc_clock_enable(bool state) {

}

void hal_adc_continous_read(bool state) {
    if(state) {
        memset(adc_tx_buffer, 0, sizeof(adc_tx_buffer));        // clear transmit buffer
        drdy_pin_callback_enable(true);
    }
    else {
        drdy_pin_callback_enable(false);
    }
}

uint16_t hal_adc_read_continous_conversion(bool wait_for_new_data) {
    
    uint16_t status = 0xFFFF;
    
    if( wait_for_new_data == true ) {
        while(new_data_flag == false);
    }
        
    status = ads131m0x_process_new_conversion(&adc_hal);

    new_data_flag = false;

    // for(uint8_t i=0; i<8; i++) {
    //     LOG_INF("%u\t0x%08X\t%.6f", i, adc_hal.conversion[i].raw, hal_adc_get_conversion(i, 1.430511474609375e-7));
    // }

    return status;
}

// This function is necessary if continous reads are not happening (if DRDY interrupt is not available most likely)
// In order to get the most recent data, the SYNC pulse can be used followed by waiting for DRDY to go low
uint16_t hal_adc_read_conversion_immediate(void) {

    uint16_t status = 0xFFFF;

    ads131m0x_resync(&adc_hal);

    if(digitalRead(PIN_ADC_DRDY) == LOW) {
        while(digitalRead(PIN_ADC_DRDY) == LOW);
    }

    while(digitalRead(PIN_ADC_DRDY) == HIGH);

    ads131m0x_start_conversion(&adc_hal);

    status = ads131m0x_process_new_conversion(&adc_hal);

    // LOG_INF("ADC Status: 0x%04X", status);
    // for(uint8_t i=0; i<8; i++) {
    //     LOG_INF("%u\t0x%08X\t%.6f", i, adc_hal.conversion[i].raw, hal_adc_get_conversion(i, 1.430511474609375e-7));
    // }

    return status;
}

void hal_adc_synchronize(void) {
    ads131m0x_resync(&adc_hal);
}

void hal_adc_enable_channels(uint8_t channel_bitmap) {
    ads131m0x_enable_channels(&adc_hal, channel_bitmap);
}

void hal_adc_disable_channels(uint8_t channel_bitmap) {
    ads131m0x_disable_channels(&adc_hal, channel_bitmap);
}

void hal_adc_external_reference(bool state) {
    if(state)
        ads131m0x_enable_external_reference(&adc_hal);
    else
        ads131m0x_disable_external_reference(&adc_hal);
}

uint8_t hal_adc_channel_gain_get(uint8_t channel) {
    if(channel >= HAL_ADC_CH_NUM)
        return 0;
    else
        return (uint8_t)adc_hal.gain[channel];
}

int32_t hal_adc_get_raw_conversion(uint8_t channel) {
    if(channel >= HAL_ADC_CH_NUM)
        return 0;
    else
        return adc_hal.conversion[channel].raw;
}

static float gain_factor[8] = {
    1.4305114746e-7,        // 1x - 1.2V / 8388608 = 1.430511474609375e-7
    7.1525573730e-8,        // 2x - 0.6V / 8388608 = 7.152557373046875e-8
    3.5762786865e-8,        // 4x - 0.3V / 8388608 = 3.5762786865234375e-8
    1.7881393433e-8,        // 8x - 0.15V / 8388608 = 1.78813934326171875e-8
    8.9406967163e-9,        // 16x - 0.075V / 8388608 = 8.94069671630859375e-9
    4.4703483581e-9,        // 32x - 0.0375V / 8388608 = 4.470348358154296875e-9
    2.2351741791e-9,        // 64x - 0.01875V / 8388608 = 2.2351741790771484375e-9
    1.1175870895e-9         // 128x - 0.009375V / 8388608 = 1.11758708953857421875e-9
};

float hal_adc_get_conversion(uint8_t channel, float conversion_factor) {
    if(channel >= HAL_ADC_CH_NUM)
        return 0;
    else
        return (float)adc_hal.conversion[channel].raw * gain_factor[adc_hal.gain[channel]] * conversion_factor;
}

void hal_adc_channel_gain_set(uint8_t channel, uint8_t pga) {
    if(channel >= HAL_ADC_CH_NUM)
        return;

    adc_hal.gain[channel] = pga;

    ads131m0x_channel_pga_update(&adc_hal);
}

void hal_adc_channel_gain_set_all(uint32_t gain_settings_packed) {

    for(uint8_t i=0; i<8; i++) {
        adc_hal.gain[i/4] = (uint8_t)( (gain_settings_packed>>(i*4)) & 0x07);
    }

    ads131m0x_channel_pga_update(&adc_hal);
}

uint16_t hal_adc_read_id(void) {
    return ads131m0x_read_id(&adc_hal);
}

uint16_t hal_adc_read_status(void) {
    return ads131m0x_read_status(&adc_hal);
}

void hal_adc_set_power_mode(adc_power_mode_t power_mode) {

    switch(power_mode) {
        // case ADC_POWER_MODE_STANDBY:
        //     ads131m0x_set_power_mode(&adc_hal, ADS131M0X_POWER_MODE_STANDBY);
        //     break;
        // case ADC_POWER_MODE_NORMAL:
        //     ads131m0x_set_power_mode(&adc_hal, ADS131M0X_POWER_MODE_NORMAL);
        //     break;
        // case ADC_POWER_MODE_DUTY_CYCLE:
        //     ads131m0x_set_power_mode(&adc_hal, ADS131M0X_POWER_MODE_DUTY_CYCLE);
        //     break;
        // case ADC_POWER_MODE_DUTY_CYCLE_FAST:
        //     ads131m0x_set_power_mode(&adc_hal, ADS131M0X_POWER_MODE_DUTY_CYCLE_FAST);
        //     break;
        // case ADC_POWER_MODE_DUTY_CYCLE_TURBO:
        //     ads131m0x_set_power_mode(&adc_hal, ADS131M0X_POWER_MODE_DUTY_CYCLE_TURBO);
        //     break;
        // case ADC_POWER_MODE_DUTY_CYCLE_TURBO_FAST:
        //     ads131m0x_set_power_mode(&adc_hal, ADS131M0X_POWER_MODE_DUTY_CYCLE_TURBO_FAST);
        //     break;
        default:
            break;
    }

}

// The battery voltage is a differential measurement in a voltage divider of 3 resistors. 1MΩ, 33k, and 100k.
// Therefore the conversion of input voltage to differnital voltage 29.1262mV per volt
// The local voltage conversion max voltage is 1.25V resulting in 1.25 / .0291262 = 42.9V max
// So each bit of 14 bit resolution is 42.9 / 2^14 = 2.624512e-3
// battery_voltage = hal_adc_sam_differential_read(ADC_SAM_BATTERY_POS, ADC_SAM_BATTERY_NEG) * 2.624512e-3;
float hal_adc_battery_read(void) {
    return analogDifferential(ADC_SAM_BATTERY_POS, ADC_SAM_BATTERY_NEG) / float( (1<<(ADC_RESOLUTION - 1))-1) * 1.000 / 0.0291262;     // where 1.000 is voltage reference
}

// IS0-n current outputs current with a ratio of 1:9500, so 1mA = 9.5A. A load resistor of 100 ohms is used to generate the voltage
// on the pin, so a 1mA current will generate 100mV on the pin. Working backwards in volts using a 1.2V reference means
// a 1.2V signal = 12mA, with a ratio of 9500:1, so 12mA * 9500 = 114A. 1.2V / 114A = 10.52631578947368mV/A
// Future versions should change R(IS) to 1kΩ to increase the signal
// For a 1.2V signal, or maximum current to be read is 1.2V / 100Ω = 12mA * 52100 = 625.2A. 1.2V / 625.2A
// At max value of 625.2A and 14 bit ressolution, each bit = 625.2A / 2^14 = 38.14697265625e-3 Amps per bit
// starter_current_local = hal_adc_sam_analog_read(ADC_SAM_STARTER_CURRENT) * 38.147e-3;
float hal_adc_starter_current_read(void) {
    return analogRead(ADC_SAM_STARTER_CURRENT) / float( (1<<(ADC_RESOLUTION))-1) * 1.000;     // where 1.000 is voltage reference
}

// This voltage is scaled at 1/4 of vccio. At max 1.25V reference, each bit of 14 bit resolution is 1.25 / 2^14 = 7.62939453125e-5
// Since we're measuring the voltage divided by 4, we need to multiply by 4 for the correct Vcc value.
// This means 1.25 / 2^14 * 4 = 0.00030517578125 per bit.
// uController_vcc = hal_adc_sam_vcc_read() * 3.05176e-4;
float hal_adc_vcc_read(void) {
    return analogDifferentialRaw(ADC_PIN_SCALEDIOVCC , ADC_PIN_IOGND)  / float( (1<<(ADC_RESOLUTION - 1))-1) * 1.000 * 4.00;     // where 1.000 is voltage reference
}
//CMIN + (Vmes+ − VoutMAX) (∆temperature/∆voltage))


// From section 17.6.10 of the ATSAMD21 datasheet, the internal temperature sensor is 0.688V at 25C and has a slope of 2.16mV/°C
// To convert to voltage, use 1.25V / 2^14 = 7.62939453125e-5 per bit
// internal_temp = (hal_adc_sam_internal_temp_read() * 7.62939453125e-5 - 0.688) / 0.00216 + 25.0;
float hal_adc_internal_temp_read(void) {
     return (analogDifferentialRaw(ADC_PIN_TEMP, ADC_PIN_GND) / float( (1<<(ADC_RESOLUTION - 1))-1) * 1.000 - 0.688) / .00216 + 25.0;     // where 1.000 is voltage reference
}