#include "hal_adc.h"
#include "hal.h"
#include <SPI.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"
#include "ATSAMD21_ADC.h"



/*********** static variables / data storage ************/
ads131m0x_hal_t adc_hal;

Adafruit_ZeroDMA adc_zeroDMA;
ZeroDMAstatus    adc_dmaStat; // DMA status codes returned by some functions

volatile bool adc_drdy_flag = false;

volatile bool adc_transfer_done = false;


/*********** callback functions for hal ****************/
static void adc_drdy_pin_change_callback(void) {
	adc_drdy_flag = true;
}

static void adc_dma_transfer_callback(Adafruit_ZeroDMA *dma) {
    (void) dma;
    adc_transfer_done = true;

    digitalWrite(PIN_ADC_CS, HIGH);
}

static void adc_set_syncResetPin(bool state) {

    digitalWrite(PIN_ADC_SYNC_RESET, state);
	
	adc_drdy_flag = false;
}


/* The DRDY pin is an active low output that indicates when new conversion data are ready in conversion mode or
that the requirements are met for current detection when in current-detect mode. Connect the DRDY pin to a
digital input on the host to trigger periodic data retrieval in conversion mode.
*/
static bool adc_dataReady(void) {
	return adc_drdy_flag;
}


static void adc_transfer_frame(void) {

    while(!adc_transfer_done) {
        // wait for transfer to complete
    }
	
	digitalWrite(PIN_ADC_CS, LOW);

    // while(length--) {
    //     *rx++ = SPI.transfer(*tx++);
    // }
    adc_dmaStat = adc_zeroDMA.startJob();

	// if(blocking) {
    //     while(!adc_transfer_done) {
    //         // wait for transfer to complete
    //     }
    // }
    // SPI.endTransaction();
}

/*************** INITIALIZATION ROUTINES ******************/

static void adc_configure_spi(void) {
	
    ADC_SPI.begin();

    adc_zeroDMA.setTrigger(SERCOM0_DMAC_ID_TX);

    adc_zeroDMA.setAction(DMA_TRIGGER_ACTON_BEAT);

    adc_dmaStat = adc_zeroDMA.allocate();

    adc_zeroDMA.addDescriptor(
        adc_hal.transfer_buffer,                    // move data from here
    #ifdef __SAMD51__
        (void *)(&SERCOM2->SPI.DATA.reg), // to here (M4)
    #else
        (void *)(&SERCOM0->SPI.DATA.reg), // to here (M0)
    #endif
        FRAME_LENGTH,                      // this many...
        DMA_BEAT_SIZE_BYTE,               // bytes/hword/words
        true,                             // increment source addr?
        false);                           // increment dest addr?

    adc_zeroDMA.setCallback(adc_dma_transfer_callback);
	
    // Placing here since we are not sharing the SPI bus with any other ICs
    ADC_SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}

static void adc_configure_drdy_interrupt(void) {

    pinMode(PIN_ADC_DRDY, INPUT);

    attachInterrupt(PIN_ADC_DRDY, adc_drdy_pin_change_callback, FALLING);
}

void hal_adc_init(adc_conversion_t * adc_raw_data_ptr) {
	
	adc_configure_spi();
	
	adc_configure_drdy_interrupt();

    adc_hal.delay_ms = &hal_delay_ms;
    adc_hal.delay_us = &hal_delay_us;
    adc_hal.set_syncResetPin = &adc_set_syncResetPin;
    adc_hal.dataReady = &adc_dataReady;
    adc_hal.transferFrame = &adc_transfer_frame;

    adc_hal.conversion = adc_raw_data_ptr;

    adc_hal.clock =   CLOCK_CH0_EN_ENABLED |
                                    CLOCK_CH1_EN_ENABLED | 
                                    CLOCK_CH2_EN_ENABLED | 
                                    CLOCK_CH3_EN_ENABLED | 
                                    CLOCK_CH4_EN_ENABLED | 
                                    CLOCK_CH5_EN_ENABLED | 
                                    CLOCK_CH6_EN_ENABLED | 
                                    CLOCK_CH7_EN_ENABLED | 
                                    CLOCK_XTAL_DISABLED |
                                    CLOCK_EXTREF_ENABLED | 
                                    CLOCK_OSR_16256 | 
                                    CLOCK_PWR_HR;

    adc_hal.config = CFG_GC_DLY_16 | CFG_GC_EN_ENABLED;

    ads131m0x_init(&adc_hal);

    /**** configure local adc settings ***/

    analogReference2(ADC_REF_VREFA);            // 1.25V reference used for ADS131M0x
    analogReferenceCompensation(true);          // compensate for VREFA drift
    analogPrescaler(ADC_PRESCALER_DIV128);      // 48MHz / 128 = 375kHz
    analogGain(ADC_GAIN_1);                     // 1x gain
    analogReadExtended(ADC_RESOLUTION);         // 14 bit resolution

}

void hal_adc_reset(void) {
    ads131m0x_reset(&adc_hal);
}

void hal_adc_sleep(void) {

}

void hal_adc_trigger_conversion(void) {
    ads131m0x_trigger_conversion(&adc_hal);
}

void hal_adc_get_new_conversions(void) {
    // ads131m0x_get_new_conversion(&adc_hal);      // we don't need this if using DMA and DRDY interrupt
    ads131m0x_parse_conversion(&adc_hal);
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


int16_t hal_adc_sam_differential_read(uint8_t pos_pin, uint8_t neg_pin) {
    return analogDifferential(pos_pin, neg_pin);
}

int16_t hal_adc_sam_analog_read(uint8_t pin) {
    return analogRead(pin);
}

int16_t hal_adc_sam_internal_temp_read(void) {
    return analogDifferentialRaw(ADC_PIN_TEMP, ADC_PIN_GND);
}

int16_t hal_adc_sam_vcc_read(void) {
    return analogDifferentialRaw(ADC_PIN_SCALEDIOVCC , ADC_PIN_GND);
}