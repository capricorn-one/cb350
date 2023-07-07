#include "hal_adc.h"
#include "hal.h"
#include <SPI.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"

#include "../ads131m0x/ads131m0x.h"


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

void hal_adc_init(void) {
	
	adc_configure_spi();
	
	adc_configure_drdy_interrupt();

    adc_hal.delay_ms = &hal_delay_ms;
    adc_hal.delay_us = &hal_delay_us;
    adc_hal.set_syncResetPin = &adc_set_syncResetPin;
    adc_hal.dataReady = &adc_dataReady;
    adc_hal.transferFrame = &adc_transfer_frame;

}

void hal_adc_update(void) {
    
    // ads131m0x_get_new_data(&adc_hal);
}