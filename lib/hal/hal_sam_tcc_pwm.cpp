#include "hal_sam_tcc_pwm.h"
#include "sam.h"


#define PWM_PIN_NUMBER  10      // PA10
#define PMW_PORT_NUMBER 0       // PORTA

void hal_sam_tcc_pwm_init() {

    // Configure output pin
    PORT->Group[PMW_PORT_NUMBER].PINCFG[PWM_PIN_NUMBER].reg = PORT_PINCFG_PMUXEN;

    if(PWM_PIN_NUMBER & 1U) // Odd numbered pin
    {
        PORT->Group[PMW_PORT_NUMBER].PMUX[PWM_PIN_NUMBER >> 1].bit.PMUXO = PORT_PMUX_PMUXO_E_Val;
    }
    else // Even numbered pin
    {
        PORT->Group[PMW_PORT_NUMBER].PMUX[PWM_PIN_NUMBER >> 1].bit.PMUXE = PORT_PMUX_PMUXE_E_Val;
    }

    // Select generator and write lock for TCC0
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC0_TCC1_Val) | GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK0_Val)  | GCLK_CLKCTRL_CLKEN;
    
    PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

    // Initialize TC3 as PWM to generate 8MHz clock
    // TCC0->COUNT8.CTRLA.reg = TCC_CTRLA_SWRST;
    TCC0->CTRLA.reg = TCC_CTRLA_SWRST;

    while(TCC0->SYNCBUSY.reg != 0U)
    {
        /* Wait for sync */
    }

    // TC3->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_WAVEGEN_NPWM;
    TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1;
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;

    // TC3->COUNT8.PER.reg = 5U;
    // TC3->COUNT8.CC[0].reg = 0U;
    // TC3->COUNT8.CC[1].reg = 3U;
    TCC0->PER.reg = 5U;
    TCC0->CC[0].reg = 3U;

    // TC3->COUNT8.INTFLAG.reg = TC_INTFLAG_MASK;

    while(TCC0->SYNCBUSY.reg != 0U)
    {
        /* Wait for sync */
    }
}

void hal_sam_tcc_pwm_start() {
    // TC3->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
    while(TCC0->SYNCBUSY.reg != 0U)
    {
        /* Wait for sync */
    }
}

void hal_sam_tcc_pwm_stop() {
    TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
    // TC3->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while(TCC0->SYNCBUSY.reg != 0U)
    {
        /* Wait for sync */
    }
}