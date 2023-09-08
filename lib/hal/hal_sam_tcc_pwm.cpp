#include "hal_sam_tcc_pwm.h"
#include "sam.h"


#define PWM_PIN_NUMBER  15
#define PMW_PORT_NUMBER 0       // PORTA

void hal_sam_tcc_pwm_init() {

    // Configure output pin
    PORT->Group[PMW_PORT_NUMBER].PINCFG[PWM_PIN_NUMBER].reg = PORT_PINCFG_PMUXEN;
    PORT->Group[PMW_PORT_NUMBER].PMUX[PWM_PIN_NUMBER >> 1].reg |= PORT_PMUX_PMUXO_E;

    // Select generator and write lock for TC3
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC2_TC3_Val) | GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK0_Val)  | GCLK_CLKCTRL_CLKEN;
    
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;

    // Initialize TC3 as PWM to generate 8MHz clock
    TC3->COUNT8.CTRLA.reg = TC_CTRLA_SWRST;

    while((TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY) != 0U)
    {
        /* Wait for sync */
    }

    TC3->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_WAVEGEN_NPWM;

    TC3->COUNT8.PER.reg = 5U;
    TC3->COUNT8.CC[0].reg = 0U;
    TC3->COUNT8.CC[1].reg = 3U;

    TC3->COUNT8.INTFLAG.reg = TC_INTFLAG_MASK;

    while((TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY) != 0U)
    {
        /* Wait for sync */
    }
}

void hal_sam_tcc_pwm_start() {
    TC3->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    while((TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY) != 0U)
    {
        /* Wait for sync */
    }
}

void hal_sam_tcc_pwm_stop() {
    TC3->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while((TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY) != 0U)
    {
        /* Wait for sync */
    }
}