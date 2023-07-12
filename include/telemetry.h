#ifndef _TELEMETRY_H
#define _TELEMETRY_H

#include "hal.h"
#include "moto_task.h"
#include "hal_adc.h"

class telemetry : public moto_task {

public:

    telemetry(const char *name) : moto_task(name, 50) {}
    telemetry(const char *name, unsigned long aInterval) : moto_task(name, aInterval) {}
    
    void init(void);
    
    float get_battery_voltage(void) { return battery_voltage; }
    float get_starter_current(void) { return adc_data[ADC_STARTER_CURRENT]; }
    float get_starter_current_local(void) { return starter_current_local; }
    float get_regulator_current(void) { return adc_data[ADC_REGULATOR_CURRENT]; }
    float get_load_current(void) { return adc_data[ADC_LOAD_CURRENT]; }

    float get_is0_current(void) { return adc_data[ADC_IS0_CURRENT]; }
    float get_is1_current(void) { return adc_data[ADC_IS1_CURRENT]; }
    float get_is2_current(void) { return adc_data[ADC_IS2_CURRENT]; }
    float get_is3_current(void) { return adc_data[ADC_IS3_CURRENT]; }
    float get_is4_current(void) { return adc_data[ADC_IS4_CURRENT]; }

    float get_internal_temp(void) { return internal_temp; }
    float get_uController_vcc(void) { return uController_vcc; }


protected:

    bool begin(void) { return true; }
    void update(void);
    void exit(void) {}

private:

    float battery_voltage;
    float starter_current_local;        /* optionall if 96A isn't enough */
    float internal_temp;
    float uController_vcc;

    adc_conversion_t adc_raw_data[HAL_ADC_CH_NUM];
    float adc_data[HAL_ADC_CH_NUM]; 

};

extern telemetry telem;

#endif