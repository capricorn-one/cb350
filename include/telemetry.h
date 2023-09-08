#ifndef _TELEMETRY_H
#define _TELEMETRY_H

#include "hal.h"
#include "moto_task.h"
#include "hal_adc.h"
#include <ArduinoJson.h>

class telemetry : public moto_task {

public:

    telemetry(const char *name) : moto_task(name, 50) {}
    telemetry(const char *name, unsigned long aInterval) : moto_task(name, aInterval) {}
    
    void init(void);
    
    int32_t get_starter_current(void) { return doc["adc"][ADC_STARTER_CURRENT]; }
    int32_t get_regulator_current(void) { return doc["adc"][ADC_REGULATOR_CURRENT]; }
    int32_t get_load_current(void) { return doc["adc"][ADC_LOAD_CURRENT]; }
    int32_t get_is0_current(void) { return doc["adc"][ADC_IS0_CURRENT]; }
    int32_t get_is1_current(void) { return doc["adc"][ADC_IS1_CURRENT]; }
    int32_t get_is2_current(void) { return doc["adc"][ADC_IS2_CURRENT]; }
    int32_t get_is3_current(void) { return doc["adc"][ADC_IS3_CURRENT]; }
    int32_t get_is4_current(void) { return doc["adc"][ADC_IS4_CURRENT]; }

    float get_battery_voltage(void) { return doc["v_bat"]; }
    float get_starter_current_local(void) { return doc["i_start"]; }
    float get_internal_temp(void) { return doc["temp"]; }
    float get_uController_vcc(void) { return doc["vcc"]; }

protected:

    bool begin(void) { return true; }
    void update(void);
    void exit(void) {}

private:


    adc_conversion_t adc_raw_data[HAL_ADC_CH_NUM];

    StaticJsonDocument<512> doc;

};

extern telemetry telem;

#endif