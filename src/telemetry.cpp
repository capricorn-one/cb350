#include "telemetry.h"
#include "power.h"
#include "hal.h"
#include "hal_adc.h"
#include "hal_outputs.h"

#define TELEM_DATA_STARTER_CURRENT      0
#define TELEM_DATA_REGULATOR_CURRENT    1
#define TELEM_DATA_LOAD_CURRENT         2
#define TELEM_DATA_IS0_CURRENT          3
#define TELEM_DATA_IS1_CURRENT          4
#define TELEM_DATA_IS2_CURRENT          5
#define TELEM_DATA_IS3_CURRENT          6
#define TELEM_DATA_IS4_CURRENT          7

bool telemetry::begin(void) {

    dsel = 0;

    // Enable measurement of current from loads on BTS7008-2
    hal_outputs_set_state(PWM_CH_DEN, true);

    hal_outputs_set_state(PWM_CH_DSEL, false);
    
    doc["name"] = "TELEM";
    doc["time"] = millis();

    I = doc.createNestedObject("I");
    V = doc.createNestedObject("V");

    return hal_adc_init(adc_raw_data);
}

bool telemetry::update(void) {

    // LOG_INF("*****TELEM******");5
    hal_adc_read_conversion();

    doc["time"] = millis();

    // Starter current uses a 0.5mΩ resistor on INA180A2 (25x gain) = V / 25 / .00005 = V * 80.
    I["START"] = hal_adc_get_conversion(ADC_STARTER_CURRENT, 80.00);
    
    // Regulator current uses a 1mΩ resistor on INA180A2 (50x gain) = V / 50 / .001 = V * 20. = 1.430511474609375e-7 * 20. = 2.86102294921875e-6
    I["REG"] = hal_adc_get_conversion(ADC_REGULATOR_CURRENT, 20.0);

    // Load current uses a 1mΩ resistor on INA180A2 (50x gain) = V / 50 / .001 = V * 20.
    I["LOAD"] = hal_adc_get_conversion(ADC_LOAD_CURRENT, 20.0);


    // IS0-n current outputs current with a ratio of ~1:5425, so 1mA = 5.4A. A load resistor of 100 ohms is used to generate the voltage
    // on the pin, so a 1mA current will generate 100mV on the pin. So a 100mV reading on the pin is 1mA, or 5.4A on the load.
    // 1V = 10mA = 54A, see datasheet for BTS7008-2EPA section 9.6.1
    if(dsel == 0) {
        I["AUX"] = hal_adc_get_conversion(ADC_IS0_CURRENT, 54.25);
        I["IGN"] = hal_adc_get_conversion(ADC_IS1_CURRENT, 54.25);
        I["HLT"] = hal_adc_get_conversion(ADC_IS2_CURRENT, 54.25);
        I["TLGT"] = hal_adc_get_conversion(ADC_IS3_CURRENT, 54.25);
        I["HBM"] = hal_adc_get_conversion(ADC_IS4_CURRENT, 54.25);
    }
    else {
        I["HORN"] = hal_adc_get_conversion(ADC_IS0_CURRENT, 54.25);
        I["COMP"] = hal_adc_get_conversion(ADC_IS1_CURRENT, 54.25);
        I["LFTS"] = hal_adc_get_conversion(ADC_IS2_CURRENT, 54.25);
        I["RGTS"] = hal_adc_get_conversion(ADC_IS3_CURRENT, 54.25);
        I["BLGT"] = hal_adc_get_conversion(ADC_IS4_CURRENT, 54.25);
    }

    // Local ADC measurements
    V["BAT"] = hal_adc_battery_read();

    I["STRTB"] = hal_adc_starter_current_read();

    V["VCC"] = hal_adc_vcc_read();

    doc["TEMP"] =   hal_adc_internal_temp_read();
    
    
    if(++dsel == 2) {

        // LOG_INF("VBAT %f\tVCC %f\tTEMP %f\tSTART %f", V["BAT"].as<float>(), V["VCC"].as<float>(), doc["TEMP"].as<float>(), I["STRTB"].as<float>());

        serializeJson(doc, LCD_SERIAL);   // send to LCD
        LCD_SERIAL.write(0xFF);

        dsel = 0;
    }


    if(dsel == 0) {
        // hal_outputs_set_state(PWM_CH_AUX_LOGO, false);
        hal_outputs_set_state(PWM_CH_DSEL, false);
    } else {
        // hal_outputs_set_duty_cycle(PWM_CH_AUX_LOGO, 0.1);
        hal_outputs_set_state(PWM_CH_DSEL, true);
    }
    
    return true;
}

// int32_t telemetry::get_value_raw(telem_data_t telem_data) {
//     return ads131m0x_get_channel_data_raw(telem_data);
// }

telemetry telem("telem");
