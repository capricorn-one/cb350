#include "starterMotor.h"
#include "power.h"
#include "hal.h"

void starterMotor::update(void) {

    if( (hal_millis() - enableTimer) > 3000)
        disable();

    solenoid_read_current();

}

bool starterMotor::begin() {
    datapoint_counter = 0;
    // // telem.setMode(TELEM_MODE_AMUX_CH3);         // Only measure voltage during starter motor trigger
    // // telem.setTaskDelay(50);                     // Match local adc reading to starter motor task rate
    enableTimer = hal_millis();
    solenoid_enable(true);

    return true;
}

void starterMotor::exit(void) {
    solenoid_enable(false);
    // telem.setMode(TELEM_MODE_AMUX_ALL);                     // Reset local ADC to swtich mux
    // telem.setTaskDelay(TELEM_TASK_DEFAULT_INTERVAL_MS);     // Revert to default telemetry scanning interval
}

void starterMotor::solenoid_enable(bool state) {

    // lin_frame_t lin_frame;

    // if(state) {
    //     lin_frame.response[0] = 0x01;
    //     lin_frame.response[1] = 0x01;
    // }
    // else {
    //     lin_frame.response[0] = 0x10;
    //     lin_frame.response[1] = 0x10;
    // }

    // lin.write(&lin_frame, MANSEN_MOTO_LIN_ID_SOLENOID_WRITE);
    pwr_output.set(POWER_OUTPUT_STARTER, state);
}

/***** FROM SOLENOID FIRMWARE *****/
// BASED ON 1.1Vref and 499 sense resistor
// Conversion from ADC value to current is ADC0.RES / 2^10 * Vref = ADC0.RES / 1023. * 1.1
// With a 1.1V reference.. this means a max current measured of 1.1/499 = 2.2mA
// 2.2mA / 20uA = 110A
// Converting voltage to resistor current (Isense) = (ADC_CONVERSION_VOLTAGE / 499)
// Resistor current Is to load current (Iload) = Is / 20uA = (ADC_CONVERSION_VOLTAGE / 499) / 20uA = ADC0.RES / 1023. * 1.1 / 499 / .00002 (@ full scale, max current read is 104 A)
// Iload = ADC_VOLTAGE / 499 / .00002 --->>> ADC0.RES / 1023. * 1.1 / 499 / .00002 = ADC0.RES * 0.10774.... (AMPS per bit)
// Let the main computer do the math to get floating point current
// Conversion should be ADC0.RES * 0.1077424 = current (float)
void starterMotor::solenoid_read_current(void) {

    // if(datapoint_counter >= 128)
    //     return;

    // lin_frame_t lin_frame;

    // // power.triggerAdcVbat();

    // if( lin.query(&lin_frame, MANSEN_MOTO_LIN_ID_SOLENOID_READ) == LIN_ERROR_NONE) {
    //     datapoints[datapoint_counter].timestamp = k_uptime_get_32();
    //     // // datapoints[datapoint_counter].voltage = telem.getTelemetryValue(TELEM_BATTERY_VOLTAGE);
    //     // datapoints[datapoint_counter].voltage = power.readAdcVbat();
    //     // datapoints[datapoint_counter].current = ((lin_frame.response[1]<<8) | lin_frame.response[0]) * 0.1077424;
    // }

}

starterMotor starter("starter");
