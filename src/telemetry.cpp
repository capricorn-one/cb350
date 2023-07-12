#include "telemetry.h"
#include "outputs.h"
#include "power.h"
#include "hal.h"
#include "hal_adc.h"

typedef enum {
    TELEM_DATA_STARTER_CURRENT = 0,
    TELEM_DATA_REGULATOR_CURRENT = 1,
    TELEM_DATA_LOAD_CURRENT = 2,
    TELEM_DATA_IS0_CURRENT = 3,
    TELEM_DATA_IS1_CURRENT = 4,
    TELEM_DATA_IS2_CURRENT = 5,
    TELEM_DATA_IS3_CURRENT = 6,
    TELEM_DATA_IS4_CURRENT = 7,
    TELEM_DATA_NUM = 8
} telem_data_t;

void telemetry::init(void) {

    // Need to setup analog inputs for battery voltage monitoring and starter current monitoring
    // Use the 12 bit SAADC on-board the nRF5340
    // int err;

    /* Configure channels individually prior to sampling. */
	// for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
	// 	if (!device_is_ready(adc_channels[i].dev)) {
	// 		printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
	// 		return;
	// 	}

	// 	err = adc_channel_setup_dt(&adc_channels[i]);
	// 	if (err < 0) {
	// 		printk("Could not setup channel #%d (%d)\n", i, err);
	// 		return;
	// 	}
	// }

    outputs[PWM_CH_DSEL].set(false);
    outputs[PWM_CH_DEN].set(true);      // enable diagnostics on BTS chips


    hal_adc_init(adc_raw_data);

    hal_adc_channel_gain_set(ADC_IS0_CURRENT, ADS131M0X_GAIN_8X);
    hal_adc_channel_gain_set(ADC_IS1_CURRENT, ADS131M0X_GAIN_8X);
    hal_adc_channel_gain_set(ADC_IS2_CURRENT, ADS131M0X_GAIN_8X);
    hal_adc_channel_gain_set(ADC_IS3_CURRENT, ADS131M0X_GAIN_8X);
    hal_adc_channel_gain_set(ADC_IS4_CURRENT, ADS131M0X_GAIN_8X);

    hal_adc_trigger_conversion();

}

void telemetry::update(void) {

    hal_adc_get_new_conversions();

    // For a 23 bit integeger, with 1.2V reference, voltage resolution is 1.2 / 2^23 = 1.430511474609375e-7 per bit.

    // Starter current uses a 1mΩ resistor on INA180A2 (25x gain) = V / 25 / .0005 = V * 80. = 1.430511474609375e-7 * 80.
    adc_data[ADC_STARTER_CURRENT] = (float)adc_raw_data[TELEM_DATA_STARTER_CURRENT].raw * 1.1444e-5;
    
    // Regulator current uses a 1mΩ resistor on INA180A2 (50x gain) = V / 50 / .001 = V * 20. = 1.430511474609375e-7 * 20. = 2.86102294921875e-6
    adc_data[ADC_REGULATOR_CURRENT] = (float)adc_raw_data[TELEM_DATA_REGULATOR_CURRENT].raw * 2.8610e-6;

    // Load current uses a 1mΩ resistor on INA180A2 (50x gain) = V / 50 / .001 = V * 20. = 1.430511474609375e-7 * 20. = 2.86102294921875e-6
    adc_data[ADC_LOAD_CURRENT] = (float)adc_raw_data[TELEM_DATA_LOAD_CURRENT].raw * 2.8610e-6;

    // IS0-n current outputs current with a ratio of 1:9500, so 1mA = 9.5A. A load resistor of 100 ohms is used to generate the voltage
    // on the pin, so a 1mA current will generate 100mV on the pin. Working backwards in volts using a 1.2V reference means
    // a 1.2V signal = 12mA, with a ratio of 9500:1, so 12mA * 9500 = 114A. 1.2V / 114A = 10.52631578947368mV/A
    // Future versions should change R(IS) to 1kΩ to increase the signal
    // However, for this version, we configure the ADS131M08 gain stage for 8X gain, resulting in max read of150mV.
    // For a 150mV signal, or maximum current to be read is 150mV / 100Ω = 1.5mA * 9500 = 14.25A. 150mV / 14.25A = 10.52631578947368mV/A
    // At max value of 14.25A and 23 bit ressolution, each bit = 14.25A / 2^23 = 1.430511474609375e-5
    adc_data[ADC_IS0_CURRENT] = (float)adc_raw_data[TELEM_DATA_IS0_CURRENT].raw * 1.430511e-5;
    adc_data[ADC_IS1_CURRENT] = (float)adc_raw_data[TELEM_DATA_IS1_CURRENT].raw * 1.430511e-5;
    adc_data[ADC_IS2_CURRENT] = (float)adc_raw_data[TELEM_DATA_IS2_CURRENT].raw * 1.430511e-5;
    adc_data[ADC_IS3_CURRENT] = (float)adc_raw_data[TELEM_DATA_IS3_CURRENT].raw * 1.430511e-5;
    adc_data[ADC_IS4_CURRENT] = (float)adc_raw_data[TELEM_DATA_IS4_CURRENT].raw * 1.430511e-5;

    // Local sam adc uses 14 bit resolution and the same external 1.25V reference source.
    // Local voltage conversion per bit is 1.25V / 2^14 = 7.62939453125e-5

    // The battery voltage is a differential measurement in a voltage divider of 3 resistors. 1MΩ, 33k, and 100k.
    // Therefore the conversion of input voltage to differnital voltage 29.1262mV per volt
    // The local voltage conversion max voltage is 1.25V resulting in 1.25 / .0291262 = 42.9V max
    // So each bit of 14 bit resolution is 42.9 / 2^14 = 2.624512e-3
    battery_voltage = hal_adc_sam_differential_read(ADC_SAM_BATTERY_POS, ADC_SAM_BATTERY_NEG) * 2.624512e-3;

    // IS0-n current outputs current with a ratio of 1:9500, so 1mA = 9.5A. A load resistor of 100 ohms is used to generate the voltage
    // on the pin, so a 1mA current will generate 100mV on the pin. Working backwards in volts using a 1.2V reference means
    // a 1.2V signal = 12mA, with a ratio of 9500:1, so 12mA * 9500 = 114A. 1.2V / 114A = 10.52631578947368mV/A
    // Future versions should change R(IS) to 1kΩ to increase the signal
    // For a 1.2V signal, or maximum current to be read is 1.2V / 100Ω = 12mA * 52100 = 625.2A. 1.2V / 625.2A
    // At max value of 625.2A and 14 bit ressolution, each bit = 625.2A / 2^14 = 38.14697265625e-3 Amps per bit
    starter_current_local = hal_adc_sam_analog_read(ADC_SAM_STARTER_CURRENT) * 38.147e-3;

    // From section 17.6.10 of the ATSAMD21 datasheet, the internal temperature sensor is 0.688V at 25C and has a slope of 2.16mV/°C
    // To convert to voltage, use 1.25V / 2^14 = 7.62939453125e-5 per bit
    internal_temp = (hal_adc_sam_internal_temp_read() * 7.62939453125e-5 - 0.688) / 0.00216 + 25.0;

    // This voltage is scaled at 1/4 of vccio. At max 1.25V reference, each bit of 14 bit resolution is 1.25 / 2^14 = 7.62939453125e-5
    // Since we're measuring the voltage divided by 4, we need to multiply by 4 for the correct Vcc value.
    // This means 1.25 / 2^14 * 4 = 0.00030517578125 per bit.
    uController_vcc = hal_adc_sam_vcc_read() * 3.05176e-4;

    hal_adc_trigger_conversion();
}

// int32_t telemetry::get_value_raw(telem_data_t telem_data) {
//     return ads131m0x_get_channel_data_raw(telem_data);
// }

telemetry telem("telem");
