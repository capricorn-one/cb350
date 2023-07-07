#include "telemetry.h"
#include "outputs.h"
#include "power.h"
#include "hal.h"
#include "hal_adc.h"

static int adc_nrf_init(void) {

    int err;
	// uint32_t count = 0;
	// uint16_t buf;

	// struct adc_sequence sequence = {
	// 	.buffer = &buf,
	// 	/* buffer size in bytes, not number of samples */
	// 	.buffer_size = sizeof(buf),
	// };

    // /* Configure channels individually prior to sampling. */
	// for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
	// 	if (!device_is_ready(adc_channels[i].dev)) {
	// 		printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
	// 		return 0;
	// 	}

	// 	err = adc_channel_setup_dt(&adc_channels[i]);
	// 	if (err < 0) {
	// 		printk("Could not setup channel #%d (%d)\n", i, err);
	// 		return 0;
	// 	}
	// }

}

static int adc_nrf_read(float *adc0, float *adc1) {
    
    return 0;
}

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
    int err;

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

    // ads131m0x_set_channel_max(TELEM_DATA_STARTER_CURRENT, 200.);            // Vadc/470 = Is.  Iload = Is * dkILIS. Iload = (Vadc/470) * 52100 ~= Vadc * 111. ... X2 for some reason?
    // // ads131m0x_set_channel_offset(TELEM_DATA_STARTER_CURRENT, -0.012);       // pregain offset

    // ads131m0x_set_channel_max(TELEM_DATA_REGULATOR_CURRENT, 20.);           // 1mΩ resistor on INA180A2 (50x gain) = V / 50 / .001 = V * 20
    // // ads131m0x_set_channel_offset(TELEM_DATA_REGULATOR_CURRENT, -0.012);     // pregain offset

    // ads131m0x_set_channel_max(TELEM_DATA_LOAD_CURRENT, 20.);                // 1mΩ resistor on INA180A2 (50x gain) = V / 50 / .001 = V * 20
    // // ads131m0x_set_channel_offset(TELEM_DATA_LOAD_CURRENT, -0.0004);         // pregain offset

    // ads131m0x_set_channel_max(TELEM_DATA_IS0_CURRENT, 4.5);                 // Vadc/1200 = Is. Iload = Is * 5500 (5.5mΩ stage)
    // // ads131m0x_set_channel_offset(TELEM_DATA_IS0_CURRENT, -0.014);          // pregain offset

    // ads131m0x_set_channel_max(TELEM_DATA_IS1_CURRENT, 4.5);
    // // ads131m0x_set_channel_offset(TELEM_DATA_IS1_CURRENT, -0.014);          // pregain offset
    
    // ads131m0x_trigger_conversion();

    hal_adc_init();


}

void telemetry::update(void) {
    hal_adc_update();

    // if(ads131m0x_get_new_data()) {

    //     // IN0 Diagnostics
    //     if(outputs[PWM_CH_DSEL].get() == false) {

    //         outputs[PWM_CH_DSEL].set(true);

    //         pwr_output.set_current(POWER_OUTPUT_SIGNAL_LEFT, ads131m0x_get_channel_data(TELEM_DATA_IS0_CURRENT));
    //         pwr_output.set_current(POWER_OUTPUT_IGNITION, ads131m0x_get_channel_data(TELEM_DATA_IS1_CURRENT));
    //         pwr_output.set_current(POWER_OUTPUT_AUXILLARY, ads131m0x_get_channel_data(TELEM_DATA_IS2_CURRENT));
    //         pwr_output.set_current(POWER_OUTPUT_HIGHBEAM, ads131m0x_get_channel_data(TELEM_DATA_IS3_CURRENT));
    //         pwr_output.set_current(POWER_OUTPUT_COMPRESSOR, ads131m0x_get_channel_data(TELEM_DATA_IS4_CURRENT));

        
    //     }
    //     // IN1 Diagnostics
    //     else {

    //         outputs[PWM_CH_DSEL].set(false);

    //         pwr_output.set_current(POWER_OUTPUT_SIGNAL_RIGHT, ads131m0x_get_channel_data(TELEM_DATA_IS0_CURRENT));
    //         pwr_output.set_current(POWER_OUTPUT_HORN, ads131m0x_get_channel_data(TELEM_DATA_IS1_CURRENT));
    //         pwr_output.set_current(POWER_OUTPUT_HEADLIGHT, ads131m0x_get_channel_data(TELEM_DATA_IS2_CURRENT));
    //         pwr_output.set_current(POWER_OUTPUT_TAIL_LIGHT, ads131m0x_get_channel_data(TELEM_DATA_IS3_CURRENT));
    //         pwr_output.set_current(POWER_OUTPUT_BRAKE_LIGHT, ads131m0x_get_channel_data(TELEM_DATA_IS4_CURRENT));

    //     }

    //     pwr_output.set_current(POWER_OUTPUT_STARTER, ads131m0x_get_channel_data(TELEM_DATA_STARTER_CURRENT));

    //     regulator_current = ads131m0x_get_channel_data(TELEM_DATA_REGULATOR_CURRENT);

    //     load_current = ads131m0x_get_channel_data(TELEM_DATA_LOAD_CURRENT);

    //     // start next conversion
    //     ads131m0x_trigger_conversion();
    // }

}

// int32_t telemetry::get_value_raw(telem_data_t telem_data) {
//     return ads131m0x_get_channel_data_raw(telem_data);
// }

telemetry telem("TELEM");