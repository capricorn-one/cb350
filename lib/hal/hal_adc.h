#ifndef HAL_ADC_H_
#define	HAL_ADC_H_

    #include <stdint.h>
    #include <stdbool.h>
    #include "../ads131m0x/ads131m0x.h"

    #define HAL_ADC_NUM             1
    #define HAL_ADC_DEV_CH_NUM      8
    #define HAL_ADC_CH_NUM          (HAL_ADC_NUM * HAL_ADC_DEV_CH_NUM)

    typedef enum adc_power_mode {
        ADC_POWER_MODE_STANDBY = 0,
        ADC_POWER_MODE_ILE = 1,
        ADC_POWER_MODE_VERY_LOW = 2,
        ADC_POWER_MODE_LOW = 3,
        ADC_POWER_MODE_HIGH = 4
    } adc_power_mode_t;

    typedef ads131m0x_conversion_t adc_conversion_t;

    bool hal_adc_init(adc_conversion_t * adc_raw_data_ptr);

    void hal_adc_reset(void);

    void hal_adc_sleep(void);

    void hal_adc_wakeup(void);

    void hal_adc_clock_enable(bool state);

    void hal_adc_continous_read(bool state);

    uint16_t hal_adc_read_continous_conversion(bool wait_for_new_data = true);

    uint16_t hal_adc_read_conversion_immediate(void);

    void hal_adc_synchronize(void);

    void hal_adc_enable_channels(uint8_t channel_bitmap);

    void hal_adc_disable_channels(uint8_t channel_bitmap);

    void hal_adc_external_reference(bool state);

    void hal_adc_channel_gain_set(uint8_t channel, uint8_t pga);

    void hal_adc_channel_gain_set_all(uint32_t gain_settings);

    uint8_t hal_adc_channel_gain_get(uint8_t channel);

    int32_t hal_adc_get_raw_conversion(uint8_t channel);

    float hal_adc_get_conversion(uint8_t channel, float conversion_factor);

    uint16_t hal_adc_read_id(void);

    uint16_t hal_adc_read_status(void);

    void hal_adc_set_power_mode(adc_power_mode_t power_mode);

    /***** local adc functions *****/
    float hal_adc_battery_read(void);

    float hal_adc_starter_current_read(void);

    float hal_adc_vcc_read(void);

    float hal_adc_internal_temp_read(void);

#endif	/* HAL_ADC_H_ */