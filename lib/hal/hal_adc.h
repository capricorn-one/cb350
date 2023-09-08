 #ifndef HAL_ADC_H_
#define	HAL_ADC_H_

    #include <stdint.h>
    #include <stdbool.h>
    #include "../ads131m0x/ads131m0x.h"

    #define HAL_ADC_CH_NUM 8

    typedef enum adc_power_mode {
        ADC_POWER_MODE_STANDBY = 0,
        ADC_POWER_MODE_ILE = 1,
        ADC_POWER_MODE_VERY_LOW = 2,
        ADC_POWER_MODE_LOW = 3,
        ADC_POWER_MODE_HIGH = 4
    } adc_power_mode_t;

    typedef ads131m0x_conversion_t adc_conversion_t;

    void hal_adc_init(adc_conversion_t * adc_raw_data_ptr);

    void hal_adc_reset(void);

    void hal_adc_sleep(void);

    void hal_adc_wakeup(void);

    void hal_adc_clock_enable(bool state);

    uint16_t hal_adc_read_conversion(void);

    void hal_adc_enable_channels(uint8_t channel_bitmap);

    void hal_adc_disable_channels(uint8_t channel_bitmap);

    void hal_adc_external_reference(bool state);

    void hal_adc_channel_gain_set(uint8_t channel, uint8_t pga);

    void hal_adc_channel_gain_set_all(uint32_t gain_settings);

    uint8_t hal_adc_channel_gain_get(uint8_t channel);

    uint16_t hal_adc_read_id(void);

    uint16_t hal_adc_read_status(void);

    void hal_adc_set_power_mode(adc_power_mode_t power_mode);

    /***** local adc functions *****/
    int16_t hal_adc_sam_differential_read(uint8_t pos_pin, uint8_t neg_pin);

    int16_t hal_adc_sam_analog_read(uint8_t pin);

    int16_t hal_adc_sam_internal_temp_read(void);

    int16_t hal_adc_sam_vcc_read(void);

#endif	/* HAL_ADC_H_ */