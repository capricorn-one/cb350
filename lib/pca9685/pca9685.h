#ifndef PCA_9685_H
#define PCA_9685_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    int8_t (*transfer)(uint8_t addr, uint8_t reg, uint8_t *data, size_t len, bool read);
    void (*delay_ms)(uint32_t ms);
} pca9685_hal_t;

class pca9685 {

public:

    pca9685(void);

    /// @brief initialize the PCA9685
    /// @param none
    void begin(pca9685_hal_t *pca9685_hal, uint8_t address);
    
    void begin(pca9685_hal_t *pca9685_hal) { begin(pca9685_hal, 0); }

    /// @brief reset the PCA9685
    /// @param none
    void reset(void);

    /// @brief put the PCA9685 to sleep
    /// @param none
    void sleep(void) { sleep(true); }

    /// @brief wake the PCA9685 from sleep
    /// @param none
    void wake(void) { sleep(false); }

    // void setPWMFreq(float freq);

    void set_duty_cycle(uint8_t ch, float duty_cycle);

    void set_state(uint8_t ch, bool state);


private:

    uint8_t i2c_addr;

    pca9685_hal_t *hal;
    /// @brief put the PCA9685 to sleep or wake it up
    /// @param state true = sleep, false = wake
    void sleep(bool state);

    void set_pwm_freq(int freq);    // 40Hz to 1000Hz using internal 25MHz oscillator

    void set_output_mode(bool totempole);

    void set_pwm_counters(uint8_t ch, uint16_t duty_cycle);

    void set_pwm_counters(uint8_t ch, uint16_t on, uint16_t off);

    int8_t i2c_write(uint8_t reg, uint8_t *data, size_t len);
    int8_t i2c_write(uint8_t reg, uint8_t data);
    int8_t i2c_read(uint8_t reg, uint8_t *data, size_t len);
};

#endif