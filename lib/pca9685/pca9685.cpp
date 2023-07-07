#include "pca9685.h"

#define MODE1_REG           0x00	//Mode  register  1
#define MODE1_ALLCAL 	    0x01  	/**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 		    0x02  	/**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 		    0x04  	/**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 		    0x08  	/**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 	    0x10  	/**< Low power mode. Oscillator off */
#define MODE1_AI 		    0x20  	/**< Auto-Increment enabled */
#define MODE1_EXTCLK 	    0x40  	/**< Use EXTCLK pin clock */
#define MODE1_RESTART 	    0x80	/**< Restart enabled */

#define MODE2_REG           0x01	//Mode  register  2
#define MODE2_OUTNE_0 	    0x01 	/**< Active LOW output enable input */
#define MODE2_OUTNE_1       0x02    /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 	    0x04 	/**< totem pole structure vs open-drain */
#define MODE2_OCH 		    0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 	    0x10  	/**< Output logic state inverted */

#define SUBADR1_REG         0x02	//I2C-bus subaddress 1
#define SUBADR2_REG         0x03	//I2C-bus subaddress 2
#define SUBADR3_REG         0x04	//I2C-bus subaddress 3
#define ALLCALLADR_REG      0x05    //LED All Call I2C-bus address
#define LED0_REG            0x6		//LED0 start register
#define LED0_ON_L_REG       0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H_REG       0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L_REG      0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H_REG      0x9		//LED0 output and brightness control byte 3
#define ALLLED_ON_L_REG     0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H_REG     0xFB	//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L_REG    0xFC	//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H_REG    0xFD	//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE_REG       0xFE	//prescaler for output frequency
#define RESET_ADDRESS       0x00    // Address to use for software reset 

#define FREQUENCY_OSCILLATOR 	25000000 	/**< Int. osc. frequency in datasheet */

pca9685::pca9685(void) {}

void pca9685::begin(pca9685_hal_t *pca9685_hal, uint8_t address) {
    
    hal = pca9685_hal;

    i2c_addr = 0x40 + address;

    // reset();

    // set_pwm_freq(1000);

    i2c_write(MODE1_REG, MODE1_AI);

    hal->delay_ms(5);
    
}

// @brief Reset the PCA9685 as defined by section 7.1.4 of the datasheet, perform a write of the PCA9685_RESET_ADDRESS
// @param none
void pca9685::reset(void) {

    

    // Send a software reset to the PCA9685
    // i2c_write(dev_i2c.bus, NULL, 0, RESET_ADDRESS);
    
    // k_sleep(K_MSEC(1));

    // // Return to normal mode with internal oscillator enabled
    // i2c_reg_write_byte(dev_i2c.bus, dev_i2c.addr, MODE1_REG, 0x00);

    i2c_write(MODE1_REG, (MODE1_RESTART | MODE1_AI));

    hal->delay_ms(5);
}

void pca9685::sleep(bool state) {

    uint8_t oldmode = 0;
    uint8_t newmode = 0;

    i2c_read(MODE1_REG, &oldmode, 1);

    if(state)
        newmode = (oldmode&0x7F) | MODE1_SLEEP; // sleep
    else
        newmode = (oldmode&0x7F) & ~MODE1_SLEEP; // wake

    i2c_write(MODE1_REG, newmode);

    hal->delay_ms(5);

}

void pca9685::set_duty_cycle(uint8_t ch, float duty_cycle) {

    if(duty_cycle > 1.00)
        duty_cycle = 1.00;
    else if(duty_cycle < 0.00)
        duty_cycle = 0.00;

    set_pwm_counters(ch, 0, duty_cycle * 4095.);
}

void pca9685::set_state(uint8_t ch, bool state) {
    if(state) {
        set_pwm_counters(ch, 4096, 0);
    }
    else {
        set_pwm_counters(ch, 0, 4096);
    }
}

void pca9685::set_pwm_freq(int freq) {

    uint8_t prescale = (FREQUENCY_OSCILLATOR / (4096 * freq)) - 1;

    uint8_t oldmode = 0;
    uint8_t newmode = 0;

    i2c_read(MODE1_REG, &oldmode, 1);

    newmode = (oldmode&0x7F) | MODE1_SLEEP;         // sleep, must be set to change PRE_SCALE_REG

    i2c_write(MODE1_REG,  newmode);

    i2c_write(PRE_SCALE_REG, prescale);

    i2c_write(PRE_SCALE_REG, oldmode);
}

void pca9685::set_output_mode(bool totempole) {

    uint8_t oldmode = 0;
    uint8_t newmode = 0;

    i2c_read(MODE1_REG, &oldmode, 1);
    if (totempole) {
        newmode = oldmode | MODE2_OUTDRV;
    } else {
        newmode = oldmode & ~MODE2_OUTDRV;
    }

    i2c_write(MODE2_REG, newmode);
}

void pca9685::set_pwm_counters(uint8_t ch, uint16_t on, uint16_t off) {

    uint8_t tx_buffer[4];

    tx_buffer[0] = on & 0xFF;
    tx_buffer[1] = on >> 8;
    tx_buffer[2] = off & 0xFF;
    tx_buffer[3] = off >> 8;

    i2c_write(LED0_ON_L_REG + (ch*4), tx_buffer, 4);
}

int8_t pca9685::i2c_write(uint8_t reg, uint8_t *data, size_t len) {
    return hal->transfer(i2c_addr, reg, data, len, false);
}

int8_t pca9685::i2c_write(uint8_t reg, uint8_t data) {
    return hal->transfer(i2c_addr, reg, &data, 1, false);
}

int8_t pca9685::i2c_read(uint8_t reg, uint8_t *data, size_t len) {
    return hal->transfer(i2c_addr, reg, data, len, true);
}