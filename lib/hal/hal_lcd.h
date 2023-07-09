#ifndef _HAL_LCD_H_
#define _HAL_LCD_H_

#include <stdint.h>
#include <stdbool.h>

    typedef struct {
        uint16_t x;
        uint16_t y;
        bool state;     // 0 for released, 1 for pressed
    } hal_lcd_touch_data_t;

    void hal_lcd_init(void);

    void hal_lcd_enable(bool state);

    void hal_lcd_reset(void);

    void hal_lcd_test_ft81x(void);

#endif /* _HAL_LCD_H_ */