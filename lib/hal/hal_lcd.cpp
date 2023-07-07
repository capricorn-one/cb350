#include "hal_lcd.h"
#include "hal.h"
#include <SPI.h>

void hal_lcd_init(void) {
    LCD_SPI.begin();
}