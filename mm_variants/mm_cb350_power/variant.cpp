/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "variant.h"

/*
 * Pins descriptions
 */

// Datasheet specifies 38 IO pins available. We don't use SWDCLK or SWDIO, so we have 36 pins.

const PinDescription g_APinDescription[]=
{
  // 0..13 - Digital pins
  // ----------------------

  // 0 - 12
  { PORTA, 10, PIO_AC_CLK, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE  },    // 0 - 8MHz ADC CLOCK
  { PORTA, 17, PIO_AC_CLK, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE  },    // 1 - 16MHz CANBUS CLOCK
  { PORTA, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },      // 2 - ADC_DRDY INPUT
  { PORTB, 10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },      // 3 - IMU_INT INPUT
  { PORTB, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },      // 4 - CAN_INT INPUT
  { PORTA, 12, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE  },   // 5 - CAN_CS OUTPUT
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE  },   // 6 - ADC_CS OUTPUT
  { PORTA, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },       // 7 - PARK INPUT

  { PORTA, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },       // 8 - KICKSTAND INPUT
  { PORTB, 22, PIO_INPUT,   PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE  },   // 9 - GROUNDED AT LCD BOARD
  { PORTB, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },    // 10 - TEENSY IO_1 
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE  },   // 11 - TEENSY IO_2
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE  },   // 12 - ADC_SYNC_RESET OUTPUT

  // 13 - 17  - Analog Inputs (5)
  { PORTA,  2, PIO_DIGITAL, (PIN_ATTR_DIGITAL | PIN_ATTR_ANALOG), ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   // 13 - ADC/AIN[0] - Seat Sensor
  { PORTA,  3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },                      // 14 - ADC/AIN[1] - Vrefin
  { PORTB,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },                      // 15 - ADC/AIN[2] - 12V Sense Negative
  { PORTB,  9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },                      // 16 - ADC/AIN[3] - 12V Sense Positive
  { PORTA,  6, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },                      // 17 - ADC/AIN[4] - Starter Current
  
  // 18 - 19
  { PORTA,  0, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },   // 18 - SDA: SERCOM1/PAD[0]
  { PORTA,  1, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },   // 19 - SCL: SERCOM1/PAD[1]

  // 20 - 22 - SERCOM0 - SPI/ADC
  { PORTA,  4, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },      // 20 - MOSI: SERCOM0/PAD[0]
  { PORTA,  5, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },      // 21 - MISO: SERCOM0/PAD[1]
  { PORTA,  7, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },      // 22 - SCK: SERCOM0/PAD[3]

  // 23/24 - SERCOM2 - I2C/IMU
  { PORTA,  8, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },      // 23 - SDA: SERCOM2/PAD[0]
  { PORTA,  9, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },      // 24 - SCL: SERCOM2/PAD[1]
  
  // 25 - 27 - SERCOM4 - SPI/CAN MCP2515
  { PORTA, 13, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },     // 25 - MISO: SERCOM4/PAD[1]
  { PORTA, 14, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },     // 26 - MOSI: SERCOM4/PAD[2]
  { PORTA, 15, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },     // 27 - SCK: SERCOM4/PAD[3]

  // 28 - 30 - SERCOM3 - UART FOR TEENSY/LCD COMMUNICATION
  { PORTA, 16, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },         // 28 - RX: SERCOM3/PAD[0]
  { PORTA, 18, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },         // 29 - TX: SERCOM3/PAD[2]
  { PORTA, 19, PIO_INPUT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },           // 30 - GROUNDED AT LCD BOARD

  // 31/32 - SERCOM5 - I2C/PCA9685
  { PORTB,  2, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // 31 - SDA: SERCOM5/PAD[0]
  { PORTB,  3, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // 32 - SCL: SERCOM5/PAD[1]
  
  // 33/34 - USB
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },         // 33 - USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },         // 34 - USB/DP

  // 35
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, PWM0_CH7, TCC0_CH7, EXTERNAL_INT_5 },              // 35 - PWM OUTPUT FOR LED DATA

} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX );
Uart Serial2( &sercom3, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX );

void SERCOM1_Handler()
{
  Serial1.IrqHandler();
}

void SERCOM3_Handler()
{
  Serial2.IrqHandler();
}