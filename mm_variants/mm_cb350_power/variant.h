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

#ifndef _VARIANT_MM_CB350_POWER_
#define _VARIANT_MM_CB350_POWER_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK	(F_CPU)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"

#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (36u)
#define NUM_DIGITAL_PINS     (36u)
#define NUM_ANALOG_INPUTS    (5u)
#define analogInputToDigitalPin(p)  ((p < 5u) ? (p) + 13u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

/*
 * Analog pins
 */

#define PIN_A0               (0ul)
#define PIN_A1               (1ul)
#define PIN_A2               (2ul)
#define PIN_A3               (3ul)
#define PIN_A4               (4ul)

#define PIN_DAC0             (0ul)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;

#define ADC_RESOLUTION		14

/*
 * Serial interfaces
 */
// BT840XE (BLE) - Serial1
#define PIN_SERIAL1_RX        (19ul)
#define PIN_SERIAL1_TX        (18ul)
#define PAD_SERIAL1_RX        (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX        (UART_TX_PAD_0)
#define BLE_SERIAL            Serial1


// Teensy/LCD Serial2
#define PIN_SERIAL2_RX        (28ul)
#define PIN_SERIAL2_TX        (29ul)
#define PAD_SERIAL2_RX        (SERCOM_RX_PAD_0)
#define PAD_SERIAL2_TX        (UART_TX_PAD_2)
#define PIN_LCD_IO1           (10u)
#define PIN_LCD_IO2           (11u)
#define LCD_SERIAL            Serial2

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

// SPI1 - CAN (MCP2515  - CAN Controller)
#define PIN_SPI_MISO        (25u)
#define PIN_SPI_MOSI        (26u)
#define PIN_SPI_SCK         (27u)
#define PIN_CAN_CS           (5u)
#define PIN_CAN_INT          (4u)
#define PERIPH_SPI          sercom4
#define PAD_SPI_TX          SPI_PAD_2_SCK_3
#define PAD_SPI_RX          SERCOM_RX_PAD_1

// SPI2 - ADC
#define PIN_SPI1_MISO        (21u)
#define PIN_SPI1_MOSI        (20u)
#define PIN_SPI1_SCK         (22u)
#define PIN_ADC_CS           (6u)
#define PIN_ADC_SYNC_RESET   (12u)
#define PIN_ADC_DRDY         (2u)
#define PERIPH_SPI1          sercom0
#define PAD_SPI1_TX          SPI_PAD_0_SCK_3
#define PAD_SPI1_RX          SERCOM_RX_PAD_1

// static const uint8_t SS	  = PIN_A2 ;	// SERCOM4 last PAD is present on A2 but HW SS isn't used. Set here only for reference.
// static const uint8_t MOSI = PIN_SPI_MOSI ;
// static const uint8_t MISO = PIN_SPI_MISO ;
// static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

// Wire1 - IMU (MPU9250 - Accelerometer/Gyroscope)
#define PIN_WIRE_SDA        (23u)
#define PIN_WIRE_SCL        (24u)
#define PIN_IMU_INT         (3u)
#define PERIPH_WIRE         sercom2
#define WIRE_IT_HANDLER     SERCOM2_Handler

// Wire - PCA9685 (PWM Controller)
#define PIN_WIRE1_SDA       (31u)
#define PIN_WIRE1_SCL       (32u)
#define PERIPH_WIRE1        sercom5
#define WIRE1_IT_HANDLER    SERCOM5_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (-1)

#define PIN_USB_DM          (33ul)
#define PIN_USB_DP          (34ul)

#define PIN_PARK            (7u)
#define PIN_KICKSTAND       (8u)

/** ADS131M0X CHANNELS **/
#define ADC_STARTER_CURRENT       (0u)
#define ADC_REGULATOR_CURRENT     (1u)
#define ADC_LOAD_CURRENT          (2u)
#define ADC_IS0_CURRENT           (3u)
#define ADC_IS1_CURRENT           (4u)
#define ADC_IS2_CURRENT           (5u)
#define ADC_IS3_CURRENT           (6u)
#define ADC_IS4_CURRENT           (7u)

#define ADC_SAM_BATTERY_POS       (A3)
#define ADC_SAM_BATTERY_NEG       (A2)
#define ADC_SAM_STARTER_CURRENT   (A4)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial1;
extern Uart Serial2;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial

#define CAN_SPI                     SPI
#define ADC_SPI                     SPI1

#define IMU_WIRE                    Wire
#define OUTPUTS_WIRE                Wire1

#endif /* _VARIANT_MM_CB350_POWER_ */
