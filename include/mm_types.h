/*
 * mm_types.h
 *
 * Created: 11/27/2020 4:41:20 PM
 *  Author: CJM28241
 */ 


#ifndef _MM_TYPES_H_
#define _MM_TYPES_H_

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
    MOTO_PUSHBUTTON_HEADLIGHTS = 0,
    MOTO_PUSHBUTTON_HORN = 1,
    MOTO_PUSHBUTTON_LEFT_SIG = 2,
    MOTO_PUSHBUTTON_CLUTCH = 3,

    MOTO_PUSHBUTTON_AUX = 4,
    MOTO_PUSHBUTTON_STARTER = 5,
    MOTO_PUSHBUTTON_RIGHT_SIG = 6,
    // MOTO_PUSHBUTTON_BRAKE = 7,

    MOTO_PUSHBUTTON_NUM = 8,
} moto_pushbutton_num_t;

typedef enum {
	MOTO_MODE_STARTUP = 0,					// Startup sequence when first turning on (connecting to battery or reset) only.
	MOTO_MODE_OFF = 1,						// Indicates the bike is turned off, specified by the value of the KEY input, must be lowest current mode
	MOTO_MODE_IGNITION = 2,					// Indicates key in in ignition lock position (normal drive position)
	MOTO_MODE_PARKING_LIGHTS = 3,			// Indicates key is in parked position, ignition off but tail light and headlight on only
	MOTO_MODE_DRIVING = 4,					// Bike has ignition on and is ready to move or is moving
	MOTO_MODE_PARKED = 5,					// Bike is parked outside of home (GPS tracking, LTE radio active, and IMU sensor active)
	MOTO_MODE_GARAGE = 6,					// Bike is in garage, key is off or disconnected, minimal power and should be charging with wifi/ble connection
} moto_mode_t;

typedef enum {
    MOTO_SWITCH_TRIGGER_RELEASE = 0,
    MOTO_SWITCH_TRIGGER_PRESS = 1,
    MOTO_SWITCH_TRIGGER_HOLD = 2,
	MOTO_SWITCH_TRIGGER_LONG_HOLD = 3,
} moto_switch_trigger_t;

typedef struct {
    float current;
    float voltage;
    uint32_t timestamp;
} solenoid_datapoint_t;

typedef enum {
    POWER_OUTPUT_IGNITION = 0,
    POWER_OUTPUT_SIGNAL_LEFT = 1,
    POWER_OUTPUT_SIGNAL_RIGHT = 2,
    POWER_OUTPUT_HEADLIGHT = 3,
    POWER_OUTPUT_BRAKE_LIGHT = 4,
    POWER_OUTPUT_HIGHBEAM = 5,
    POWER_OUTPUT_TAIL_LIGHT = 6,
    POWER_OUTPUT_HORN = 7,
    POWER_OUTPUT_AUXILLARY = 8,
    POWER_OUTPUT_STARTER = 9,
    POWER_OUTPUT_COMPRESSOR = 10,
	POWER_OUTPUT_NUM = 11
} power_output_enum_t;

typedef enum {
    VFD_LED_INDICATOR_SIGNAL_RIGHT = 0,
    VFD_LED_INDICATOR_WARNING = 1,
    VFD_LED_INDICATOR_NEUTRAL = 2,
    VFD_LED_INDICATOR_BRIGHTS = 3,
    VFD_LED_INDICATOR_SIGNAL_LEFT = 4,
    
} vfd_led_indicator_t;

typedef struct {
    uint16_t speed;
    uint16_t rpm;
    struct {
        uint8_t num;
        uint8_t temperature;
    } gear;
} vfd_telem_t;


// Frame length is defined by the ID number
// 0 - 31 = 2 bytes
// 32 - 47 = 4 bytes
// 48 - 63 = 8 bytes
typedef enum {
	
	/**** 2 byte data packet, ID RANGE 0-31****/
	MANSEN_MOTO_LIN_ID_OUTPUT_STATES_WRITE = 5,
	
	MANSEN_MOTO_LIN_ID_GEAR_INDICATOR_READ = 8,
	
	MANSEN_MOTO_LIN_ID_SOLENOID_READ = 20,
	MANSEN_MOTO_LIN_ID_SOLENOID_WRITE = 21,
	
	
	/**** 4 byte data packet, ID RANGE 32-47****/
	MANSEN_MOTO_LIN_ID_HANDLEBAR_READ = 36,

	/**** 8 byte data packet, ID RANGE 48-63****/

	
} MANSEN_MOTO_LIN_ID_t;


// CAN BUS MESSAGES

#define MM_CAN_ID_HANDLEBAR_SWITCH_STATES       0x101
#define MM_CAN_ID_VFD_TELEM                     0x111

#define MM_CAN_ID_HUB_OUTPUT_STATES             0x201



#ifdef	__cplusplus
}
#endif

#endif /* _MM_TYPES_H_ */