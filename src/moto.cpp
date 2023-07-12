#include "moto.h"
#include "turnSignals.h"
#include "telemetry.h"
#include "moto_can.h"
#include "moto.h"
#include "imu.h"
#include "GUI.h"
#include "starterMotor.h"
#include "power.h"
#include "hal.h"
#include "outputs.h"

// static void accelerometer_int1_callback(uint8_t state) { }
// static void accelerometer_int2_callback(uint8_t state) { }

// static void hall_sensor_int_callback(uint8_t state) { moto.hallSensorInputCallback(state); }
// static void moto_ignition_lock_int_callback(uint8_t state) { moto.ignitionLockInputCallback(state); }
// static void moto_park_int_callback(uint8_t state) { moto.parkInputCallback(state); }
// static void moto_brake_int_callback(uint8_t state) { moto.rearBrakeInputCallback(state); }
// static void moto_kickstand_int_callback(uint8_t state) { moto.kickstandInputCallback(state); }


// HANDLEBAR SWITCH CALLBACKS
static void highBeamPushButtonCallback(moto_switch_trigger_t);
static void hornPushButtonCallback(moto_switch_trigger_t);
static void leftSignalPushButtonCallback(moto_switch_trigger_t);
static void clutchPushButtonCallback(moto_switch_trigger_t);
static void auxillaryPushButtonCallback(moto_switch_trigger_t);
static void starterPushButtonCallback(moto_switch_trigger_t);
static void rightSignalPushButtonCallback(moto_switch_trigger_t);
// static void frontBrakePushButtonCallback(moto_switch_trigger_t);

void motorcycle::init() {

    pwr_output.init();

    outputClass::init();

    // Flash the local LED to show that the program is running
    // Reset auxiillary power... need to understand why this fucks with the LIN
    outputs[PWM_CH_LED_LOCAL].set(true);
    pwr_output.set(POWER_OUTPUT_AUXILLARY, false);
    delay(500);
    outputs[PWM_CH_LED_LOCAL].set(false);
    pwr_output.set(POWER_OUTPUT_AUXILLARY, true);
    delay(250);

    outputs[PWM_CH_LED_LOCAL].set(true);
    delay(100);
    outputs[PWM_CH_LED_LOCAL].set(false);



    setMode(MOTO_MODE_STARTUP);
}


// main loop for reading switches and LIFE OR DEATH critical functions
void motorcycle::update() {

    switch(moto_mode) {

        case MOTO_MODE_STARTUP:             startupModeUpdate();            break;
        case MOTO_MODE_OFF:                 offModeUpdate();                break;
        case MOTO_MODE_IGNITION:            ignitionModeUpdate();           break;
        case MOTO_MODE_PARKING_LIGHTS:      parkingLightsModeUpdate();      break;
        case MOTO_MODE_DRIVING:             drivingModeUpdate();            break;
        case MOTO_MODE_PARKED:              parkedModeUpdate();             break;
        case MOTO_MODE_GARAGE:              garageModeUpdate();             break;
    }

}

void motorcycle::setMode(moto_mode_t mode) {

    moto_mode = mode;

    switch(moto_mode) {

        // Called only once on startup of device, mainly used for debugging
        case MOTO_MODE_STARTUP:

            /******* START MOTO TASKS ******/
            moto.enable();
            // gui.start(GUI_TASK_DEFAULT_INTERVAL_MS);
            // blinkers.start(BLINKERS_TASK_DEFAULT_INTERVAL_MS);
            break;
        
        // Called when bike is put in shutdown mode (key is in off position)
        case MOTO_MODE_OFF:
        case MOTO_MODE_DRIVING:
        case MOTO_MODE_PARKED:
        case MOTO_MODE_GARAGE:

            gui.disable();
            blinkers.disable();

            // delay here waiting for tasks to exit...
            // vTaskDelay(100);
            pwr_output.set(POWER_OUTPUT_IGNITION, false);
            pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);
            pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
            pwr_output.set(POWER_OUTPUT_HEADLIGHT, false);
            pwr_output.set(POWER_OUTPUT_BRAKE_LIGHT, false);
            pwr_output.set(POWER_OUTPUT_HIGHBEAM, false);
            pwr_output.set(POWER_OUTPUT_TAIL_LIGHT, false);
            pwr_output.set(POWER_OUTPUT_HORN, false);
            pwr_output.set(POWER_OUTPUT_AUXILLARY, false);
            pwr_output.set(POWER_OUTPUT_STARTER, false);
            pwr_output.set(POWER_OUTPUT_COMPRESSOR, false);

            // enter sleep mode for ESP32 and shutdown peripherals
            // power.sleep();
            break;

        // Called when bike ignition is turned on (key position only, not actual ignition)
        case MOTO_MODE_IGNITION:

            pwr_output.set(POWER_OUTPUT_IGNITION, true);
            pwr_output.set(POWER_OUTPUT_TAIL_LIGHT, true);
            pwr_output.set(POWER_OUTPUT_AUXILLARY, true);
            pwr_output.set(POWER_OUTPUT_HEADLIGHT, true);


            gui.enable();
            blinkers.enable();
            // exit sleep mode for ESP32 and shutdown peripherals
            // power.full();
            break;
        
        case MOTO_MODE_PARKING_LIGHTS:

            pwr_output.set(POWER_OUTPUT_IGNITION, false);
            pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);
            pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
            pwr_output.set(POWER_OUTPUT_BRAKE_LIGHT, false);
            pwr_output.set(POWER_OUTPUT_HIGHBEAM, false);
            pwr_output.set(POWER_OUTPUT_TAIL_LIGHT, false);
            pwr_output.set(POWER_OUTPUT_HORN, false);
            pwr_output.set(POWER_OUTPUT_STARTER, false);
            pwr_output.set(POWER_OUTPUT_AUXILLARY, false);
            pwr_output.set(POWER_OUTPUT_HEADLIGHT, true);

            break;

    }
}

void motorcycle::startupModeUpdate(void) {

    telem.enable();
    imu.enable();
    moto_can.enable();
    gui.enable();
    
    ignitionModeUpdate();       // temporary until makes more sense later
}

void motorcycle::offModeUpdate(void) {

}

void motorcycle::ignitionModeUpdate(void) {

    // static uint8_t msg = 0;

    // 4 messages get called at every update (~20ms intevals)
    // if(msg == 0) {
    //     gearIndicatorQuery();
    // }
    // else {
    //     handleBarQuery();
    // }
    // if(++msg == 4)
    //     msg = 0;
}

void motorcycle::parkingLightsModeUpdate(void) {

}

void motorcycle::drivingModeUpdate(void) {

}

void motorcycle::parkedModeUpdate(void) {

}

void motorcycle::garageModeUpdate(void) {

}

void motorcycle::setIgnitionState(bool state) {

    // Probably a bunch of error checking/safety shit here
    
    pwr_output.set(POWER_OUTPUT_IGNITION, state);
    LOG_INF("%s - Ignition Set to %d", tag(), state);
}

void motorcycle::setAuxillaryPowerState(bool state) {
    pwr_output.set(POWER_OUTPUT_AUXILLARY, state);
    LOG_INF("%s - AUX Power Set to %d", tag(), state);
}

void motorcycle::ignitionLockInputCallback(bool state) {
    LOG_INF("%s - Lock Input Set to %d", tag(), state);
}

void motorcycle::parkInputCallback(bool state) {
    LOG_INF("%s - Park Input Set to %d", tag(), state);
}

// void motorcycle::rearBrakeInputCallback(bool state) {
    
//     rear_brake_state = !state;      // inverted

//     if(rear_brake_state)
//         pwr_output.set(POWER_OUTPUT_BRAKE_LIGHT, true);
//     else if(!front_brake_state) {
//         pwr_output.set(POWER_OUTPUT_BRAKE_LIGHT, 0);
//     }

//     LOG_INF("%s - Brake Light Set to %d", tag(), output[POWER_OUTPUT_BRAKE_LIGHT].enabled());
// }


void motorcycle::kickstandInputCallback(bool state) {
    LOG_INF("%s - Kickstand Set to %d", tag(), state);
}

// inverted input
// low means magnet present, seat down
// high means magnet not present, seat up)
void motorcycle::hallSensorInputCallback(bool state) {
    hallSensorState = !state;
}

void motorcycle::handleBarCallbackHandler(moto_pushbutton_num_t pb_num, moto_switch_trigger_t trigger) {

    switch(pb_num) {
        case MOTO_PUSHBUTTON_HEADLIGHTS:            highBeamPushButtonCallback(trigger);        break;
        case MOTO_PUSHBUTTON_HORN:                  hornPushButtonCallback(trigger);            break;
        case MOTO_PUSHBUTTON_LEFT_SIG:              leftSignalPushButtonCallback(trigger);      break;
        case MOTO_PUSHBUTTON_CLUTCH:                clutchPushButtonCallback(trigger);          break;
        case MOTO_PUSHBUTTON_AUX:                   auxillaryPushButtonCallback(trigger);       break;
        case MOTO_PUSHBUTTON_STARTER:               starterPushButtonCallback(trigger);         break;
        case MOTO_PUSHBUTTON_RIGHT_SIG:             rightSignalPushButtonCallback(trigger);     break;
        // case MOTO_PUSHBUTTON_BRAKE:                 frontBrakePushButtonCallback(trigger);      break;
        default: break;
    }

    switch_trigger_states[pb_num] = trigger;        // must be called after so we know prev state
}

void motorcycle::vfdTelemetryCallbackHandler(vfd_telem_t *telem) {
    memcpy(&vfd_telem, telem, sizeof(vfd_telem));
}

void motorcycle::ignitionEnable(bool state) {
    pwr_output.set(POWER_OUTPUT_IGNITION, state);
}

motorcycle moto("moto");


/**** END OF MOTORCYCLE CLASS ****/


// LIN SWITCH INPUTS
// LEFT SIDE
void highBeamPushButtonCallback(moto_switch_trigger_t trig) {
    static moto_switch_trigger_t prev_trig = MOTO_SWITCH_TRIGGER_RELEASE;

    LOG_INF("HEADLIGHT CALLBACK WITH TRIG %u", trig);
    if(prev_trig == MOTO_SWITCH_TRIGGER_HOLD) {
        if(trig == MOTO_SWITCH_TRIGGER_PRESS)
            pwr_output.set(POWER_OUTPUT_HIGHBEAM, false);
    }
    else {
        if(trig == MOTO_SWITCH_TRIGGER_PRESS)
            pwr_output.set(POWER_OUTPUT_HIGHBEAM, true);
        else if(trig == MOTO_SWITCH_TRIGGER_RELEASE)
            pwr_output.set(POWER_OUTPUT_HIGHBEAM, false);
    }
    prev_trig = trig;
}

void hornPushButtonCallback(moto_switch_trigger_t trig) {
    LOG_INF("HORN CALLBACK WITH TRIG %u", trig);
    switch(trig) {
        case MOTO_SWITCH_TRIGGER_PRESS:     pwr_output.set(POWER_OUTPUT_HORN, true);    break;
        case MOTO_SWITCH_TRIGGER_RELEASE:   pwr_output.set(POWER_OUTPUT_HORN, false);   break;
        default:        break;
    }
}

void leftSignalPushButtonCallback(moto_switch_trigger_t trig) {

    LOG_INF("LEFT SIG CALLBACK WITH TRIG %u", trig);

    if( (trig == MOTO_SWITCH_TRIGGER_HOLD) ) {
        if(moto.getTriggerState(MOTO_PUSHBUTTON_RIGHT_SIG) == MOTO_SWITCH_TRIGGER_HOLD) {
            blinkers.setMode(TURN_SIGNAL_MODE_HAZARD);
        }
        else {
            blinkers.setMode(TURN_SIGNAL_MODE_LEFT_LONG);
        }
    }
    else if(trig == MOTO_SWITCH_TRIGGER_PRESS) {
        if(blinkers.getMode() == TURN_SIGNAL_MODE_LEFT_LONG) {
            blinkers.setMode(TURN_SIGNAL_MODE_OFF);
        }
        else {
            blinkers.setMode(TURN_SIGNAL_MODE_LEFT_SHORT);
        }
    }
}


void clutchPushButtonCallback(moto_switch_trigger_t trig) {
    LOG_INF("CLUTCH CALLBACK WITH TRIG %u", trig);
}

// RIGHT SIDE
void auxillaryPushButtonCallback(moto_switch_trigger_t trig) {

}


// If engine is OFF, press turns ignition ON, hold sends command to SOLENOID, release turns off SOLENOID
// If engine is ON, press turns ignition OFF
void starterPushButtonCallback(moto_switch_trigger_t trig) {

    // All switch releases kill the motor
    if(trig == MOTO_SWITCH_TRIGGER_RELEASE) {
        starter.disable();
    }
    else {
        // If ignition is enabled
        if(pwr_output.get(POWER_OUTPUT_IGNITION)) {

            if( trig >= MOTO_SWITCH_TRIGGER_PRESS ) {
                starter.enable();
            }

            // // Check.. is motor running? if so.. we're trying to kill it.
            // if(moto.getRPMS() > 1000) {
            //     if(trig == MOTO_SWITCH_TRIGGER_PRESS) {
            //         moto.ignitionEnable(false);
            //         LOG_INF("STARTER - IGNITION KILL CALLED WITH RPM at %u", moto.getRPMS());
            //     }
            // }
            // // If the motor isn't running... I guess we're trying to start it...
            // else {
            //     // Respond to press or hold (allows for long hold to turn on ignition and starter motor in one press)
                
            // }
        }
        else {
            if(trig >= MOTO_SWITCH_TRIGGER_PRESS)
                moto.ignitionEnable(true);
        }
    }
}

void rightSignalPushButtonCallback(moto_switch_trigger_t trig) {

    LOG_INF("RIGHT SIG CALLBACK WITH TRIG %u", trig);

    if( (trig == MOTO_SWITCH_TRIGGER_HOLD) ) {
        if(moto.getTriggerState(MOTO_PUSHBUTTON_LEFT_SIG) == MOTO_SWITCH_TRIGGER_HOLD) {
            blinkers.setMode(TURN_SIGNAL_MODE_HAZARD);
        }
        else {
            blinkers.setMode(TURN_SIGNAL_MODE_RIGHT_LONG);
        }
    }
    else if(trig == MOTO_SWITCH_TRIGGER_PRESS) {
        if(blinkers.getMode() == TURN_SIGNAL_MODE_RIGHT_LONG) {
            blinkers.setMode(TURN_SIGNAL_MODE_OFF);
        }
        else {
            blinkers.setMode(TURN_SIGNAL_MODE_RIGHT_SHORT);
        }
    }
}