#include "turnSignals.h"
#include "power.h"
#include "hal.h"

bool turn_signals::update() {

    if(mode == TURN_SIGNAL_MODE_OFF) {
        return true;
    }

    uint32_t elapsedTime = hal_millis() - blinker_timer;    

    if( elapsedTime < BLINKER_TIMER_MS) {

        switch(mode) {
            case TURN_SIGNAL_MODE_LEFT_SHORT:
            case TURN_SIGNAL_MODE_LEFT_LONG:
                pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(false, false);
                pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(true, false);
                mm_power::update_all();
                break;
            
            case TURN_SIGNAL_MODE_RIGHT_SHORT:
            case TURN_SIGNAL_MODE_RIGHT_LONG:
                pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(false, false);
                pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(true, false);
                mm_power::update_all();
                break;
            
            case TURN_SIGNAL_MODE_HAZARD:
                pwr_output[POWER_OUTPUT_TAIL_LIGHT].set(true, false);
                pwr_output[POWER_OUTPUT_HEADLIGHT].set(true, false);
                pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(true, false);
                pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(true, false);
                mm_power::update_all();
                break;
            
            default:
                pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(false, false);
                pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(false, false);
                mm_power::update_all();
                break;         
        }

    }
    else if (elapsedTime < (BLINKER_TIMER_MS * 2)) {
        
        if(mode == TURN_SIGNAL_MODE_HAZARD) {
            pwr_output[POWER_OUTPUT_TAIL_LIGHT].set(false, false);
            pwr_output[POWER_OUTPUT_HEADLIGHT].set(false, false);
            pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(false, false);
            pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(false, false);
            mm_power::update_all();
        }
        else if ( (mode == TURN_SIGNAL_MODE_LEFT_SHORT) || (mode == TURN_SIGNAL_MODE_LEFT_LONG) ) {
            pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(false, false);
            pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(false, false);
            mm_power::update_all();
        }
        else if( (mode == TURN_SIGNAL_MODE_RIGHT_SHORT) || (mode == TURN_SIGNAL_MODE_RIGHT_LONG) ) {
            pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(false, false);
            pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(false, false);
            mm_power::update_all();
        }

    }
    else {
        if(signal_count > 0) {
            if(--signal_count == 0) {
                setMode(TURN_SIGNAL_MODE_OFF);
            }
        }
        blinker_timer = hal_millis();
    }

    return true;

}

void turn_signals::setMode(turn_signal_mode_t newMode) {

    LOG_INF("BLINKER - SET MODE %u", newMode);

    switch(newMode) {

        case TURN_SIGNAL_MODE_OFF:
            if ( (mode == TURN_SIGNAL_MODE_LEFT_SHORT) || (mode == TURN_SIGNAL_MODE_LEFT_LONG) ) {
                pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(false, false);
                pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(false, false);
                mm_power::update_all();
            }
            else if( (mode == TURN_SIGNAL_MODE_RIGHT_SHORT) || (mode == TURN_SIGNAL_MODE_RIGHT_LONG) ) {
                pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(false, false);
                pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(false, false);
                mm_power::update_all();
            }
            else if( mode == TURN_SIGNAL_MODE_HAZARD) {

                pwr_output[POWER_OUTPUT_TAIL_LIGHT].set(false, false);
                pwr_output[POWER_OUTPUT_HEADLIGHT].set(false, false);
                pwr_output[POWER_OUTPUT_SIGNAL_LEFT].set(false, false);
                pwr_output[POWER_OUTPUT_SIGNAL_RIGHT].set(false, false);
                mm_power::update_all();
            }
            signal_count = 0;
            break;

        case TURN_SIGNAL_MODE_LEFT_SHORT:
            if(mode == TURN_SIGNAL_MODE_OFF)
                blinker_timer = hal_millis();
            signal_count = BLINKER_SHORT_COUNT;
            break;

        case TURN_SIGNAL_MODE_LEFT_LONG:
            signal_count = -1;
            break;
        
        case TURN_SIGNAL_MODE_RIGHT_SHORT:
            if(mode == TURN_SIGNAL_MODE_OFF)
                blinker_timer = hal_millis();
            signal_count = BLINKER_SHORT_COUNT;
            break;

        case TURN_SIGNAL_MODE_RIGHT_LONG:
            signal_count = -1;
            break;
        
        case TURN_SIGNAL_MODE_HAZARD:
            signal_count = -1;
            break;
    }

    if( (mode == TURN_SIGNAL_MODE_OFF) && (newMode != TURN_SIGNAL_MODE_OFF) ) {
        enable();
    }
    
    if( (mode != TURN_SIGNAL_MODE_OFF) && (newMode == TURN_SIGNAL_MODE_OFF) ) {
        disable();
    }
    
    mode = newMode;
}

turn_signals blinkers("blinkers");
