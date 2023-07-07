#include "turnSignals.h"
#include "power.h"
#include "hal.h"

void turn_signals::update() {

    if(mode == TURN_SIGNAL_MODE_OFF) {
        return;
    }

    uint32_t elapsedTime = hal_millis() - blinker_timer;    

    if( elapsedTime < BLINKER_TIMER_MS) {

        switch(mode) {
            case TURN_SIGNAL_MODE_LEFT_SHORT:
            case TURN_SIGNAL_MODE_LEFT_LONG:
                pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
                pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, true);
                break;
            
            case TURN_SIGNAL_MODE_RIGHT_SHORT:
            case TURN_SIGNAL_MODE_RIGHT_LONG:
                pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);
                pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, true);
                break;
            
            case TURN_SIGNAL_MODE_HAZARD:
                pwr_output.set(POWER_OUTPUT_TAIL_LIGHT, true);
                pwr_output.set(POWER_OUTPUT_HEADLIGHT, true);
                pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, true);
                pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, true);
                break;
            
            default:
                pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);
                pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
                break;         
        }

    }
    else if (elapsedTime < (BLINKER_TIMER_MS * 2)) {
        
        if(mode == TURN_SIGNAL_MODE_HAZARD) {
            pwr_output.set(POWER_OUTPUT_TAIL_LIGHT, false);
            pwr_output.set(POWER_OUTPUT_HEADLIGHT, false);
            pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
            pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);
        }
        else if ( (mode == TURN_SIGNAL_MODE_LEFT_SHORT) || (mode == TURN_SIGNAL_MODE_LEFT_LONG) ) {
            pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
            pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);   
        }
        else if( (mode == TURN_SIGNAL_MODE_RIGHT_SHORT) || (mode == TURN_SIGNAL_MODE_RIGHT_LONG) ) {
            pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
            pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);   
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

}

void turn_signals::setMode(turn_signal_mode_t newMode) {

    LOG_INF("BLINKER - SET MODE %u", newMode);

    switch(newMode) {

        case TURN_SIGNAL_MODE_OFF:
            if ( (mode == TURN_SIGNAL_MODE_LEFT_SHORT) || (mode == TURN_SIGNAL_MODE_LEFT_LONG) ) {
                pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
                pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);
            }
            else if( (mode == TURN_SIGNAL_MODE_RIGHT_SHORT) || (mode == TURN_SIGNAL_MODE_RIGHT_LONG) ) {
                pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
                pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);
            }
            else if( mode == TURN_SIGNAL_MODE_HAZARD) {
                pwr_output.set(POWER_OUTPUT_TAIL_LIGHT, false);
                pwr_output.set(POWER_OUTPUT_HEADLIGHT, false);
                pwr_output.set(POWER_OUTPUT_SIGNAL_LEFT, false);
                pwr_output.set(POWER_OUTPUT_SIGNAL_RIGHT, false);
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

    mode = newMode;
}

turn_signals blinkers("blinkers");
