#ifndef MOTO_H
#define MOTO_H

#include <stdint.h>
#include "mm_types.h"
#include "moto_task.h"

#define MOTORCYCLE_TASK_DEFAULT_INTERVAL_MS     50

#define MOTO_SWITCH_HOLD_TIMER_MS               500
#define MOTO_SWITCH_LONG_HOLD_TIMER_MS          2000


class motorcycle : public moto_task {

public:

    motorcycle(const char *name) : moto_task(name, 20) {}
    motorcycle(const char *name, unsigned long aInterval) : moto_task(name, aInterval) {}
    
    void init(void);

    uint8_t getGearNumber(void) { return vfd_telem.gear.num; }

    /// @brief get the gear temperature
    /// @param none
    /// @return uint8_t temperature in degrees C
    int8_t getGearTemperature(void) { return vfd_telem.gear.temperature; }  

    void setMode(moto_mode_t mode);
    moto_mode_t getMode(void) { return moto_mode; }

    void setIgnitionState(bool state);
    void setAuxillaryPowerState(bool state);
    
    void ignitionLockInputCallback(bool state);
    void parkInputCallback(bool state);
    // void rearBrakeInputCallback(bool state);
    
    /// @brief callback for the kickstand input
    /// @param state true if kickstand is down, false if kickstand is up
    /// @return none
    /// @note this function is called from the interrupt context
    void kickstandInputCallback(bool state);

    /// @brief callback for the hall sensor input
    /// @param state true if hall sensor is triggered, false if hall sensor is not triggered
    /// @return none
    void hallSensorInputCallback(bool state);

    /// @brief return the state of the hall sensor
    /// @param none
    /// @return bool true if hall sensor is triggered, false if hall sensor is not triggered
    bool getHallSensorState(void) { return hallSensorState; }

    bool getRearBrakeState(void) { return rear_brake_state; }
    
    bool getFrontBrakeState(void) { return front_brake_state; }
    void setFrontBrakeState(bool state) { front_brake_state = state; }
    
    void handleBarCallbackHandler(moto_pushbutton_num_t pb_num, moto_switch_trigger_t trigger);

    void vfdTelemetryCallbackHandler(vfd_telem_t *telem);

    moto_switch_trigger_t getTriggerState(moto_pushbutton_num_t pb_num) { return switch_trigger_states[pb_num]; }

    void ignitionEnable(bool state);

    /// @brief get the current RPMs
    /// @param none
    /// @return uint16_t RPMs
    uint16_t getRPMS(void) { return vfd_telem.rpm; }

    /// @brief get the current speed in MPH
    /// @param none
    /// @return uint16_t speed in MPH
    uint16_t getSpeedMPH(void) {return vfd_telem.speed; }

protected:

    bool begin(void) { return true; }
    void update(void);
    void exit(void) {}

private:

    moto_mode_t moto_mode;

    moto_switch_trigger_t switch_trigger_states[MOTO_PUSHBUTTON_NUM];

    bool front_brake_state;
    bool rear_brake_state;

    vfd_telem_t vfd_telem;

    bool hallSensorState;

    void startupModeUpdate(void);
    void offModeUpdate(void);
    void ignitionModeUpdate(void);
    void parkingLightsModeUpdate(void);
    void drivingModeUpdate(void);
    void parkedModeUpdate(void);
    void garageModeUpdate(void);
    
};

extern motorcycle moto;

#endif