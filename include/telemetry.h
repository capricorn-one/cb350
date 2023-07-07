#ifndef _TELEMETRY_H
#define _TELEMETRY_H

#include "hal.h"
#include "moto_task.h"

class telemetry : public moto_task {

public:

    telemetry(const char *name) : moto_task(name) {}
    ~telemetry() {}
    
    void init(void);
    
    float get_battery_voltage(void) { return battery_voltage; }
    float get_starter_current(void) { return starter_current_local; }
    float get_regulator_current(void) { return regulator_current; }
    float get_load_current(void) { return load_current; }

protected:

    bool begin(void) { return true; }
    void update(void);
    void exit(void) {}

private:

    float regulator_current;
    float load_current;

    float battery_voltage;
    float starter_current_local;

};

extern telemetry telem;

#endif