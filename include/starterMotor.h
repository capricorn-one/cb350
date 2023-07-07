#ifndef __STARTER_MOTOR_H__
#define __STARTER_MOTOR_H__

#include "mm_types.h"
#include "moto_task.h"

class starterMotor : public moto_task {

public:

    starterMotor(const char * name) : moto_task(name, 10) {}
    starterMotor(const char *name, unsigned long aInterval) : moto_task(name, aInterval) {}

    uint8_t getDatapointMax(void) { return datapoint_counter; }
    solenoid_datapoint_t * getDatapoints(void) { return datapoints; }
    solenoid_datapoint_t getDatapoint(uint8_t n) { return datapoints[n&0x7F]; }

protected:

    bool begin(void);
    void update(void);
    void exit(void);

private:

    uint32_t enableTimer;

    void solenoid_enable(bool state);

    void solenoid_read_current(void);

    solenoid_datapoint_t datapoints[128];
    uint8_t datapoint_counter;
};

extern starterMotor starter;

#endif