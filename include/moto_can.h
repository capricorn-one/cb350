#ifndef MOTO_CAN_H
#define MOTO_CAN_H

#include <stdint.h>
#include "moto_task.h"

class canbus : public moto_task {

public:

    canbus(const char *name) : moto_task(name, 10) {}
    canbus(const char *name, unsigned long aInterval) : moto_task(name, aInterval) {}
    
    void send_output_state_change(uint16_t states);

protected:

    bool begin(void);
    bool start(void) { return true; }
    bool update(void);
    void exit(void) {}

private:

    bool ready;

};

extern canbus moto_can;

#endif