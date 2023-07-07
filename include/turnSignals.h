#ifndef _TURN_SIGNALS_H_
#define _TURN_SIGNALS_H_

#include <stdint.h>
#include <stdbool.h>
#include "moto.h"
#include "moto_task.h"

#define BLINKER_TIMER_MS        300     // ~100 bps as per SAE J590b between 60-120
#define BLINKER_SHORT_COUNT     5

typedef enum {
    TURN_SIGNAL_MODE_OFF = 0,
    TURN_SIGNAL_MODE_LEFT_SHORT = 1,
    TURN_SIGNAL_MODE_LEFT_LONG = 2,
    TURN_SIGNAL_MODE_RIGHT_SHORT = 3,
    TURN_SIGNAL_MODE_RIGHT_LONG = 4,
    TURN_SIGNAL_MODE_HAZARD = 5,
} turn_signal_mode_t;


class turn_signals : public moto_task {

public:

    turn_signals(const char *name) : moto_task(name) {}
    ~turn_signals() {}

    void setMode(turn_signal_mode_t newMode);
    turn_signal_mode_t getMode(void) { return mode; }

protected:

    bool begin(void) { return true; }
    void update(void);
    void exit(void) {}

private:

    turn_signal_mode_t mode;

    int8_t signal_count;

    uint32_t blinker_timer;

};

extern turn_signals blinkers;

#endif