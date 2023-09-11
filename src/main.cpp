#include <Arduino.h>
#include <ArduinoJson.h>
#include <TaskScheduler.h>
#include "moto.h"
#include "telemetry.h"
#include "moto_can.h"
#include "ble.h"
#include "turnSignals.h"
#include "imu.h"

typedef struct {
    uint16_t on;
    uint16_t off;
} temp_t;

Scheduler ts;

void setup() {

  hal_init();

  moto_can.init(&ts);

  telem.init(&ts);
  
  blinkers.init(&ts);

  // imu.init(&ts);

  // ble.init(&ts);

  moto.init(&ts);

  LOG_INF("\n*********** CB350 STARTUP ***********");

  moto.setMode(MOTO_MODE_STARTUP);

}

void loop() {

  ts.execute();

}
