#include <Arduino.h>
#include <hal.h>
#include "outputs.h"
#include "moto.h"

#define _TASK_OO_CALLBACKS

#include <TaskScheduler.h>

void setup() {

  hal_init();

  moto.init();
  
}

void loop() {

  moto_task::run();

}

