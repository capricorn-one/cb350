#include <Arduino.h>
#include <hal.h>
#include "outputs.h"
#include "moto.h"
#include "telemetry.h"
#include "moto_can.h"
#include "gui.h"

#define _TASK_OO_CALLBACKS

#include <TaskScheduler.h>

void setup() {

  hal_init();

  outputClass::init();

  // Flash the local LED to show that the program is running
  outputs[PWM_CH_LED_LOCAL].set(true);
  delay(250);
  outputs[PWM_CH_LED_LOCAL].set(false);
  delay(250);
  outputs[PWM_CH_LED_LOCAL].set(true);


  telem.init();
  
  moto_can.init();
  
  moto.init();
  
  gui.init();

  
  
}

void loop() {

  moto_task::run();

}

