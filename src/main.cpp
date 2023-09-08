#include <Arduino.h>
#include <ArduinoJson.h>
#include <hal.h>
#include "outputs.h"
#include "moto.h"
#include "telemetry.h"
#include "moto_can.h"
#include "ble.h"

#define _TASK_OO_CALLBACKS

#include <TaskScheduler.h>

void setup() {

  hal_init();

  telem.init();
  
  moto_can.init();
  
  moto.init();

  ble.init();
 
  Serial.println("Setup complete.");
}

#define MAX_OUTPUTS 1000

void loop() {

  // uint16_t side_lights = 0;

  moto_task::run();

  // outputs[PWM_CH_AUX_SIDELIGHTS].set_duty_cycle(0.0);

  // while(1) {

  //   outputs[PWM_CH_AUX_LOGO].set_duty_cycle( side_lights / (float)MAX_OUTPUTS / 5.00 );
  //   outputs[PWM_CH_LED_LOCAL].set_duty_cycle( side_lights / (float)MAX_OUTPUTS / 5.00 );

  //   if(++side_lights > MAX_OUTPUTS) side_lights = 0;

  //   delay(5);

  // }

}

