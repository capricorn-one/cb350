#include <Arduino.h>
#include <hal.h>
#include "outputs.h"
#include "moto.h"

void setup() {

  hal_init();

  outputClass::init();

  // Flash the local LED to show that the program is running
  outputs[PWM_CH_LED_LOCAL].set(true);
  delay(250);
  outputs[PWM_CH_LED_LOCAL].set(false);
  delay(250);
  outputs[PWM_CH_LED_LOCAL].set(true);


}

void loop() {



}

