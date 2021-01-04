
#include <avr/power.h>
#include <Wire.h>
#include "src/PD_UFP.h"

class PD_UFP_c PD_UFP;

void setup() {
  Wire.begin();
  PD_UFP.init_PPS(PPS_V(4.2), PPS_A(2.0));
  PD_UFP.clock_prescale_set(2);
  clock_prescale_set(clock_div_2);
}

void loop() {
  PD_UFP.run();
  if (PD_UFP.is_PPS_ready()) {          // PPS trigger success
    PD_UFP.set_output(1);               // Turn on load switch 
    PD_UFP.set_led(1);                  // PPS output 4.2V 2.0A ready
  } else if (PD_UFP.is_power_ready()) { // Fail to trigger PPS, fall back
    PD_UFP.set_output(0);               // Turn off load switch
    PD_UFP.blink_led(400);              // blink LED
  }
}
