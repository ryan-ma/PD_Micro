

#include <avr/power.h>
#include <Wire.h>
#include "src/PD_UFP.h"

// To log raw packet, init PD_UFP with parameter PD_UFP(PD_UFP(PD_LOG_LEVEL_VERBOSE)
class PD_UFP_log_c PD_UFP(PD_LOG_LEVEL_VERBOSE);

void setup() {
  Serial1.begin(115200*2);  // Serial1 is hardware serial on Pin 1 and 2
  Wire.begin();
  PD_UFP.init_PPS(PPS_V(4.2), PPS_A(2.0));
  PD_UFP.clock_prescale_set(2);
  clock_prescale_set(clock_div_2);
}

void loop() {
  PD_UFP.run();
  PD_UFP.print_status(Serial1);
  if (PD_UFP.is_PPS_ready()) {          // PPS trigger success
    PD_UFP.set_output(1);               // Turn on load switch 
    PD_UFP.set_led(1);                  // PPS output 4.2V 2.0A ready
  } else if (PD_UFP.is_power_ready()) { // Fail to trigger PPS, fall back
    PD_UFP.set_output(0);               // Turn off load switch
    PD_UFP.blink_led(400);              // blink LED
  }
}
