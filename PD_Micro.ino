
#include <Wire.h>
#include "src/PD_UFP.h"

class PD_UFP_c PD_UFP;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  PD_UFP.init(PD_POWER_OPTION_MAX_20V);
}

void loop() {
  PD_UFP.run();
  if (PD_UFP.is_power_ready()) { 
    if (PD_UFP.get_voltage() == PD_V(20.0) && PD_UFP.get_current() >= PD_A(1.5)) {
      PD_UFP.set_output(1);   // Turn on load switch 
      PD_UFP.set_led(1);      // Output reach 20V and 1.5A, set indicators on
    } else {
      PD_UFP.set_output(0);   // Turn off load switch
      PD_UFP.blink_led(400);  // Output less than 20V or 1.5A, blink LED
    }
  }
}
