
#include <Wire.h>
#include "src/PD_UFP.h"

class PD_UFP_c PD_UFP;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  PD_UFP.init(PD_POWER_OPTION_MAX_12V);
}

void loop() {
  PD_UFP.run();
  if (PD_UFP.is_power_ready()) { 
    if (PD_UFP.get_voltage() == PD_V(12.0) && PD_UFP.get_current() >= PD_A(3.0)) {
      PD_UFP.set_output(1);
      PD_UFP.set_led(1);
    } else {
      PD_UFP.set_output(0);
      PD_UFP.blink_led(400);
    }
  }
}
