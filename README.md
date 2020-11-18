# PD_Micro
ATMega32U4 Arduino board with USB Type-C connector and Power Delivery

<img src="images/pd-micro-render-front-small.png" alt="Front Render" width="300">

<img src="images/pd-micro-render-back-small.png" alt="Back Render" width="300">

## Specs:
- ATmega32U4 running at 5 V and 16 MHz
- FUSB302 USB-C PHY (USB PD communication on CC pins)
- TPS62175 DC-DC for 5-20V input, 5V 500mA max output
- On-board 30V 10.4A P-channel MOSFET load switch
- 5 LED for power delivery voltage level
- 3 LED for power delivery current level
- 3.5 mm, 2 position terminal block for power output
- 1.6 x 0.7 inches (0.3 inches longer than pro micro)

<img src="images/pd-micro-pinout-small.png" alt="Pin out" width="600">


# Usage

## Basic Demo

Open PD_Micro.ino with Arduino Sketch. Choose board `Arduino Leoardo`

<img src="images/pd-micro-arduino-sketch.png" alt="arduino-sketch" width="500">


## Set power option
Allocate a PD_UFP object
```
class PD_UFP_c PD_UFP;
```

Initialize it with one of the power option. This set the maximun voltage/current. 
```
PD_UFP.init(PD_POWER_OPTION_MAX_20V);
```

```
enum PD_power_option_t {
    PD_POWER_OPTION_MAX_5V      = 0,
    PD_POWER_OPTION_MAX_9V      = 1,
    PD_POWER_OPTION_MAX_12V     = 2,
    PD_POWER_OPTION_MAX_15V     = 3,
    PD_POWER_OPTION_MAX_20V     = 4,
    PD_POWER_OPTION_MAX_VOLTAGE = 5,
    PD_POWER_OPTION_MAX_CURRENT = 6,
    PD_POWER_OPTION_MAX_POWER   = 7,
};
```

## Run USB PD state machine
Prior to USB PD negotiation completed, `PD_UFP.run()` must be called in a short interval, less than 10ms, to ensure state machine response to negotiation message in time. Long response time may result in power reset cycle initiated by USB PD host.

## Wait for USB PD negotiation completed
` PD_UFP.is_power_ready()` is set when
- PD negotiation completed, PD host sent a power ready message, or
- PD host not exist, CC pins voltage are checked and power level determined.

The negotiation process typically take less than a second.

## Examine the negotiated power value
Examine the negotiated power value. Turn on load switch only when the power requirments are met. PD Micro itself do not have PD over current protection. Any over power may trigger USB PD host power protection, result in voltage dip and MCU reset. 

```
if (PD_UFP.get_voltage() == PD_V(20.0) && PD_UFP.get_current() >= PD_A(1.5)) {
  PD_UFP.set_output(1);   // Turn on load switch 
  PD_UFP.set_led(1);      // Output reach 20V and 1.5A, set indicators on
} else {
  PD_UFP.set_output(0);   // Turn off load switch
  PD_UFP.blink_led(400);  // Output less than 20V or 1.5A, blink LED
}
```

# Bootloader
PD Micro uses `Caterina-promicro16.hex` bootloader provided by Sparkfun. Program it by avrdude and set the corresponding efuse.
```
avrdude -p m32u4 -P usb -c avrispmkii -U flash:w:Caterina-promicro16.hex -U efuse:w:0xcb:m -U hfuse:w:0xd8:m -U lfuse:w:0xff:m
```
