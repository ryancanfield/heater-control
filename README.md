## Overview

This sketch implements an integral-cycle-control dimmer for controlling a high power heating element. The TRIAC is only controlled at the zero crossing so full AC cycles are always provided to the controlled element.
The intent of this type of control is to:
* Reduce current transients to cold heater coils
* Reduce electrical switch noise and harmonics
* Reduce audible noise from heater coils.

Variable control over a heater coil is able to control temperature to a high degree of stability compared to a simple on-off control.
One application for this is an under-desk heater, where rapid changes to output are distracting and undesired.

## Hardware Requirements
* **Arduino Board:** [e.g., Arduino Uno, ESP32 Dev Module, Arduino Nano]
* **Components:**
    * [TRIAC board with zero-crossing output]
    * [2x momentary buttons]
    * [DHT11 or DHT22 Temperature Sensor]
    * [Breadboard, Jumper Wires, etc.]

## Software Requirements

List any software dependencies beyond the Arduino IDE itself.

* **Arduino IDE:** Version [1.8.19] or newer.
* **Libraries:**
    All libraries are installed using the Arduino IDO
    * [DHT] - DHT temperature sensor.
    * [OneButton] - Button objects for detecting long and short presses.
    * [PID_v1] - Simple PID control.
## License
This project is licensed under the [MIT License](LICENSE.txt) - see the [LICENSE](LICENSE.txt) file for details.
