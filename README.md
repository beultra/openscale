# Decent Open Scale (aka "openscale")
Decent Open Scale
Copyright 2024 Decent Espresso International Ltd

Credits:
Invention and authorship: Chen Zhichao (aka "Sofronio")

# Simplified bare bones version for BT only use with the Decent machine.

Only requires the HX711 library below. Note for the wiring - any two pins
available can be used on an ESP32 module - just set those up in so_config.h .

The scale does not need to be put into pairing mode. This version is designed
to be permanently powered via USB (or some other supply) and will not turn
off. Just about any ESP32 board/module should work.

To calibrate the load cell, enable the serial monitor. Baud rate should be
11500. Send 'c' and enter to the module, and follow the instructions. Just
use any known weight.

# Introduction:
The Decent Open Scale is a full open sourced(software/hardware/cad design) BLE scale. Currently you can use it with de1app and Decent Espresso machine. But with Decent Scale API, you can use it for anything.<br />
To make it work, you need at least an ESP32 for MCU, a Loadcell for weighing, an HX711 for Loadcell ADC, a SSD1306 for OLED display, a MPU6050 for gyro function.<br />
If you want to use it unplugged, you'll also need a 3.7v battery.<br />
If you only want to burn the firmware, please read How to upload HEX file.<br />

# Library needed:
AceButton https://github.com/bxparks/AceButton <br />
Stopwatch_RT https://github.com/RobTillaart/Stopwatch_RT <br />
HX711_ADC https://github.com/olkal/HX711_ADC <br />
u8g2 https://github.com/olikraus/u8g2 <br />

# How to upload HEX file
Use OpenJumper™ Serial Assistant, link as below.<br />
https://www.cnblogs.com/wind-under-the-wing/p/14686625.html <br />
