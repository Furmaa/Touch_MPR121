# 4 application examples for 2 DEMO touch boards (Layout A and 'Round') with Arduino UNO and a custom made LED switching and supply circuit. 

# Project outline:

+ Application 1, rect_DEMO_proximity: Layout A, includes proximity sensing with added electrode inputs. LED array lights up on proximity with hardcoded intensity. Other LEDs light up proportional to touch intensity. 
+ Application 2, round_DEMO_slider: Layout 'Round', sliding control of outer and inner facing LEDs, light prop. to touch intensity. ON/OFF with middle electrode.
+ Application 3, round_DEMO_sinus: Layout 'Round', press button control of sinus wave light show. ON/OFF with middle electrode.
+ Application 4, round_DEMO_turnwheel: Layout 'Round', turnwheel/dialing the light intensity up clockwise, down counterclockwise. ON/OFF with middle electrode.
+ plotElectrodes: visualize electrode output on Arduino IDE serial plotter.
+ datasheets and application manuals for touch sensing ASIC MPR121
+ flowcharts, assembly documentation: program_flowcharts.pptx, circuit_assembly.docx
+ circuit schematics
+ Adafruit_MPR121_mod: modified Adafruit library, copy this in your local Arduino library
- PCB for integrated support circuit
- detailed Arduino code explanation: hints and remarks commented @ crucial parts in the sketch .ino files and the modified Adafruit library files .h .cpp

# based on: 

Adafruit MPR121 Library [![Build Status](https://github.com/adafruit/Adafruit_MPR121/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_MPR121/actions)

<a href="https://www.adafruit.com/products/1982"><img src="https://cdn-shop.adafruit.com/970x728/1982-00.jpg" height="300"/></a>

Tested and works great with the Adafruit MPR121
  * https://www.adafruit.com/products/1982
  * https://www.adafruit.com/product/2024
  * https://www.adafruit.com/product/2340
 
Check out the links above for our tutorials and wiring diagrams. 
These sensors use I2C to communicate, 2+ pins are required to interface

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

Written by Limor Fried/Ladyada (Adafruit Industries).
MIT license, all text above must be included in any redistribution

# follow these guides to set up an Arduino project in VSCode: 
https://learn.sparkfun.com/tutorials/efficient-arduino-programming-with-arduino-cli-and-visual-studio-code/all
easiest 
https://mithatkonar.com/wiki/doku.php/arduino/configuring_visual_studio_code_for_arduino_development
Making use of a full-featured C/C++ IDE - still needs local Arduino IDE installation to work!