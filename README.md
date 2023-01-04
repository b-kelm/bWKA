# Small Wind Turbine Generator System 

Author: Benjamin Kelm
Date: 04.01.2023 
Name: bWKA -  baby WindKraftAnlage (german acronym)


## General Description and Limits of Operation
This software has been written for the use of the bWKA hardware, as detailed in this video: "Build your own Wind Turbine - Part 1 (Theory and Design)" by Benjamin Kelm on Youtube

**The software does the following:**
1. Read out the PWM signal a servo tester
2. Control rotational speed of a motor with an ESC (via Dshot protocol) - in this case of a windmill, the ESC *regeneratively breaks* the motor, feeding energy back into the battery
3. Log Telemetry Data from the ESC onto an micro-SD card.

**Note:** This project is *educational*. Do not expect high power outputs, nor expect it to scale to bigger turbines without adaptation. 


## Electronics Description

Target Board: Teensy 4.1 
Environment: [Teensyduino](https://www.pjrc.com/teensy/td_download.html) 

### Components Used:

1. [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) with micro-SD card inserted
2. [KISS ESC 32A](https://www.flyduino.net/shop/product/pr2200-kiss-esc-3-6s-32a-45a-limit-32bit-brushless-motor-ctrl-2961) (needs to support Regenerative Breaking)
3. [PWM Servo Tester](https://hobbyking.com/de_de/hobbykingtm-servo-tester.html) or similar

### Wiring:

ESC is wired up normally to Battery and Motor
TLM Pad (or TX) on the ESC is wired to the Teensy on *Pin XY*.
The Servo Tester is wired to the Teensy on *Pin 14*

## Program Description
The program is heavily relying on these two sources:
- [teensyshot](https://github.com/jacqu/teensyshot) by Jacques Gangloff and Arda Yigit
- [teensy SD Logger](https://forum.pjrc.com/threads/66165-Minimalistic-SdFat-Datalogger-for-Teensy4-1-Example) by MBorgerson

To compile, the following files from **teensyshot** need to be in the same directory:
- DSHOT.h and .cpp
- ESCCMD.h and .cpp



## Original Idea
The original idea was to develop an MPPT controller similar to [this one](https://www.hackster.io/philippedc/a-wind-turbine-mppt-regulator-with-an-arduino-uno-783462). Once I have a current sensor implemented, the code can be adapted to track the maximum power.


## Disclaimer
This program is free software. It comes without any warranty, to the extent permitted by applicable law. 
You can redistribute it and/or modify it under the terms of the [MIT License](https://github.com/git/git-scm.com/blob/main/MIT-LICENSE.txt), as published by Scott Chacon and others.
