# DCS-BIOS Arduino Library

This is an Arduino library that makes it easy to write sketches that talk to DCS-BIOS.

For more information and documentation, see the [DCS-BIOS project.](https://github.com/dcs-bios/dcs-bios)

v0.2.12
--------
add polling support to new controls (by Blue73 [ED Forum](https://forums.eagle.ru/showthread.php?t=230222) )

v0.2.13
--------
Add Pullrequests from orginal site:

Syamskoy:
-Added support for stm32 board

Exo7:
-Adding dimmer function : to control panel backlighting with an Arduino output (see Dimmer.txt)
-Adding bcdWheels input type : to work with bcd thumb wheels.
-Edit encoder.h to work with CTS288 encoders.
[GitHub DCS-BIOS Arduino-library](https://github.com/dcs-bios/dcs-bios-arduino-library/pulls) )

v0.2.14
--------
New Matrix-Compatible Button class
This class expects a char array that saves the state of all buttons in a button matrix.
It modifies the initiation and the class to look for a certain value in an array,
instead of a high or low on a physical pin.

ChronoZoggt - via Discord