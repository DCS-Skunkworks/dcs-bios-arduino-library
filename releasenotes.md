## v0.3.0

First version released after forking from the dcs-bios repo.  Several changes are included and only vaguely documented as the primary developer becomes oriented within the project.

- Remove pollInputCurrent in favor of DcsBios::resetAllStates(), which can be used to re-sync the state of every input.  Intended to be called from the sketch using something like:

  ```c++
  void onAcftNameBufferChange(char* newValue) {
  
    // Change of Aircraft
    DcsBios::resetAllStates();
  }
  
  DcsBios::StringBuffer<16> AcftNameBuffer(0x0000, onAcftNameBufferChange);
  ```

- AnalogMultiPos extracted into it's own file, and optimized/fixed.

- Resolved "hanging else" issues throughout which could affect certain compilers (but not Arduino, so no-factor for most applications).

- Differentiated Arduino library name from original fork.

- Resolve redef warning for PRR0 for certain boards (Teensy++2.0 for certain, likely more).

- Added RotaryAcceleratedEncoder, an extended RotaryEncoder which aims to solve issues with noisy/faulty rotaries by tracking rotary momentum and using that as a filter to counter-momentum signalling.

- Removed several SwitchMatrix references which appeared to be incomplete in the original fork.

- Added debounce support for Switch3Pos

---------
v0.2.22
---------
#add AnalogMultiPos
Usefull for resistor ladders for rotary switches to reduce inputs.

from Tekadept - via Discord

---------
v0.2.21
---------
#add Inverted Potentiometer Class 
 This class will allow users who have wired their potentiometers the wrong way round to simply invert the input in code, 
 rather than having to re-solder everything the other way around.

 from HydroZA - https://github.com/HydroZA/dcs-bios-arduino-library


#add Mod to allow UART selection on Mega in RS485 Slave 
 (see folder dcs-bios-arduino-library-0.2.21\src\internal\UART.Mod)

 from Pademelon - via Discord

---------
v0.2.20
---------
#add debounce for Switch2Pos

 from alexproust - https://github.com/alexproust/dcs-bios-arduino-library

---------
v0.2.19
---------
#New Font for CDU  (/examples/CDU Font)
 Then rename the DefaultsFonts.c file in your UTFT library to DefaultFonts.bak and place the new file in the same subdir.

 from Arjan - https://forums.eagle.ru/showthread.php?t=250363

---------
v0.2.18
---------
#Fixes for Dimmer
#Fixes for Matrix-Compatible Button

#Fixes made by Azza276 and ChronoZoggt - via Discord, ED Forums

#Add Button_Matrix_Example by Azza276

#Seperate the Switch Matrix Library in an second Zip with other librarys

---------
v0.2.17 - does not work
---------
#Add Switch Matrix Library

by dagoston93 - https://github.com/dagoston93/SwitchMatrix

---------
v0.2.16
---------
#Support for a matrix of 2 and 3 position switches.
#Support for multi-position rotary switches configured as voltage dividers (1k resistors between pins) up to 20 positions.

from Dehuman - https://forums.eagle.ru/showthread.php?t=240525

---------
v0.2.15
---------
#Dynamic Mapping based on Aircraft Type Code
 This is the code that will enable you to change the control mapping based on the aircraft loaded. 
 I've tested it with my A-10C Fuel Panel which only has a bunch of Switch2's and a couple of POTs, 
 but I've shown how to also use with the other switch types. 
 I've had to add a small member function to all the DCS-BIOS input libraries to gain access to the msg_ private variable.

 from Blue73 - https://forums.eagle.ru/showthread.php?t=231236

---------
v0.2.14
---------
#New Matrix-Compatible Button class
 This class expects a char array that saves the state of all buttons in a button matrix.
 It modifies the initiation and the class to look for a certain value in an array,
 instead of a high or low on a physical pin.

ChronoZoggt - via Discord

---------
v0.2.13
---------
#Add Pullrequests from orginal site:

Syamskoy:
-Added support for stm32 board

Exo7:
-Adding dimmer function : to control panel backlighting with an Arduino output (see Dimmer.txt)
-Adding bcdWheels input type : to work with bcd thumb wheels.
-Edit encoder.h to work with CTS288 encoders.

https://github.com/dcs-bios/dcs-bios-arduino-library/pulls

---------
v0.2.12
---------
So far the switches.h and potentiometers.h functions have a new function call pollInputCurrent(). 
With regular polling the initial state of your switch boxes will be force pushed to DCS. 
This is useful as the current control state is synced with the sim control state.

from Blue73 - https://forums.eagle.ru/showpost.php?p=3766416&postcount=1

---------
v0.2.11
---------
Last version from jboecker
https://github.com/dcs-bios/dcs-bios-arduino-library/releases