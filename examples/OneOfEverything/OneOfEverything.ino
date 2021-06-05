#define DCSBIOS_DEFAULT_SERIAL
#include <DcsBios.h>

/* This file serves two purposes:
 *#1 - Provided compilable demonstrations of each type of controls availble in DCS-Bios arduino library.  This is not meant to be deployed onto
 *      hardware, as several pins conflict and the messages are made up.
 *#2 - Provide a very basic testing suite that developers can use to ensure that changes do not break existing sketches
 */

// Buttons
///////////
// The simplest push button, a single momentary button on a single pin, which sends ARG_0 to MSG_0
DcsBios::ActionButton iffDec("IFF_CODE", "INC", 1);

// Used when a physical switch is a momentary button, but needs to sent alternating arguments each time it is pressed
DcsBios::ToggleButton toggleButtonExample("MSG_0", "ARG_0", "ARG_1", 1);

// Switches
////////////
// A standard two position on/off
DcsBios::Switch2Pos switch2PosExample("MSG_0", 1);
// A three position on/off/on switch
DcsBios::Switch3Pos switch3PosExample("MSG_0", 1, 2);
// A multiple position switch, often a rotary switch
const byte multiPosPins[4] = {1,2,3,4};
DcsBios::SwitchMultiPos switchMulitPosExample("MSG_0", multiPosPins, 4);

// Analogs
///////////
// Use an analog input, divided into discrete steps
DcsBios::AnalogMultiPos analogMultiPosExample("MSG_0", 1, 10);

// Other stuff
// A Binary Coded Decimal wheel usually displaying digits for numeric entry, i.e. IFF code wheels.
DcsBios::BcdWheel bcdWheelExample("MSG_0", 1, 2);
// A special case of bcdWheel that will send a radio frequency instead of raw digit
DcsBios::RadioPreset radioPresetExample("MSG_0", 1, 2, 3, 4, 5);

// Spinning things
///////////////////
// Rotary encoder on two pins to send INC/DEC arguments when rotated
DcsBios::RotaryEncoder rotaryEncoderExample("MSG_0", "ARG_DEC", "ARG_INC", 1, 2);
// A rotary encoder which will send larger increments when used continuously.  Originally written for faster gross adjustments to HSI.
DcsBios::RotaryAcceleratedEncoder rotaryAcceleratedEncoderExample("MSG_0", "ARG_DEC", "ARG_INC", "FAST_INC", "FAST_DEC", 1, 2);
// A linear/analog axis on a single pin
DcsBios::Potentiometer potentiometerExample("MSG_0", 1);
// An inverted version of a linear axis control
DcsBios::InvertedPotentiometer invertedPotentiometerExample("MSG_0", 1);

// Outputs
///////////
// A single LED
DcsBios::LED masterCaution(0x1012, 0x0800, 13);
// An analog output with a value that comes from a DCS address
DcsBios::Dimmer(0x1012, 13);
// A servo motor controlleed from DCS, i.e. a guage.
DcsBios::ServoOutput(0x1012, 13, 544, 2400);

void setup() {
  DcsBios::setup();
}

void loop() {
  DcsBios::loop();
}
