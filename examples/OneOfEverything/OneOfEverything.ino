/* use '#define DCSBIOS_DEFAULT_SERIAL' instead if your Arduino board
 *  does not feature an ATMega328 or ATMega2650 controller.
 */
#define DCSBIOS_DEFAULT_SERIAL
#include <DcsBios.h>

DcsBios::AnalogMultiPos analogMultiPosExample("MSG_0", 1, 10);
DcsBios::RadioPreset radioPresetExample("MSG_0", 1, 2, 3, 4, 5);
DcsBios::BcdWheel bcdWheelExample("MSG_0", 1, 2);
DcsBios::ActionButton actionButtonExample("MSG_0", "ARG_0", 1);
DcsBios::ToggleButton toggleButtonExample("MSG_0", "ARG_0", "ARG_1", 1);
DcsBios::RotaryEncoder rotaryEncoderExample("MSG_0", "ARG_DEC", "ARG_INC", 1, 2);
DcsBios::RotaryAcceleratedEncoder rotaryAcceleratedEncoderExample("MSG_0", "ARG_DEC", "ARG_INC", "FAST_INC", "FAST_DEC", 1, 2);
DcsBios::Potentiometer potentiometerExample("MSG_0", 1);
DcsBios::InvertedPotentiometer invertedPotentiometerExample("MSG_0", 1);
DcsBios::Switch2Pos switch2PosExample("MSG_0", 1);
DcsBios::Switch3Pos switch3PosExample("MSG_0", 1, 2);
const byte multiPosPins[4] = {1,2,3,4};
DcsBios::SwitchMultiPos switchMulitPosExample("MSG_0", multiPosPins, 4);

DcsBios::Dimmer defaultDimmerExample(0x1012, 5);
DcsBios::Dimmer invertedDimmerExample(0x1012, 5, 200,0);

DcsBios::LED masterCaution(0x1012, 0x0800, 13);

void setup() {
  DcsBios::setup();
}

void loop() {
  DcsBios::loop();
}
