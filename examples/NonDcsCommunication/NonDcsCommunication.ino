#define DCSBIOS_DEFAULT_SERIAL
#include "DcsBios.h"

// This example demonstrates the ability for non-DCS applications to communicate with your arduino sketch.
// Ensure that the NonDcsStreamBuffer<###> initializes a buffer large enough to receive whatever strings you plan on sending.
// Note that DCS must be closed or idle/paused for this to work reliably.
// Sample usage: Have a PC side script/app to test LEDs pre flight, or turn off all zero servos after DCS has been closed.
// From the PC side, you must send the following hex characters to "break out" of the DCS sync stream:
// 0xA2, 0x3E, 0x62, 0x98 
// followed by the string you want to send and a newline \n.

void setup() {
  DcsBios::setup();
}

void loop() {
  DcsBios::loop();
}

void onNonDcsLineReceived(char* newValue) {
    // You received a string from an application other than DCS.
    // React in some way by writing code here.
    // The string that was sent is pointed to by newValue,
    // so you can react conditionally.
    if( strcmp(newValue, "Power=Off") == 0 )
    {
      digitalWrite(1, LOW); // Turn off an LED on PIN 1
      analogWrite(2,0); // Zero a servo on pin 2
    }
}
DcsBios::NonDcsStreamBuffer<24> nonDcsLineBuffer(onNonDcsLineReceived);
