#define DCSBIOS_RS485_SLAVE 1
#define DCSBIOS_RS485_R4
#define TXENABLE_PIN 2
#define DCSBIOS_RS485_SLAVE_LARGE_BUFFER

#include "DcsBios.h"

// Console lighting removed — using onboard LED for Master Caution only

// Use the onboard LED for Master Caution so it's accessible to other users
// Address and mask taken from DCS datarefs for master caution
DcsBios::LED masterCaution(0x1012, 0x0800, LED_BUILTIN);

void setup() {
	DcsBios::setup();
}

void loop() {
	DcsBios::loop();
}