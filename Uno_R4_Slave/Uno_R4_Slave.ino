
#define DCSBIOS_RS485_SLAVE 1
#define DCSBIOS_RS485_R4
#define TXENABLE_PIN 2
#define DCSBIOS_RS485_SLAVE_LARGE_BUFFER

// DCS console brightness output uses PWM pin 11.
const uint8_t CONSOLE_BRIGHTNESS_PWM_PIN = 11;

#include "DcsBios.h"

void onPltIntLightConsoleChange(unsigned int newValue) {
	analogWrite(CONSOLE_BRIGHTNESS_PWM_PIN, (uint8_t)(newValue >> 8));
}

DcsBios::IntegerBuffer pltIntLightConsoleBuffer(0x2d8a, 0xffff, 0, onPltIntLightConsoleChange);

void setup() {
	pinMode(CONSOLE_BRIGHTNESS_PWM_PIN, OUTPUT);
	analogWrite(CONSOLE_BRIGHTNESS_PWM_PIN, 0);

	for (uint8_t i = 0; i < 3; i++) {
		analogWrite(CONSOLE_BRIGHTNESS_PWM_PIN, 255);
		delay(500);
		analogWrite(CONSOLE_BRIGHTNESS_PWM_PIN, 0);
		delay(500);
	}
	DcsBios::setup();
}

void loop() {
	DcsBios::loop();
}
