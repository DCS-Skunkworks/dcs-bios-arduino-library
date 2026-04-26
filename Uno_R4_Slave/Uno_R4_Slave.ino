
#define DCSBIOS_RS485_SLAVE 1
#define TXENABLE_PIN 2
#define DCSBIOS_RS485_SLAVE_LARGE_BUFFER

// DCS console brightness output uses PWM pin 11.
const uint8_t CONSOLE_BRIGHTNESS_PWM_PIN = 11;
const uint8_t RS485_RX_LED_PIN = LED_RX;
const uint8_t RS485_TX_LED_PIN = LED_TX;
const bool RS485_ACTIVITY_LEDS_ACTIVE_LOW = true;
const uint16_t ACTIVITY_LED_PULSE_MS = 80;
static uint32_t rxPulseUntilMs = 0;
static uint32_t txPulseUntilMs = 0;
static bool sawRs485Rx = false;
static bool sawRs485Tx = false;

void onRs485RxActivity();
void onRs485TxActivity();

#define DCSBIOS_RS485_RX_ACTIVITY() onRs485RxActivity()
#define DCSBIOS_RS485_TX_ACTIVITY() onRs485TxActivity()

#include "DcsBios.h"

static bool pulseActive(uint32_t pulseUntilMs, uint32_t nowMs) {
	return (int32_t)(pulseUntilMs - nowMs) > 0;
}

void onRs485RxActivity() {
	sawRs485Rx = true;
	rxPulseUntilMs = millis() + ACTIVITY_LED_PULSE_MS;
}

void onRs485TxActivity() {
	sawRs485Tx = true;
	txPulseUntilMs = millis() + ACTIVITY_LED_PULSE_MS;
}

static void setActivityLed(uint8_t pin, bool on) {
	if (RS485_ACTIVITY_LEDS_ACTIVE_LOW) {
		digitalWrite(pin, on ? LOW : HIGH);
	} else {
		digitalWrite(pin, on ? HIGH : LOW);
	}
}

void onPltIntLightConsoleChange(unsigned int newValue) {
	analogWrite(CONSOLE_BRIGHTNESS_PWM_PIN, (uint8_t)(newValue >> 8));
}

DcsBios::IntegerBuffer pltIntLightConsoleBuffer(0x2d8a, 0xffff, 0, onPltIntLightConsoleChange);

void setup() {
	pinMode(CONSOLE_BRIGHTNESS_PWM_PIN, OUTPUT);
	analogWrite(CONSOLE_BRIGHTNESS_PWM_PIN, 0);
	pinMode(RS485_RX_LED_PIN, OUTPUT);
	pinMode(RS485_TX_LED_PIN, OUTPUT);
	setActivityLed(RS485_RX_LED_PIN, false);
	setActivityLed(RS485_TX_LED_PIN, false);

	// Quick self-test so we know these indicators are wired and controllable.
	setActivityLed(RS485_RX_LED_PIN, true);
	delay(150);
	setActivityLed(RS485_RX_LED_PIN, false);
	setActivityLed(RS485_TX_LED_PIN, true);
	delay(150);
	setActivityLed(RS485_TX_LED_PIN, false);

	for (uint8_t i = 0; i < 3; i++) {
		analogWrite(CONSOLE_BRIGHTNESS_PWM_PIN, 255);
		delay(500);
		analogWrite(CONSOLE_BRIGHTNESS_PWM_PIN, 0);
		delay(500);
	}
	DcsBios::setup();
}

void loop() {
	uint32_t nowMs = millis();
	setActivityLed(RS485_RX_LED_PIN, sawRs485Rx || pulseActive(rxPulseUntilMs, nowMs));
	setActivityLed(RS485_TX_LED_PIN, sawRs485Tx || pulseActive(txPulseUntilMs, nowMs));

	DcsBios::loop();
}
