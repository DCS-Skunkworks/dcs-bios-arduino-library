/*
  =============================================================================
  ESP32-S3 RS-485 Slave for DCS-BIOS
  =============================================================================

  This is an RS485 slave implementation for ESP32-S3. The slave:
  - Receives broadcast export data from the master (cockpit state)
  - Responds to poll requests with any pending input commands
  - Controls physical cockpit hardware (LEDs, servos, displays)
  - Reads physical inputs (buttons, switches, encoders)

  This is MUCH simpler than the Master because:
  - No USB CDC complexity (RS485 only)
  - Reactive design (only responds when polled)
  - Single-threaded operation (no FreeRTOS tasks needed)

  REQUIRES: ESP32-S3 (compile error on other ESP32 variants)

  WIRING for Waveshare ESP32-S3-RS485-CAN:

  ESP32-S3 Pin     RS-485 Transceiver
  ------------     ------------------
  GPIO 17 (TX) --> DI  (Driver Input)
  GPIO 18 (RX) <-- RO  (Receiver Output)
  GPIO 21      --> DE + /RE (Direction Enable)

  RS485 BUS:
  Connect A and B to the RS485 bus (shared with Master and other Slaves)

  =============================================================================
  CONFIGURATION
  =============================================================================
*/

// Define this BEFORE including DcsBios.h
// The number (1-127) is this slave's unique address on the RS485 bus
#define DCSBIOS_RS485_SLAVE 1

// Disable servo library if not using servos (saves memory)
#define DCSBIOS_DISABLE_SERVO

// RS485 pin configuration for Waveshare ESP32-S3-RS485-CAN
#ifndef RS485_TX_PIN
#define RS485_TX_PIN 17       // UART TX -> RS485 DI
#endif

#ifndef RS485_RX_PIN
#define RS485_RX_PIN 18       // UART RX <- RS485 RO
#endif

#ifndef RS485_EN_PIN
#define RS485_EN_PIN 21       // Direction control (active high = TX)
#endif

#include "DcsBios.h"

/*
  =============================================================================
  DCS-BIOS CONTROLS - Add your cockpit controls here
  =============================================================================

  Examples:

  // LED connected to GPIO 2, controlled by Master Caution light
  DcsBios::LED masterCaution(0x1234, 0x0001, 2);

  // Button connected to GPIO 4, sends "UFC_1" command
  DcsBios::Switch2Pos ufc1("UFC_1", 4);

  // Rotary encoder on pins 5,6 for heading bug
  DcsBios::RotaryEncoder hdgBug("HDG_BUG", "-1", "+1", 5, 6);

  // Potentiometer on analog pin for brightness control
  DcsBios::Potentiometer brightness("BRIGHTNESS", A0);

  See the DCS-BIOS documentation for the full list of available controls
  and their addresses for your specific aircraft module.
*/

// ============================================================================
// EXAMPLE: Onboard LED shows communication status
// ============================================================================

// Most ESP32-S3 boards have an onboard LED, often on GPIO 48 or GPIO 2
// Uncomment and adjust the pin for your board:
// #define ONBOARD_LED 48

#ifdef ONBOARD_LED
void setup() {
    pinMode(ONBOARD_LED, OUTPUT);
    DcsBios::setup();
}

void loop() {
    // Blink LED to show we're running
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
        lastBlink = millis();
        digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
    }

    DcsBios::loop();
}
#else
// No onboard LED - simple setup/loop
void setup() {
    DcsBios::setup();
}

void loop() {
    DcsBios::loop();
}
#endif
