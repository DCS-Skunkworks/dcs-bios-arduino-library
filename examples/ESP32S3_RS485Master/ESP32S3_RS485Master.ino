/*
  ESP32-S3 RS-485 Master for DCS-BIOS

  This example is specifically configured for ESP32-S3 boards.
  The S3 has USB CDC on GPIO19/20, so we use different pins for RS-485.

  Hardware Requirements:
  - ESP32-S3 board (DevKitC, etc.)
  - MAX485, MAX3485, or similar RS-485 transceiver module

  Recommended Wiring for ESP32-S3:

  ESP32-S3 Pin     RS-485 Transceiver
  ------------     ------------------
  GPIO 17 (TX) --> DI  (Driver Input)
  GPIO 18 (RX) <-- RO  (Receiver Output)
  GPIO 4       --> DE + /RE (tied together)

  Power:
  - VCC on transceiver -> 3.3V (for 3.3V transceivers) or 5V
  - GND -> GND
  - Connect 120 ohm termination resistor at bus ends

  RS-485 Bus:
  - A (non-inverting) connects to A on all devices
  - B (inverting) connects to B on all devices
  - Use twisted pair cable for A and B
  - Maximum bus length: ~1200m at 250kbps
*/

// Required: Tell DCS-BIOS this is an RS-485 Master
#define DCSBIOS_RS485_MASTER

// Required: TX Enable pin for half-duplex control
#define TXENABLE_PIN 4

// Configure UART pins for ESP32-S3
// Using GPIO 17 for TX and GPIO 18 for RX (avoids USB pins)
#define RS485_TX_PIN 17
#define RS485_RX_PIN 18

// Use UART1 for RS-485 (UART0 is for PC communication)
#define RS485_UART_NUM 1

#include "DcsBios.h"

void setup() {
    DcsBios::setup();
}

void loop() {
    DcsBios::loop();
}
