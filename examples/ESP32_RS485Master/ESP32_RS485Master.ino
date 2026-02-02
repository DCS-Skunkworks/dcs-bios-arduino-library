/*
  ESP32 RS-485 Master for DCS-BIOS

  This sketch turns an ESP32 (including S2 and S3 variants) into a
  DCS-BIOS RS-485 bus master. It bridges between the PC (via USB Serial)
  and RS-485 slave devices on the bus.

  Hardware Requirements:
  - ESP32, ESP32-S2, or ESP32-S3 board
  - MAX485, MAX3485, or similar RS-485 transceiver module

  Default Wiring (can be changed with defines below):

  ESP32 Pin        RS-485 Transceiver
  ---------        ------------------
  GPIO 17 (TX) --> DI  (Driver Input)
  GPIO 16 (RX) <-- RO  (Receiver Output)
  GPIO 4       --> DE + /RE (Driver Enable, active low Receiver Enable)

  The DE and /RE pins should be tied together for half-duplex operation.

  Optional: To use different pins, define them before including DcsBios.h:

    #define RS485_TX_PIN 17
    #define RS485_RX_PIN 16
    #define TXENABLE_PIN 4

  Note: This implementation uses a single RS-485 bus (UART1).
  Unlike the Arduino Mega version which supports 3 buses, the ESP32
  implementation focuses on simplicity and reliability with a single bus.

  For most cockpit builds, a single RS-485 bus with properly addressed
  slaves is sufficient and simplifies wiring.
*/

// Required: Tell DCS-BIOS this is an RS-485 Master
#define DCSBIOS_RS485_MASTER

// Required: Define the TX Enable pin for the RS-485 transceiver
// This pin controls DE (Driver Enable) and /RE (Receiver Enable)
#define TXENABLE_PIN 4

// Optional: Define custom UART pins (defaults shown)
// #define RS485_TX_PIN 17
// #define RS485_RX_PIN 16

// Optional: Use a different UART number (default is UART1)
// UART0 is used for USB Serial communication with PC
// #define RS485_UART_NUM 1

#include "DcsBios.h"

void setup() {
    DcsBios::setup();
}

void loop() {
    DcsBios::loop();
}
