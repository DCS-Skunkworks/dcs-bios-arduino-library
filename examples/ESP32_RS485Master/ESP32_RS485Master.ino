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
  GPIO 4       --> DE + /RE (Direction Enable, active high)

  The DE and /RE pins should be tied together for half-duplex operation.
  The ESP32's UART hardware controls the direction pin automatically
  when using UART_MODE_RS485_HALF_DUPLEX.

  For auto-direction RS-485 boards (hardware handles TX/RX switching):
    #define TXENABLE_PIN -1

  IMPORTANT: This implementation uses the ESP-IDF UART driver for proper
  RS-485 half-duplex timing, not Arduino's HardwareSerial.
*/

// Required: Tell DCS-BIOS this is an RS-485 Master
#define DCSBIOS_RS485_MASTER
#define DCSBIOS_DISABLE_SERVO

// Required: Define the TX Enable pin for the RS-485 transceiver
// This pin controls DE (Driver Enable) and /RE (Receiver Enable)
// Set to -1 for auto-direction boards
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
