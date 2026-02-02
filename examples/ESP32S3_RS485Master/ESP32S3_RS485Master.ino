/*
  ESP32-S3 RS-485 Master for DCS-BIOS

  This example is configured for boards like the Waveshare ESP32-S3-RS485-CAN:
  https://www.waveshare.com/esp32-s3-rs485-can.htm

  Hardware Requirements:
  - ESP32-S3 board with RS-485 transceiver
  - MAX485, MAX3485, or similar RS-485 transceiver

  Waveshare ESP32-S3-RS485-CAN Pinout:

  ESP32-S3 Pin     RS-485 Transceiver
  ------------     ------------------
  GPIO 17 (TX) --> DI  (Driver Input)
  GPIO 18 (RX) <-- RO  (Receiver Output)
  GPIO 21      --> DE + /RE (Direction Enable)

  Power:
  - VCC on transceiver -> 3.3V or 5V
  - GND -> GND
  - Connect 120 ohm termination resistor at bus ends

  RS-485 Bus:
  - A (non-inverting) connects to A on all devices
  - B (inverting) connects to B on all devices
  - Use twisted pair cable for A and B

  IMPORTANT: This implementation uses the ESP-IDF UART driver with
  UART_MODE_RS485_HALF_DUPLEX for proper direction control timing.
*/

// Required: Tell DCS-BIOS this is an RS-485 Master
#define DCSBIOS_RS485_MASTER
#define DCSBIOS_DISABLE_SERVO

// TX Enable pin for half-duplex direction control
// Set to -1 if your board has auto-direction hardware
#define TXENABLE_PIN 21

// Configure UART pins for Waveshare ESP32-S3-RS485-CAN
#define RS485_TX_PIN 17
#define RS485_RX_PIN 18

// Use UART1 for RS-485 (UART0 is for USB/PC communication)
#define RS485_UART_NUM 1

#include "DcsBios.h"

void setup() {
    DcsBios::setup();
}

void loop() {
    DcsBios::loop();
}
