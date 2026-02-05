/*
  =============================================================================
  ESP32-S3 RS-485 Master for DCS-BIOS
  =============================================================================

  This implementation mirrors the AVR Arduino Mega RS485 Master exactly:
  - Same state machine
  - Same ring buffer sizes (128 bytes export, 32 bytes message)
  - Same timing (1ms poll timeout, 5ms RX timeout)
  - Same byte-by-byte transmission
  - Same protocol format

  REQUIRES: ESP32-S3 (compile error on other ESP32 variants)

  WIRING for Waveshare ESP32-S3-RS485-CAN:

  ESP32-S3 Pin     RS-485 Transceiver
  ------------     ------------------
  GPIO 17 (TX) --> DI  (Driver Input)
  GPIO 18 (RX) <-- RO  (Receiver Output)
  GPIO 21      --> DE + /RE (Direction Enable)

  =============================================================================
*/

#define DCSBIOS_RS485_MASTER
#define DCSBIOS_DISABLE_SERVO

// Pin configuration for Waveshare ESP32-S3-RS485-CAN
#define TXENABLE_PIN 21
#define RS485_TX_PIN 17
#define RS485_RX_PIN 18
#define RS485_UART_NUM 1

#include "DcsBios.h"

void setup() {
    DcsBios::setup();
}

void loop() {
    DcsBios::loop();
}
