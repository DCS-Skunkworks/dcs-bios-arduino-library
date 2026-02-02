/*
  =============================================================================
  ESP32-S3 RS-485 Master for DCS-BIOS
  =============================================================================

  This implementation leverages the ESP32-S3's dual-core architecture for
  superior performance compared to Arduino Mega:

  ARCHITECTURE:
  ┌─────────────────────────────────────────────────────────────────────────┐
  │  Core 0 (PRO_CPU)              │  Core 1 (APP_CPU)                      │
  │  ────────────────              │  ─────────────────                      │
  │  PC Serial Task                │  Arduino loop()                         │
  │  - High priority               │  - RS485 State Machine                  │
  │  - Reads USB Serial            │  - Broadcasts export data               │
  │  - Feeds lock-free FIFO        │  - Polls slaves                         │
  │  - Forwards slave responses    │  - Processes responses                  │
  └─────────────────────────────────────────────────────────────────────────┘

  This dual-core design ensures that PC serial data is NEVER missed,
  even during RS485 bus operations. The AVR Arduino Mega achieves similar
  behavior through hardware interrupts, but the ESP32-S3's dual-core
  FreeRTOS approach is cleaner and more scalable.

  ADVANTAGES OVER ARDUINO MEGA:
  - True parallel execution (not just ISR preemption)
  - 4KB lock-free FIFO buffer (vs 128 bytes on Mega)
  - 240MHz clock (vs 16MHz on Mega)
  - Hardware RS485 direction control
  - More GPIO and memory for future expansion

  SUPPORTED HARDWARE:
  - ESP32-S3 DevKitC
  - Waveshare ESP32-S3-RS485-CAN
  - Any ESP32-S3 board with RS-485 transceiver

  NOTE: This implementation REQUIRES ESP32-S3. Other ESP32 variants
  (original ESP32, S2, C3, etc.) are not supported due to different
  core architectures.

  WIRING for Waveshare ESP32-S3-RS485-CAN:

  ESP32-S3 Pin     RS-485 Transceiver
  ------------     ------------------
  GPIO 17 (TX) --> DI  (Driver Input)
  GPIO 18 (RX) <-- RO  (Receiver Output)
  GPIO 21      --> DE + /RE (Direction Enable)

  For custom boards, define pins before including DcsBios.h:

    #define RS485_TX_PIN 17
    #define RS485_RX_PIN 18
    #define TXENABLE_PIN 21

  For auto-direction RS-485 transceivers (hardware handles TX/RX switching):

    #define TXENABLE_PIN -1

  =============================================================================
*/

// Required: Tell DCS-BIOS this is an RS-485 Master
#define DCSBIOS_RS485_MASTER

// Disable servo (not needed for master, saves memory)
#define DCSBIOS_DISABLE_SERVO

// -----------------------------------------------------------------------------
// PIN CONFIGURATION for Waveshare ESP32-S3-RS485-CAN
// -----------------------------------------------------------------------------

// TX Enable pin for half-duplex direction control
// Set to -1 if your board has auto-direction hardware
#define TXENABLE_PIN 21

// UART pins for RS-485 bus
#define RS485_TX_PIN 17
#define RS485_RX_PIN 18

// Use UART1 for RS-485 (UART0 is for USB/PC communication)
#define RS485_UART_NUM 1

// -----------------------------------------------------------------------------

#include "DcsBios.h"

void setup() {
    DcsBios::setup();
}

void loop() {
    DcsBios::loop();
}
