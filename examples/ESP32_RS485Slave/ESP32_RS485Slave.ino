/*
  =============================================================================
  ESP32 RS-485 Slave for DCS-BIOS
  =============================================================================

  Works on ALL ESP32 variants: ESP32, S2, S3, C3, C6, H2

  Under the hood, the library uses ISR-driven bare-metal UART by default:
    - UART RX interrupt fires on every byte (1-byte FIFO threshold)
    - RS485 state machine runs inside the ISR for zero polling latency
    - Response sent immediately from ISR when polled by master
    - Matches AVR's USART RX interrupt behavior exactly
    - RISC-V fence instruction for FIFO stability on C3/C6/H2
    - Echo prevention: disables RX interrupt during TX, flushes FIFO after

  If ISR mode doesn't work on your chip, add this before #include:
    #define USE_ISR_MODE 0    // Falls back to ESP-IDF UART driver polling

  WIRING:
    ESP32 Pin        RS-485 Transceiver
    -----------      ------------------
    RS485_TX_PIN --> DI  (Driver Input)
    RS485_RX_PIN <-- RO  (Receiver Output)
    RS485_DE_PIN --> DE + /RE (only if using manual direction control)

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

// RS485 pin configuration (adjust for your board)
#define RS485_TX_PIN  17      // UART TX -> RS485 DI
#define RS485_RX_PIN  18      // UART RX <- RS485 RO
#define RS485_DE_PIN  -1      // DE pin, or -1 for auto-direction transceivers

// Optional: change UART number (default is 1)
// #define RS485_UART_NUM 1

// Optional: switch to driver-based fallback mode
// #define USE_ISR_MODE 0

#include "DcsBios.h"

/*
  =============================================================================
  DCS-BIOS CONTROLS - Add your cockpit controls here
  =============================================================================

  Use the same DcsBios:: classes as any other DCS-BIOS sketch.
  The library handles all RS485 communication automatically.

  Examples (F/A-18C Hornet):

  // 2-position switch on pin 4 for Master Arm
  DcsBios::Switch2Pos masterArm("MASTER_ARM_SW", 4);

  // LED on pin 13 driven by Master Caution light
  DcsBios::LED masterCaution(0x7408, 0x0200, 13);

  // Rotary encoder on pins 5,6 for heading bug
  DcsBios::RotaryEncoder hdgBug("UFC_COMM1_CHANNEL_SELECT", "-3200", "+3200", 5, 6);

  // Potentiometer on analog pin for brightness
  DcsBios::Potentiometer instPnlDimmer("INST_PNL_DIMMER", A0);

  See the DCS-BIOS control reference for your aircraft module.
*/

// =============================================================================
// TEST CONTROLS (matching slave.ino test setup)
// =============================================================================

// Master Arm switch on pin 39
DcsBios::Switch2Pos masterArmSw("MASTER_ARM_SW", 39);

// UFC Option Select 1 button on pin 0 (boot button on most ESP32 boards)
DcsBios::Switch2Pos ufcOpt1Btn("UFC_OS1", 0);

// Master Caution Ready LED on pin 40
DcsBios::LED mcReadyLed(0x740C, 0x8000, 40);

// =============================================================================
// ARDUINO SETUP AND LOOP
// =============================================================================

void setup() {
    DcsBios::setup();
}

void loop() {
    DcsBios::loop();
}
