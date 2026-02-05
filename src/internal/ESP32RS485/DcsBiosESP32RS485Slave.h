#ifndef _DCSBIOS_ESP32_RS485_SLAVE_H_
#define _DCSBIOS_ESP32_RS485_SLAVE_H_

#ifdef DCSBIOS_RS485_SLAVE
#ifdef ARDUINO_ARCH_ESP32

// ============================================================================
// ESP32 RS485 SLAVE - Dual-Path Architecture
// ============================================================================
//
// Two operating modes selected at compile time:
//
//   USE_ISR_MODE 1 (default):
//     - UART RX interrupt fires immediately when byte arrives
//     - State machine runs IN the ISR - no polling latency
//     - Response sent immediately from ISR when poll detected
//     - Matches AVR's RXC interrupt behavior exactly
//     - Uses periph_module_enable (no driver install/delete) for C6 compatibility
//     - Adds RISC-V fence instruction for FIFO read stability on C3/C6/H2
//
//   USE_ISR_MODE 0 (fallback):
//     - Uses ESP-IDF UART driver for portable RX/TX
//     - State machine runs in main loop (tight polling)
//     - Slightly higher latency but works on any ESP32 variant
//     - Use this if ISR mode doesn't work on your specific chip
//
// Both modes work on: ESP32, S2, S3, C3, C6, H2
//
// The slave:
// 1. Listens for broadcast data (addr=0) -> passes to DCS-BIOS parser
// 2. Listens for poll requests (addr=our ID) -> responds with any pending data
// 3. Ignores messages for other slaves (but waits for their response)
//
// ============================================================================

#include "Arduino.h"
#include <stdint.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <rom/ets_sys.h>
#include <soc/soc_caps.h>
#include <esp_timer.h>

#include "../RingBuffer.h"

// ============================================================================
// OPERATING MODE SELECTION
// ============================================================================
// 1 = ISR-driven (lowest latency, like AVR)
// 0 = Driver-based (portable fallback)
// User can override by defining before #include <DcsBios.h>
#ifndef USE_ISR_MODE
#define USE_ISR_MODE 1
#endif

#if USE_ISR_MODE
// ISR mode needs direct hardware access
#include <hal/uart_ll.h>
#include <hal/gpio_ll.h>
#include <soc/uart_struct.h>
#include <soc/gpio_struct.h>
#include <soc/uart_periph.h>
#include <esp_intr_alloc.h>
#include <esp_rom_gpio.h>
// Peripheral module control - handle both old and new ESP-IDF locations
#if __has_include(<esp_private/periph_ctrl.h>)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <soc/periph_defs.h>
#endif // USE_ISR_MODE

// ============================================================================
// CONFIGURATION DEFAULTS (user can override before #include <DcsBios.h>)
// ============================================================================

#ifndef RS485_UART_NUM
#define RS485_UART_NUM 1
#endif

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 17
#endif

#ifndef RS485_RX_PIN
#define RS485_RX_PIN 18
#endif

// DE pin: set to -1 for auto-direction transceivers, or GPIO number for manual DE
#ifndef RS485_DE_PIN
#define RS485_DE_PIN -1
#endif

// Slave ID must be defined by user
#ifndef DCSBIOS_RS485_SLAVE
#error "DCSBIOS_RS485_SLAVE must be defined with slave address (1-127)"
#endif

#define RS485_BAUD_RATE 250000

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

#ifndef SYNC_TIMEOUT_US
#define SYNC_TIMEOUT_US 500             // 500us silence = sync detected
#endif

// TX Warm-up delays in MICROSECONDS (portable across all ESP32 variants)
// These give the transceiver time to switch to TX mode before data is sent
#ifndef TX_WARMUP_DELAY_MANUAL_US
#define TX_WARMUP_DELAY_MANUAL_US 50    // Manual DE: wait after DE asserted
#endif

#ifndef TX_WARMUP_DELAY_AUTO_US
#define TX_WARMUP_DELAY_AUTO_US 50      // Auto-direction: wait for RX->TX switch
#endif

#ifndef TX_COOLDOWN_DELAY_US
#define TX_COOLDOWN_DELAY_US 0          // Post-TX delay before DE deassert (0=disabled)
#endif

// ============================================================================
// TX MODE SELECTION (ISR mode only)
// ============================================================================
// 0 = Buffered mode: Build response in buffer, write all at once to FIFO
// 1 = Byte-by-byte mode: Write each byte individually, wait for TX idle
//     (optional AVR-style pacing; higher ISR occupancy than buffered mode)
#ifndef TX_MODE_BYTE_BY_BYTE
#define TX_MODE_BYTE_BY_BYTE 0
#endif

// ============================================================================
// BUFFER SIZES
// ============================================================================

#ifndef MESSAGE_BUFFER_SIZE
#define MESSAGE_BUFFER_SIZE 64
#endif

#ifndef EXPORT_BUFFER_SIZE
#define EXPORT_BUFFER_SIZE 256
#endif

// ============================================================================
// UART HARDWARE MAPPING (ISR mode)
// ============================================================================

#if USE_ISR_MODE
#if RS485_UART_NUM == 0
#define RS485_PERIPH_MODULE PERIPH_UART0_MODULE
#elif RS485_UART_NUM == 1
#define RS485_PERIPH_MODULE PERIPH_UART1_MODULE
#elif RS485_UART_NUM == 2
#define RS485_PERIPH_MODULE PERIPH_UART2_MODULE
#else
#error "Invalid RS485_UART_NUM (must be 0, 1, or 2)"
#endif
#endif // USE_ISR_MODE

// ============================================================================
// RS485 STATE MACHINE
// ============================================================================

namespace DcsBios {

    enum RS485State {
        STATE_SYNC,
        STATE_RX_WAIT_ADDRESS,
        STATE_RX_WAIT_MSGTYPE,
        STATE_RX_WAIT_DATALENGTH,
        STATE_RX_WAIT_DATA,
        STATE_RX_WAIT_CHECKSUM,
        STATE_RX_WAIT_ANSWER_DATALENGTH,
        STATE_RX_WAIT_ANSWER_MSGTYPE,
        STATE_RX_WAIT_ANSWER_DATA,
        STATE_RX_WAIT_ANSWER_CHECKSUM
    };

    enum RxDataType {
        RXDATA_IGNORE,
        RXDATA_DCSBIOS_EXPORT
    };

    // Forward declarations
    extern ProtocolParser parser;
    extern RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

    // Function to queue messages for sending
    bool tryToSendDcsBiosMessage(const char* msg, const char* arg);

    void setup();
    void loop();
    void resetAllStates();
}

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_SLAVE
#endif // _DCSBIOS_ESP32_RS485_SLAVE_H_
