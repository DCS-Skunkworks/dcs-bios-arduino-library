#ifndef _DCSBIOS_ESP32_RS485_MASTER_H_
#define _DCSBIOS_ESP32_RS485_MASTER_H_

#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

// ============================================================================
// ESP32 RS485 MASTER - Bare-Metal ISR Multi-Bus Implementation
// ============================================================================
//
// Protocol-perfect RS485 Master for DCS-BIOS using bare-metal UART with
// ISR-driven RX and inline blocking TX. Matches AVR DcsBiosNgRS485Master
// behavior exactly, and SURPASSES it in broadcast strategy and poll scanning.
//
// SUPPORTED ESP32 VARIANTS:
// - ESP32 (Classic)    - Dual-core, UART0 via USB-to-serial chip, 3 UARTs
// - ESP32-S2           - Single-core, Native USB CDC, 2 UARTs
// - ESP32-S3           - Dual-core, Native USB CDC, 3 UARTs
// - ESP32-C3           - Single-core RISC-V, USB Serial/JTAG, 2 UARTs
// - ESP32-C6           - Single-core RISC-V, USB Serial/JTAG, 2 UARTs
//
// ARCHITECTURE:
// - ISR-driven RX: Each byte triggers an ISR that processes the state machine
//   immediately, just like AVR's USART_RX_vect.
// - Inline blocking TX: No FreeRTOS tasks for TX. TX blocks loop() briefly.
// - Echo prevention: RX interrupt is disabled during TX and RX FIFO is flushed
//   after TX completes, mirroring AVR's set_txen()/clear_txen() behavior.
// - Bare-metal UART: Uses periph_module_enable + uart_ll_* for direct hardware
//   access, matching the proven slave implementation.
// - FreeRTOS PC input task: Dedicated task drains Serial into RingBuffer<1024>
//   even while main loop blocks during TX. Matches AVR's ISR-driven PC RX.
// - Inline PC forwarding: After each bus->loop(), slave responses are forwarded
//   to PC immediately - reduces latency by one full loop cycle.
// - Multi-bus: Up to 3 independent RS485 buses with linked-list iteration.
// - Chunked broadcasts: Max 64 bytes per burst, polls every 2ms minimum.
// - Safe poll scan: No infinite loop when zero slaves present.
// - Timeout zero byte: Sent on behalf of non-responding slaves (AVR-compatible).
//
// PROTOCOL (Exact match to AVR DcsBiosNgRS485Master):
//
//   BROADCAST (Master -> All Slaves):
//     [0x00] [0x00] [Length] [Data...] [Checksum]
//
//   POLL (Master -> Specific Slave):
//     [SlaveAddr] [0x00] [0x00]
//
//   SLAVE RESPONSE:
//     [DataLength] [MsgType] [Data...] [Checksum]  - if slave has data
//     [0x00]                                        - if slave has nothing
//
// ============================================================================

#include "Arduino.h"
#include <stdint.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <soc/soc_caps.h>

// Bare-metal UART access (same pattern as slave ISR mode)
#include <hal/uart_ll.h>
#include <hal/gpio_ll.h>
#include <soc/uart_struct.h>
#include <soc/gpio_struct.h>
#include <soc/uart_periph.h>
#include <esp_intr_alloc.h>

// Peripheral module control - handle both old and new ESP-IDF locations
#if __has_include(<esp_private/periph_ctrl.h>)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <soc/periph_defs.h>

// FreeRTOS for PC Serial input task
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../RingBuffer.h"

// ============================================================================
// CONFIGURATION DEFAULTS (user can override before #include <DcsBios.h>)
// ============================================================================

// --- BUS 1 (Primary, enabled by default) ---
#ifndef RS485_BUS1_TX_PIN
#define RS485_BUS1_TX_PIN 17
#endif

#ifndef RS485_BUS1_RX_PIN
#define RS485_BUS1_RX_PIN 18
#endif

#ifndef RS485_BUS1_DE_PIN
#define RS485_BUS1_DE_PIN -1        // -1 = auto-direction mode
#endif

#ifndef RS485_BUS1_UART_NUM
#define RS485_BUS1_UART_NUM 1
#endif

// --- BUS 2 (Disabled by default) ---
#ifndef RS485_BUS2_TX_PIN
#define RS485_BUS2_TX_PIN -1
#endif

#ifndef RS485_BUS2_RX_PIN
#define RS485_BUS2_RX_PIN -1
#endif

#ifndef RS485_BUS2_DE_PIN
#define RS485_BUS2_DE_PIN -1
#endif

#ifndef RS485_BUS2_UART_NUM
#define RS485_BUS2_UART_NUM -1
#endif

// --- BUS 3 (Disabled by default) ---
#ifndef RS485_BUS3_TX_PIN
#define RS485_BUS3_TX_PIN -1
#endif

#ifndef RS485_BUS3_RX_PIN
#define RS485_BUS3_RX_PIN -1
#endif

#ifndef RS485_BUS3_DE_PIN
#define RS485_BUS3_DE_PIN -1
#endif

#ifndef RS485_BUS3_UART_NUM
#define RS485_BUS3_UART_NUM -1
#endif

// --- Common Configuration ---
#ifndef RS485_BAUD_RATE
#define RS485_BAUD_RATE 250000
#endif

// TX Warmup Delays (microseconds)
#ifndef TX_WARMUP_DELAY_MANUAL_US
#define TX_WARMUP_DELAY_MANUAL_US 50    // DE pin stabilization time
#endif

#ifndef TX_WARMUP_DELAY_AUTO_US
#define TX_WARMUP_DELAY_AUTO_US 50      // Auto-direction transceiver settling
#endif

// Buffer Sizes
#ifndef EXPORT_BUFFER_SIZE
#define EXPORT_BUFFER_SIZE 256          // Buffer for PC -> Slaves data (per bus)
#endif

#ifndef MESSAGE_BUFFER_SIZE
#define MESSAGE_BUFFER_SIZE 64          // Buffer for Slave -> PC data (per bus)
#endif

#ifndef PC_RX_BUFFER_SIZE
#define PC_RX_BUFFER_SIZE 1024          // Buffer for PC Serial input task
#endif

// Timing Constants (microseconds) - matches AVR exactly
#ifndef POLL_TIMEOUT_US
#define POLL_TIMEOUT_US 1000            // 1ms timeout for slave response
#endif

#ifndef RX_TIMEOUT_US
#define RX_TIMEOUT_US 5000              // 5ms timeout for complete message
#endif

#ifndef MAX_POLL_INTERVAL_US
#define MAX_POLL_INTERVAL_US 2000       // Ensure polls at least every 2ms
#endif

// Broadcast chunking - prevents bus hogging during heavy export traffic
#ifndef MAX_BROADCAST_CHUNK
#define MAX_BROADCAST_CHUNK 64          // Max bytes per broadcast burst
#endif

// Slave address range to scan (1-127 valid, 0 is broadcast)
#ifndef MIN_SLAVE_ADDRESS
#define MIN_SLAVE_ADDRESS 1
#endif

#ifndef MAX_SLAVE_ADDRESS
#define MAX_SLAVE_ADDRESS 127
#endif

// Internal array size
#define MAX_SLAVES 128

// ============================================================================
// COMPILE-TIME BUS DETECTION
// ============================================================================

#define RS485_BUS1_ENABLED (RS485_BUS1_UART_NUM >= 0 && RS485_BUS1_TX_PIN >= 0 && RS485_BUS1_RX_PIN >= 0)
#define RS485_BUS2_ENABLED (RS485_BUS2_UART_NUM >= 0 && RS485_BUS2_TX_PIN >= 0 && RS485_BUS2_RX_PIN >= 0)
#define RS485_BUS3_ENABLED (RS485_BUS3_UART_NUM >= 0 && RS485_BUS3_TX_PIN >= 0 && RS485_BUS3_RX_PIN >= 0)

#define RS485_NUM_BUSES ((RS485_BUS1_ENABLED ? 1 : 0) + (RS485_BUS2_ENABLED ? 1 : 0) + (RS485_BUS3_ENABLED ? 1 : 0))

#if RS485_NUM_BUSES == 0
    #error "At least one RS485 bus must be enabled (set RS485_BUS1 pins)"
#endif

// PC Serial - works on ALL ESP32 variants
#ifndef PC_SERIAL
#define PC_SERIAL Serial
#endif

namespace DcsBios {

    // ========================================================================
    // STATE MACHINE
    // ========================================================================

    enum MasterState {
        MASTER_STATE_IDLE,
        MASTER_STATE_RX_WAIT_DATALENGTH,
        MASTER_STATE_RX_WAIT_MSGTYPE,
        MASTER_STATE_RX_WAIT_DATA,
        MASTER_STATE_RX_WAIT_CHECKSUM
    };

    // ========================================================================
    // RS485 MASTER CLASS - One instance per bus
    // ========================================================================

    // Forward declaration for ISR
    class RS485Master;
    static void IRAM_ATTR masterUartIsrHandler(void* arg);

    class RS485Master {
    private:
        // Hardware configuration
        int uartNum;
        int txPin, rxPin, dePin;

        // Bare-metal UART hardware pointer
        uart_dev_t* uartHw;
        intr_handle_t uartIntrHandle;

        // State machine (volatile - modified by ISR)
        volatile MasterState state;
        volatile int64_t rxStartTime;
        volatile uint8_t rxtxLen;
        volatile uint8_t rxMsgType;
        volatile int64_t lastPollTime;

        // Slave tracking
        volatile bool slavePresent[MAX_SLAVES];
        uint8_t pollAddressCounter;
        uint8_t scanAddressCounter;
        uint8_t currentPollAddress;

        // DE pin control
        inline void setDE(bool high);

        // TX helpers - inline blocking
        inline void prepareForTransmit();
        inline void finishTransmit();
        inline void txWriteBlocking(const uint8_t* data, uint16_t len);

    public:
        // Buffers (public for PC aggregation and ISR access)
        RingBuffer<EXPORT_BUFFER_SIZE> exportData;
        RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

        // Linked list for iteration
        RS485Master* next;
        static RS485Master* first;

        RS485Master(int uartNum, int txPin, int rxPin, int dePin);
        void init();
        void advancePollAddress();
        void sendPollFrame(uint8_t addr);
        void sendBroadcastFrame();
        void sendTimeoutZeroByte();
        void IRAM_ATTR rxISR();
        void loop();
    };

    // ========================================================================
    // DCSBIOS INTERFACE
    // ========================================================================

    void setup();
    void loop();
}

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
#endif // _DCSBIOS_ESP32_RS485_MASTER_H_
