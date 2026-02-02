#ifndef _DCSBIOS_ESP32_RS485_MASTER_H_
#define _DCSBIOS_ESP32_RS485_MASTER_H_

#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

// ============================================================================
// ESP32-S3 REQUIREMENT CHECK
// ============================================================================
// This implementation requires ESP32-S3 for optimal dual-core performance.
// The S3's dual-core Xtensa LX7 architecture allows true parallel execution
// of PC serial reception and RS485 bus operations.

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
    #error "DCS-BIOS RS485 Master requires ESP32-S3. Other ESP32 variants are not supported."
#endif

#include "Arduino.h"
#include <stdint.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// UART for RS485 bus (UART1 recommended, UART0 is for PC/USB)
#ifndef RS485_UART_NUM
#define RS485_UART_NUM 1
#endif

// Default pins for ESP32-S3 (can be overridden before including DcsBios.h)
#ifndef RS485_RX_PIN
#define RS485_RX_PIN 18
#endif

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 17
#endif

// TX Enable pin for RS485 direction control (-1 for auto-direction boards)
#ifndef TXENABLE_PIN
#error "TXENABLE_PIN must be defined for RS485 Master mode (use -1 for auto-direction boards)"
#endif

// Baud rate (DCS-BIOS standard)
#define RS485_BAUD_RATE 250000

// Protocol constants
#define RS485_ADDR_BROADCAST    0
#define RS485_MSGTYPE_POLL      0
#define RS485_CHECKSUM          0x72

// Timing constants (microseconds)
#define RS485_POLL_TIMEOUT_US   1500
#define RS485_RX_TIMEOUT_US     5000

// Maximum slaves on bus
#define RS485_MAX_SLAVES        32

// ============================================================================
// LOCK-FREE SPSC RING BUFFER
// ============================================================================
// Single-Producer Single-Consumer lock-free ring buffer for inter-core
// communication. Core 0 produces (PC Serial), Core 1 consumes (RS485 TX).
// No mutexes needed - just atomic reads/writes of head/tail indices.

#define EXPORT_FIFO_SIZE 4096  // 4KB buffer - handles ~160ms of data at 250kbaud

namespace DcsBios {

    class alignas(64) LockFreeFIFO {
    private:
        uint8_t buffer[EXPORT_FIFO_SIZE];
        volatile size_t head;  // Written by producer (Core 0)
        volatile size_t tail;  // Written by consumer (Core 1)

    public:
        LockFreeFIFO() : head(0), tail(0) {}

        // Producer: Add byte to buffer (called from Core 0)
        inline bool put(uint8_t c) {
            size_t next = (head + 1) % EXPORT_FIFO_SIZE;
            if (next == tail) return false;  // Buffer full
            buffer[head] = c;
            head = next;  // Atomic on ESP32 (32-bit aligned)
            return true;
        }

        // Consumer: Get byte from buffer (called from Core 1)
        inline bool get(uint8_t* c) {
            if (tail == head) return false;  // Buffer empty
            *c = buffer[tail];
            tail = (tail + 1) % EXPORT_FIFO_SIZE;
            return true;
        }

        // Consumer: Check available bytes
        inline size_t available() const {
            size_t h = head;  // Read once
            size_t t = tail;
            return (h >= t) ? (h - t) : (EXPORT_FIFO_SIZE - t + h);
        }

        // Consumer: Check if empty
        inline bool isEmpty() const {
            return head == tail;
        }

        // Consumer: Peek without removing
        inline bool peek(uint8_t* c) const {
            if (tail == head) return false;
            *c = buffer[tail];
            return true;
        }

        // Drain up to 'max' bytes into destination buffer, return count
        inline size_t drain(uint8_t* dest, size_t max) {
            size_t count = 0;
            while (count < max && tail != head) {
                dest[count++] = buffer[tail];
                tail = (tail + 1) % EXPORT_FIFO_SIZE;
            }
            return count;
        }
    };

    // ============================================================================
    // RS485 STATE MACHINE
    // ============================================================================

    enum class RS485State : uint8_t {
        IDLE,
        BROADCAST_SENDING,
        BROADCAST_WAIT_COMPLETE,
        POLL_SENDING,
        POLL_WAIT_COMPLETE,
        RX_WAIT_LENGTH,
        RX_WAIT_MSGTYPE,
        RX_WAIT_DATA,
        RX_WAIT_CHECKSUM
    };

    // ============================================================================
    // RS485 MASTER CLASS
    // ============================================================================

    class RS485Master {
    private:
        uart_port_t uartNum;
        int txEnablePin;
        int rxPin;
        int txPin;

        // State machine
        volatile RS485State state;

        // Polling state
        volatile uint8_t currentPollAddr;
        volatile uint8_t scanAddrCounter;
        volatile bool slavePresent[RS485_MAX_SLAVES + 1];

        // TX state
        uint8_t txBuffer[300];
        size_t txLen;
        uint8_t txChecksum;

        // RX state
        uint8_t rxBuffer[64];
        size_t rxLen;
        size_t rxExpected;
        uint8_t rxMsgType;

        // Timing
        volatile uint32_t opStartUs;

        // Timeout tracking
        volatile bool expectTimeoutAfterData;
        volatile uint8_t skipTimeoutsAfterBroadcast;

        // Statistics
        volatile uint32_t broadcastCount;
        volatile uint32_t pollCount;
        volatile uint32_t responseCount;

        // Internal methods
        void advanceToNextSlave();
        void startBroadcast();
        void startPoll(uint8_t addr);
        void processRx();
        void handleResponse();
        void handleTimeout();

    public:
        // Export data FIFO (fed by Core 0, consumed by Core 1)
        LockFreeFIFO exportFifo;

        // Message buffer for slave responses (small, simple buffer is fine)
        uint8_t messageOutBuffer[128];
        volatile size_t messageOutHead;
        volatile size_t messageOutTail;

        RS485Master();
        void begin();
        void loop();

        // Called by txLoop to get data to send to PC
        bool hasMessageData() const { return messageOutTail != messageOutHead; }
        uint8_t getMessageByte() {
            uint8_t c = messageOutBuffer[messageOutTail];
            messageOutTail = (messageOutTail + 1) % sizeof(messageOutBuffer);
            return c;
        }
        void putMessageByte(uint8_t c) {
            messageOutBuffer[messageOutHead] = c;
            messageOutHead = (messageOutHead + 1) % sizeof(messageOutBuffer);
        }
    };

    // ============================================================================
    // PC SERIAL TASK (Runs on Core 0)
    // ============================================================================
    // This task runs continuously on Core 0, reading from USB Serial and
    // feeding the lock-free FIFO. It never blocks on RS485 operations.

    void pcSerialTask(void* param);

    // ============================================================================
    // GLOBAL INSTANCES
    // ============================================================================

    extern RS485Master rs485Master;
    extern TaskHandle_t pcSerialTaskHandle;

    void setup();
    void loop();
}

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER

#endif // _DCSBIOS_ESP32_RS485_MASTER_H_
