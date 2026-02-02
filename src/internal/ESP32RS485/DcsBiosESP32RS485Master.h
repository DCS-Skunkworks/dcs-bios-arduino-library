#ifndef _DCSBIOS_ESP32_RS485_MASTER_H_
#define _DCSBIOS_ESP32_RS485_MASTER_H_

#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

// ============================================================================
// ESP32-S3 RS485 MASTER - DEFINITIVE IMPLEMENTATION
// ============================================================================
//
// This implementation addresses ALL identified root causes:
//
// 1. MANUAL DE CONTROL - Not using UART_MODE_RS485_HALF_DUPLEX
//    The auto mode's DE timing is not cycle-perfect like AVR's TX ISR.
//    We control DE via GPIO with calculated timing.
//
// 2. THREAD-SAFE IPC - Using FreeRTOS StreamBuffer
//    The library's RingBuffer is not thread-safe for dual-core.
//    StreamBuffer is lock-free for single-reader/single-writer.
//
// 3. USB BURST HANDLING - Large buffers + dedicated task
//    USB Full-Speed delivers data in 1ms bursts (~25 bytes).
//    Dedicated task on Core 0 drains USB buffer continuously.
//
// 4. PACKET-BASED TX - Write entire packet at once
//    Prevents gaps between bytes that could cause early DE drop.
//
// 5. CALCULATED TX TIMING - Deterministic DE drop
//    At 250kbaud, each byte = 40µs. We calculate exact TX time.
//
// ============================================================================

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
    #error "DCS-BIOS RS485 Master requires ESP32-S3."
#endif

#include "Arduino.h"
#include <stdint.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/stream_buffer.h>
#include <esp_timer.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#ifndef RS485_UART_NUM
#define RS485_UART_NUM 1
#endif

#ifndef RS485_RX_PIN
#define RS485_RX_PIN 18
#endif

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 17
#endif

#ifndef TXENABLE_PIN
#error "TXENABLE_PIN must be defined"
#endif

#define RS485_BAUD_RATE 250000

// Timing constants (microseconds)
#define US_PER_BYTE 40                  // At 250kbaud: 10 bits / 250000 = 40µs
#define TX_MARGIN_US 20                 // Safety margin after calculated TX time
#define POLL_TIMEOUT_US 1000            // 1ms timeout for slave response
#define RX_TIMEOUT_US 5000              // 5ms timeout for complete message

// Buffer sizes
#define PC_STREAM_BUFFER_SIZE 1024      // USB CDC to RS485 (handles 1ms bursts)
#define RS485_UART_RX_BUFFER 512        // UART RX buffer
#define RS485_UART_TX_BUFFER 512        // UART TX buffer
#define EXPORT_BUFFER_SIZE 256          // Export data accumulator
#define MESSAGE_BUFFER_SIZE 64          // Slave response buffer

namespace DcsBios {

    // ============================================================================
    // SIMPLE RING BUFFER - For single-core use only (within RS485 task)
    // ============================================================================

    template<unsigned int SIZE>
    class SimpleRingBuffer {
    private:
        volatile uint8_t buffer[SIZE];
        volatile uint8_t head;
        volatile uint8_t tail;
    public:
        volatile bool complete;

        SimpleRingBuffer() : head(0), tail(0), complete(false) {}

        inline void put(uint8_t c) {
            buffer[head] = c;
            head = (head + 1) % SIZE;
        }

        inline uint8_t get() {
            uint8_t c = buffer[tail];
            tail = (tail + 1) % SIZE;
            return c;
        }

        inline bool isEmpty() const { return head == tail; }
        inline bool isNotEmpty() const { return head != tail; }
        inline uint8_t getLength() const { return (head - tail) % SIZE; }
        inline void clear() { head = tail = 0; complete = false; }
    };

    // ============================================================================
    // RS485 MASTER CLASS
    // ============================================================================

    class RS485Master {
    public:
        // Export data buffer (fed from StreamBuffer, drained to RS485)
        SimpleRingBuffer<EXPORT_BUFFER_SIZE> exportData;

        // Message buffer for slave responses
        SimpleRingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

        // Slave tracking
        volatile bool slave_present[128];

        // State machine
        volatile uint8_t state;
        enum State {
            IDLE,
            // TX states (Master → Slaves)
            TX_SENDING,
            TX_WAIT_COMPLETE,
            // Poll states
            POLL_SENDING,
            POLL_WAIT_COMPLETE,
            // RX states (Slaves → Master)
            RX_WAIT_DATALENGTH,
            RX_WAIT_MSGTYPE,
            RX_WAIT_DATA,
            RX_WAIT_CHECKSUM
        };

    private:
        uart_port_t uartNum;
        gpio_num_t dePin;

        // Polling
        volatile uint8_t poll_address;
        volatile uint8_t poll_address_counter;
        volatile uint8_t scan_address_counter;

        // TX/RX state
        volatile uint8_t rxtx_len;
        volatile uint8_t rx_msgtype;
        volatile uint8_t checksum;
        volatile int64_t tx_complete_time;   // Calculated time when TX is complete
        volatile int64_t rx_start_time;

        void advancePollAddress();
        void setDE(bool high);
        void sendPacket(const uint8_t* data, size_t len);
        int64_t calculateTxCompleteTime(size_t bytes);

    public:
        RS485Master();
        void begin();
        void loop();

        // Feed export data from StreamBuffer
        void feedExportByte(uint8_t c);
    };

    // ============================================================================
    // PC CONNECTION CLASS
    // ============================================================================

    class MasterPCConnection {
    private:
        StreamBufferHandle_t pcToRS485Stream;
        TaskHandle_t rxTaskHandle;
        volatile uint32_t tx_start_time;

        static void rxTaskFunc(void* param);

    public:
        MasterPCConnection();
        void begin();

        // Drain StreamBuffer into RS485 export buffer
        void processStreamBuffer();

        // Send slave responses to PC
        void txProcess();

        void checkTimeout();

        // Get stream buffer handle for task
        StreamBufferHandle_t getStreamBuffer() { return pcToRS485Stream; }
    };

    extern RS485Master rs485master;
    extern MasterPCConnection pcConnection;

    void setup();
    void loop();
}

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
#endif // _DCSBIOS_ESP32_RS485_MASTER_H_
