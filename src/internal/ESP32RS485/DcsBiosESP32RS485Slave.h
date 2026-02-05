#ifndef _DCSBIOS_ESP32_RS485_SLAVE_H_
#define _DCSBIOS_ESP32_RS485_SLAVE_H_

#ifdef DCSBIOS_RS485_SLAVE
#ifdef ARDUINO_ARCH_ESP32

// ============================================================================
// ESP32-S3 RS485 SLAVE IMPLEMENTATION
// ============================================================================
//
// This is significantly simpler than the Master because:
// 1. NO USB CDC - RS485 only communication
// 2. REACTIVE - Only responds when polled by master
// 3. SHORT TX - Only sends brief response packets
// 4. NO DUAL-CORE needed - Single-threaded is fine
//
// The slave:
// 1. Listens for broadcast data (addr=0) → passes to DCS-BIOS parser
// 2. Listens for poll requests (addr=our ID) → responds with any pending data
// 3. Ignores messages for other slaves (but waits for their response)
//
// ============================================================================

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
    #error "DCS-BIOS RS485 Slave requires ESP32-S3."
#endif

#include "Arduino.h"
#include <stdint.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include "../RingBuffer.h"

// ============================================================================
// CONFIGURATION
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

#ifndef RS485_EN_PIN
#define RS485_EN_PIN 21
#endif

// Slave ID must be defined by user
#ifndef DCSBIOS_RS485_SLAVE
#error "DCSBIOS_RS485_SLAVE must be defined with slave address (1-127)"
#endif

#define RS485_BAUD_RATE 250000

// Timing constants (microseconds)
#define US_PER_BYTE 40              // At 250kbaud: 10 bits / 250000 = 40µs
#define TX_MARGIN_US 20             // Safety margin after calculated TX time
#define SYNC_TIMEOUT_US 500         // 500µs of silence to sync
#define RX_TIMEOUT_US 5000          // 5ms timeout

// Buffer sizes
#define RS485_UART_BUFFER_SIZE 512
#define MESSAGE_BUFFER_SIZE 32

namespace DcsBios {

    // Forward declaration
    extern ProtocolParser parser;

    // Message buffer for outgoing commands
    extern RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

    // ============================================================================
    // RS485 SLAVE CLASS
    // ============================================================================

    class RS485Slave {
    public:
        // State machine
        volatile uint8_t state;
        enum State {
            UNINITIALIZED,
            SYNC,               // Wait for 500µs of silence to synchronize
            RX_WAIT_ADDRESS,    // Waiting for slave address byte
            RX_WAIT_MSGTYPE,    // Waiting for message type byte
            RX_WAIT_DATALENGTH, // Waiting for data length byte
            RX_WAIT_DATA,       // Receiving data bytes
            RX_WAIT_CHECKSUM,   // Waiting for checksum byte
            RX_WAIT_ANSWER_DATALENGTH,  // Another slave is responding - wait for length
            RX_WAIT_ANSWER_MSGTYPE,     // Another slave's response msgtype
            RX_WAIT_ANSWER_DATA,        // Another slave's response data
            RX_WAIT_ANSWER_CHECKSUM,    // Another slave's response checksum
            TX_SENDING,         // Transmitting response
            TX_WAIT_COMPLETE    // Waiting for TX to complete
        };

    private:
        uart_port_t uartNum;
        gpio_num_t dePin;
        uint8_t slaveAddress;

        // RX state
        volatile uint8_t rx_slave_address;
        volatile uint8_t rx_msgtype;
        volatile uint8_t rxtx_len;
        volatile int64_t last_rx_time;

        // TX state
        volatile int64_t tx_complete_time;

        // Data type being received
        enum RxDataType {
            RXDATA_IGNORE,
            RXDATA_DCSBIOS_EXPORT
        };
        volatile uint8_t rx_datatype;

        void setDE(bool high);
        void sendResponse();
        int64_t calculateTxCompleteTime(size_t bytes);

    public:
        RS485Slave(uint8_t address);
        void begin();
        void loop();
    };

    extern RS485Slave rs485slave;

    // Function to queue messages for sending
    bool tryToSendDcsBiosMessage(const char* msg, const char* arg);

    void setup();
    void loop();
    void resetAllStates();
}

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_SLAVE
#endif // _DCSBIOS_ESP32_RS485_SLAVE_H_
