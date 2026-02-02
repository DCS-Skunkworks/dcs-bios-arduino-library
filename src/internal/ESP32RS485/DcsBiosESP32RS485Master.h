#ifndef _DCSBIOS_ESP32_RS485_MASTER_H_
#define _DCSBIOS_ESP32_RS485_MASTER_H_

#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

// ============================================================================
// ESP32-S3 REQUIREMENT
// ============================================================================
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
    #error "DCS-BIOS RS485 Master requires ESP32-S3."
#endif

#include "Arduino.h"
#include <stdint.h>
#include <driver/uart.h>

#include "../RingBuffer.h"  // Use library's existing RingBuffer

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
#define RS485_MAX_SLAVES 32

namespace DcsBios {

    // ============================================================================
    // RS485 MASTER - Mimics AVR implementation exactly
    // ============================================================================

    class RS485Master {
    public:
        // Export data buffer - fed by PC serial, drained to slaves
        // 128 bytes like AVR
        RingBuffer<128> exportData;

        // Message buffer for slave responses to PC
        RingBuffer<32> messageBuffer;

        // Slave tracking
        volatile bool slave_present[128];

        // State machine
        volatile uint8_t state;
        enum State {
            IDLE,
            POLL_ADDRESS_SENT,
            POLL_MSGTYPE_SENT,
            POLL_DATALENGTH_SENT,
            TIMEOUT_ZEROBYTE_SENT,
            RX_WAIT_DATALENGTH,
            RX_WAIT_MSGTYPE,
            RX_WAIT_DATA,
            RX_WAIT_CHECKSUM,
            TX_ADDRESS_SENT,
            TX_MSGTYPE_SENT,
            TX,
            TX_CHECKSUM_SENT
        };

    private:
        uart_port_t uartNum;
        int txEnablePin;

        volatile uint8_t poll_address;
        volatile uint8_t poll_address_counter;
        volatile uint8_t scan_address_counter;
        volatile uint8_t rxtx_len;
        volatile uint8_t rx_msgtype;
        volatile uint8_t checksum;
        volatile uint32_t rx_start_time;

        void advancePollAddress();

    public:
        RS485Master();
        void begin();
        void loop();

        // Called from main loop to feed export data (mimics AVR rxISR)
        inline void feedExportByte(uint8_t c) {
            exportData.put(c);
        }
    };

    // ============================================================================
    // PC CONNECTION - Mimics AVR MasterPCConnection
    // ============================================================================

    class MasterPCConnection {
    private:
        volatile uint32_t tx_start_time;

    public:
        MasterPCConnection();
        void begin();

        // Read from PC Serial, feed to RS485 master (mimics AVR rxISR behavior)
        void rxProcess();

        // Send slave responses to PC (mimics AVR udreISR behavior)
        void txProcess();

        void checkTimeout();
    };

    extern RS485Master rs485master;
    extern MasterPCConnection pcConnection;

    void setup();
    void loop();
}

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
#endif // _DCSBIOS_ESP32_RS485_MASTER_H_
