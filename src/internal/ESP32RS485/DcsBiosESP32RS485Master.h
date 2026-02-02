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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

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

// Buffer sizes - larger to handle burst data
#define PC_SERIAL_RX_BUFFER_SIZE 1024   // ~40ms at 250kbaud
#define RS485_RX_BUFFER_SIZE 512
#define RS485_TX_BUFFER_SIZE 512

namespace DcsBios {

    // ============================================================================
    // RS485 MASTER - Uses UART event queue for non-blocking operation
    // ============================================================================

    class RS485Master {
    public:
        // Export data buffer - fed by PC serial, drained to slaves
        RingBuffer<256> exportData;  // Larger buffer

        // Message buffer for slave responses to PC
        RingBuffer<64> messageBuffer;  // Larger buffer

        // Slave tracking
        volatile bool slave_present[128];

        // State machine - includes TX_WAIT states for non-blocking operation
        volatile uint8_t state;
        enum State {
            IDLE,
            // Polling states
            POLL_ADDRESS_SENT,
            POLL_MSGTYPE_SENT,
            POLL_DATALENGTH_SENT,
            POLL_WAIT_TX_DONE,          // Wait for TX complete (non-blocking)
            // RX states
            RX_WAIT_DATALENGTH,
            RX_WAIT_MSGTYPE,
            RX_WAIT_DATA,
            RX_WAIT_CHECKSUM,
            // Timeout state
            TIMEOUT_ZEROBYTE_SENT,
            TIMEOUT_WAIT_TX_DONE,
            // Broadcast TX states
            TX_ADDRESS_SENT,
            TX_MSGTYPE_SENT,
            TX,
            TX_CHECKSUM_SENT,
            TX_WAIT_TX_DONE
        };

    private:
        uart_port_t uartNum;
        int txEnablePin;
        QueueHandle_t uartEventQueue;   // UART event queue for TX_DONE

        volatile uint8_t poll_address;
        volatile uint8_t poll_address_counter;
        volatile uint8_t scan_address_counter;
        volatile uint8_t rxtx_len;
        volatile uint8_t rx_msgtype;
        volatile uint8_t checksum;
        volatile uint32_t rx_start_time;
        volatile bool txDoneFlag;       // Set by event processing

        void advancePollAddress();
        void processUartEvents();       // Process UART event queue

    public:
        RS485Master();
        void begin();
        void loop();

        // Called to feed export data (from PC serial task)
        inline void feedExportByte(uint8_t c) {
            exportData.put(c);
        }
    };

    // ============================================================================
    // PC CONNECTION - Uses dedicated high-priority task for reading
    // ============================================================================

    class MasterPCConnection {
    private:
        volatile uint32_t tx_start_time;
        TaskHandle_t rxTaskHandle;      // Handle for RX task

        static void rxTaskFunc(void* param);  // FreeRTOS task function

    public:
        MasterPCConnection();
        void begin();

        // Send slave responses to PC
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
