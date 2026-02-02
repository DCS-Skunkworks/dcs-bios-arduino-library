#ifndef _DCSBIOS_ESP32_RS485_MASTER_H_
#define _DCSBIOS_ESP32_RS485_MASTER_H_

#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "Arduino.h"
#include <stdint.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#include "../RingBuffer.h"

// Default pin configurations for ESP32
// These can be overridden by defining them before including DcsBios.h

// UART1 (RS485 bus) - default pins for ESP32
#ifndef RS485_UART_NUM
#define RS485_UART_NUM 1
#endif

#ifndef RS485_RX_PIN
#define RS485_RX_PIN 16
#endif

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 17
#endif

// TX Enable pin for RS485 transceiver
// Can be set to -1 for auto-direction boards
#ifndef TXENABLE_PIN
#error "TXENABLE_PIN must be defined for RS485 Master mode (use -1 for auto-direction boards)"
#endif

// Baud rate for RS485 bus (must match DCS-BIOS protocol)
#define RS485_BAUD_RATE 250000

// Protocol constants
#define RS485_ADDR_BROADCAST 0
#define RS485_MSGTYPE_POLL 0
#define RS485_CHECKSUM_PLACEHOLDER 0x72

// Timeout constants (microseconds)
#define RS485_POLL_TIMEOUT_US 1500
#define RS485_RX_TIMEOUT_US 5000

// Maximum slaves on bus
#define RS485_MAX_SLAVES 32

// Export buffer size (FIFO for raw data relay)
#define RS485_EXPORT_FIFO_SIZE 512

// Input buffer size for slave responses
#define RS485_INPUT_BUFFER_SIZE 64

namespace DcsBios {

    /**
     * RS485 State Machine States
     */
    enum class RS485State : uint8_t {
        IDLE,
        BROADCAST_MSGTYPE,
        BROADCAST_LENGTH,
        BROADCAST_DATA,
        BROADCAST_CHECKSUM,
        BROADCAST_WAIT_COMPLETE,
        POLL_MSGTYPE,
        POLL_LENGTH,
        POLL_WAIT_COMPLETE,
        RX_WAIT_LENGTH,
        RX_WAIT_MSGTYPE,
        RX_WAIT_DATA,
        RX_WAIT_CHECKSUM
    };

    /**
     * RS485Master - Handles communication with RS485 slave devices
     * Uses ESP-IDF UART driver for proper RS485 half-duplex support
     */
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
        volatile uint8_t pollAddrCounter;
        volatile uint8_t scanAddrCounter;
        volatile bool isScanning;
        volatile bool slavePresent[RS485_MAX_SLAVES + 1];

        // TX state
        uint8_t txExportData[512];
        size_t txExportLen;
        size_t txExportIdx;
        uint8_t txChecksum;

        // RX state
        uint8_t rxBuffer[RS485_INPUT_BUFFER_SIZE];
        size_t rxLen;
        size_t rxExpected;
        uint8_t rxMsgType;

        // Timing
        volatile uint32_t opStartUs;

        // Export data FIFO (raw bytes from PC)
        uint8_t exportFifo[RS485_EXPORT_FIFO_SIZE];
        volatile size_t exportHead;
        volatile size_t exportTail;
        volatile size_t exportCount;

        // Message buffer for slave responses (to forward to PC)
        DcsBios::RingBuffer<64> messageBuffer;

        // Expected timeout tracking
        volatile bool expectTimeoutAfterData;
        volatile uint8_t skipTimeoutsAfterBroadcast;

        // Internal methods
        void advanceToNextSlave();
        void prepareExportData();
        void startBroadcast();
        void processBroadcastTx();
        void startPoll(uint8_t addr);
        void processPollTx();
        void processRx();
        void handleResponse();
        void handleTimeout();
        void processInputCommand(const uint8_t* data, size_t len);

    public:
        RS485Master();
        void begin();
        void loop();
        void feedExportData(uint8_t byte);
        bool hasMessageData();
        uint8_t getMessageByte();
    };

    /**
     * MasterPCConnection - Handles communication with PC running DCS-BIOS
     */
    class MasterPCConnection {
    private:
        volatile uint32_t txStartTime;

    public:
        MasterPCConnection();
        void begin();
        void loop();
        void rxLoop();
        void txLoop();
    };

    // Global instances
    extern RS485Master rs485Master;
    extern MasterPCConnection pcConnection;

    void setup();
    void loop();
}

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER

#endif // _DCSBIOS_ESP32_RS485_MASTER_H_
