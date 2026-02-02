#ifndef _DCSBIOS_ESP32_RS485_MASTER_H_
#define _DCSBIOS_ESP32_RS485_MASTER_H_

#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "Arduino.h"
#include <stdint.h>
#include <HardwareSerial.h>

#include "../RingBuffer.h"

// Default pin configurations for ESP32
// These can be overridden by defining them before including DcsBios.h

// UART1 (RS485 bus) - default pins for ESP32
#ifndef RS485_UART_NUM
#define RS485_UART_NUM 1
#endif

#ifndef RS485_RX_PIN
#define RS485_RX_PIN 16  // Default RX pin for UART1
#endif

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 17  // Default TX pin for UART1
#endif

// TX Enable pin for RS485 transceiver (directly from sketch define)
#ifndef TXENABLE_PIN
#error "TXENABLE_PIN must be defined for RS485 Master mode"
#endif

// PC connection uses default USB Serial (UART0)
// Baud rate for RS485 bus (must match DCS-BIOS protocol)
#define RS485_BAUD_RATE 250000

// Protocol state definitions (must match Protocol.h)
#ifndef DCSBIOS_STATE_WAIT_FOR_SYNC
#define DCSBIOS_STATE_WAIT_FOR_SYNC 0
#define DCSBIOS_STATE_ADDRESS_LOW 1
#define DCSBIOS_STATE_ADDRESS_HIGH 2
#define DCSBIOS_STATE_COUNT_LOW 3
#define DCSBIOS_STATE_COUNT_HIGH 4
#define DCSBIOS_STATE_DATA_LOW 5
#define DCSBIOS_STATE_DATA_HIGH 6
#endif

namespace DcsBios {

    // Forward declaration
    extern void eouDetected();

    /**
     * EndOfUpdateDetector - Detects end-of-update markers in the export stream
     * Same implementation as AVR version for protocol compatibility
     */
    class EndOfUpdateDetector {
        uint8_t state;
        uint16_t address;
        uint16_t count;
        uint16_t data;
        uint8_t sync_byte_count;

    public:
        EndOfUpdateDetector() : state(0), address(0), count(0), data(0), sync_byte_count(0) {}

        void processChar(uint8_t c) {
            switch(state) {
                case DCSBIOS_STATE_WAIT_FOR_SYNC:
                    break;

                case DCSBIOS_STATE_ADDRESS_LOW:
                    address = (unsigned int)c;
                    state = DCSBIOS_STATE_ADDRESS_HIGH;
                    break;

                case DCSBIOS_STATE_ADDRESS_HIGH:
                    address = (c << 8) | address;
                    if (address != 0x5555) {
                        state = DCSBIOS_STATE_COUNT_LOW;
                    } else {
                        state = DCSBIOS_STATE_WAIT_FOR_SYNC;
                    }
                    break;

                case DCSBIOS_STATE_COUNT_LOW:
                    count = (unsigned int)c;
                    state = DCSBIOS_STATE_COUNT_HIGH;
                    break;

                case DCSBIOS_STATE_COUNT_HIGH:
                    count = (c << 8) | count;
                    state = DCSBIOS_STATE_DATA_LOW;
                    break;

                case DCSBIOS_STATE_DATA_LOW:
                    data = (unsigned int)c;
                    count--;
                    state = DCSBIOS_STATE_DATA_HIGH;
                    break;

                case DCSBIOS_STATE_DATA_HIGH:
                    data = (c << 8) | data;
                    count--;
                    if (address == 0xfffe) eouDetected();
                    address += 2;
                    if (count == 0)
                        state = DCSBIOS_STATE_ADDRESS_LOW;
                    else
                        state = DCSBIOS_STATE_DATA_LOW;
                    break;
            }

            if (c == 0x55)
                sync_byte_count++;
            else
                sync_byte_count = 0;

            if (sync_byte_count == 4) {
                state = DCSBIOS_STATE_ADDRESS_LOW;
                sync_byte_count = 0;
            }
        }
    };

    /**
     * RS485Master - Handles communication with RS485 slave devices
     *
     * This is a single-bus implementation for ESP32. For multiple buses,
     * instantiate multiple RS485Master objects with different UART numbers.
     */
    class RS485Master {
    private:
        HardwareSerial* serial;
        uint8_t txEnablePin;

        volatile uint8_t poll_address;
        volatile uint8_t scan_address_counter;
        volatile uint8_t poll_address_counter;
        volatile unsigned long rx_start_time;

        volatile uint8_t rxtx_len;
        volatile uint8_t rx_msgtype;
        volatile uint8_t checksum;

        void advancePollAddress();

        inline void setTxEnable() {
            digitalWrite(txEnablePin, HIGH);
        }

        inline void clearTxEnable() {
            digitalWrite(txEnablePin, LOW);
        }

        inline void txByte(uint8_t c) {
            setTxEnable();
            serial->write(c);
        }

        inline void flushTx() {
            serial->flush();  // Wait for TX to complete
        }

    public:
        static RS485Master* firstRS485Master;
        RS485Master* nextRS485Master;

        DcsBios::RingBuffer<128> exportData;
        DcsBios::RingBuffer<32> messageBuffer;
        volatile bool slave_present[128];
        volatile uint8_t state;

        enum RS485State {
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

        RS485Master(HardwareSerial* serial, uint8_t txEnablePin, int8_t rxPin = -1, int8_t txPin = -1);
        void begin();
        void loop();
    };

    // Static member initialization
    RS485Master* RS485Master::firstRS485Master = NULL;

    /**
     * MasterPCConnection - Handles communication with the PC running DCS-BIOS
     * Uses the default Serial (USB) connection
     */
    class MasterPCConnection {
    private:
        volatile unsigned long tx_start_time;
        void advanceTxBuffer();

    public:
        RS485Master* next_tx_rs485_master;

        MasterPCConnection();
        void begin();
        void loop();
        void rxLoop();
        void txLoop();
        void checkTimeout();
    };

    void setup();
    void loop();
}

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER

#endif // _DCSBIOS_ESP32_RS485_MASTER_H_
