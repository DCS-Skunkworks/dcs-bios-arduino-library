#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Master.h"

namespace DcsBios {

    // Global instances
    HardwareSerial RS485Serial(RS485_UART_NUM);
    RS485Master rs485Master(&RS485Serial, TXENABLE_PIN, RS485_RX_PIN, RS485_TX_PIN);
    MasterPCConnection pcConnection;
    RingBuffer<32>* pc_tx_buffer = NULL;

    // End-of-update callback (can be implemented by user if needed)
    void __attribute__((weak)) eouDetected() {
        // Default empty implementation
    }

    // ==================== RS485Master Implementation ====================

    RS485Master::RS485Master(HardwareSerial* serial, uint8_t txEnablePin, int8_t rxPin, int8_t txPin)
        : serial(serial), txEnablePin(txEnablePin) {

        // Build circular singly-linked list of all RS485Master instances
        if (firstRS485Master == NULL) {
            firstRS485Master = this;
            this->nextRS485Master = this;
            pcConnection.next_tx_rs485_master = this;
        } else {
            this->nextRS485Master = firstRS485Master->nextRS485Master;
            firstRS485Master->nextRS485Master = this;
        }

        state = IDLE;
        poll_address_counter = 1;
        scan_address_counter = 1;
        rxtx_len = 0;
        rx_msgtype = 0;
        checksum = 0;
        poll_address = 1;
        rx_start_time = 0;

        // Slave ID 0 will not be polled; used for broadcast
        slave_present[0] = true;
        for (uint8_t i = 1; i < 128; i++) {
            slave_present[i] = false;
        }
    }

    void RS485Master::begin() {
        // Configure TX enable pin
        pinMode(txEnablePin, OUTPUT);
        clearTxEnable();

        // Initialize UART for RS485 with specified pins
        serial->begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
        serial->setTimeout(1);

        // Clear any garbage in buffers
        while (serial->available()) {
            serial->read();
        }
    }

    void RS485Master::advancePollAddress() {
        poll_address_counter = (poll_address_counter + 1) % 128;

        // Advance until next present slave ID
        while (!slave_present[poll_address_counter]) {
            poll_address_counter = (poll_address_counter + 1) % 128;
        }

        if (poll_address_counter == 0) {
            // Instead of polling slave ID 0, scan for a new device
            scan_address_counter = (scan_address_counter + 1) % 128;

            // Advance to slave ID that is not present
            while (slave_present[scan_address_counter]) {
                scan_address_counter = (scan_address_counter + 1) % 128;
            }
            poll_address = scan_address_counter;
            return;
        }

        poll_address = poll_address_counter;
    }

    void RS485Master::loop() {
        // Process based on current state
        switch (state) {
            case IDLE:
                if (exportData.isNotEmpty()) {
                    // We have export data to send to all slaves (broadcast)
                    rxtx_len = exportData.getLength();
                    checksum = 0;
                    setTxEnable();
                    serial->write((uint8_t)0);  // Address 0 = broadcast
                    state = TX_ADDRESS_SENT;
                } else if (messageBuffer.isEmpty() && !messageBuffer.complete) {
                    // Message buffer is free, poll a device
                    advancePollAddress();
                    setTxEnable();
                    serial->write(poll_address);
                    state = POLL_ADDRESS_SENT;
                } else {
                    pcConnection.checkTimeout();
                }
                break;

            case POLL_ADDRESS_SENT:
                serial->write((uint8_t)0x00);  // Message type 0 = poll
                state = POLL_MSGTYPE_SENT;
                break;

            case POLL_MSGTYPE_SENT:
                serial->write((uint8_t)0);  // Data length 0 for poll
                state = POLL_DATALENGTH_SENT;
                break;

            case POLL_DATALENGTH_SENT:
                // Wait for TX to complete, then switch to RX
                serial->flush();
                clearTxEnable();
                rx_start_time = micros();
                state = RX_WAIT_DATALENGTH;
                break;

            case RX_WAIT_DATALENGTH:
                // Timeout for non-existing devices (1ms)
                if ((micros() - rx_start_time) > 1000) {
                    slave_present[poll_address] = false;
                    // Send zero data length on behalf of non-existing device
                    setTxEnable();
                    serial->write((uint8_t)0);
                    state = TIMEOUT_ZEROBYTE_SENT;
                } else if (serial->available()) {
                    rxtx_len = serial->read();
                    slave_present[poll_address] = true;
                    rx_start_time = micros();
                    if (rxtx_len > 0) {
                        state = RX_WAIT_MSGTYPE;
                    } else {
                        state = IDLE;
                    }
                }
                break;

            case TIMEOUT_ZEROBYTE_SENT:
                serial->flush();
                clearTxEnable();
                state = IDLE;
                break;

            case RX_WAIT_MSGTYPE:
                if ((micros() - rx_start_time) > 5000) {
                    // Timeout - abort and send newline to PC
                    messageBuffer.clear();
                    messageBuffer.put('\n');
                    messageBuffer.complete = true;
                    state = IDLE;
                } else if (serial->available()) {
                    rx_msgtype = serial->read();
                    rxtx_len--;  // msgtype counts towards length
                    rx_start_time = micros();
                    state = RX_WAIT_DATA;
                }
                break;

            case RX_WAIT_DATA:
                if ((micros() - rx_start_time) > 5000) {
                    messageBuffer.clear();
                    messageBuffer.put('\n');
                    messageBuffer.complete = true;
                    state = IDLE;
                } else {
                    while (serial->available() && rxtx_len > 0) {
                        uint8_t data = serial->read();
                        messageBuffer.put(data);
                        rxtx_len--;
                        rx_start_time = micros();
                    }
                    if (rxtx_len == 0) {
                        state = RX_WAIT_CHECKSUM;
                    }
                }
                break;

            case RX_WAIT_CHECKSUM:
                if ((micros() - rx_start_time) > 5000) {
                    messageBuffer.clear();
                    messageBuffer.put('\n');
                    messageBuffer.complete = true;
                    state = IDLE;
                } else if (serial->available()) {
                    serial->read();  // Read and discard checksum (TODO: verify)
                    messageBuffer.complete = true;
                    state = IDLE;
                }
                break;

            case TX_ADDRESS_SENT:
                serial->write((uint8_t)0);  // Message type 0
                state = TX_MSGTYPE_SENT;
                break;

            case TX_MSGTYPE_SENT:
                serial->write(rxtx_len);
                state = TX;
                break;

            case TX:
                if (rxtx_len == 0) {
                    serial->write(checksum);
                    state = TX_CHECKSUM_SENT;
                } else {
                    // Send as many bytes as possible
                    while (rxtx_len > 0 && exportData.isNotEmpty()) {
                        uint8_t data = exportData.get();
                        serial->write(data);
                        checksum ^= data;
                        rxtx_len--;
                    }
                }
                break;

            case TX_CHECKSUM_SENT:
                serial->flush();
                clearTxEnable();
                state = IDLE;
                break;

            default:
                state = IDLE;
                break;
        }
    }

    // ==================== MasterPCConnection Implementation ====================

    MasterPCConnection::MasterPCConnection() : tx_start_time(0), next_tx_rs485_master(nullptr) {
    }

    void MasterPCConnection::begin() {
        // Initialize PC connection (USB Serial)
        Serial.begin(RS485_BAUD_RATE);

        // Wait for serial connection with timeout for standalone operation
        unsigned long startTime = millis();
        while (!Serial && (millis() - startTime) < 3000) {
            delay(10);
        }
    }

    void MasterPCConnection::advanceTxBuffer() {
        if (next_tx_rs485_master == nullptr) return;

        RS485Master* start = next_tx_rs485_master;

        while (true) {
            next_tx_rs485_master = next_tx_rs485_master->nextRS485Master;

            if (next_tx_rs485_master->messageBuffer.isEmpty() &&
                next_tx_rs485_master->messageBuffer.complete) {
                next_tx_rs485_master->messageBuffer.complete = false;
            }

            if (next_tx_rs485_master->messageBuffer.isNotEmpty()) {
                pc_tx_buffer = &(next_tx_rs485_master->messageBuffer);
                tx_start_time = micros();
                return;
            }

            if (next_tx_rs485_master == start) break;
        }

        pc_tx_buffer = nullptr;
    }

    void MasterPCConnection::checkTimeout() {
        if (pc_tx_buffer && (micros() - tx_start_time > 5000)) {
            pc_tx_buffer->clear();
            pc_tx_buffer->put('\n');
            pc_tx_buffer->complete = true;
        }
    }

    void MasterPCConnection::rxLoop() {
        // Receive data from PC and forward to export buffer
        while (Serial.available()) {
            uint8_t c = Serial.read();
            rs485Master.exportData.put(c);
        }
    }

    void MasterPCConnection::txLoop() {
        // Transmit data from RS485 slaves to PC
        if (pc_tx_buffer == nullptr) {
            advanceTxBuffer();
        }

        if (pc_tx_buffer == nullptr) return;

        // Send all available data
        while (pc_tx_buffer->isNotEmpty()) {
            Serial.write(pc_tx_buffer->get());
        }

        if (pc_tx_buffer->isEmpty()) {
            if (pc_tx_buffer->complete) {
                pc_tx_buffer->complete = false;
                advanceTxBuffer();
            }
        }
    }

    void MasterPCConnection::loop() {
        rxLoop();
        txLoop();
    }

    // ==================== DcsBios Setup and Loop ====================

    void setup() {
        // Initialize PC connection first
        pcConnection.begin();

        // Initialize RS485 Master
        rs485Master.begin();
    }

    void loop() {
        // Process RS485 communication
        rs485Master.loop();

        // Process PC communication
        pcConnection.loop();
    }

}  // namespace DcsBios

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
