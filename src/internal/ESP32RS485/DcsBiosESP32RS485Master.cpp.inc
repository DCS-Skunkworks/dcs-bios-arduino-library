#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Master.h"

namespace DcsBios {

    // Global instances
    RS485Master rs485Master;
    MasterPCConnection pcConnection;

    // ==================== RS485Master Implementation ====================

    RS485Master::RS485Master()
        : uartNum((uart_port_t)RS485_UART_NUM),
          txEnablePin(TXENABLE_PIN),
          rxPin(RS485_RX_PIN),
          txPin(RS485_TX_PIN),
          state(RS485State::IDLE),
          currentPollAddr(1),
          pollAddrCounter(1),
          scanAddrCounter(1),
          isScanning(false),
          txExportLen(0),
          txExportIdx(0),
          txChecksum(0),
          rxLen(0),
          rxExpected(0),
          rxMsgType(0),
          opStartUs(0),
          exportHead(0),
          exportTail(0),
          exportCount(0),
          expectTimeoutAfterData(false),
          skipTimeoutsAfterBroadcast(0)
    {
        // Initialize slave present array
        // Index 0 is broadcast address, mark as "present" to avoid polling it
        slavePresent[0] = true;
        for (uint8_t i = 1; i <= RS485_MAX_SLAVES; i++) {
            slavePresent[i] = false;
        }
    }

    void RS485Master::begin() {
        // Configure UART
        uart_config_t uart_config = {
            .baud_rate = RS485_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };

        // Install UART driver
        esp_err_t err = uart_driver_install(uartNum, 256, 256, 0, NULL, 0);
        if (err != ESP_OK) {
            Serial.printf("[RS485] UART driver install failed: %d\n", err);
            return;
        }

        err = uart_param_config(uartNum, &uart_config);
        if (err != ESP_OK) {
            Serial.printf("[RS485] UART param config failed: %d\n", err);
            return;
        }

        // Set UART pins
        // txEnablePin can be -1 for auto-direction boards
        err = uart_set_pin(uartNum, txPin, rxPin, txEnablePin, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            Serial.printf("[RS485] UART set pin failed: %d\n", err);
            return;
        }

        // Set RS485 mode based on whether we have a direction control pin
        if (txEnablePin >= 0) {
            // Manual direction control - ESP32 drives EN pin for TX/RX switching
            err = uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX);
            if (err != ESP_OK) {
                Serial.printf("[RS485] RS485 half-duplex mode failed: %d\n", err);
                return;
            }
        } else {
            // Auto-direction board - hardware handles TX/RX switching
            err = uart_set_mode(uartNum, UART_MODE_UART);
            if (err != ESP_OK) {
                Serial.printf("[RS485] UART mode failed: %d\n", err);
                return;
            }
        }

        // Flush any garbage
        uart_flush_input(uartNum);

        Serial.printf("[RS485] Init OK: %d baud, TX=%d, RX=%d, EN=%d\n",
                      RS485_BAUD_RATE, txPin, rxPin, txEnablePin);
    }

    void RS485Master::feedExportData(uint8_t byte) {
        // Add to FIFO, discarding oldest if full
        if (exportCount >= RS485_EXPORT_FIFO_SIZE) {
            exportTail = (exportTail + 1) % RS485_EXPORT_FIFO_SIZE;
            exportCount--;
        }
        exportFifo[exportHead] = byte;
        exportHead = (exportHead + 1) % RS485_EXPORT_FIFO_SIZE;
        exportCount++;
    }

    bool RS485Master::hasMessageData() {
        return messageBuffer.isNotEmpty();
    }

    uint8_t RS485Master::getMessageByte() {
        return messageBuffer.get();
    }

    void RS485Master::advanceToNextSlave() {
        static uint8_t scanCounter = 0;
        isScanning = false;

        // Every ~50 polls, probe an unknown address to discover new slaves
        scanCounter++;
        if (scanCounter >= 50) {
            scanCounter = 0;

            // Find an unknown address to probe
            for (uint8_t i = 1; i <= RS485_MAX_SLAVES; i++) {
                scanAddrCounter++;
                if (scanAddrCounter > RS485_MAX_SLAVES) scanAddrCounter = 1;

                if (!slavePresent[scanAddrCounter]) {
                    currentPollAddr = scanAddrCounter;
                    isScanning = true;
                    state = RS485State::IDLE;
                    return;
                }
            }
            // All addresses already known, fall through to normal polling
        }

        // Normal polling - find next known-present slave
        for (uint8_t i = 0; i < RS485_MAX_SLAVES; i++) {
            currentPollAddr++;
            if (currentPollAddr > RS485_MAX_SLAVES) {
                currentPollAddr = 1;
            }

            if (slavePresent[currentPollAddr]) {
                break;
            }
        }

        state = RS485State::IDLE;
    }

    void RS485Master::prepareExportData() {
        txExportLen = 0;

        if (exportCount == 0) {
            return;
        }

        // Drain FIFO, cap at ~250 bytes per broadcast
        size_t bytesToSend = (exportCount > 250) ? 250 : exportCount;

        for (size_t i = 0; i < bytesToSend; i++) {
            txExportData[txExportLen++] = exportFifo[exportTail];
            exportTail = (exportTail + 1) % RS485_EXPORT_FIFO_SIZE;
            exportCount--;
        }
    }

    void RS485Master::startBroadcast() {
        // Allow slave to resync after potential scan timeout
        delayMicroseconds(500);

        prepareExportData();

        if (txExportLen == 0) {
            state = RS485State::IDLE;
            return;
        }

        opStartUs = micros();

        // Calculate checksum: XOR of address, msgtype, length, and all data
        txChecksum = RS485_ADDR_BROADCAST ^ RS485_MSGTYPE_POLL ^ (uint8_t)txExportLen;
        for (size_t i = 0; i < txExportLen; i++) {
            txChecksum ^= txExportData[i];
        }

        // Send broadcast address (0)
        uint8_t addr = RS485_ADDR_BROADCAST;
        uart_write_bytes(uartNum, (const char*)&addr, 1);
        state = RS485State::BROADCAST_MSGTYPE;

        skipTimeoutsAfterBroadcast = 10;
    }

    void RS485Master::processBroadcastTx() {
        switch (state) {
            case RS485State::BROADCAST_MSGTYPE: {
                uint8_t msgtype = RS485_MSGTYPE_POLL;
                uart_write_bytes(uartNum, (const char*)&msgtype, 1);
                state = RS485State::BROADCAST_LENGTH;
                break;
            }
            case RS485State::BROADCAST_LENGTH: {
                uint8_t len = (uint8_t)txExportLen;
                uart_write_bytes(uartNum, (const char*)&len, 1);
                txExportIdx = 0;
                state = RS485State::BROADCAST_DATA;
                break;
            }
            case RS485State::BROADCAST_DATA: {
                // Send data bytes
                size_t txFree = 0;
                uart_get_tx_buffer_free_size(uartNum, &txFree);
                while (txExportIdx < txExportLen && txFree > 1) {
                    uart_write_bytes(uartNum, (const char*)&txExportData[txExportIdx], 1);
                    txExportIdx++;
                    txFree--;
                }
                if (txExportIdx >= txExportLen) {
                    state = RS485State::BROADCAST_CHECKSUM;
                }
                break;
            }
            case RS485State::BROADCAST_CHECKSUM: {
                uart_write_bytes(uartNum, (const char*)&txChecksum, 1);
                state = RS485State::BROADCAST_WAIT_COMPLETE;
                break;
            }
            case RS485State::BROADCAST_WAIT_COMPLETE: {
                uart_wait_tx_done(uartNum, 1);
                uart_flush_input(uartNum);
                state = RS485State::IDLE;
                break;
            }
            default:
                break;
        }
    }

    void RS485Master::startPoll(uint8_t addr) {
        currentPollAddr = addr;
        opStartUs = micros();

        // Send poll: [address] [msgtype=0] [length=0]
        uart_write_bytes(uartNum, (const char*)&addr, 1);
        state = RS485State::POLL_MSGTYPE;
    }

    void RS485Master::processPollTx() {
        switch (state) {
            case RS485State::POLL_MSGTYPE: {
                uint8_t msgtype = RS485_MSGTYPE_POLL;
                uart_write_bytes(uartNum, (const char*)&msgtype, 1);
                state = RS485State::POLL_LENGTH;
                break;
            }
            case RS485State::POLL_LENGTH: {
                // Polls have NO data (length=0)
                uint8_t len = 0;
                uart_write_bytes(uartNum, (const char*)&len, 1);
                state = RS485State::POLL_WAIT_COMPLETE;
                break;
            }
            case RS485State::POLL_WAIT_COMPLETE: {
                uart_wait_tx_done(uartNum, 1);

                // Clear RX buffer and prepare for response
                rxLen = 0;
                rxExpected = 0;
                rxMsgType = 0;
                opStartUs = micros();
                state = RS485State::RX_WAIT_LENGTH;
                break;
            }
            default:
                break;
        }
    }

    void RS485Master::handleResponse() {
        uint8_t idx = currentPollAddr - 1;
        if (idx < RS485_MAX_SLAVES) {
            slavePresent[currentPollAddr] = true;
        }

        // Process input command if present
        if (rxLen > 0 && rxMsgType == 0) {
            processInputCommand(rxBuffer, rxLen);
            expectTimeoutAfterData = true;
        }
    }

    void RS485Master::handleTimeout() {
        // Send 0x00 on behalf of non-responding slave
        uint8_t zero = 0x00;
        uart_write_bytes(uartNum, (const char*)&zero, 1);
        uart_wait_tx_done(uartNum, 1);

        // Expected timeout after receiving data - don't count as error
        if (expectTimeoutAfterData) {
            expectTimeoutAfterData = false;
            return;
        } else if (skipTimeoutsAfterBroadcast > 0 && !isScanning) {
            skipTimeoutsAfterBroadcast--;
            return;
        }

        // Mark slave as not present if scanning
        if (isScanning) {
            slavePresent[currentPollAddr] = false;
        }
    }

    void RS485Master::processInputCommand(const uint8_t* data, size_t len) {
        // Forward command to PC via message buffer
        for (size_t i = 0; i < len; i++) {
            messageBuffer.put(data[i]);
        }
        // Ensure newline termination
        if (len > 0 && data[len-1] != '\n') {
            messageBuffer.put('\n');
        }
    }

    void RS485Master::processRx() {
        size_t available = 0;
        uart_get_buffered_data_len(uartNum, &available);

        if (available == 0) {
            uint32_t elapsed = micros() - opStartUs;
            if (elapsed > RS485_POLL_TIMEOUT_US) {
                handleTimeout();
                uart_flush_input(uartNum);
                advanceToNextSlave();
            }
            return;
        }

        uint8_t byte;
        int n;

        switch (state) {
            case RS485State::RX_WAIT_LENGTH: {
                n = uart_read_bytes(uartNum, &byte, 1, 0);
                if (n != 1) break;

                if (byte == 0x00) {
                    // Empty response - slave has no data
                    handleResponse();
                    uart_flush_input(uartNum);
                    advanceToNextSlave();
                    return;
                }

                rxExpected = byte;
                rxLen = 0;
                state = RS485State::RX_WAIT_MSGTYPE;
                break;
            }

            case RS485State::RX_WAIT_MSGTYPE: {
                n = uart_read_bytes(uartNum, &byte, 1, 0);
                if (n != 1) break;

                rxMsgType = byte;
                if (rxExpected > 0) {
                    state = RS485State::RX_WAIT_DATA;
                } else {
                    state = RS485State::RX_WAIT_CHECKSUM;
                }
                break;
            }

            case RS485State::RX_WAIT_DATA: {
                while (available > 0 && rxLen < rxExpected) {
                    n = uart_read_bytes(uartNum, &byte, 1, 0);
                    if (n != 1) break;

                    if (rxLen < RS485_INPUT_BUFFER_SIZE) {
                        rxBuffer[rxLen] = byte;
                    }
                    rxLen++;
                    available--;
                }

                if (rxLen >= rxExpected) {
                    state = RS485State::RX_WAIT_CHECKSUM;
                }
                break;
            }

            case RS485State::RX_WAIT_CHECKSUM: {
                n = uart_read_bytes(uartNum, &byte, 1, 0);
                if (n != 1) break;

                // Verify checksum (0x72 is the expected value from slaves)
                if (byte == RS485_CHECKSUM_PLACEHOLDER) {
                    handleResponse();
                }

                uart_flush_input(uartNum);
                advanceToNextSlave();
                break;
            }

            default:
                state = RS485State::IDLE;
                break;
        }

        // Check for RX timeout during data reception
        if (state >= RS485State::RX_WAIT_LENGTH && state <= RS485State::RX_WAIT_CHECKSUM) {
            uint32_t elapsed = micros() - opStartUs;
            if (elapsed > RS485_RX_TIMEOUT_US) {
                handleTimeout();
                uart_flush_input(uartNum);
                advanceToNextSlave();
            }
        }
    }

    void RS485Master::loop() {
        switch (state) {
            case RS485State::IDLE: {
                // If there's export data → BROADCAST first
                // Only poll when FIFO is empty
                if (exportCount > 0) {
                    startBroadcast();
                } else {
                    startPoll(currentPollAddr);
                }
                break;
            }

            case RS485State::BROADCAST_MSGTYPE:
            case RS485State::BROADCAST_LENGTH:
            case RS485State::BROADCAST_DATA:
            case RS485State::BROADCAST_CHECKSUM:
            case RS485State::BROADCAST_WAIT_COMPLETE:
                processBroadcastTx();
                break;

            case RS485State::POLL_MSGTYPE:
            case RS485State::POLL_LENGTH:
            case RS485State::POLL_WAIT_COMPLETE:
                processPollTx();
                break;

            case RS485State::RX_WAIT_LENGTH:
            case RS485State::RX_WAIT_MSGTYPE:
            case RS485State::RX_WAIT_DATA:
            case RS485State::RX_WAIT_CHECKSUM:
                processRx();
                break;

            default:
                state = RS485State::IDLE;
                break;
        }
    }

    // ==================== MasterPCConnection Implementation ====================

    MasterPCConnection::MasterPCConnection() : txStartTime(0) {
    }

    void MasterPCConnection::begin() {
        // Initialize PC connection (USB Serial)
        Serial.begin(RS485_BAUD_RATE);

        // Wait for serial connection with timeout
        unsigned long startTime = millis();
        while (!Serial && (millis() - startTime) < 3000) {
            delay(10);
        }
    }

    void MasterPCConnection::rxLoop() {
        // Receive data from PC and forward to RS485 export buffer
        while (Serial.available()) {
            uint8_t c = Serial.read();
            rs485Master.feedExportData(c);
        }
    }

    void MasterPCConnection::txLoop() {
        // Forward slave responses to PC
        while (rs485Master.hasMessageData()) {
            Serial.write(rs485Master.getMessageByte());
        }
    }

    void MasterPCConnection::loop() {
        rxLoop();
        txLoop();
    }

    // ==================== DcsBios Setup and Loop ====================

    void setup() {
        pcConnection.begin();
        rs485Master.begin();
    }

    void loop() {
        rs485Master.loop();
        pcConnection.loop();
    }

}  // namespace DcsBios

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
