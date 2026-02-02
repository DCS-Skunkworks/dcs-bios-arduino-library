#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Master.h"

namespace DcsBios {

    // ============================================================================
    // GLOBAL INSTANCES
    // ============================================================================

    RS485Master rs485Master;
    TaskHandle_t pcSerialTaskHandle = NULL;

    // ============================================================================
    // PC SERIAL TASK - Runs on Core 0
    // ============================================================================
    // This task runs in parallel with the main Arduino loop (Core 1).
    // It continuously reads from USB Serial and feeds the lock-free FIFO.
    // Priority is set high to ensure we never miss incoming bytes.

    void pcSerialTask(void* param) {
        // Increase Serial RX buffer for extra safety
        Serial.setRxBufferSize(1024);
        Serial.begin(RS485_BAUD_RATE);

        // Wait for Serial to initialize
        while (!Serial && millis() < 3000) {
            vTaskDelay(1);
        }

        // Main loop - runs forever on Core 0
        for (;;) {
            // Read all available bytes from Serial
            while (Serial.available()) {
                uint8_t c = Serial.read();
                // Feed into lock-free FIFO (consumed by RS485 master on Core 1)
                rs485Master.exportFifo.put(c);
            }

            // Forward any slave responses to PC
            while (rs485Master.hasMessageData()) {
                Serial.write(rs485Master.getMessageByte());
            }

            // Small yield to prevent watchdog issues, but keep responsive
            // At 250kbaud, bytes arrive every 40us, so we check frequently
            vTaskDelay(1);  // 1 tick = 1ms on default FreeRTOS config
        }
    }

    // ============================================================================
    // RS485 MASTER IMPLEMENTATION
    // ============================================================================

    RS485Master::RS485Master()
        : uartNum((uart_port_t)RS485_UART_NUM),
          txEnablePin(TXENABLE_PIN),
          rxPin(RS485_RX_PIN),
          txPin(RS485_TX_PIN),
          state(RS485State::IDLE),
          currentPollAddr(1),
          scanAddrCounter(1),
          txLen(0),
          txChecksum(0),
          rxLen(0),
          rxExpected(0),
          rxMsgType(0),
          opStartUs(0),
          expectTimeoutAfterData(false),
          skipTimeoutsAfterBroadcast(0),
          broadcastCount(0),
          pollCount(0),
          responseCount(0),
          messageOutHead(0),
          messageOutTail(0)
    {
        // Initialize slave tracking
        slavePresent[0] = true;  // Broadcast address, always "present"
        for (uint8_t i = 1; i <= RS485_MAX_SLAVES; i++) {
            slavePresent[i] = false;
        }
    }

    void RS485Master::begin() {
        // Configure UART for RS485
        uart_config_t uart_config = {
            .baud_rate = RS485_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };

        // Install UART driver with larger buffers
        ESP_ERROR_CHECK(uart_driver_install(uartNum, 512, 512, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(uartNum, &uart_config));

        // Set pins - txEnablePin can be -1 for auto-direction boards
        ESP_ERROR_CHECK(uart_set_pin(uartNum, txPin, rxPin, txEnablePin, UART_PIN_NO_CHANGE));

        // Set RS485 mode
        if (txEnablePin >= 0) {
            ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX));
        } else {
            ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_UART));
        }

        uart_flush_input(uartNum);
    }

    void RS485Master::advanceToNextSlave() {
        static uint8_t scanCounter = 0;

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
                    return;
                }
            }
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
    }

    void RS485Master::startBroadcast() {
        // Allow slaves to resync
        delayMicroseconds(500);

        // Drain FIFO into TX buffer (max 250 bytes per broadcast)
        txLen = exportFifo.drain(txBuffer + 3, 250);

        if (txLen == 0) {
            state = RS485State::IDLE;
            return;
        }

        // Build packet header: [Addr=0] [MsgType=0] [Length]
        txBuffer[0] = RS485_ADDR_BROADCAST;
        txBuffer[1] = RS485_MSGTYPE_POLL;
        txBuffer[2] = (uint8_t)txLen;

        // Calculate checksum
        txChecksum = txBuffer[0] ^ txBuffer[1] ^ txBuffer[2];
        for (size_t i = 0; i < txLen; i++) {
            txChecksum ^= txBuffer[3 + i];
        }

        // Append checksum
        txBuffer[3 + txLen] = txChecksum;

        // Send entire packet at once (more efficient than byte-by-byte)
        uart_write_bytes(uartNum, (const char*)txBuffer, 4 + txLen);

        state = RS485State::BROADCAST_SENDING;
        broadcastCount++;
        skipTimeoutsAfterBroadcast = 10;
    }

    void RS485Master::startPoll(uint8_t addr) {
        currentPollAddr = addr;
        opStartUs = micros();

        // Build poll packet: [Addr] [MsgType=0] [Length=0]
        uint8_t pollPacket[3] = { addr, RS485_MSGTYPE_POLL, 0 };
        uart_write_bytes(uartNum, (const char*)pollPacket, 3);

        state = RS485State::POLL_SENDING;
        pollCount++;
    }

    void RS485Master::handleResponse() {
        if (currentPollAddr > 0 && currentPollAddr <= RS485_MAX_SLAVES) {
            slavePresent[currentPollAddr] = true;
        }
        responseCount++;

        // Forward command to PC if present
        if (rxLen > 0 && rxMsgType == 0) {
            for (size_t i = 0; i < rxLen; i++) {
                putMessageByte(rxBuffer[i]);
            }
            // Ensure newline termination
            if (rxLen > 0 && rxBuffer[rxLen - 1] != '\n') {
                putMessageByte('\n');
            }
            expectTimeoutAfterData = true;
        }
    }

    void RS485Master::handleTimeout() {
        // Send 0x00 on behalf of non-responding slave
        uint8_t zero = 0x00;
        uart_write_bytes(uartNum, (const char*)&zero, 1);
        uart_wait_tx_done(uartNum, 1);

        if (expectTimeoutAfterData) {
            expectTimeoutAfterData = false;
            return;
        }

        if (skipTimeoutsAfterBroadcast > 0) {
            skipTimeoutsAfterBroadcast--;
            return;
        }

        // Mark as not present (will be scanned again later)
        if (currentPollAddr > 0 && currentPollAddr <= RS485_MAX_SLAVES) {
            slavePresent[currentPollAddr] = false;
        }
    }

    void RS485Master::processRx() {
        size_t available = 0;
        uart_get_buffered_data_len(uartNum, &available);

        if (available == 0) {
            // Check timeout
            uint32_t elapsed = micros() - opStartUs;
            if (elapsed > RS485_POLL_TIMEOUT_US) {
                handleTimeout();
                uart_flush_input(uartNum);
                advanceToNextSlave();
                state = RS485State::IDLE;
            }
            return;
        }

        uint8_t byte;
        int n;

        switch (state) {
            case RS485State::RX_WAIT_LENGTH:
                n = uart_read_bytes(uartNum, &byte, 1, 0);
                if (n == 1) {
                    if (byte == 0x00) {
                        // Empty response
                        handleResponse();
                        uart_flush_input(uartNum);
                        advanceToNextSlave();
                        state = RS485State::IDLE;
                    } else {
                        rxExpected = byte;
                        rxLen = 0;
                        state = RS485State::RX_WAIT_MSGTYPE;
                    }
                    opStartUs = micros();
                }
                break;

            case RS485State::RX_WAIT_MSGTYPE:
                n = uart_read_bytes(uartNum, &byte, 1, 0);
                if (n == 1) {
                    rxMsgType = byte;
                    state = (rxExpected > 0) ? RS485State::RX_WAIT_DATA : RS485State::RX_WAIT_CHECKSUM;
                    opStartUs = micros();
                }
                break;

            case RS485State::RX_WAIT_DATA:
                while (available > 0 && rxLen < rxExpected) {
                    n = uart_read_bytes(uartNum, &byte, 1, 0);
                    if (n != 1) break;
                    if (rxLen < sizeof(rxBuffer)) {
                        rxBuffer[rxLen] = byte;
                    }
                    rxLen++;
                    available--;
                    opStartUs = micros();
                }
                if (rxLen >= rxExpected) {
                    state = RS485State::RX_WAIT_CHECKSUM;
                }
                break;

            case RS485State::RX_WAIT_CHECKSUM:
                n = uart_read_bytes(uartNum, &byte, 1, 0);
                if (n == 1) {
                    if (byte == RS485_CHECKSUM) {
                        handleResponse();
                    }
                    uart_flush_input(uartNum);
                    advanceToNextSlave();
                    state = RS485State::IDLE;
                }
                break;

            default:
                break;
        }

        // RX timeout check
        if (state >= RS485State::RX_WAIT_LENGTH && state <= RS485State::RX_WAIT_CHECKSUM) {
            if ((micros() - opStartUs) > RS485_RX_TIMEOUT_US) {
                handleTimeout();
                uart_flush_input(uartNum);
                advanceToNextSlave();
                state = RS485State::IDLE;
            }
        }
    }

    void RS485Master::loop() {
        switch (state) {
            case RS485State::IDLE:
                // Priority: broadcast export data first, then poll slaves
                if (!exportFifo.isEmpty()) {
                    startBroadcast();
                } else {
                    startPoll(currentPollAddr);
                }
                break;

            case RS485State::BROADCAST_SENDING:
                // Wait for TX complete
                if (uart_wait_tx_done(uartNum, 0) == ESP_OK) {
                    uart_flush_input(uartNum);
                    state = RS485State::IDLE;
                }
                break;

            case RS485State::POLL_SENDING:
                // Wait for TX complete, then switch to RX
                if (uart_wait_tx_done(uartNum, 0) == ESP_OK) {
                    rxLen = 0;
                    rxExpected = 0;
                    rxMsgType = 0;
                    opStartUs = micros();
                    state = RS485State::RX_WAIT_LENGTH;
                }
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

    // ============================================================================
    // DCSBIOS SETUP AND LOOP
    // ============================================================================

    void setup() {
        // Initialize RS485 Master (runs on Core 1 with Arduino loop)
        rs485Master.begin();

        // Create PC Serial task on Core 0 with high priority
        // This ensures continuous serial reception in parallel with RS485 ops
        xTaskCreatePinnedToCore(
            pcSerialTask,           // Task function
            "PCSerial",             // Name
            4096,                   // Stack size (bytes)
            NULL,                   // Parameters
            configMAX_PRIORITIES - 1, // High priority
            &pcSerialTaskHandle,    // Task handle
            0                       // Core 0 (PRO_CPU)
        );

        // Small delay to let the task start
        delay(100);
    }

    void loop() {
        // RS485 state machine runs on Core 1 (APP_CPU) with Arduino loop
        rs485Master.loop();
    }

}  // namespace DcsBios

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
