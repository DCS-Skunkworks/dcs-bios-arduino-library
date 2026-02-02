#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Master.h"

namespace DcsBios {

    // Global instances
    RS485Master rs485master;
    MasterPCConnection pcConnection;

    // ========================================================================
    // RS485 MASTER IMPLEMENTATION - DEFINITIVE VERSION
    //
    // Key changes from all previous attempts:
    // 1. NO UART_MODE_RS485_HALF_DUPLEX - Manual DE control
    // 2. PACKET-BASED TX - Entire packet written at once
    // 3. CALCULATED TX TIME - Deterministic DE drop timing
    // 4. STREAMBUFFER - Thread-safe IPC with USB task
    // ========================================================================

    RS485Master::RS485Master()
        : uartNum((uart_port_t)RS485_UART_NUM),
          dePin((gpio_num_t)TXENABLE_PIN),
          state(IDLE),
          poll_address(1),
          poll_address_counter(1),
          scan_address_counter(1),
          rxtx_len(0),
          rx_msgtype(0),
          checksum(0),
          tx_complete_time(0),
          rx_start_time(0)
    {
        slave_present[0] = true;  // Broadcast address, mark as "present" to skip
        for (uint8_t i = 1; i < 128; i++) {
            slave_present[i] = false;
        }
    }

    void RS485Master::begin() {
        // =====================================================================
        // GPIO: Manual DE pin control
        // This is CRITICAL - we don't trust the UART driver's auto control
        // =====================================================================
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dePin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        setDE(false);  // Start in RX mode

        // =====================================================================
        // UART: Normal mode (NOT RS485 auto mode!)
        // We handle DE manually for precise timing
        // =====================================================================
        uart_config_t uart_config = {
            .baud_rate = RS485_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };

        // Install driver WITHOUT RS485 mode - we control DE manually
        uart_driver_install(uartNum, RS485_UART_RX_BUFFER, RS485_UART_TX_BUFFER, 0, NULL, 0);
        uart_param_config(uartNum, &uart_config);
        uart_set_pin(uartNum, RS485_TX_PIN, RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        // IMPORTANT: Use normal UART mode, NOT RS485 mode
        // This gives us full control over DE timing
        uart_set_mode(uartNum, UART_MODE_UART);

        uart_flush_input(uartNum);
    }

    void RS485Master::setDE(bool high) {
        // Set Driver Enable pin
        // HIGH = Transmit mode (we're talking)
        // LOW = Receive mode (we're listening)
        gpio_set_level(dePin, high ? 1 : 0);
    }

    int64_t RS485Master::calculateTxCompleteTime(size_t bytes) {
        // Calculate when TX will be complete
        // At 250kbaud, each byte = 40µs (10 bits per byte)
        // Add margin for safety
        int64_t now = esp_timer_get_time();  // Microseconds
        int64_t tx_duration = (bytes * US_PER_BYTE) + TX_MARGIN_US;
        return now + tx_duration;
    }

    void RS485Master::sendPacket(const uint8_t* data, size_t len) {
        // =====================================================================
        // PACKET-BASED TX: Write entire packet at once
        // This prevents gaps between bytes that could confuse timing
        // =====================================================================

        // 1. Raise DE (enter TX mode)
        setDE(true);

        // 2. Small delay to ensure DE is stable before TX starts
        //    At 250kbaud, 1 bit = 4µs, so 10µs is ~2.5 bits - plenty
        delayMicroseconds(10);

        // 3. Write all bytes to UART FIFO at once
        uart_write_bytes(uartNum, (const char*)data, len);

        // 4. Calculate when TX will be complete
        tx_complete_time = calculateTxCompleteTime(len);
    }

    void RS485Master::advancePollAddress() {
        poll_address_counter = (poll_address_counter + 1) % 128;

        // Skip to next present slave
        while (!slave_present[poll_address_counter]) {
            poll_address_counter = (poll_address_counter + 1) % 128;
        }

        if (poll_address_counter == 0) {
            // Instead of polling broadcast address, scan for new device
            scan_address_counter = (scan_address_counter + 1) % 128;
            while (slave_present[scan_address_counter]) {
                scan_address_counter = (scan_address_counter + 1) % 128;
            }
            poll_address = scan_address_counter;
            return;
        }

        poll_address = poll_address_counter;
    }

    void RS485Master::feedExportByte(uint8_t c) {
        exportData.put(c);
    }

    void RS485Master::loop() {
        int64_t now = esp_timer_get_time();

        // =====================================================================
        // STEP 1: Check if TX is complete (for TX_WAIT states)
        // This is the CRITICAL timing - we drop DE exactly when TX is done
        // =====================================================================
        if (state == TX_WAIT_COMPLETE || state == POLL_WAIT_COMPLETE) {
            if (now >= tx_complete_time) {
                // TX is complete - drop DE to enter RX mode
                setDE(false);

                if (state == TX_WAIT_COMPLETE) {
                    // Broadcast complete, return to IDLE
                    state = IDLE;
                } else {
                    // Poll complete, wait for slave response
                    uart_flush_input(uartNum);
                    rx_start_time = now;
                    state = RX_WAIT_DATALENGTH;
                }
            }
            // Don't process anything else while waiting for TX complete
            return;
        }

        // =====================================================================
        // STEP 2: Process incoming data from slaves (RX states)
        // =====================================================================
        if (state >= RX_WAIT_DATALENGTH && state <= RX_WAIT_CHECKSUM) {
            size_t available = 0;
            uart_get_buffered_data_len(uartNum, &available);

            while (available > 0) {
                uint8_t c;
                if (uart_read_bytes(uartNum, &c, 1, 0) != 1) break;
                available--;

                switch (state) {
                    case RX_WAIT_DATALENGTH:
                        rxtx_len = c;
                        slave_present[poll_address] = true;
                        if (rxtx_len > 0) {
                            state = RX_WAIT_MSGTYPE;
                        } else {
                            state = IDLE;
                        }
                        rx_start_time = now;
                        break;

                    case RX_WAIT_MSGTYPE:
                        rx_msgtype = c;
                        state = RX_WAIT_DATA;
                        rx_start_time = now;
                        break;

                    case RX_WAIT_DATA:
                        messageBuffer.put(c);
                        rxtx_len--;
                        if (rxtx_len == 0) {
                            state = RX_WAIT_CHECKSUM;
                        }
                        rx_start_time = now;
                        break;

                    case RX_WAIT_CHECKSUM:
                        // Checksum received (ignore validation like AVR)
                        messageBuffer.complete = true;
                        state = IDLE;
                        break;
                }
            }

            // Timeout handling for RX states
            if (state == RX_WAIT_DATALENGTH && (now - rx_start_time) > POLL_TIMEOUT_US) {
                // No response from slave - mark as not present
                slave_present[poll_address] = false;
                state = IDLE;
            }

            if ((state == RX_WAIT_MSGTYPE || state == RX_WAIT_DATA || state == RX_WAIT_CHECKSUM)
                && (now - rx_start_time) > RX_TIMEOUT_US) {
                // Timeout during message reception
                messageBuffer.clear();
                messageBuffer.put('\n');
                messageBuffer.complete = true;
                state = IDLE;
            }

            return;
        }

        // =====================================================================
        // STEP 3: IDLE state - start new operations
        // =====================================================================
        if (state == IDLE) {
            // Priority 1: Send export data to slaves (broadcast)
            if (exportData.isNotEmpty()) {
                // Build broadcast packet: [Addr=0] [MsgType=0] [Length] [Data...] [Checksum]
                uint8_t packet[EXPORT_BUFFER_SIZE + 4];
                uint8_t len = exportData.getLength();
                uint8_t cs = 0;

                packet[0] = 0;      // Broadcast address
                packet[1] = 0;      // Message type
                packet[2] = len;    // Data length

                for (uint8_t i = 0; i < len; i++) {
                    uint8_t b = exportData.get();
                    packet[3 + i] = b;
                    cs ^= b;
                }
                packet[3 + len] = cs;  // Checksum

                // Send entire packet at once
                sendPacket(packet, 4 + len);
                state = TX_WAIT_COMPLETE;
                return;
            }

            // Priority 2: Poll a slave for input commands
            if (messageBuffer.isEmpty() && !messageBuffer.complete) {
                advancePollAddress();

                // Build poll packet: [SlaveAddr] [MsgType=0] [Length=0]
                uint8_t packet[3];
                packet[0] = poll_address;
                packet[1] = 0;  // Message type
                packet[2] = 0;  // Data length = 0 (poll request)

                sendPacket(packet, 3);
                state = POLL_WAIT_COMPLETE;
                return;
            }

            // Nothing to do - check for timeout on incomplete messages
            pcConnection.checkTimeout();
        }
    }

    // ========================================================================
    // PC CONNECTION IMPLEMENTATION
    //
    // Uses FreeRTOS StreamBuffer for thread-safe IPC between:
    // - Core 0: USB CDC reader task (high priority)
    // - Core 1: RS485 state machine (main loop)
    //
    // StreamBuffer is lock-free for single-reader/single-writer scenarios.
    // ========================================================================

    MasterPCConnection::MasterPCConnection()
        : pcToRS485Stream(NULL), rxTaskHandle(NULL), tx_start_time(0) {}

    // Static task function - runs on Core 0
    void MasterPCConnection::rxTaskFunc(void* param) {
        MasterPCConnection* self = (MasterPCConnection*)param;
        StreamBufferHandle_t stream = self->pcToRS485Stream;

        // This task runs forever, draining USB CDC buffer into StreamBuffer
        // It runs on Core 0 (where USB stack runs) with high priority
        for (;;) {
            // Read all available bytes from USB CDC
            while (Serial.available()) {
                uint8_t c = Serial.read();
                // Send to StreamBuffer (non-blocking, drop if full)
                xStreamBufferSend(stream, &c, 1, 0);
            }

            // Yield for 1 tick (~1ms) - matches USB Full-Speed frame rate
            vTaskDelay(1);
        }
    }

    void MasterPCConnection::begin() {
        // Initialize Serial with large RX buffer to handle USB bursts
        Serial.setRxBufferSize(PC_STREAM_BUFFER_SIZE);
        Serial.begin(RS485_BAUD_RATE);

        while (!Serial && millis() < 3000) {
            delay(1);
        }

        // Create StreamBuffer for thread-safe IPC
        // Trigger level = 1 means receiver unblocks when any byte is available
        pcToRS485Stream = xStreamBufferCreate(PC_STREAM_BUFFER_SIZE, 1);

        if (pcToRS485Stream == NULL) {
            // Failed to create StreamBuffer - fatal error
            while (1) {
                delay(1000);
            }
        }

        // Create USB reader task on Core 0 (where USB stack runs)
        // Priority 10 is high - ensures USB buffer is drained promptly
        xTaskCreatePinnedToCore(
            rxTaskFunc,         // Task function
            "USBCDCReader",     // Task name
            4096,               // Stack size
            this,               // Parameter (this pointer)
            10,                 // Priority (high)
            &rxTaskHandle,      // Task handle
            0                   // Core 0 (USB stack runs here)
        );
    }

    void MasterPCConnection::processStreamBuffer() {
        // Drain StreamBuffer into RS485 export buffer
        // This is called from the main loop on Core 1
        uint8_t c;
        while (xStreamBufferReceive(pcToRS485Stream, &c, 1, 0) > 0) {
            rs485master.feedExportByte(c);
        }
    }

    void MasterPCConnection::txProcess() {
        // Send slave responses to PC
        if (rs485master.messageBuffer.isEmpty()) {
            if (rs485master.messageBuffer.complete) {
                rs485master.messageBuffer.complete = false;
            }
            return;
        }

        // Send all available bytes to PC
        while (rs485master.messageBuffer.isNotEmpty()) {
            Serial.write(rs485master.messageBuffer.get());
        }
        tx_start_time = micros();
    }

    void MasterPCConnection::checkTimeout() {
        if (rs485master.messageBuffer.isNotEmpty() && (micros() - tx_start_time) > RX_TIMEOUT_US) {
            rs485master.messageBuffer.clear();
            rs485master.messageBuffer.put('\n');
            rs485master.messageBuffer.complete = true;
        }
    }

    // ========================================================================
    // DCSBIOS SETUP AND LOOP
    // ========================================================================

    void setup() {
        // Initialize PC connection (creates USB reader task on Core 0)
        pcConnection.begin();

        // Initialize RS485 master (manual DE control, normal UART mode)
        rs485master.begin();
    }

    void loop() {
        // Main loop runs on Core 1

        // 1. Drain USB data from StreamBuffer into export buffer
        pcConnection.processStreamBuffer();

        // 2. Run RS485 state machine
        rs485master.loop();

        // 3. Send any slave responses to PC
        pcConnection.txProcess();
    }

}  // namespace DcsBios

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
