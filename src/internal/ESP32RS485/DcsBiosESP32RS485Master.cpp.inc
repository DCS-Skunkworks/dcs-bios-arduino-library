#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Master.h"

namespace DcsBios {

    // Global instances
    RS485Master rs485master;
    MasterPCConnection pcConnection;

    // ========================================================================
    // RS485 MASTER IMPLEMENTATION
    // Uses UART event queue for TRUE non-blocking TX complete detection
    // This mimics AVR's TX complete interrupt (USART_TX_vect)
    // ========================================================================

    RS485Master::RS485Master()
        : uartNum((uart_port_t)RS485_UART_NUM),
          txEnablePin(TXENABLE_PIN),
          uartEventQueue(NULL),
          state(IDLE),
          poll_address(1),
          poll_address_counter(1),
          scan_address_counter(1),
          rxtx_len(0),
          rx_msgtype(0),
          checksum(0),
          rx_start_time(0),
          txDoneFlag(false)
    {
        slave_present[0] = true;
        for (uint8_t i = 1; i < 128; i++) {
            slave_present[i] = false;
        }
    }

    void RS485Master::begin() {
        uart_config_t uart_config = {
            .baud_rate = RS485_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };

        // Install UART driver WITH event queue - this gives us TX_DONE events!
        // The event queue is the key to non-blocking TX complete detection
        uart_driver_install(uartNum, RS485_RX_BUFFER_SIZE, RS485_TX_BUFFER_SIZE,
                           20, &uartEventQueue, 0);
        uart_param_config(uartNum, &uart_config);
        uart_set_pin(uartNum, RS485_TX_PIN, RS485_RX_PIN, txEnablePin, UART_PIN_NO_CHANGE);

        // Enable RS485 half-duplex mode - driver handles DE/RE automatically
        if (txEnablePin >= 0) {
            uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX);
        }

        uart_flush_input(uartNum);
    }

    void RS485Master::processUartEvents() {
        // Process all pending UART events (non-blocking)
        // This is how we detect TX_DONE without blocking!
        uart_event_t event;
        while (xQueueReceive(uartEventQueue, &event, 0) == pdTRUE) {
            switch (event.type) {
                case UART_TX_DONE:
                    txDoneFlag = true;
                    break;
                case UART_DATA:
                    // Data available - handled in main loop
                    break;
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    // Overflow - flush and continue
                    uart_flush_input(uartNum);
                    break;
                default:
                    break;
            }
        }
    }

    void RS485Master::advancePollAddress() {
        poll_address_counter = (poll_address_counter + 1) % 128;

        while (!slave_present[poll_address_counter]) {
            poll_address_counter = (poll_address_counter + 1) % 128;
        }

        if (poll_address_counter == 0) {
            scan_address_counter = (scan_address_counter + 1) % 128;
            while (slave_present[scan_address_counter]) {
                scan_address_counter = (scan_address_counter + 1) % 128;
            }
            poll_address = scan_address_counter;
            return;
        }

        poll_address = poll_address_counter;
    }

    void RS485Master::loop() {
        // =====================================================================
        // STEP 1: Process UART events (non-blocking TX_DONE detection)
        // This replaces blocking uart_wait_tx_done()
        // =====================================================================
        processUartEvents();

        // =====================================================================
        // STEP 2: Handle TX_WAIT states - transition when TX is done
        // =====================================================================
        if (txDoneFlag) {
            switch (state) {
                case POLL_WAIT_TX_DONE:
                    // Poll packet sent, switch to RX mode
                    uart_flush_input(uartNum);
                    rx_start_time = micros();
                    state = RX_WAIT_DATALENGTH;
                    txDoneFlag = false;
                    break;

                case TIMEOUT_WAIT_TX_DONE:
                    state = IDLE;
                    txDoneFlag = false;
                    break;

                case TX_WAIT_TX_DONE:
                    state = IDLE;
                    txDoneFlag = false;
                    break;

                default:
                    // Clear flag if not in a wait state
                    txDoneFlag = false;
                    break;
            }
        }

        // =====================================================================
        // STEP 3: Handle RX states - read data from slaves
        // =====================================================================
        size_t available = 0;
        uart_get_buffered_data_len(uartNum, &available);

        while (available > 0 && state >= RX_WAIT_DATALENGTH && state <= RX_WAIT_CHECKSUM) {
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
                    rx_start_time = micros();
                    break;

                case RX_WAIT_MSGTYPE:
                    rx_msgtype = c;
                    state = RX_WAIT_DATA;
                    rx_start_time = micros();
                    break;

                case RX_WAIT_DATA:
                    messageBuffer.put(c);
                    rxtx_len--;
                    if (rxtx_len == 0) {
                        state = RX_WAIT_CHECKSUM;
                    }
                    rx_start_time = micros();
                    break;

                case RX_WAIT_CHECKSUM:
                    messageBuffer.complete = true;
                    state = IDLE;
                    break;
            }
        }

        // =====================================================================
        // STEP 4: Handle IDLE state - start new operations
        // =====================================================================
        if (state == IDLE) {
            if (exportData.isNotEmpty()) {
                // We have export data to send - start broadcast
                rxtx_len = exportData.getLength();
                checksum = 0;

                uint8_t addr = 0;  // Broadcast address
                uart_write_bytes(uartNum, (const char*)&addr, 1);
                state = TX_ADDRESS_SENT;

            } else if (messageBuffer.isEmpty() && !messageBuffer.complete) {
                // Message buffer free, poll a device
                advancePollAddress();

                uint8_t addr = poll_address;
                uart_write_bytes(uartNum, (const char*)&addr, 1);
                state = POLL_ADDRESS_SENT;

            } else {
                pcConnection.checkTimeout();
            }
        }

        // =====================================================================
        // STEP 5: TX state machine - send bytes (like AVR UDRE ISR)
        // All sends are non-blocking, we wait for TX_DONE via event queue
        // =====================================================================
        size_t txFree = 0;
        uart_get_tx_buffer_free_size(uartNum, &txFree);

        if (txFree > 0) {
            uint8_t txByte;

            switch (state) {
                // --- POLL sequence ---
                case POLL_ADDRESS_SENT:
                    txByte = 0;  // msgtype
                    uart_write_bytes(uartNum, (const char*)&txByte, 1);
                    state = POLL_MSGTYPE_SENT;
                    break;

                case POLL_MSGTYPE_SENT:
                    txByte = 0;  // data length = 0 for poll
                    uart_write_bytes(uartNum, (const char*)&txByte, 1);
                    state = POLL_DATALENGTH_SENT;
                    break;

                case POLL_DATALENGTH_SENT:
                    // Transition to wait state - TX_DONE event will trigger next state
                    state = POLL_WAIT_TX_DONE;
                    break;

                // --- Broadcast TX sequence ---
                case TX_ADDRESS_SENT:
                    txByte = 0;  // msgtype
                    uart_write_bytes(uartNum, (const char*)&txByte, 1);
                    state = TX_MSGTYPE_SENT;
                    break;

                case TX_MSGTYPE_SENT:
                    txByte = rxtx_len;  // data length
                    uart_write_bytes(uartNum, (const char*)&txByte, 1);
                    state = TX;
                    break;

                case TX:
                    if (rxtx_len == 0) {
                        uart_write_bytes(uartNum, (const char*)&checksum, 1);
                        state = TX_CHECKSUM_SENT;
                    } else {
                        txByte = exportData.get();
                        checksum ^= txByte;
                        uart_write_bytes(uartNum, (const char*)&txByte, 1);
                        rxtx_len--;
                    }
                    break;

                case TX_CHECKSUM_SENT:
                    // Transition to wait state
                    state = TX_WAIT_TX_DONE;
                    break;

                // --- Timeout ---
                case TIMEOUT_ZEROBYTE_SENT:
                    state = TIMEOUT_WAIT_TX_DONE;
                    break;
            }
        }

        // =====================================================================
        // STEP 6: Timeout handling
        // =====================================================================

        // Timeout for non-existing devices (1ms)
        if (state == RX_WAIT_DATALENGTH && (micros() - rx_start_time) > 1000) {
            slave_present[poll_address] = false;
            uint8_t zero = 0;
            uart_write_bytes(uartNum, (const char*)&zero, 1);
            state = TIMEOUT_ZEROBYTE_SENT;
        }

        // Timeout during message reception (5ms)
        if ((state == RX_WAIT_MSGTYPE || state == RX_WAIT_DATA || state == RX_WAIT_CHECKSUM)
            && (micros() - rx_start_time) > 5000) {
            messageBuffer.clear();
            messageBuffer.put('\n');
            messageBuffer.complete = true;
            state = IDLE;
        }
    }

    // ========================================================================
    // PC CONNECTION IMPLEMENTATION
    // Uses a dedicated HIGH-PRIORITY task on Core 0 for reading PC serial
    // This mimics AVR's USART0_RX ISR which fires on every byte
    // ========================================================================

    MasterPCConnection::MasterPCConnection()
        : tx_start_time(0), rxTaskHandle(NULL) {}

    // Static task function - runs on Core 0 with high priority
    // This task ONLY reads from Serial and feeds data to exportData buffer
    // It runs independently of the main loop, like an ISR
    void MasterPCConnection::rxTaskFunc(void* param) {
        // This task runs forever, constantly checking for PC serial data
        // It's pinned to Core 0, while main loop runs on Core 1
        // This gives us TRUE concurrency like AVR ISRs

        for (;;) {
            // Read all available bytes
            while (Serial.available()) {
                uint8_t c = Serial.read();
                rs485master.feedExportByte(c);
            }

            // Small yield to prevent watchdog issues
            // 1ms delay is acceptable - at 250kbaud, ~25 bytes arrive per ms
            // With 256-byte buffer, we have 10ms margin
            vTaskDelay(1);
        }
    }

    void MasterPCConnection::begin() {
        // Initialize Serial with larger RX buffer
        Serial.setRxBufferSize(PC_SERIAL_RX_BUFFER_SIZE);
        Serial.begin(RS485_BAUD_RATE);

        while (!Serial && millis() < 3000) {
            delay(1);
        }

        // Create high-priority task on Core 0 for PC serial reading
        // Priority 10 is high (higher than most system tasks)
        // Core 0: Protocol CPU (handles WiFi, BT, etc) - we use it for Serial
        // Core 1: Application CPU - we use it for RS485 state machine
        xTaskCreatePinnedToCore(
            rxTaskFunc,         // Task function
            "PCSerialRX",       // Task name
            4096,               // Stack size
            NULL,               // Parameters
            10,                 // Priority (high)
            &rxTaskHandle,      // Task handle
            0                   // Core 0
        );
    }

    void MasterPCConnection::txProcess() {
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
        if (rs485master.messageBuffer.isNotEmpty() && (micros() - tx_start_time) > 5000) {
            rs485master.messageBuffer.clear();
            rs485master.messageBuffer.put('\n');
            rs485master.messageBuffer.complete = true;
        }
    }

    // ========================================================================
    // DCSBIOS SETUP AND LOOP
    // ========================================================================

    void setup() {
        pcConnection.begin();   // Starts the PC serial RX task on Core 0
        rs485master.begin();    // Initializes RS485 with event queue
    }

    void loop() {
        // Main loop runs on Core 1
        // PC serial reading is handled by dedicated task on Core 0
        // This gives us true concurrency like AVR's ISRs

        rs485master.loop();
        pcConnection.txProcess();
    }

}  // namespace DcsBios

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
