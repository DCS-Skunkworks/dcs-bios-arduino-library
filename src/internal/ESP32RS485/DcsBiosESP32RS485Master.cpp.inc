#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Master.h"

namespace DcsBios {

    // Global instances
    RS485Master rs485master;
    MasterPCConnection pcConnection;

    // ========================================================================
    // RS485 MASTER IMPLEMENTATION - Mirrors AVR exactly
    // CRITICAL: No blocking calls! Must poll for TX completion.
    // ========================================================================

    RS485Master::RS485Master()
        : uartNum((uart_port_t)RS485_UART_NUM),
          txEnablePin(TXENABLE_PIN),
          state(IDLE),
          poll_address(1),
          poll_address_counter(1),
          scan_address_counter(1),
          rxtx_len(0),
          rx_msgtype(0),
          checksum(0),
          rx_start_time(0)
    {
        // Slave 0 is broadcast, mark as "present" to skip polling it
        slave_present[0] = true;
        for (uint8_t i = 1; i < 128; i++) {
            slave_present[i] = false;
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

        uart_driver_install(uartNum, 256, 256, 0, NULL, 0);
        uart_param_config(uartNum, &uart_config);
        uart_set_pin(uartNum, RS485_TX_PIN, RS485_RX_PIN, txEnablePin, UART_PIN_NO_CHANGE);

        if (txEnablePin >= 0) {
            uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX);
        }

        uart_flush_input(uartNum);
    }

    void RS485Master::advancePollAddress() {
        poll_address_counter = (poll_address_counter + 1) % 128;

        // Advance to next present slave
        while (!slave_present[poll_address_counter]) {
            poll_address_counter = (poll_address_counter + 1) % 128;
        }

        if (poll_address_counter == 0) {
            // Instead of polling slave 0, scan for new device
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
        // =================================================================
        // CRITICAL: Read PC serial BEFORE any state processing
        // This mimics AVR ISR firing independently of main code
        // =================================================================
        pcConnection.rxProcess();

        // =================================================================
        // Handle RX states - check for incoming data from slaves
        // =================================================================
        size_t available = 0;
        uart_get_buffered_data_len(uartNum, &available);

        while (available > 0 && state >= RX_WAIT_DATALENGTH && state <= RX_WAIT_CHECKSUM) {
            uint8_t c;
            if (uart_read_bytes(uartNum, &c, 1, 0) != 1) break;
            available--;

            // Read PC serial between every byte processed
            pcConnection.rxProcess();

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
                    // Checksum received (ignore validation like AVR)
                    messageBuffer.complete = true;
                    state = IDLE;
                    break;
            }
        }

        // Read PC serial after RX processing
        pcConnection.rxProcess();

        // =================================================================
        // Handle TX wait states - NON-BLOCKING check for TX completion
        // =================================================================
        switch (state) {
            case POLL_WAIT_TX_COMPLETE:
                if (isTxComplete()) {
                    uart_flush_input(uartNum);
                    rx_start_time = micros();
                    state = RX_WAIT_DATALENGTH;
                }
                pcConnection.rxProcess();
                break;

            case TIMEOUT_WAIT_TX_COMPLETE:
                if (isTxComplete()) {
                    state = IDLE;
                }
                pcConnection.rxProcess();
                break;

            case TX_WAIT_COMPLETE:
                if (isTxComplete()) {
                    state = IDLE;
                }
                pcConnection.rxProcess();
                break;
        }

        // =================================================================
        // IDLE state - start new operations
        // =================================================================
        if (state == IDLE) {
            pcConnection.rxProcess();

            if (exportData.isNotEmpty()) {
                // We have export data to send - start broadcast
                rxtx_len = exportData.getLength();
                checksum = 0;

                // Send address byte (0 = broadcast)
                uint8_t addr = 0;
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

        // Read PC serial after IDLE processing
        pcConnection.rxProcess();

        // =================================================================
        // TX state machine - send bytes one at a time (like AVR UDRE ISR)
        // NON-BLOCKING: only send when TX buffer has space
        // =================================================================
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
                    pcConnection.rxProcess();
                    break;

                case POLL_MSGTYPE_SENT:
                    txByte = 0;  // data length = 0 for poll
                    uart_write_bytes(uartNum, (const char*)&txByte, 1);
                    state = POLL_DATALENGTH_SENT;
                    pcConnection.rxProcess();
                    break;

                case POLL_DATALENGTH_SENT:
                    // Move to wait state - NON-BLOCKING
                    state = POLL_WAIT_TX_COMPLETE;
                    pcConnection.rxProcess();
                    break;

                // --- Broadcast TX sequence ---
                case TX_ADDRESS_SENT:
                    txByte = 0;  // msgtype
                    uart_write_bytes(uartNum, (const char*)&txByte, 1);
                    state = TX_MSGTYPE_SENT;
                    pcConnection.rxProcess();
                    break;

                case TX_MSGTYPE_SENT:
                    txByte = rxtx_len;  // data length
                    uart_write_bytes(uartNum, (const char*)&txByte, 1);
                    state = TX;
                    pcConnection.rxProcess();
                    break;

                case TX:
                    pcConnection.rxProcess();  // Read before potentially long operation
                    if (rxtx_len == 0) {
                        // Send checksum
                        uart_write_bytes(uartNum, (const char*)&checksum, 1);
                        state = TX_CHECKSUM_SENT;
                    } else {
                        // Send next data byte from export buffer
                        txByte = exportData.get();
                        checksum ^= txByte;
                        uart_write_bytes(uartNum, (const char*)&txByte, 1);
                        rxtx_len--;
                    }
                    pcConnection.rxProcess();
                    break;

                case TX_CHECKSUM_SENT:
                    // Move to wait state - NON-BLOCKING
                    state = TX_WAIT_COMPLETE;
                    pcConnection.rxProcess();
                    break;

                // --- Timeout byte ---
                case TIMEOUT_ZEROBYTE_SENT:
                    // Move to wait state - NON-BLOCKING
                    state = TIMEOUT_WAIT_TX_COMPLETE;
                    pcConnection.rxProcess();
                    break;
            }
        }

        // Read PC serial after TX processing
        pcConnection.rxProcess();

        // =================================================================
        // Timeout handling
        // =================================================================

        // Timeout for non-existing devices (1ms like AVR)
        if (state == RX_WAIT_DATALENGTH && (micros() - rx_start_time) > 1000) {
            slave_present[poll_address] = false;
            // Send zero byte on behalf of non-existing device
            uint8_t zero = 0;
            uart_write_bytes(uartNum, (const char*)&zero, 1);
            state = TIMEOUT_ZEROBYTE_SENT;
            pcConnection.rxProcess();
        }

        // Timeout during message reception (5ms like AVR)
        if ((state == RX_WAIT_MSGTYPE || state == RX_WAIT_DATA || state == RX_WAIT_CHECKSUM)
            && (micros() - rx_start_time) > 5000) {
            messageBuffer.clear();
            messageBuffer.put('\n');
            messageBuffer.complete = true;
            state = IDLE;
            pcConnection.rxProcess();
        }

        // Final PC serial read
        pcConnection.rxProcess();
    }

    // ========================================================================
    // PC CONNECTION IMPLEMENTATION - Mirrors AVR exactly
    // ========================================================================

    MasterPCConnection::MasterPCConnection() : tx_start_time(0) {}

    void MasterPCConnection::begin() {
        Serial.begin(RS485_BAUD_RATE);
        while (!Serial && millis() < 3000) {
            delay(1);
        }
    }

    int MasterPCConnection::rxProcess() {
        // Read ALL available bytes from PC and feed to RS485 master
        // This mimics the AVR USART0_RX ISR which fires on every byte
        int count = 0;
        while (Serial.available()) {
            uint8_t c = Serial.read();
            rs485master.feedExportByte(c);
            count++;
        }
        return count;
    }

    void MasterPCConnection::txProcess() {
        // Send slave responses to PC
        // This mimics the AVR USART0_UDRE ISR
        if (rs485master.messageBuffer.isEmpty()) {
            if (rs485master.messageBuffer.complete) {
                rs485master.messageBuffer.complete = false;
            }
            return;
        }

        // Send all available bytes
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
        pcConnection.begin();
        rs485master.begin();
    }

    void loop() {
        // RS485 loop handles PC serial reading internally at every opportunity
        rs485master.loop();

        // Send any slave responses to PC
        pcConnection.txProcess();
    }

}  // namespace DcsBios

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
