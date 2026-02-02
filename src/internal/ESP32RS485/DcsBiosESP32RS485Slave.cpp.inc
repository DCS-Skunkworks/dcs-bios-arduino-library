#ifdef DCSBIOS_RS485_SLAVE
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Slave.h"

namespace DcsBios {

    // ========================================================================
    // GLOBAL INSTANCES
    // ========================================================================

    ProtocolParser parser;
    RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;
    RS485Slave rs485slave(DCSBIOS_RS485_SLAVE);

    // ========================================================================
    // MESSAGE SENDING
    // ========================================================================

    bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
        if (messageBuffer.complete) return false;  // Buffer occupied

        messageBuffer.clear();
        const char* c = msg;
        while (*c) {
            messageBuffer.put(*c++);
        }
        messageBuffer.put(' ');
        c = arg;
        while (*c) {
            messageBuffer.put(*c++);
        }
        messageBuffer.put('\n');

        messageBuffer.complete = true;
        PollingInput::setMessageSentOrQueued();
        return true;
    }

    // ========================================================================
    // RS485 SLAVE IMPLEMENTATION
    // ========================================================================

    RS485Slave::RS485Slave(uint8_t address)
        : uartNum((uart_port_t)RS485_UART_NUM),
          dePin((gpio_num_t)RS485_EN_PIN),
          slaveAddress(address),
          state(UNINITIALIZED),
          rx_slave_address(0),
          rx_msgtype(0),
          rxtx_len(0),
          last_rx_time(0),
          tx_complete_time(0),
          rx_datatype(RXDATA_IGNORE)
    {
    }

    void RS485Slave::begin() {
        // =====================================================================
        // GPIO: Manual DE pin control
        // =====================================================================
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dePin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        setDE(false);  // Start in RX mode (listening)

        // =====================================================================
        // UART: Normal mode with manual DE control
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

        uart_driver_install(uartNum, RS485_UART_BUFFER_SIZE, RS485_UART_BUFFER_SIZE, 0, NULL, 0);
        uart_param_config(uartNum, &uart_config);
        uart_set_pin(uartNum, RS485_TX_PIN, RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_set_mode(uartNum, UART_MODE_UART);  // Normal UART mode, manual DE

        uart_flush_input(uartNum);
    }

    void RS485Slave::setDE(bool high) {
        gpio_set_level(dePin, high ? 1 : 0);
    }

    int64_t RS485Slave::calculateTxCompleteTime(size_t bytes) {
        int64_t now = esp_timer_get_time();
        int64_t tx_duration = (bytes * US_PER_BYTE) + TX_MARGIN_US;
        return now + tx_duration;
    }

    void RS485Slave::sendResponse() {
        // Build response packet: [Length] [MsgType] [Data...] [Checksum]
        uint8_t packet[MESSAGE_BUFFER_SIZE + 3];
        uint8_t len = messageBuffer.getLength();
        uint8_t checksum = 0x72;  // Fixed checksum like AVR implementation

        packet[0] = len + 1;  // Length includes msgtype
        packet[1] = 0;        // Message type = 0

        for (uint8_t i = 0; i < len; i++) {
            packet[2 + i] = messageBuffer.get();
        }
        packet[2 + len] = checksum;

        // Raise DE, send packet
        setDE(true);
        delayMicroseconds(10);  // Small delay for DE to stabilize
        uart_write_bytes(uartNum, (const char*)packet, 3 + len);

        tx_complete_time = calculateTxCompleteTime(3 + len);
        state = TX_WAIT_COMPLETE;
    }

    void RS485Slave::loop() {
        int64_t now = esp_timer_get_time();

        // =====================================================================
        // Handle TX completion
        // =====================================================================
        if (state == TX_WAIT_COMPLETE) {
            if (now >= tx_complete_time) {
                // TX complete - return to RX mode
                setDE(false);
                messageBuffer.clear();
                messageBuffer.complete = false;
                state = RX_WAIT_ADDRESS;
            }
            return;  // Don't process RX while TX in progress
        }

        // =====================================================================
        // Process incoming bytes
        // =====================================================================
        size_t available = 0;
        uart_get_buffered_data_len(uartNum, &available);

        while (available > 0) {
            uint8_t c;
            if (uart_read_bytes(uartNum, &c, 1, 0) != 1) break;
            available--;

            last_rx_time = now;

            switch (state) {
                case UNINITIALIZED:
                    // First byte received - start sync
                    state = SYNC;
                    break;

                case SYNC:
                    // Wait for silence, but we received a byte - stay in sync
                    // The silence check is done below, outside this switch
                    break;

                case RX_WAIT_ADDRESS:
                    rx_slave_address = c;
                    state = RX_WAIT_MSGTYPE;
                    break;

                case RX_WAIT_MSGTYPE:
                    rx_msgtype = c;
                    state = RX_WAIT_DATALENGTH;
                    break;

                case RX_WAIT_DATALENGTH:
                    rxtx_len = c;

                    if (rxtx_len == 0) {
                        // No data - message complete, handle it
                        goto handle_message_complete;
                    }

                    // Determine what to do with incoming data
                    if (rx_slave_address == 0 && rx_msgtype == 0) {
                        // Broadcast export data
                        rx_datatype = RXDATA_DCSBIOS_EXPORT;
                    } else {
                        rx_datatype = RXDATA_IGNORE;
                    }

                    state = RX_WAIT_DATA;
                    break;

                case RX_WAIT_DATA:
                    rxtx_len--;
                    if (rx_datatype == RXDATA_DCSBIOS_EXPORT) {
                        // Feed to protocol parser
                        parser.processChar(c);
                    }
                    if (rxtx_len == 0) {
                        state = RX_WAIT_CHECKSUM;
                    }
                    break;

                case RX_WAIT_CHECKSUM:
                    // Ignore checksum validation (like AVR)
                    goto handle_message_complete;

                case RX_WAIT_ANSWER_DATALENGTH:
                    // Another slave is responding
                    rxtx_len = c;
                    if (rxtx_len == 0) {
                        state = RX_WAIT_ADDRESS;
                    } else {
                        state = RX_WAIT_ANSWER_MSGTYPE;
                    }
                    break;

                case RX_WAIT_ANSWER_MSGTYPE:
                    state = RX_WAIT_ANSWER_DATA;
                    break;

                case RX_WAIT_ANSWER_DATA:
                    rxtx_len--;
                    if (rxtx_len == 0) {
                        state = RX_WAIT_ANSWER_CHECKSUM;
                    }
                    break;

                case RX_WAIT_ANSWER_CHECKSUM:
                    // Ignore checksum, back to waiting for next message
                    state = RX_WAIT_ADDRESS;
                    break;

                default:
                    break;
            }

            continue;  // Process next byte

        handle_message_complete:
            if (rx_slave_address == 0) {
                // Broadcast messages must not be answered
                state = RX_WAIT_ADDRESS;
            } else if (rx_slave_address == slaveAddress) {
                // This message is for us - we need to respond
                if (rx_msgtype == 0 && rxtx_len == 0) {
                    // We've been polled for DCS-BIOS input data
                    if (!messageBuffer.complete) {
                        // Nothing to send - respond with zero length
                        uint8_t zero_response[1] = {0};
                        setDE(true);
                        delayMicroseconds(10);
                        uart_write_bytes(uartNum, (const char*)zero_response, 1);
                        tx_complete_time = calculateTxCompleteTime(1);
                        state = TX_WAIT_COMPLETE;
                    } else {
                        // We have data to send
                        sendResponse();
                    }
                } else {
                    // Unexpected message type - resync
                    state = SYNC;
                }
            } else {
                // Message is for another slave - wait for their response
                state = RX_WAIT_ANSWER_DATALENGTH;
            }
        }

        // =====================================================================
        // Sync detection - if no byte received for 500µs, we're synced
        // =====================================================================
        if (state == SYNC) {
            if ((now - last_rx_time) > SYNC_TIMEOUT_US) {
                state = RX_WAIT_ADDRESS;
            }
        }
    }

    // ========================================================================
    // DCSBIOS SETUP AND LOOP
    // ========================================================================

    void setup() {
        rs485slave.begin();
    }

    void loop() {
        // Process RS485 communication
        rs485slave.loop();

        // Poll inputs (buttons, switches, encoders, etc.)
        PollingInput::pollInputs();

        // Process export data listeners (LEDs, servos, displays, etc.)
        ExportStreamListener::loopAll();
    }

    void resetAllStates() {
        PollingInput::resetAllStates();
    }

}  // namespace DcsBios

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_SLAVE
