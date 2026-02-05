#ifdef DCSBIOS_RS485_SLAVE
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Slave.h"

namespace DcsBios {

    // ========================================================================
    // GLOBAL INSTANCES
    // ========================================================================

    ProtocolParser parser;
    RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

    // ========================================================================
    // EXPORT DATA BUFFER - ISR queues bytes here, main loop processes
    // ========================================================================
    // This decouples the ISR from the ProtocolParser, keeping ISR time short
    // and letting the parser run safely in the main loop context.

    static volatile uint8_t exportBuffer[EXPORT_BUFFER_SIZE];
    static volatile uint8_t exportWritePos = 0;
    static volatile uint8_t exportReadPos = 0;

    // Returns free slots in export queue (one slot kept empty to distinguish full/empty)
    static inline uint8_t exportBufferAvailableForWrite() {
        return (uint8_t)((exportReadPos - exportWritePos - 1 + EXPORT_BUFFER_SIZE) % EXPORT_BUFFER_SIZE);
    }

    // ========================================================================
    // RS485 STATE MACHINE VARIABLES (all volatile for ISR access)
    // ========================================================================

    static volatile RS485State rs485State = STATE_SYNC;
    static volatile uint8_t rxSlaveAddress = 0;
    static volatile uint8_t rxMsgType = 0;
    static volatile uint8_t rxtxLen = 0;
    static volatile RxDataType rxDataType = RXDATA_IGNORE;
    static volatile int64_t lastRxTime = 0;
    static volatile uint32_t rxByteCount = 0;

    // ========================================================================
    // EXPORT BUFFER OVERFLOW HANDLING
    // ========================================================================
    // Match AVR behavior under overload: force re-sync instead of silently
    // continuing with corrupted data. Drop queued partial export data so
    // parser restarts cleanly on next frame.

    static inline void IRAM_ATTR handleExportOverflowISR(int64_t now) {
        rs485State = STATE_SYNC;
        lastRxTime = now;
        exportReadPos = 0;
        exportWritePos = 0;
    }

    static inline void handleExportOverflow(int64_t now) {
        rs485State = STATE_SYNC;
        lastRxTime = now;
        exportReadPos = 0;
        exportWritePos = 0;
    }

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

    // ########################################################################
    //
    //    ISR MODE IMPLEMENTATION
    //
    // ########################################################################

#if USE_ISR_MODE

    // ========================================================================
    // UART HARDWARE POINTER
    // ========================================================================

#if RS485_UART_NUM == 0
    static uart_dev_t* const uartHw = &UART0;
#elif RS485_UART_NUM == 1
    static uart_dev_t* const uartHw = &UART1;
#elif RS485_UART_NUM == 2
    static uart_dev_t* const uartHw = &UART2;
#endif
    static intr_handle_t uartIntrHandle;

    // ========================================================================
    // DIRECT GPIO CONTROL (portable across Xtensa and RISC-V)
    // ========================================================================

#if RS485_DE_PIN >= 0
    static inline void IRAM_ATTR setDE_ISR(bool high) {
        gpio_ll_set_level(&GPIO, (gpio_num_t)RS485_DE_PIN, high ? 1 : 0);
    }
#else
    #define setDE_ISR(x)
#endif

    // ========================================================================
    // TX FROM ISR - Send response immediately!
    // ========================================================================

    // Helper: write one byte and wait for it to transmit (true byte-by-byte like AVR)
    static inline void IRAM_ATTR txByteWaitIdle(uint8_t b) {
        uart_ll_write_txfifo(uartHw, &b, 1);
        while (!uart_ll_is_tx_idle(uartHw));  // Wait for byte to fully transmit
    }

    // Warm up UART and enable DE for transmission
    static inline void IRAM_ATTR prepareForTransmit() {
#if RS485_DE_PIN >= 0
        // Manual DE control: enable driver, then wait for stabilization
        setDE_ISR(true);
        ets_delay_us(TX_WARMUP_DELAY_MANUAL_US);
#else
        // Auto-direction: wait for transceiver to detect TX and switch
        ets_delay_us(TX_WARMUP_DELAY_AUTO_US);
#endif
    }

    static void IRAM_ATTR sendResponseISR() {
        uint8_t len = messageBuffer.getLength();

        // Disable RX interrupt during TX to prevent echo bytes triggering ISR
        uart_ll_disable_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

        // Warm up UART and enable DE
        prepareForTransmit();

#if TX_MODE_BYTE_BY_BYTE
        // === BYTE-BY-BYTE MODE ===
        // Send each byte individually, wait for TX idle after each (like AVR UDRE)
        txByteWaitIdle(len);         // Length byte
        txByteWaitIdle(0);           // MsgType = 0

        for (uint8_t i = 0; i < len; i++) {
            txByteWaitIdle(messageBuffer.get());
        }

        txByteWaitIdle(0x72);        // Checksum
#else
        // === BUFFERED MODE ===
        // Build complete response in local buffer for continuous transmission
        uint8_t txBuf[MESSAGE_BUFFER_SIZE + 4];  // len + msgtype + data + checksum
        uint8_t txLen = 0;

        txBuf[txLen++] = len;        // Length byte
        txBuf[txLen++] = 0;          // MsgType = 0

        for (uint8_t i = 0; i < len; i++) {
            txBuf[txLen++] = messageBuffer.get();
        }

        txBuf[txLen++] = 0x72;       // Checksum

        // Send entire buffer at once - FIFO is 128 bytes, plenty for our messages
        uart_ll_write_txfifo(uartHw, txBuf, txLen);
#endif

        // Wait for transmission to fully complete
        while (!uart_ll_is_tx_idle(uartHw));

#if RS485_DE_PIN >= 0 && TX_COOLDOWN_DELAY_US > 0
        // Allow transceiver TX->RX turnaround before releasing DE
        ets_delay_us(TX_COOLDOWN_DELAY_US);
#endif

        // Disable driver
        setDE_ISR(false);

        // Flush RX FIFO (echo bytes)
        uart_ll_rxfifo_rst(uartHw);

        // Re-enable RX interrupt
        uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
        uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

        // Clear message buffer
        messageBuffer.clear();
        messageBuffer.complete = false;

        rs485State = STATE_RX_WAIT_ADDRESS;
    }

    static void IRAM_ATTR sendZeroLengthResponseISR() {
        uint8_t zero = 0;

        // Disable RX interrupt during TX
        uart_ll_disable_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

        // Warm up UART and enable DE
        prepareForTransmit();

        // Send zero length byte
        uart_ll_write_txfifo(uartHw, &zero, 1);

        // Wait for transmission to complete
        while (!uart_ll_is_tx_idle(uartHw));

#if RS485_DE_PIN >= 0 && TX_COOLDOWN_DELAY_US > 0
        // Allow transceiver TX->RX turnaround before releasing DE
        ets_delay_us(TX_COOLDOWN_DELAY_US);
#endif

        // Disable driver
        setDE_ISR(false);

        // Flush RX FIFO (echo)
        uart_ll_rxfifo_rst(uartHw);

        // Re-enable RX interrupt
        uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
        uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

        rs485State = STATE_RX_WAIT_ADDRESS;
    }

    // ========================================================================
    // UART RX ISR - This is where the magic happens!
    // Fires immediately when byte arrives, processes state machine,
    // responds instantly to poll requests.
    // ========================================================================

    static void IRAM_ATTR uart_isr_handler(void *arg) {
        uint32_t uart_intr_status = uart_ll_get_intsts_mask(uartHw);

        // Process all available bytes
        while (uart_ll_get_rxfifo_len(uartHw) > 0) {
            uint8_t c;
            uart_ll_read_rxfifo(uartHw, &c, 1);
#ifdef __riscv
            // RISC-V (C3, C6, H2): memory fence ensures FIFO read pointer update
            // propagates before next FIFO length check. Without this, the CPU can
            // read faster than the pointer updates, causing stale/duplicate reads.
            __asm__ __volatile__("fence");
#endif

            rxByteCount++;
            int64_t now = esp_timer_get_time();

            // Sync detection - if gap > 500us, reset to wait for address
            if (rs485State == STATE_SYNC) {
                if ((now - lastRxTime) >= SYNC_TIMEOUT_US) {
                    rs485State = STATE_RX_WAIT_ADDRESS;
                    // Fall through to process this byte as address
                } else {
                    lastRxTime = now;
                    continue;  // Stay in sync, discard byte
                }
            }

            switch (rs485State) {
                case STATE_RX_WAIT_ADDRESS:
                    rxSlaveAddress = c;
                    rs485State = STATE_RX_WAIT_MSGTYPE;
                    break;

                case STATE_RX_WAIT_MSGTYPE:
                    rxMsgType = c;
                    rs485State = STATE_RX_WAIT_DATALENGTH;
                    break;

                case STATE_RX_WAIT_DATALENGTH:
                    rxtxLen = c;

                    if (rxtxLen == 0) {
                        // Message complete - handle it
                        if (rxSlaveAddress == 0) {
                            // Broadcast - ignore
                            rs485State = STATE_RX_WAIT_ADDRESS;
                        } else if (rxSlaveAddress == DCSBIOS_RS485_SLAVE) {
                            // Poll for us! Respond IMMEDIATELY!
                            if (rxMsgType == 0) {
                                if (messageBuffer.complete) {
                                    sendResponseISR();
                                } else {
                                    sendZeroLengthResponseISR();
                                }
                            } else {
                                rs485State = STATE_SYNC;
                            }
                        } else {
                            // Poll for another slave - wait for their response
                            rs485State = STATE_RX_WAIT_ANSWER_DATALENGTH;
                        }
                    } else {
                        // Has data - determine type
                        if (rxSlaveAddress == 0 && rxMsgType == 0) {
                            rxDataType = RXDATA_DCSBIOS_EXPORT;
                        } else {
                            rxDataType = RXDATA_IGNORE;
                        }
                        rs485State = STATE_RX_WAIT_DATA;
                    }
                    break;

                case STATE_RX_WAIT_DATA:
                    rxtxLen--;

                    // Queue export data for main loop processing
                    if (rxDataType == RXDATA_DCSBIOS_EXPORT) {
                        if (exportBufferAvailableForWrite() == 0) {
                            handleExportOverflowISR(now);
                            break;
                        }
                        uint8_t nextPos = (exportWritePos + 1) % EXPORT_BUFFER_SIZE;
                        exportBuffer[exportWritePos] = c;
                        exportWritePos = nextPos;
                    }

                    if (rxtxLen == 0) {
                        rs485State = STATE_RX_WAIT_CHECKSUM;
                    }
                    break;

                case STATE_RX_WAIT_CHECKSUM:
                    // Message complete
                    if (rxSlaveAddress == 0) {
                        rs485State = STATE_RX_WAIT_ADDRESS;
                    } else if (rxSlaveAddress == DCSBIOS_RS485_SLAVE) {
                        // This was addressed to us with data - respond
                        if (rxMsgType == 0) {
                            if (messageBuffer.complete) {
                                sendResponseISR();
                            } else {
                                sendZeroLengthResponseISR();
                            }
                        } else {
                            rs485State = STATE_SYNC;
                        }
                    } else {
                        rs485State = STATE_RX_WAIT_ANSWER_DATALENGTH;
                    }
                    break;

                case STATE_RX_WAIT_ANSWER_DATALENGTH:
                    rxtxLen = c;
                    if (rxtxLen == 0) {
                        rs485State = STATE_RX_WAIT_ADDRESS;
                    } else {
                        rs485State = STATE_RX_WAIT_ANSWER_MSGTYPE;
                    }
                    break;

                case STATE_RX_WAIT_ANSWER_MSGTYPE:
                    rs485State = STATE_RX_WAIT_ANSWER_DATA;
                    break;

                case STATE_RX_WAIT_ANSWER_DATA:
                    rxtxLen--;
                    if (rxtxLen == 0) {
                        rs485State = STATE_RX_WAIT_ANSWER_CHECKSUM;
                    }
                    break;

                case STATE_RX_WAIT_ANSWER_CHECKSUM:
                    rs485State = STATE_RX_WAIT_ADDRESS;
                    break;

                default:
                    rs485State = STATE_SYNC;
                    break;
            }

            lastRxTime = now;
        }

        // Clear interrupt status
        uart_ll_clr_intsts_mask(uartHw, uart_intr_status);
    }

    // ========================================================================
    // ISR MODE - HARDWARE INITIALIZATION
    // ========================================================================

    static void initRS485Hardware() {
        // =====================================================================
        // GPIO: DE pin configuration
        // =====================================================================
#if RS485_DE_PIN >= 0
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << RS485_DE_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        setDE_ISR(false);  // Start in RX mode
#endif

        // =====================================================================
        // BARE-METAL UART SETUP
        // Uses periph_module_enable + uart_param_config WITHOUT installing the
        // full UART driver. This avoids the stale interrupt allocation issue
        // that prevents ISR registration on ESP32-C6 when using driver
        // install/delete.
        // =====================================================================

        periph_module_enable(RS485_PERIPH_MODULE);

        uart_config_t uart_config = {
            .baud_rate = RS485_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };
        ESP_ERROR_CHECK(uart_param_config((uart_port_t)RS485_UART_NUM, &uart_config));

        ESP_ERROR_CHECK(uart_set_pin((uart_port_t)RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // Ensure RX pin has pullup for stable idle state
        gpio_set_pull_mode((gpio_num_t)RS485_RX_PIN, GPIO_PULLUP_ONLY);

        // Trigger interrupt on every byte for lowest latency
        uart_ll_set_rxfifo_full_thr(uartHw, 1);

        uart_ll_clr_intsts_mask(uartHw, UART_LL_INTR_MASK);
        uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

        ESP_ERROR_CHECK(esp_intr_alloc(uart_periph_signal[RS485_UART_NUM].irq,
                                        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
                                        uart_isr_handler, NULL, &uartIntrHandle));

        rs485State = STATE_SYNC;
        lastRxTime = esp_timer_get_time();
    }

    // ########################################################################
    //
    //    DRIVER MODE IMPLEMENTATION (fallback)
    //
    // ########################################################################

#else // !USE_ISR_MODE

    // ========================================================================
    // DE PIN CONTROL (driver mode - non-ISR)
    // ========================================================================

#if RS485_DE_PIN >= 0
    static inline void setDE(bool high) {
        gpio_set_level((gpio_num_t)RS485_DE_PIN, high ? 1 : 0);
    }
#else
    #define setDE(x)
#endif

    // ========================================================================
    // TX FUNCTIONS - Send response using ESP-IDF UART driver
    // ========================================================================

    static void sendResponse() {
        uint8_t len = messageBuffer.getLength();

        // Build response packet
        uint8_t txBuf[MESSAGE_BUFFER_SIZE + 4];
        uint8_t txLen = 0;

        txBuf[txLen++] = len;        // Length byte
        txBuf[txLen++] = 0;          // MsgType = 0

        for (uint8_t i = 0; i < len; i++) {
            txBuf[txLen++] = messageBuffer.get();
        }

        txBuf[txLen++] = 0x72;       // Checksum

        // Enable DE for transmission
#if RS485_DE_PIN >= 0
        setDE(true);
        ets_delay_us(TX_WARMUP_DELAY_MANUAL_US);
#else
        ets_delay_us(TX_WARMUP_DELAY_AUTO_US);
#endif

        // Send via driver
        uart_write_bytes((uart_port_t)RS485_UART_NUM, (const char*)txBuf, txLen);
        uart_wait_tx_done((uart_port_t)RS485_UART_NUM, pdMS_TO_TICKS(10));

        // Release DE
        setDE(false);

        // Flush any echo bytes from RX buffer
        uart_flush_input((uart_port_t)RS485_UART_NUM);

        // Clear message buffer
        messageBuffer.clear();
        messageBuffer.complete = false;

        rs485State = STATE_RX_WAIT_ADDRESS;
    }

    static void sendZeroLengthResponse() {
        uint8_t zero = 0;

        // Enable DE for transmission
#if RS485_DE_PIN >= 0
        setDE(true);
        ets_delay_us(TX_WARMUP_DELAY_MANUAL_US);
#else
        ets_delay_us(TX_WARMUP_DELAY_AUTO_US);
#endif

        // Send single zero byte
        uart_write_bytes((uart_port_t)RS485_UART_NUM, (const char*)&zero, 1);
        uart_wait_tx_done((uart_port_t)RS485_UART_NUM, pdMS_TO_TICKS(10));

        // Release DE
        setDE(false);

        // Flush any echo bytes
        uart_flush_input((uart_port_t)RS485_UART_NUM);

        rs485State = STATE_RX_WAIT_ADDRESS;
    }

    // ========================================================================
    // RS485 RX BYTE PROCESSOR - called from main loop (driver mode)
    // ========================================================================

    static void processRxByte(uint8_t c) {
        int64_t now = esp_timer_get_time();
        rxByteCount++;

        // Sync detection - if gap > 500us, reset to wait for address
        if (rs485State == STATE_SYNC) {
            if ((now - lastRxTime) >= SYNC_TIMEOUT_US) {
                rs485State = STATE_RX_WAIT_ADDRESS;
                // Fall through to process this byte as address
            } else {
                lastRxTime = now;
                return;  // Stay in sync, discard byte
            }
        }

        switch (rs485State) {
            case STATE_RX_WAIT_ADDRESS:
                rxSlaveAddress = c;
                rs485State = STATE_RX_WAIT_MSGTYPE;
                break;

            case STATE_RX_WAIT_MSGTYPE:
                rxMsgType = c;
                rs485State = STATE_RX_WAIT_DATALENGTH;
                break;

            case STATE_RX_WAIT_DATALENGTH:
                rxtxLen = c;

                if (rxtxLen == 0) {
                    if (rxSlaveAddress == 0) {
                        rs485State = STATE_RX_WAIT_ADDRESS;
                    } else if (rxSlaveAddress == DCSBIOS_RS485_SLAVE) {
                        // Poll for us - respond!
                        if (rxMsgType == 0) {
                            if (messageBuffer.complete) {
                                sendResponse();
                            } else {
                                sendZeroLengthResponse();
                            }
                        } else {
                            rs485State = STATE_SYNC;
                        }
                    } else {
                        rs485State = STATE_RX_WAIT_ANSWER_DATALENGTH;
                    }
                } else {
                    if (rxSlaveAddress == 0 && rxMsgType == 0) {
                        rxDataType = RXDATA_DCSBIOS_EXPORT;
                    } else {
                        rxDataType = RXDATA_IGNORE;
                    }
                    rs485State = STATE_RX_WAIT_DATA;
                }
                break;

            case STATE_RX_WAIT_DATA:
                rxtxLen--;
                if (rxDataType == RXDATA_DCSBIOS_EXPORT) {
                    if (exportBufferAvailableForWrite() == 0) {
                        handleExportOverflow(now);
                        break;
                    }
                    uint8_t nextPos = (exportWritePos + 1) % EXPORT_BUFFER_SIZE;
                    exportBuffer[exportWritePos] = c;
                    exportWritePos = nextPos;
                }
                if (rxtxLen == 0) {
                    rs485State = STATE_RX_WAIT_CHECKSUM;
                }
                break;

            case STATE_RX_WAIT_CHECKSUM:
                if (rxSlaveAddress == 0) {
                    rs485State = STATE_RX_WAIT_ADDRESS;
                } else if (rxSlaveAddress == DCSBIOS_RS485_SLAVE) {
                    if (rxMsgType == 0) {
                        if (messageBuffer.complete) {
                            sendResponse();
                        } else {
                            sendZeroLengthResponse();
                        }
                    } else {
                        rs485State = STATE_SYNC;
                    }
                } else {
                    rs485State = STATE_RX_WAIT_ANSWER_DATALENGTH;
                }
                break;

            case STATE_RX_WAIT_ANSWER_DATALENGTH:
                rxtxLen = c;
                if (rxtxLen == 0) {
                    rs485State = STATE_RX_WAIT_ADDRESS;
                } else {
                    rs485State = STATE_RX_WAIT_ANSWER_MSGTYPE;
                }
                break;

            case STATE_RX_WAIT_ANSWER_MSGTYPE:
                rs485State = STATE_RX_WAIT_ANSWER_DATA;
                break;

            case STATE_RX_WAIT_ANSWER_DATA:
                rxtxLen--;
                if (rxtxLen == 0) {
                    rs485State = STATE_RX_WAIT_ANSWER_CHECKSUM;
                }
                break;

            case STATE_RX_WAIT_ANSWER_CHECKSUM:
                rs485State = STATE_RX_WAIT_ADDRESS;
                break;

            default:
                rs485State = STATE_SYNC;
                break;
        }

        lastRxTime = now;
    }

    // ========================================================================
    // DRIVER MODE - HARDWARE INITIALIZATION
    // ========================================================================

    static void initRS485Hardware() {
        // =====================================================================
        // GPIO: DE pin configuration
        // =====================================================================
#if RS485_DE_PIN >= 0
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << RS485_DE_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        setDE(false);  // Start in RX mode
#endif

        // =====================================================================
        // ESP-IDF UART DRIVER SETUP - Portable across ALL ESP32 variants
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

        // Install driver with RX buffer (256) and TX buffer (256)
        ESP_ERROR_CHECK(uart_driver_install((uart_port_t)RS485_UART_NUM, 256, 256, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config((uart_port_t)RS485_UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin((uart_port_t)RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // Flush any stale data
        uart_flush_input((uart_port_t)RS485_UART_NUM);

        rs485State = STATE_SYNC;
        lastRxTime = esp_timer_get_time();
    }

#endif // USE_ISR_MODE

    // ========================================================================
    // PROCESS EXPORT DATA (called from main loop - both modes)
    // ========================================================================

    static void processExportData() {
        while (exportReadPos != exportWritePos) {
            uint8_t c = exportBuffer[exportReadPos];
            exportReadPos = (exportReadPos + 1) % EXPORT_BUFFER_SIZE;
            parser.processChar(c);
        }
    }

    // ========================================================================
    // DCSBIOS SETUP AND LOOP
    // ========================================================================

    void setup() {
        initRS485Hardware();
    }

    void loop() {
#if !USE_ISR_MODE
        // Driver mode: read bytes from UART driver and process through state machine
        {
            uint8_t rxBuf[64];
            int len = uart_read_bytes((uart_port_t)RS485_UART_NUM, rxBuf, sizeof(rxBuf), 0);
            for (int i = 0; i < len; i++) {
                processRxByte(rxBuf[i]);
            }
        }
#endif

        // Process export data queued by ISR/state machine (for LED updates etc)
        processExportData();

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
