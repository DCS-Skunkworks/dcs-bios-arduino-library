#ifdef DCSBIOS_RS485_MASTER
#ifdef ARDUINO_ARCH_ESP32

#include "DcsBiosESP32RS485Master.h"

namespace DcsBios {

    // ========================================================================
    // STATIC MEMBER INITIALIZATION
    // ========================================================================

    RS485Master* RS485Master::first = nullptr;

    // ========================================================================
    // PC RX RING BUFFER - uint16_t indices for buffers > 256 bytes
    // ========================================================================
    // The library's RingBuffer uses uint8_t positions (max SIZE=256).
    // The PC RX buffer needs 1024 bytes, so we use a dedicated template
    // with uint16_t positions. Same SPSC lock-free pattern.

    template<unsigned int SIZE>
    class PCRingBuffer {
        volatile uint8_t buffer[SIZE];
        volatile uint16_t writepos;
        volatile uint16_t readpos;
    public:
        PCRingBuffer() : writepos(0), readpos(0) {}
        inline void put(uint8_t c) { buffer[writepos] = c; writepos = (writepos + 1) % SIZE; }
        inline uint8_t get() { uint8_t ret = buffer[readpos]; readpos = (readpos + 1) % SIZE; return ret; }
        inline bool isEmpty() const { return readpos == writepos; }
        inline bool isNotEmpty() const { return readpos != writepos; }
        inline uint16_t getLength() const { return (uint16_t)(writepos - readpos) % SIZE; }
        inline void clear() { readpos = writepos = 0; }
        inline uint16_t availableForWrite() const { return SIZE - getLength() - 1; }
    };

    // ========================================================================
    // PC INPUT TASK - Dedicated FreeRTOS task for PC Serial reception
    // ========================================================================
    // Solves: loop() blocks during broadcast TX (up to 2.5ms per chunk).
    // During that time, PC bytes buffer in USB CDC. This task ensures they
    // are captured continuously into pcRxBuffer, regardless of TX blocking.
    //
    // Single-writer (this task), single-reader (main loop) is naturally
    // lock-free with volatile positions. On ESP32, volatile DRAM accesses
    // are coherent across cores.

    static PCRingBuffer<PC_RX_BUFFER_SIZE> pcRxBuffer;
    static TaskHandle_t pcInputTaskHandle = nullptr;

    static void pcInputTaskFunc(void* param) {
        (void)param;
        for (;;) {
            // Drain all available bytes from USB CDC into pcRxBuffer
            while (PC_SERIAL.available()) {
                if (pcRxBuffer.availableForWrite() > 0) {
                    pcRxBuffer.put((uint8_t)PC_SERIAL.read());
                } else {
                    // Buffer full - discard to prevent USB CDC overflow
                    // Should never happen with 1024 bytes, but safety first
                    PC_SERIAL.read();
                }
            }
            // Yield for 1 tick (~1ms on ESP32 Arduino, configTICK_RATE_HZ=1000)
            // Matches USB Full-Speed 1ms frame rate
            vTaskDelay(1);
        }
    }

    static void startPCInputTask() {
#if CONFIG_FREERTOS_UNICORE
        // Single-core (C3, C6, S2, H2): can't pin, use regular xTaskCreate
        // FreeRTOS preemptive scheduling ensures this task runs between loop() calls
        xTaskCreate(
            pcInputTaskFunc,
            "PCInput",
            2048,               // Minimal stack - just Serial.read() + RingBuffer.put()
            nullptr,
            5,                  // Priority 5: preempts Arduino loop() (priority 1)
            &pcInputTaskHandle
        );
#else
        // Dual-core (ESP32, S3): pin to Core 0 where USB/Serial stack has affinity
        xTaskCreatePinnedToCore(
            pcInputTaskFunc,
            "PCInput",
            2048,
            nullptr,
            5,
            &pcInputTaskHandle,
            0                   // Core 0
        );
#endif
    }

    // ========================================================================
    // UART ISR HANDLER - shared function, bus identified via void* arg
    // ========================================================================

    static void IRAM_ATTR masterUartIsrHandler(void* arg) {
        RS485Master* bus = (RS485Master*)arg;
        bus->rxISR();
    }

    // ========================================================================
    // RS485 MASTER - CONSTRUCTOR
    // ========================================================================

    RS485Master::RS485Master(int uartNum, int txPin, int rxPin, int dePin)
        : uartNum(uartNum), txPin(txPin), rxPin(rxPin), dePin(dePin),
          uartHw(nullptr), uartIntrHandle(nullptr),
          state(MASTER_STATE_IDLE), rxStartTime(0), rxtxLen(0), rxMsgType(0),
          lastPollTime(0),
          pollAddressCounter(MIN_SLAVE_ADDRESS), scanAddressCounter(MIN_SLAVE_ADDRESS),
          currentPollAddress(MIN_SLAVE_ADDRESS),
          next(nullptr)
    {
        // Add to linked list
        if (first == nullptr) {
            first = this;
        } else {
            RS485Master* p = first;
            while (p->next != nullptr) p = p->next;
            p->next = this;
        }

        // Initialize slave tracking
        slavePresent[0] = true;  // Address 0 is broadcast sentinel
        for (int i = 1; i < MAX_SLAVES; i++) {
            slavePresent[i] = false;
        }
    }

    // ========================================================================
    // DE PIN CONTROL
    // ========================================================================

    inline void RS485Master::setDE(bool high) {
        if (dePin >= 0) {
            gpio_ll_set_level(&GPIO, (gpio_num_t)dePin, high ? 1 : 0);
        }
    }

    // ========================================================================
    // TX HELPERS - Inline blocking, called from loop() context
    // ========================================================================
    // Mirrors AVR's set_txen/clear_txen pattern:
    //   1. Disable RX interrupt (no echo enters state machine)
    //   2. Assert DE
    //   3. Write bytes to FIFO, wait for idle
    //   4. De-assert DE
    //   5. Flush RX FIFO (discard any echo)
    //   6. Re-enable RX interrupt

    inline void RS485Master::prepareForTransmit() {
        // Disable RX interrupt - equivalent to AVR's set_txen disabling RXEN
        uart_ll_disable_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

        // Assert DE pin (manual mode) or wait for auto-direction settling
        if (dePin >= 0) {
            setDE(true);
            ets_delay_us(TX_WARMUP_DELAY_MANUAL_US);
        } else {
            ets_delay_us(TX_WARMUP_DELAY_AUTO_US);
        }
    }

    inline void RS485Master::finishTransmit() {
        // Wait for all bytes to exit shift register
        while (!uart_ll_is_tx_idle(uartHw));

        // De-assert DE pin
        setDE(false);

        // Flush RX FIFO - discard any echo bytes
        uart_ll_rxfifo_rst(uartHw);

        // Re-enable RX interrupt - equivalent to AVR's clear_txen enabling RXEN
        uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
        uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
    }

    inline void RS485Master::txWriteBlocking(const uint8_t* data, uint16_t len) {
        uint16_t sent = 0;
        while (sent < len) {
            uint32_t space = uart_ll_get_txfifo_len(uartHw);
            if (space > 0) {
                uint16_t n = (uint16_t)((len - sent < space) ? (len - sent) : space);
                uart_ll_write_txfifo(uartHw, data + sent, n);
                sent += n;
            }
        }
    }

    // ========================================================================
    // init() - Bare-metal UART setup (from slave ISR-mode pattern)
    // ========================================================================

    void RS485Master::init() {
        // --- DE pin setup ---
        if (dePin >= 0) {
            gpio_config_t io_conf = {
                .pin_bit_mask = (1ULL << dePin),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            gpio_config(&io_conf);
            setDE(false);  // Start in RX mode
        }

        // --- Bare-metal UART setup ---
        // Map UART number to hardware register pointer
        periph_module_t periphModule;
        switch (uartNum) {
            case 0:
                uartHw = &UART0;
                periphModule = PERIPH_UART0_MODULE;
                break;
            case 1:
                uartHw = &UART1;
                periphModule = PERIPH_UART1_MODULE;
                break;
#if SOC_UART_NUM > 2
            case 2:
                uartHw = &UART2;
                periphModule = PERIPH_UART2_MODULE;
                break;
#endif
            default:
                return;
        }

        periph_module_enable(periphModule);

        uart_config_t uart_config = {
            .baud_rate = RS485_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };
        ESP_ERROR_CHECK(uart_param_config((uart_port_t)uartNum, &uart_config));

        ESP_ERROR_CHECK(uart_set_pin((uart_port_t)uartNum, txPin, rxPin,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // Ensure RX pin has pullup for stable idle state
        gpio_set_pull_mode((gpio_num_t)rxPin, GPIO_PULLUP_ONLY);

        // --- Configure RX interrupt ---
        // Trigger interrupt on every byte for lowest latency
        uart_ll_set_rxfifo_full_thr(uartHw, 1);

        uart_ll_clr_intsts_mask(uartHw, UART_LL_INTR_MASK);
        uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

        // --- Register ISR ---
        // Pass 'this' as arg so the ISR knows which bus triggered it
        ESP_ERROR_CHECK(esp_intr_alloc(uart_periph_signal[uartNum].irq,
                                        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
                                        masterUartIsrHandler, this, &uartIntrHandle));

        // Flush any stale data from FIFO (init only)
        uart_ll_rxfifo_rst(uartHw);

        state = MASTER_STATE_IDLE;
        lastPollTime = esp_timer_get_time();
    }

    // ========================================================================
    // advancePollAddress() - safe wraparound (fixes AVR infinite-loop bug)
    // ========================================================================

    void RS485Master::advancePollAddress() {
        pollAddressCounter++;
        if (pollAddressCounter > MAX_SLAVE_ADDRESS) {
            pollAddressCounter = MIN_SLAVE_ADDRESS;
        }

        // Skip to next known slave
        uint8_t startAddr = pollAddressCounter;
        while (!slavePresent[pollAddressCounter]) {
            pollAddressCounter++;
            if (pollAddressCounter > MAX_SLAVE_ADDRESS) {
                pollAddressCounter = MIN_SLAVE_ADDRESS;
            }
            if (pollAddressCounter == startAddr) {
                // No known slaves - scan for new ones
                scanAddressCounter++;
                if (scanAddressCounter > MAX_SLAVE_ADDRESS) {
                    scanAddressCounter = MIN_SLAVE_ADDRESS;
                }
                currentPollAddress = scanAddressCounter;
                return;
            }
        }
        currentPollAddress = pollAddressCounter;
    }

    // ========================================================================
    // TX FUNCTIONS - inline blocking (no FreeRTOS task/queue)
    // ========================================================================

    void RS485Master::sendPollFrame(uint8_t addr) {
        uint8_t frame[3] = { addr, 0x00, 0x00 };

        prepareForTransmit();
        txWriteBlocking(frame, sizeof(frame));
        finishTransmit();

        // Start RX timeout - equivalent to AVR's txcISR setting rx_start_time
        rxStartTime = esp_timer_get_time();
        state = MASTER_STATE_RX_WAIT_DATALENGTH;
    }

    void RS485Master::sendBroadcastFrame() {
        uint16_t available = exportData.getLength();
        if (available == 0) return;

        uint8_t dataLen = (uint8_t)((available > MAX_BROADCAST_CHUNK) ? MAX_BROADCAST_CHUNK : available);
        uint8_t frame[MAX_BROADCAST_CHUNK + 4];
        uint16_t idx = 0;

        frame[idx++] = 0x00;       // broadcast address
        frame[idx++] = 0x00;       // msg type
        frame[idx++] = dataLen;    // payload length

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < dataLen; i++) {
            uint8_t b = exportData.get();
            frame[idx++] = b;
            checksum ^= b;
        }
        frame[idx++] = checksum;

        prepareForTransmit();
        txWriteBlocking(frame, idx);
        finishTransmit();

        // Broadcast done, stay IDLE (no response expected)
    }

    void RS485Master::sendTimeoutZeroByte() {
        // AVR behavior: send zero byte on behalf of non-responding slave
        // to keep listening slaves synchronized with bus traffic
        uint8_t zero = 0x00;

        prepareForTransmit();
        txWriteBlocking(&zero, 1);
        finishTransmit();

        state = MASTER_STATE_IDLE;
    }

    // ========================================================================
    // rxISR() - called from UART ISR, processes RX state machine per byte
    // Mirrors AVR's RS485Master::rxISR() exactly
    // ========================================================================

    void IRAM_ATTR RS485Master::rxISR() {
        uint32_t uart_intr_status = uart_ll_get_intsts_mask(uartHw);

        while (uart_ll_get_rxfifo_len(uartHw) > 0) {
            uint8_t c;
            uart_ll_read_rxfifo(uartHw, &c, 1);
#ifdef __riscv
            // RISC-V (C3, C6, H2): memory fence ensures FIFO read pointer
            // update propagates before next FIFO length check
            __asm__ __volatile__("fence");
#endif

            switch (state) {
                case MASTER_STATE_RX_WAIT_DATALENGTH:
                    rxtxLen = c;
                    slavePresent[currentPollAddress] = true;
                    if (rxtxLen > 0) {
                        state = MASTER_STATE_RX_WAIT_MSGTYPE;
                    } else {
                        state = MASTER_STATE_IDLE;
                    }
                    rxStartTime = esp_timer_get_time();
                    break;

                case MASTER_STATE_RX_WAIT_MSGTYPE:
                    rxMsgType = c;
                    (void)rxMsgType;
                    state = MASTER_STATE_RX_WAIT_DATA;
                    rxStartTime = esp_timer_get_time();
                    break;

                case MASTER_STATE_RX_WAIT_DATA:
                    messageBuffer.put(c);
                    if (--rxtxLen == 0) {
                        state = MASTER_STATE_RX_WAIT_CHECKSUM;
                    }
                    rxStartTime = esp_timer_get_time();
                    break;

                case MASTER_STATE_RX_WAIT_CHECKSUM:
                    // Checksum intentionally ignored (AVR behavior)
                    messageBuffer.complete = true;
                    state = MASTER_STATE_IDLE;
                    rxStartTime = esp_timer_get_time();
                    break;

                default:
                    // Unexpected byte while IDLE - discard
                    break;
            }
        }

        uart_ll_clr_intsts_mask(uartHw, uart_intr_status);
    }

    // ========================================================================
    // loop() - main scheduler, called from Arduino loop()
    // Only handles: starting TX when IDLE, checking RX timeouts
    // Mirrors AVR's RS485Master::loop() exactly
    // ========================================================================

    void RS485Master::loop() {
        int64_t now = esp_timer_get_time();

        if (state == MASTER_STATE_IDLE) {
            // Priority 1: Send export data (broadcast) if available
            // But ensure we poll at least every MAX_POLL_INTERVAL_US
            if (exportData.isNotEmpty() && (now - lastPollTime) < MAX_POLL_INTERVAL_US) {
                sendBroadcastFrame();
                return;
            }

            // Priority 2: Poll a slave if message buffer is free
            if (messageBuffer.isEmpty() && !messageBuffer.complete) {
                advancePollAddress();
                sendPollFrame(currentPollAddress);
                lastPollTime = now;
            }
            return;
        }

        // Timeout: waiting for first response byte (1ms, matches AVR)
        if (state == MASTER_STATE_RX_WAIT_DATALENGTH) {
            if ((now - rxStartTime) > POLL_TIMEOUT_US) {
                slavePresent[currentPollAddress] = false;
                sendTimeoutZeroByte();
                return;
            }
        }

        // Timeout: mid-message (5ms, matches AVR)
        if (state == MASTER_STATE_RX_WAIT_MSGTYPE ||
            state == MASTER_STATE_RX_WAIT_DATA ||
            state == MASTER_STATE_RX_WAIT_CHECKSUM) {
            if ((now - rxStartTime) > RX_TIMEOUT_US) {
                // AVR-compatible: send newline marker to PC stream on timeout
                messageBuffer.clear();
                messageBuffer.put('\n');
                messageBuffer.complete = true;
                state = MASTER_STATE_IDLE;

                // Flush any remaining bytes from FIFO
                uart_ll_rxfifo_rst(uartHw);
                uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
                uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
                return;
            }
        }
    }

    // ========================================================================
    // BUS INSTANCES - Only enabled buses are instantiated
    // ========================================================================

#if RS485_BUS1_ENABLED
    RS485Master bus1(RS485_BUS1_UART_NUM, RS485_BUS1_TX_PIN, RS485_BUS1_RX_PIN, RS485_BUS1_DE_PIN);
#endif

#if RS485_BUS2_ENABLED
    RS485Master bus2(RS485_BUS2_UART_NUM, RS485_BUS2_TX_PIN, RS485_BUS2_RX_PIN, RS485_BUS2_DE_PIN);
#endif

#if RS485_BUS3_ENABLED
    RS485Master bus3(RS485_BUS3_UART_NUM, RS485_BUS3_TX_PIN, RS485_BUS3_RX_PIN, RS485_BUS3_DE_PIN);
#endif

    // ========================================================================
    // PC COMMUNICATION - Aggregates all buses
    // ========================================================================

    /**
     * drainPCInput() - Drain PC RX buffer into ALL bus export buffers
     * Reads from pcRxBuffer (filled by PCInputTask) instead of Serial directly.
     * This decouples PC data capture from the main loop's TX blocking.
     */
    static void drainPCInput() {
        while (pcRxBuffer.isNotEmpty()) {
            uint8_t c = pcRxBuffer.get();

            // Send to all enabled buses
            RS485Master* bus = RS485Master::first;
            while (bus != nullptr) {
                if (bus->exportData.availableForWrite() > 0) {
                    bus->exportData.put(c);
                }
                bus = bus->next;
            }
        }
    }

    /**
     * sendToPC() - Forward slave responses from ANY bus to PC
     */
    static void sendToPC() {
        RS485Master* bus = RS485Master::first;
        while (bus != nullptr) {
            if (bus->messageBuffer.complete) {
                while (bus->messageBuffer.isNotEmpty()) {
                    PC_SERIAL.write(bus->messageBuffer.get());
                }
                bus->messageBuffer.complete = false;
            }
            bus = bus->next;
        }
    }

    // ========================================================================
    // DCSBIOS SETUP AND LOOP
    // ========================================================================

    void setup() {
        // Initialize USB CDC for PC communication
        PC_SERIAL.begin(250000);

        // Start PC Serial input task (captures PC bytes even during TX blocking)
        startPCInputTask();

        // Initialize all enabled buses
        RS485Master* bus = RS485Master::first;
        while (bus != nullptr) {
            bus->init();
            bus = bus->next;
        }
    }

    void loop() {
        // Drain PC data from pcRxBuffer into bus export buffers
        drainPCInput();

        // Forward any pending slave responses to PC (catch-all pass)
        sendToPC();

        // Process each bus's state machine
        RS485Master* bus = RS485Master::first;
        while (bus != nullptr) {
            bus->loop();

            // INLINE FORWARDING: if this bus just completed a slave
            // response, forward it to PC right now instead of waiting for
            // the next loop() iteration. Reduces slave->PC latency by one
            // full cycle.
            if (bus->messageBuffer.complete) {
                while (bus->messageBuffer.isNotEmpty()) {
                    PC_SERIAL.write(bus->messageBuffer.get());
                }
                bus->messageBuffer.complete = false;
            }

            bus = bus->next;
        }
    }

}  // namespace DcsBios

#endif // ARDUINO_ARCH_ESP32
#endif // DCSBIOS_RS485_MASTER
