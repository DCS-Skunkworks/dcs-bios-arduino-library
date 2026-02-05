/**
 * ESP32 RS485 SLAVE - INTERRUPT-DRIVEN (AVR-style)
 *
 * Key design:
 * 1. UART RX interrupt fires immediately when byte arrives
 * 2. State machine runs IN the ISR - no polling latency
 * 3. Response sent immediately from ISR when poll detected
 * 4. This matches AVR's RXC interrupt behavior exactly
 *
 * The main loop only handles non-time-critical tasks:
 * - Input polling (switches, buttons)
 * - Export data parsing (LED updates)
 */

#define SLAVE_ADDRESS 1

// Pin Configuration
#define RS485_TX_PIN    19
#define RS485_RX_PIN    18
#define RS485_DE_PIN    -1    // Set to -1 for auto-direction transceivers

// UART Configuration
#define RS485_UART_NUM  1
#define RS485_BAUD_RATE 250000

// ============================================================================
// CLOCK SOURCE SELECTION
// ============================================================================
// Uses UART_SCLK_DEFAULT with getApbFrequency() for portability across all ESP32 variants
// (ESP32, S2, S3, C3, C6 - works regardless of actual APB clock frequency)

// Buffer Sizes
#define MESSAGE_BUFFER_SIZE    64

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
#define SYNC_TIMEOUT_US      500    // 500µs silence = sync detected

// TX Warm-up delays in MICROSECONDS (portable across all ESP32 variants)
// These give the transceiver time to switch to TX mode before data is sent
#define TX_WARMUP_DELAY_MANUAL_US    50    // Manual DE: wait after DE asserted
#define TX_WARMUP_DELAY_AUTO_US      50    // Auto-direction: wait for RX→TX switch

// ============================================================================
// TX MODE SELECTION
// ============================================================================
// 0 = Buffered mode: Build response in buffer, write all at once to FIFO
// 1 = Byte-by-byte mode: Write each byte individually, wait for TX idle
//     (default - mimics AVR behavior, better pacing with FreeRTOS tasks)
#define TX_MODE_BYTE_BY_BYTE    1

// ============================================================================
// DEBUG OPTIONS
// ============================================================================
#define UDP_DEBUG_ENABLE    1
#define WIFI_SSID           "TestNetwork"
#define WIFI_PASSWORD       "TestingOnly"

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <hal/uart_ll.h>
#include <rom/ets_sys.h>        // For ets_delay_us() - portable across all ESP32 variants
#include <hal/gpio_ll.h>
#include <soc/uart_struct.h>
#include <soc/gpio_struct.h>
#include <soc/uart_periph.h>
#include <soc/soc_caps.h>       // For SOC_GPIO_PIN_COUNT - chip-specific GPIO count
#include <esp_timer.h>
#include <esp_intr_alloc.h>
#include <esp_rom_gpio.h>

#if UDP_DEBUG_ENABLE
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP udpDbg;
static IPAddress udpDbgTarget(255, 255, 255, 255);
static bool udpDbgConnected = false;
static char udpDbgBuf[256];

void udpDbgInit() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void udpDbgCheck() {
    if (!udpDbgConnected && WiFi.status() == WL_CONNECTED) {
        udpDbgConnected = true;
        udpDbgSend("=== SLAVE %d ONLINE (ISR MODE) === IP=%s", SLAVE_ADDRESS, WiFi.localIP().toString().c_str());
    }
}

void udpDbgSend(const char* fmt, ...) {
    if (!udpDbgConnected) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(udpDbgBuf, sizeof(udpDbgBuf), fmt, args);
    va_end(args);
    udpDbg.beginPacket(udpDbgTarget, 4210);
    udpDbg.write((uint8_t*)udpDbgBuf, strlen(udpDbgBuf));
    udpDbg.endPacket();
}
#else
#define udpDbgInit()
#define udpDbgCheck()
#define udpDbgSend(...)
#endif

#if SLAVE_ADDRESS < 1 || SLAVE_ADDRESS > 127
    #error "SLAVE_ADDRESS must be between 1 and 127"
#endif

// ============================================================================
// DCS-BIOS PROTOCOL PARSER (for export data - runs in main loop)
// ============================================================================

#define DCSBIOS_STATE_WAIT_FOR_SYNC  0
#define DCSBIOS_STATE_ADDRESS_LOW    1
#define DCSBIOS_STATE_ADDRESS_HIGH   2
#define DCSBIOS_STATE_COUNT_LOW      3
#define DCSBIOS_STATE_COUNT_HIGH     4
#define DCSBIOS_STATE_DATA_LOW       5
#define DCSBIOS_STATE_DATA_HIGH      6

class ExportStreamListener;
extern ExportStreamListener* firstExportStreamListener;

class ExportStreamListener {
protected:
    uint16_t firstAddressOfInterest;
    uint16_t lastAddressOfInterest;

public:
    ExportStreamListener* nextExportStreamListener;

    ExportStreamListener(uint16_t firstAddr, uint16_t lastAddr) {
        firstAddressOfInterest = firstAddr & ~0x01;
        lastAddressOfInterest = lastAddr & ~0x01;

        if (firstExportStreamListener == nullptr) {
            firstExportStreamListener = this;
            nextExportStreamListener = nullptr;
            return;
        }

        ExportStreamListener** prevNextPtr = &firstExportStreamListener;
        ExportStreamListener* nextESL = firstExportStreamListener;

        while (nextESL &&
               ((nextESL->lastAddressOfInterest < lastAddressOfInterest) ||
                (nextESL->lastAddressOfInterest == lastAddressOfInterest &&
                 nextESL->firstAddressOfInterest < firstAddressOfInterest))) {
            prevNextPtr = &(nextESL->nextExportStreamListener);
            nextESL = nextESL->nextExportStreamListener;
        }

        nextExportStreamListener = nextESL;
        *prevNextPtr = this;
    }

    inline uint16_t getFirstAddressOfInterest() { return firstAddressOfInterest; }
    inline uint16_t getLastAddressOfInterest() { return lastAddressOfInterest; }

    virtual void onDcsBiosWrite(unsigned int address, unsigned int value) {}
    virtual void onConsistentData() {}
    virtual void loop() {}

    static void loopAll() {
        ExportStreamListener* el = firstExportStreamListener;
        while (el) {
            el->loop();
            el = el->nextExportStreamListener;
        }
    }
};

ExportStreamListener* firstExportStreamListener = nullptr;

class IntegerBuffer : public ExportStreamListener {
private:
    volatile uint16_t data;
    uint16_t mask;
    uint8_t shift;
    void (*callback)(unsigned int);
    volatile bool dirty;
    volatile bool receivedData;

public:
    IntegerBuffer(uint16_t address, uint16_t mask, uint8_t shift, void (*callback)(unsigned int))
        : ExportStreamListener(address, address), mask(mask), shift(shift),
          callback(callback), data(0), dirty(false), receivedData(false) {}

    virtual void onDcsBiosWrite(unsigned int address, unsigned int value) override {
        uint16_t newValue = (value & mask) >> shift;
        if (!receivedData || data != newValue) {
            data = newValue;
            dirty = true;
            receivedData = true;
        }
    }

    virtual void loop() override {
        if (dirty && callback) {
            dirty = false;
            callback(data);
        }
    }

    uint16_t getData() {
        dirty = false;
        return data;
    }

    bool hasUpdatedData() { return dirty; }
};

template<unsigned int LENGTH>
class StringBuffer : public ExportStreamListener {
private:
    char receivingBuffer[LENGTH + 1];
    char userBuffer[LENGTH + 1];
    volatile bool receivingDirty;
    bool userDirty;
    void (*callback)(char*);

    void setChar(unsigned int index, unsigned char value) {
        if (receivingBuffer[index] == value) return;
        receivingBuffer[index] = value;
        receivingDirty = true;
    }

public:
    StringBuffer(uint16_t address, void (*callback)(char*))
        : ExportStreamListener(address, address + LENGTH), callback(callback),
          receivingDirty(false), userDirty(false) {
        memset(receivingBuffer, ' ', LENGTH);
        receivingBuffer[LENGTH] = '\0';
        userBuffer[LENGTH] = '\0';
    }

    virtual void onDcsBiosWrite(unsigned int address, unsigned int data) override {
        unsigned int index = address - firstAddressOfInterest;
        setChar(index, ((char*)&data)[0]);
        if (LENGTH > index + 1) {
            setChar(index + 1, ((char*)&data)[1]);
        }
    }

    virtual void onConsistentData() override {
        if (receivingDirty) {
            memcpy(userBuffer, receivingBuffer, LENGTH);
            receivingDirty = false;
            userDirty = true;
        }
    }

    virtual void loop() override {
        if (userDirty && callback) {
            userDirty = false;
            callback(userBuffer);
        }
    }

    char* getData() {
        userDirty = false;
        return userBuffer;
    }

    bool hasUpdatedData() { return userDirty; }
};

// Parser instance - processes export data queued by ISR
class ProtocolParser {
private:
    uint8_t state;
    uint16_t address;
    uint16_t count;
    uint16_t data;
    uint8_t sync_byte_count;
    ExportStreamListener* startESL;

public:
    ProtocolParser() : state(DCSBIOS_STATE_WAIT_FOR_SYNC), sync_byte_count(0), startESL(nullptr) {}

    void processChar(uint8_t c) {
        switch (state) {
            case DCSBIOS_STATE_WAIT_FOR_SYNC:
                break;

            case DCSBIOS_STATE_ADDRESS_LOW:
                address = c;
                state = DCSBIOS_STATE_ADDRESS_HIGH;
                break;

            case DCSBIOS_STATE_ADDRESS_HIGH:
                address |= (c << 8);
                if (address != 0x5555) {
                    state = DCSBIOS_STATE_COUNT_LOW;
                } else {
                    state = DCSBIOS_STATE_WAIT_FOR_SYNC;
                }
                break;

            case DCSBIOS_STATE_COUNT_LOW:
                count = c;
                state = DCSBIOS_STATE_COUNT_HIGH;
                break;

            case DCSBIOS_STATE_COUNT_HIGH:
                count |= (c << 8);
                state = DCSBIOS_STATE_DATA_LOW;
                break;

            case DCSBIOS_STATE_DATA_LOW:
                data = c;
                count--;
                state = DCSBIOS_STATE_DATA_HIGH;
                break;

            case DCSBIOS_STATE_DATA_HIGH:
                data |= (c << 8);
                count--;

                while (startESL && startESL->getLastAddressOfInterest() < address) {
                    startESL->onConsistentData();
                    startESL = startESL->nextExportStreamListener;
                }

                if (startESL) {
                    ExportStreamListener* el = startESL;
                    while (el) {
                        if (el->getFirstAddressOfInterest() > address) break;
                        if (el->getFirstAddressOfInterest() <= address &&
                            el->getLastAddressOfInterest() >= address) {
                            el->onDcsBiosWrite(address, data);
                        }
                        el = el->nextExportStreamListener;
                    }
                }

                address += 2;
                state = (count == 0) ? DCSBIOS_STATE_ADDRESS_LOW : DCSBIOS_STATE_DATA_LOW;
                break;
        }

        if (c == 0x55) {
            sync_byte_count++;
        } else {
            sync_byte_count = 0;
        }

        if (sync_byte_count == 4) {
            state = DCSBIOS_STATE_ADDRESS_LOW;
            sync_byte_count = 0;
            startESL = firstExportStreamListener;
        }
    }
};

// ============================================================================
// INPUT POLLING SYSTEM
// ============================================================================

static volatile bool messageSentOrQueued = false;

class PollingInput {
private:
    virtual void resetState() = 0;
    virtual void pollInput() = 0;
    unsigned long pollingIntervalMs;
    unsigned long lastPollTime;
    PollingInput* nextPollingInput;

public:
    static PollingInput* firstPollingInput;

    static void setMessageSentOrQueued() { messageSentOrQueued = true; }

    PollingInput(unsigned long pollIntervalMs = 0)
        : pollingIntervalMs(pollIntervalMs), lastPollTime(0) {
        nextPollingInput = firstPollingInput;
        firstPollingInput = this;
    }

    static void pollInputs() {
        PollingInput* pi = firstPollingInput;
        if (!pi) return;

        unsigned long now = millis();
        PollingInput* lastSender = nullptr;

        do {
            messageSentOrQueued = false;

            if (now - pi->lastPollTime >= pi->pollingIntervalMs) {
                pi->pollInput();
                if (messageSentOrQueued) {
                    lastSender = pi;
                }
                pi->lastPollTime = now;
            }

            if (pi->nextPollingInput == nullptr) {
                pi->nextPollingInput = firstPollingInput;
            }

            pi = pi->nextPollingInput;
        } while (pi != firstPollingInput);

        if (lastSender && (firstPollingInput != pi)) {
            firstPollingInput = lastSender->nextPollingInput;
        }
    }

    static void resetAllStates() {
        PollingInput* pi = firstPollingInput;
        if (!pi) return;

        do {
            pi->resetState();
            pi = pi->nextPollingInput;
        } while (pi != firstPollingInput);
    }
};

PollingInput* PollingInput::firstPollingInput = nullptr;

// ============================================================================
// RING BUFFER (lock-free for ISR/main communication)
// ============================================================================

template<unsigned int SIZE>
class RingBuffer {
private:
    volatile uint8_t buffer[SIZE];
    volatile uint8_t writepos;
    volatile uint8_t readpos;

public:
    volatile bool complete;

    RingBuffer() : writepos(0), readpos(0), complete(false) {}

    // Called from main loop only
    void put(uint8_t c) {
        buffer[writepos] = c;
        writepos = (writepos + 1) % SIZE;
    }

    // Called from ISR - get for transmission
    uint8_t IRAM_ATTR getISR() {
        uint8_t ret = buffer[readpos];
        readpos = (readpos + 1) % SIZE;
        return ret;
    }

    uint8_t IRAM_ATTR getLengthISR() { return (writepos - readpos) % SIZE; }
    bool IRAM_ATTR isCompleteISR() { return complete; }

    void IRAM_ATTR clearISR() {
        readpos = 0;
        writepos = 0;
        complete = false;
    }

    bool isEmpty() { return readpos == writepos; }
    void clear() { readpos = 0; writepos = 0; }
};

// ============================================================================
// GLOBAL INSTANCES
// ============================================================================

static ProtocolParser parser;
static RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

// Export data buffer - ISR queues bytes here, main loop processes
#define EXPORT_BUFFER_SIZE 256
static volatile uint8_t exportBuffer[EXPORT_BUFFER_SIZE];
static volatile uint8_t exportWritePos = 0;
static volatile uint8_t exportReadPos = 0;

bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
    if (messageBuffer.complete) return false;

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

// ============================================================================
// RS485 STATE MACHINE - RUNS IN ISR!
// ============================================================================

enum RS485State {
    STATE_SYNC,
    STATE_RX_WAIT_ADDRESS,
    STATE_RX_WAIT_MSGTYPE,
    STATE_RX_WAIT_DATALENGTH,
    STATE_RX_WAIT_DATA,
    STATE_RX_WAIT_CHECKSUM,
    STATE_RX_WAIT_ANSWER_DATALENGTH,
    STATE_RX_WAIT_ANSWER_MSGTYPE,
    STATE_RX_WAIT_ANSWER_DATA,
    STATE_RX_WAIT_ANSWER_CHECKSUM
};

enum RxDataType {
    RXDATA_IGNORE,
    RXDATA_DCSBIOS_EXPORT
};

// All volatile for ISR access
static volatile RS485State rs485State = STATE_SYNC;
static volatile uint8_t rxSlaveAddress = 0;
static volatile uint8_t rxMsgType = 0;
static volatile uint8_t rxtxLen = 0;
static volatile RxDataType rxDataType = RXDATA_IGNORE;
static volatile int64_t lastRxTime = 0;
static volatile uint32_t isrByteCount = 0;      // DEBUG: count bytes received by ISR
static volatile uint32_t isrTriggerCount = 0;   // DEBUG: count ISR invocations

// Direct hardware pointers for ISR
static uart_dev_t* const uartHw = &UART1;
static intr_handle_t uartIntrHandle;

// ============================================================================
// DIRECT GPIO CONTROL (faster than gpio_set_level in ISR)
// ============================================================================

#if RS485_DE_PIN >= 0
static inline void IRAM_ATTR setDE_ISR(bool high) {
    if (high) {
        GPIO.out_w1ts = (1ULL << RS485_DE_PIN);
    } else {
        GPIO.out_w1tc = (1ULL << RS485_DE_PIN);
    }
}
#else
#define setDE_ISR(x)
#endif

// ============================================================================
// TX FROM ISR - Send response immediately!
// ============================================================================

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
    uint8_t len = messageBuffer.getLengthISR();

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
        txByteWaitIdle(messageBuffer.getISR());
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
        txBuf[txLen++] = messageBuffer.getISR();
    }

    txBuf[txLen++] = 0x72;       // Checksum

    // Send entire buffer at once - FIFO is 128 bytes, plenty for our messages
    uart_ll_write_txfifo(uartHw, txBuf, txLen);
#endif

    // Wait for transmission to fully complete
    while (!uart_ll_is_tx_idle(uartHw));

    // Disable driver
    setDE_ISR(false);

    // Flush RX FIFO (echo bytes)
    uart_ll_rxfifo_rst(uartHw);

    // Re-enable RX interrupt
    uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
    uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

    // Clear message buffer
    messageBuffer.clearISR();

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

    // Disable driver
    setDE_ISR(false);

    // Flush RX FIFO (echo)
    uart_ll_rxfifo_rst(uartHw);

    // Re-enable RX interrupt
    uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
    uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

    rs485State = STATE_RX_WAIT_ADDRESS;
}

// ============================================================================
// UART RX ISR - This is where the magic happens!
// Fires immediately when byte arrives, processes state machine, responds instantly
// ============================================================================

static void IRAM_ATTR uart_isr_handler(void *arg) {
    uint32_t uart_intr_status = uart_ll_get_intsts_mask(uartHw);
    isrTriggerCount++;  // DEBUG: count ISR invocations

    // Process all available bytes
    while (uart_ll_get_rxfifo_len(uartHw) > 0) {
        uint8_t c;
        uart_ll_read_rxfifo(uartHw, &c, 1);
        isrByteCount++;  // DEBUG: count bytes

        int64_t now = esp_timer_get_time();

        // Sync detection - if gap > 500µs, reset to wait for address
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
                    } else if (rxSlaveAddress == SLAVE_ADDRESS) {
                        // Poll for us! Respond IMMEDIATELY!
                        if (rxMsgType == 0) {
                            if (messageBuffer.isCompleteISR()) {
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
                    uint8_t nextPos = (exportWritePos + 1) % EXPORT_BUFFER_SIZE;
                    if (nextPos != exportReadPos) {  // Not full
                        exportBuffer[exportWritePos] = c;
                        exportWritePos = nextPos;
                    }
                }

                if (rxtxLen == 0) {
                    rs485State = STATE_RX_WAIT_CHECKSUM;
                }
                break;

            case STATE_RX_WAIT_CHECKSUM:
                // Message complete
                if (rxSlaveAddress == 0) {
                    rs485State = STATE_RX_WAIT_ADDRESS;
                } else if (rxSlaveAddress == SLAVE_ADDRESS) {
                    // This was addressed to us with data - respond
                    if (rxMsgType == 0) {
                        if (messageBuffer.isCompleteISR()) {
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

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

static void initRS485Hardware() {
    Serial.println("  [1] Configuring DE GPIO pin...");
    // =========================================================================
    // GPIO: DE pin for RS485 direction control
    // =========================================================================
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
    Serial.println("  [1] DE GPIO configured OK");
#else
    Serial.println("  [1] No DE pin configured");
#endif

    // =========================================================================
    // BARE-METAL UART SETUP (no driver - we handle everything ourselves)
    // This gives us full control and lowest latency like AVR
    // =========================================================================

    Serial.println("  [2] Configuring UART via ESP-IDF driver...");

    // Use ESP-IDF UART driver for initial configuration (handles chip-specific differences)
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };

    Serial.println("      [2a] Installing UART driver...");
    Serial.flush();
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t)RS485_UART_NUM, 256, 0, 0, NULL, 0));

    Serial.println("      [2b] Configuring UART parameters...");
    Serial.flush();
    ESP_ERROR_CHECK(uart_param_config((uart_port_t)RS485_UART_NUM, &uart_config));

    Serial.println("      [2c] Setting UART pins...");
    Serial.flush();
    ESP_ERROR_CHECK(uart_set_pin((uart_port_t)RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    Serial.println("      [2d] Removing UART driver (keeping HW config)...");
    Serial.flush();
    ESP_ERROR_CHECK(uart_driver_delete((uart_port_t)RS485_UART_NUM));

    Serial.println("  [2] UART configured via driver OK");

    // Re-connect pins after driver deletion (driver might have disconnected them)
    Serial.println("  [3] Re-connecting UART pins...");
    gpio_set_direction((gpio_num_t)RS485_TX_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(RS485_TX_PIN, UART_PERIPH_SIGNAL(RS485_UART_NUM, SOC_UART_TX_PIN_IDX), false, false);
    gpio_set_direction((gpio_num_t)RS485_RX_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)RS485_RX_PIN, GPIO_PULLUP_ONLY);
    esp_rom_gpio_connect_in_signal(RS485_RX_PIN, UART_PERIPH_SIGNAL(RS485_UART_NUM, SOC_UART_RX_PIN_IDX), false);
    Serial.println("  [3] UART pins re-connected OK");

    Serial.println("  [4] Configuring RX FIFO threshold...");
    // Configure RX FIFO threshold - trigger on every byte for lowest latency
    uart_ll_set_rxfifo_full_thr(uartHw, 1);
    Serial.println("  [4] RX FIFO threshold set OK");

    Serial.println("  [5] Clearing and enabling interrupts...");
    // Clear any pending interrupts
    uart_ll_clr_intsts_mask(uartHw, UART_LL_INTR_MASK);

    // Enable RX FIFO full interrupt
    uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
    Serial.println("  [5] Interrupts configured OK");

    Serial.println("  [6] Registering ISR...");
    // Register our ISR directly (no driver to conflict with)
    ESP_ERROR_CHECK(esp_intr_alloc(uart_periph_signal[RS485_UART_NUM].irq,
                                    ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
                                    uart_isr_handler, NULL, &uartIntrHandle));
    Serial.println("  [6] ISR registered OK");

    rs485State = STATE_SYNC;
    lastRxTime = esp_timer_get_time();

    Serial.println("  [7] RS485 initialization complete!");
    udpDbgSend("RS485 ISR mode initialized, DE pin=%d", RS485_DE_PIN);
}

// ============================================================================
// PROCESS EXPORT DATA (called from main loop)
// ============================================================================

static void processExportData() {
    // Process bytes queued by ISR
    while (exportReadPos != exportWritePos) {
        uint8_t c = exportBuffer[exportReadPos];
        exportReadPos = (exportReadPos + 1) % EXPORT_BUFFER_SIZE;
        parser.processChar(c);
    }
}

// ============================================================================
// INPUT/OUTPUT CLASSES (AVR-style with proper debounce)
// ============================================================================

#define POLL_EVERY_TIME 0

class Switch2Pos : public PollingInput {
private:
    const char* msg;
    uint8_t pin;
    int8_t lastState;           // Last state sent to DCS
    int8_t debounceSteadyState; // Current debounce tracking state
    bool reverse;
    unsigned long debounceDelay;
    unsigned long lastDebounceTime;

    void resetState() override {
        lastState = (lastState == 0) ? -1 : 0;  // Force re-send on next stable read
    }

    void pollInput() override {
        int8_t state = digitalRead(pin);
        if (reverse) state = !state;

        unsigned long now = millis();

        // If state changed, reset debounce timer
        if (state != debounceSteadyState) {
            lastDebounceTime = now;
            debounceSteadyState = state;
        }

        // Only act if state has been stable for debounce period
        if ((now - lastDebounceTime) >= debounceDelay) {
            if (debounceSteadyState != lastState) {
                if (tryToSendDcsBiosMessage(msg, state == HIGH ? "0" : "1")) {
                    lastState = debounceSteadyState;
                }
                // If not sent (buffer busy), will retry next loop
            }
        }
    }

public:
    Switch2Pos(const char* msg, uint8_t pin, bool reverse = false, unsigned long debounceDelay = 5)
        : PollingInput(POLL_EVERY_TIME), msg(msg), pin(pin), reverse(reverse),
          debounceDelay(debounceDelay), lastDebounceTime(0) {
        pinMode(pin, INPUT_PULLUP);
        lastState = digitalRead(pin);
        if (reverse) lastState = !lastState;
        debounceSteadyState = lastState;
    }
};

class LED : public IntegerBuffer {
private:
    uint8_t pin;

public:
    LED(uint16_t address, uint16_t mask, uint8_t pin)
        : IntegerBuffer(address, mask, 0, nullptr), pin(pin) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    void loop() override {
        if (hasUpdatedData()) {
            digitalWrite(pin, getData() ? HIGH : LOW);
        }
    }
};

// ============================================================================
// TEST PINS
// ============================================================================

#define SWITCH_PIN      23
#define BUTTON_PIN      -1
#define MC_READY_PIN    22

#if SWITCH_PIN >= 0
Switch2Pos masterArmSw("MASTER_ARM_SW", SWITCH_PIN);
#endif

#if BUTTON_PIN >= 0
Switch2Pos ufcOpt1Btn("UFC_OS1", BUTTON_PIN);
#endif

#if MC_READY_PIN >= 0
LED mcReadyLed(0x740C, 0x8000, MC_READY_PIN);
#endif

// ============================================================================
// ARDUINO SETUP AND LOOP
// ============================================================================

void setup() {
    // Initialize Serial for debug output
    Serial.begin(115200);
    delay(3000);  // Give serial time to connect
    Serial.println();
    Serial.println("===========================================");
    Serial.println("ESP32 RS485 Slave - ISR Mode Starting...");
    Serial.printf("Chip Model: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("CPU Freq: %d MHz, Flash: %d MB\n", ESP.getCpuFreqMHz(), ESP.getFlashChipSize() / 1024 / 1024);
    Serial.printf("Max valid GPIO: %d\n", SOC_GPIO_PIN_COUNT - 1);
    Serial.printf("Slave Address: %d\n", SLAVE_ADDRESS);
    Serial.printf("Baud Rate: %d\n", RS485_BAUD_RATE);
    Serial.printf("TX Pin: %d, RX Pin: %d, DE Pin: %d\n", RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN);
    Serial.printf("Switch Pin: %d, MC Ready LED Pin: %d\n", SWITCH_PIN, MC_READY_PIN);
    Serial.println("===========================================");

    // Check if configured pins are valid for this chip
    #if SWITCH_PIN >= 0
    if (SWITCH_PIN >= SOC_GPIO_PIN_COUNT) {
        Serial.printf("*** WARNING: SWITCH_PIN %d is INVALID for this chip (max=%d)! ***\n",
                      SWITCH_PIN, SOC_GPIO_PIN_COUNT - 1);
    }
    #endif
    #if MC_READY_PIN >= 0
    if (MC_READY_PIN >= SOC_GPIO_PIN_COUNT) {
        Serial.printf("*** WARNING: MC_READY_PIN %d is INVALID for this chip (max=%d)! ***\n",
                      MC_READY_PIN, SOC_GPIO_PIN_COUNT - 1);
    }
    #endif
    if (RS485_TX_PIN >= SOC_GPIO_PIN_COUNT || RS485_RX_PIN >= SOC_GPIO_PIN_COUNT) {
        Serial.printf("*** WARNING: UART pins may be INVALID for this chip (max=%d)! ***\n",
                      SOC_GPIO_PIN_COUNT - 1);
    }

    udpDbgInit();

    Serial.println("Initializing RS485 hardware...");
    initRS485Hardware();
    Serial.println("RS485 hardware initialized!");
    Serial.println("Entering main loop...");
}

static unsigned long lastHeartbeat = 0;
static unsigned long loopCount = 0;

void loop() {
    udpDbgCheck();

    // Process export data queued by ISR (for LED updates etc)
    processExportData();

    // Poll inputs (switches, buttons)
    PollingInput::pollInputs();

    // Update outputs (LEDs)
    ExportStreamListener::loopAll();

    // Heartbeat every 5 seconds
    loopCount++;
    if (millis() - lastHeartbeat >= 5000) {
        lastHeartbeat = millis();

        // DEBUG: Check UART status directly (bypass ISR)
        uint32_t fifoLen = uart_ll_get_rxfifo_len(uartHw);
        uint32_t intRaw = uartHw->int_raw.val;
        uint32_t intEna = uartHw->int_ena.val;
        uint32_t intSt = uartHw->int_st.val;

        Serial.printf("[ALIVE] loops=%lu, state=%d, isrTrig=%lu, isrBytes=%lu\n",
                      loopCount, (int)rs485State, isrTriggerCount, isrByteCount);
        Serial.printf("        FIFO=%lu, intRaw=0x%04X, intEna=0x%04X, intSt=0x%04X\n",
                      fifoLen, intRaw, intEna, intSt);
        udpDbgSend("ALIVE isrTrig=%lu FIFO=%lu intRaw=0x%X intEna=0x%X", isrTriggerCount, fifoLen, intRaw, intEna);
        loopCount = 0;
    }
}
