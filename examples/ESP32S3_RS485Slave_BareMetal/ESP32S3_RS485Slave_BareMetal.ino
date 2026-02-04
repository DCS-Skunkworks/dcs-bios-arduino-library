/**
 * ESP32-S3 RS485 SLAVE - INTERRUPT-DRIVEN (AVR-style)
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

// Pin Configuration (Waveshare ESP32-S3-RS485-CAN)
#define RS485_TX_PIN    17
#define RS485_RX_PIN    18
#define RS485_DE_PIN    21    // Set to -1 for auto-direction transceivers

// UART Configuration
#define RS485_UART_NUM  1
#define RS485_BAUD_RATE 250000

// ============================================================================
// CLOCK SOURCE SELECTION
// ============================================================================
#define UART_CLOCK_SOURCE   UART_SCLK_XTAL

// Buffer Sizes
#define MESSAGE_BUFFER_SIZE    64

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
#define SYNC_TIMEOUT_US      500    // 500µs silence = sync detected

// ============================================================================
// DEBUG OPTIONS
// ============================================================================
#define UDP_DEBUG_ENABLE    0
#define WIFI_SSID           "TestNetwork"
#define WIFI_PASSWORD       "TestingOnly"

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <hal/uart_ll.h>
#include <hal/gpio_ll.h>
#include <soc/uart_struct.h>
#include <soc/gpio_struct.h>
#include <soc/uart_periph.h>
#include <esp_timer.h>
#include <esp_intr_alloc.h>

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

            if (now - pi->lastPollTime > pi->pollingIntervalMs) {
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

static void IRAM_ATTR sendResponseISR() {
    uint8_t len = messageBuffer.getLengthISR();

    // Enable driver
    setDE_ISR(true);

    // Send length byte
    uart_ll_write_txfifo(uartHw, (const uint8_t*)&len, 1);
    while (!uart_ll_is_tx_idle(uartHw));

    // Send msgtype (0)
    uint8_t zero = 0;
    uart_ll_write_txfifo(uartHw, &zero, 1);
    while (!uart_ll_is_tx_idle(uartHw));

    // Send data bytes
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = messageBuffer.getISR();
        uart_ll_write_txfifo(uartHw, &b, 1);
        while (!uart_ll_is_tx_idle(uartHw));
    }

    // Send checksum
    uint8_t checksum = 0x72;
    uart_ll_write_txfifo(uartHw, &checksum, 1);
    while (!uart_ll_is_tx_idle(uartHw));

    // Disable driver
    setDE_ISR(false);

    // Flush RX (echo)
    uart_ll_rxfifo_rst(uartHw);

    // Clear message buffer
    messageBuffer.clearISR();

    rs485State = STATE_RX_WAIT_ADDRESS;
}

static void IRAM_ATTR sendZeroLengthResponseISR() {
    // Enable driver
    setDE_ISR(true);

    // Send zero length
    uint8_t zero = 0;
    uart_ll_write_txfifo(uartHw, &zero, 1);
    while (!uart_ll_is_tx_idle(uartHw));

    // Disable driver
    setDE_ISR(false);

    // Flush RX (echo)
    uart_ll_rxfifo_rst(uartHw);

    rs485State = STATE_RX_WAIT_ADDRESS;
}

// ============================================================================
// UART RX ISR - This is where the magic happens!
// Fires immediately when byte arrives, processes state machine, responds instantly
// ============================================================================

static void IRAM_ATTR uart_isr_handler(void *arg) {
    uint32_t uart_intr_status = uart_ll_get_intsts_mask(uartHw);

    // Process all available bytes
    while (uart_ll_get_rxfifo_len(uartHw) > 0) {
        uint8_t c;
        uart_ll_read_rxfifo(uartHw, &c, 1);

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

    Serial.println("  [2] Configuring UART...");
    // =========================================================================
    // UART: Configure for 250kbaud, install driver first for pin config
    // =========================================================================
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_CLOCK_SOURCE
    };

    Serial.println("  [3] Installing UART driver...");
    // Install driver with minimal buffers (we handle RX in ISR)
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t)RS485_UART_NUM, 256, 0, 0, NULL, 0));
    Serial.println("  [3] UART driver installed OK");

    Serial.println("  [4] Configuring UART params...");
    ESP_ERROR_CHECK(uart_param_config((uart_port_t)RS485_UART_NUM, &uart_config));
    Serial.println("  [4] UART params configured OK");

    Serial.println("  [5] Setting UART pins...");
    ESP_ERROR_CHECK(uart_set_pin((uart_port_t)RS485_UART_NUM,
                                  RS485_TX_PIN, RS485_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    Serial.println("  [5] UART pins set OK");

    Serial.println("  [6] Disabling default RX interrupt...");
    // Disable driver's default ISR handling
    ESP_ERROR_CHECK(uart_disable_rx_intr((uart_port_t)RS485_UART_NUM));
    Serial.println("  [6] Default RX interrupt disabled OK");

    // =========================================================================
    // Register our own ISR for immediate response
    // =========================================================================

    Serial.println("  [7] Setting RX FIFO threshold...");
    // Configure RX FIFO threshold - trigger on every byte for lowest latency
    uart_ll_set_rxfifo_full_thr(uartHw, 1);
    Serial.println("  [7] RX FIFO threshold set OK");

    Serial.println("  [8] Enabling RX FIFO interrupt...");
    // Enable RX FIFO full interrupt
    uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
    Serial.println("  [8] RX FIFO interrupt enabled OK");

    Serial.println("  [9] Allocating custom ISR...");
    // Allocate and register ISR using the peripheral signal table
    ESP_ERROR_CHECK(esp_intr_alloc(uart_periph_signal[RS485_UART_NUM].irq,
                                    ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
                                    uart_isr_handler, NULL, &uartIntrHandle));
    Serial.println("  [9] Custom ISR allocated OK");

    rs485State = STATE_SYNC;
    lastRxTime = esp_timer_get_time();

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
// INPUT/OUTPUT CLASSES
// ============================================================================

class Switch2Pos : public PollingInput {
private:
    const char* msg;
    uint8_t pin;
    bool lastState;

    void resetState() override {
        lastState = !digitalRead(pin);
    }

    void pollInput() override {
        bool state = !digitalRead(pin);
        if (state != lastState) {
            if (tryToSendDcsBiosMessage(msg, state ? "1" : "0")) {
                lastState = state;
            }
        }
    }

public:
    Switch2Pos(const char* msg, uint8_t pin, bool reversePolarity = false)
        : PollingInput(50), msg(msg), pin(pin), lastState(false) {
        pinMode(pin, INPUT_PULLUP);
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

#define SWITCH_PIN      2
#define BUTTON_PIN      0
#define MC_READY_PIN    1

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
    delay(1000);  // Give serial time to connect
    Serial.println();
    Serial.println("===========================================");
    Serial.println("ESP32-S3 RS485 Slave - ISR Mode Starting...");
    Serial.printf("Slave Address: %d\n", SLAVE_ADDRESS);
    Serial.printf("Baud Rate: %d\n", RS485_BAUD_RATE);
    Serial.printf("TX Pin: %d, RX Pin: %d, DE Pin: %d\n", RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN);
    Serial.println("===========================================");

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
        Serial.printf("[ALIVE] loops=%lu, state=%d, exportBuf=%d/%d\n",
                      loopCount, (int)rs485State,
                      exportReadPos, exportWritePos);
        loopCount = 0;
    }
}
