/**
 * ESP32-S3 RS485 SLAVE - MANUAL DE CONTROL
 *
 * Key changes:
 * 1. Uses manual DE control (GPIO) instead of UART_MODE_RS485_HALF_DUPLEX
 * 2. Matches ESP32 Master approach for consistent timing
 * 3. Protocol: Length byte = DATA bytes only (not including msgtype)
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
// Options:
//   UART_SCLK_DEFAULT - APB clock (~80MHz) - most common
//   UART_SCLK_XTAL    - Crystal clock (40MHz) - more stable
//   UART_SCLK_RTC     - RTC clock (8MHz) - low power but less accurate
// ============================================================================
#define UART_CLOCK_SOURCE   UART_SCLK_XTAL

// Buffer Sizes
#define UART_RX_BUFFER_SIZE    512
#define UART_TX_BUFFER_SIZE    256
#define MESSAGE_BUFFER_SIZE    64

// ============================================================================
// TIMING CONFIGURATION - Tweak these to debug corruption issues
// ============================================================================
#define SYNC_TIMEOUT_US      500    // 500µs silence = sync detected
#define RX_TIMEOUT_SYMBOLS   10    // UART RX timeout in bit periods. Was 12, why? check AVR and protocol, whats the optimal value based on protocol and 250000 baud rate?
#define PRE_TX_DELAY_US       0     // Delay after DE high, before first byte. Setting to 1000 makes slave send blanks, setting to 0 works
#define FRAME_TIMEOUT_US      0     // Reset if stuck mid-frame (0 = disabled)
#define POST_TX_DELAY_US      0     // Any delay here causes the slave to send blanks.. why?

// ============================================================================
// DEBUG OPTIONS
// ============================================================================
#define UDP_DEBUG_ENABLE    1
#define DEBUG_TX_HEX        1       // Log transmitted bytes as hex
#define WIFI_SSID           "TestNetwork"
#define WIFI_PASSWORD       "TestingOnly"

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <hal/uart_ll.h>
#include <soc/uart_struct.h>
#include <esp_timer.h>

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
        udpDbgSend("=== SLAVE %d ONLINE === IP=%s", SLAVE_ADDRESS, WiFi.localIP().toString().c_str());
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
// DCS-BIOS PROTOCOL PARSER
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

class ProtocolParser {
private:
    volatile uint8_t state;
    volatile uint16_t address;
    volatile uint16_t count;
    volatile uint16_t data;
    volatile uint8_t sync_byte_count;
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

static bool messageSentOrQueued = false;

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
// RING BUFFER
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

    __attribute__((always_inline)) void put(uint8_t c) {
        buffer[writepos] = c;
        writepos = (writepos + 1) % SIZE;
    }

    __attribute__((always_inline)) uint8_t get() {
        uint8_t ret = buffer[readpos];
        readpos = (readpos + 1) % SIZE;
        return ret;
    }

    __attribute__((always_inline)) bool isEmpty() { return readpos == writepos; }
    __attribute__((always_inline)) bool isNotEmpty() { return readpos != writepos; }
    __attribute__((always_inline)) uint8_t getLength() { return (writepos - readpos) % SIZE; }
    __attribute__((always_inline)) void clear() { readpos = 0; writepos = 0; }
    __attribute__((always_inline)) uint8_t availableForWrite() { return SIZE - getLength() - 1; }
};

// ============================================================================
// GLOBAL INSTANCES
// ============================================================================

static ProtocolParser parser;
static RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

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

    udpDbgSend("Q: %s %s", msg, arg);
    return true;
}

// ============================================================================
// RS485 SLAVE STATE MACHINE
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
    STATE_RX_WAIT_ANSWER_CHECKSUM,
    STATE_TX_SENDING
};

enum RxDataType {
    RXDATA_IGNORE,
    RXDATA_DCSBIOS_EXPORT
};

static volatile RS485State rs485State = STATE_SYNC;
static volatile uint8_t rxSlaveAddress = 0;
static volatile uint8_t rxMsgType = 0;
static volatile uint8_t rxtxLen = 0;
static volatile RxDataType rxDataType = RXDATA_IGNORE;
static volatile int64_t lastRxTime = 0;

static uart_port_t uartNum = (uart_port_t)RS485_UART_NUM;

// ============================================================================
// MANUAL DE CONTROL - Like the ESP32 Master for precise timing
// ============================================================================

static gpio_num_t dePin = (gpio_num_t)RS485_DE_PIN;

static void setDE(bool high) {
    gpio_set_level(dePin, high ? 1 : 0);
}

static void initRS485Hardware() {
    // =========================================================================
    // GPIO: Manual DE pin control (NOT using UART's auto RS485 mode!)
    // This matches the ESP32 Master approach for consistent timing
    // =========================================================================
#if RS485_DE_PIN >= 0
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << dePin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    setDE(false);  // Start in RX mode
#endif

    // =========================================================================
    // UART: Normal mode (NOT RS485 auto mode!)
    // We handle DE manually for precise timing control
    // =========================================================================
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_CLOCK_SOURCE  // Configurable clock source
    };

    ESP_ERROR_CHECK(uart_driver_install(uartNum, UART_RX_BUFFER_SIZE,
                                         UART_TX_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uartNum, &uart_config));

    // Set pins WITHOUT RTS pin - we control DE manually
    ESP_ERROR_CHECK(uart_set_pin(uartNum,
                                  RS485_TX_PIN,
                                  RS485_RX_PIN,
                                  UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE));

    // Use normal UART mode, NOT RS485 auto mode
    ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_UART));

    ESP_ERROR_CHECK(uart_set_rx_timeout(uartNum, RX_TIMEOUT_SYMBOLS));
    uart_flush_input(uartNum);

    rs485State = STATE_SYNC;
    lastRxTime = esp_timer_get_time();
}

// ============================================================================
// TX HANDLING - BYTE-BY-BYTE TRANSMISSION (like AVR ISR-driven approach)
// ============================================================================

// Spinlock for critical sections - prevents WiFi/radio ISRs from corrupting TX timing
static portMUX_TYPE txMutex = portMUX_INITIALIZER_UNLOCKED;

// Get the hardware UART device pointer for direct register access
static uart_dev_t* const uartHw = &UART1;

// Write ONE byte and wait for it to be transmitted before returning
// This mimics AVR's ISR-driven byte-by-byte transmission with natural gaps
static inline void txByte(uint8_t c) {
    // Write byte to FIFO
    uart_ll_write_txfifo(uartHw, &c, 1);
    // Wait for this byte to be fully transmitted (shift register empty)
    while (!uart_ll_is_tx_idle(uartHw)) {
        // Spin
    }
}

static void sendResponse() {
    uint8_t packet[MESSAGE_BUFFER_SIZE + 4];
    uint8_t len = messageBuffer.getLength();

    packet[0] = len;      // Length = DATA bytes only (NOT including msgtype!)
    packet[1] = 0;        // Message type = 0

    for (uint8_t i = 0; i < len; i++) {
        packet[2 + i] = messageBuffer.get();
    }

    packet[2 + len] = 0x72;  // Checksum

    size_t totalBytes = 3 + len;

    // =========================================================================
    // CRITICAL SECTION with byte-by-byte transmission (like AVR ISR approach)
    // =========================================================================
    portENTER_CRITICAL(&txMutex);

#if RS485_DE_PIN >= 0
    setDE(true);
    // Small delay to let transceiver fully enable before first bit
    // The MAX13488E needs ~30ns, but GPIO + bus settling might need more
    delayMicroseconds(2);
#endif

    // Transmit byte-by-byte, waiting for each to complete (like AVR ISR)
    for (size_t i = 0; i < totalBytes; i++) {
        txByte(packet[i]);
    }

#if RS485_DE_PIN >= 0
    setDE(false);  // Return to RX mode
#endif

    portEXIT_CRITICAL(&txMutex);

    messageBuffer.clear();
    messageBuffer.complete = false;
    rs485State = STATE_RX_WAIT_ADDRESS;

    // Debug logging - show actual bytes transmitted
#if DEBUG_TX_HEX
    static char hexBuf[128];
    int pos = snprintf(hexBuf, sizeof(hexBuf), "TX[%d]: ", (int)totalBytes);
    for (size_t i = 0; i < totalBytes && pos < 120; i++) {
        pos += snprintf(hexBuf + pos, sizeof(hexBuf) - pos, "%02X ", packet[i]);
    }
    udpDbgSend("%s", hexBuf);
#else
    udpDbgSend("TX len=%d", len);
#endif
}

static void sendZeroLengthResponse() {
    portENTER_CRITICAL(&txMutex);

#if RS485_DE_PIN >= 0
    setDE(true);
    delayMicroseconds(2);  // Let transceiver enable
#endif

    txByte(0);  // Zero-length response

#if RS485_DE_PIN >= 0
    setDE(false);
#endif

    portEXIT_CRITICAL(&txMutex);

    rs485State = STATE_RX_WAIT_ADDRESS;
}

// ============================================================================
// MAIN PROCESSING LOOP - EXACTLY AS OLD VERSION
// ============================================================================

static void processRS485() {
    int64_t now = esp_timer_get_time();

    // =========================================================================
    // FRAME TIMEOUT - Reset if stuck mid-frame too long
    // Note: AVR doesn't have this, but ESP32 needs it to recover from noise
    // Set FRAME_TIMEOUT_US to 0 to disable (like AVR)
    // =========================================================================
#if FRAME_TIMEOUT_US > 0
    if (rs485State != STATE_SYNC && rs485State != STATE_RX_WAIT_ADDRESS) {
        if ((now - lastRxTime) >= FRAME_TIMEOUT_US) {
            rs485State = STATE_SYNC;
        }
    }
#endif

    // Sync detection - 500µs silence means ready for new frame (like AVR)
    if (rs485State == STATE_SYNC && (now - lastRxTime) >= SYNC_TIMEOUT_US) {
        rs485State = STATE_RX_WAIT_ADDRESS;
    }

    // Process all available RX bytes - NO unnecessary function calls!
    size_t available = 0;
    uart_get_buffered_data_len(uartNum, &available);

    while (available > 0) {
        uint8_t c;
        if (uart_read_bytes(uartNum, &c, 1, 0) != 1) break;
        available--;

        // Capture current time for this byte
        int64_t rxTime = esp_timer_get_time();

        switch (rs485State) {
            case STATE_SYNC:
                // AVR behavior: if 500µs passed since last byte, this is start of new frame
                if ((rxTime - lastRxTime) < SYNC_TIMEOUT_US) {
                    lastRxTime = rxTime;  // Update and stay in sync
                    break;
                }
                // 500µs passed - transition and FALL THROUGH to process as address!
                rs485State = STATE_RX_WAIT_ADDRESS;
                // FALL THROUGH!

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
                    goto handle_message_complete;
                }

                if (rxSlaveAddress == 0 && rxMsgType == 0) {
                    rxDataType = RXDATA_DCSBIOS_EXPORT;
                } else {
                    rxDataType = RXDATA_IGNORE;
                }

                rs485State = STATE_RX_WAIT_DATA;
                break;

            case STATE_RX_WAIT_DATA:
                rxtxLen--;

                if (rxDataType == RXDATA_DCSBIOS_EXPORT) {
                    parser.processChar(c);
                }

                if (rxtxLen == 0) {
                    rs485State = STATE_RX_WAIT_CHECKSUM;
                }
                break;

            case STATE_RX_WAIT_CHECKSUM:
                goto handle_message_complete;

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
                break;
        }

        // Update lastRxTime for frame timeout tracking (except SYNC which handles it)
        if (rs485State != STATE_SYNC) {
            lastRxTime = rxTime;
        }

        continue;

    handle_message_complete:
        if (rxSlaveAddress == 0) {
            rs485State = STATE_RX_WAIT_ADDRESS;
        } else if (rxSlaveAddress == SLAVE_ADDRESS) {
            if (rxMsgType == 0 && rxtxLen == 0) {
                if (!messageBuffer.complete) {
                    sendZeroLengthResponse();
                } else {
                    sendResponse();
                }
            } else {
                rs485State = STATE_SYNC;
            }
        } else {
            rs485State = STATE_RX_WAIT_ANSWER_DATALENGTH;
        }
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
    udpDbgInit();
    initRS485Hardware();
}

void loop() {
    udpDbgCheck();
    processRS485();
    PollingInput::pollInputs();
    ExportStreamListener::loopAll();
}
