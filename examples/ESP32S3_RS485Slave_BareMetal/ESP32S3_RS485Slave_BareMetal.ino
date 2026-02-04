/**
 * =============================================================================
 * ESP32 RS485 SLAVE - HARDWARE RS485 MODE IMPLEMENTATION
 * =============================================================================
 *
 * High-performance RS485 Slave for DCS-BIOS using the ESP32's NATIVE HARDWARE
 * RS485 mode for CYCLE-ACCURATE DE pin timing.
 *
 * SUPPORTED ESP32 VARIANTS:
 * - ESP32 (Classic)    - Dual-core
 * - ESP32-S2           - Single-core
 * - ESP32-S3           - Dual-core
 * - ESP32-C3           - Single-core RISC-V
 * - ESP32-C6           - Single-core RISC-V
 *
 * All variants support UART_MODE_RS485_HALF_DUPLEX for hardware DE control.
 *
 * =============================================================================
 * CRITICAL HARDWARE FEATURE: UART_MODE_RS485_HALF_DUPLEX
 * =============================================================================
 *
 * The ESP32 UART peripheral has a dedicated RS485 mode that provides:
 *
 * 1. AUTOMATIC DE ASSERTION
 *    - Hardware asserts RTS (used as DE) when TX FIFO receives data
 *    - No software intervention required
 *    - Timing: Within 1 APB clock cycle (~12.5ns @ 80MHz)
 *
 * 2. AUTOMATIC DE DEASSERTION
 *    - Hardware deasserts RTS when the LAST STOP BIT leaves the shift register
 *    - Cycle-accurate to the baud rate clock
 *    - At 250kbaud: Precision of ±4µs (1 bit time)
 *    - NO SOFTWARE JITTER - completely hardware-controlled
 *
 * 3. COLLISION DETECTION (Optional)
 *    - Hardware can detect bus collisions during transmission
 *    - Not used in this implementation (single-master bus)
 *
 * This is the ONLY way to achieve true protocol-compliant RS485 timing on ESP32
 * without external auto-direction transceivers.
 *
 * =============================================================================
 * TIMING ANALYSIS
 * =============================================================================
 *
 * Previous (Software) Implementation:
 * -----------------------------------
 * DE Assert:    Software trigger → ~100-500ns (register write latency)
 * DE Deassert:  Calculated time + poll → ±10-50µs JITTER
 *
 * This (Hardware) Implementation:
 * -------------------------------
 * DE Assert:    Hardware trigger → <12.5ns (1 APB cycle)
 * DE Deassert:  Hardware trigger → ±0ns (synchronous to UART clock)
 *                                  Actual precision: ±4µs (1 bit @ 250kbaud)
 *
 * The hardware RS485 mode eliminates ALL software-induced timing jitter for
 * the DE pin. The only timing variation is the inherent ±0.5 bit uncertainty
 * in the UART's baud rate generator, which is unavoidable in any implementation.
 *
 * =============================================================================
 * HARDWARE CONFIGURATION (adjust pins for your board)
 * =============================================================================
 *
 * Default pins are for Waveshare ESP32-S3-RS485-CAN:
 * TX Pin: GPIO 17 → RS485 DI (Driver Input)
 * RX Pin: GPIO 18 ← RS485 RO (Receiver Output)
 * DE Pin: GPIO 21 → RS485 DE + /RE (Hardware-controlled via RTS), or -1 for auto-dir
 * Speed:  250,000 bps (8N1)
 *
 * IMPORTANT: The DE pin is configured as the UART's RTS output. The hardware
 * RS485 mode repurposes RTS for automatic direction control.
 *
 * =============================================================================
 * LICENSE: Same as DCS-BIOS Arduino Library
 * =============================================================================
 */

// ============================================================================
// CONFIGURATION - Customize for your setup
// ============================================================================

// Slave address (1-127) - MUST be unique on the RS485 bus
#define SLAVE_ADDRESS 1

// Pin Configuration (Waveshare ESP32-S3-RS485-CAN defaults)
#define RS485_TX_PIN    17
#define RS485_RX_PIN    18
#define RS485_DE_PIN    -1    // -1 = auto-direction mode (board may have built-in auto-dir)
                              // Set to 21 if manual DE control is needed

// DE Pin Polarity - TRY CHANGING THIS IF YOU SEE CORRUPTED DATA!
// 0 = Normal (DE HIGH during TX) - standard for most RS485 transceivers
// 1 = Inverted (DE LOW during TX) - some boards have inverters on DE line
#define RS485_DE_INVERT 0

// DE Control Mode (only used when RS485_DE_PIN >= 0)
// 0 = Hardware RS485 mode (ESP32 controls DE automatically via RTS)
// 1 = Manual GPIO mode (we control DE pin explicitly - more compatible)
// Note: Ignored when RS485_DE_PIN = -1 (auto-direction mode)
#define RS485_DE_MANUAL 0

// UART Configuration
#define RS485_UART_NUM  1        // UART1 (UART0 is typically used for USB/debug)
#define RS485_BAUD_RATE 250000   // DCS-BIOS standard baud rate

// Buffer Sizes
#define UART_RX_BUFFER_SIZE    512
#define UART_TX_BUFFER_SIZE    256
#define MESSAGE_BUFFER_SIZE    64   // Max pending message size

// Timing Constants
#define SYNC_TIMEOUT_US        500       // 500µs silence = sync detected (matches AVR)
#define RX_TIMEOUT_SYMBOLS     12        // ~480µs at 250kbaud (12 symbol times)
#define RX_FRAME_TIMEOUT_US    500000    // 500ms - reset if stuck mid-frame (safety net)

// ============================================================================
// UDP DEBUG - Uses CockpitOS debug console on port 4210
// ============================================================================
// This sends debug data via UDP without affecting RS485 timing.
// Set UDP_DEBUG_ENABLE to 1 and configure WiFi to use.

#define UDP_DEBUG_ENABLE    1       // Set to 1 to enable UDP debug (0 = disabled for best timing)
#define UDP_DEBUG_IP        "255.255.255.255"  // Broadcast to all
#define UDP_DEBUG_PORT      4210    // CockpitOS debug port
#define UDP_DEBUG_NAME      "RS485-SLAVE"     // Device identifier in debug messages
#define WIFI_SSID           "TestNetwork"
#define WIFI_PASSWORD       "TestingOnly"

// Legacy serial debug (only used if UDP_DEBUG_ENABLE is 0)
#define DEBUG_VERBOSE       0       // Set to 1 for detailed frame-by-frame logging

// Debug macros - use UDP if enabled, otherwise Serial
#if UDP_DEBUG_ENABLE
    // Forward declaration - actual class defined after includes
    #define DBG_INIT()      // Handled in setup()
    #define DBG(x)
    #define DBGLN(x)
    #define DBGF(...)       udpDbgSend(__VA_ARGS__)
#else
    #define DBG_INIT()      Serial.begin(115200); delay(2000)
    #define DBG(x)          Serial.print(x)
    #define DBGLN(x)        Serial.println(x)
    #define DBGF(...)       Serial.printf(__VA_ARGS__)
#endif

// Verbose logging for detailed protocol debugging (very spammy - disabled)
#if DEBUG_VERBOSE && !UDP_DEBUG_ENABLE
    #define DBGV(x)         Serial.print(x)
    #define DBGVLN(x)       Serial.println(x)
    #define DBGVF(...)      Serial.printf(__VA_ARGS__)
#else
    #define DBGV(x)
    #define DBGVLN(x)
    #define DBGVF(...)
#endif

// ============================================================================
// ESP32 HARDWARE INCLUDES
// ============================================================================

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <hal/uart_ll.h>
#include <soc/uart_struct.h>
#include <esp_timer.h>

#if UDP_DEBUG_ENABLE
#include <WiFi.h>
#include <WiFiUdp.h>

// UDP Debug - lightweight, non-blocking
static WiFiUDP udpDbg;
static IPAddress udpDbgTarget;
static bool udpDbgConnected = false;
static char udpDbgBuf[256];

void udpDbgInit() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    udpDbgTarget.fromString(UDP_DEBUG_IP);
}

void udpDbgCheck() {
    if (!udpDbgConnected && WiFi.status() == WL_CONNECTED) {
        udpDbgConnected = true;
        udpDbgSend("=== %s ONLINE === addr=%d IP=%s", UDP_DEBUG_NAME, SLAVE_ADDRESS, WiFi.localIP().toString().c_str());
    }
}

void udpDbgSend(const char* fmt, ...) {
    if (!udpDbgConnected) return;
    int pos = snprintf(udpDbgBuf, sizeof(udpDbgBuf), "[%s] ", UDP_DEBUG_NAME);
    va_list args;
    va_start(args, fmt);
    vsnprintf(udpDbgBuf + pos, sizeof(udpDbgBuf) - pos, fmt, args);
    va_end(args);
    udpDbg.beginPacket(udpDbgTarget, UDP_DEBUG_PORT);
    udpDbg.write((uint8_t*)udpDbgBuf, strlen(udpDbgBuf));
    udpDbg.endPacket();
}
#else
// Dummy functions when UDP debug is disabled
#define udpDbgInit()
#define udpDbgCheck()
#endif

// ============================================================================
// COMPILE-TIME CHECKS
// ============================================================================

#if !defined(ESP32)
    #error "This implementation requires an ESP32 variant (ESP32, S2, S3, C3, C6, etc.)"
#endif

#if SLAVE_ADDRESS < 1 || SLAVE_ADDRESS > 127
    #error "SLAVE_ADDRESS must be between 1 and 127"
#endif

// ============================================================================
// DCS-BIOS PROTOCOL PARSER
// ============================================================================

// Parser states for export data stream
#define DCSBIOS_STATE_WAIT_FOR_SYNC  0
#define DCSBIOS_STATE_ADDRESS_LOW    1
#define DCSBIOS_STATE_ADDRESS_HIGH   2
#define DCSBIOS_STATE_COUNT_LOW      3
#define DCSBIOS_STATE_COUNT_HIGH     4
#define DCSBIOS_STATE_DATA_LOW       5
#define DCSBIOS_STATE_DATA_HIGH      6

// Forward declarations
class ExportStreamListener;
extern ExportStreamListener* firstExportStreamListener;

/**
 * ExportStreamListener - Observer pattern for DCS-BIOS state changes
 */
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

/**
 * IntegerBuffer - Monitors a specific DCS-BIOS address with mask/shift
 */
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

/**
 * StringBuffer - Monitors a string at a DCS-BIOS address range
 */
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

/**
 * ProtocolParser - Parses the DCS-BIOS export data stream
 */
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
    if (messageBuffer.complete) {
        DBGF("[MSG BLOCKED: buffer busy] %s %s\n", msg, arg);
        return false;
    }

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

    DBGF("[MSG QUEUED: %s %s] len=%d\n", msg, arg, messageBuffer.getLength());
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
    STATE_TX_SENDING     // Hardware handles TX completion automatically!
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
// MANUAL DE CONTROL HELPERS
// ============================================================================
#if RS485_DE_PIN >= 0 && RS485_DE_MANUAL
static inline void deAssert() {
    #if RS485_DE_INVERT
        digitalWrite(RS485_DE_PIN, LOW);   // Inverted: LOW = TX enabled
    #else
        digitalWrite(RS485_DE_PIN, HIGH);  // Normal: HIGH = TX enabled
    #endif
}

static inline void deRelease() {
    #if RS485_DE_INVERT
        digitalWrite(RS485_DE_PIN, HIGH);  // Inverted: HIGH = RX enabled
    #else
        digitalWrite(RS485_DE_PIN, LOW);   // Normal: LOW = RX enabled
    #endif
}
#else
#define deAssert()
#define deRelease()
#endif

// Debug counters for periodic status report
#if UDP_DEBUG_ENABLE
static uint32_t dbgPollCount = 0;      // Polls received for us
static uint32_t dbgBcastCount = 0;     // Broadcasts received
static uint32_t dbgTxCount = 0;        // Data responses sent
static uint32_t dbgErrCount = 0;       // Protocol errors
static uint32_t dbgLastReport = 0;     // Last status report time
#endif

// ============================================================================
// HARDWARE RS485 INITIALIZATION
// ============================================================================
/**
 * initRS485Hardware() - Initialize UART in HARDWARE RS485 MODE
 *
 * ==========================================================================
 * THIS IS THE KEY DIFFERENCE FROM THE PREVIOUS IMPLEMENTATION
 * ==========================================================================
 *
 * The ESP32 UART peripheral has a dedicated RS485 mode activated by:
 *   uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX)
 *
 * In this mode:
 * - The RTS pin is repurposed as DE (Driver Enable)
 * - DE is AUTOMATICALLY asserted when data enters TX FIFO
 * - DE is AUTOMATICALLY deasserted when last stop bit transmits
 * - ALL timing is handled by hardware with ZERO software jitter
 *
 * The hardware implements this at the register level:
 * - UART_RS485_EN: Enables RS485 mode
 * - UART_RS485TX_RX_EN: Enables TX/RX switching
 * - UART_RS485RXBY_TX_EN: RX is disabled during TX (collision avoidance)
 *
 * Timing precision:
 * - DE assertion: <1 APB clock cycle after TX FIFO write (~12.5ns @ 80MHz)
 * - DE deassertion: Synchronous to baud clock, ~0ns jitter
 *                   (limited only by baud rate tolerance, typ. ±0.1%)
 */
static void initRS485Hardware() {
    // =========================================================================
    // UART Configuration
    // =========================================================================
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_XTAL  // Crystal oscillator - most stable for accurate baud rate
    };

    // Install UART driver (no event queue needed for basic operation)
    ESP_ERROR_CHECK(uart_driver_install(uartNum, UART_RX_BUFFER_SIZE,
                                         UART_TX_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uartNum, &uart_config));

#if RS485_DE_PIN >= 0 && RS485_DE_MANUAL
    // =========================================================================
    // MODE 1: MANUAL DE Control (GPIO-controlled, more compatible)
    // =========================================================================
    // TX  → RS485 DI
    // RX  ← RS485 RO
    // DE  → Controlled by GPIO (we assert/release manually around TX)
    //
    // This mode is MORE COMPATIBLE than hardware RS485 mode because:
    // - We have explicit control over DE timing
    // - Works with transceivers that have specific timing requirements
    // - Avoids ESP32 UART_MODE_RS485_HALF_DUPLEX quirks
    // =========================================================================
    ESP_ERROR_CHECK(uart_set_pin(uartNum,
                                  RS485_TX_PIN,      // TX
                                  RS485_RX_PIN,      // RX
                                  UART_PIN_NO_CHANGE,// No RTS (we use GPIO for DE)
                                  UART_PIN_NO_CHANGE // CTS not used
                                  ));

    // Use regular UART mode - we control DE manually
    ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_UART));

    // Configure DE pin as output and set to RX mode (DE released)
    pinMode(RS485_DE_PIN, OUTPUT);
    deRelease();

    DBGF("[DE: MANUAL GPIO %d]\n", RS485_DE_PIN);

#elif RS485_DE_PIN >= 0
    // =========================================================================
    // MODE 2: Hardware DE Control (RS485_DE_PIN is a valid GPIO)
    // =========================================================================
    // TX  → RS485 DI
    // RX  ← RS485 RO
    // RTS → RS485 DE (Hardware-controlled by UART peripheral)
    //
    // The ESP32 UART's RS485 mode automatically:
    // - Asserts DE when TX FIFO has data (<12.5ns latency)
    // - Deasserts DE when last stop bit transmitted (~0ns jitter)
    // =========================================================================
    ESP_ERROR_CHECK(uart_set_pin(uartNum,
                                  RS485_TX_PIN,      // TX
                                  RS485_RX_PIN,      // RX
                                  RS485_DE_PIN,      // RTS → used as DE
                                  UART_PIN_NO_CHANGE // CTS not used
                                  ));

    // Enable hardware RS485 mode with automatic DE control
    ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX));

#if RS485_DE_INVERT
    // Invert DE polarity if needed (some boards have inverters on DE line)
    ESP_ERROR_CHECK(uart_set_line_inverse(uartNum, UART_SIGNAL_RTS_INV));
    DBGF("[DE: HARDWARE INVERTED]\n");
#else
    DBGF("[DE: HARDWARE NORMAL]\n");
#endif

#else
    // =========================================================================
    // MODE 2: Auto-Direction Transceiver (RS485_DE_PIN = -1)
    // =========================================================================
    // TX  → RS485 DI
    // RX  ← RS485 RO
    // DE  → Controlled by transceiver (e.g., MAX13487E, MAX3485 with auto-dir)
    //
    // The transceiver automatically detects TX activity and switches direction.
    // No DE pin control needed from the MCU - just regular UART mode.
    // =========================================================================
    ESP_ERROR_CHECK(uart_set_pin(uartNum,
                                  RS485_TX_PIN,      // TX
                                  RS485_RX_PIN,      // RX
                                  UART_PIN_NO_CHANGE,// No RTS/DE needed
                                  UART_PIN_NO_CHANGE // No CTS needed
                                  ));

    // Use regular UART mode - transceiver handles direction switching
    ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_UART));

#endif

    // =========================================================================
    // RX TIMEOUT CONFIGURATION
    // =========================================================================
    // Configure hardware RX timeout for sync detection
    // At 250kbaud: 1 symbol = 40µs
    // 12 symbols = 480µs ≈ 500µs sync timeout
    ESP_ERROR_CHECK(uart_set_rx_timeout(uartNum, RX_TIMEOUT_SYMBOLS));

    // Flush any garbage
    uart_flush_input(uartNum);

    // Initialize state
    rs485State = STATE_SYNC;
    lastRxTime = esp_timer_get_time();
}

// ============================================================================
// TX HANDLING - Now much simpler with hardware DE control!
// ============================================================================

/**
 * sendResponse() - Transmit response packet to master
 *
 * Simple and elegant - just write bytes to UART and wait for completion.
 * For auto-direction transceivers, no DE control is needed.
 * For hardware RS485 mode, the UART peripheral handles DE automatically.
 */
static void sendResponse() {
    uint8_t packet[MESSAGE_BUFFER_SIZE + 4];
    uint8_t len = messageBuffer.getLength();

    packet[0] = len;      // Length = data bytes only (NOT including msgtype)
    packet[1] = 0;        // Message type = 0 (DCS-BIOS data)

    for (uint8_t i = 0; i < len; i++) {
        packet[2 + i] = messageBuffer.get();
    }

    packet[2 + len] = 0x72;  // Fixed checksum per DCS-BIOS protocol

    size_t totalBytes = 3 + len;

    // Small turnaround delay before TX (AVR uses tx_delay_byte = ~40µs)
    delayMicroseconds(50);

    // TX - hardware or auto-direction handles the rest
    uart_write_bytes(uartNum, (const char*)packet, totalBytes);
    ESP_ERROR_CHECK(uart_wait_tx_done(uartNum, pdMS_TO_TICKS(10)));

    // Read echo bytes (don't just flush - capture them for analysis)
    delayMicroseconds(200);
    size_t echo1 = 0;
    uart_get_buffered_data_len(uartNum, &echo1);

    uint8_t echoBuf[32];
    size_t echoRead = 0;
    if (echo1 > 0) {
        echoRead = uart_read_bytes(uartNum, echoBuf, (echo1 < 32) ? echo1 : 32, 0);
    }

    // Check for stray bytes
    delayMicroseconds(50);
    size_t echo2 = 0;
    uart_get_buffered_data_len(uartNum, &echo2);
    if (echo2 > 0) {
        uart_flush_input(uartNum);
    }

    // Clear message buffer and return to RX state
    messageBuffer.clear();
    messageBuffer.complete = false;
    rs485State = STATE_RX_WAIT_ADDRESS;

    // Log AFTER TX complete - show echo bytes when present
    if (echo1 > 0 || echo2 > 0) {
        DBGF("[TX] len=%d echo=%d[%02X %02X %02X %02X] stray=%d\n",
             len, echo1,
             echoRead > 0 ? echoBuf[0] : 0,
             echoRead > 1 ? echoBuf[1] : 0,
             echoRead > 2 ? echoBuf[2] : 0,
             echoRead > 3 ? echoBuf[3] : 0,
             echo2);
    }
}

/**
 * sendZeroLengthResponse() - Respond with empty packet (no data to send)
 */
static void sendZeroLengthResponse() {
    uint8_t response = 0;

    // Small turnaround delay before TX
    delayMicroseconds(50);

    // TX - hardware or auto-direction handles the rest
    uart_write_bytes(uartNum, (const char*)&response, 1);
    ESP_ERROR_CHECK(uart_wait_tx_done(uartNum, pdMS_TO_TICKS(10)));

    // Wait for echo to arrive and flush (same as sendResponse)
    delayMicroseconds(200);
    uart_flush_input(uartNum);

    // Paranoid check
    size_t stray = 0;
    uart_get_buffered_data_len(uartNum, &stray);
    if (stray > 0) {
        delayMicroseconds(50);
        uart_flush_input(uartNum);
    }

    rs485State = STATE_RX_WAIT_ADDRESS;
}

// ============================================================================
// MAIN PROCESSING LOOP
// ============================================================================

/**
 * processRS485() - Main state machine processing
 *
 * MATCHES OLD WORKING VERSION EXACTLY:
 * - Frame timeout safety (500ms reset if stuck mid-frame)
 * - 500µs sync detection
 * - Simple lastRxTime update inside loop
 * - Simple switch case for STATE_SYNC (just break)
 */
static void processRS485() {
    int64_t now = esp_timer_get_time();

    // =========================================================================
    // Frame Timeout Safety - Reset if stuck mid-frame for too long
    // =========================================================================
    if (rs485State != STATE_SYNC && (now - lastRxTime) >= RX_FRAME_TIMEOUT_US) {
        rs485State = STATE_SYNC;
    }

    // =========================================================================
    // Sync Detection - 500µs of bus silence (matches AVR behavior exactly)
    // =========================================================================
    if (rs485State == STATE_SYNC) {
        if ((now - lastRxTime) >= SYNC_TIMEOUT_US) {
            rs485State = STATE_RX_WAIT_ADDRESS;
        }
    }

    // =========================================================================
    // Process all available RX bytes
    // =========================================================================
    size_t available = 0;
    uart_get_buffered_data_len(uartNum, &available);

    while (available > 0) {
        uint8_t c;
        if (uart_read_bytes(uartNum, &c, 1, 0) != 1) break;
        available--;

        // Update lastRxTime ONCE at start of loop (simple pattern from OLD version)
        lastRxTime = now;

        switch (rs485State) {
            case STATE_SYNC:
                // Byte received during sync - stay in sync, wait for silence
                break;

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

        continue;

    handle_message_complete:
        if (rxSlaveAddress == 0) {
            // Broadcast - no response
            rs485State = STATE_RX_WAIT_ADDRESS;
        } else if (rxSlaveAddress == SLAVE_ADDRESS) {
            // This message is for us!
            if (rxMsgType == 0 && rxtxLen == 0) {
                if (!messageBuffer.complete) {
                    // No data to send - respond immediately with zero-length
                    sendZeroLengthResponse();
                } else {
                    // Send our queued message
                    sendResponse();
                }
            } else {
                // Unexpected message type/length - reset to sync
                rs485State = STATE_SYNC;
            }
        } else {
            // Message for another slave - wait for their response
            rs485State = STATE_RX_WAIT_ANSWER_DATALENGTH;
        }
    }
}

// ============================================================================
// INPUT/OUTPUT CLASSES
// ============================================================================

/**
 * Switch2Pos - Simple two-position switch
 */
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

/**
 * LED - Simple LED output
 */
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
// TEST PINS - Directly usable for testing on Waveshare ESP32-S3-RS485-CAN
// ============================================================================
// Choose GPIO pins that don't conflict with RS485 (17, 18, 21) or USB (19, 20)
// Set any pin to -1 to disable that test control

#define SWITCH_PIN      -1     // Test switch input (toggle switch), -1 to disable
#define BUTTON_PIN      0      // Test button input (momentary pushbutton), -1 to disable
#define MC_READY_PIN    -1     // Test LED output, -1 to disable

// ============================================================================
// DCS-BIOS INPUTS (Physical controls -> Sim)
// ============================================================================

#if SWITCH_PIN >= 0
// F/A-18C Master Arm Switch - toggle switch test
// Connect a toggle switch between SWITCH_PIN and GND
// Internal pullup enabled: switch ON (to GND) = "1", switch OFF = "0"
Switch2Pos masterArmSw("MASTER_ARM_SW", SWITCH_PIN);
#endif

#if BUTTON_PIN >= 0
// F/A-18C UFC Option Select 1 - momentary pushbutton test
// Connect a momentary button between BUTTON_PIN and GND
// Internal pullup enabled: press = "1", release = "0"
Switch2Pos ufcOpt1Btn("UFC_OS1", BUTTON_PIN);
#endif

// ============================================================================
// DCS-BIOS OUTPUTS (Sim state -> LEDs)
// ============================================================================

#if MC_READY_PIN >= 0
// F/A-18C Master Caution READY Light (yellow)
// Address: 0x740C (29708), Mask: 0x8000 (32768)
// Connect an LED (with resistor) between MC_READY_PIN and GND
LED mcReadyLed(0x740C, 0x8000, MC_READY_PIN);
#endif

// ============================================================================
// ARDUINO SETUP AND LOOP
// ============================================================================

void setup() {
    // Initialize debug output
    DBG_INIT();
    udpDbgInit();  // Start WiFi for UDP debug (non-blocking)

    // Initialize RS485 with HARDWARE DE control
    initRS485Hardware();
}

void loop() {
    udpDbgCheck();  // Check WiFi connection for UDP debug
    processRS485();
    PollingInput::pollInputs();
    ExportStreamListener::loopAll();
}

// ============================================================================
// TECHNICAL DOCUMENTATION: HARDWARE RS485 MODE
// ============================================================================
/**
 * WHY HARDWARE RS485 MODE IS SUPERIOR
 * ====================================
 *
 * The ESP32's UART peripheral contains dedicated RS485 support that operates
 * at the hardware level, completely independent of software timing.
 *
 * REGISTER-LEVEL OPERATION
 * ------------------------
 * When UART_MODE_RS485_HALF_DUPLEX is enabled, the following happens:
 *
 * 1. UART_RS485_CONF_REG is configured:
 *    - Bit 0 (UART_RS485_EN): Set to 1, enabling RS485 mode
 *    - Bit 1 (UART_RS485TX_RX_EN): Set to 1, enabling auto TX/RX switching
 *    - Bit 2 (UART_RS485RXBY_TX_EN): Set to 1, disabling RX during TX
 *
 * 2. When data is written to TX FIFO:
 *    - Hardware state machine detects FIFO not empty
 *    - RTS (DE) pin is asserted HIGH within 1 APB clock cycle
 *    - TX shift register begins transmitting
 *
 * 3. When last byte's stop bit is transmitted:
 *    - Hardware detects TX shift register empty AND TX FIFO empty
 *    - RTS (DE) pin is deasserted LOW synchronously to UART clock
 *    - RX is re-enabled automatically
 *
 * TIMING PRECISION
 * ----------------
 * Software approach (previous version):
 *   - DE assertion: Register write latency ~100-500ns
 *   - DE deassertion: Loop polling + FreeRTOS jitter = ±10-50µs
 *
 * Hardware RS485 mode (this version):
 *   - DE assertion: <12.5ns (1 APB clock @ 80MHz)
 *   - DE deassertion: ZERO jitter, synchronous to UART bit clock
 *                     Precision: ±4µs (1 bit time at 250kbaud)
 *
 * The only "jitter" in hardware mode is the inherent ±0.5 bit uncertainty
 * in any asynchronous serial communication, which is unavoidable regardless
 * of implementation approach.
 *
 * COMPARISON TABLE
 * ----------------
 *
 * | Feature          | Software DE    | Hardware RS485     | AVR TXC ISR    |
 * |------------------|----------------|--------------------| ---------------|
 * | DE Assert Jitter | ±100-500ns     | <12.5ns            | ~60ns          |
 * | DE Deassert Jitter | ±10-50µs     | ~0ns (sync)        | ~250ns         |
 * | CPU Overhead     | Polling loop   | Zero               | ISR overhead   |
 * | Code Complexity  | High           | Low                | Medium         |
 * | Reliability      | OS-dependent   | Hardware-guaranteed| Interrupt-safe |
 *
 * WHEN TO USE EACH APPROACH
 * -------------------------
 * Hardware RS485 Mode (this implementation):
 *   + Best timing precision
 *   + Lowest CPU overhead
 *   + Most reliable
 *   - Requires DE pin to be connected to UART RTS
 *
 * Software DE Control:
 *   + DE pin can be any GPIO
 *   + More flexible for unusual wiring
 *   - Timing jitter from software
 *   - Higher CPU overhead
 *
 * External Auto-Direction Transceiver (MAX13487E, etc.):
 *   + Zero MCU involvement in DE timing
 *   + Works with any MCU/UART
 *   - Additional component cost
 *   - May have propagation delay specs to consider
 */
