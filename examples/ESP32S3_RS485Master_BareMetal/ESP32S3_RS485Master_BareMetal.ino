/**
 * =============================================================================
 * ESP32 RS485 MASTER - BARE METAL MULTI-BUS IMPLEMENTATION
 * =============================================================================
 *
 * Protocol-perfect RS485 Master for DCS-BIOS using ESP32.
 * Supports up to 3 independent RS485 buses (like AVR DcsBiosNgRS485Master).
 *
 * SUPPORTED ESP32 VARIANTS:
 * - ESP32 (Classic)    - Dual-core, UART0 via USB-to-serial chip, 3 UARTs
 * - ESP32-S2           - Single-core, Native USB CDC, 2 UARTs
 * - ESP32-S3           - Dual-core, Native USB CDC, 3 UARTs
 * - ESP32-C3           - Single-core RISC-V, USB Serial/JTAG, 2 UARTs
 * - ESP32-C6           - Single-core RISC-V, USB Serial/JTAG, 2 UARTs
 *
 * All variants support UART_MODE_RS485_HALF_DUPLEX for hardware DE control.
 *
 * =============================================================================
 * MULTI-BUS ARCHITECTURE
 * =============================================================================
 *
 * Like AVR, this implementation supports multiple independent RS485 buses:
 * - BUS1: Primary bus (enabled by default)
 * - BUS2: Secondary bus (disabled by default, set pins to enable)
 * - BUS3: Tertiary bus (disabled by default, set pins to enable)
 *
 * Each bus has:
 * - Independent slave tracking (addresses 1-127)
 * - Independent polling and broadcast
 * - Separate message buffers
 *
 * Export data from PC is broadcast to ALL enabled buses.
 * Slave responses from ANY bus are forwarded to PC.
 *
 * Disabled buses (pins = -1) are completely compiled out - zero overhead.
 *
 * =============================================================================
 * PROTOCOL (Exact match to AVR DcsBiosNgRS485Master)
 * =============================================================================
 *
 * BROADCAST (Master → All Slaves):
 *   [0x00] [0x00] [Length] [Data...] [Checksum]
 *   Address 0 = broadcast, all slaves receive but don't respond
 *
 * POLL (Master → Specific Slave):
 *   [SlaveAddr] [0x00] [0x00]
 *   Requests slave to send any pending data
 *
 * SLAVE RESPONSE:
 *   [DataLength] [MsgType] [Data...] [Checksum]  - if slave has data
 *   [0x00]                                        - if slave has nothing
 *   Note: DataLength = number of DATA bytes only, NOT including MsgType
 *
 * =============================================================================
 * LICENSE: Same as DCS-BIOS Arduino Library
 * =============================================================================
 */

// ============================================================================
// MULTI-BUS CONFIGURATION
// ============================================================================
// Set pins to -1 to disable a bus. Disabled buses have ZERO overhead.
// At least BUS1 must be enabled.

// BUS 1 - Primary RS485 bus (enabled by default)
#define BUS1_TX_PIN     17
#define BUS1_RX_PIN     18
#define BUS1_DE_PIN     -1      // -1 = auto-direction mode (board may have built-in auto-dir)
#define BUS1_UART_NUM    1       // UART1

// DE Control Mode - Try MANUAL (1) if hardware RS485 mode has issues with your transceiver
#define RS485_DE_MANUAL  0       // 0 = Hardware RS485 mode, 1 = Manual GPIO mode

// BUS 2 - Secondary RS485 bus (disabled by default)
#define BUS2_TX_PIN     -1      // Set to valid GPIO to enable
#define BUS2_RX_PIN     -1
#define BUS2_DE_PIN     -1
#define BUS2_UART_NUM   -1      // Set to 2 to use UART2

// BUS 3 - Tertiary RS485 bus (disabled by default)
#define BUS3_TX_PIN     -1      // Set to valid GPIO to enable
#define BUS3_RX_PIN     -1
#define BUS3_DE_PIN     -1
#define BUS3_UART_NUM   -1      // Set to valid UART number to enable

// ============================================================================
// COMMON CONFIGURATION
// ============================================================================

#define RS485_BAUD_RATE 250000

// Buffer Sizes
#define UART_RX_BUFFER_SIZE    512
#define UART_TX_BUFFER_SIZE    256
#define EXPORT_BUFFER_SIZE     256   // Buffer for PC → Slaves data (per bus)
#define MESSAGE_BUFFER_SIZE    64    // Buffer for Slave → PC data (per bus)

// Timing Constants (microseconds)
#define POLL_TIMEOUT_US      1000    // 1ms - timeout waiting for slave response
#define RX_TIMEOUT_US        5000    // 5ms - timeout for complete message (matches AVR)
#define SYNC_TIMEOUT_US      500     // 500µs silence = sync
#define MAX_POLL_INTERVAL_US 2000    // Ensure we poll at least every 2ms

// ============================================================================
// UDP DEBUG LOGGING - Set to 1 to enable WiFi-based debug output
// ============================================================================
// This sends debug data via UDP without affecting RS485 timing.
// Useful for debugging when USB Serial would add latency.

// UDP DEBUG - Uses CockpitOS debug console on port 4210
#define UDP_DEBUG_ENABLE    1       // Set to 1 to enable UDP debug
#define UDP_DEBUG_IP        "255.255.255.255"  // Broadcast to all
#define UDP_DEBUG_PORT      4210    // CockpitOS debug port
#define UDP_DEBUG_NAME      "RS485-MASTER"    // Device identifier in debug messages
#define WIFI_SSID           "TestNetwork"
#define WIFI_PASSWORD       "TestingOnly"

// Broadcast chunking - prevents bus hogging during heavy export traffic
#define MAX_BROADCAST_CHUNK  64      // Max bytes per broadcast burst

// Slave address range to scan (1-127 valid, 0 is broadcast)
#define MIN_SLAVE_ADDRESS     1       // First slave address to poll
#define MAX_SLAVE_ADDRESS    32       // Last slave address to poll

// DEBUG: Suppress broadcasts to test if bus congestion is the issue
// Set to 1 to disable all broadcasts (only polling will occur)
#define SUPPRESS_BROADCASTS 0       // 0 = normal, 1 = disable broadcasts for testing

// Internal array size - don't change
#define MAX_SLAVES          128

// ============================================================================
// COMPILE-TIME BUS DETECTION
// ============================================================================

#define BUS1_ENABLED (BUS1_UART_NUM >= 0 && BUS1_TX_PIN >= 0 && BUS1_RX_PIN >= 0)
#define BUS2_ENABLED (BUS2_UART_NUM >= 0 && BUS2_TX_PIN >= 0 && BUS2_RX_PIN >= 0)
#define BUS3_ENABLED (BUS3_UART_NUM >= 0 && BUS3_TX_PIN >= 0 && BUS3_RX_PIN >= 0)

#define NUM_BUSES_ENABLED ((BUS1_ENABLED ? 1 : 0) + (BUS2_ENABLED ? 1 : 0) + (BUS3_ENABLED ? 1 : 0))

#if NUM_BUSES_ENABLED == 0
    #error "At least one RS485 bus must be enabled (set BUS1 pins)"
#endif

// ============================================================================
// ESP32 HARDWARE INCLUDES
// ============================================================================

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#if UDP_DEBUG_ENABLE
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

// PC Serial - works on ALL ESP32 variants
#define PC_SERIAL Serial

// ============================================================================
// UDP DEBUG CLASS - Non-blocking debug output via WiFi
// ============================================================================

#if UDP_DEBUG_ENABLE
class UdpDebug {
private:
    WiFiUDP udp;
    IPAddress targetIP;
    bool connected;
    char buffer[256];
    unsigned long lastStatusTime;

public:
    UdpDebug() : connected(false), lastStatusTime(0) {}

    void begin() {
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        targetIP.fromString(UDP_DEBUG_IP);
        connected = false;
    }

    void checkConnection() {
        if (!connected && WiFi.status() == WL_CONNECTED) {
            connected = true;
            send("=== %s ONLINE === IP: %s", UDP_DEBUG_NAME, WiFi.localIP().toString().c_str());
            lastStatusTime = millis();
        }
        if (connected && (millis() - lastStatusTime) > 30000) {
            send("[%s] heartbeat", UDP_DEBUG_NAME);
            lastStatusTime = millis();
        }
    }

    void send(const char* fmt, ...) {
        if (!connected) return;
        int pos = snprintf(buffer, sizeof(buffer), "[%s] ", UDP_DEBUG_NAME);
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer + pos, sizeof(buffer) - pos, fmt, args);
        va_end(args);
        udp.beginPacket(targetIP, UDP_DEBUG_PORT);
        udp.write((uint8_t*)buffer, strlen(buffer));
        udp.endPacket();
    }

    void sendHex(const char* prefix, uint8_t* data, size_t len) {
        if (!connected) return;
        int pos = snprintf(buffer, sizeof(buffer), "[%s] %s: ", UDP_DEBUG_NAME, prefix);
        for (size_t i = 0; i < len && pos < (int)sizeof(buffer) - 4; i++) {
            pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02X ", data[i]);
        }
        udp.beginPacket(targetIP, UDP_DEBUG_PORT);
        udp.write((uint8_t*)buffer, strlen(buffer));
        udp.endPacket();
    }
};

UdpDebug udpDbg;
#define UDP_DBG(fmt, ...) udpDbg.send(fmt, ##__VA_ARGS__)
#define UDP_DBG_HEX(prefix, data, len) udpDbg.sendHex(prefix, data, len)
#else
#define UDP_DBG(fmt, ...)
#define UDP_DBG_HEX(prefix, data, len)
#endif

// ============================================================================
// RING BUFFER
// ============================================================================

template<unsigned int SIZE>
class RingBuffer {
private:
    volatile uint8_t buffer[SIZE];
    volatile uint16_t writepos;
    volatile uint16_t readpos;

public:
    volatile bool complete;

    RingBuffer() : writepos(0), readpos(0), complete(false) {}

    inline void put(uint8_t c) {
        buffer[writepos] = c;
        writepos = (writepos + 1) % SIZE;
    }

    inline uint8_t get() {
        uint8_t ret = buffer[readpos];
        readpos = (readpos + 1) % SIZE;
        return ret;
    }

    inline uint8_t peek() const { return buffer[readpos]; }
    inline bool isEmpty() const { return readpos == writepos; }
    inline bool isNotEmpty() const { return readpos != writepos; }
    inline uint16_t getLength() const { return (writepos - readpos) % SIZE; }
    inline void clear() { readpos = writepos = 0; }
    inline uint16_t availableForWrite() const { return SIZE - getLength() - 1; }
};

// ============================================================================
// RS485 MASTER CLASS - One instance per bus
// ============================================================================

enum MasterState {
    STATE_IDLE,
    STATE_RX_WAIT_DATALENGTH,
    STATE_RX_WAIT_MSGTYPE,
    STATE_RX_WAIT_DATA,
    STATE_RX_WAIT_CHECKSUM
};

class RS485Master {
private:
    // Hardware configuration
    uart_port_t uartNum;
    int txPin, rxPin, dePin;

    // State machine
    MasterState state;
    int64_t rxStartTime;
    uint8_t rxtxLen;
    uint8_t rxMsgType;
    int64_t lastPollTime;

    // Slave tracking
    bool slavePresent[MAX_SLAVES];
    uint8_t pollAddressCounter;
    uint8_t scanAddressCounter;
    uint8_t currentPollAddress;

public:
    // Buffers (public for PC aggregation)
    RingBuffer<EXPORT_BUFFER_SIZE> exportData;
    RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

    // Linked list for iteration
    RS485Master* next;
    static RS485Master* first;

    RS485Master(int uartNum, int txPin, int rxPin, int dePin)
        : uartNum((uart_port_t)uartNum), txPin(txPin), rxPin(rxPin), dePin(dePin),
          state(STATE_IDLE), rxStartTime(0), rxtxLen(0), rxMsgType(0),
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
        slavePresent[0] = true;  // Address 0 is broadcast
        for (int i = 1; i < MAX_SLAVES; i++) {
            slavePresent[i] = false;
        }
    }

    void init() {
        uart_config_t uart_config = {
            .baud_rate = RS485_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };

        ESP_ERROR_CHECK(uart_driver_install(uartNum, UART_RX_BUFFER_SIZE,
                                             UART_TX_BUFFER_SIZE, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(uartNum, &uart_config));

#if RS485_DE_MANUAL
        // Manual DE mode - use regular UART and control DE via GPIO
        ESP_ERROR_CHECK(uart_set_pin(uartNum, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_UART));
        if (dePin >= 0) {
            pinMode(dePin, OUTPUT);
            digitalWrite(dePin, LOW);  // Start in RX mode
        }
#else
        if (dePin >= 0) {
            // Hardware RS485 mode with automatic DE control
            ESP_ERROR_CHECK(uart_set_pin(uartNum, txPin, rxPin, dePin, UART_PIN_NO_CHANGE));
            ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX));
        } else {
            // Auto-direction transceiver mode
            ESP_ERROR_CHECK(uart_set_pin(uartNum, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
            ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_UART));
        }
#endif

        // Flush stale data at init ONLY - never during normal operation
        uart_flush_input(uartNum);
        state = STATE_IDLE;
        lastPollTime = esp_timer_get_time();
    }

    // ========================================================================
    // DIRECT TX - Synchronous, inline (no FreeRTOS task overhead)
    // ========================================================================
    // Previous implementation used a FreeRTOS queue + task for TX which:
    //   1. Added 200-500µs non-deterministic context switch latency
    //   2. Called uart_flush_input() before EVERY send, destroying any
    //      pending slave response bytes in the RX buffer
    // This synchronous approach eliminates both problems.

    void sendDirect(const uint8_t* data, uint16_t length) {
#if RS485_DE_MANUAL
        if (dePin >= 0) {
            digitalWrite(dePin, HIGH);  // Enable transmitter
            delayMicroseconds(50);      // DE warmup
        }
#endif

        uart_write_bytes(uartNum, (const char*)data, length);
        uart_wait_tx_done(uartNum, 50 / portTICK_PERIOD_MS);

#if RS485_DE_MANUAL
        if (dePin >= 0) {
            delayMicroseconds(10);      // Ensure last stop bit completes
            digitalWrite(dePin, LOW);   // Back to RX mode

            // Discard echo bytes (we hear our own TX in manual DE mode)
            delayMicroseconds(100);
            uint8_t discardBuf[64];
            size_t available = 0;
            uart_get_buffered_data_len(uartNum, &available);
            size_t toDiscard = (available > length) ? length : available;
            while (toDiscard > 0) {
                size_t chunk = (toDiscard > sizeof(discardBuf)) ? sizeof(discardBuf) : toDiscard;
                uart_read_bytes(uartNum, discardBuf, chunk, 0);
                toDiscard -= chunk;
            }
        }
#endif

        // Turnaround delay - let transceiver switch back to RX
        delayMicroseconds(50);
    }

    void advancePollAddress() {
        // Advance through address range, wrapping at MAX_SLAVE_ADDRESS
        pollAddressCounter++;
        if (pollAddressCounter > MAX_SLAVE_ADDRESS) {
            pollAddressCounter = MIN_SLAVE_ADDRESS;
        }

        // Skip to next known slave (for efficiency with large address ranges)
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

    void sendBroadcast() {
        uint8_t available = exportData.getLength();
        if (available == 0) return;

        uint8_t len = (available > MAX_BROADCAST_CHUNK) ? MAX_BROADCAST_CHUNK : available;

        uint8_t packet[MAX_BROADCAST_CHUNK + 4];
        packet[0] = 0x00;
        packet[1] = 0x00;
        packet[2] = len;

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < len; i++) {
            uint8_t b = exportData.get();
            packet[3 + i] = b;
            checksum ^= b;
        }
        packet[3 + len] = checksum;

        sendDirect(packet, 4 + len);
    }

    void sendPoll(uint8_t slaveAddr) {
        uint8_t packet[3] = { slaveAddr, 0x00, 0x00 };
        sendDirect(packet, 3);
    }

    void sendTimeoutZeroByte() {
        uint8_t zero = 0x00;
        sendDirect(&zero, 1);
    }

    void loop() {
        int64_t now = esp_timer_get_time();

        switch (state) {
            case STATE_IDLE:
#if !SUPPRESS_BROADCASTS
                // Send pending broadcast data before polling
                if (exportData.isNotEmpty() && (now - lastPollTime) < MAX_POLL_INTERVAL_US) {
                    sendBroadcast();
                    return;
                }
#else
                // Broadcasts suppressed - just discard any export data
                while (exportData.isNotEmpty()) {
                    exportData.get();
                }
#endif
                if (messageBuffer.isEmpty() && !messageBuffer.complete) {
                    advancePollAddress();
                    sendPoll(currentPollAddress);
                    lastPollTime = now;
                    // TX is done synchronously - go straight to waiting for response
                    rxStartTime = esp_timer_get_time();
                    state = STATE_RX_WAIT_DATALENGTH;
                }
                break;

            case STATE_RX_WAIT_DATALENGTH:
                if ((now - rxStartTime) > POLL_TIMEOUT_US) {
                    slavePresent[currentPollAddress] = false;
                    sendTimeoutZeroByte();
                    state = STATE_IDLE;
                    return;
                }
                {
                    size_t available = 0;
                    uart_get_buffered_data_len(uartNum, &available);
                    if (available > 0) {
                        uint8_t c;
                        uart_read_bytes(uartNum, &c, 1, 0);
                        rxtxLen = c;
                        slavePresent[currentPollAddress] = true;
                        if (rxtxLen > 0 && rxtxLen <= 64) {
                            UDP_DBG("RX addr=%d len=%d", currentPollAddress, rxtxLen);
                            state = STATE_RX_WAIT_MSGTYPE;
                            rxStartTime = now;
                        } else {
                            // Zero length or garbage - back to idle
                            state = STATE_IDLE;
                        }
                    }
                }
                break;

            case STATE_RX_WAIT_MSGTYPE:
                if ((now - rxStartTime) > RX_TIMEOUT_US) {
                    messageBuffer.clear();
                    state = STATE_IDLE;
                    return;
                }
                {
                    size_t available = 0;
                    uart_get_buffered_data_len(uartNum, &available);
                    if (available > 0) {
                        uint8_t c;
                        uart_read_bytes(uartNum, &c, 1, 0);
                        rxMsgType = c;
                        state = STATE_RX_WAIT_DATA;
                    }
                }
                break;

            case STATE_RX_WAIT_DATA:
                if ((now - rxStartTime) > RX_TIMEOUT_US) {
                    messageBuffer.clear();
                    state = STATE_IDLE;
                    return;
                }
                {
                    size_t available = 0;
                    uart_get_buffered_data_len(uartNum, &available);
                    while (available > 0 && rxtxLen > 0) {
                        uint8_t c;
                        uart_read_bytes(uartNum, &c, 1, 0);
                        messageBuffer.put(c);
                        rxtxLen--;
                        available--;
                    }
                    if (rxtxLen == 0) {
                        state = STATE_RX_WAIT_CHECKSUM;
                    }
                }
                break;

            case STATE_RX_WAIT_CHECKSUM:
                if ((now - rxStartTime) > RX_TIMEOUT_US) {
                    messageBuffer.clear();
                    state = STATE_IDLE;
                    return;
                }
                {
                    size_t available = 0;
                    uart_get_buffered_data_len(uartNum, &available);
                    if (available > 0) {
                        uint8_t c;
                        uart_read_bytes(uartNum, &c, 1, 0);
                        (void)c;  // Checksum ignored (like AVR)
                        UDP_DBG("RX_OK addr=%d len=%d", currentPollAddress, messageBuffer.getLength());
                        messageBuffer.complete = true;
                        state = STATE_IDLE;
                    }
                }
                break;

            default:
                state = STATE_IDLE;
                break;
        }
    }
};

// Static member initialization
RS485Master* RS485Master::first = nullptr;

// ============================================================================
// BUS INSTANCES - Only enabled buses are instantiated
// ============================================================================

#if BUS1_ENABLED
RS485Master bus1(BUS1_UART_NUM, BUS1_TX_PIN, BUS1_RX_PIN, BUS1_DE_PIN);
#endif

#if BUS2_ENABLED
RS485Master bus2(BUS2_UART_NUM, BUS2_TX_PIN, BUS2_RX_PIN, BUS2_DE_PIN);
#endif

#if BUS3_ENABLED
RS485Master bus3(BUS3_UART_NUM, BUS3_TX_PIN, BUS3_RX_PIN, BUS3_DE_PIN);
#endif

// ============================================================================
// PC COMMUNICATION - Aggregates all buses
// ============================================================================

/**
 * processPCInput() - Read export data from PC and distribute to ALL buses
 */
static void processPCInput() {
    while (PC_SERIAL.available()) {
        uint8_t c = PC_SERIAL.read();

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

// ============================================================================
// ARDUINO SETUP AND LOOP
// ============================================================================

void setup() {
    // Initialize USB CDC for PC communication
    PC_SERIAL.begin(250000);
    delay(3000);

#if UDP_DEBUG_ENABLE
    // Initialize WiFi for UDP debug (non-blocking)
    udpDbg.begin();
    PC_SERIAL.println("UDP Debug: Connecting to WiFi...");
#endif

    // Initialize all enabled buses
    RS485Master* bus = RS485Master::first;
    while (bus != nullptr) {
        bus->init();
        bus = bus->next;
    }

    PC_SERIAL.println("RS485 Master initialized (inline TX)");
    UDP_DBG("Master initialized, buses=%d", NUM_BUSES_ENABLED);
}

void loop() {
#if UDP_DEBUG_ENABLE
    // Check WiFi connection status (non-blocking)
    udpDbg.checkConnection();
#endif

    // Read export data from PC (distributes to all buses)
    processPCInput();

    // Forward any pending slave responses to PC
    sendToPC();

    // Process each bus's state machine
    RS485Master* bus = RS485Master::first;
    while (bus != nullptr) {
        bus->loop();
        bus = bus->next;
    }
}
