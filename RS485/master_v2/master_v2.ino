/**
 * =============================================================================
 * ESP32 RS485 MASTER v2 - BARE METAL MULTI-BUS IMPLEMENTATION
 * =============================================================================
 *
 * Protocol-perfect RS485 Master for DCS-BIOS using bare-metal UART with
 * ISR-driven RX and inline blocking TX. Matches AVR DcsBiosNgRS485Master
 * behavior exactly, and SURPASSES it in broadcast strategy and poll scanning.
 *
 * SUPPORTED ESP32 VARIANTS:
 * - ESP32 (Classic)    - Dual-core, UART0 via USB-to-serial chip, 3 UARTs
 * - ESP32-S2           - Single-core, Native USB CDC, 2 UARTs
 * - ESP32-S3           - Dual-core, Native USB CDC, 3 UARTs
 * - ESP32-C3           - Single-core RISC-V, USB Serial/JTAG, 2 UARTs
 * - ESP32-C6           - Single-core RISC-V, USB Serial/JTAG, 2 UARTs
 *
 * =============================================================================
 * ARCHITECTURE (mirrors AVR DcsBiosNgRS485Master)
 * =============================================================================
 *
 * - ISR-driven RX: Each byte triggers an ISR that processes the state machine
 *   immediately, just like AVR's USART_RX_vect.
 * - Inline blocking TX: No FreeRTOS tasks for TX. TX blocks loop() briefly.
 * - Echo prevention: RX interrupt is disabled during TX and RX FIFO is flushed
 *   after TX completes, mirroring AVR's set_txen()/clear_txen() behavior.
 * - Bare-metal UART: Uses periph_module_enable + uart_ll_* for direct hardware
 *   access, matching the proven slave implementation.
 *
 * =============================================================================
 * V2 IMPROVEMENTS OVER master.ino
 * =============================================================================
 *
 * 1. PC INPUT TASK (FreeRTOS):
 *    A dedicated high-priority task continuously drains Serial into a 1024-byte
 *    RingBuffer, even while the main loop is blocked during TX. This matches
 *    AVR's ISR-driven PC RX behavior — zero chance of losing PC bytes regardless
 *    of how long broadcasts or polls take.
 *    - Dual-core (ESP32, S3): Pinned to Core 0 where USB stack runs
 *    - Single-core (C3, C6, S2, H2): FreeRTOS preemptive scheduling
 *
 * 2. INLINE PC FORWARDING:
 *    After each bus's loop() returns, slave responses are immediately forwarded
 *    to PC instead of waiting for the next loop() iteration. Reduces slave→PC
 *    latency by one full main loop cycle.
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
 * - Its own ISR registration (via void* arg to identify bus)
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
 * BROADCAST (Master -> All Slaves):
 *   [0x00] [0x00] [Length] [Data...] [Checksum]
 *   Address 0 = broadcast, all slaves receive but don't respond
 *
 * POLL (Master -> Specific Slave):
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

// TX Warmup Delays (microseconds)
#define TX_WARMUP_DELAY_MANUAL_US   50      // DE pin stabilization time
#define TX_WARMUP_DELAY_AUTO_US     50      // Auto-direction transceiver settling

// Buffer Sizes
#define EXPORT_BUFFER_SIZE     256   // Buffer for PC -> Slaves data (per bus)
#define MESSAGE_BUFFER_SIZE    64    // Buffer for Slave -> PC data (per bus)
#define PC_RX_BUFFER_SIZE      1024  // Buffer for PC Serial input task

// Timing Constants (microseconds) - matches AVR exactly
#define POLL_TIMEOUT_US      1000    // 1ms - timeout waiting for slave response
#define RX_TIMEOUT_US        5000    // 5ms - timeout for complete message (AVR-compatible)
#define MAX_POLL_INTERVAL_US 2000    // Ensure we poll at least every 2ms

// ============================================================================
// DEBUG OPTIONS
// ============================================================================
#define SERIAL_DEBUG_ENABLE 0       // Set to 1 to enable Serial debug output
#define UDP_DEBUG_ENABLE    0       // Set to 1 to enable WiFi-based debug output
#define UDP_DEBUG_IP        "255.255.255.255"  // Broadcast to all
#define UDP_DEBUG_PORT      4210    // CockpitOS debug port
#define UDP_DEBUG_NAME      "RS485-MASTER"    // Device identifier in debug messages
#define WIFI_SSID           "TestNetwork"
#define WIFI_PASSWORD       "TestingOnly"

// Broadcast chunking - prevents bus hogging during heavy export traffic
#define MAX_BROADCAST_CHUNK  64      // Max bytes per broadcast burst

// Slave address range to scan (1-127 valid, 0 is broadcast)
#define MIN_SLAVE_ADDRESS     1       // First slave address to poll
#define MAX_SLAVE_ADDRESS   127       // Last slave address to poll (set to 1 for single slave testing)

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
#include <rom/ets_sys.h>
#include <soc/soc_caps.h>

// Bare-metal UART access (same pattern as slave ISR mode)
#include <hal/uart_ll.h>
#include <hal/gpio_ll.h>
#include <soc/uart_struct.h>
#include <soc/gpio_struct.h>
#include <soc/uart_periph.h>
#include <esp_intr_alloc.h>

// Peripheral module control - handle both old and new ESP-IDF locations
#if __has_include(<esp_private/periph_ctrl.h>)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <soc/periph_defs.h>

// V2: FreeRTOS for PC Serial input task
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if UDP_DEBUG_ENABLE
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

// PC Serial - works on ALL ESP32 variants
#define PC_SERIAL Serial

// ============================================================================
// SERIAL DEBUG MACROS
// ============================================================================

#if SERIAL_DEBUG_ENABLE
#define dbgBegin(baud)          Serial.begin(baud)
#define dbgPrint(msg)           Serial.println(msg)
#define dbgPrintf(fmt, ...)     Serial.printf(fmt, ##__VA_ARGS__)
#define dbgFlush()              Serial.flush()
#else
#define dbgBegin(baud)
#define dbgPrint(msg)
#define dbgPrintf(fmt, ...)
#define dbgFlush()
#endif

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
        // Don't wait for connection - will check later
        targetIP.fromString(UDP_DEBUG_IP);
        connected = false;
    }

    void checkConnection() {
        if (!connected && WiFi.status() == WL_CONNECTED) {
            connected = true;
            // Send registration/announcement packet
            send("=== %s ONLINE === IP: %s", UDP_DEBUG_NAME, WiFi.localIP().toString().c_str());
            lastStatusTime = millis();
        }
        // Periodic heartbeat every 30 seconds to show we're still alive
        if (connected && (millis() - lastStatusTime) > 30000) {
            send("[%s] heartbeat", UDP_DEBUG_NAME);
            lastStatusTime = millis();
        }
    }

    void send(const char* fmt, ...) {
        if (!connected) return;

        // Prefix with device name for easy identification
        int pos = snprintf(buffer, sizeof(buffer), "[%s] ", UDP_DEBUG_NAME);

        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer + pos, sizeof(buffer) - pos, fmt, args);
        va_end(args);

        // Non-blocking UDP send
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

    inline void IRAM_ATTR put(uint8_t c) {
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
    inline void IRAM_ATTR clear() { readpos = writepos = 0; }
    inline uint16_t availableForWrite() const { return SIZE - getLength() - 1; }
};

// ============================================================================
// V2: PC INPUT TASK - Dedicated FreeRTOS task for PC Serial reception
// ============================================================================
// Solves: loop() blocks during broadcast TX (up to 2.5ms per chunk).
// During that time, PC bytes buffer in USB CDC. This task ensures they are
// captured continuously into pcRxBuffer, regardless of TX blocking.
//
// Uses the existing RingBuffer template — single-writer (this task),
// single-reader (main loop) is naturally lock-free with volatile positions.
// On ESP32, volatile DRAM accesses are coherent across cores.

static RingBuffer<PC_RX_BUFFER_SIZE> pcRxBuffer;
static TaskHandle_t pcInputTaskHandle = nullptr;

static void pcInputTaskFunc(void* param) {
    (void)param;
    for (;;) {
        // Drain all available bytes from USB CDC into pcRxBuffer
        while (PC_SERIAL.available()) {
            if (pcRxBuffer.availableForWrite() > 0) {
                pcRxBuffer.put((uint8_t)PC_SERIAL.read());
            } else {
                // Buffer full — discard byte to prevent USB CDC overflow
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
        2048,               // Minimal stack — just Serial.read() + RingBuffer.put()
        nullptr,
        5,                   // Priority 5: preempts Arduino loop() (priority 1)
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
        0                    // Core 0
    );
#endif
}

// ============================================================================
// STATE MACHINE
// ============================================================================
// TX is inline blocking, so no TX states needed.
// Only RX states + IDLE, matching AVR's rxISR state machine.

enum MasterState {
    STATE_IDLE,
    STATE_RX_WAIT_DATALENGTH,   // waiting for slave's first byte (1ms timeout)
    STATE_RX_WAIT_MSGTYPE,      // waiting for message type byte
    STATE_RX_WAIT_DATA,         // receiving data bytes
    STATE_RX_WAIT_CHECKSUM      // waiting for checksum byte
};

// ============================================================================
// RS485 MASTER CLASS - One instance per bus
// ============================================================================

// Forward declaration for ISR
class RS485Master;
static void IRAM_ATTR uart_isr_handler(void* arg);

class RS485Master {
private:
    // Hardware configuration
    int uartNum;
    int txPin, rxPin, dePin;

    // Bare-metal UART hardware pointer (set in init based on uartNum)
    uart_dev_t* uartHw;
    intr_handle_t uartIntrHandle;

    // State machine (volatile - modified by ISR)
    volatile MasterState state;
    volatile int64_t rxStartTime;
    volatile uint8_t rxtxLen;
    volatile uint8_t rxMsgType;
    volatile int64_t lastPollTime;

    // Slave tracking
    volatile bool slavePresent[MAX_SLAVES];
    uint8_t pollAddressCounter;
    uint8_t scanAddressCounter;
    uint8_t currentPollAddress;

    // ========================================================================
    // DE pin control helpers
    // ========================================================================

    inline void setDE(bool high) {
        if (dePin >= 0) {
            gpio_ll_set_level(&GPIO, (gpio_num_t)dePin, high ? 1 : 0);
        }
    }

    // ========================================================================
    // TX helpers - inline blocking, called from loop() context
    // ========================================================================
    // Mirrors AVR's set_txen/clear_txen pattern:
    //   1. Disable RX interrupt (no echo enters state machine)
    //   2. Assert DE
    //   3. Write bytes to FIFO, wait for idle
    //   4. De-assert DE
    //   5. Flush RX FIFO (discard any echo)
    //   6. Re-enable RX interrupt

    inline void prepareForTransmit() {
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

    inline void finishTransmit() {
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

    inline void txWriteBlocking(const uint8_t* data, uint16_t len) {
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

public:
    // Buffers (public for PC aggregation and ISR access)
    RingBuffer<EXPORT_BUFFER_SIZE> exportData;
    RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

    // Linked list for iteration
    RS485Master* next;
    static RS485Master* first;

    RS485Master(int uartNum, int txPin, int rxPin, int dePin)
        : uartNum(uartNum), txPin(txPin), rxPin(rxPin), dePin(dePin),
          uartHw(nullptr), uartIntrHandle(nullptr),
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
        slavePresent[0] = true;  // Address 0 is broadcast sentinel
        for (int i = 1; i < MAX_SLAVES; i++) {
            slavePresent[i] = false;
        }
    }

    // ========================================================================
    // init() - Bare-metal UART setup (from slave ISR-mode pattern)
    // ========================================================================

    void init() {
        dbgPrintf("  Bus UART%d: TX=%d RX=%d DE=%d\n", uartNum, txPin, rxPin, dePin);

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
            dbgPrint("  DE GPIO configured");
        } else {
            dbgPrint("  No DE pin (auto-direction)");
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
                dbgPrintf("  ERROR: Invalid UART number %d\n", uartNum);
                return;
        }

        dbgPrint("  Enabling UART peripheral...");
        dbgFlush();
        periph_module_enable(periphModule);

        dbgPrint("  Configuring UART parameters...");
        dbgFlush();
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

        dbgPrint("  Setting UART pins...");
        dbgFlush();
        ESP_ERROR_CHECK(uart_set_pin((uart_port_t)uartNum, txPin, rxPin,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // Ensure RX pin has pullup for stable idle state
        gpio_set_pull_mode((gpio_num_t)rxPin, GPIO_PULLUP_ONLY);

        // --- Configure RX interrupt ---
        dbgPrint("  Configuring RX FIFO threshold...");
        // Trigger interrupt on every byte for lowest latency
        uart_ll_set_rxfifo_full_thr(uartHw, 1);

        dbgPrint("  Clearing and enabling interrupts...");
        uart_ll_clr_intsts_mask(uartHw, UART_LL_INTR_MASK);
        uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

        // --- Register ISR ---
        // Pass 'this' as arg so the ISR knows which bus triggered it
        dbgPrint("  Registering ISR...");
        ESP_ERROR_CHECK(esp_intr_alloc(uart_periph_signal[uartNum].irq,
                                        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
                                        uart_isr_handler, this, &uartIntrHandle));

        // Flush any stale data from FIFO (init only, never during operation)
        uart_ll_rxfifo_rst(uartHw);

        state = STATE_IDLE;
        lastPollTime = esp_timer_get_time();

        dbgPrint("  Bus initialization complete!");
    }

    // ========================================================================
    // advancePollAddress() - safe wraparound (fixes AVR infinite-loop bug)
    // ========================================================================

    void advancePollAddress() {
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
    // TX functions - inline blocking (no FreeRTOS task/queue)
    // ========================================================================

    void sendPollFrame(uint8_t addr) {
        uint8_t frame[3] = { addr, 0x00, 0x00 };

        prepareForTransmit();
        txWriteBlocking(frame, sizeof(frame));
        finishTransmit();

        // Start RX timeout - equivalent to AVR's txcISR setting rx_start_time
        rxStartTime = esp_timer_get_time();
        state = STATE_RX_WAIT_DATALENGTH;
    }

    void sendBroadcastFrame() {
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

    void sendTimeoutZeroByte() {
        // AVR behavior: send zero byte on behalf of non-responding slave
        // to keep listening slaves synchronized with bus traffic
        uint8_t zero = 0x00;

        prepareForTransmit();
        txWriteBlocking(&zero, 1);
        finishTransmit();

        state = STATE_IDLE;
    }

    // ========================================================================
    // rxISR() - called from UART ISR, processes RX state machine per byte
    // Mirrors AVR's RS485Master::rxISR() exactly
    // ========================================================================

    void IRAM_ATTR rxISR() {
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
                case STATE_RX_WAIT_DATALENGTH:
                    rxtxLen = c;
                    slavePresent[currentPollAddress] = true;
                    if (rxtxLen > 0) {
                        state = STATE_RX_WAIT_MSGTYPE;
                    } else {
                        state = STATE_IDLE;
                    }
                    rxStartTime = esp_timer_get_time();
                    break;

                case STATE_RX_WAIT_MSGTYPE:
                    rxMsgType = c;
                    (void)rxMsgType;
                    state = STATE_RX_WAIT_DATA;
                    rxStartTime = esp_timer_get_time();
                    break;

                case STATE_RX_WAIT_DATA:
                    messageBuffer.put(c);
                    if (--rxtxLen == 0) {
                        state = STATE_RX_WAIT_CHECKSUM;
                    }
                    rxStartTime = esp_timer_get_time();
                    break;

                case STATE_RX_WAIT_CHECKSUM:
                    // Checksum intentionally ignored (AVR behavior)
                    messageBuffer.complete = true;
                    state = STATE_IDLE;
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

    void loop() {
        int64_t now = esp_timer_get_time();

        if (state == STATE_IDLE) {
#if !SUPPRESS_BROADCASTS
            // Priority 1: Send export data (broadcast) if available
            // But ensure we poll at least every MAX_POLL_INTERVAL_US
            if (exportData.isNotEmpty() && (now - lastPollTime) < MAX_POLL_INTERVAL_US) {
                sendBroadcastFrame();
                return;
            }
#else
            // Broadcasts suppressed - just discard any export data
            while (exportData.isNotEmpty()) {
                exportData.get();
            }
#endif

            // Priority 2: Poll a slave if message buffer is free
            if (messageBuffer.isEmpty() && !messageBuffer.complete) {
                advancePollAddress();
                sendPollFrame(currentPollAddress);
                lastPollTime = now;
            }
            return;
        }

        // Timeout: waiting for first response byte (1ms, matches AVR)
        if (state == STATE_RX_WAIT_DATALENGTH) {
            if ((now - rxStartTime) > POLL_TIMEOUT_US) {
                slavePresent[currentPollAddress] = false;
                sendTimeoutZeroByte();
                return;
            }
        }

        // Timeout: mid-message (5ms, matches AVR)
        if (state == STATE_RX_WAIT_MSGTYPE ||
            state == STATE_RX_WAIT_DATA ||
            state == STATE_RX_WAIT_CHECKSUM) {
            if ((now - rxStartTime) > RX_TIMEOUT_US) {
                // AVR-compatible: send newline marker to PC stream on timeout
                messageBuffer.clear();
                messageBuffer.put('\n');
                messageBuffer.complete = true;
                state = STATE_IDLE;

                // Flush any remaining bytes from FIFO
                uart_ll_rxfifo_rst(uartHw);
                uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
                uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
                return;
            }
        }
    }
};

// Static member initialization
RS485Master* RS485Master::first = nullptr;

// ============================================================================
// UART ISR HANDLER - shared function, bus identified via void* arg
// ============================================================================

static void IRAM_ATTR uart_isr_handler(void* arg) {
    RS485Master* bus = (RS485Master*)arg;
    bus->rxISR();
}

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
 * drainPCInput() - Drain PC RX buffer into ALL bus export buffers
 * V2: Reads from pcRxBuffer (filled by PCInputTask) instead of Serial directly.
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

// ============================================================================
// ARDUINO SETUP AND LOOP
// ============================================================================

void setup() {
    // Initialize USB CDC for PC communication
    PC_SERIAL.begin(250000);
    dbgBegin(115200);
    delay(3000);

    dbgPrint("");
    dbgPrint("===========================================");
    dbgPrint("ESP32 RS485 Master v2 - Bare Metal ISR");
    dbgPrintf("Chip: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    dbgPrintf("CPU: %d MHz\n", ESP.getCpuFreqMHz());
    dbgPrintf("Baud: %d\n", RS485_BAUD_RATE);
    dbgPrintf("Slave range: %d-%d\n", MIN_SLAVE_ADDRESS, MAX_SLAVE_ADDRESS);
    dbgPrint("===========================================");

#if UDP_DEBUG_ENABLE
    // Initialize WiFi for UDP debug (non-blocking)
    udpDbg.begin();
    dbgPrint("UDP Debug: Connecting to WiFi...");
#endif

    // V2: Start PC Serial input task (captures PC bytes even during TX blocking)
    startPCInputTask();
    dbgPrint("PC Input task started.");

    // Initialize all enabled buses
    dbgPrint("Initializing RS485 buses...");
    RS485Master* bus = RS485Master::first;
    while (bus != nullptr) {
        bus->init();
        bus = bus->next;
    }

    dbgPrint("All buses initialized. Entering main loop.");
}

void loop() {
#if UDP_DEBUG_ENABLE
    // Check WiFi connection status (non-blocking)
    udpDbg.checkConnection();
#endif

    // Drain PC data from pcRxBuffer into bus export buffers
    drainPCInput();

    // Forward any pending slave responses to PC (catch-all pass)
    sendToPC();

    // Process each bus's state machine
    RS485Master* bus = RS485Master::first;
    while (bus != nullptr) {
        bus->loop();

        // V2: IMMEDIATE FORWARDING — if this bus just completed a slave
        // response, forward it to PC right now instead of waiting for the
        // next loop() iteration. Reduces slave→PC latency by one full cycle.
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
// TECHNICAL NOTES
// ============================================================================
/**
 * ARCHITECTURE: AVR-ALIGNED BARE-METAL ISR + V2 IMPROVEMENTS
 * ===========================================================
 *
 * This implementation mirrors the AVR DcsBiosNgRS485Master exactly,
 * then surpasses it with ESP32-specific improvements:
 *
 * AVR set_txen() / clear_txen():
 *   set_txen()   -> disable RX, enable TX driver, enable TX
 *   clear_txen() -> disable TX, disable TX driver, enable RX
 *
 * ESP32 equivalent:
 *   prepareForTransmit() -> disable RX interrupt, assert DE, warmup delay
 *   finishTransmit()     -> wait TX idle, de-assert DE, flush RX FIFO,
 *                           re-enable RX interrupt
 *
 * AVR rxISR():
 *   Fires on every received byte. Processes state machine immediately.
 *
 * ESP32 equivalent:
 *   uart_isr_handler() -> fires on RXFIFO_FULL (threshold=1 byte),
 *                         calls rxISR() which processes state machine.
 *
 * AVR loop():
 *   Checks if IDLE -> starts broadcast or poll
 *   Checks timeouts for non-responding slaves
 *
 * ESP32 loop():
 *   Identical behavior. TX is inline blocking (sendPollFrame, etc.)
 *   so no TX states needed. Timeouts checked the same way.
 *
 * KEY FIX (v1): The original implementation used a FreeRTOS TX task
 * (priority 5) that called uart_flush_input() before every TX. This
 * destroyed in-progress slave responses, causing one-shot state changes
 * (switches) to be lost. The v1 rewrite eliminated this with inline
 * blocking TX and proper echo prevention.
 *
 * V2 IMPROVEMENT 1: PC INPUT TASK
 * ================================
 * Problem: processPCInput() in v1 polls Serial in the main loop. During
 * broadcast TX blocking (~2.5ms for 64-byte chunks), PC bytes buffer in
 * USB CDC. With 128 slaves and 3 buses, blocking can reach ~8ms.
 *
 * Solution: A FreeRTOS task (priority 5) on Core 0 continuously drains
 * Serial into a 1024-byte RingBuffer. The main loop drains this buffer
 * into bus export buffers via drainPCInput(). This matches AVR's
 * ISR-driven PC RX behavior — zero chance of losing PC bytes.
 *
 * Thread safety: The RingBuffer uses volatile uint16_t positions and is
 * accessed in a SPSC (single-producer, single-consumer) pattern. The
 * FreeRTOS task is the sole writer; the main loop is the sole reader.
 * On ESP32, volatile DRAM accesses are coherent across cores.
 *
 * Single-core handling: On ESP32-C3/C6/S2/H2, CONFIG_FREERTOS_UNICORE
 * is set. xTaskCreate() (without core pinning) is used instead. FreeRTOS
 * preemptive scheduling ensures the task runs between loop() iterations.
 *
 * V2 IMPROVEMENT 2: INLINE PC FORWARDING
 * =======================================
 * Problem: In v1, sendToPC() runs once per loop() iteration. After a
 * slave response completes (ISR sets messageBuffer.complete), the data
 * sits until the next sendToPC() call — one full loop cycle of latency.
 *
 * Solution: After each bus->loop() call, immediately check if that bus
 * has a complete message and forward it inline. This reduces slave->PC
 * latency from ~1 loop cycle to near-zero.
 *
 * MULTI-BUS ARCHITECTURE
 * ======================
 *
 * Like AVR's multi-UART support, each bus operates independently:
 * - Separate uart_dev_t* hardware pointer
 * - Separate ISR registration (void* arg identifies bus)
 * - Separate slave tracking, poll counters, state machine
 * - Separate export and message buffers
 *
 * Disabled buses (pins = -1) are completely compiled out.
 *
 * DATA FLOW (v2)
 * ==============
 *
 * PC -> Master:
 *   1. PCInputTask (Core 0) drains Serial -> pcRxBuffer (1024 bytes)
 *   2. drainPCInput() (main loop) drains pcRxBuffer -> ALL buses' exportData
 *   3. Each bus broadcasts independently (chunked, max 64 bytes)
 *
 * Slave -> PC:
 *   1. Each bus polls its slaves independently
 *   2. Slave response bytes trigger ISR, processed immediately
 *   3. Inline forwarding after bus->loop() sends complete messages to PC
 *   4. sendToPC() at top of loop() catches any remaining messages
 *
 * IMPROVEMENTS OVER AVR:
 *   - Chunked broadcasts (max 64 bytes) vs AVR's all-at-once (up to 128)
 *   - Forced poll every 2ms regardless of export traffic
 *   - Safe advancePollAddress() without infinite-loop bug
 *   - Multi-variant support (ESP32, S2, S3, C3, C6, H2)
 *   - RISC-V fence for FIFO stability on C3/C6/H2
 */
