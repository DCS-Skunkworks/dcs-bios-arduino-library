/**
 * =============================================================================
 * ESP32-S3 RS485 MASTER - BARE METAL IMPLEMENTATION
 * =============================================================================
 *
 * Protocol-perfect RS485 Master for DCS-BIOS using ESP32-S3 hardware RS485 mode.
 *
 * This implementation uses the SAME hardware RS485 approach that works perfectly
 * for the Slave: UART_MODE_RS485_HALF_DUPLEX with uart_wait_tx_done().
 *
 * =============================================================================
 * MASTER ROLE
 * =============================================================================
 *
 * The Master is the bus orchestrator:
 * 1. Receives export data from PC via USB CDC
 * 2. Broadcasts export data to all slaves (address 0)
 * 3. Polls each slave in round-robin sequence
 * 4. Receives slave responses and forwards to PC
 * 5. Tracks which slaves are present on the bus
 * 6. Scans for new devices when idle
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
 *   [Length] [MsgType] [Data...] [Checksum]  - if slave has data
 *   [0x00]                                    - if slave has nothing
 *
 * TIMEOUT HANDLING:
 *   - 1ms: If no response, mark slave as not present
 *   - 5ms: If incomplete message, abort and reset
 *
 * =============================================================================
 * HARDWARE CONFIGURATION
 * =============================================================================
 *
 * Device: ESP32-S3 (Waveshare ESP32-S3-RS485-CAN or compatible)
 * TX Pin: GPIO 17 → RS485 DI
 * RX Pin: GPIO 18 ← RS485 RO
 * DE Pin: GPIO 21 → RS485 DE (Hardware-controlled via RTS)
 * USB:    Native USB CDC for PC communication
 * Speed:  250,000 bps (8N1)
 *
 * =============================================================================
 * LICENSE: Same as DCS-BIOS Arduino Library
 * =============================================================================
 */

// ============================================================================
// CONFIGURATION
// ============================================================================

// Pin Configuration (Waveshare ESP32-S3-RS485-CAN defaults)
#define RS485_TX_PIN    17
#define RS485_RX_PIN    18
#define RS485_DE_PIN    21    // Set to -1 for auto-direction transceivers

// UART Configuration
#define RS485_UART_NUM  1
#define RS485_BAUD_RATE 250000

// Buffer Sizes
#define UART_RX_BUFFER_SIZE    512
#define UART_TX_BUFFER_SIZE    256
#define EXPORT_BUFFER_SIZE     256   // Buffer for PC → Slaves data
#define MESSAGE_BUFFER_SIZE    64    // Buffer for Slave → PC data

// Timing Constants (microseconds)
#define POLL_TIMEOUT_US     1000     // 1ms - timeout waiting for slave response
#define RX_TIMEOUT_US       5000     // 5ms - timeout for complete message
#define SYNC_TIMEOUT_US     500      // 500µs silence = sync

// Maximum number of slaves (addresses 1-127)
#define MAX_SLAVES          128

// ============================================================================
// ESP32 HARDWARE INCLUDES
// ============================================================================

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_timer.h>

// Use native USB CDC on ESP32-S3
#if CONFIG_IDF_TARGET_ESP32S3
    #define PC_SERIAL Serial    // Native USB CDC
#else
    #error "This implementation requires ESP32-S3"
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

    inline uint8_t peek() const {
        return buffer[readpos];
    }

    inline bool isEmpty() const { return readpos == writepos; }
    inline bool isNotEmpty() const { return readpos != writepos; }
    inline uint16_t getLength() const { return (writepos - readpos) % SIZE; }
    inline void clear() { readpos = writepos = 0; }
    inline uint16_t availableForWrite() const { return SIZE - getLength() - 1; }
};

// ============================================================================
// GLOBAL BUFFERS
// ============================================================================

// Export data buffer: PC → Master → Slaves (broadcast)
static RingBuffer<EXPORT_BUFFER_SIZE> exportData;

// Message buffer: Slaves → Master → PC
static RingBuffer<MESSAGE_BUFFER_SIZE> messageBuffer;

// ============================================================================
// MASTER STATE MACHINE
// ============================================================================

enum MasterState {
    STATE_IDLE,

    // Transmit states (broadcast export data)
    STATE_TX_SENDING,

    // Poll states
    STATE_POLL_SENDING,
    STATE_RX_WAIT_DATALENGTH,
    STATE_RX_WAIT_MSGTYPE,
    STATE_RX_WAIT_DATA,
    STATE_RX_WAIT_CHECKSUM,

    // Timeout handling
    STATE_TIMEOUT_SENDING
};

static volatile MasterState masterState = STATE_IDLE;
static volatile int64_t rxStartTime = 0;
static volatile uint8_t rxtxLen = 0;
static volatile uint8_t rxMsgType = 0;
static volatile uint8_t txChecksum = 0;

// Slave tracking
static bool slavePresent[MAX_SLAVES];
static uint8_t pollAddressCounter = 1;
static uint8_t scanAddressCounter = 1;
static uint8_t currentPollAddress = 1;

// UART handle
static uart_port_t uartNum = (uart_port_t)RS485_UART_NUM;

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

static void initRS485Hardware() {
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

#if RS485_DE_PIN >= 0
    // Hardware RS485 mode with automatic DE control
    ESP_ERROR_CHECK(uart_set_pin(uartNum, RS485_TX_PIN, RS485_RX_PIN,
                                  RS485_DE_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX));
#else
    // Auto-direction transceiver mode
    ESP_ERROR_CHECK(uart_set_pin(uartNum, RS485_TX_PIN, RS485_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(uartNum, UART_MODE_UART));
#endif

    // Flush any garbage
    uart_flush_input(uartNum);

    // Initialize slave presence tracking
    slavePresent[0] = true;  // Address 0 is broadcast, always "present"
    for (int i = 1; i < MAX_SLAVES; i++) {
        slavePresent[i] = false;
    }

    masterState = STATE_IDLE;
}

// ============================================================================
// SLAVE ADDRESS MANAGEMENT
// ============================================================================

/**
 * advancePollAddress() - Select next slave to poll
 *
 * Round-robin through present slaves. When we would poll address 0
 * (which is broadcast, not a real slave), we instead scan for a new device.
 */
static void advancePollAddress() {
    // Advance to next address
    pollAddressCounter = (pollAddressCounter + 1) % MAX_SLAVES;

    // Skip to next present slave
    while (!slavePresent[pollAddressCounter]) {
        pollAddressCounter = (pollAddressCounter + 1) % MAX_SLAVES;
    }

    if (pollAddressCounter == 0) {
        // Instead of polling address 0 (broadcast), scan for a new device
        scanAddressCounter = (scanAddressCounter + 1) % MAX_SLAVES;

        // Find an address that isn't present
        while (slavePresent[scanAddressCounter]) {
            scanAddressCounter = (scanAddressCounter + 1) % MAX_SLAVES;
            if (scanAddressCounter == 0) {
                // All addresses present, just poll address 1
                currentPollAddress = 1;
                return;
            }
        }

        currentPollAddress = scanAddressCounter;
        return;
    }

    currentPollAddress = pollAddressCounter;
}

// ============================================================================
// TRANSMIT FUNCTIONS
// ============================================================================

/**
 * sendBroadcast() - Send export data to all slaves
 *
 * Packet format: [0x00] [0x00] [Length] [Data...] [Checksum]
 */
static void sendBroadcast() {
    uint8_t len = exportData.getLength();
    if (len == 0) return;

    // Build packet
    uint8_t packet[EXPORT_BUFFER_SIZE + 4];
    packet[0] = 0x00;       // Address 0 = broadcast
    packet[1] = 0x00;       // Message type 0
    packet[2] = len;        // Data length

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = exportData.get();
        packet[3 + i] = b;
        checksum ^= b;
    }
    packet[3 + len] = checksum;

    // Send packet - hardware RS485 mode handles DE automatically
    uart_write_bytes(uartNum, (const char*)packet, 4 + len);

    // Wait for TX to complete (hardware deasserts DE after last stop bit)
    uart_wait_tx_done(uartNum, pdMS_TO_TICKS(50));

    // Back to idle
    masterState = STATE_IDLE;
}

/**
 * sendPoll() - Poll a specific slave for data
 *
 * Packet format: [SlaveAddr] [0x00] [0x00]
 */
static void sendPoll(uint8_t slaveAddr) {
    uint8_t packet[3];
    packet[0] = slaveAddr;  // Slave address
    packet[1] = 0x00;       // Message type 0
    packet[2] = 0x00;       // Data length 0 (poll request)

    // Send poll packet
    uart_write_bytes(uartNum, (const char*)packet, 3);
    uart_wait_tx_done(uartNum, pdMS_TO_TICKS(10));

    // Start timeout for slave response
    rxStartTime = esp_timer_get_time();
    masterState = STATE_RX_WAIT_DATALENGTH;
}

/**
 * sendTimeoutZeroByte() - Send zero-length response on behalf of missing slave
 *
 * When a slave doesn't respond, we send [0x00] to maintain protocol timing
 * for any other devices listening on the bus.
 */
static void sendTimeoutZeroByte() {
    uint8_t zero = 0x00;
    uart_write_bytes(uartNum, (const char*)&zero, 1);
    uart_wait_tx_done(uartNum, pdMS_TO_TICKS(10));
    masterState = STATE_IDLE;
}

// ============================================================================
// PC COMMUNICATION (USB CDC)
// ============================================================================

/**
 * processPCInput() - Read export data from PC and buffer it
 */
static void processPCInput() {
    while (PC_SERIAL.available() && exportData.availableForWrite() > 0) {
        uint8_t c = PC_SERIAL.read();
        exportData.put(c);
    }
}

/**
 * sendToPC() - Forward slave response data to PC
 */
static void sendToPC() {
    while (messageBuffer.isNotEmpty()) {
        PC_SERIAL.write(messageBuffer.get());
    }
    messageBuffer.complete = false;
}

// ============================================================================
// MAIN PROCESSING LOOP
// ============================================================================

static void processMaster() {
    int64_t now = esp_timer_get_time();

    switch (masterState) {
        case STATE_IDLE:
            // Priority 1: Forward any pending slave data to PC
            if (messageBuffer.complete) {
                sendToPC();
            }

            // Priority 2: Broadcast export data if we have any
            if (exportData.isNotEmpty()) {
                sendBroadcast();
                return;
            }

            // Priority 3: Poll next slave (only if message buffer is free)
            if (messageBuffer.isEmpty() && !messageBuffer.complete) {
                advancePollAddress();
                sendPoll(currentPollAddress);
            }
            break;

        case STATE_RX_WAIT_DATALENGTH:
            // Check for timeout (1ms)
            if ((now - rxStartTime) > POLL_TIMEOUT_US) {
                // Slave didn't respond - mark as not present
                slavePresent[currentPollAddress] = false;
                sendTimeoutZeroByte();
                return;
            }

            // Check for received byte
            {
                size_t available = 0;
                uart_get_buffered_data_len(uartNum, &available);
                if (available > 0) {
                    uint8_t c;
                    uart_read_bytes(uartNum, &c, 1, 0);

                    rxtxLen = c;
                    slavePresent[currentPollAddress] = true;  // Slave responded!

                    if (rxtxLen > 0) {
                        masterState = STATE_RX_WAIT_MSGTYPE;
                        rxStartTime = now;  // Reset timeout for rest of message
                    } else {
                        // Zero-length response = slave has nothing to send
                        masterState = STATE_IDLE;
                    }
                }
            }
            break;

        case STATE_RX_WAIT_MSGTYPE:
            // Check for timeout
            if ((now - rxStartTime) > RX_TIMEOUT_US) {
                messageBuffer.clear();
                messageBuffer.put('\n');
                messageBuffer.complete = true;
                masterState = STATE_IDLE;
                return;
            }

            {
                size_t available = 0;
                uart_get_buffered_data_len(uartNum, &available);
                if (available > 0) {
                    uint8_t c;
                    uart_read_bytes(uartNum, &c, 1, 0);
                    rxMsgType = c;
                    rxtxLen--;  // Length includes msgtype
                    masterState = STATE_RX_WAIT_DATA;
                }
            }
            break;

        case STATE_RX_WAIT_DATA:
            // Check for timeout
            if ((now - rxStartTime) > RX_TIMEOUT_US) {
                messageBuffer.clear();
                messageBuffer.put('\n');
                messageBuffer.complete = true;
                masterState = STATE_IDLE;
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
                    masterState = STATE_RX_WAIT_CHECKSUM;
                }
            }
            break;

        case STATE_RX_WAIT_CHECKSUM:
            // Check for timeout
            if ((now - rxStartTime) > RX_TIMEOUT_US) {
                messageBuffer.clear();
                messageBuffer.put('\n');
                messageBuffer.complete = true;
                masterState = STATE_IDLE;
                return;
            }

            {
                size_t available = 0;
                uart_get_buffered_data_len(uartNum, &available);
                if (available > 0) {
                    uint8_t c;
                    uart_read_bytes(uartNum, &c, 1, 0);
                    // TODO: Verify checksum (currently ignored like AVR)
                    messageBuffer.complete = true;
                    masterState = STATE_IDLE;
                }
            }
            break;

        default:
            masterState = STATE_IDLE;
            break;
    }
}

// ============================================================================
// ARDUINO SETUP AND LOOP
// ============================================================================

void setup() {
    // Initialize USB CDC for PC communication
    PC_SERIAL.begin(250000);  // Match DCS-BIOS baud rate

    // Wait a moment for USB to enumerate
    delay(100);

    // Initialize RS485 hardware
    initRS485Hardware();
}

void loop() {
    // Read any available data from PC
    processPCInput();

    // Process Master state machine
    processMaster();
}

// ============================================================================
// TECHNICAL NOTES
// ============================================================================
/**
 * WHY THIS WORKS (Same as Slave)
 * ==============================
 *
 * The key insight is that UART_MODE_RS485_HALF_DUPLEX combined with
 * uart_wait_tx_done() provides the same cycle-accurate DE timing that
 * AVR achieves with its TXC interrupt:
 *
 * 1. uart_write_bytes() puts data in TX FIFO
 * 2. Hardware immediately asserts DE (RTS pin)
 * 3. UART transmits all bytes
 * 4. uart_wait_tx_done() blocks until shift register is empty
 * 5. Hardware deasserts DE after last stop bit
 * 6. We return to RX mode
 *
 * This is exactly what the AVR does with its TXC ISR, just implemented
 * differently. The timing precision is the same: DE drops immediately
 * after the last stop bit exits the shift register.
 *
 * PROTOCOL COMPATIBILITY
 * ======================
 *
 * This implementation exactly matches the AVR RS485 Master protocol:
 *
 * - Broadcast: [0x00][0x00][Len][Data...][Checksum]
 * - Poll:      [Addr][0x00][0x00]
 * - Response:  [Len][MsgType][Data...][Checksum] or [0x00]
 *
 * - 1ms timeout for slave response
 * - 5ms timeout for complete message
 * - Round-robin polling of present slaves
 * - Scanning for new devices when idle
 * - Zero-byte response on behalf of missing slaves
 *
 * COMPARISON TO PREVIOUS ESP32 MASTER
 * ===================================
 *
 * The previous ESP32 Master implementation was fundamentally flawed:
 * - Used manual DE control with calculated timing
 * - Timing calculations had jitter from CPU load
 * - Complex dual-core architecture with FreeRTOS
 * - StreamBuffer overhead for inter-core communication
 *
 * This implementation:
 * - Uses hardware RS485 mode (same as working Slave)
 * - No timing calculations needed
 * - Single-core, simple loop
 * - Direct UART operations
 */
