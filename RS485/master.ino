/**
 * ESP32 RS485 MASTER - High-performance, protocol-strict implementation
 *
 * Design goals:
 * - Match AVR DcsBiosNgRS485Master protocol behavior
 * - Maximize throughput and minimize poll latency on ESP32
 * - Stay robust with up to 127 polled slave addresses
 *
 * Protocol behavior (AVR-compatible):
 * - Broadcast export frame: [0x00][0x00][len][data...][checksum]
 * - Poll frame:             [slaveAddr][0x00][0x00]
 * - Slave response:         [len][msgtype][data...][checksum] or [0x00]
 * - On missing slave timeout, master transmits a single 0x00 byte on bus
 *   (keeps listening slaves synchronized with bus traffic)
 */

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <hal/uart_ll.h>
#include <soc/uart_struct.h>
#include <soc/uart_periph.h>
#include <esp_intr_alloc.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <soc/soc_caps.h>

#if __has_include(<esp_private/periph_ctrl.h>)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <soc/periph_defs.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// RS485 bus UART (master <-> slaves)
#define RS485_UART_NUM              1
#define RS485_TX_PIN                17
#define RS485_RX_PIN                18
#define RS485_DE_PIN               -1     // -1 for auto-direction transceivers
#define RS485_BAUD_RATE        250000

// PC link UART (DCS-BIOS Hub <-> master)
// Use Serial (USB CDC/UART0 depending board support package)
#define PC_BAUD_RATE            250000

// Timing (protocol compatible with AVR behavior)
#define POLL_TIMEOUT_US           1000    // wait for first response byte
#define RX_TIMEOUT_US             5000    // mid-frame timeout
#define TX_WARMUP_DELAY_MANUAL_US   50
#define TX_WARMUP_DELAY_AUTO_US     50

// Buffer sizes
#define EXPORT_RING_SIZE          1024    // PC -> RS485
#define PC_TX_RING_SIZE           2048    // RS485 -> PC
#define MAX_BROADCAST_DATA         250    // fits in uint8 length field

// ============================================================================
// UART hardware mapping
// ============================================================================

#if RS485_UART_NUM == 0
static uart_dev_t* const uartHw = &UART0;
#define RS485_PERIPH_MODULE PERIPH_UART0_MODULE
#elif RS485_UART_NUM == 1
static uart_dev_t* const uartHw = &UART1;
#define RS485_PERIPH_MODULE PERIPH_UART1_MODULE
#elif RS485_UART_NUM == 2
static uart_dev_t* const uartHw = &UART2;
#define RS485_PERIPH_MODULE PERIPH_UART2_MODULE
#else
#error "Invalid RS485_UART_NUM (must be 0, 1, or 2)"
#endif

#if RS485_DE_PIN >= 0
static inline void IRAM_ATTR setDE(bool high) {
    gpio_set_level((gpio_num_t)RS485_DE_PIN, high ? 1 : 0);
}
#else
#define setDE(x)
#endif

// ============================================================================
// Ring buffer
// ============================================================================

template<unsigned int SIZE>
class RingBuffer {
private:
    volatile uint8_t buffer[SIZE];
    volatile uint16_t writepos;
    volatile uint16_t readpos;

public:
    RingBuffer() : writepos(0), readpos(0) {}

    inline bool isEmpty() const { return readpos == writepos; }
    inline bool isNotEmpty() const { return readpos != writepos; }

    inline uint16_t getLength() const {
        return (uint16_t)((writepos - readpos + SIZE) % SIZE);
    }

    inline uint16_t availableForWrite() const {
        return (uint16_t)(SIZE - getLength() - 1);
    }

    inline void clear() {
        readpos = 0;
        writepos = 0;
    }

    inline bool put(uint8_t c) {
        uint16_t next = (uint16_t)((writepos + 1) % SIZE);
        if (next == readpos) return false;
        buffer[writepos] = c;
        writepos = next;
        return true;
    }

    inline uint8_t get() {
        uint8_t v = buffer[readpos];
        readpos = (uint16_t)((readpos + 1) % SIZE);
        return v;
    }
};

// ============================================================================
// Master state
// ============================================================================

enum MasterState {
    STATE_IDLE,
    STATE_RX_WAIT_DATALENGTH,
    STATE_RX_WAIT_MSGTYPE,
    STATE_RX_WAIT_DATA,
    STATE_RX_WAIT_CHECKSUM,
    STATE_TX_TIMEOUT_ZERO
};

static volatile MasterState state = STATE_IDLE;
static volatile uint8_t rxtxLen = 0;
static volatile uint8_t rxMsgType = 0;
static volatile uint8_t pollAddress = 1;
static volatile int64_t rxStartTimeUs = 0;

// Slave discovery/polling map (AVR-compatible strategy)
static volatile bool slavePresent[128];
static uint8_t scanAddressCounter = 1;
static uint8_t pollAddressCounter = 1;

static RingBuffer<EXPORT_RING_SIZE> exportData;
static RingBuffer<PC_TX_RING_SIZE> pcTxData;

static volatile uint32_t rs485RxBytes = 0;
static volatile uint32_t timeoutCount = 0;
static volatile uint32_t pcTxDropped = 0;

static intr_handle_t uartIntrHandle;

// ============================================================================
// Poll address scheduling (same behavior as AVR)
// ============================================================================

static inline void advancePollAddress() {
    pollAddressCounter = (uint8_t)((pollAddressCounter + 1) % 128);
    while (!slavePresent[pollAddressCounter]) {
        pollAddressCounter = (uint8_t)((pollAddressCounter + 1) % 128);
    }

    if (pollAddressCounter == 0) {
        scanAddressCounter = (uint8_t)((scanAddressCounter + 1) % 128);
        while (slavePresent[scanAddressCounter]) {
            scanAddressCounter = (uint8_t)((scanAddressCounter + 1) % 128);
        }
        pollAddress = scanAddressCounter;
        return;
    }

    pollAddress = pollAddressCounter;
}

// ============================================================================
// RS485 TX helpers
// ============================================================================

static inline void IRAM_ATTR prepareForTransmitISR() {
#if RS485_DE_PIN >= 0
    setDE(true);
    ets_delay_us(TX_WARMUP_DELAY_MANUAL_US);
#else
    ets_delay_us(TX_WARMUP_DELAY_AUTO_US);
#endif
}

static inline void IRAM_ATTR finishTransmitISR() {
    setDE(false);
    uart_ll_rxfifo_rst(uartHw);  // purge echo
    uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
    uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
}

static inline void IRAM_ATTR txWriteBlockingISR(const uint8_t* data, uint16_t len) {
    uint16_t sent = 0;
    while (sent < len) {
        uint32_t used = uart_ll_get_txfifo_len(uartHw);
        if (used < 127) {
            uint16_t space = (uint16_t)(127 - used);
            uint16_t n = (uint16_t)((len - sent < space) ? (len - sent) : space);
            uart_ll_write_txfifo(uartHw, data + sent, n);
            sent += n;
        }
    }
    while (!uart_ll_is_tx_idle(uartHw));
}

static inline void sendPollFrame(uint8_t addr) {
    uint8_t frame[3] = { addr, 0x00, 0x00 };

    uart_ll_disable_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
    prepareForTransmitISR();
    txWriteBlockingISR(frame, sizeof(frame));

    rxStartTimeUs = esp_timer_get_time();
    state = STATE_RX_WAIT_DATALENGTH;

    finishTransmitISR();
}

static inline void sendTimeoutZeroByte() {
    uint8_t zero = 0x00;

    uart_ll_disable_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
    prepareForTransmitISR();
    txWriteBlockingISR(&zero, 1);

    state = STATE_IDLE;

    finishTransmitISR();
}

static inline void sendBroadcastFrame() {
    uint16_t available = exportData.getLength();
    if (available == 0) return;

    uint8_t dataLen = (uint8_t)((available > MAX_BROADCAST_DATA) ? MAX_BROADCAST_DATA : available);
    uint8_t frame[MAX_BROADCAST_DATA + 4];
    uint16_t idx = 0;

    frame[idx++] = 0x00;    // address
    frame[idx++] = 0x00;    // msgtype
    frame[idx++] = dataLen; // payload length

    for (uint8_t i = 0; i < dataLen; i++) {
        frame[idx++] = exportData.get();
    }

    frame[idx++] = 0x00;    // AVR-compatible checksum behavior

    uart_ll_disable_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
    prepareForTransmitISR();
    txWriteBlockingISR(frame, idx);
    state = STATE_IDLE;
    finishTransmitISR();
}

// ============================================================================
// RS485 RX ISR
// ============================================================================

static void IRAM_ATTR uart_isr_handler(void* arg) {
    uint32_t uart_intr_status = uart_ll_get_intsts_mask(uartHw);

    while (uart_ll_get_rxfifo_len(uartHw) > 0) {
        uint8_t c;
        uart_ll_read_rxfifo(uartHw, &c, 1);
#ifdef __riscv
        __asm__ __volatile__("fence");
#endif
        rs485RxBytes++;

        switch (state) {
            case STATE_RX_WAIT_DATALENGTH:
                rxtxLen = c;
                slavePresent[pollAddress] = true;
                if (rxtxLen == 0) {
                    state = STATE_IDLE;
                } else {
                    state = STATE_RX_WAIT_MSGTYPE;
                    rxStartTimeUs = esp_timer_get_time();
                }
                break;

            case STATE_RX_WAIT_MSGTYPE:
                rxMsgType = c;
                (void)rxMsgType;
                state = STATE_RX_WAIT_DATA;
                rxStartTimeUs = esp_timer_get_time();
                break;

            case STATE_RX_WAIT_DATA:
                if (!pcTxData.put(c)) {
                    pcTxDropped++;
                }
                if (--rxtxLen == 0) {
                    state = STATE_RX_WAIT_CHECKSUM;
                }
                rxStartTimeUs = esp_timer_get_time();
                break;

            case STATE_RX_WAIT_CHECKSUM:
                // Checksum intentionally ignored (AVR behavior)
                state = STATE_IDLE;
                rxStartTimeUs = esp_timer_get_time();
                break;

            default:
                break;
        }
    }

    uart_ll_clr_intsts_mask(uartHw, uart_intr_status);
}

// ============================================================================
// PC link helpers
// ============================================================================

static inline void pumpPcRxToExport() {
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        if (!exportData.put((uint8_t)c)) {
            // drop oldest strategy could be used, but keep deterministic: drop new byte
            break;
        }
    }
}

static inline void flushPcTx() {
    while (pcTxData.isNotEmpty() && Serial.availableForWrite() > 0) {
        uint8_t c = pcTxData.get();
        Serial.write(&c, 1);
    }
}

// ============================================================================
// RS485 hardware init
// ============================================================================

static void initRS485Hardware() {
#if RS485_DE_PIN >= 0
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RS485_DE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    setDE(false);
#endif

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

    gpio_set_pull_mode((gpio_num_t)RS485_RX_PIN, GPIO_PULLUP_ONLY);

    uart_ll_set_rxfifo_full_thr(uartHw, 1);
    uart_ll_clr_intsts_mask(uartHw, UART_LL_INTR_MASK);
    uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);

    ESP_ERROR_CHECK(esp_intr_alloc(uart_periph_signal[RS485_UART_NUM].irq,
                                    ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
                                    uart_isr_handler, NULL, &uartIntrHandle));

    state = STATE_IDLE;
}

// ============================================================================
// Main loop scheduler
// ============================================================================

static inline void masterLoop() {
    if (state == STATE_IDLE) {
        if (exportData.isNotEmpty()) {
            sendBroadcastFrame();
            return;
        }

        advancePollAddress();
        sendPollFrame(pollAddress);
        return;
    }

    int64_t now = esp_timer_get_time();

    if (state == STATE_RX_WAIT_DATALENGTH) {
        if ((now - rxStartTimeUs) > POLL_TIMEOUT_US) {
            timeoutCount++;
            slavePresent[pollAddress] = false;
            state = STATE_TX_TIMEOUT_ZERO;
            sendTimeoutZeroByte();
            return;
        }
    } else if (state == STATE_RX_WAIT_MSGTYPE ||
               state == STATE_RX_WAIT_DATA ||
               state == STATE_RX_WAIT_CHECKSUM) {
        if ((now - rxStartTimeUs) > RX_TIMEOUT_US) {
            timeoutCount++;
            // AVR-compatible behavior: send newline marker to PC stream on timeout
            pcTxData.put('\n');
            state = STATE_IDLE;
            uart_ll_rxfifo_rst(uartHw);
            uart_ll_clr_intsts_mask(uartHw, UART_INTR_RXFIFO_FULL);
            uart_ll_ena_intr_mask(uartHw, UART_INTR_RXFIFO_FULL);
            return;
        }
    }
}

// ============================================================================
// Arduino setup/loop
// ============================================================================

void setup() {
    Serial.begin(PC_BAUD_RATE);
    delay(1000);

    if (RS485_TX_PIN >= SOC_GPIO_PIN_COUNT || RS485_RX_PIN >= SOC_GPIO_PIN_COUNT) {
        Serial.printf("WARNING: RS485 pins may be invalid for this chip (max=%d)\n", SOC_GPIO_PIN_COUNT - 1);
    }

    // slave id 0 is never polled; used as scan sentinel like AVR
    slavePresent[0] = true;
    for (int i = 1; i < 128; i++) slavePresent[i] = false;

    initRS485Hardware();

    Serial.println("ESP32 RS485 master ready");
}

void loop() {
    pumpPcRxToExport();
    masterLoop();
    flushPcTx();
}
