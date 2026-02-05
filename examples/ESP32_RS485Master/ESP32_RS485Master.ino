/**
 * ESP32 RS485 Master Example for DCS-BIOS
 *
 * This sketch turns your ESP32 into an RS485 bus master that:
 *   - Receives export data from the DCS-BIOS PC application via USB Serial
 *   - Broadcasts that data to all slaves on the RS485 bus
 *   - Polls each slave for pending commands (switch/encoder changes)
 *   - Forwards slave responses back to the PC application
 *
 * SUPPORTED ESP32 VARIANTS:
 *   ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6
 *
 * WIRING:
 *   ESP32 TX Pin  -> RS485 Module DI (Driver Input)
 *   ESP32 RX Pin  -> RS485 Module RO (Receiver Output)
 *   ESP32 DE Pin  -> RS485 Module DE + RE (or -1 for auto-direction boards)
 *   RS485 A/B     -> Bus (daisy-chain to all slaves)
 *
 * CONFIGURATION:
 *   Define pins before #include <DcsBios.h> to override defaults.
 *   Defaults: TX=17, RX=18, DE=-1 (auto-direction), UART1
 *
 *   For multi-bus setups, define RS485_BUS2_* and RS485_BUS3_* pins.
 *
 * PERFORMANCE TUNING:
 *   The defaults below are already optimized for maximum performance and
 *   lowest latency with up to 128 slaves. You should NOT need to change them.
 *
 *   MAX_SLAVE_ADDRESS   = 127   Full scan range (1-127). Lower ONLY if you
 *                                know your exact slave count and want faster
 *                                discovery (e.g., 4 if you only have 4 slaves).
 *   MAX_BROADCAST_CHUNK = 64    Sweet spot: fits in UART FIFO, keeps TX
 *                                blocking to ~2.5ms max at 250kbaud.
 *   MAX_POLL_INTERVAL_US = 2000 Guarantees a poll every 2ms even during
 *                                heavy broadcast traffic. Do not increase.
 *   POLL_TIMEOUT_US     = 1000  1ms timeout for slave response. Matches AVR.
 *   RX_TIMEOUT_US       = 5000  5ms timeout for incomplete messages.
 */

// Disable servo library for ESP32s
#define DCSBIOS_DISABLE_SERVO

// --- Bus 1 Configuration (override defaults if needed) ---
#define RS485_BUS1_TX_PIN   17
#define RS485_BUS1_RX_PIN   18
#define RS485_BUS1_DE_PIN   -1     // -1 = auto-direction, or GPIO number
// #define RS485_BUS1_UART_NUM  1

// --- Bus 2 (uncomment to enable a second RS485 bus) ---
// #define RS485_BUS2_TX_PIN   10
// #define RS485_BUS2_RX_PIN   11
// #define RS485_BUS2_DE_PIN   -1
// #define RS485_BUS2_UART_NUM  2

// --- Performance tuning (defaults are already optimal — change only if needed) ---
// #define MAX_SLAVE_ADDRESS    127   // Set to your actual slave count for faster discovery
// #define MAX_BROADCAST_CHUNK  64    // Max bytes per broadcast burst (64 = optimal)
// #define MAX_POLL_INTERVAL_US 2000  // Max us between polls (2ms = optimal)

#define DCSBIOS_RS485_MASTER
#include <DcsBios.h>

void setup() {
    DcsBios::setup();
}

void loop() {
    DcsBios::loop();
}
